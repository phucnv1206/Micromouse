#include "main.h"
#include "uart_protocol.h"

#define UART_PROTOCOL_RX_QUEUE_SIZE 4U

extern UART_HandleTypeDef huart2;

/*
 * Trạng thái bộ phân tích gói nhận:
 * - WAIT_HEADER_1: tìm byte đầu tiên của header AA.
 * - WAIT_HEADER_2: đã thấy AA, chờ byte BB tiếp theo để xác nhận header.
 * - READ_PAYLOAD: đã lưu header, tiếp tục nhận đến khi đủ 8 byte của gói RX.
 *
 * Tách việc phân tích theo từng byte giúp đồng bộ lại khi UART bắt đầu giữa
 * một gói hoặc khi một byte bị mất; dữ liệu chỉ được đưa vào hàng đợi sau khi
 * đã ghép đủ một khung có kích thước cố định.
 */
typedef enum {
  UART_RX_WAIT_HEADER_1,
  UART_RX_WAIT_HEADER_2,
  UART_RX_READ_PAYLOAD
} UartProtocol_RxState;

static uint8_t tx_frame[UART_PROTOCOL_TX_FRAME_SIZE];
static uint8_t rx_byte;
static uint8_t rx_work_frame[UART_PROTOCOL_RX_FRAME_SIZE];
static uint8_t rx_work_index;
static UartProtocol_RxState rx_state;
/*
 * Hàng đợi vòng lưu các gói RX đã nhận hoàn chỉnh:
 * callback UART là bên ghi (producer), còn vòng lặp chính là bên đọc (consumer).
 * Hàng đợi khai báo 4 ô nhưng dành một ô để phân biệt trạng thái đầy/rỗng,
 * do đó tối đa giữ được 3 gói chưa đọc.
 */
static uint8_t rx_frames[UART_PROTOCOL_RX_QUEUE_SIZE][UART_PROTOCOL_RX_FRAME_SIZE];
static volatile uint8_t rx_head;
static volatile uint8_t rx_tail;
static volatile uint8_t rx_active;
static volatile HAL_StatusTypeDef rx_status;
static volatile uint32_t rx_error_code;
/* Đếm số gói hoàn chỉnh bị bỏ khi hàng đợi đầy; callback không chờ để tránh kéo dài ISR. */
static volatile uint32_t dropped_frame_count;

/*
 * Ghi mẫu bit int16_t theo thứ tự big-endian, không đổi mã hóa signed.
 */
static void UartProtocol_WriteInt16BE(uint8_t *destination, int16_t value) {
  uint16_t bits = (uint16_t)value;
  destination[0] = (uint8_t)(bits >> 8U);
  destination[1] = (uint8_t)bits;
}

static int16_t UartProtocol_ReadInt16BE(const uint8_t *source) {
  uint16_t bits = (uint16_t)(((uint16_t)source[0] << 8U) | source[1]);
  int32_t value = bits <= INT16_MAX ? (int32_t)bits : (int32_t)bits - 65536;
  return (int16_t)value;
}

/*
 * Đưa một byte mới nhận được qua bộ phân tích khung RX.
 * Khi đủ 8 byte (AA BB, mã lệnh, lệnh phải, lệnh trái và byte dự phòng),
 * hàm chuyển nguyên khung sang hàng đợi để vòng lặp chính xử lý sau.
 */
static void UartProtocol_ProcessRxByte(uint8_t byte) {
  if (rx_state == UART_RX_WAIT_HEADER_1) {
    if (byte == UART_PROTOCOL_HEADER_BYTE_1) {
      rx_work_frame[0] = byte;
      rx_state = UART_RX_WAIT_HEADER_2;
    }
    return;
  }

  if (rx_state == UART_RX_WAIT_HEADER_2) {
    if (byte == UART_PROTOCOL_HEADER_BYTE_2) {
      rx_work_frame[1] = byte;
      rx_work_index = UART_PROTOCOL_HEADER_SIZE;
      rx_state = UART_RX_READ_PAYLOAD;
    } else if (byte == UART_PROTOCOL_HEADER_BYTE_1) {
      /* Byte AA mới có thể là đầu header kế tiếp nếu BB trước đó bị mất. */
      rx_work_frame[0] = byte;
    } else {
      rx_state = UART_RX_WAIT_HEADER_1;
    }
    return;
  }

  /* Header đã được lưu; nối từng byte dữ liệu cho đến khi đủ kích thước khung RX. */
  rx_work_frame[rx_work_index++] = byte;
  if (rx_work_index == UART_PROTOCOL_RX_FRAME_SIZE) {
    /* Chừa một ô trống để phân biệt hàng đợi đầy với hàng đợi rỗng. */
    uint8_t next_head =
        (uint8_t)((rx_head + 1U) % UART_PROTOCOL_RX_QUEUE_SIZE);

    if (next_head == rx_tail) {
      /* Không ghi đè gói cũ chưa được vòng lặp chính đọc. */
      dropped_frame_count++;
    } else {
      for (uint32_t i = 0U; i < UART_PROTOCOL_RX_FRAME_SIZE; i++) {
        rx_frames[rx_head][i] = rx_work_frame[i];
      }
      rx_head = next_head;
    }

    rx_state = UART_RX_WAIT_HEADER_1;
  }
}

/*
 * Khởi tạo trạng thái phân tích và hàng đợi nhận, xóa bộ đếm gói bị rơi,
 * sau đó bắt đầu nhận byte đầu tiên bằng ngắt UART.
 * Trả về trạng thái HAL để bên gọi biết việc khởi động nhận có thành công không.
 */
HAL_StatusTypeDef UartProtocol_Init(void) {
  rx_head = 0U;
  rx_tail = 0U;
  rx_state = UART_RX_WAIT_HEADER_1;
  rx_work_index = 0U;
  rx_active = 0U;
  rx_status = HAL_OK;
  rx_error_code = HAL_UART_ERROR_NONE;
  dropped_frame_count = 0U;

  return UartProtocol_StartReceive();
}

/*
 * Gửi một khung TX 16 byte bằng DMA. Chỉ chấp nhận khung bắt đầu bằng AA BB.
 * Sao chép dữ liệu vào tx_frame tĩnh vì DMA tiếp tục đọc bộ đệm sau khi hàm
 * trả về; nhờ đó dữ liệu không phụ thuộc vào thời gian sống của bộ đệm bên gọi.
 */
HAL_StatusTypeDef UartProtocol_Send(const uint8_t frame[UART_PROTOCOL_TX_FRAME_SIZE]) {
  if (frame == NULL || frame[0] != UART_PROTOCOL_HEADER_BYTE_1 || frame[1] != UART_PROTOCOL_HEADER_BYTE_2) {
    return HAL_ERROR;
  }

  for (uint32_t i = 0U; i < UART_PROTOCOL_TX_FRAME_SIZE; i++) {
    tx_frame[i] = frame[i];
  }

  return HAL_UART_Transmit_DMA(&huart2, tx_frame, UART_PROTOCOL_TX_FRAME_SIZE);
}

/*
 * Đóng gói telemetry thành khung TX 16 byte:
 * AA BB, sau đó lần lượt là data1..data7 dạng int16_t big-endian.
 * Tổng cộng 2 + 7*2 = 16 byte.
 * Việc truyền thực tế được giao cho UartProtocol_Send().
 */
HAL_StatusTypeDef UartProtocol_SendTelemetry(const UartProtocol_Telemetry *telemetry) {
  uint8_t frame[UART_PROTOCOL_TX_FRAME_SIZE] = {
    UART_PROTOCOL_HEADER_BYTE_1,
    UART_PROTOCOL_HEADER_BYTE_2
  };

  if (telemetry == NULL) {
    return HAL_ERROR;
  }

  UartProtocol_WriteInt16BE(&frame[2] , telemetry->data1);
  UartProtocol_WriteInt16BE(&frame[4] , telemetry->data2);
  UartProtocol_WriteInt16BE(&frame[6] , telemetry->data3);
  UartProtocol_WriteInt16BE(&frame[8] , telemetry->data4);
  UartProtocol_WriteInt16BE(&frame[10], telemetry->data5);
  UartProtocol_WriteInt16BE(&frame[12], telemetry->data6);
  UartProtocol_WriteInt16BE(&frame[14], telemetry->data7);

  return UartProtocol_Send(frame);
}

/*
 * Lấy gói RX hoàn chỉnh cũ nhất khỏi hàng đợi và chép vào bộ đệm của bên gọi.
 * Tạm khóa ngắt để callback UART không thể cập nhật hàng đợi trong lúc kiểm tra
 * chỉ số hoặc sao chép khung; sau đó khôi phục đúng trạng thái ngắt ban đầu.
 * Trả HAL_BUSY nếu chưa có gói, HAL_ERROR nếu con trỏ đích không hợp lệ.
 */
HAL_StatusTypeDef UartProtocol_Receive(UartProtocol_RxFrame *frame) {
  uint8_t rawFrame[UART_PROTOCOL_RX_FRAME_SIZE];
  uint32_t primask;

  if (frame == NULL) {
    return HAL_ERROR;
  }

  /* Bảo vệ chỉ số hàng đợi và thao tác chép khung khỏi callback ngắt UART. */
  primask = __get_PRIMASK();
  __disable_irq();
  if (rx_tail == rx_head) {
    if (primask == 0U) {
      __enable_irq();
    }
    return HAL_BUSY;
  }

  for (uint32_t i = 0U; i < UART_PROTOCOL_RX_FRAME_SIZE; i++) {
    rawFrame[i] = rx_frames[rx_tail][i];
  }
  rx_tail = (uint8_t)((rx_tail + 1U) % UART_PROTOCOL_RX_QUEUE_SIZE);
  if (primask == 0U) {
    __enable_irq();
  }

  frame->header1 = rawFrame[0];
  frame->header2 = rawFrame[1];
  frame->command = rawFrame[2];
  frame->data1 = UartProtocol_ReadInt16BE(&rawFrame[3]);
  frame->data2 = UartProtocol_ReadInt16BE(&rawFrame[5]);
  frame->data3 = rawFrame[7];
  return HAL_OK;
}

/*
 * Bắt đầu nhận một byte bằng ngắt UART2 nếu hiện chưa có yêu cầu nhận đang chạy.
 * Nhận từng byte cho phép bộ phân tích tìm lại header AA BB khi luồng dữ liệu
 * bị lệch; trạng thái HAL và mã lỗi được lưu để bên gọi có thể kiểm tra.
 */
HAL_StatusTypeDef UartProtocol_StartReceive(void) {
  HAL_StatusTypeDef status;

  if (rx_active != 0U) {
    return HAL_OK;
  }

  status = HAL_UART_Receive_IT(&huart2, &rx_byte, 1U);
  if (status == HAL_OK) {
    rx_active = 1U;
    rx_status = HAL_OK;
    rx_error_code = HAL_UART_ERROR_NONE;
  } else {
    rx_status = status;
    rx_error_code = huart2.ErrorCode;
  }

  return status;
}

/* Trả về trạng thái gần nhất của thao tác bắt đầu nhận UART. */
HAL_StatusTypeDef UartProtocol_GetRxStatus(void) {
  return rx_status;
}

/* Trả về mã lỗi HAL UART gần nhất được lưu lại trong quá trình nhận. */
uint32_t UartProtocol_GetRxErrorCode(void) {
  return rx_error_code;
}

/* Trả về số khung hoàn chỉnh đã bị loại bỏ vì hàng đợi nhận bị đầy. */
uint32_t UartProtocol_GetDroppedFrameCount(void) {
  return dropped_frame_count;
}

/*
 * Callback HAL chạy sau khi nhận đủ một byte trên USART2:
 * đánh dấu yêu cầu hiện tại đã kết thúc, phân tích byte vừa nhận rồi lập tức
 * kích hoạt yêu cầu nhận byte tiếp theo để giảm nguy cơ bỏ lỡ dữ liệu.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART2) {
    rx_active = 0U;
    UartProtocol_ProcessRxByte(rx_byte);

    HAL_StatusTypeDef status = UartProtocol_StartReceive();
    if (status != HAL_OK) {
      rx_status = status;
      rx_error_code = huart2.ErrorCode;
    }
  }
}

/*
 * Callback HAL khi UART2 phát hiện lỗi nhận:
 * đánh dấu bộ nhận đang rảnh và lưu trạng thái/mã lỗi để ứng dụng kiểm tra,
 * sau đó có thể gọi UartProtocol_StartReceive() để thử khởi động lại.
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART2) {
    rx_active = 0U;
    rx_status = HAL_ERROR;
    rx_error_code = huart->ErrorCode;
  }
}
