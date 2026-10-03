#ifndef MICROMOUSE_UART_PROTOCOL_H
#define MICROMOUSE_UART_PROTOCOL_H

#include "stm32f4xx_hal.h"

#define UART_PROTOCOL_HEADER_SIZE 2U
#define UART_PROTOCOL_TX_FRAME_SIZE 16U
#define UART_PROTOCOL_RX_FRAME_SIZE 8U
#define UART_PROTOCOL_HEADER_BYTE_1 0xAAU
#define UART_PROTOCOL_HEADER_BYTE_2 0xBBU

/*
 * Các trường RX tương ứng khung dây 8 byte:
 * header1, header2, command, data1 (int16 big-endian),
 * data2 (int16 big-endian), data3.
 */
typedef struct {
  uint8_t header1;
  uint8_t header2;
  uint8_t command;
  int16_t data1;
  int16_t data2;
  uint8_t data3;
} UartProtocol_RxFrame;

/* Bảy giá trị debug int16_t, tuần tự hóa big-endian sau header AA BB. */
typedef struct {
  int16_t data1;
  int16_t data2;
  int16_t data3;
  int16_t data4;
  int16_t data5;
  int16_t data6;
  int16_t data7;
} UartProtocol_Telemetry;

HAL_StatusTypeDef UartProtocol_Init(void);
/* Khởi tạo bộ phân tích/hàng đợi RX và bắt đầu nhận UART bằng ngắt. */
/* Gửi một khung TX đủ 16 byte; hai byte đầu bắt buộc là header AA BB. */
HAL_StatusTypeDef UartProtocol_Send(const uint8_t frame[UART_PROTOCOL_TX_FRAME_SIZE]);
/* Đóng gói 7 trường telemetry theo big-endian sau header rồi gửi khung TX. */
HAL_StatusTypeDef UartProtocol_SendTelemetry(const UartProtocol_Telemetry *telemetry);
/* Lấy khung RX kế tiếp, parse data1/data2 thành số int16_t có dấu. */
HAL_StatusTypeDef UartProtocol_Receive(UartProtocol_RxFrame *frame);
/* Bắt đầu/khởi động lại nhận UART nếu hiện không có yêu cầu nhận đang hoạt động. */
HAL_StatusTypeDef UartProtocol_StartReceive(void);
/* Đọc trạng thái nhận UART gần nhất và mã lỗi HAL để ứng dụng chẩn đoán sự cố. */
HAL_StatusTypeDef UartProtocol_GetRxStatus(void);
uint32_t UartProtocol_GetRxErrorCode(void);
/* Đọc số khung RX hoàn chỉnh bị bỏ do hàng đợi nhận đã đầy. */
uint32_t UartProtocol_GetDroppedFrameCount(void);

#endif
