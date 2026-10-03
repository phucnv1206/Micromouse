#ifndef MICROMOUSE_UART_PROTOCOL_H
#define MICROMOUSE_UART_PROTOCOL_H

#include "stm32f4xx_hal.h"

#define UART_PROTOCOL_HEADER_SIZE 2U
#define UART_PROTOCOL_TX_FRAME_SIZE 16U
#define UART_PROTOCOL_RX_FRAME_SIZE 8U
#define UART_PROTOCOL_HEADER_BYTE_1 0xAAU
#define UART_PROTOCOL_HEADER_BYTE_2 0xBBU
#define UART_PROTOCOL_COMMAND_OFFSET 2U
#define UART_PROTOCOL_DATA1_OFFSET 3U
#define UART_PROTOCOL_DATA2_OFFSET 5U

/*
 * Cấu trúc khung nhận (RX), tổng cộng 8 byte:
 *   [0..1] Header AA BB
 *   [2]    Mã lệnh
 *   [3..4] Lệnh động cơ phải, int16_t có dấu, big-endian
 *   [5..6] Lệnh động cơ trái, int16_t có dấu, big-endian
 *   [7]    Byte dự phòng
 * Lệnh động cơ được mã hóa dạng bù 2 và có miền hợp lệ -100..100.
 */

/*
 * Các trường telemetry được tuần tự hóa theo đúng thứ tự khai báo sau header
 * AA BB; mỗi trường chiếm 2 byte uint16_t và được gửi theo thứ tự big-endian.
 * Tên có hậu tố "encoded" cho biết giá trị đã được ứng dụng quy đổi về dạng
 * số nguyên trước khi đóng gói; module giao thức chỉ chịu trách nhiệm truyền.
 */
typedef struct {
  uint16_t x_mm_encoded;
  uint16_t y_mm_encoded;
  uint16_t heading_half_degrees;
  uint16_t left_speed_mm_s_encoded;
  uint16_t right_speed_mm_s_encoded;
  uint16_t data1; /* Giá trị telemetry 16-bit do ứng dụng quy định ý nghĩa. */
  uint16_t data2; /* Giá trị telemetry 16-bit do ứng dụng quy định ý nghĩa. */
} UartProtocol_Telemetry;

HAL_StatusTypeDef UartProtocol_Init(void);
/* Khởi tạo bộ phân tích/hàng đợi RX và bắt đầu nhận UART bằng ngắt. */
/* Gửi một khung TX đủ 16 byte; hai byte đầu bắt buộc là header AA BB. */
HAL_StatusTypeDef UartProtocol_Send(const uint8_t frame[UART_PROTOCOL_TX_FRAME_SIZE]);
/* Đóng gói 7 trường telemetry theo big-endian sau header rồi gửi khung TX. */
HAL_StatusTypeDef UartProtocol_SendTelemetry(const UartProtocol_Telemetry *telemetry);
/* Lấy khung RX 8 byte kế tiếp; trả HAL_BUSY nếu hiện không có khung chờ. */
HAL_StatusTypeDef UartProtocol_Receive(uint8_t frame[UART_PROTOCOL_RX_FRAME_SIZE]);
/* Bắt đầu/khởi động lại nhận UART nếu hiện không có yêu cầu nhận đang hoạt động. */
HAL_StatusTypeDef UartProtocol_StartReceive(void);
/* Đọc trạng thái nhận UART gần nhất và mã lỗi HAL để ứng dụng chẩn đoán sự cố. */
HAL_StatusTypeDef UartProtocol_GetRxStatus(void);
uint32_t UartProtocol_GetRxErrorCode(void);
/* Đọc số khung RX hoàn chỉnh bị bỏ do hàng đợi nhận đã đầy. */
uint32_t UartProtocol_GetDroppedFrameCount(void);

#endif
