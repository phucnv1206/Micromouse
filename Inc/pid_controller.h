#ifndef MICROMOUSE_PID_CONTROLLER_H
#define MICROMOUSE_PID_CONTROLLER_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ================= MÃ LỆNH ĐIỀU KHIỂN (COMMAND CODES) ================= */
#define CMD_MANUAL_DRIVE  0x00U /* Điều khiển trực tiếp motor (data1 = right, data2 = left) */
#define CMD_STOP          0x00U /* Dừng xe (data1 = 0, data2 = 0) */
#define CMD_ROTATE_ANGLE  0x03U /* Xoay tại chỗ một góc chỉ định PID (data1 = delta_angle_deg, data2 = 0) */
#define CMD_RESET_ODOM    0x04U /* Reset Odometry về (0,0) và 90° */
#define CMD_EMERGENCY_STOP 0x05U /* Dừng khẩn cấp mọi lệnh tự động */
#define CMD_NAV_TO_POINT  0x06U /* Chạy tới điểm tọa độ đích (data1 = target_x_mm, data2 = target_y_mm) */

/* ================= TRẠNG THÁI PHẢN HỒI LỆNH (DATA15 LOW BYTE) ================= */
#define STATUS_IDLE       0x00U /* Sẵn sàng / Không có lệnh đang chạy */
#define STATUS_RUNNING    0x01U /* Đang thực thi lệnh */
#define STATUS_COMPLETED  0x02U /* Lệnh đã thực hiện hoàn thành thành công */
#define STATUS_ERROR      0x03U /* Báo lỗi (quá thời gian, kẹt bánh) */

/* Chế độ chuyển động của xe */
typedef enum {
  MOTION_MODE_STOP = 0,
  MOTION_MODE_MANUAL,
  MOTION_MODE_ROTATE_ANGLE,
  MOTION_MODE_NAV_TO_POINT
} MotionMode;

/* Khởi tạo hệ thống điều khiển chuyển động */
void Motion_Init(void);

/* Xử lý nhận lệnh mới từ UART */
void Motion_ProcessCommand(uint8_t cmd, int16_t data1, int16_t data2);

/* Cập nhật vòng điều khiển PID (gọi định kỳ 100 Hz trong loop) */
void Motion_Update(float dt_s);

/* Đặt trạng thái dừng khẩn cấp */
void Motion_Stop(void);

/* Lấy trạng thái phản hồi lệnh (đóng gói vào telemetry data15: Byte cao = Mã lệnh, Byte thấp = Trạng thái) */
int16_t Motion_GetFeedbackStatusWord(void);

/* Báo trạng thái cho lệnh */
void Motion_SetCommandStatus(uint8_t cmd, uint8_t status);

#ifdef __cplusplus
}
#endif

#endif /* MICROMOUSE_PID_CONTROLLER_H */
