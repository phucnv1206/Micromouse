#ifndef MICROMOUSE_ODOMETRY_H
#define MICROMOUSE_ODOMETRY_H

#include "encoder.h"

typedef struct{
  /* Tọa độ trong mặt phẳng (mm) và góc hướng (độ), chuẩn hóa về [0, 360). */
  float x_mm;
  float y_mm;
  float heading_deg;
  /*
   * Vận tốc mỗi bánh được trung bình trong ODOMETRY_SPEED_AVERAGE_WINDOW_MS.
   * Tốc độ góc dùng rad/s, tốc độ tuyến tính dùng mm/s.
   */
  float wheel_angular_rad_s[ENCODER_MOTOR_COUNT];
  float wheel_linear_mm_s[ENCODER_MOTOR_COUNT];
  /* Chiều quay suy ra từ mẫu odometry gần nhất: -1 lùi, 0 đứng yên, 1 tiến. */
  int8_t wheel_direction[ENCODER_MOTOR_COUNT];
  /* Vận tốc tịnh tiến tại tâm xe (mm/s) và tốc độ quay thân xe (rad/s). */
  float linear_mm_s;
  float angular_rad_s;
} Odometry_State;

/*
 * Khởi tạo pose ban đầu, lấy snapshot encoder làm mốc và xóa các bộ tích lũy.
 * Cần gọi sau Encoder_Init(); trả HAL_ERROR nếu thông số hình học không hợp lệ
 * hoặc không đọc được encoder.
 */
HAL_StatusTypeDef Odometry_Init(void);
/*
 * Gọi định kỳ tại ODOMETRY_UPDATE_FREQUENCY_HZ sau Odometry_Init().
 * Tính chênh lệch encoder đã xử lý tràn bộ đếm, đổi sang quãng đường từng bánh,
 * rồi tích phân mô hình vi sai để cập nhật x, y và heading.
 * Vận tốc bánh được tính trung bình theo cửa sổ cấu hình riêng để giảm lượng tử
 * do encoder chỉ tạo số count nguyên.
 */
HAL_StatusTypeDef Odometry_Update(void);
/*
 * Sao chép trạng thái pose/vận tốc mới nhất ra state.
 * Trả HAL_ERROR nếu state là NULL hoặc odometry chưa được khởi tạo.
 */
HAL_StatusTypeDef Odometry_GetState(Odometry_State *state);
/*
 * Lấy tốc độ có dấu của một bánh theo mm/s; trả HAL_ERROR nếu tham số sai
 * hoặc odometry chưa khởi tạo.
 */
HAL_StatusTypeDef Odometry_GetWheelSpeedMmS(Encoder_Motor motor, float *speed_mm_s);

#endif
