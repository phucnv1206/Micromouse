#ifndef MICROMOUSE_MPU6050_H
#define MICROMOUSE_MPU6050_H

#include "stm32f4xx_hal.h"

/*
 * Module đọc gyro Z của MPU6050 qua I2C1 dùng chung với bus ToF.
 * Khi MPU6050_ENABLED = 0, tất cả hàm vẫn tồn tại nhưng trả giá trị an toàn
 * (HAL_OK hoặc 0.0f) để phần còn lại firmware không cần nhánh điều kiện.
 *
 * Luồng sử dụng:
 *   1. Gọi Mpu6050_Init() một lần sau I2C1 đã khởi tạo.
 *   2. Gọi Mpu6050_Update() ở tần số ODOMETRY_UPDATE_FREQUENCY_HZ (1000 Hz)
 *      cùng nhịp với Odometry_Update().
 *   3. Đọc kết quả đã lọc qua Mpu6050_GetGyroZ().
 */

/*
 * Khởi tạo MPU6050: thức thiết bị, đặt full-scale gyro range và xóa bộ lọc.
 * Trả HAL_ERROR nếu thiết bị không phản hồi hoặc ghi cấu hình thất bại.
 */
HAL_StatusTypeDef Mpu6050_Init(void);

/*
 * Đọc hai byte thanh ghi GYRO_ZOUT và cập nhật low-pass filter.
 * Gọi định kỳ tại ODOMETRY_UPDATE_FREQUENCY_HZ để bộ lọc hoạt động đúng
 * hằng số thời gian được thiết kế. Trả HAL_ERROR nếu chưa khởi tạo hoặc
 * giao tiếp I2C thất bại.
 */
HAL_StatusTypeDef Mpu6050_Update(void);

/*
 * Trả vận tốc góc quanh trục Z đã lọc, đơn vị °/s.
 * Khi MPU6050_ENABLED = 0 hoặc chưa gọi Mpu6050_Init(), luôn trả 0.0f.
 * Đây là hàm tiện ích theo cú pháp yêu cầu: gyroZ = getGyro();
 */
float Mpu6050_GetGyroZ(void);

#endif /* MICROMOUSE_MPU6050_H */

