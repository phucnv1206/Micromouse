#ifndef MICROMOUSE_MPU6500_H
#define MICROMOUSE_MPU6500_H

#include "stm32f4xx_hal.h"

/*
 * Module điều khiển cảm biến IMU MPU6500 qua giao tiếp I2C2 riêng biệt.
 * Chân giao tiếp: PB10 (SCL, AF4), PB3 (SDA, AF9 - đã tắt chức năng SWV).
 *
 * Quy trình sử dụng:
 *   1. Mpu6500_Init(): Khởi tạo I2C2, giải phóng chân PB3, cấu hình clock PLL và dải đo ±250 dps.
 *   2. Mpu6500_Update(): Gọi định kỳ ở tần số 1000 Hz đồng bộ cùng Odometry để lọc low-pass Gyro Z.
 *   3. Mpu6500_GetGyroZ(): Đọc vận tốc góc trục Z đã lọc (°/s).
 */

HAL_StatusTypeDef Mpu6500_Init(void);
HAL_StatusTypeDef Mpu6500_Update(void);
float Mpu6500_GetGyroZ(void);

/* Các hàm chẩn đoán MPU6500 */
int16_t Mpu6500_GetInitStep(void);
uint32_t Mpu6500_GetReadCount(void);
uint32_t Mpu6500_GetLastHalError(void);

#endif /* MICROMOUSE_MPU6500_H */

