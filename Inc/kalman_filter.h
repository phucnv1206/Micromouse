#ifndef MICROMOUSE_KALMAN_FILTER_H
#define MICROMOUSE_KALMAN_FILTER_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Bộ lọc Kalman kết hợp Encoder và GyroZ (MPU6500) cho Odometry.
 * Vector trạng thái:
 *   x[0] = theta: Góc hướng xe (độ, 0..360)
 *   x[1] = omega: Vận tốc góc thực tế thân xe (độ/s)
 *   x[2] = bias:  Sai số trôi (zero-rate offset bias) của GyroZ (độ/s)
 *
 * Quy ước chiều:
 *   Gyro dương  -> góc theta tăng
 *   Gyro âm     -> góc theta giảm
 */

typedef struct {
  float theta_deg;     /* Góc hướng ước lượng (độ) */
  float omega_dps;     /* Vận tốc góc ước lượng (độ/s) */
  float gyro_bias_dps; /* Bias GyroZ ước lượng (độ/s) */
  float P[3][3];       /* Ma trận hiệp phương sai sai số */
  bool  is_initialized;
} KalmanOdometry_State;

/* Khởi tạo bộ lọc với góc ban đầu và bias gyro ban đầu */
void KalmanOdometry_Init(float initial_heading_deg, float initial_gyro_bias);

/* Đặt lại góc hướng (ví dụ khi nhận lệnh Reset Odom) mà vẫn giữ nguyên bias đã học */
void KalmanOdometry_Reset(float initial_heading_deg);

/* Bước dự báo (Prediction) theo chu kỳ dt_s (giây) */
void KalmanOdometry_Predict(float dt_s);

/* Bước cập nhật đo lường từ GyroZ MPU6500 (độ/s) */
void KalmanOdometry_UpdateGyro(float gyro_z_dps);

/* Bước cập nhật đo lường từ Wheel Encoders (độ/s) */
void KalmanOdometry_UpdateEncoder(float enc_omega_dps, bool slip_detected);

/* Bước cập nhật khi xe đứng yên (Zero Velocity Update - ZUPT) */
void KalmanOdometry_UpdateZeroVelocity(void);

/* Các hàm truy xuất dữ liệu trạng thái đã lọc */
float KalmanOdometry_GetHeading(void);
float KalmanOdometry_GetAngularVelocity(void);
float KalmanOdometry_GetAngularVelocityRad(void);
float KalmanOdometry_GetGyroBias(void);
void  KalmanOdometry_GetState(KalmanOdometry_State *out_state);

#ifdef __cplusplus
}
#endif

#endif /* MICROMOUSE_KALMAN_FILTER_H */

