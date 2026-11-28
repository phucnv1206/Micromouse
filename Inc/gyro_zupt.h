#ifndef MICROMOUSE_GYRO_ZUPT_H
#define MICROMOUSE_GYRO_ZUPT_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Bộ ước lượng hướng từ GyroZ (MPU6500) kết hợp bù trôi & khóa đứng yên (ZUPT)
 * và dự phòng tích phân góc từ Encoder.
 * Trạng thái:
 *   theta_deg     : Góc hướng xe (độ, [0, 360))
 *   omega_dps     : Vận tốc góc thực tế thân xe (độ/s)
 *   gyro_bias_dps : Sai số trôi (zero-rate offset bias) của GyroZ (độ/s)
 */

typedef struct {
  float theta_deg;     /* Góc hướng ước lượng (độ) */
  float omega_dps;     /* Vận tốc góc ước lượng (độ/s) */
  float gyro_bias_dps; /* Bias GyroZ ước lượng (độ/s) */
  bool  is_initialized;
} GyroZupt_State;

/* Khởi tạo bộ ước lượng với góc ban đầu và bias gyro ban đầu */
void GyroZupt_Init(float initial_heading_deg, float initial_gyro_bias);

/* Đặt lại góc hướng (ví dụ khi nhận lệnh Reset Odom) mà vẫn giữ nguyên bias đã học */
void GyroZupt_Reset(float initial_heading_deg);

/* Cập nhật chu kỳ dt_s (giây) */
void GyroZupt_SetDt(float dt_s);

/* Cập nhật giá trị đo vận tốc góc thô từ GyroZ MPU6500 (độ/s) */
void GyroZupt_UpdateGyro(float gyro_z_dps);

/* Tích phân góc hướng khi xe di chuyển (dùng GyroZ đã bù trôi, dự phòng Encoder) */
void GyroZupt_UpdateHeading(float enc_omega_dps);

/* Khóa đứng yên khi hai bánh dừng (Zero-Velocity Update - ZUPT) */
void GyroZupt_UpdateZeroVelocity(void);

/* Các hàm truy xuất trạng thái góc và vận tốc góc */
float GyroZupt_GetHeading(void);
float GyroZupt_GetAngularVelocity(void);
float GyroZupt_GetAngularVelocityRad(void);
float GyroZupt_GetGyroBias(void);
void  GyroZupt_GetState(GyroZupt_State *out_state);

#ifdef __cplusplus
}
#endif

#endif /* MICROMOUSE_GYRO_ZUPT_H */
