#include "gyro_zupt.h"
#include "config.h"
#include <math.h>
#include <stddef.h>

#define GYRO_ZUPT_BIAS_LIMIT_DPS 25.0f /* Giới hạn trôi zero-rate của GyroZ (°/s) */

static GyroZupt_State s_zupt;
static float current_dt_s = 0.01f;
static float latest_gyro_z_raw = 0.0f;

void GyroZupt_Init(float initial_heading_deg, float initial_gyro_bias) {
  s_zupt.theta_deg = initial_heading_deg;
  s_zupt.omega_dps = 0.0f;
  s_zupt.gyro_bias_dps = initial_gyro_bias;
  latest_gyro_z_raw = 0.0f;
  current_dt_s = 0.01f;
  s_zupt.is_initialized = true;
}

void GyroZupt_Reset(float initial_heading_deg) {
  s_zupt.theta_deg = initial_heading_deg;
  s_zupt.omega_dps = 0.0f;
  /* Giữ nguyên gyro_bias_dps đã học được */
}

void GyroZupt_SetDt(float dt_s) {
  if (!s_zupt.is_initialized) return;
  if (dt_s > 0.0001f && dt_s < 0.5f) {
    current_dt_s = dt_s;
  }
}

void GyroZupt_UpdateGyro(float gyro_z_dps) {
  if (!s_zupt.is_initialized) return;
  latest_gyro_z_raw = gyro_z_dps;
  /* DLPF phần cứng 41 Hz đã lọc nhiễu cao tần; dùng thẳng giá trị unbiased để giảm trễ pha. */
}

void GyroZupt_UpdateHeading(float enc_omega_dps) {
  if (!s_zupt.is_initialized) return;

  /*
   * MPU6500 Gyro Z đo trực tiếp vận tốc góc với độ phân giải cao và không bị lượng tử hóa.
   * Encoder ở chu kỳ 1 ms có độ phân giải thô (chỉ nhảy 0 hoặc ±37.1°/s), nên khi xe quay
   * chậm ở cuối cú xoay, encoder không có xung ở hầu hết các chu kỳ 1 ms.
   * Góc quay tích phân được dẫn xuất trực tiếp từ Gyro Z đã bù trôi (hoặc dự phòng Encoder nếu tắt Gyro).
   */
  float gyro_unbiased = latest_gyro_z_raw - s_zupt.gyro_bias_dps;
  float gyro_delta_deg = gyro_unbiased * current_dt_s;

#if defined(ODOMETRY_USE_GYRO_FUSION) && (ODOMETRY_USE_GYRO_FUSION == 0)
  float step_delta_deg = enc_omega_dps * current_dt_s;
  s_zupt.omega_dps = enc_omega_dps;
#else
  (void)enc_omega_dps;
  float step_delta_deg = gyro_delta_deg;
  s_zupt.omega_dps = gyro_unbiased;
#endif

  s_zupt.theta_deg += step_delta_deg;

  /* Chuẩn hóa góc về khoảng [0, 360) độ */
  while (s_zupt.theta_deg >= 360.0f) {
    s_zupt.theta_deg -= 360.0f;
  }
  while (s_zupt.theta_deg < 0.0f) {
    s_zupt.theta_deg += 360.0f;
  }
}

void GyroZupt_UpdateZeroVelocity(void) {
  if (!s_zupt.is_initialized) return;

  /*
   * Zero-Velocity Update (ZUPT):
   * Khi bánh xe đứng yên (Encoder đếm 0 xung):
   * 1. Ép vận tốc góc về 0 để triệt tiêu rung lắc giả
   * 2. Giữ nguyên góc theta không trôi
   * 3. Tự động cập nhật bias GyroZ bằng bộ lọc thông thấp bậc nhất (tau ~ 1 s)
   */
  s_zupt.omega_dps = 0.0f;

  float bias_diff = latest_gyro_z_raw - s_zupt.gyro_bias_dps;
  s_zupt.gyro_bias_dps += 0.001f * bias_diff;

  if (s_zupt.gyro_bias_dps > GYRO_ZUPT_BIAS_LIMIT_DPS) {
    s_zupt.gyro_bias_dps = GYRO_ZUPT_BIAS_LIMIT_DPS;
  } else if (s_zupt.gyro_bias_dps < -GYRO_ZUPT_BIAS_LIMIT_DPS) {
    s_zupt.gyro_bias_dps = -GYRO_ZUPT_BIAS_LIMIT_DPS;
  }
}

float GyroZupt_GetHeading(void) {
  float h = s_zupt.theta_deg;
  while (h >= 360.0f) h -= 360.0f;
  while (h < 0.0f)   h += 360.0f;
  return h;
}

float GyroZupt_GetAngularVelocity(void) {
  return s_zupt.omega_dps;
}

float GyroZupt_GetAngularVelocityRad(void) {
  return s_zupt.omega_dps * (3.14159265358979323846f / 180.0f);
}

float GyroZupt_GetGyroBias(void) {
  return s_zupt.gyro_bias_dps;
}

void GyroZupt_GetState(GyroZupt_State *out_state) {
  if (out_state != NULL) {
    *out_state = s_zupt;
  }
}
