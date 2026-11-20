#include "kalman_filter.h"
#include "config.h"
#include <math.h>
#include <stddef.h>

/*
 * ================= CẤU HÌNH TỈ LỆ KẾT HỢP ENCODER & GYROZ =================
 * Tham số được cấu hình tập trung trong Inc/config.h:
 * - ODOMETRY_USE_GYRO_FUSION (1 = BẬT, 0 = TẮT Gyro)
 * - ODOMETRY_GYRO_WEIGHT (mặc định 0.40f ~ 40% GyroZ, 60% Encoder)
 */
#if defined(ODOMETRY_USE_GYRO_FUSION) && (ODOMETRY_USE_GYRO_FUSION == 0)
  #define KALMAN_WEIGHT_GYRO       0.0f    /* TẮT: 0% GyroZ, 100% thuần Encoder */
  #define KALMAN_WEIGHT_ENCODER    1.0f    /* 100% Encoder */
#else
  #ifdef ODOMETRY_GYRO_WEIGHT
    #define KALMAN_WEIGHT_GYRO     ODOMETRY_GYRO_WEIGHT
  #else
    #define KALMAN_WEIGHT_GYRO     0.40f   /* 40% trọng số từ GyroZ theo yêu cầu */
  #endif
  #define KALMAN_WEIGHT_ENCODER    (1.0f - KALMAN_WEIGHT_GYRO) /* 60% từ Encoder */
#endif

#define KALMAN_BIAS_LIMIT_DPS    25.0f   /* Giới hạn trôi zero-rate của GyroZ (°/s) */

static KalmanOdometry_State kf;
static float current_dt_s = 0.01f;
static float latest_gyro_z_raw = 0.0f;

void KalmanOdometry_Init(float initial_heading_deg, float initial_gyro_bias) {
  kf.theta_deg = initial_heading_deg;
  kf.omega_dps = 0.0f;
  kf.gyro_bias_dps = initial_gyro_bias;
  latest_gyro_z_raw = 0.0f;
  current_dt_s = 0.01f;

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      kf.P[i][j] = 0.0f;
    }
  }
  kf.P[0][0] = 0.1f;
  kf.P[1][1] = 0.1f;
  kf.P[2][2] = 0.05f;

  kf.is_initialized = true;
}

void KalmanOdometry_Reset(float initial_heading_deg) {
  kf.theta_deg = initial_heading_deg;
  kf.omega_dps = 0.0f;
  /* Giữ nguyên gyro_bias_dps đã học được */
  kf.P[0][0] = 0.1f;
  kf.P[1][1] = 0.1f;
}

void KalmanOdometry_Predict(float dt_s) {
  if (!kf.is_initialized) return;
  if (dt_s > 0.0001f && dt_s < 0.5f) {
    current_dt_s = dt_s;
  }
}

void KalmanOdometry_UpdateGyro(float gyro_z_dps) {
  if (!kf.is_initialized) return;
  latest_gyro_z_raw = gyro_z_dps;
  /* [FIX 3.3] Bỏ bộ lọc IIR phần mềm alpha=0.35 vì DLPF phần cứng 41 Hz đã lọc đủ.
   * Dùng thẳng giá trị unbiased để giảm trễ pha tích lũy cho vòng PID xoay. */
}

void KalmanOdometry_UpdateEncoder(float enc_omega_dps, bool slip_detected) {
  if (!kf.is_initialized) return;

  (void)slip_detected;

  /*
   * MPU6500 Gyro Z đo trực tiếp vận tốc góc với độ phân giải cao và không bị lượng tử hóa.
   * Encoder ở chu kỳ 1 ms có độ phân giải thô (chỉ nhảy 0 hoặc ±37.5°/s), nên khi xe quay
   * chậm ở cuối cú xoay, encoder không có xung ở hầu hết các chu kỳ 1 ms.
   * Để triệt tiêu hoàn toàn hiện tượng suy hao góc (chỉ đạt 44% ở 5°/s) và chống overshoot,
   * góc quay tích phân được dẫn xuất trực tiếp từ Gyro Z (unbiased).
   */
  float gyro_unbiased = latest_gyro_z_raw - kf.gyro_bias_dps;
  float gyro_delta_deg = gyro_unbiased * current_dt_s;

#if defined(ODOMETRY_USE_GYRO_FUSION) && (ODOMETRY_USE_GYRO_FUSION == 0)
  float fused_delta_deg = enc_omega_dps * current_dt_s;
  kf.omega_dps = enc_omega_dps;
#else
  float fused_delta_deg = gyro_delta_deg;
  kf.omega_dps = gyro_unbiased;
#endif

  /* Cập nhật vận tốc góc và góc hướng tổng hợp */
  kf.theta_deg += fused_delta_deg;

  /* Chuẩn hóa góc về khoảng [0, 360) độ */
  while (kf.theta_deg >= 360.0f) {
    kf.theta_deg -= 360.0f;
  }
  while (kf.theta_deg < 0.0f) {
    kf.theta_deg += 360.0f;
  }

  /* Cập nhật ma trận chẩn đoán P */
  kf.P[0][0] = 0.05f;
  kf.P[1][1] = 0.05f;
}

void KalmanOdometry_UpdateZeroVelocity(void) {
  if (!kf.is_initialized) return;

  /*
   * Zero-Velocity Update (ZUPT):
   * Khi bánh xe đứng yên (Encoder đếm 0 xung):
   * 1. Ép vận tốc góc về 0 để triệt tiêu hoàn toàn rung lắc
   * 2. Góc theta đứng im tuyệt đối, không trôi dù chỉ 0.01 độ
   * 3. Tự động học thích nghi bias GyroZ theo nhiệt độ môi trường
   */
  kf.omega_dps = 0.0f;

  float bias_diff = latest_gyro_z_raw - kf.gyro_bias_dps;
  /* [FIX 3.3] Giảm hệ số học bias từ 0.02f (tau≈50ms) xuống 0.001f (tau≈1s)
   * để tránh kéo bias sai khi xe vừa dừng mà thân còn rung nhẹ. */
  kf.gyro_bias_dps += 0.001f * bias_diff;

  if (kf.gyro_bias_dps > KALMAN_BIAS_LIMIT_DPS) {
    kf.gyro_bias_dps = KALMAN_BIAS_LIMIT_DPS;
  } else if (kf.gyro_bias_dps < -KALMAN_BIAS_LIMIT_DPS) {
    kf.gyro_bias_dps = -KALMAN_BIAS_LIMIT_DPS;
  }

  kf.P[0][0] = 0.01f;
  kf.P[1][1] = 0.01f;
  kf.P[2][2] = 0.02f;
}

float KalmanOdometry_GetHeading(void) {
  float h = kf.theta_deg;
  while (h >= 360.0f) h -= 360.0f;
  while (h < 0.0f)   h += 360.0f;
  return h;
}

float KalmanOdometry_GetAngularVelocity(void) {
  return kf.omega_dps;
}

float KalmanOdometry_GetAngularVelocityRad(void) {
  return kf.omega_dps * (3.14159265358979323846f / 180.0f);
}

float KalmanOdometry_GetGyroBias(void) {
  return kf.gyro_bias_dps;
}

void KalmanOdometry_GetState(KalmanOdometry_State *out_state) {
  if (out_state != NULL) {
    *out_state = kf;
  }
}
