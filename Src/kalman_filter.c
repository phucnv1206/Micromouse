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
static float gyro_filtered_dps = 0.0f;

void KalmanOdometry_Init(float initial_heading_deg, float initial_gyro_bias) {
  kf.theta_deg = initial_heading_deg;
  kf.omega_dps = 0.0f;
  kf.gyro_bias_dps = initial_gyro_bias;
  gyro_filtered_dps = 0.0f;
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
  gyro_filtered_dps = 0.0f;
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

  /*
   * 1. Khử zero-rate bias của MPU6500
   * 2. Lọc thông thấp nhẹ IIR (alpha = 0.35) để triệt tiêu hoàn toàn rung chấn vi cơ từ motor
   */
  float unbiased = gyro_z_dps - kf.gyro_bias_dps;
  gyro_filtered_dps += 0.35f * (unbiased - gyro_filtered_dps);
}

void KalmanOdometry_UpdateEncoder(float enc_omega_dps, bool slip_detected) {
  if (!kf.is_initialized) return;

  (void)slip_detected; /* Không can thiệp theo chống trượt theo yêu cầu */

  /*
   * ================= NGUYÊN TẮC KẾT HỢP GÓC =================
   * Chỉ khi nào góc theta suy từ Encoder THAY ĐỔI (|enc_omega_dps| > 0)
   * thì mới kết hợp với GyroZ để xác định góc quay chính xác!
   * Khi Encoder không quay (theta từ encoder không đổi, e.g. xe đứng yên hoặc chạy thẳng tắp):
   * Khóa góc theta, không để GyroZ gây trôi hay rung giật.
   */
  if (fabsf(enc_omega_dps) < 0.001f) {
    kf.omega_dps = 0.0f;
    return;
  }

  /*
   * Khi góc theta từ Encoder đang thay đổi:
   * 1. Biến thiên góc từ Encoder trong chu kỳ: delta_theta_enc = enc_omega_dps * dt
   * 2. Biến thiên góc từ GyroZ trong chu kỳ: delta_theta_gyro = (gyro_z - bias) * dt
   * 3. Kết hợp xác định góc thực tế nằm trong khoảng đó:
   *    Ví dụ (Gyro 40%, Encoder 60%): Encoder quay 30°, Gyro tích phân ra 29°:
   *    fused_delta = 0.60 * 30° + 0.40 * 29° = 29.6°
   */
  float enc_delta_deg = enc_omega_dps * current_dt_s;
  float gyro_unbiased = latest_gyro_z_raw - kf.gyro_bias_dps;
  float gyro_delta_deg = gyro_unbiased * current_dt_s;

  float fused_delta_deg = (KALMAN_WEIGHT_ENCODER * enc_delta_deg) +
                          (KALMAN_WEIGHT_GYRO    * gyro_delta_deg);

  /* Cập nhật vận tốc góc và góc hướng tổng hợp */
  kf.omega_dps = fused_delta_deg / current_dt_s;
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
  gyro_filtered_dps = 0.0f;

  float bias_diff = latest_gyro_z_raw - kf.gyro_bias_dps;
  kf.gyro_bias_dps += 0.02f * bias_diff;

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
