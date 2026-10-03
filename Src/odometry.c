#include "main.h"
#include "odometry.h"
#include "config.h"
#include <math.h>

#if ODOMETRY_ENABLED

#define ODOMETRY_PI 3.14159265358979323846f
#define ODOMETRY_TWO_PI (2.0f * ODOMETRY_PI)

/* Trạng thái pose/vận tốc mới nhất được các API bên dưới chia sẻ. */
static Odometry_State odometry_state;
/* Count lần đọc trước dùng để tính số xung phát sinh trong một chu kỳ odometry. */
static uint16_t previous_count[ENCODER_MOTOR_COUNT];
/* Tích lũy số bước có dấu qua nhiều mẫu để làm mượt vận tốc encoder. */
static int32_t speed_window_steps[ENCODER_MOTOR_COUNT];
/* Số mẫu đã gom trong cửa sổ trung bình vận tốc. */
static uint32_t speed_window_sample_count;
/* Chặn cập nhật/lấy state trước khi Odometry_Init hoàn tất. */
static uint8_t odometry_initialized;

/* Số lần gọi Odometry_Update tạo nên một cửa sổ tốc độ. */
#define ODOMETRY_SPEED_AVERAGE_SAMPLES \
  ((ODOMETRY_UPDATE_FREQUENCY_HZ * ODOMETRY_SPEED_AVERAGE_WINDOW_MS) / 1000U)

/* Quy đổi số bước encoder trên một vòng trục motor sang số bước một vòng bánh. */
static float Odometry_WheelStepsPerRevolution(void) {
  return (float)ENCODER_STEPS_PER_MOTOR_REV * ODOMETRY_GEAR_RATIO;
}

/*
 * Tính vận tốc góc bánh (rad/s) từ tổng bước có dấu trong cửa sổ đo.
 * sample_count / tần số cập nhật là thời gian thực của cửa sổ, tính bằng giây.
 */
static float Odometry_GetWheelAngularSpeed(int32_t wheel_step_delta, uint32_t sample_count) {
  return ((float)wheel_step_delta * ODOMETRY_TWO_PI * (float)ODOMETRY_UPDATE_FREQUENCY_HZ) / ((float)sample_count * Odometry_WheelStepsPerRevolution());
}

/* Tính vận tốc tuyến tính bánh (mm/s) từ bước encoder tích lũy trong cửa sổ. */
static float Odometry_CalculateWheelSpeedMmS(int32_t wheel_step_delta, uint32_t sample_count) {
  return ((float)wheel_step_delta * ODOMETRY_PI * ODOMETRY_WHEEL_DIAMETER_MM * (float)ODOMETRY_UPDATE_FREQUENCY_HZ) / ((float)sample_count * Odometry_WheelStepsPerRevolution());
}

/*
 * Khởi tạo trạng thái odometry:
 * kiểm tra kích thước hình học, lấy count hiện tại làm mốc để không tính
 * chuyển động xảy ra trước lúc khởi tạo, rồi xóa pose và cửa sổ vận tốc.
 */
HAL_StatusTypeDef Odometry_Init(void) {
  Encoder_Snapshot snapshot;

  /* Reject invalid wheel geometry before calculating distance or heading. */
  if (ODOMETRY_GEAR_RATIO <= 0.0f || ODOMETRY_WHEEL_DIAMETER_MM <= 0.0f || ODOMETRY_WHEEL_TRACK_MM <= 0.0f || ENCODER_STEPS_PER_MOTOR_REV == 0U || Encoder_GetSnapshot(&snapshot) != HAL_OK) {
    return HAL_ERROR;
  }

  odometry_state.x_mm = 0.0f;
  odometry_state.y_mm = 0.0f;
  odometry_state.heading_deg = ODOMETRY_INITIAL_HEADING_DEG;
  odometry_state.linear_mm_s = 0.0f;
  odometry_state.angular_rad_s = 0.0f;
  for (uint32_t i = 0U; i < ENCODER_MOTOR_COUNT; i++) {
    odometry_state.wheel_angular_rad_s[i] = 0.0f;
    odometry_state.wheel_linear_mm_s[i] = 0.0f;
    odometry_state.wheel_direction[i] = 0;
    previous_count[i] = snapshot.count[i];
    speed_window_steps[i] = 0;
  }
  speed_window_sample_count = 0U;

  odometry_initialized = 1U;
  return HAL_OK;
}

HAL_StatusTypeDef Odometry_Update(void) {
  Encoder_Snapshot snapshot;
  float distance_mm[ENCODER_MOTOR_COUNT];
  float wheel_circumference_mm;
  float steps_per_wheel_revolution;
  float left_distance_mm;
  float right_distance_mm;
  float heading_before_rad;
  float heading_delta_rad;
  float distance_center_mm;
  const int32_t count_modulus = (int32_t)ENCODER_COUNT_MAX + 1;
  const int32_t max_unambiguous_delta = count_modulus / 2;

  /*
   * Cần có snapshot trước đó để tính delta count và cần tần số gọi cố định
   * vì cả tích phân pose lẫn đổi bước sang vận tốc đều dựa vào thời gian mẫu.
   */
  if (odometry_initialized == 0U || Encoder_GetSnapshot(&snapshot) != HAL_OK) {
    return HAL_ERROR;
  }

  steps_per_wheel_revolution = Odometry_WheelStepsPerRevolution();
  wheel_circumference_mm = ODOMETRY_PI * ODOMETRY_WHEEL_DIAMETER_MM;

  /* Xử lý độc lập từng bánh: delta count, bù tràn, hiệu chỉnh chiều và đổi đơn vị. */
  for (uint32_t i = 0U; i < ENCODER_MOTOR_COUNT; i++) {
    int32_t step_delta = (int32_t)snapshot.count[i] - (int32_t)previous_count[i];
    int32_t encoder_sign = i == ENCODER_LEFT ? ODOMETRY_LEFT_ENCODER_SIGN : ODOMETRY_RIGHT_ENCODER_SIGN;
    int32_t wheel_step_delta;

    previous_count[i] = snapshot.count[i];
    /*
     * Count phần cứng chạy vòng từ 0 đến ENCODER_COUNT_MAX. Chọn hiệu có dấu
     * ngắn nhất khi count vượt biên; giả định mỗi lần cập nhật không thể mất
     * quá nửa dải đếm, nếu không chiều/độ lớn delta sẽ mơ hồ.
     */
    if (step_delta > max_unambiguous_delta) {
      step_delta -= count_modulus;
    } else if (step_delta < -max_unambiguous_delta) {
      step_delta += count_modulus;
    }

    wheel_step_delta = step_delta * encoder_sign;
    speed_window_steps[i] += wheel_step_delta;
    /* Quãng đường mẫu này dùng cho pose; không đợi hết cửa sổ lọc vận tốc. */
    distance_mm[i] = ((float)wheel_step_delta * wheel_circumference_mm) / steps_per_wheel_revolution;

    odometry_state.wheel_direction[i] = wheel_step_delta > 0 ? 1 : (wheel_step_delta < 0 ? -1 : 0);
  }

  /*
   * Tốc độ encoder có độ phân giải thô nếu chỉ dùng một mẫu. Gom nhiều mẫu
   * rồi chia theo tổng thời gian để giảm lượng tử; tốc độ giữ nguyên giữa
   * hai lần hoàn tất cửa sổ trung bình.
   */
  speed_window_sample_count++;
  if (speed_window_sample_count >= ODOMETRY_SPEED_AVERAGE_SAMPLES) {
    for (uint32_t i = 0U; i < ENCODER_MOTOR_COUNT; i++) {
      odometry_state.wheel_angular_rad_s[i] = Odometry_GetWheelAngularSpeed(speed_window_steps[i], speed_window_sample_count);
      odometry_state.wheel_linear_mm_s[i] = Odometry_CalculateWheelSpeedMmS(speed_window_steps[i], speed_window_sample_count);
      speed_window_steps[i] = 0;
    }
    speed_window_sample_count = 0U;
  }

  left_distance_mm = distance_mm[ENCODER_LEFT];
  right_distance_mm = distance_mm[ENCODER_RIGHT];
  /*
   * Mô hình truyền động vi sai:
   * d_heading = (quãng đường phải - quãng đường trái) / khoảng cách hai bánh.
   * Tịnh tiến tâm xe bằng trung bình quãng đường hai bánh.
   */
  heading_delta_rad = (right_distance_mm - left_distance_mm) / ODOMETRY_WHEEL_TRACK_MM;
  distance_center_mm = (right_distance_mm + left_distance_mm) * 0.5f;
  heading_before_rad = odometry_state.heading_deg * (ODOMETRY_PI / 180.0f);

  if (heading_delta_rad > -0.000001f && heading_delta_rad < 0.000001f) {
    /* Chạy gần thẳng: dùng góc giữa mẫu để tránh chia cho góc quay rất nhỏ. */
    float mid_heading_rad = heading_before_rad + heading_delta_rad * 0.5f;
    odometry_state.x_mm += distance_center_mm * cosf(mid_heading_rad);
    odometry_state.y_mm += distance_center_mm * sinf(mid_heading_rad);
  } else {
    /* Khi đang quay, tích phân cung tròn chính xác giữa hai mẫu odometry. */
    float heading_after_rad = heading_before_rad + heading_delta_rad;
    float turn_radius_mm = distance_center_mm / heading_delta_rad;

    odometry_state.x_mm += turn_radius_mm * (sinf(heading_after_rad) - sinf(heading_before_rad));
    odometry_state.y_mm -= turn_radius_mm * (cosf(heading_after_rad) - cosf(heading_before_rad));
  }

  odometry_state.heading_deg += heading_delta_rad * (180.0f / ODOMETRY_PI);
  while (odometry_state.heading_deg >= 360.0f) {
    odometry_state.heading_deg -= 360.0f;
  }
  while (odometry_state.heading_deg < 0.0f) {
    odometry_state.heading_deg += 360.0f;
  }

  odometry_state.linear_mm_s = (odometry_state.wheel_linear_mm_s[ENCODER_LEFT] + odometry_state.wheel_linear_mm_s[ENCODER_RIGHT]) * 0.5f;
  odometry_state.angular_rad_s = (odometry_state.wheel_linear_mm_s[ENCODER_RIGHT] - odometry_state.wheel_linear_mm_s[ENCODER_LEFT]) / ODOMETRY_WHEEL_TRACK_MM;
  return HAL_OK;
}

/* Sao chép toàn bộ trạng thái hiện hành; state có thể là biến local của caller. */
HAL_StatusTypeDef Odometry_GetState(Odometry_State *state) {
  if (state == NULL || odometry_initialized == 0U) {
    return HAL_ERROR;
  }

  *state = odometry_state;
  return HAL_OK;
}

/* Trả vận tốc bánh đã lọc theo cửa sổ trung bình, đơn vị mm/s. */
HAL_StatusTypeDef Odometry_GetWheelSpeedMmS(Encoder_Motor motor, float *speed_mm_s) {
  if (speed_mm_s == NULL || motor >= ENCODER_MOTOR_COUNT || odometry_initialized == 0U) {
    return HAL_ERROR;
  }

  *speed_mm_s = odometry_state.wheel_linear_mm_s[motor];
  return HAL_OK;
}

#else

HAL_StatusTypeDef Odometry_Init(void) {
  /* Giữ API hợp lệ khi biên dịch tắt odometry. */
  return HAL_OK;
}

HAL_StatusTypeDef Odometry_Update(void) {
  return HAL_OK;
}

HAL_StatusTypeDef Odometry_GetState(Odometry_State *state) {
  if (state == NULL) {
    return HAL_ERROR;
  }

  /* Khi tính năng bị tắt, trả một trạng thái 0 có định nghĩa rõ ràng. */
  *state = (Odometry_State){0};
  return HAL_OK;
}

HAL_StatusTypeDef Odometry_GetWheelSpeedMmS(Encoder_Motor motor, float *speed_mm_s) {
  if (speed_mm_s == NULL || motor >= ENCODER_MOTOR_COUNT) {
    return HAL_ERROR;
  }

  *speed_mm_s = 0.0f;
  return HAL_OK;
}

#endif
