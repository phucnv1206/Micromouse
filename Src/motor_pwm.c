#include "motor_pwm.h"
#include "config.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim11;

/*
 * Ánh xạ lệnh tốc độ có dấu sang độ rộng xung:
 * 0 trả offset; lệnh khác 0 bắt đầu ngay ngoài mép deadzone.
 * Độ lớn lệnh quyết định phần trăm dải maxzone sử dụng.
 */
static uint16_t MotorPwm_MapCommand(int16_t command, uint16_t offset_us, uint16_t deadzone_us, uint16_t maxzone_us) {
  uint32_t magnitude;
  uint32_t command_magnitude;

  if (command == 0) {
    return offset_us;
  }

  command_magnitude = (uint32_t)(command < 0 ? -command : command);
  magnitude = deadzone_us + (command_magnitude * maxzone_us + (MOTOR_COMMAND_MAX / 2)) / MOTOR_COMMAND_MAX;

  return command < 0 ? (uint16_t)(offset_us - magnitude) : (uint16_t)(offset_us + magnitude);
}

/* Đặt trước xung trung tính rồi khởi chạy hai kênh timer PWM. */
HAL_StatusTypeDef MotorPwm_Init(void) {
  if (MotorPwm_Set(MOTOR_PWM_RIGHT, 0) != HAL_OK || MotorPwm_Set(MOTOR_PWM_LEFT, 0) != HAL_OK) {
    return HAL_ERROR;
  }

  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK) {
    return HAL_ERROR;
  }

  if (HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1) != HAL_OK) {
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    return HAL_ERROR;
  }

  return HAL_OK;
}

HAL_StatusTypeDef MotorPwm_Set(MotorPwm_Channel motor, int16_t command) {
  TIM_HandleTypeDef *timer;
  uint32_t channel;
  uint32_t pulse;
  uint16_t offset_us;
  uint16_t deadzone_us;
  uint16_t maxzone_us;

  /* Bão hòa lệnh để PWM không thể nhận giá trị ngoài miền -100..100. */
  if (command < -MOTOR_COMMAND_MAX) {
    command = -MOTOR_COMMAND_MAX;
  } else if (command > MOTOR_COMMAND_MAX) {
    command = MOTOR_COMMAND_MAX;
  }

  /* Ánh xạ từng bánh tới timer/kênh PWM đã nối trên bo mạch. */
  switch (motor)
  {
    case MOTOR_PWM_RIGHT:
      timer = &htim1;
      channel = TIM_CHANNEL_1;
      offset_us = MOTOR_RIGHT_PWM_OFFSET_US;
      deadzone_us = MOTOR_RIGHT_PWM_DEADZONE_US;
      maxzone_us = MOTOR_RIGHT_PWM_MAXZONE_US;
      break;
    case MOTOR_PWM_LEFT:
      timer = &htim11;
      channel = TIM_CHANNEL_1;
      offset_us = MOTOR_LEFT_PWM_OFFSET_US;
      deadzone_us = MOTOR_LEFT_PWM_DEADZONE_US;
      maxzone_us = MOTOR_LEFT_PWM_MAXZONE_US;
      break;
    default:
      return HAL_ERROR;
  }

  uint16_t pulse_width_us = MotorPwm_MapCommand(command, offset_us, deadzone_us, maxzone_us);

#if MOTOR_LEFT_REVERSE_DIRECTION
  /* Đảo chiều motor trái bằng cách phản chiếu xung quanh offset đã hiệu chỉnh. */
  if (motor == MOTOR_PWM_LEFT) {
    pulse_width_us = (2U * offset_us) - pulse_width_us;
  }
#endif

  /*
   * Đổi độ rộng xung sang giá trị compare. Với bộ đếm timer 1 MHz,
   * mỗi count tương ứng 1 micro giây; phép cộng 500000 làm tròn phép chia.
   */
  pulse = ((uint32_t)pulse_width_us * MOTOR_PWM_COUNTER_HZ + 500000U) / 1000000U;
  if (pulse > __HAL_TIM_GET_AUTORELOAD(timer) + 1U) {
    return HAL_ERROR;
  }
  __HAL_TIM_SET_COMPARE(timer, channel, pulse);

  return HAL_OK;
}

/* Các wrapper đặt tên theo vị trí bánh để nơi sử dụng dễ đọc hơn. */
HAL_StatusTypeDef RunMotorRight(int16_t command) {
  return MotorPwm_Set(MOTOR_PWM_RIGHT, command);
}

HAL_StatusTypeDef RunMotorLeft(int16_t command) {
  return MotorPwm_Set(MOTOR_PWM_LEFT, command);
}
