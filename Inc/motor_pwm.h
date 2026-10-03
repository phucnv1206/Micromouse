#ifndef MICROMOUSE_MOTOR_PWM_H
#define MICROMOUSE_MOTOR_PWM_H

#include "config.h"

typedef enum {
  MOTOR_PWM_RIGHT,
  MOTOR_PWM_LEFT
} MotorPwm_Channel;

/*
 * Khởi tạo hai kênh PWM và đặt xung trung tính.
 * Trả HAL_ERROR nếu cấu hình hoặc khởi động timer PWM thất bại.
 */
HAL_StatusTypeDef MotorPwm_Init(void);
/*
 * Đặt lệnh tốc độ cho motor trong khoảng -100..100; 0 là dừng tại offset.
 * Lệnh có dấu được ánh xạ qua deadzone/maxzone riêng của motor thành độ rộng
 * xung PWM micro giây. Trả HAL_ERROR nếu motor hoặc lệnh không hợp lệ.
 */
HAL_StatusTypeDef MotorPwm_Set(MotorPwm_Channel motor, int16_t command);
/* Hàm tiện ích đặt lệnh tốc độ có dấu cho motor bên phải/bên trái. */
HAL_StatusTypeDef RunMotorRight(int16_t command);
HAL_StatusTypeDef RunMotorLeft(int16_t command);

#endif
