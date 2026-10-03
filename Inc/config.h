#ifndef MICROMOUSE_CONFIG_H
#define MICROMOUSE_CONFIG_H

#include "stm32f4xx_hal.h"

/* Scheduler rates and servo-style motor pulse settings. */
#define CONTROL_LOOP_FREQUENCY_HZ 100U
#define UART_TELEMETRY_FREQUENCY_HZ 40U
#ifndef ODOMETRY_ENABLED
#define ODOMETRY_ENABLED 1U
#endif
#ifndef ENCODER_ENABLED
#define ENCODER_ENABLED 1U
#endif
#ifndef TOF_ENABLED
#define TOF_ENABLED 1U
#endif

#define ODOMETRY_UPDATE_FREQUENCY_HZ 1000U
#define ODOMETRY_TIMER_COUNTER_HZ 1000000U
#define ODOMETRY_SPEED_AVERAGE_WINDOW_MS 10U
#define MOTOR_PWM_PULSE_MIN_US 600U
#define MOTOR_PWM_PULSE_MAX_US 2400U
#define MOTOR_COMMAND_MAX 100
/*
 * Hiệu chuẩn PWM độc lập cho từng bánh:
 * offset là xung dừng thực tế; deadzone là khoảng đầu ra tối thiểu để thắng
 * vùng chết; maxzone là dải xung điều khiển cộng thêm sau mép deadzone.
 * Ví dụ offset=1500, deadzone=100, maxzone=200:
 * dải đầu ra là 1200..1800 us; lệnh +1..+100 đi từ ngay ngoài 1600 đến 1800 us.
 */
#define MOTOR_RIGHT_PWM_OFFSET_US 1532U
#define MOTOR_RIGHT_PWM_DEADZONE_US 35U
#define MOTOR_RIGHT_PWM_MAXZONE_US 800U
#define MOTOR_LEFT_PWM_OFFSET_US 1526U
#define MOTOR_LEFT_PWM_DEADZONE_US 35U
#define MOTOR_LEFT_PWM_MAXZONE_US 800U
#define MOTOR_LEFT_REVERSE_DIRECTION 1U
#define MOTOR_PWM_FREQUENCY_HZ 50U
#define MOTOR_PWM_COUNTER_HZ 1000000U
/* Nút dùng pull-up: trạng thái nhấn là RESET; chống dội 20 ms, giữ 800 ms phát HOLD. */
#define BUTTON_DEBOUNCE_MS 20U
#define BUTTON_HOLD_TIME_MS 800U
/* Tốc độ I2C1 và timeout cho thao tác blocking dò thiết bị. */
#define I2C_CLOCK_SPEED_HZ 400000U
#define I2C_TIMEOUT_MS 100U
/* Địa chỉ I2C 7-bit riêng và thời gian tối đa cho một lần đo của mỗi VL53L0X. */
#define VL53L0X_DEFAULT_I2C_ADDRESS 0x29U
#define TOF_SENSOR_RIGHT_I2C_ADDRESS 0x30U
#define TOF_SENSOR_MID_I2C_ADDRESS 0x31U
#define TOF_SENSOR_LEFT_I2C_ADDRESS 0x32U
#define TOF_OUTER_MEASUREMENT_TIMING_BUDGET_US 25000U /* Budget trái/phải; mục tiêu khoảng 40 Hz. */
/* VL53L1X ULD chỉ hỗ trợ budget rời rạc (20 ms là mức gần 25 ms nhất). */
#define TOF_MID_TIMING_BUDGET_MS 20U
#define TOF_MID_INTER_MEASUREMENT_MS 25U
/* Thời gian giữ XSHUT thấp, đợi boot và giãn cách khởi tạo giữa các cảm biến. */
#define TOF_XSHUT_RESET_DELAY_MS 2U
#define TOF_SENSOR_BOOT_DELAY_MS 10U
#define TOF_SENSOR_INTER_INIT_DELAY_MS 100U

/* Wheel and encoder values used by odometry. */
#define ENCODER_COUNT_MAX 10000U
#define ENCODER_STEPS_PER_MOTOR_REV 28U
#define ODOMETRY_GEAR_RATIO 100.0f
#define ODOMETRY_WHEEL_DIAMETER_MM 44.4f
#define ODOMETRY_WHEEL_TRACK_MM 77.8f
#define ODOMETRY_LEFT_ENCODER_SIGN -1
#define ODOMETRY_RIGHT_ENCODER_SIGN 1
#define ODOMETRY_INITIAL_HEADING_DEG 90.0f

/* Board pin assignments. */

#define LED1 GPIO_PIN_13
#define LED1_Port GPIOC
#define LED2 GPIO_PIN_14
#define LED2_Port GPIOC
#define LED3 GPIO_PIN_15
#define LED3_Port GPIOC 
#define BUTTON1 GPIO_PIN_0
#define BUTTON1_Port GPIOA
#define BUTTON2 GPIO_PIN_1
#define BUTTON2_Port GPIOA
#define TX GPIO_PIN_2
#define TX_Port GPIOA
#define RX GPIO_PIN_3
#define RX_Port GPIOA
#define TOF_XSHUT_RIGHT GPIO_PIN_13
#define TOF_XSHUT_RIGHT_Port GPIOB
#define TOF_XSHUT_MID GPIO_PIN_14
#define TOF_XSHUT_MID_Port GPIOB
#define TOF_XSHUT_LEFT GPIO_PIN_15
#define TOF_XSHUT_LEFT_Port GPIOB
#define MOTOR_RIGHT GPIO_PIN_8
#define MOTOR_RIGHT_Port GPIOA
#define EN_RIGHT_B GPIO_PIN_11
#define EN_RIGHT_B_Port GPIOA
#define EN_RIGHT_B_EXTI_IRQn EXTI15_10_IRQn
#define EN_RIGHT_A GPIO_PIN_12
#define EN_RIGHT_A_Port GPIOA
#define EN_RIGHT_A_EXTI_IRQn EXTI15_10_IRQn
#define EN_LEFT_A GPIO_PIN_4
#define EN_LEFT_A_Port GPIOB
#define EN_LEFT_A_EXTI_IRQn EXTI4_IRQn
#define EN_LEFT_B GPIO_PIN_5
#define EN_LEFT_B_Port GPIOB
#define EN_LEFT_B_EXTI_IRQn EXTI9_5_IRQn
#define SCL GPIO_PIN_6
#define SCL_Port GPIOB
#define SDA GPIO_PIN_7
#define SDA_Port GPIOB
#define MOTOR_LEFT GPIO_PIN_9
#define MOTOR_LEFT_Port GPIOB

void Config_Init(void);
HAL_StatusTypeDef Config_I2C1_Init(void);
HAL_StatusTypeDef Config_OdometryTimer_Init(void);
void Config_OdometryTimer_Start(void);
void Config_I2C1_GPIO_Init(void);
void Config_I2C1_GPIO_DeInit(void);
void Config_TIM1_GPIO_Init(void);
void Config_TIM11_GPIO_Init(void);
void Config_USART2_GPIO_Init(void);
void Config_USART2_GPIO_DeInit(void);

/* Kiểm tra chéo các giá trị cấu hình sau khi toàn bộ macro đã được khai báo. */
#if ODOMETRY_ENABLED && !ENCODER_ENABLED
#error "ODOMETRY_ENABLED requires ENCODER_ENABLED"
#endif

#if CONTROL_LOOP_FREQUENCY_HZ < 1U || CONTROL_LOOP_FREQUENCY_HZ > 1000U
#error "CONTROL_LOOP_FREQUENCY_HZ must be between 1 and 1000"
#endif

#if ODOMETRY_UPDATE_FREQUENCY_HZ < 1U || ODOMETRY_UPDATE_FREQUENCY_HZ > ODOMETRY_TIMER_COUNTER_HZ
#error "ODOMETRY_UPDATE_FREQUENCY_HZ must be between 1 and ODOMETRY_TIMER_COUNTER_HZ"
#endif

#if 1000U % CONTROL_LOOP_FREQUENCY_HZ != 0U
#error "CONTROL_LOOP_FREQUENCY_HZ must divide 1000"
#endif

#if ODOMETRY_TIMER_COUNTER_HZ % ODOMETRY_UPDATE_FREQUENCY_HZ != 0U
#error "ODOMETRY_UPDATE_FREQUENCY_HZ must divide ODOMETRY_TIMER_COUNTER_HZ"
#endif

#if ODOMETRY_SPEED_AVERAGE_WINDOW_MS < 1U \
    || (ODOMETRY_UPDATE_FREQUENCY_HZ * ODOMETRY_SPEED_AVERAGE_WINDOW_MS) % 1000U != 0U
#error "ODOMETRY_SPEED_AVERAGE_WINDOW_MS must contain a whole number of odometry updates"
#endif

#if UART_TELEMETRY_FREQUENCY_HZ < 1U || 1000U % UART_TELEMETRY_FREQUENCY_HZ != 0U
#error "UART_TELEMETRY_FREQUENCY_HZ must divide 1000"
#endif

#if MOTOR_PWM_FREQUENCY_HZ < 1U || MOTOR_PWM_COUNTER_HZ % MOTOR_PWM_FREQUENCY_HZ != 0U
#error "MOTOR_PWM_FREQUENCY_HZ must divide MOTOR_PWM_COUNTER_HZ"
#endif

#if MOTOR_RIGHT_PWM_OFFSET_US <= (MOTOR_RIGHT_PWM_DEADZONE_US + MOTOR_RIGHT_PWM_MAXZONE_US) \
    || MOTOR_RIGHT_PWM_OFFSET_US + MOTOR_RIGHT_PWM_DEADZONE_US + MOTOR_RIGHT_PWM_MAXZONE_US > MOTOR_PWM_PULSE_MAX_US \
    || MOTOR_RIGHT_PWM_OFFSET_US - MOTOR_RIGHT_PWM_DEADZONE_US - MOTOR_RIGHT_PWM_MAXZONE_US < MOTOR_PWM_PULSE_MIN_US \
    || MOTOR_RIGHT_PWM_MAXZONE_US == 0U
#error "Invalid right motor PWM offset, deadzone, or maxzone"
#endif

#if MOTOR_LEFT_PWM_OFFSET_US <= (MOTOR_LEFT_PWM_DEADZONE_US + MOTOR_LEFT_PWM_MAXZONE_US) \
    || MOTOR_LEFT_PWM_OFFSET_US + MOTOR_LEFT_PWM_DEADZONE_US + MOTOR_LEFT_PWM_MAXZONE_US > MOTOR_PWM_PULSE_MAX_US \
    || MOTOR_LEFT_PWM_OFFSET_US - MOTOR_LEFT_PWM_DEADZONE_US - MOTOR_LEFT_PWM_MAXZONE_US < MOTOR_PWM_PULSE_MIN_US \
    || MOTOR_LEFT_PWM_MAXZONE_US == 0U
#error "Invalid left motor PWM offset, deadzone, or maxzone"
#endif

#if TOF_SENSOR_RIGHT_I2C_ADDRESS < 0x08U || TOF_SENSOR_RIGHT_I2C_ADDRESS > 0x77U \
    || TOF_SENSOR_MID_I2C_ADDRESS < 0x08U || TOF_SENSOR_MID_I2C_ADDRESS > 0x77U \
    || TOF_SENSOR_LEFT_I2C_ADDRESS < 0x08U || TOF_SENSOR_LEFT_I2C_ADDRESS > 0x77U
#error "VL53L0X addresses must be valid 7-bit I2C addresses"
#endif

#if TOF_SENSOR_RIGHT_I2C_ADDRESS == TOF_SENSOR_MID_I2C_ADDRESS \
    || TOF_SENSOR_RIGHT_I2C_ADDRESS == TOF_SENSOR_LEFT_I2C_ADDRESS \
    || TOF_SENSOR_MID_I2C_ADDRESS == TOF_SENSOR_LEFT_I2C_ADDRESS
#error "Each ToF sensor must have a unique I2C address"
#endif

#if TOF_OUTER_MEASUREMENT_TIMING_BUDGET_US < 17000U
#error "VL53L0X timing budget must be at least 17000 microseconds"
#endif

#if TOF_MID_TIMING_BUDGET_MS < 15U || TOF_MID_INTER_MEASUREMENT_MS < TOF_MID_TIMING_BUDGET_MS
#error "VL53L1X needs a supported timing budget and an inter-measurement period at least as long"
#endif

#endif /* MICROMOUSE_CONFIG_H */
