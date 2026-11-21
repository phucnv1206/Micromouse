#ifndef MICROMOUSE_CONFIG_H
#define MICROMOUSE_CONFIG_H

#include "stm32f4xx_hal.h"

/* Scheduler rates and servo-style motor pulse settings. */
#define CONTROL_LOOP_FREQUENCY_HZ 100U
#define UART_TELEMETRY_FREQUENCY_HZ 40U

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
#define I2C_CLOCK_SPEED_HZ 800000U
#define I2C_TIMEOUT_MS 100U
/* Địa chỉ I2C 7-bit riêng và thời gian tối đa cho một lần đo của mỗi VL53L0X. */
#define VL53L0X_DEFAULT_I2C_ADDRESS 0x29U
#define TOF_SENSOR_RIGHT_I2C_ADDRESS 0x30U
#define TOF_SENSOR_MID_I2C_ADDRESS 0x31U
#define TOF_SENSOR_LEFT_I2C_ADDRESS 0x32U
#define TOF_OUTER_MEASUREMENT_TIMING_BUDGET_US 25000U /* Budget trái/phải; mục tiêu khoảng 40 Hz. */
/* Trọng số mẫu mới của low-pass ToF (1..100%); giá trị lớn phản hồi nhanh hơn. */
#define TOF_LOWPASS_ALPHA_PERCENT 50U
/* Ngưỡng phát hiện bước nhảy khoảng cách ToF (mm):
 * Khi khoảng cách thay đổi lớn hơn ngưỡng này (ví dụ > 50mm do rẽ góc, gặp ngã ba, cửa thông):
 * bỏ qua low-pass và gán trực tiếp giá trị mới để triệt tiêu hoàn toàn độ trễ.
 * Chỉ lọc làm mượt khi biến thiên nhỏ hơn hoặc bằng ngưỡng (khử nhiễu rung khi chạy dọc tường).
 * Đặt 0 để tắt (luôn luôn lọc low-pass). */
#define TOF_LOWPASS_STEP_THRESHOLD_MM 50U
/* VL53L1X ULD chỉ hỗ trợ budget rời rạc (20 ms là mức gần 25 ms nhất). */
#define TOF_MID_TIMING_BUDGET_MS 20U
#define TOF_MID_INTER_MEASUREMENT_MS 25U
/* Thời gian giữ XSHUT thấp, đợi boot và giãn cách khởi tạo giữa các cảm biến. */
#define TOF_XSHUT_RESET_DELAY_MS 2U
#define TOF_SENSOR_BOOT_DELAY_MS 10U
#define TOF_SENSOR_INTER_INIT_DELAY_MS 100U

/* ── MPU6500 ─────────────────────────────────────────────────────────────────
 * MPU6500 dùng I2C2 riêng để không xung đột bus I2C1 của ToF VL53L0X/L1X.
 * STM32F401RCT6 LQFP64 - I2C2 pin mapping:
 *   PB10 = I2C2_SCL  (AF4)
 *   PB3  = I2C2_SDA  (AF9)  ← Đã tắt SWV (TRACESWO) trên DBGMCU để giải phóng PB3
 * Địa chỉ I2C 7-bit (AD0 nối GND → 0x68, nối VCC → 0x69).
 * WHO_AM_I mặc định của MPU6500 là 0x70 (112).
 * Gyro full-scale range: 250 | 500 | 1000 | 2000 (°/s).
 * Alpha low-pass IIR: 1..100 (%). Giá trị nhỏ làm mượt mạnh, lớn phản hồi nhanh.
 * ──────────────────────────────────────────────────────────────────────────── */
#define MPU6500_SCL                      GPIO_PIN_10
#define MPU6500_SCL_Port                 GPIOB
#define MPU6500_SCL_AF                   GPIO_AF4_I2C2
#define MPU6500_SDA                      GPIO_PIN_3
#define MPU6500_SDA_Port                 GPIOB
#define MPU6500_SDA_AF                   GPIO_AF9_I2C2
#define MPU6500_I2C_ADDRESS              0x68U
#define MPU6500_WHO_AM_I_VAL             0x70U
#define MPU6500_I2C_CLOCK_SPEED_HZ       800000U
#define MPU6500_GYRO_FULL_SCALE_DPS      250
#define MPU6500_GYRO_LOWPASS_ALPHA_PERCENT 100U /* 100% = tắt lọc IIR mềm, tránh trễ 6ms (DLPF phần cứng 41Hz đã lọc) */
/*
 * Hệ số hiệu chuẩn tỉ lệ đo góc con quay hồi chuyển (Gyro Scale Calibration Factor):
 * Cảm biến MEMS MPU6500 có dung sai độ nhạy xuất xưởng ±2~3%.
 * Thực tế đo đạc: khi xe quay 360° thì odometry tính ra 364° (sai số +1.11%).
 * Hệ số bù chuẩn hóa: 360.0f / 364.0f ≈ 0.98901099f.
 */
#define MPU6500_GYRO_SCALE_FACTOR        (360.0f / 364.0f)

/* Wheel and encoder values used by odometry. */
#define ENCODER_COUNT_MAX 10000U
#define ENCODER_STEPS_PER_MOTOR_REV 28U
#define ODOMETRY_GEAR_RATIO 99.0f
#define ODOMETRY_WHEEL_DIAMETER_MM 44.5f
/*
 * Khoảng cách hai bánh hiệu dụng (Effective Wheel Track):
 * Khoảng cách cơ khí đo được là 77.0mm. Khi xe quay tại chỗ, tiếp xúc lốp
 * và vi trượt bề mặt tạo ra vệt quay hiệu dụng:
 * L_eff = 77.0mm * (364.0 / 360.0) ≈ 77.8556mm.
 * Hiệu chuẩn giá trị này giúp tính toán góc từ Encoder chuẩn xác 360° khi quay đủ 360°.
 */
#define ODOMETRY_WHEEL_TRACK_MM          (77.0f * (364.0f / 360.0f))
#define ODOMETRY_LEFT_ENCODER_SIGN -1
#define ODOMETRY_RIGHT_ENCODER_SIGN 1
#define ODOMETRY_INITIAL_HEADING_DEG 90.0f
#define ODOMETRY_INITIAL_X_MM 300.0f
#define ODOMETRY_INITIAL_Y_MM -45.0f

/* ── PID VẬN TỐC BÁNH XE (WHEEL SPEED PID) THEO ĐỘ RỘNG XUNG PWM (us) ─────────
 * Vòng lặp điều khiển kín độc lập cho từng bánh xuất trực tiếp độ rộng xung PWM (us):
 *   setpoint (mm/s) → PID → độ rộng xung PWM [offset ± deadzone ± delta_us]
 * dt = ODOMETRY_SPEED_AVERAGE_WINDOW_MS / 1000.0 = 0.01 s (vận tốc cập nhật mỗi 10 ms).
 * Độ phân giải đạt 1 us (thay vì dải -100..100 bị chia thô mỗi nấc 8 us), giúp xe chạy cực mịn.
 * Anti-windup: tích phân I bị clamp ở ±WHEEL_PID_INTEGRAL_LIMIT_US.
 * ──────────────────────────────────────────────────────────────────────────── */
#define WHEEL_PID_KP_US                  2.40f   /* Hệ số P: 2.40 us trên mỗi mm/s lỗi vận tốc */
#define WHEEL_PID_KI_US                  4.80f   /* Hệ số I: 4.80 us/(mm/s * s) bù tĩnh ma sát */
#define WHEEL_PID_KD_US                  0.040f  /* Hệ số D: 0.040 us/(mm/s^2) giảm chấn vi phân */
#define WHEEL_PID_INTEGRAL_LIMIT_US      320.0f  /* Clamp tích phân I ±limit để chống windup (us) */

/* Tương thích dải cũ -100..100 */
#define WHEEL_PID_KP                     0.30f   /* Hệ số P (tỉ lệ): tăng để phản hồi nhanh hơn */
#define WHEEL_PID_KI                     0.60f   /* Hệ số I (tích phân): bù offset tĩnh ma sát */
#define WHEEL_PID_KD                     0.005f  /* Hệ số D (vi phân): giảm dao động */
#define WHEEL_PID_INTEGRAL_LIMIT         40.0f   /* Clamp tích phân I ±limit để chống windup */
#define STRAIGHT_TARGET_SPEED_MM_S       300.0f  /* Vận tốc mục tiêu khi điều khiển thủ công W/S */
#define NAV_CRUISE_SPEED_MM_S            300.0f  /* Vận tốc hành trình tới điểm đích (Nav) */
#define NAV_MIN_SPEED_MM_S               35.0f   /* Vận tốc tối thiểu hãm tốc gần đích (thắng ma sát) */

/* ── THAM SỐ PID XOAY GÓC (ROTATION PID) ───────────────────────────────────
 * Vòng lặp PD đơn điều khiển góc xoay tại chỗ:
 *   u = ROT_PID_KP * errorAngle - ROT_PID_KD * omega
 * ──────────────────────────────────────────────────────────────────────────── */
#define ROT_PID_KP                       0.5f    /* Hệ số tỉ lệ P sai số góc */
#define ROT_PID_KD                       0.02f   /* Giảm chấn vi phân vận tốc góc (D trên Gyro) */
#define ROT_PID_CRUISE_PWM               32.0f   /* Giới hạn tốc độ PWM hành trình khi quay */
#define ROT_PID_MIN_PWM                  14.0f   /* Xung PWM tối thiểu bù deadzone ma sát tĩnh */
#define ROT_PID_ANGLE_TOL_DEG            3.5f    /* Dung sai góc chấp nhận được (độ) */
#define ROT_PID_TIMEOUT_MS               5000U   /* Thời gian tối đa bảo vệ cho một lệnh xoay góc (ms) */

/* Tương thích macro cũ */
#define KP_ROT_ANG                       ROT_PID_KP
#define KD_ROT_ANG                       ROT_PID_KD
#define CRUISE_PWM                       ROT_PID_CRUISE_PWM
#define MIN_ANG_PWM                      ROT_PID_MIN_PWM
#define ANGLE_TOL_DEG                    ROT_PID_ANGLE_TOL_DEG
#define ANGLE_TIMEOUT                    ROT_PID_TIMEOUT_MS

/* ── THAM SỐ DẪN ĐƯỜNG VÀ BÁM HƯỚNG KHI CHẠY THẲNG (NAV PURSUIT & HEADING) ─
 * Khi xe quay tới góc ngắm điểm đích (dung sai 0.5 độ), xe lập tức chuyển
 * sang chạy thẳng tới đích. Trong quá trình chạy, xe liên tục tính toán góc
 * đường ngắm trực tiếp tới đích:
 *   - Nếu góc lệch nằm trong dung sai 0.5 độ (NAV_HEADING_DEADBAND_DEG): xe chạy thẳng tắp.
 *   - Nếu lệch > 0.5 độ: PID điều chỉnh tốc độ vi sai 2 bánh để uốn xe đúng hướng
 *     ngay trong lúc xe ĐANG CHẠY THẲNG TIẾN TỚI (không dừng lại).
 * ──────────────────────────────────────────────────────────────────────────── */
#define NAV_TURN_TARGET_TOL_DEG          0.5f    /* Dung sai góc kết thúc xoay: vào vùng 0.5 độ là chạy thẳng ngay */
#define NAV_HEADING_DEADBAND_DEG         0.5f    /* Vùng chết dung sai góc khi chạy thẳng (độ) */
#define NAV_HEADING_KP                   8.0f    /* Hệ số P bám hướng (mm/s trên mỗi độ lệch) */
#define NAV_HEADING_KD                   0.20f   /* Hệ số D giảm chấn tốc độ quay thân xe (mm/s trên mỗi dps) */
#define MAX_STRAIGHT_CORRECTION_MM_S     220.0f  /* Mức giảm tốc tối đa của bánh trong khi ôm cua (mm/s) */
#define NAV_CORNER_BLEND_RADIUS_MM       120.0f  /* Bán kính chuyển mốc uốn cung tròn liên tục trước khi tới điểm (mm) */
#define NAV_MAX_ARC_CORNER_ANGLE_DEG     115.0f  /* Góc lệch tối đa cho phép chuyển tiếp ôm cua động liên tục không dừng xe (độ) */

/* ── CẤU HÌNH KẾT HỢP GYROZ VÀO ODOMETRY ─────────────────────────────────────
 * ODOMETRY_USE_GYRO_FUSION:
 *   1 = BẬT áp dụng GyroZ vào Odometry (kết hợp góc quay chuẩn xác)
 *   0 = TẮT GyroZ, Odometry chạy thuần 100% Encoder bánh xe
 * ODOMETRY_GYRO_WEIGHT:
 *   Hệ số trọng số của GyroZ (0.60f = 60% GyroZ, 40% Encoder)
 * ──────────────────────────────────────────────────────────────────────────── */
#define ODOMETRY_USE_GYRO_FUSION          1U
#define ODOMETRY_GYRO_WEIGHT              0.60f

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
HAL_StatusTypeDef Config_I2C2_Init(void);
HAL_StatusTypeDef Config_OdometryTimer_Init(void);
void Config_OdometryTimer_Start(void);
void Config_I2C1_GPIO_Init(void);
void Config_I2C1_GPIO_DeInit(void);
void Config_TIM1_GPIO_Init(void);
void Config_TIM11_GPIO_Init(void);
void Config_USART2_GPIO_Init(void);
void Config_USART2_GPIO_DeInit(void);

/* Kiểm tra chéo các giá trị cấu hình sau khi toàn bộ macro đã được khai báo. */
#if CONTROL_LOOP_FREQUENCY_HZ < 1U || CONTROL_LOOP_FREQUENCY_HZ > 1000U
#error "CONTROL_LOOP_FREQUENCY_HZ must be between 1 and 1000"
#endif /* MICROMOUSE_CONFIG_H */

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

#if TOF_LOWPASS_ALPHA_PERCENT < 1U || TOF_LOWPASS_ALPHA_PERCENT > 100U
#error "TOF_LOWPASS_ALPHA_PERCENT must be between 1 and 100"
#endif

#if TOF_MID_TIMING_BUDGET_MS < 15U || TOF_MID_INTER_MEASUREMENT_MS < TOF_MID_TIMING_BUDGET_MS
#error "VL53L1X needs a supported timing budget and an inter-measurement period at least as long"
#endif

#if MPU6500_GYRO_LOWPASS_ALPHA_PERCENT < 1U || MPU6500_GYRO_LOWPASS_ALPHA_PERCENT > 100U
#error "MPU6500_GYRO_LOWPASS_ALPHA_PERCENT must be between 1 and 100"
#endif

#if MPU6500_I2C_ADDRESS < 0x08U || MPU6500_I2C_ADDRESS > 0x77U
#error "MPU6500_I2C_ADDRESS must be a valid 7-bit I2C address (0x08..0x77)"
#endif

#endif /* MICROMOUSE_CONFIG_H */
