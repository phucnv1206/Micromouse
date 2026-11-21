#include "pid_controller.h"
#include "motor_pwm.h"
#include "kalman_filter.h"
#include "odometry.h"
#include "config.h"
#include <math.h>

/* Các tham số PID xoay góc được định nghĩa tập trung trong Inc/config.h:
 * ROT_PID_KP, ROT_PID_KD, ROT_PID_CRUISE_PWM, ROT_PID_MIN_PWM, ROT_PID_ANGLE_TOL_DEG, ROT_PID_TIMEOUT_MS
 */
#define MANUAL_WATCHDOG_TIMEOUT_MS 250U /* Lớp bảo vệ: Quá 250ms không có nhịp UART -> Dừng khẩn cấp */

/* Chuẩn hóa hiệu góc giữa đích và hiện tại về khoảng [-180, +180] độ */
static float NormalizeAngleDiff(float target_deg, float current_deg) {
  float diff = target_deg - current_deg;
  while (diff > 180.0f) diff -= 360.0f;
  while (diff < -180.0f) diff += 360.0f;
  return diff;
}

/* ================= ĐỊNH NGHĨA THAM SỐ HEADING-HOLD (ĐI THẲNG) ================= */
#define KP_STRAIGHT              NAV_HEADING_KP   /* Hệ số P bám hướng (mm/s trên mỗi độ lệch) */
#define KD_STRAIGHT              NAV_HEADING_KD   /* Giảm chấn vi phân vận tốc góc (mm/s trên mỗi dps) */

/* ================= PID VẬN TỐC BÁNH XE ================= */
typedef struct {
  float integral;    /* Tích phân lỗi */
  float prev_error;  /* Lỗi kỳ trước (cho vi phân) */
} WheelSpeedPID;

static WheelSpeedPID s_pid_right;
static WheelSpeedPID s_pid_left;

/* Reset trạng thái PID vận tốc */
static void WheelSpeedPID_Reset(WheelSpeedPID *pid) {
  pid->integral   = 0.0f;
  pid->prev_error = 0.0f;
}

/*
 * Tính đầu ra PID vận tốc bánh (trả về PWM [-100..100]).
 * setpoint_mm_s: vận tốc mục tiêu (mm/s, dương = tiến).
 * actual_mm_s  : vận tốc thực đo từ odometry (mm/s).
 * dt_s         : thời gian bước (s) = ODOMETRY_SPEED_AVERAGE_WINDOW_MS / 1000.
 */
static float WheelSpeedPID_Update(WheelSpeedPID *pid,
                                   float setpoint_mm_s,
                                   float actual_mm_s,
                                   float dt_s) {
  float error = setpoint_mm_s - actual_mm_s;

  /* Tích phân có anti-windup: clamp trước khi cộng */
  pid->integral += error * dt_s;
  if (pid->integral >  WHEEL_PID_INTEGRAL_LIMIT) pid->integral =  WHEEL_PID_INTEGRAL_LIMIT;
  if (pid->integral < -WHEEL_PID_INTEGRAL_LIMIT) pid->integral = -WHEEL_PID_INTEGRAL_LIMIT;

  float derivative = (error - pid->prev_error) / dt_s;
  pid->prev_error  = error;

  float output = WHEEL_PID_KP * error
               + WHEEL_PID_KI * pid->integral
               + WHEEL_PID_KD * derivative;

  if (output >  100.0f) output =  100.0f;
  if (output < -100.0f) output = -100.0f;

  return output;
}

/* ================= ĐỊNH NGHĨA THAM SỐ ĐIỀU KHIỂN ĐIỂM (NAV TO POINT) ================= */
typedef enum {
  NAV_STAGE_TURN = 0,           /* Giai đoạn 1: Xoay đầu hướng về điểm đích */
  NAV_STAGE_DRIVE_STRAIGHT      /* Giai đoạn 2: Chạy thẳng tới điểm đích */
} NavStage;

#define NAV_ARRIVE_TOL_MM        18.0f   /* Bán kính dung sai tới đích (mm) */
#define NAV_TIMEOUT_MS           25000U  /* Thời gian tối đa bảo vệ cho một lệnh chạy tới điểm (25s) */

static MotionMode currentMode = MOTION_MODE_STOP;
static uint8_t activeCommand = CMD_STOP;
static uint8_t commandStatus = STATUS_IDLE;
static uint32_t statusTimestamp = 0U;
static uint32_t lastManualCommandTick = 0U;

/* Biến phục vụ PID xoay góc */
static float targetHeadingDeg = 0.0f;
static uint32_t turnStartTick = 0U;
static uint8_t turnSettleCount = 0U;

/* Biến phục vụ điều khiển đi thẳng */
static bool straightLockActive = false;
static float straightTargetHeadingDeg = 0.0f;
static float straightTargetSpeedMmS = 0.0f;  /* Setpoint vận tốc mm/s (thay thế straightBaseSpeed PWM) */

/* Biến phục vụ Nav To Point */
static NavStage navStage = NAV_STAGE_TURN;
static float navStartX = 0.0f;
static float navStartY = 0.0f;
static float navTargetX = 0.0f;
static float navTargetY = 0.0f;
static float navTargetHeadingDeg = 0.0f;
static uint32_t navStartTick = 0U;
static float navPrevTurnError = 0.0f;

void Motion_Init(void) {
  currentMode = MOTION_MODE_STOP;
  activeCommand = CMD_STOP;
  commandStatus = STATUS_IDLE;
  statusTimestamp = HAL_GetTick();
  lastManualCommandTick = HAL_GetTick();
  straightLockActive = false;
  straightTargetSpeedMmS = 0.0f;
  WheelSpeedPID_Reset(&s_pid_right);
  WheelSpeedPID_Reset(&s_pid_left);
  navStage = NAV_STAGE_TURN;
  navPrevTurnError = 0.0f;

  RunMotorRight(0);
  RunMotorLeft(0);
}

void Motion_Stop(void) {
  currentMode = MOTION_MODE_STOP;
  activeCommand = CMD_EMERGENCY_STOP;
  commandStatus = STATUS_IDLE;
  statusTimestamp = HAL_GetTick();
  straightLockActive = false;
  straightTargetSpeedMmS = 0.0f;
  WheelSpeedPID_Reset(&s_pid_right);
  WheelSpeedPID_Reset(&s_pid_left);
  navPrevTurnError = 0.0f;

  RunMotorRight(0);
  RunMotorLeft(0);
}

void Motion_SetCommandStatus(uint8_t cmd, uint8_t status) {
  activeCommand = cmd;
  commandStatus = status;
  statusTimestamp = HAL_GetTick();
}

void Motion_ProcessCommand(uint8_t cmd, int16_t data1, int16_t data2) {
  /*
   * 1. Nếu đang trong chế độ tự động (XOAY GÓC hoặc CHẠY TỚI ĐIỂM ĐÍCH):
   *    - Các gói nhịp định kỳ rảnh (data1 == 0 && data2 == 0) KHÔNG ĐƯỢC ngắt lệnh tự động!
   *    - Lặp lại chính lệnh tự động hiện tại cũng không reset lại trạng thái!
   *    - Chỉ dừng hoặc hủy nếu có lệnh dừng khẩn cấp (CMD_EMERGENCY_STOP) hoặc
   *      người dùng nhấn phím điều khiển thủ công (data1 != 0 || data2 != 0).
   */
  if (currentMode == MOTION_MODE_ROTATE_ANGLE || currentMode == MOTION_MODE_NAV_TO_POINT) {
    if (cmd == activeCommand) {
      if (cmd == CMD_NAV_TO_POINT) {
        /* Nếu nhận được tọa độ mục tiêu MỚI khác tọa độ đang chạy, PHẢI chấp nhận chuyển hướng ngay! */
        if (data1 == (int16_t)navTargetX && data2 == (int16_t)navTargetY) {
          return; /* Đúng là gói trùng tọa độ cũ, bỏ qua */
        }
        /* Tọa độ mới -> Cho phép lọt xuống dưới để cập nhật navTargetX/Y ngay lập tức! */
      } else {
        return; /* Đang thực thi dở lệnh xoay góc, bỏ qua gói trùng */
      }
    }
    if (cmd == CMD_EMERGENCY_STOP) {
      Motion_Stop();
      return;
    }
    if (cmd == CMD_MANUAL_DRIVE || cmd == CMD_STOP) {
      if (data1 == 0 && data2 == 0) {
        return; /* Gói idle rảnh từ ESP32: Tiếp tục tự động đến đích, không ngắt lệnh! */
      } else {
        /* Người dùng chủ động bấm phím WASD can thiệp: Hủy lệnh tự động chuyển sang thủ công */
        Motion_Stop();
        currentMode = MOTION_MODE_MANUAL;
        activeCommand = CMD_MANUAL_DRIVE;
        commandStatus = STATUS_IDLE;
      }
    }
  }

  /* 2. Xử lý các loại lệnh */
  switch (cmd) {
    case CMD_EMERGENCY_STOP:
      Motion_Stop();
      break;

    case CMD_MANUAL_DRIVE: {
      /*
       * Điều khiển 2 bánh xe từ bàn phím WASD:
       * data1 = Bánh Phải (-100 đến 100)
       * data2 = Bánh Trái (-100 đến 100)
       */
      int16_t r = data1;
      int16_t l = data2;

      if (r > 100) r = 100;
      if (r < -100) r = -100;
      if (l > 100) l = 100;
      if (l < -100) l = -100;

      if (r == 0 && l == 0) {
        straightLockActive = false;
        straightTargetSpeedMmS = 0.0f;
        WheelSpeedPID_Reset(&s_pid_right);
        WheelSpeedPID_Reset(&s_pid_left);
        RunMotorRight(0);
        RunMotorLeft(0);
        currentMode = MOTION_MODE_STOP;
        activeCommand = CMD_STOP;
      } else {
        currentMode = MOTION_MODE_MANUAL;
        activeCommand = CMD_MANUAL_DRIVE;
        lastManualCommandTick = HAL_GetTick(); /* Cập nhật nhịp nhận lệnh thủ công */

        /* Nếu là lệnh đi thẳng (W tiến: r == l > 0, hoặc S lùi: r == l < 0) */
        if (r == l && r != 0) {
          if (!straightLockActive) {
            straightTargetHeadingDeg = KalmanOdometry_GetHeading();
            straightLockActive = true;
            WheelSpeedPID_Reset(&s_pid_right);
            WheelSpeedPID_Reset(&s_pid_left);
          }
          /* Setpoint vận tốc mm/s: r > 0 → tiến, r < 0 → lùi.
           * Motion_Update() sẽ gọi WheelSpeedPID với setpoint này kèm bù heading-hold. */
          if (r > 0) {
            straightTargetSpeedMmS = STRAIGHT_TARGET_SPEED_MM_S;
          } else {
            straightTargetSpeedMmS = -STRAIGHT_TARGET_SPEED_MM_S;
          }
        } else {
          /* Lệnh quay tay A/D (r != l): xuất PWM vi sai trực tiếp không khóa góc */
          straightLockActive = false;
          straightTargetSpeedMmS = 0.0f;
          WheelSpeedPID_Reset(&s_pid_right);
          WheelSpeedPID_Reset(&s_pid_left);
          RunMotorRight(r);
          RunMotorLeft(l);
        }
      }
      break;
    }

    case CMD_ROTATE_ANGLE: {
      /*
       * Lệnh xoay góc tại chỗ tự động PID (Turn by Angle):
       * data1 = Góc muốn xoay tương đối (độ, ví dụ 90, -90, 180...)
       */
      if (data1 == 0) {
        Motion_Stop();
        return;
      }

      float currentHeading = KalmanOdometry_GetHeading();
      targetHeadingDeg = currentHeading + (float)data1;
      while (targetHeadingDeg >= 360.0f) targetHeadingDeg -= 360.0f;
      while (targetHeadingDeg < 0.0f)   targetHeadingDeg += 360.0f;

      turnStartTick = HAL_GetTick();
      turnSettleCount = 0U;
      currentMode = MOTION_MODE_ROTATE_ANGLE;
      activeCommand = CMD_ROTATE_ANGLE;
      commandStatus = STATUS_RUNNING;
      statusTimestamp = HAL_GetTick();
      break;
    }

    case CMD_NAV_TO_POINT: {
      /*
       * Lệnh chạy tới điểm tọa độ mục tiêu (Go to Waypoint X, Y in mm):
       * data1 = X_target (mm)
       * data2 = Y_target (mm)
       */
      Odometry_State odom;
      if (Odometry_GetState(&odom) != HAL_OK) {
        odom.x_mm = 0.0f;
        odom.y_mm = 0.0f;
      }
      float startX = odom.x_mm;
      float startY = odom.y_mm;
      float targetX = (float)data1;
      float targetY = (float)data2;

      float dx = targetX - startX;
      float dy = targetY - startY;
      float dist_mm = sqrtf(dx * dx + dy * dy);

      if (dist_mm < NAV_ARRIVE_TOL_MM) {
        RunMotorRight(0);
        RunMotorLeft(0);
        WheelSpeedPID_Reset(&s_pid_right);
        WheelSpeedPID_Reset(&s_pid_left);
        currentMode = MOTION_MODE_STOP;
        activeCommand = CMD_NAV_TO_POINT;
        commandStatus = STATUS_COMPLETED;
        statusTimestamp = HAL_GetTick();
        return;
      }

      /* Nội suy góc đường đi từ vị trí hiện tại tới đích (đơn vị độ [0, 360)) */
      float pathAngleRad = atan2f(dy, dx);
      float pathAngleDeg = pathAngleRad * (180.0f / 3.141592653589793f);
      while (pathAngleDeg >= 360.0f) pathAngleDeg -= 360.0f;
      while (pathAngleDeg < 0.0f)   pathAngleDeg += 360.0f;

      float currentHeading = KalmanOdometry_GetHeading();
      float initialAngleErr = NormalizeAngleDiff(pathAngleDeg, currentHeading);

      navTargetX = targetX;
      navTargetY = targetY;
      navStartX = startX;
      navStartY = startY;
      navTargetHeadingDeg = pathAngleDeg;
      WheelSpeedPID_Reset(&s_pid_right);
      WheelSpeedPID_Reset(&s_pid_left);
      navStartTick = HAL_GetTick();

      /* Nếu xe đã quay đúng hướng tới đích (dung sai <= 0.5 độ), cho chạy thẳng luôn không cần xoay */
      if (fabsf(initialAngleErr) <= NAV_TURN_TARGET_TOL_DEG) {
        navStage = NAV_STAGE_DRIVE_STRAIGHT;
      } else {
        navStage = NAV_STAGE_TURN; /* Giai đoạn 1: Quay đầu hướng về tọa độ đích */
        navPrevTurnError = initialAngleErr;
      }

      currentMode = MOTION_MODE_NAV_TO_POINT;
      activeCommand = CMD_NAV_TO_POINT;
      commandStatus = STATUS_RUNNING;
      statusTimestamp = HAL_GetTick();
      break;
    }

    case CMD_RESET_ODOM: {
      Odometry_Reset();
      Motion_Stop();
      activeCommand = CMD_RESET_ODOM;
      commandStatus = STATUS_COMPLETED;
      statusTimestamp = HAL_GetTick();
      break;
    }

    default:
      break;
  }
}

void Motion_Update(float dt_s) {
  if (dt_s <= 0.0f) return;

  if (currentMode == MOTION_MODE_STOP) {
    /* Sau khi hoàn thành lệnh 2 giây thì tự động chuyển badge về IDLE */
    if (commandStatus == STATUS_COMPLETED && (HAL_GetTick() - statusTimestamp > 2000U)) {
      commandStatus = STATUS_IDLE;
    }
    return;
  }

  if (currentMode == MOTION_MODE_MANUAL) {
    /*
     * LỚP BẢO VỆ PHẦN CỨNG ĐỘC LẬP TRÊN STM32:
     * Nếu xe đang chạy thủ công mà quá 250ms không có gói UART nhịp mới từ ESP32:
     * STM32 TỰ ĐỘNG DỪNG CẢ 2 ĐỘNG CƠ NGAY LẬP TỨC để bảo vệ xe không bị đâm tường!
     */
    if (HAL_GetTick() - lastManualCommandTick > MANUAL_WATCHDOG_TIMEOUT_MS) {
      RunMotorRight(0);
      RunMotorLeft(0);
      currentMode = MOTION_MODE_STOP;
      activeCommand = CMD_STOP;
      commandStatus = STATUS_ERROR;
      statusTimestamp = HAL_GetTick();
      straightLockActive = false;
      return;
    }

    /* PID điều khiển vận tốc 2 bánh xe khi bấm W (tiến) hoặc S (lùi) */
    if (straightLockActive && straightTargetSpeedMmS != 0.0f) {
      float currentHeading = KalmanOdometry_GetHeading();
      float currentOmega   = KalmanOdometry_GetAngularVelocity();
      float errorHeading   = NormalizeAngleDiff(straightTargetHeadingDeg, currentHeading);

      /* Heading-hold: tính hiệu chỉnh tốc độ vi sai (mm/s) giữa 2 bánh */
      float p_term = KP_STRAIGHT * errorHeading;
      float d_term = KD_STRAIGHT * currentOmega;
      float correctionMmS = p_term - d_term;

      if (correctionMmS >  MAX_STRAIGHT_CORRECTION_MM_S) correctionMmS =  MAX_STRAIGHT_CORRECTION_MM_S;
      if (correctionMmS < -MAX_STRAIGHT_CORRECTION_MM_S) correctionMmS = -MAX_STRAIGHT_CORRECTION_MM_S;

      /* Setpoint từng bánh = tốc độ nền ± bù heading */
      float setR = straightTargetSpeedMmS + correctionMmS;
      float setL = straightTargetSpeedMmS - correctionMmS;

      /* Đọc vận tốc thực từ Odometry */
      Odometry_State odom;
      float actualR = 0.0f, actualL = 0.0f;
      if (Odometry_GetState(&odom) == HAL_OK) {
        actualR = odom.wheel_linear_mm_s[ENCODER_RIGHT];
        actualL = odom.wheel_linear_mm_s[ENCODER_LEFT];
      }

      /* Chạy PID từng bánh, dt = 10ms (cửa sổ trung bình vận tốc) */
      const float dt = (float)ODOMETRY_SPEED_AVERAGE_WINDOW_MS / 1000.0f;
      float pwmR = WheelSpeedPID_Update(&s_pid_right, setR, actualR, dt);
      float pwmL = WheelSpeedPID_Update(&s_pid_left,  setL, actualL, dt);

      RunMotorRight((int16_t)roundf(pwmR));
      RunMotorLeft((int16_t)roundf(pwmL));
    }
    return;
  }

  if (currentMode == MOTION_MODE_ROTATE_ANGLE) {
    /* Vòng lặp điều khiển PID xoay góc tại chỗ */
    float currentHeading = KalmanOdometry_GetHeading();
    float currentOmega = KalmanOdometry_GetAngularVelocity();
    float errorAngle = NormalizeAngleDiff(targetHeadingDeg, currentHeading);
    float absErr = fabsf(errorAngle);

    /* 1. Kiểm tra điều kiện hoàn thành: sai số góc <= 2.0 độ và vận tốc góc <= 4.0 độ/s */
    if (absErr <= ANGLE_TOL_DEG && fabsf(currentOmega) <= 4.0f) {
      turnSettleCount++;
      if (turnSettleCount >= 6U) { // Duy trì ổn định 6 chu kỳ (60ms) trong vùng dung sai 2 độ
        RunMotorRight(0);
        RunMotorLeft(0);
        currentMode = MOTION_MODE_STOP;
        activeCommand = CMD_ROTATE_ANGLE;
        commandStatus = STATUS_COMPLETED;
        statusTimestamp = HAL_GetTick();
        return;
      }
    } else {
      turnSettleCount = 0U;
    }

    /* 2. Kiểm tra timeout bảo vệ chống kẹt xe */
    if (HAL_GetTick() - turnStartTick > ANGLE_TIMEOUT) {
      RunMotorRight(0);
      RunMotorLeft(0);
      currentMode = MOTION_MODE_STOP;
      activeCommand = CMD_ROTATE_ANGLE;
      commandStatus = STATUS_ERROR;
      statusTimestamp = HAL_GetTick();
      return;
    }

    /* 3. Điều khiển PID góc xoay mượt mà liên tục (không bị giật hay khựng bước nhảy): */
    float p_term = KP_ROT_ANG * errorAngle;
    float d_term = KD_ROT_ANG * currentOmega;
    float u = p_term - d_term;

    /* Giới hạn tốc độ hành trình tối đa an toàn */
    if (u > CRUISE_PWM) u = CRUISE_PWM;
    if (u < -CRUISE_PWM) u = -CRUISE_PWM;

    /* Bù deadzone ma sát tĩnh của bánh xe: chỉ áp dụng khi xe bị kẹt ma sát (vận tốc góc nhỏ)
     * và giảm dần xung khi vào gần góc đích để triệt tiêu hoàn toàn rung lắc */
    if (absErr > ANGLE_TOL_DEG && fabsf(currentOmega) < 5.0f) {
      float minPwm = (absErr < 8.0f) ? (MIN_ANG_PWM * (absErr / 8.0f)) : MIN_ANG_PWM;
      if (u > 0.0f && u < minPwm) u = minPwm;
      if (u < 0.0f && u > -minPwm) u = -minPwm;
    }

    /* u > 0 (xoay trái CCW): Bánh phải tiến, Bánh trái lùi */
    int16_t rPwm = (int16_t)roundf(u);
    int16_t lPwm = (int16_t)roundf(-u);

    RunMotorRight(rPwm);
    RunMotorLeft(lPwm);
    return;
  }

  if (currentMode == MOTION_MODE_NAV_TO_POINT) {
    /* 1. Kiểm tra timeout an toàn (25s) */
    if (HAL_GetTick() - navStartTick > NAV_TIMEOUT_MS) {
      RunMotorRight(0);
      RunMotorLeft(0);
      WheelSpeedPID_Reset(&s_pid_right);
      WheelSpeedPID_Reset(&s_pid_left);
      currentMode = MOTION_MODE_STOP;
      activeCommand = CMD_NAV_TO_POINT;
      commandStatus = STATUS_ERROR;
      statusTimestamp = HAL_GetTick();
      return;
    }

    /* Giai đoạn 1: Quay đầu hướng về tọa độ điểm đích */
    if (navStage == NAV_STAGE_TURN) {
      float currentHeading = KalmanOdometry_GetHeading();
      float currentOmega = KalmanOdometry_GetAngularVelocity();
      float errorAngle = NormalizeAngleDiff(navTargetHeadingDeg, currentHeading);
      float absErr = fabsf(errorAngle);

      /* [THIẾT KẾ MỚI]: Ngay khi xe quay đến góc mong muốn (dung sai 0.5 độ hoặc vừa cắt qua đích)
       * thì cho xe chuyển ngay sang chạy thẳng tới điểm đích, KHÔNG dừng xe, KHÔNG chờ ổn định! */
      bool crossedZero = (navPrevTurnError * errorAngle < 0.0f) && (absErr <= 5.0f);
      if (absErr <= NAV_TURN_TARGET_TOL_DEG || crossedZero) {
        WheelSpeedPID_Reset(&s_pid_right);
        WheelSpeedPID_Reset(&s_pid_left);
        navStage = NAV_STAGE_DRIVE_STRAIGHT;
        return;
      }
      navPrevTurnError = errorAngle;

      /* Bảo vệ an toàn timeout: Nếu sau 2.5s mà góc đã khá gần (<= 15 độ),
       * vẫn cho xe chuyển sang chạy thẳng và để thuật toán Dynamic Pursuit tự nắn lái khi chạy */
      uint32_t turnElapsed = HAL_GetTick() - navStartTick;
      if (turnElapsed > 2500U) {
        if (absErr <= 15.0f) {
          WheelSpeedPID_Reset(&s_pid_right);
          WheelSpeedPID_Reset(&s_pid_left);
          navStage = NAV_STAGE_DRIVE_STRAIGHT;
          return;
        } else if (turnElapsed > 5000U) {
          /* Bị kẹt hoàn toàn do vật cản: dừng an toàn */
          RunMotorRight(0);
          RunMotorLeft(0);
          currentMode = MOTION_MODE_STOP;
          activeCommand = CMD_NAV_TO_POINT;
          commandStatus = STATUS_ERROR;
          statusTimestamp = HAL_GetTick();
          return;
        }
      }

      /* Tính toán điều khiển xoay mượt mà: */
      float p_term = KP_ROT_ANG * errorAngle;
      float d_term = KD_ROT_ANG * currentOmega;
      float u = p_term - d_term;

      if (u > CRUISE_PWM) u = CRUISE_PWM;
      if (u < -CRUISE_PWM) u = -CRUISE_PWM;

      /* Chỉ bù deadzone ma sát tĩnh khi xe bị đứng yên (omega nhỏ) để tránh vọt lố/rung lắc */
      if (absErr > 2.0f && fabsf(currentOmega) < 4.0f) {
        float minPwm = (absErr < 8.0f) ? (MIN_ANG_PWM * (absErr / 8.0f)) : MIN_ANG_PWM;
        if (u > 0.0f && u < minPwm) u = minPwm;
        if (u < 0.0f && u > -minPwm) u = -minPwm;
      }

      RunMotorRight((int16_t)roundf(u));
      RunMotorLeft((int16_t)roundf(-u));
      return;
    }

    /* Giai đoạn 2: Chạy thẳng đến điểm đích với bộ bám hướng động (Dynamic Pursuit) */
    if (navStage == NAV_STAGE_DRIVE_STRAIGHT) {
      Odometry_State odom;
      if (Odometry_GetState(&odom) != HAL_OK) {
        odom.x_mm = navStartX;
        odom.y_mm = navStartY;
      }

      float dx = navTargetX - odom.x_mm;
      float dy = navTargetY - odom.y_mm;
      float remainingDist = sqrtf(dx * dx + dy * dy);

      /* Kiểm tra vector chiếu để phát hiện xe đã tới hoặc vượt qua đích */
      float totalDx = navTargetX - navStartX;
      float totalDy = navTargetY - navStartY;
      float proj = totalDx * dx + totalDy * dy;

      if (remainingDist <= NAV_ARRIVE_TOL_MM || (remainingDist <= 60.0f && proj <= 0.0f)) {
        RunMotorRight(0);
        RunMotorLeft(0);
        WheelSpeedPID_Reset(&s_pid_right);
        WheelSpeedPID_Reset(&s_pid_left);
        currentMode = MOTION_MODE_STOP;
        activeCommand = CMD_NAV_TO_POINT;
        commandStatus = STATUS_COMPLETED;
        statusTimestamp = HAL_GetTick();
        return;
      }

      /* Ham toc muot khi gan toi dich (khoang cach < 100mm) */
      float targetSpeedMmS = NAV_CRUISE_SPEED_MM_S;
      if (remainingDist < 100.0f) {
        targetSpeedMmS = NAV_MIN_SPEED_MM_S + (NAV_CRUISE_SPEED_MM_S - NAV_MIN_SPEED_MM_S) * (remainingDist / 100.0f);
        if (targetSpeedMmS < NAV_MIN_SPEED_MM_S) targetSpeedMmS = NAV_MIN_SPEED_MM_S;
      }

      /* Dynamic target heading (Pursuit): luôn ngắm thẳng vào tọa độ đích thực tế;
       * khi khoảng cách < 35mm (sắp tới đích), giữ nguyên navTargetHeadingDeg
       * để tránh góc atan2 bị nhảy giật do sai số mm ở cực cận. */
      float directAngleDeg;
      if (remainingDist < 35.0f) {
        directAngleDeg = navTargetHeadingDeg;
      } else {
        float directAngleRad = atan2f(dy, dx);
        directAngleDeg = directAngleRad * (180.0f / 3.141592653589793f);
        while (directAngleDeg >= 360.0f) directAngleDeg -= 360.0f;
        while (directAngleDeg <  0.0f)   directAngleDeg += 360.0f;
        navTargetHeadingDeg = directAngleDeg;
      }

      float currentHeading = KalmanOdometry_GetHeading();
      float currentOmega   = KalmanOdometry_GetAngularVelocity();
      float errorHeading   = NormalizeAngleDiff(directAngleDeg, currentHeading);
      float absHeadingErr  = fabsf(errorHeading);

      /* [THIẾT KẾ MỚI]: Bám hướng động khi đang chạy thẳng:
       * - Dung sai 0.5 độ (NAV_HEADING_DEADBAND_DEG): trong phạm vi 0.5 độ xe chạy thẳng tắp (correction = 0).
       * - Nếu sai lệch > 0.5 độ: PID góc xoay điều chỉnh vi sai tốc độ 2 bánh để uốn xe đúng hướng,
       *   xe VẪN DUY TRÌ TIẾN THẲNG LIÊN TỤC, TUYỆT ĐỐI KHÔNG DỪNG LẠI. */
      float correctionMmS = 0.0f;
      if (absHeadingErr > NAV_HEADING_DEADBAND_DEG) {
        float effectiveError = (errorHeading > 0.0f) ? (errorHeading - NAV_HEADING_DEADBAND_DEG)
                                                     : (errorHeading + NAV_HEADING_DEADBAND_DEG);

        float p_term = NAV_HEADING_KP * effectiveError;
        float d_term = NAV_HEADING_KD * currentOmega;
        correctionMmS = p_term - d_term;

        /* Giới hạn bù vi sai tối đa để không bao giờ làm bánh xe bị quay ngược chiều,
         * xe luôn duy trì tiến thẳng về phía trước */
        float maxCorr = targetSpeedMmS * 0.85f;
        if (maxCorr > MAX_STRAIGHT_CORRECTION_MM_S) {
          maxCorr = MAX_STRAIGHT_CORRECTION_MM_S;
        }
        if (correctionMmS >  maxCorr) correctionMmS =  maxCorr;
        if (correctionMmS < -maxCorr) correctionMmS = -maxCorr;
      }

      /* Setpoint từng bánh = tốc độ hành trình ± bù vi sai bám hướng */
      float setR = targetSpeedMmS + correctionMmS;
      float setL = targetSpeedMmS - correctionMmS;

      /* Doc van toc thuc tu Odometry */
      float actualR = odom.wheel_linear_mm_s[ENCODER_RIGHT];
      float actualL = odom.wheel_linear_mm_s[ENCODER_LEFT];

      const float dt = (float)ODOMETRY_SPEED_AVERAGE_WINDOW_MS / 1000.0f;
      float pwmR = WheelSpeedPID_Update(&s_pid_right, setR, actualR, dt);
      float pwmL = WheelSpeedPID_Update(&s_pid_left,  setL, actualL, dt);

      RunMotorRight((int16_t)roundf(pwmR));
      RunMotorLeft((int16_t)roundf(pwmL));
      return;
    }
  }
}

int16_t Motion_GetFeedbackStatusWord(void) {
  /*
   * Telemetry data15 (16-bit):
   * Byte cao: mã lệnh (activeCommand)
   * Byte thấp: trạng thái thực thi (commandStatus)
   */
  return (int16_t)(((uint16_t)activeCommand << 8U) | (uint16_t)commandStatus);
}
