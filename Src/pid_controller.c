#include "pid_controller.h"
#include "motor_pwm.h"
#include "gyro_zupt.h"
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
  float prev_actual; /* Vận tốc thực tế kỳ trước (cho vi phân trên biến đo - Derivative on Measurement) */
  bool  initialized; /* Đánh dấu đã khởi tạo prev_actual để tránh xung giật D sau reset */
} WheelSpeedPID;

static WheelSpeedPID s_pid_right;
static WheelSpeedPID s_pid_left;

/* Reset trạng thái PID vận tốc */
static void WheelSpeedPID_Reset(WheelSpeedPID *pid) {
  pid->integral    = 0.0f;
  pid->prev_actual = 0.0f;
  pid->initialized = false;
}

/*
 * Tính độ rộng xung PWM (us) trực tiếp từ PID vận tốc bánh xe.
 * setpoint_mm_s: vận tốc mục tiêu (mm/s, dương = tiến).
 * actual_mm_s  : vận tốc thực đo từ odometry (mm/s).
 * dt_s         : thời gian bước (s) = ODOMETRY_SPEED_AVERAGE_WINDOW_MS / 1000.
 * offset_us, deadzone_us, maxzone_us: thông số PWM hiệu chuẩn riêng của từng bánh.
 * Trả về: Độ rộng xung PWM thực tế (us) với độ phân giải siêu mịn 1 us.
 */
static float WheelSpeedPID_Update(WheelSpeedPID *pid,
                                   float setpoint_mm_s,
                                   float actual_mm_s,
                                   float dt_s,
                                   uint16_t offset_us,
                                   uint16_t deadzone_us,
                                   uint16_t maxzone_us) {
  /* Nếu setpoint = 0 và xe đã dừng: trả về offset xung trung tính */
  if (setpoint_mm_s == 0.0f && fabsf(actual_mm_s) < 5.0f) {
    pid->integral = 0.0f;
    pid->prev_actual = actual_mm_s;
    pid->initialized = true;
    return (float)offset_us;
  }

  if (!pid->initialized) {
    pid->prev_actual = actual_mm_s;
    pid->initialized = true;
  }

  float error = setpoint_mm_s - actual_mm_s;

  /* Tích phân có anti-windup: clamp trước khi cộng (theo us) */
  pid->integral += error * dt_s;
  if (pid->integral >  WHEEL_PID_INTEGRAL_LIMIT_US) pid->integral =  WHEEL_PID_INTEGRAL_LIMIT_US;
  if (pid->integral < -WHEEL_PID_INTEGRAL_LIMIT_US) pid->integral = -WHEEL_PID_INTEGRAL_LIMIT_US;

  /* Vi phân trên biến đo (Derivative on Measurement) triệt tiêu hoàn toàn xung giật vi phân */
  float derivative = -(actual_mm_s - pid->prev_actual) / dt_s;
  pid->prev_actual = actual_mm_s;

  float u_delta = WHEEL_PID_KP_US * error
                + WHEEL_PID_KI_US * pid->integral
                + WHEEL_PID_KD_US * derivative;

  /* Bão hòa độ lệch xung trong giới hạn maxzone_us của động cơ */
  if (u_delta >  (float)maxzone_us) u_delta =  (float)maxzone_us;
  if (u_delta < -(float)maxzone_us) u_delta = -(float)maxzone_us;

  /*
   * Ánh xạ sang độ rộng xung PWM (us) có bù deadzone ma sát tĩnh:
   * - Trong vùng [-deadzone, 0] khi đang chạy tiến: giữ xung trung tính offset_us (tránh nhảy bậc
   *   sang phanh ngược chỉ vì sai số nhỏ).
   * - Khi u_delta < -deadzone (cần hãm mạnh để ôm cua ICR hoặc giảm tốc gấp): cho phép phanh chủ động (< offset - dz).
   */
  float pulse_us;
  float dz = (float)deadzone_us;
  if (setpoint_mm_s >= 0.0f) {
    if (u_delta > 0.0f) {
      pulse_us = (float)offset_us + dz + u_delta;
    } else if (actual_mm_s > 15.0f) {
      pulse_us = (u_delta >= -dz) ? (float)offset_us : ((float)offset_us + u_delta);
    } else if (u_delta < 0.0f) {
      pulse_us = (float)offset_us - dz + u_delta;
    } else {
      pulse_us = (float)offset_us;
    }
  } else {
    if (u_delta < 0.0f) {
      pulse_us = (float)offset_us - dz + u_delta;
    } else if (actual_mm_s < -15.0f) {
      pulse_us = (u_delta <= dz) ? (float)offset_us : ((float)offset_us + u_delta);
    } else if (u_delta > 0.0f) {
      pulse_us = (float)offset_us + dz + u_delta;
    } else {
      pulse_us = (float)offset_us;
    }
  }

  if (pulse_us > (float)MOTOR_PWM_PULSE_MAX_US) pulse_us = (float)MOTOR_PWM_PULSE_MAX_US;
  if (pulse_us < (float)MOTOR_PWM_PULSE_MIN_US) pulse_us = (float)MOTOR_PWM_PULSE_MIN_US;

  return pulse_us;
}

/* ================= ĐỊNH NGHĨA THAM SỐ ĐIỀU KHIỂN ĐIỂM (NAV TO POINT) ================= */
typedef enum {
  NAV_STAGE_TURN = 0,           /* Giai đoạn 1: Xoay đầu hướng về điểm đích (quay tại chỗ R = 0) */
  NAV_STAGE_DRIVE_STRAIGHT,     /* Giai đoạn 2: Chạy thẳng tới điểm đích với PID khóa hướng */
  NAV_STAGE_ARC_ICR             /* Giai đoạn 3: Ôm cung tròn vi sai ICR (bánh ngoài Vmax, bánh trong tính theo R) */
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
static bool navIsFinalGoal = true;
static float s_smoothSetL = 0.0f;
static float s_smoothSetR = 0.0f;

/* Biến phục vụ ôm cung tròn vi sai ICR */
static float s_arcRadiusMm = NAV_ARC_RADIUS_NOMINAL_MM;
static int8_t s_arcDirection = 0; /* +1 = rẽ trái, -1 = rẽ phải */
static float s_arcOuterSpeedMmS = NAV_ARC_OUTER_SPEED_MM_S;
static float s_arcTargetHeadingDeg = 0.0f;
static float s_arcToutX = 0.0f;
static float s_arcToutY = 0.0f;
static float s_arcU2X = 0.0f;
static float s_arcU2Y = 0.0f;
static float s_arcNextTargetX = 0.0f;
static float s_arcNextTargetY = 0.0f;
static bool s_arcNextIsFinal = false;
static float s_arcPrevHeadingErr = 0.0f;
static uint32_t s_arcExitGraceUntilTick = 0U;

/* Biến theo dõi không gian hẹp / gặp vật cản gần (ép quay tại chỗ) */
static bool s_isTightSpace = false;
static bool s_inEmergencySpin = false;

void Motion_SetTightSpace(bool is_tight) {
  s_isTightSpace = is_tight;
}

bool Motion_IsTightSpace(void) {
  return s_isTightSpace;
}

void Motion_UpdateSensors(float left_mm, bool left_valid,
                          float mid_mm,  bool mid_valid,
                          float right_mm, bool right_valid) {
  /*
   * TÌNH HUỐNG 1: "GẶP VẬT CẢN MÀ KHÔNG GIAN CHO XE BÉ":
   * - Cảm biến giữa (phía trước xe) phát hiện cản gần: <= 140 mm (14 cm)
   * - Hoặc cảm biến 2 bên phát hiện tường / mép cản quá sát thành xe: <= 85 mm (8.5 cm)
   */
  bool tight = false;
  if (mid_valid && mid_mm > 10.0f && mid_mm <= 140.0f) {
    tight = true;
  }
  if (left_valid && left_mm > 10.0f && left_mm <= 85.0f) {
    tight = true;
  }
  if (right_valid && right_mm > 10.0f && right_mm <= 85.0f) {
    tight = true;
  }
  s_isTightSpace = tight;
}

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
  navIsFinalGoal = true;
  s_arcNextIsFinal = false;
  s_smoothSetL = 0.0f;
  s_smoothSetR = 0.0f;
  s_inEmergencySpin = false;

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
  navIsFinalGoal = true;
  s_arcNextIsFinal = false;
  s_smoothSetL = 0.0f;
  s_smoothSetR = 0.0f;
  s_inEmergencySpin = false;

  RunMotorRight(0);
  RunMotorLeft(0);
}

void Motion_SetCommandStatus(uint8_t cmd, uint8_t status) {
  activeCommand = cmd;
  commandStatus = status;
  statusTimestamp = HAL_GetTick();
}

void Motion_NavToPointEx(float targetX, float targetY, bool isFinalGoal, bool forceTurn) {
  Odometry_State odom;
  if (Odometry_GetState(&odom) != HAL_OK) {
    odom.x_mm = 0.0f;
    odom.y_mm = 0.0f;
  }
  float startX = odom.x_mm;
  float startY = odom.y_mm;

  float dx = targetX - startX;
  float dy = targetY - startY;
  float dist_mm = sqrtf(dx * dx + dy * dy);

  if (isFinalGoal && dist_mm < NAV_ARRIVE_TOL_MM) {
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

  float pathAngleRad = atan2f(dy, dx);
  float pathAngleDeg = pathAngleRad * (180.0f / 3.141592653589793f);
  while (pathAngleDeg >= 360.0f) pathAngleDeg -= 360.0f;
  while (pathAngleDeg < 0.0f)   pathAngleDeg += 360.0f;

  float currentHeading = GyroZupt_GetHeading();
  float initialAngleErr = NormalizeAngleDiff(pathAngleDeg, currentHeading);

  navTargetX = targetX;
  navTargetY = targetY;
  navStartX = startX;
  navStartY = startY;
  navTargetHeadingDeg = pathAngleDeg;
  navStartTick = HAL_GetTick();
  navIsFinalGoal = isFinalGoal;
  s_inEmergencySpin = false;

  if (forceTurn) {
    /* Ép quay tại chỗ (R = 0) cho hết góc lệch trước khi chạy thẳng */
    WheelSpeedPID_Reset(&s_pid_right);
    WheelSpeedPID_Reset(&s_pid_left);
    s_smoothSetL = 0.0f;
    s_smoothSetR = 0.0f;
    if (fabsf(initialAngleErr) <= 1.5f) {
      navStage = NAV_STAGE_DRIVE_STRAIGHT;
    } else {
      navStage = NAV_STAGE_TURN;
      navPrevTurnError = initialAngleErr;
    }
  } else {
    bool isMovingTransition = (currentMode == MOTION_MODE_NAV_TO_POINT) &&
                              (navStage == NAV_STAGE_DRIVE_STRAIGHT) &&
                              (fabsf(initialAngleErr) <= NAV_MAX_ARC_CORNER_ANGLE_DEG);
    bool isGentleLaunch = (currentMode == MOTION_MODE_STOP) &&
                          (fabsf(initialAngleErr) <= 10.0f);

    if (isMovingTransition || isGentleLaunch) {
      navStage = NAV_STAGE_DRIVE_STRAIGHT;
    } else {
      WheelSpeedPID_Reset(&s_pid_right);
      WheelSpeedPID_Reset(&s_pid_left);
      s_smoothSetL = 0.0f;
      s_smoothSetR = 0.0f;
      if (fabsf(initialAngleErr) <= NAV_TURN_TARGET_TOL_DEG) {
        navStage = NAV_STAGE_DRIVE_STRAIGHT;
      } else {
        navStage = NAV_STAGE_TURN;
        navPrevTurnError = initialAngleErr;
      }
    }
  }

  currentMode = MOTION_MODE_NAV_TO_POINT;
  activeCommand = CMD_NAV_TO_POINT;
  commandStatus = STATUS_RUNNING;
  statusTimestamp = HAL_GetTick();
}

void Motion_NavToPoint(float targetX, float targetY, bool isFinalGoal) {
  Motion_NavToPointEx(targetX, targetY, isFinalGoal, false);
}

void Motion_StartArcICR(float radius_mm, int8_t dir, float target_exit_heading_deg,
                        float tout_x, float tout_y, float u2_x, float u2_y,
                        float next_target_x, float next_target_y, bool next_is_final) {
  s_arcRadiusMm = radius_mm;
  s_arcDirection = dir;
  float normTarget = target_exit_heading_deg;
  while (normTarget >= 360.0f) normTarget -= 360.0f;
  while (normTarget < 0.0f) normTarget += 360.0f;
  s_arcTargetHeadingDeg = normTarget;
  s_arcToutX = tout_x;
  s_arcToutY = tout_y;
  s_arcU2X = u2_x;
  s_arcU2Y = u2_y;
  s_arcNextTargetX = next_target_x;
  s_arcNextTargetY = next_target_y;
  s_arcNextIsFinal = next_is_final;
  s_arcOuterSpeedMmS = NAV_ARC_OUTER_SPEED_MM_S;

  s_smoothSetL = 0.0f;
  s_smoothSetR = 0.0f;
  s_arcPrevHeadingErr = 0.0f;
  s_arcExitGraceUntilTick = 0U;

  /* Hạ tức thời tích phân I của bánh trong theo tỉ lệ vận tốc mục tiêu cung tròn:
   * Triệt tiêu hiện tượng tích phân cũ ở 250-300 mm/s giữ bánh trong quay nhanh làm xe văng rộng khỏi khúc cua */
  float innerRatio = (radius_mm - ODOMETRY_WHEEL_TRACK_MM * 0.5f) /
                     (radius_mm + ODOMETRY_WHEEL_TRACK_MM * 0.5f);
  if (innerRatio < 0.0f) innerRatio = 0.0f;
  if (dir > 0) {
    /* Rẽ trái: bánh trái là bánh trong */
    if (s_pid_left.integral > 0.0f) {
      s_pid_left.integral *= innerRatio;
    }
  } else {
    /* Rẽ phải: bánh phải là bánh trong */
    if (s_pid_right.integral > 0.0f) {
      s_pid_right.integral *= innerRatio;
    }
  }

  navStage = NAV_STAGE_ARC_ICR;
  navStartTick = HAL_GetTick();
  s_inEmergencySpin = false;
  currentMode = MOTION_MODE_NAV_TO_POINT;
  activeCommand = CMD_NAV_TO_POINT;
  commandStatus = STATUS_RUNNING;
  statusTimestamp = HAL_GetTick();
}

bool Motion_IsArcActive(void) {
  return (currentMode == MOTION_MODE_NAV_TO_POINT && navStage == NAV_STAGE_ARC_ICR);
}

bool Motion_IsTurningInPlace(void) {
  return (currentMode == MOTION_MODE_ROTATE_ANGLE) ||
         (currentMode == MOTION_MODE_NAV_TO_POINT && (navStage == NAV_STAGE_TURN || s_inEmergencySpin));
}

void Motion_SetTargetHeading(float heading_deg) {
  while (heading_deg >= 360.0f) heading_deg -= 360.0f;
  while (heading_deg < 0.0f)   heading_deg += 360.0f;
  navTargetHeadingDeg = heading_deg;
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
            straightTargetHeadingDeg = GyroZupt_GetHeading();
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

      float currentHeading = GyroZupt_GetHeading();
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
      /* Gọi hàm điều hướng với is_final_goal = true (lệnh đơn lẻ từ UART dừng khi tới điểm) */
      Motion_NavToPoint((float)data1, (float)data2, true);
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
      float currentHeading = GyroZupt_GetHeading();
      float currentOmega   = GyroZupt_GetAngularVelocity();
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

      /* Chạy PID từng bánh theo độ rộng xung PWM (us) siêu mịn */
      const float dt = (float)ODOMETRY_SPEED_AVERAGE_WINDOW_MS / 1000.0f;
      float pulseR = WheelSpeedPID_Update(&s_pid_right, setR, actualR, dt,
                                          MOTOR_RIGHT_PWM_OFFSET_US, MOTOR_RIGHT_PWM_DEADZONE_US, MOTOR_RIGHT_PWM_MAXZONE_US);
      float pulseL = WheelSpeedPID_Update(&s_pid_left,  setL, actualL, dt,
                                          MOTOR_LEFT_PWM_OFFSET_US, MOTOR_LEFT_PWM_DEADZONE_US, MOTOR_LEFT_PWM_MAXZONE_US);

      RunMotorRightPulseUs((uint16_t)roundf(pulseR));
      RunMotorLeftPulseUs((uint16_t)roundf(pulseL));
    }
    return;
  }

  if (currentMode == MOTION_MODE_ROTATE_ANGLE) {
    /* Vòng lặp điều khiển PID xoay góc tại chỗ */
    float currentHeading = GyroZupt_GetHeading();
    float currentOmega = GyroZupt_GetAngularVelocity();
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

    /* [LỆNH XOAY GÓC TẠI CHỖ (CMD_ROTATE_ANGLE)]:
     * - Nếu không gian hẹp (s_isTightSpace == true) hoặc khi lệnh yêu cầu xoay tại chỗ:
     *   QUAY TẠI CHỖ (SPIN IN PLACE, R = 0): 2 bánh quay ngược chiều nhau quanh tâm trục bánh.
     *   Bán kính quay = 0, mũi xe không bị văng ra xa, không va chạm với vật cản!
     * - Nếu không gian thoáng (s_isTightSpace == false):
     *   Cua gắt (Pivot turn quanh bánh bên cua dừng hẳn).
     */
    int16_t pwm = (int16_t)roundf(fabsf(u));
    if (s_isTightSpace) {
      /* Quay tại chỗ: Bánh trong lùi, bánh ngoài tiến */
      if (errorAngle > 0.0f) {
        RunMotorLeft(-pwm);
        RunMotorRight(pwm);
      } else if (errorAngle < 0.0f) {
        RunMotorRight(-pwm);
        RunMotorLeft(pwm);
      } else {
        RunMotorRight(0);
        RunMotorLeft(0);
      }
    } else {
      /* Cua gắt: Bánh bên cua dừng hẳn */
      if (errorAngle > 0.0f) {
        RunMotorLeft(0);
        RunMotorRight(pwm);
      } else if (errorAngle < 0.0f) {
        RunMotorRight(0);
        RunMotorLeft(pwm);
      } else {
        RunMotorRight(0);
        RunMotorLeft(0);
      }
    }
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
      float currentHeading = GyroZupt_GetHeading();
      float currentOmega = GyroZupt_GetAngularVelocity();
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
      if (absErr > NAV_TURN_TARGET_TOL_DEG && fabsf(currentOmega) < 5.0f) {
        float minPwm = (absErr < 6.0f) ? fmaxf(10.0f, MIN_ANG_PWM * (absErr / 6.0f)) : MIN_ANG_PWM;
        if (u > 0.0f && u < minPwm) u = minPwm;
        if (u < 0.0f && u > -minPwm) u = -minPwm;
      }

      /*
       * [ĐIỀU HƯỚNG XOAY ĐẦU HƯỚNG VỀ MỐC ĐÍCH]:
       * QUAY TẠI CHỖ (SPIN IN PLACE, R = 0): 2 bánh quay ngược chiều nhau quanh tâm trục bánh.
       * Tâm quay tại chính giữa xe, đầu xe không bị văng ra xa, không va chạm với vật cản!
       */
      int16_t pwm = (int16_t)roundf(fabsf(u));
      if (errorAngle > 0.0f) {
        RunMotorLeft(-pwm);
        RunMotorRight(pwm);
      } else if (errorAngle < 0.0f) {
        RunMotorRight(-pwm);
        RunMotorLeft(pwm);
      } else {
        RunMotorRight(0);
        RunMotorLeft(0);
      }
      return;
    }

    /* Giai đoạn 3: Ôm cung tròn vi sai ICR (với góc tiếp tuyến thoát cố định và lead-in braking) */
    if (navStage == NAV_STAGE_ARC_ICR) {
      Odometry_State odom;
      if (Odometry_GetState(&odom) != HAL_OK) {
        odom.x_mm = navStartX;
        odom.y_mm = navStartY;
      }
      const float dt = (float)ODOMETRY_SPEED_AVERAGE_WINDOW_MS / 1000.0f;
      float actualR = odom.wheel_linear_mm_s[ENCODER_RIGHT];
      float actualL = odom.wheel_linear_mm_s[ENCODER_LEFT];

      float currentHeading = GyroZupt_GetHeading();
      float errHead = NormalizeAngleDiff(s_arcTargetHeadingDeg, currentHeading);
      float absErr = fabsf(errHead);
      float remAngle = (s_arcDirection > 0) ? errHead : -errHead;

      /* Kiểm tra thoát cua nghiêm ngặt:
       * 1. remAngle <= 0.0f (đã quay đủ/vượt qua góc tiếp tuyến đích theo chiều cua)
       * 2. absErr <= 2.5f (đầu xe đã thẳng hàng)
       * 3. crossedZero (vừa cắt qua điểm 0)
       * 4. passedTout (đã vượt qua mặt phẳng tiếp tuyến Tout) */
      bool crossedZero = (s_arcPrevHeadingErr != 0.0f) && (s_arcPrevHeadingErr * errHead < 0.0f);
      float projTout = (odom.x_mm - s_arcToutX) * s_arcU2X + (odom.y_mm - s_arcToutY) * s_arcU2Y;
      bool passedTout = (projTout >= 2.0f && absErr <= 10.0f);

      if (remAngle <= 0.0f || absErr <= 2.0f || crossedZero || passedTout) {
        /* Cân bằng tích phân I hai bánh khi trả thẳng tại Tout để không bị lắc đuôi chữ S sau cua */
        float exitIntegral = fmaxf(s_pid_left.integral, s_pid_right.integral);
        if (exitIntegral > 0.0f) {
          s_pid_left.integral  = exitIntegral;
          s_pid_right.integral = exitIntegral;
        }
        s_smoothSetL = 0.0f;
        s_smoothSetR = 0.0f;
        navStartX = s_arcToutX;
        navStartY = s_arcToutY;
        navTargetX = s_arcNextTargetX;
        navTargetY = s_arcNextTargetY;
        navTargetHeadingDeg = s_arcTargetHeadingDeg;
        navIsFinalGoal = s_arcNextIsFinal;
        navStage = NAV_STAGE_DRIVE_STRAIGHT;
        s_arcPrevHeadingErr = 0.0f;
        s_arcExitGraceUntilTick = HAL_GetTick() + 250U;
        return;
      }
      s_arcPrevHeadingErr = errHead;

      /* Tâm hình học C của cung tròn tính từ điểm tiếp tuyến thoát Tout và pháp tuyến vào tâm:
       * Phản hồi vòng kín sai số bán kính (errR > 0 là xe đang bị văng rộng ra ngoài cung -> siết bánh trong) */
      float n2x = -(float)s_arcDirection * s_arcU2Y;
      float n2y =  (float)s_arcDirection * s_arcU2X;
      float arcCx = s_arcToutX + s_arcRadiusMm * n2x;
      float arcCy = s_arcToutY + s_arcRadiusMm * n2y;
      float curRadMm = sqrtf((odom.x_mm - arcCx) * (odom.x_mm - arcCx) +
                             (odom.y_mm - arcCy) * (odom.y_mm - arcCy));
      float radErrMm = curRadMm - s_arcRadiusMm; /* > 0: văng rộng ra ngoài cung; < 0: ôm quá sát tâm */

      /* Vận tốc cung tròn + bù sai số bán kính vòng kín + trả thẳng mượt trong 5° cuối: */
      float zoneOuter = (odom.x_mm >= 900.0f) ? NAV_SCAN_ZONE_SPEED_MM_S : s_arcOuterSpeedMmS;
      float zoneInner = zoneOuter * (s_arcRadiusMm - ODOMETRY_WHEEL_TRACK_MM * 0.5f) /
                        (s_arcRadiusMm + ODOMETRY_WHEEL_TRACK_MM * 0.5f);
      float radCorr = radErrMm * 3.2f;
      if (radCorr >  85.0f) radCorr =  85.0f;
      if (radCorr < -65.0f) radCorr = -65.0f;

      float outerSpd = zoneOuter;
      float innerSpd = zoneInner - radCorr;
      if (innerSpd < 0.0f)      innerSpd = 0.0f;
      if (innerSpd > zoneOuter) innerSpd = zoneOuter;

      if (remAngle <= 5.0f && remAngle > 0.0f) {
        float blend = remAngle / 5.0f;
        outerSpd = zoneOuter;
        innerSpd = zoneOuter + blend * (innerSpd - zoneOuter);
      }

      float setR, setL;
      if (s_arcDirection > 0) {
        setR = outerSpd;
        setL = innerSpd;
      } else {
        setL = outerSpd;
        setR = innerSpd;
      }

      float pulseR = WheelSpeedPID_Update(&s_pid_right, setR, actualR, dt,
                                          MOTOR_RIGHT_PWM_OFFSET_US, MOTOR_RIGHT_PWM_DEADZONE_US, MOTOR_RIGHT_PWM_MAXZONE_US);
      float pulseL = WheelSpeedPID_Update(&s_pid_left,  setL, actualL, dt,
                                          MOTOR_LEFT_PWM_OFFSET_US, MOTOR_LEFT_PWM_DEADZONE_US, MOTOR_LEFT_PWM_MAXZONE_US);
      RunMotorRightPulseUs((uint16_t)roundf(pulseR));
      RunMotorLeftPulseUs((uint16_t)roundf(pulseL));
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

      float totalDx = navTargetX - navStartX;
      float totalDy = navTargetY - navStartY;
      float proj = totalDx * dx + totalDy * dy;

      /* Kiểm tra dừng hoàn thành: CHỈ dừng lại khi đây là mốc đích cuối cùng (navIsFinalGoal == true) */
      if (navIsFinalGoal) {
        if (remainingDist <= NAV_ARRIVE_TOL_MM || (remainingDist <= 60.0f && proj <= 0.0f)) {
          RunMotorRight(0);
          RunMotorLeft(0);
          WheelSpeedPID_Reset(&s_pid_right);
          WheelSpeedPID_Reset(&s_pid_left);
          s_smoothSetL = 0.0f;
          s_smoothSetR = 0.0f;
          currentMode = MOTION_MODE_STOP;
          activeCommand = CMD_NAV_TO_POINT;
          commandStatus = STATUS_COMPLETED;
          statusTimestamp = HAL_GetTick();
          return;
        }
      }

      /* Centerline Lookahead Pursuit (130mm): bám chặt trục thẳng navStart -> navTarget để triệt tiêu sai số ngang (CTE);
       * Khi khoảng cách < 35mm (sắp tới đích), HOẶC đối với mốc trung gian (!navIsFinalGoal) mà xe đã
       * vượt qua mặt phẳng đích (proj <= 0): GIỮ NGUYÊN navTargetHeadingDeg, TUYỆT ĐỐI KHÔNG quay đầu xe lại! */
      float directAngleDeg;
      if (remainingDist < 35.0f || (!navIsFinalGoal && proj <= 0.0f)) {
        directAngleDeg = navTargetHeadingDeg;
      } else {
        float aimX = navTargetX;
        float aimY = navTargetY;
        float segLen = sqrtf(totalDx * totalDx + totalDy * totalDy);
        if (segLen > 1.0f) {
          float ux = totalDx / segLen;
          float uy = totalDy / segLen;
          float sAlong = (odom.x_mm - navStartX) * ux + (odom.y_mm - navStartY) * uy;
          if (sAlong < segLen - 20.0f) {
            float sAim = sAlong + 95.0f;
            if (sAim < 0.0f)   sAim = 0.0f;
            if (sAim > segLen) sAim = segLen;
            aimX = navStartX + sAim * ux;
            aimY = navStartY + sAim * uy;
          }
        }
        float directAngleRad = atan2f(aimY - odom.y_mm, aimX - odom.x_mm);
        directAngleDeg = directAngleRad * (180.0f / 3.141592653589793f);
        while (directAngleDeg >= 360.0f) directAngleDeg -= 360.0f;
        while (directAngleDeg <  0.0f)   directAngleDeg += 360.0f;
        navTargetHeadingDeg = directAngleDeg;
      }

      float currentHeading = GyroZupt_GetHeading();
      float currentOmega   = GyroZupt_GetAngularVelocity();
      float errorHeading   = NormalizeAngleDiff(directAngleDeg, currentHeading);
      float absHeadingErr  = fabsf(errorHeading);

      /* Vận tốc hành trình cơ sở tới đích (X >= 900 mm giảm về 250 mm/s để quét ToF ổn định; X < 900 mm giữ 300 mm/s): */
      float baseCruiseMmS = (odom.x_mm >= 900.0f) ? NAV_SCAN_ZONE_SPEED_MM_S : NAV_CRUISE_SPEED_MM_S;
      float cruiseSpeedMmS = baseCruiseMmS;
      if (absHeadingErr > 22.0f) {
        float turnScale = 1.0f - fminf(0.55f, (absHeadingErr - 22.0f) / 30.0f);
        cruiseSpeedMmS *= turnScale;
        if (cruiseSpeedMmS < 135.0f) cruiseSpeedMmS = 135.0f;
      }
      if (navIsFinalGoal && remainingDist < 100.0f) {
        float speedToGoal = NAV_MIN_SPEED_MM_S + (baseCruiseMmS - NAV_MIN_SPEED_MM_S) * (remainingDist / 100.0f);
        if (speedToGoal < cruiseSpeedMmS) cruiseSpeedMmS = speedToGoal;
      }
      if (cruiseSpeedMmS < NAV_MIN_SPEED_MM_S) cruiseSpeedMmS = NAV_MIN_SPEED_MM_S;

      /*
       * ====================================================================
       * CHIẾN THUẬT ÔM CUA THÍCH ỨNG THEO TÌNH HÌNH THỰC TẾ:
       * ====================================================================
       * 1. GẶP VẬT CẢN MÀ KHÔNG GIAN HẸP (s_isTightSpace == true) VÀ LỆCH GÓC RÕ RỆT (absHeadingErr > 12 độ):
       *    -> PHẢI QUAY TẠI CHỖ (SPIN IN PLACE, R = 0), KHÔNG QUAY CUNG TRÒN!
       *    -> Bánh bên trong lùi (-pwm), bánh bên ngoài tiến (+pwm).
       *    -> Tâm quay tại chính giữa xe, đầu xe không tịnh tiến văng vào vật cản!
       *
       * 2. GÓC CUA LỚN HƠN (absHeadingErr > 45 ĐỘ, GÓC ĐỈNH <= 135 ĐỘ NHƯ GÓC VUÔNG 90 ĐỘ):
       *    -> CUA GẮT PIVOT TURN (BÁNH 1 BÊN DỪNG HẲN)!
       *    -> Bánh bên cua dừng hẳn = 0 (RunMotorLeft(0) hoặc RunMotorRight(0)).
       *    -> Bánh ngoài tiến với turnSpeed (NAV_PIVOT_OUTER_SPEED_MM_S = 160 mm/s).
       *    -> Bo gắt góc cua fillet tại đúng đỉnh mốc.
       *
       * 3. NHỮNG GÓC NHẸ > |135 ĐỘ| (GÓC LỆCH HƯỚNG 12 ĐỘ < absHeadingErr <= 45 ĐỘ):
       *    -> CHO CHẠY GÓC CUNG TRÒN (SMOOTH ARC TURN)!
       *    -> Cả 2 bánh cùng quay TIẾN. Bánh ngoài chạy tốc độ hành trình, bánh trong giảm tốc
       *       tỉ lệ với góc lệch nhưng luôn giữ vận tốc dương (tối thiểu 50 mm/s, không dừng).
       *    -> Xe lướt mượt qua góc cua theo hình cung tròn tự nhiên, không bị giật hay khựng.
       *
       * 4. ĐI THẲNG / LỆCH GÓC NHỎ (absHeadingErr <= 12 ĐỘ):
       *    -> Cả 2 bánh chạy tốc độ cao cruiseSpeedMmS, bám tuyến thẳng bằng vi sai PD nhỏ.
       * ====================================================================
       */
      const float dt = (float)ODOMETRY_SPEED_AVERAGE_WINDOW_MS / 1000.0f;
      float actualR = odom.wheel_linear_mm_s[ENCODER_RIGHT];
      float actualL = odom.wheel_linear_mm_s[ENCODER_LEFT];

      /* 1. Lệch góc lớn (> 40 độ hoặc > 28 độ khi hẹp): Quay tại chỗ tái định hướng (bỏ qua trong 250ms sau khi thoát arc) */
      bool inGrace = (HAL_GetTick() < s_arcExitGraceUntilTick);
      if (!inGrace && (absHeadingErr > 40.0f || (s_isTightSpace && absHeadingErr > 28.0f))) {
        s_inEmergencySpin = true;
        float p_term = KP_ROT_ANG * errorHeading;
        float d_term = KD_ROT_ANG * currentOmega;
        float u = p_term - d_term;
        if (u > CRUISE_PWM) u = CRUISE_PWM;
        if (u < -CRUISE_PWM) u = -CRUISE_PWM;
        if (absHeadingErr > NAV_TURN_TARGET_TOL_DEG && fabsf(currentOmega) < 5.0f) {
          float minPwm = (absHeadingErr < 6.0f) ? fmaxf(10.0f, MIN_ANG_PWM * (absHeadingErr / 6.0f)) : MIN_ANG_PWM;
          if (u > 0.0f && u < minPwm) u = minPwm;
          if (u < 0.0f && u > -minPwm) u = -minPwm;
        }
        int16_t pwm = (int16_t)roundf(fabsf(u));
        if (errorHeading > 0.0f) {
          RunMotorLeft(-pwm);
          RunMotorRight(pwm);
        } else {
          RunMotorRight(-pwm);
          RunMotorLeft(pwm);
        }
        WheelSpeedPID_Reset(&s_pid_left);
        WheelSpeedPID_Reset(&s_pid_right);
        s_smoothSetL = 0.0f;
        s_smoothSetR = 0.0f;
        return;
      }
      s_inEmergencySpin = false;

      /* 2. GIAI ĐOẠN CHẠY THẲNG: Giữ hướng tắp bằng PID vi sai Heading-Hold */

      /* Chạy thẳng hoặc bám hướng tinh chỉnh: vi sai PD nhỏ quanh deadband */
      float targetCorrection = 0.0f;
      if (absHeadingErr > NAV_HEADING_DEADBAND_DEG) {
        float effectiveError = (errorHeading > 0.0f) ? (errorHeading - NAV_HEADING_DEADBAND_DEG)
                                                     : (errorHeading + NAV_HEADING_DEADBAND_DEG);
        float p_term = NAV_HEADING_KP * effectiveError;
        float d_term = NAV_HEADING_KD * currentOmega;
        targetCorrection = p_term - d_term;
        if (targetCorrection >  MAX_STRAIGHT_CORRECTION_MM_S) targetCorrection =  MAX_STRAIGHT_CORRECTION_MM_S;
        if (targetCorrection < -MAX_STRAIGHT_CORRECTION_MM_S) targetCorrection = -MAX_STRAIGHT_CORRECTION_MM_S;
      }
      /* Cua trái (errorHeading > 0) -> targetCorrection > 0 -> setR tăng, setL giảm nhẹ */
      float setR = cruiseSpeedMmS + targetCorrection * 0.5f;
      float setL = cruiseSpeedMmS - targetCorrection * 0.5f;

      float pulseR = WheelSpeedPID_Update(&s_pid_right, setR, actualR, dt,
                                          MOTOR_RIGHT_PWM_OFFSET_US, MOTOR_RIGHT_PWM_DEADZONE_US, MOTOR_RIGHT_PWM_MAXZONE_US);
      float pulseL = WheelSpeedPID_Update(&s_pid_left,  setL, actualL, dt,
                                          MOTOR_LEFT_PWM_OFFSET_US, MOTOR_LEFT_PWM_DEADZONE_US, MOTOR_LEFT_PWM_MAXZONE_US);

      RunMotorRightPulseUs((uint16_t)roundf(pulseR));
      RunMotorLeftPulseUs((uint16_t)roundf(pulseL));
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
