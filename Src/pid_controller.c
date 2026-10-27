#include "pid_controller.h"
#include "motor_pwm.h"
#include "kalman_filter.h"
#include "odometry.h"
#include <math.h>

/* ================= ĐỊNH NGHĨA THAM SỐ XOAY GÓC TỰ ĐỘNG PID ================= */
#define KP_ROT_ANG    0.5f   /* Hệ số tỉ lệ P  */
#define KD_ROT_ANG    0.01f   /* Giảm chấn vi phân vận tốc góc */
#define CRUISE_PWM    32.0f   /* Tốc độ hành trình khi quay */
#define MAX_ANG_PWM   45.0f   /* Tốc độ xoay tối đa an toàn */
#define MIN_ANG_PWM   14.0f   /* Bù deadzone ma sát tĩnh của bánh xe */
#define ANGLE_TOL_DEG 2.0f    /* Dung sai góc chấp nhận được (2.0 độ theo yêu cầu) */
#define ANGLE_TIMEOUT 5000U   /* Thời gian tối đa bảo vệ cho một lệnh xoay góc (ms) */
#define MANUAL_WATCHDOG_TIMEOUT_MS 250U /* Lớp bảo vệ phần cứng STM32: Quá 250ms không có nhịp UART -> Dừng khẩn cấp */

/* ================= ĐỊNH NGHĨA THAM SỐ PID ĐI THẲNG (HEADING-HOLD) ================= */
#define KP_STRAIGHT              0.85f  /* Hệ số P bám góc khi đi thẳng tiến/lùi */
#define KD_STRAIGHT              0.02f  /* Giảm chấn vi phân vận tốc góc */
#define MAX_STRAIGHT_CORRECTION  15.0f  /* Giới hạn bù PWM tối đa tránh giật xe */

/* ================= ĐỊNH NGHĨA THAM SỐ ĐIỀU KHIỂN ĐIỂM (NAV TO POINT) ================= */
typedef enum {
  NAV_STAGE_TURN = 0,           /* Giai đoạn 1: Xoay đầu hướng về điểm đích */
  NAV_STAGE_DRIVE_STRAIGHT      /* Giai đoạn 2: Chạy thẳng tới điểm đích */
} NavStage;

#define NAV_CRUISE_PWM           26.0f  /* Tốc độ hành trình khi chạy tới điểm */
#define NAV_MIN_PWM              14.0f  /* Tốc độ tối thiểu thắng ma sát tĩnh */
#define NAV_ARRIVE_TOL_MM        18.0f  /* Bán kính dung sai tới đích (mm) */
#define NAV_TIMEOUT_MS           25000U /* Thời gian tối đa bảo vệ cho một lệnh chạy tới điểm (25s) */

static MotionMode currentMode = MOTION_MODE_STOP;
static uint8_t activeCommand = CMD_STOP;
static uint8_t commandStatus = STATUS_IDLE;
static uint32_t statusTimestamp = 0U;
static uint32_t lastManualCommandTick = 0U;

/* Biến phục vụ PID xoay góc */
static float targetHeadingDeg = 0.0f;
static uint32_t turnStartTick = 0U;
static uint8_t turnSettleCount = 0U;

/* Biến phục vụ PID đi thẳng */
static bool straightLockActive = false;
static float straightTargetHeadingDeg = 0.0f;
static int16_t straightBaseSpeed = 0;

/* Biến phục vụ Nav To Point */
static NavStage navStage = NAV_STAGE_TURN;
static float navStartX = 0.0f;
static float navStartY = 0.0f;
static float navTargetX = 0.0f;
static float navTargetY = 0.0f;
static float navTargetHeadingDeg = 0.0f;
static uint32_t navStartTick = 0U;
static uint8_t navTurnSettleCount = 0U;

/* Chuẩn hóa hiệu góc giữa đích và hiện tại về khoảng [-180, +180] độ */
static float NormalizeAngleDiff(float target_deg, float current_deg) {
  float diff = target_deg - current_deg;
  while (diff > 180.0f) diff -= 360.0f;
  while (diff < -180.0f) diff += 360.0f;
  return diff;
}

void Motion_Init(void) {
  currentMode = MOTION_MODE_STOP;
  activeCommand = CMD_STOP;
  commandStatus = STATUS_IDLE;
  statusTimestamp = HAL_GetTick();
  lastManualCommandTick = HAL_GetTick();
  straightLockActive = false;
  straightBaseSpeed = 0;
  navStage = NAV_STAGE_TURN;

  RunMotorRight(0);
  RunMotorLeft(0);
}

void Motion_Stop(void) {
  currentMode = MOTION_MODE_STOP;
  activeCommand = CMD_EMERGENCY_STOP;
  commandStatus = STATUS_IDLE;
  statusTimestamp = HAL_GetTick();
  straightLockActive = false;
  straightBaseSpeed = 0;

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
      return; /* Đang thực thi dở, bỏ qua gói trùng */
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
        straightBaseSpeed = 0;
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
          }
          straightBaseSpeed = r;
          RunMotorRight(r);
          RunMotorLeft(l);
        } else {
          /* Lệnh quay tay A/D (r != l): xuất PWM vi sai trực tiếp không khóa góc */
          straightLockActive = false;
          straightBaseSpeed = 0;
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

      navTargetX = targetX;
      navTargetY = targetY;
      navStartX = startX;
      navStartY = startY;
      navTargetHeadingDeg = pathAngleDeg;
      navStage = NAV_STAGE_TURN; /* Giai đoạn 1: Quay đầu hướng về tọa độ đích */
      navTurnSettleCount = 0U;
      navStartTick = HAL_GetTick();

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

    /* PID điều khiển xe đi thẳng tuyệt đối khi bấm W (tiến) hoặc S (lùi) */
    if (straightLockActive && straightBaseSpeed != 0) {
      float currentHeading = KalmanOdometry_GetHeading();
      float currentOmega = KalmanOdometry_GetAngularVelocity();
      float errorHeading = NormalizeAngleDiff(straightTargetHeadingDeg, currentHeading);

      float p_term = KP_STRAIGHT * errorHeading;
      float d_term = KD_STRAIGHT * currentOmega;
      float correction = p_term - d_term;

      if (correction > MAX_STRAIGHT_CORRECTION) correction = MAX_STRAIGHT_CORRECTION;
      if (correction < -MAX_STRAIGHT_CORRECTION) correction = -MAX_STRAIGHT_CORRECTION;

      int16_t rPwm, lPwm;
      if (straightBaseSpeed > 0) {
        /* Chạy tiến (W): lệch phải (error > 0, corr > 0) -> tăng phải, giảm trái để quẹo trái về trục */
        rPwm = (int16_t)roundf(straightBaseSpeed + correction);
        lPwm = (int16_t)roundf(straightBaseSpeed - correction);
      } else {
        /* Chạy lùi (S): lệch phải (error > 0, corr > 0) -> lùi phải mạnh hơn, lùi trái nhẹ hơn */
        rPwm = (int16_t)roundf(straightBaseSpeed - correction);
        lPwm = (int16_t)roundf(straightBaseSpeed + correction);
      }

      if (rPwm > 100) rPwm = 100;
      if (rPwm < -100) rPwm = -100;
      if (lPwm > 100) lPwm = 100;
      if (lPwm < -100) lPwm = -100;

      RunMotorRight(rPwm);
      RunMotorLeft(lPwm);
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

    /* Bù deadzone ma sát tĩnh của bánh xe: nếu chưa tới đích thì giữ xung tối thiểu thắng ma sát */
    if (absErr > ANGLE_TOL_DEG) {
      if (u > 0.0f && u < MIN_ANG_PWM) u = MIN_ANG_PWM;
      if (u < 0.0f && u > -MIN_ANG_PWM) u = -MIN_ANG_PWM;
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

      if (absErr <= ANGLE_TOL_DEG && fabsf(currentOmega) <= 4.0f) {
        navTurnSettleCount++;
        if (navTurnSettleCount >= 6U) {
          RunMotorRight(0);
          RunMotorLeft(0);
          navStage = NAV_STAGE_DRIVE_STRAIGHT;
          return;
        }
      } else {
        navTurnSettleCount = 0U;
      }

      float p_term = KP_ROT_ANG * errorAngle;
      float d_term = KD_ROT_ANG * currentOmega;
      float u = p_term - d_term;

      if (u > CRUISE_PWM) u = CRUISE_PWM;
      if (u < -CRUISE_PWM) u = -CRUISE_PWM;

      if (absErr > ANGLE_TOL_DEG) {
        if (u > 0.0f && u < MIN_ANG_PWM) u = MIN_ANG_PWM;
        if (u < 0.0f && u > -MIN_ANG_PWM) u = -MIN_ANG_PWM;
      }

      RunMotorRight((int16_t)roundf(u));
      RunMotorLeft((int16_t)roundf(-u));
      return;
    }

    /* Giai đoạn 2: Chạy thẳng đến điểm đích với PID giữ hướng và hãm tốc mượt */
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

      if (remainingDist <= NAV_ARRIVE_TOL_MM || proj <= 0.0f) {
        RunMotorRight(0);
        RunMotorLeft(0);
        currentMode = MOTION_MODE_STOP;
        activeCommand = CMD_NAV_TO_POINT;
        commandStatus = STATUS_COMPLETED;
        statusTimestamp = HAL_GetTick();
        return;
      }

      /* Hãm tốc mượt khi gần tới đích (khoảng cách < 100mm) */
      float speed = NAV_CRUISE_PWM;
      if (remainingDist < 100.0f) {
        speed = NAV_MIN_PWM + (NAV_CRUISE_PWM - NAV_MIN_PWM) * (remainingDist / 100.0f);
        if (speed < NAV_MIN_PWM) speed = NAV_MIN_PWM;
      }

      /* Dynamic target heading (Pursuit): luôn ngắm thẳng vào tọa độ đích */
      float directAngleRad = atan2f(dy, dx);
      float directAngleDeg = directAngleRad * (180.0f / 3.141592653589793f);
      while (directAngleDeg >= 360.0f) directAngleDeg -= 360.0f;
      while (directAngleDeg < 0.0f)   directAngleDeg += 360.0f;

      float currentHeading = KalmanOdometry_GetHeading();
      float currentOmega = KalmanOdometry_GetAngularVelocity();
      float errorHeading = NormalizeAngleDiff(directAngleDeg, currentHeading);

      float p_term = KP_STRAIGHT * errorHeading;
      float d_term = KD_STRAIGHT * currentOmega;
      float correction = p_term - d_term;

      if (correction > MAX_STRAIGHT_CORRECTION) correction = MAX_STRAIGHT_CORRECTION;
      if (correction < -MAX_STRAIGHT_CORRECTION) correction = -MAX_STRAIGHT_CORRECTION;

      int16_t rPwm = (int16_t)roundf(speed + correction);
      int16_t lPwm = (int16_t)roundf(speed - correction);

      if (rPwm > 100) rPwm = 100;
      if (rPwm < -100) rPwm = -100;
      if (lPwm > 100) lPwm = 100;
      if (lPwm < -100) lPwm = -100;

      RunMotorRight(rPwm);
      RunMotorLeft(lPwm);
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
