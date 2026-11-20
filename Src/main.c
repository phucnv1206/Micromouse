#include "main.h"
#include "motor_pwm.h"
#include "encoder.h"
#include "odometry.h"
#include "uart_protocol.h"
#include "led.h"
#include "button.h"
#include "tof_sensors.h"
#include "mpu6500.h"
#include "kalman_filter.h"
#include "pid_controller.h"
#include "i2c_bus.h"
#include "navigation.h"
#include "map_grid.h"

/* Biến cờ chế độ quét và lưu bản đồ độc lập trên STM32 */
static bool isMappingModeActive = false;



/* Tổng chu kỳ CPU bận trong cửa sổ tải hiện tại; thời gian ngủ WFI không tính. */
static uint32_t cpuBusyCyclesWindow;
static uint32_t cpuLoadWindowStartTick;
/* Tải CPU theo 0,1%; 123 tương ứng 12,3%, gửi qua telemetry.data2. */
static uint16_t cpuLoadTenthsPercent;
static UartProtocol_Telemetry debugTelemetry;
static Odometry_State debugOdometryState;
static uint16_t debugLeftDistanceMm;
static uint16_t debugMidDistanceMm;
static uint16_t debugRightDistanceMm;
static bool LeftValid;
static bool MidValid;
static bool RightValid;

static float x_left, x_right, x_mid;
static float y_left, y_right, y_mid;

/* Bộ đếm số tick timer odometry đã đến hạn; volatile vì được cộng dồn trong ngắt TIM3. */
static volatile uint16_t odometryUpdatePending;
/* Ngắt TIM3 chạy theo tần số odometry (1 kHz) và tích lũy số tick cho vòng lặp chính. */
void TIM3_IRQHandler(void) {
  if ((TIM3->SR & TIM_SR_UIF) != 0U) {
    /* Xóa cờ cập nhật để timer có thể phát sinh ngắt ở chu kỳ kế tiếp. */
    TIM3->SR &= ~TIM_SR_UIF;
    /* Tích lũy số tick, không dùng boolean để tránh mất tick khi main bị nghẽn */
    odometryUpdatePending++;
  }
}

/*
 * Yield callback đăng ký với I2cBus: được gọi trong vòng chờ DMA của ToF.
 * Nếu TIM3 đã phát sinh tick odometry trong lúc DMA đang chạy, hàm này xử lý ngay
 * thay vì để odometry bị trễ cho đến khi I2C hoàn tất.
 * Không dùng taskRan vì đây là ngữ cảnh yield, không phải vòng lặp chính.
 */
static void I2cYield_ProcessOdometry(void) {
  uint32_t primask = __get_PRIMASK();
  uint16_t dueTicks = 0U;
  __disable_irq();
  if (odometryUpdatePending != 0U) {
    dueTicks = odometryUpdatePending;
    odometryUpdatePending = 0U;
  }
  if (primask == 0U) {
    __enable_irq();
  }
  if (dueTicks != 0U) {
    Mpu6500_Update();
    if (Odometry_Update(dueTicks) != HAL_OK) {
      /* [FIX 1.1] Lỗi odometry trong yield: bỏ qua, không treo CPU */
    }
  }
}



static void readUart(void){
  UartProtocol_RxFrame rxFrame;
  HAL_StatusTypeDef receiveStatus;
  do {
    receiveStatus = UartProtocol_Receive(&rxFrame);
    if (receiveStatus == HAL_OK) {
      if (rxFrame.command == CMD_DUMP_MAP) {
        /* Trích xuất bản đồ: Chỉ thực hiện khi xe KHÔNG trong chế độ quét/lưu map */
        if (!isMappingModeActive) {
          Map_DumpToUart();
        }
      } else if (rxFrame.command == CMD_SET_OBSTACLES) {
        /* Thông báo trước 2 vị trí vật cản ngẫu nhiên từ Web: slotTop (1 hoặc 2), slotBottom (3 hoặc 4) */
        uint8_t slotTop = (uint8_t)rxFrame.data1;
        uint8_t slotBottom = (uint8_t)rxFrame.data2;
        Map_Init(slotTop, slotBottom);
      } else {
        /* Chuyển mã lệnh và dữ liệu vào bộ điều khiển chuyển động PID */
        Motion_ProcessCommand(rxFrame.command, rxFrame.data1, rxFrame.data2);
      }
    } else if (receiveStatus != HAL_BUSY) {
      /* [FIX 1.4] Lỗi UART nhận: bỏ qua, StartReceive sẽ được gọi lại từ ErrorCallback */
    }
  } while (receiveStatus == HAL_OK);
}




static void loop(void) {
  readUart();

  /* Chu kỳ điều khiển vòng lặp chính 100 Hz (dt = 0.01 giây) */
  const float dt_s = 1.0f / (float)CONTROL_LOOP_FREQUENCY_HZ;
  Motion_Update(dt_s);
}


static void debug(void) {
  /* Tính tọa độ vật thể/tường phát hiện bởi 3 cảm biến ToF dựa trên navigation và pose xe */
  leftpose((float)debugLeftDistanceMm, debugOdometryState.x_mm, debugOdometryState.y_mm, debugOdometryState.heading_deg, &x_left, &y_left);
  midpose((float)debugMidDistanceMm, debugOdometryState.x_mm, debugOdometryState.y_mm, debugOdometryState.heading_deg, &x_mid, &y_mid);
  rightpose((float)debugRightDistanceMm, debugOdometryState.x_mm, debugOdometryState.y_mm, debugOdometryState.heading_deg, &x_right, &y_right);

  /* Khi bật chế độ quét: STM32 quét và lưu vào ma trận 60x40 (3cm) độc lập, không ảnh hưởng web */
  if (isMappingModeActive) {
    Map_UpdateAllSensors(debugOdometryState.x_mm, debugOdometryState.y_mm, debugOdometryState.heading_deg,
                         x_left, y_left, LeftValid,
                         x_mid, y_mid, MidValid,
                         x_right, y_right, RightValid);
  }

  debugTelemetry.data1  = (int16_t)debugOdometryState.x_mm;
  debugTelemetry.data2  = (int16_t)debugOdometryState.y_mm;
  debugTelemetry.data3  = (int16_t)debugOdometryState.heading_deg;
  debugTelemetry.data4  = (int16_t)KalmanOdometry_GetAngularVelocity();
  debugTelemetry.data5  = (int16_t)x_left;
  debugTelemetry.data6  = (int16_t)y_left;
  debugTelemetry.data7  = (int16_t)x_mid;
  debugTelemetry.data8  = (int16_t)y_mid;
  debugTelemetry.data9  = (int16_t)x_right;
  debugTelemetry.data10 = (int16_t)y_right;
  debugTelemetry.data11 = (int16_t)debugLeftDistanceMm;
  debugTelemetry.data12 = (int16_t)debugMidDistanceMm;
  debugTelemetry.data13 = (int16_t)debugRightDistanceMm;
  /* Truyền tải CPU (x10). Nếu đang bật chế độ quét map, đổi dấu âm để Web & ESP32 nhận biết tức thời */
  debugTelemetry.data14 = isMappingModeActive ? (int16_t)(-(int16_t)cpuLoadTenthsPercent - 1) : (int16_t)cpuLoadTenthsPercent;
  debugTelemetry.data15 = Motion_GetFeedbackStatusWord();
}




int main(void) {
  /*
   * Khởi tạo ngoại vi rồi chạy điều khiển motor và telemetry; odometry
   */
  const uint32_t loopPeriodMs = 1000U / CONTROL_LOOP_FREQUENCY_HZ;
  const uint32_t telemetryPeriodMs = 1000U / UART_TELEMETRY_FREQUENCY_HZ;
  uint32_t nextLoopTick;
  uint32_t nextTelemetryTick;

  HAL_Init();
  Config_Init();
  Button_Init();

  /* Khởi tạo hệ thống LED */
  Led_Init();

  /*
   * Vòng lặp 3 giây chờ xe ổn định khi bật nguồn:
   * Cho phép người dùng đặt xe đứng yên trên sàn phẳng trước khi hiệu chuẩn Gyro.
   * - Pha 1: LED 1 nhấp nháy nhanh 3 phát (80ms sáng, 80ms tắt).
   * - Pha 2: LED 1 nhấp nháy chậm liên tục (250ms sáng, 250ms tắt) đến hết 3s.
   * - Kết thúc: LED 1 sáng liên tục báo hiệu bắt đầu khởi tạo phần cứng & cali Gyro.
   */
  {
    uint32_t startupTick = HAL_GetTick();
    uint8_t fastBlinkDone = 0U;

    /* Nháy nhanh 3 phát (80ms ON, 80ms OFF), pause 5s để không tự lặp lại */
    Led_SetPattern(1U, 3U, 80U, 80U, 5000U);

    while ((HAL_GetTick() - startupTick) < 3000U) {
      if (fastBlinkDone == 0U && (HAL_GetTick() - startupTick) >= 480U) {
        /* Chuyển sang nhấp nháy chậm liên tục */
        Led_SetPattern(1U, 1U, 250U, 500U, 0U);
        fastBlinkDone = 1U;
      }
      Led_Update();
      HAL_Delay(5U);
    }

    /* Sau 3s: LED 1 sáng liên tục */
    Led_SetPattern(1U, 1U, 100U, 0U, 0U);
    Led_Update();
  }

  /* Khởi tạo bản đồ 60x40 độ phân giải 3cm với 17 cản tĩnh và vùng cấm chữ U */
  Map_Init(0, 0);


  /* Khởi tạo các ngoại vi và cảm biến */
  if (Encoder_Init() != HAL_OK) {
    Error_Handler();
  }
  if (Mpu6500_Init() != HAL_OK) {
    Error_Handler();
  }
  if (Odometry_Init() != HAL_OK) {
    Error_Handler();
  }
  if (Config_OdometryTimer_Init() != HAL_OK) {
    Error_Handler();
  }
  I2cBus_SetYieldCallback(I2cYield_ProcessOdometry);

  if (MotorPwm_Init() != HAL_OK) {
    Error_Handler();
  }
  Motion_Init();
  if (UartProtocol_Init() != HAL_OK) {
    Error_Handler();
  }
  if (TofSensors_Init() != HAL_OK) {
    Error_Handler();
  }

  /* Bật DWT CYCCNT để đo thời gian xử lý và tải CPU bằng số chu kỳ CPU. */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  /* Đảm bảo ITM và Trace không can thiệp vào chân PB3 (TRACESWO) */
  ITM->TCR = 0U;
  ITM->TER = 0U;
  DBGMCU->CR &= ~(DBGMCU_CR_TRACE_IOEN | DBGMCU_CR_TRACE_MODE);
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
 
  odometryUpdatePending = 0U;
  cpuBusyCyclesWindow = 0U;
  cpuLoadTenthsPercent = 0U;
  /* Bắt đầu nhịp TIM3 */
  Config_OdometryTimer_Start();

  /* Khởi tạo deadline tuyệt đối để các task giữ được tần số đã cấu hình. */
  uint32_t startTick = HAL_GetTick();
  cpuLoadWindowStartTick = startTick;
  nextLoopTick = startTick + loopPeriodMs;
  nextTelemetryTick = startTick + telemetryPeriodMs;





  while (1) {
    uint32_t workStartCycles = DWT->CYCCNT;
    uint32_t now = HAL_GetTick();
    uint8_t taskRan = 0U;

    Led_Update();
    Button_Update();

    /* Ấn giữ Nút 1 để bật / tắt chế độ quét lưu map (LED 2 sáng / tắt) */
    if (b1.hold()) {
      isMappingModeActive = !isMappingModeActive;
      if (isMappingModeActive) {
        /* Bật chế độ quét: Bật LED 2 sáng liên tục */
        Led_SetPattern(2U, 1U, 1000U, 0U, 0U);
      } else {
        /* Tắt chế độ quét: Tắt LED 2 */
        Led_SetPattern(2U, 0U, 0U, 0U, 0U);
      }
    }

    /* Kiểm tra cờ timer odometry */
    uint32_t primask = __get_PRIMASK();
    uint16_t odometryDueTicks = 0U;
    __disable_irq();
    if (odometryUpdatePending != 0U) {
      odometryDueTicks = odometryUpdatePending;
      odometryUpdatePending = 0U;
    }
    if (primask == 0U) {
      __enable_irq();
    }

    /* Xử lý odometry trong main context, không làm phép toán nặng trong ISR. */
    if (odometryDueTicks != 0U) {
      Mpu6500_Update();
      if (Odometry_Update(odometryDueTicks) != HAL_OK) {
        /* [FIX 1.1] Lỗi odometry: bỏ qua chu kỳ này, không treo xe */
      }
      taskRan = 1U;
    }

    /* Chạy xử lý lệnh motor theo deadline, không phụ thuộc nhịp odometry. */
    if ((int32_t)(now - nextLoopTick) >= 0) {
      loop();
      nextLoopTick += loopPeriodMs;
      /* Nếu bị trễ hơn một chu kỳ thì đặt lại deadline để tránh chạy bù dồn dập. */
      if ((int32_t)(now - nextLoopTick) >= 0) {
        nextLoopTick = now + loopPeriodMs;
      }
      taskRan = 1U;
    }

    /* Đóng gói trạng thái odometry và hai trường dữ liệu ứng dụng để gửi UART. */
    if ((int32_t)(now - nextTelemetryTick) >= 0) {
      if (Odometry_GetState(&debugOdometryState) != HAL_OK) {
        Error_Handler();
      }
      if (!TofSensors_UpdateDistances(&debugLeftDistanceMm, &debugMidDistanceMm, &debugRightDistanceMm, &LeftValid, &MidValid, &RightValid)) {
        /* [FIX 1.1] Lỗi ToF: bỏ qua chu kỳ, không gọi Error_Handler để tránh treo xe */
      }
      debug();

      if (UartProtocol_SendTelemetry(&debugTelemetry) != HAL_OK) {
        /* [FIX 1.1] UART TX bận: bỏ qua gói này, không gọi Error_Handler */
      }

      nextTelemetryTick += telemetryPeriodMs;
      /* Bỏ qua deadline lỡ nhịp thay vì phát liên tiếp nhiều packet bù. */
      if ((int32_t)(now - nextTelemetryTick) >= 0) {
        nextTelemetryTick = now + telemetryPeriodMs;
      }
      taskRan = 1U;
    }

    /*
     * Chốt thời điểm trước WFI để thời gian ngủ không tính là CPU bận.
     * Chênh lệch CYCCNT của lượt này được cộng vào bộ tích lũy tải CPU.
     * Đây là ước lượng theo thời gian vòng chính hoạt động; thời gian ngủ WFI
     * bị loại trừ, còn ngắt xảy ra trong lúc xử lý được tính vào khoảng đo.
     */
    uint32_t workEndCycles = DWT->CYCCNT;
    cpuBusyCyclesWindow += workEndCycles - workStartCycles;
    uint32_t loadWindowElapsedMs = HAL_GetTick() - cpuLoadWindowStartTick;
    if (loadWindowElapsedMs >= 1000U) {
      uint64_t windowCycles = ((uint64_t)SystemCoreClock * loadWindowElapsedMs) / 1000U;
      uint64_t loadPercent = windowCycles != 0U ? ((uint64_t)cpuBusyCyclesWindow * 1000U + windowCycles / 2U) / windowCycles : 0U;
      cpuLoadTenthsPercent = loadPercent > 1000U ? 1000U : (uint16_t)loadPercent;
      cpuBusyCyclesWindow = 0U;
      cpuLoadWindowStartTick = HAL_GetTick();
    }

    if (taskRan == 0U) {
      /*
       * Ngăn race giữa kiểm tra cờ và WFI: nếu TIM3 đã đặt cờ thì không ngủ,
       * nếu chưa thì ngắt TIM3 sẽ đánh thức CPU khi tới hạn odometry.
       */
      uint32_t sleepPrimask = __get_PRIMASK();
      __disable_irq();
      if (odometryUpdatePending == 0U) {
        __WFI();
      }
      if (sleepPrimask == 0U) {
        __enable_irq();
      }
    }
  }
}

void Error_Handler(void) {
  /* [FIX 1.1] Dừng khẩn cấp động cơ ngay lập tức trước khi vào vòng lặp lỗi */
  Motion_Stop();
  Led_SetPattern(1U, 1U, 50U, 50U, 50U);
  while (1) {
    Led_Update();
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {
  /* Có thể đặt breakpoint tại đây để xem file và dòng gây lỗi assert. */
}
#endif
