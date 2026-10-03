#include "main.h"
#include "motor_pwm.h"
#include "encoder.h"
#include "odometry.h"
#include "uart_protocol.h"
#include "led.h"
#include "button.h"
#include "tof_sensors.h"


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
#if ODOMETRY_ENABLED
/* Cờ báo timer odometry vừa đến hạn; volatile vì được cập nhật trong ngắt TIM3. */
static volatile uint8_t odometryUpdatePending;
/* Ngắt TIM3 chạy theo tần số odometry và chỉ báo công việc cho vòng lặp chính. */
void TIM3_IRQHandler(void) {
  if ((TIM3->SR & TIM_SR_UIF) != 0U) {
    /* Xóa cờ cập nhật để timer có thể phát sinh ngắt ở chu kỳ kế tiếp. */
    TIM3->SR &= ~TIM_SR_UIF;
    /* Vòng chính đọc và xóa cờ này rồi mới thực hiện Odometry_Update(). */
    odometryUpdatePending = 1U;
  }
}
#endif


/*
 * Vòng điều khiển 100 Hz: lấy hết các frame UART đã xếp hàng, giữ lệnh hợp lệ
 * mới nhất rồi cập nhật PWM phải/trái. HAL_BUSY có nghĩa là hàng đợi đã hết;
 * lỗi nhận khác được chuyển sang Error_Handler().
 */

static void readUart(void){
  UartProtocol_RxFrame rxFrame;
  int16_t motorRightCommand = 0;
  int16_t motorLeftCommand = 0;
  uint8_t commandReceived = 0U;
  HAL_StatusTypeDef receiveStatus;
  do {
    receiveStatus = UartProtocol_Receive(&rxFrame);
    if (receiveStatus == HAL_OK) {
      /* Lệnh ngoài -100..100 được MotorPwm_Set() bão hòa về giới hạn gần nhất. */
      motorRightCommand = rxFrame.data1;
      motorLeftCommand = rxFrame.data2;
      commandReceived = 1U;
    } else if (receiveStatus != HAL_BUSY) {
      Error_Handler();
    }
  } while (receiveStatus == HAL_OK);
  if (commandReceived != 0U) {
    RunMotorRight(motorRightCommand);
    RunMotorLeft(motorLeftCommand);
  }
}



static void loop(void) {
  readUart();
  /* code here*/


  
}



static void debug(void) {
  
#if ODOMETRY_ENABLED
  debugTelemetry.data1 = (int16_t)debugOdometryState.x_mm;
  debugTelemetry.data2 = (int16_t)debugOdometryState.y_mm;
  debugTelemetry.data3 = (int16_t)debugOdometryState.heading_deg;
#else
  debugTelemetry.data1 = 0;
  debugTelemetry.data2 = 0;
  debugTelemetry.data3 = 0;
#endif
  debugTelemetry.data4 = (int16_t)debugMidDistanceMm;
  debugTelemetry.data5 = (int16_t)debugLeftDistanceMm;
  debugTelemetry.data6 = (int16_t)debugRightDistanceMm;
  debugTelemetry.data7 = (int16_t)cpuLoadTenthsPercent;
}




int main(void) {
  /*
   * Khởi tạo ngoại vi rồi chạy điều khiển motor và telemetry; odometry
   * chỉ được bật khi ODOMETRY_ENABLED=1.
   */
  const uint32_t loopPeriodMs = 1000U / CONTROL_LOOP_FREQUENCY_HZ;
  const uint32_t telemetryPeriodMs = 1000U / UART_TELEMETRY_FREQUENCY_HZ;
  uint32_t nextLoopTick;
  uint32_t nextTelemetryTick;

  HAL_Init();
  Config_Init();
  Button_Init();

  /* Cấu hình pattern LED test; Led_Update() được gọi định kỳ bên dưới. */
  Led_Init();
  // Led_SetPattern(1U, 1U, 100U, 0U, 100U);
  // Led_SetPattern(2U, 2U, 100U, 0U, 100U);
  // Led_SetPattern(3U, 3U, 100U, 0U, 100U);


  /* Chỉ khởi tạo encoder khi được bật; odometry yêu cầu encoder hoạt động. */
#if ENCODER_ENABLED
  if (Encoder_Init() != HAL_OK) {
    Error_Handler();
  }
#endif
#if ODOMETRY_ENABLED
  if (Odometry_Init() != HAL_OK) {
    Error_Handler();
  }
  /* Chỉ bật timer khi odometry được cấu hình. */
  if (Config_OdometryTimer_Init() != HAL_OK) {
    Error_Handler();
  }
#endif
  if (MotorPwm_Init() != HAL_OK) {
    Error_Handler();
  }
  if (UartProtocol_Init() != HAL_OK) {
    Error_Handler();
  }
  /* Chỉ khởi tạo ToF và I2C khi bật cấu hình cảm biến. */
#if TOF_ENABLED
  TofSensors_Init();
#endif

  /* Bật DWT CYCCNT để đo thời gian xử lý và tải CPU bằng số chu kỳ CPU. */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

#if ODOMETRY_ENABLED
  odometryUpdatePending = 0U;
#endif
  cpuBusyCyclesWindow = 0U;
  cpuLoadTenthsPercent = 0U;
  /* Bắt đầu nhịp TIM3 chỉ khi odometry được bật. */
#if ODOMETRY_ENABLED
  Config_OdometryTimer_Start();
#endif

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

    /* Chỉ kiểm tra cờ timer khi odometry được bật. */
#if ODOMETRY_ENABLED
    uint32_t primask = __get_PRIMASK();
    uint8_t odometryDue = 0U;
    __disable_irq();
    if (odometryUpdatePending != 0U) {
      odometryUpdatePending = 0U;
      odometryDue = 1U;
    }
    if (primask == 0U) {
      __enable_irq();
    }

    /* Xử lý odometry trong main context, không làm phép toán nặng trong ISR. */
    if (odometryDue != 0U) {
      if (Odometry_Update() != HAL_OK) {
        Error_Handler();
      }
      taskRan = 1U;
    }
#endif

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
#if ODOMETRY_ENABLED
      if (Odometry_GetState(&debugOdometryState) != HAL_OK) {
        Error_Handler();
      }
#endif
#if TOF_ENABLED
      if (!TofSensors_UpdateDistances(&debugLeftDistanceMm, &debugMidDistanceMm,
                                      &debugRightDistanceMm)) {
        Error_Handler();
      }
#endif
      debug();

      if (UartProtocol_SendTelemetry(&debugTelemetry) != HAL_OK) {
        Error_Handler();
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
#if ODOMETRY_ENABLED
      if (odometryUpdatePending == 0U) {
        __WFI();
      }
#else
      __WFI();
#endif
      if (sleepPrimask == 0U) {
        __enable_irq();
      }
    }
  }
}

void Error_Handler(void) {
  Led_SetPattern(1U, 1U, 100U, 100U, 100U);
  while (1) {
    Led_Update();
    
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {
  /* Có thể đặt breakpoint tại đây để xem file và dòng gây lỗi assert. */
}
#endif
