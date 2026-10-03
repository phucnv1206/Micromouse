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

/* Payload UART là int16 big-endian lưu dưới dạng bit pattern uint16. */
static int16_t DecodeMotorCommand(uint16_t encodedCommand) {
  int32_t signedCommand = encodedCommand <= INT16_MAX ? (int32_t)encodedCommand : (int32_t)encodedCommand - 65536;

  return (int16_t)signedCommand;
}

/*
 * Đổi số có dấu thành uint16 theo quy ước ESP32:
 * giải mã bằng encoded - 32767. Giá trị được làm tròn về số nguyên gần nhất
 * và chặn ở biên biểu diễn để không tràn khi đóng gói UART.
 */
static uint16_t EncodeSignedTelemetryValue(float value) {
  int32_t signedValue;

  /* ESP32 giải mã giá trị mm và mm/s có dấu bằng cách lấy mã hóa trừ 32767. */
  if (value <= -32767.0f) {
    return 0U;
  } else if (value >= 32768.0f) {
    return 65535U;
  }

  signedValue = (int32_t)(value >= 0.0f ? value + 0.5f : value - 0.5f);
  return (uint16_t)(signedValue + 32767);
}

/*
 * Vòng điều khiển 100 Hz: lấy hết các frame UART đã xếp hàng, giữ lệnh hợp lệ
 * mới nhất rồi cập nhật PWM phải/trái. HAL_BUSY có nghĩa là hàng đợi đã hết;
 * lỗi nhận khác được chuyển sang Error_Handler().
 */
static void loop(void) {
  uint8_t rxFrame[UART_PROTOCOL_RX_FRAME_SIZE];
  int16_t motorRightCommand = 0;
  int16_t motorLeftCommand = 0;
  uint8_t commandReceived = 0U;
  HAL_StatusTypeDef receiveStatus;


  /* Đọc hết frame đang chờ để không dùng lệnh cũ nếu queue có nhiều frame. */
  do {
    receiveStatus = UartProtocol_Receive(rxFrame);
    if (receiveStatus == HAL_OK) {
      uint16_t encodedRight = ((uint16_t)rxFrame[UART_PROTOCOL_DATA1_OFFSET] << 8U) | rxFrame[UART_PROTOCOL_DATA1_OFFSET + 1U];
      uint16_t encodedLeft = ((uint16_t)rxFrame[UART_PROTOCOL_DATA2_OFFSET] << 8U) | rxFrame[UART_PROTOCOL_DATA2_OFFSET + 1U];
      int16_t rightCommand = DecodeMotorCommand(encodedRight);
      int16_t leftCommand = DecodeMotorCommand(encodedLeft);

      /* Lệnh ngoài -100..100 được MotorPwm_Set() bão hòa về giới hạn gần nhất. */
      motorRightCommand = rightCommand;
      motorLeftCommand = leftCommand;
      commandReceived = 1U;
    } else if (receiveStatus != HAL_BUSY) {
      Error_Handler();
    }
  } while (receiveStatus == HAL_OK);
  if (commandReceived != 0U) {
    RunMotorRight(motorRightCommand - 100);
    RunMotorLeft(motorLeftCommand - 100);
  }
}

int main(void) {
  /*
   * Khởi tạo ngoại vi rồi chạy ba công việc độc lập:
   * odometry theo TIM3, lệnh motor theo CONTROL_LOOP_FREQUENCY_HZ,
   * telemetry theo UART_TELEMETRY_FREQUENCY_HZ.
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


  /* Khởi tạo encoder trước odometry để odometry lấy được snapshot ban đầu. */
  if (Encoder_Init() != HAL_OK) {

    Error_Handler();
  }
  if (Odometry_Init() != HAL_OK) {
    Error_Handler();
  }
  /* Khởi tạo bộ định thời phần cứng tạo nhịp cập nhật odometry. */
  if (Config_OdometryTimer_Init() != HAL_OK) {
    Error_Handler();
  }
  if (MotorPwm_Init() != HAL_OK) {
    
    Error_Handler();
  }
  if (UartProtocol_Init() != HAL_OK) {
    
    Error_Handler();
  }
  /* Khởi tạo ba cảm biến tuần tự; LED tương ứng sáng sau khi mỗi sensor init xong. */
  TofSensors_Init();

  /* Bật DWT CYCCNT để đo thời gian xử lý và tải CPU bằng số chu kỳ CPU. */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  odometryUpdatePending = 0U;
  cpuBusyCyclesWindow = 0U;
  cpuLoadTenthsPercent = 0U;
  /* Bắt đầu nhịp TIM3 sau khi mọi module cần thiết đã sẵn sàng. */
  Config_OdometryTimer_Start();

  /* Khởi tạo deadline tuyệt đối để các task giữ được tần số đã cấu hình. */
  uint32_t startTick = HAL_GetTick();
  cpuLoadWindowStartTick = startTick;
  nextLoopTick = startTick + loopPeriodMs;
  nextTelemetryTick = startTick + telemetryPeriodMs;

  while (1) {
    /* Bắt đầu đo phần xử lý của lượt này; đoạn ngủ WFI sẽ không được tính. */
    uint32_t workStartCycles = DWT->CYCCNT;
    uint32_t now = HAL_GetTick();
    uint8_t taskRan = 0U;

    Led_Update();
    Button_Update();

    /* Chỉ khóa ngắt trong lúc đọc/xóa cờ dùng chung với ISR TIM3. */
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
      Odometry_State state;
      UartProtocol_Telemetry telemetry;
      uint16_t leftDistanceMm;
      uint16_t midDistanceMm;
      uint16_t rightDistanceMm;

      if (Odometry_GetState(&state) != HAL_OK) {
        Error_Handler();
      }
      if (TofSensors_UpdateDistances(&leftDistanceMm, &midDistanceMm, &rightDistanceMm) != HAL_OK) {
        Error_Handler();
      }
      telemetry.x_mm_encoded = EncodeSignedTelemetryValue(state.x_mm);
      telemetry.y_mm_encoded = EncodeSignedTelemetryValue(state.y_mm);
      telemetry.heading_half_degrees = (uint16_t)(state.heading_deg * 0.5f + 0.5f) % 180U;
      /* Dùng tạm hai trường tốc độ để gửi khoảng cách trái/phải đã mã hóa signed, đơn vị mm. */
      telemetry.left_speed_mm_s_encoded = EncodeSignedTelemetryValue((float)leftDistanceMm);
      telemetry.right_speed_mm_s_encoded = EncodeSignedTelemetryValue((float)rightDistanceMm);
      /* Dùng tạm data1 để gửi khoảng cách cảm biến giữa, đơn vị mm. */
      telemetry.data1 = midDistanceMm;
      /* data2: tải CPU trung bình theo 0,1%; 0..1000 tương ứng 0..100,0%. */
      telemetry.data2 = cpuLoadTenthsPercent;

      if (UartProtocol_SendTelemetry(&telemetry) != HAL_OK) {
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
      uint64_t loadPercent = windowCycles != 0U
                                 ? ((uint64_t)cpuBusyCyclesWindow * 1000U + windowCycles / 2U) / windowCycles
                                 : 0U;

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
