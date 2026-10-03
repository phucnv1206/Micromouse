#include "led.h"
#include "config.h"

typedef struct {
  uint16_t count;       /* Số nhịp sáng trong một chu kỳ pattern. */
  uint16_t on_ms;       /* Thời gian LED sáng ở mỗi nhịp. */
  uint16_t off_ms;      /* Thời gian LED tắt giữa các nhịp. */
  uint16_t pause_ms;    /* Thời gian nghỉ sau khi hoàn tất một chuỗi nháy. */
  uint16_t flashes;     /* Số nhịp đã bắt đầu trong chu kỳ hiện tại. */
  uint32_t time_line;   /* Mốc HAL tick của trạng thái hiện tại. */
  uint8_t changed;      /* Báo pattern mới cần khởi chạy lại từ đầu. */
  uint8_t bright;       /* 1 khi đang ở pha sáng, 0 khi đang ở pha tắt. */
  uint8_t paused;       /* 1 khi đã nháy đủ số lần và đang nghỉ. */
  uint8_t new_period;   /* Báo cần bắt đầu chu kỳ nháy mới. */
} LedPattern;

/* Mỗi LED có state machine và lịch nháy độc lập. */
static LedPattern patterns[LED_COUNT];

/* Đặt tất cả state machine về ban đầu và đảm bảo các chân LED đang tắt. */
void Led_Init(void) {
  for (uint8_t index = 0U; index < LED_COUNT; index++) {
    patterns[index] = (LedPattern){0};
  }

  /* LED1 trên PC13 active-low; mức SET là trạng thái tắt. */
  HAL_GPIO_WritePin(LED1_Port, LED1, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED2_Port, LED2, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED3_Port, LED3, GPIO_PIN_RESET);
}

HAL_StatusTypeDef Led_SetPattern(uint8_t led_number, uint16_t count, uint16_t on_ms, uint16_t off_ms, uint16_t pause_ms) {
  LedPattern *pattern;

  if (led_number < 1U || led_number > LED_COUNT) {
    return HAL_ERROR;
  }

  /* Chỉ đánh dấu thay đổi khi thông số khác để không restart pattern vô cớ. */
  pattern = &patterns[led_number - 1U];
  if (pattern->count != count || pattern->on_ms != on_ms || pattern->off_ms != off_ms || pattern->pause_ms != pause_ms) {
    pattern->count = count;
    pattern->on_ms = on_ms;
    pattern->off_ms = off_ms;
    pattern->pause_ms = pause_ms;
    pattern->changed = 1U;
  }

  return HAL_OK;
}

static GPIO_PinState Led_UpdatePattern(LedPattern *pattern, uint32_t now) {
  /* Không có nhịp hoặc thời gian sáng bằng 0 nghĩa là pattern tắt. */
  if (pattern->count == 0U || pattern->on_ms == 0U) {
    pattern->bright = 0U;
    pattern->paused = 0U;
    pattern->new_period = 1U;
    return GPIO_PIN_RESET;
  }

  /* Không có thời gian tắt thì xem như LED sáng liên tục. */
  if (pattern->off_ms == 0U) {
    pattern->bright = 1U;
    pattern->paused = 0U;
    pattern->new_period = 0U;
    return GPIO_PIN_SET;
  }

  /* Cấu hình mới bắt đầu lại chuỗi nháy từ nhịp đầu tiên. */
  if (pattern->changed != 0U) {
    pattern->new_period = 1U;
    pattern->paused = 0U;
    pattern->changed = 0U;
  }

  /* Máy trạng thái: bắt đầu chu kỳ, nghỉ, sáng, hoặc chờ hết thời gian tắt. */
  if (pattern->new_period != 0U) {
    pattern->new_period = 0U;
    pattern->flashes = 1U;
    pattern->time_line = now;
    pattern->bright = 1U;
  } else if (pattern->paused != 0U) {
    if ((uint32_t)(now - pattern->time_line) >= pattern->pause_ms) {
      pattern->paused = 0U;
      pattern->new_period = 1U;
    }
  } else if (pattern->bright != 0U) {
    if ((uint32_t)(now - pattern->time_line) >= pattern->on_ms) {
      if (pattern->flashes == pattern->count) {
        pattern->paused = 1U;
      }
      pattern->bright = 0U;
      pattern->time_line = now;
    }
  } else if ((uint32_t)(now - pattern->time_line) >= pattern->off_ms) {
    pattern->flashes++;
    pattern->bright = 1U;
    pattern->time_line = now;
  }

  return pattern->paused == 0U && pattern->bright != 0U ? GPIO_PIN_SET : GPIO_PIN_RESET;
}

void Led_Update(void) {
  uint32_t now = HAL_GetTick();
  GPIO_PinState led1State = Led_UpdatePattern(&patterns[0], now);

  /* Cập nhật từng LED riêng biệt; hàm không delay nên không chặn task khác. */
  HAL_GPIO_WritePin(LED1_Port, LED1, led1State == GPIO_PIN_SET ? GPIO_PIN_RESET : GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED2_Port, LED2, Led_UpdatePattern(&patterns[1], now));
  HAL_GPIO_WritePin(LED3_Port, LED3, Led_UpdatePattern(&patterns[2], now));
}
