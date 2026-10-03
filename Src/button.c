#include "main.h"
#include "button.h"

typedef enum {
  BUTTON_ID_1 = 0U,
  BUTTON_ID_2,
  BUTTON_ID_COUNT
} Button_Id;

#define BUTTON_EVENT_CLICK_FLAG (1U << 0U)
#define BUTTON_EVENT_HOLD_FLAG (1U << 1U)

typedef struct {
  GPIO_TypeDef *port;             /* Cổng GPIO của nút. */
  uint16_t pin;                   /* Chân GPIO tương ứng. */
  GPIO_PinState candidateState;   /* Mức thô đang chờ qua thời gian chống dội. */
  GPIO_PinState stableState;      /* Mức đã xác nhận ổn định sau chống dội. */
  uint32_t stateChangedTick;      /* Thời điểm mức thô đổi lần gần nhất. */
  uint32_t pressedTick;           /* Thời điểm bắt đầu trạng thái nhấn ổn định. */
  uint8_t pendingEvents;          /* Các sự kiện chưa được ứng dụng đọc. */
  uint8_t holdEventSent;          /* Tránh phát lặp sự kiện HOLD trong cùng lần nhấn. */
} Button_State;

static Button_State buttons[BUTTON_ID_COUNT] = {
  {BUTTON1_Port, BUTTON1, GPIO_PIN_SET, GPIO_PIN_SET, 0U, 0U, 0U, 0U},
  {BUTTON2_Port, BUTTON2, GPIO_PIN_SET, GPIO_PIN_SET, 0U, 0U, 0U, 0U}
};

static bool Button_ConsumeEvent(Button_Id button, uint8_t eventFlag) {
  if ((buttons[button].pendingEvents & eventFlag) == 0U) {
    return false;
  }

  buttons[button].pendingEvents &= (uint8_t)~eventFlag;
  return true;
}

static bool Button1_Click(void) {
  return Button_ConsumeEvent(BUTTON_ID_1, BUTTON_EVENT_CLICK_FLAG);
}

static bool Button1_Hold(void) {
  return Button_ConsumeEvent(BUTTON_ID_1, BUTTON_EVENT_HOLD_FLAG);
}

static bool Button2_Click(void) {
  return Button_ConsumeEvent(BUTTON_ID_2, BUTTON_EVENT_CLICK_FLAG);
}

static bool Button2_Hold(void) {
  return Button_ConsumeEvent(BUTTON_ID_2, BUTTON_EVENT_HOLD_FLAG);
}

Button b1 = {Button1_Click, Button1_Hold};
Button b2 = {Button2_Click, Button2_Hold};

/*
 * Đọc mức chân và chỉ xác nhận thay đổi khi mức mới ổn định đủ thời gian debounce.
 * Với điện trở kéo lên, chân SET là nhả nút và RESET là đang nhấn.
 */
static uint8_t Button_ReadStableState(Button_State *button, uint32_t now) {
  GPIO_PinState rawState = HAL_GPIO_ReadPin(button->port, button->pin);

  if (rawState != button->candidateState) {
    button->candidateState = rawState;
    button->stateChangedTick = now;
  }

  if (button->candidateState != button->stableState && (uint32_t)(now - button->stateChangedTick) >= BUTTON_DEBOUNCE_MS) {
    button->stableState = button->candidateState;
    return 1U;
  }

  return 0U;
}

void Button_Init(void) {
  uint32_t now = HAL_GetTick();

  for (uint32_t i = 0U; i < BUTTON_ID_COUNT; i++) {
    GPIO_PinState initialState = HAL_GPIO_ReadPin(buttons[i].port, buttons[i].pin);

    buttons[i].candidateState = initialState;
    buttons[i].stableState = initialState;
    buttons[i].stateChangedTick = now;
    buttons[i].pressedTick = now;
    buttons[i].pendingEvents = 0U;
    buttons[i].holdEventSent = 0U;
  }
}

void Button_Update(void) {
  uint32_t now = HAL_GetTick();

  for (uint32_t i = 0U; i < BUTTON_ID_COUNT; i++) {
    Button_State *button = &buttons[i];
    uint8_t stateChanged = Button_ReadStableState(button, now);

    if (stateChanged != 0U) {
      if (button->stableState == GPIO_PIN_RESET) {
        /* Bắt đầu lần nhấn mới; bộ đếm giữ tính từ lúc nhấn đã qua debounce. */
        button->pressedTick = now;
        button->holdEventSent = 0U;
      } else if (button->holdEventSent == 0U) {
        /* Click chỉ phát khi nhả; nếu ngưỡng hold vừa qua trong lúc debounce nhả,
         * vẫn phân loại lần nhấn đó là HOLD thay vì phát nhầm CLICK. */
        button->pendingEvents |= (uint32_t)(now - button->pressedTick) >= BUTTON_HOLD_TIME_MS ? BUTTON_EVENT_HOLD_FLAG : BUTTON_EVENT_CLICK_FLAG;
      }
    }

    if (button->stableState == GPIO_PIN_RESET && button->holdEventSent == 0U && (uint32_t)(now - button->pressedTick) >= BUTTON_HOLD_TIME_MS) {
      /* Phát HOLD khi vẫn đang nhấn và chỉ phát đúng một lần cho mỗi lần nhấn. */
      button->pendingEvents |= BUTTON_EVENT_HOLD_FLAG;
      button->holdEventSent = 1U;
    }
  }
}
