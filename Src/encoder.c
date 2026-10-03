#include "main.h"
#include "encoder.h"
#include "config.h"

#if ENCODER_ENABLED

/*
 * Bảng giải mã quadrature: index = (trạng thái AB trước << 2) | AB hiện tại.
 * Hai pha A/B lệch pha nhau; chuyển sang trạng thái kế tiếp cộng 1, chuyển
 * ngược lại trừ 1. Không đổi trạng thái hoặc nhảy hai bit cùng lúc thì bỏ qua.
 */
static const int8_t transition_delta[16] = {
   0,  1, -1,  0,
  -1,  0,  0,  1,
   1,  0,  0, -1,
   0, -1,  1,  0
};

/* Dữ liệu ISR EXTI và hàm chính cùng truy cập nên phải khai báo volatile. */
static volatile uint8_t previous_state[ENCODER_MOTOR_COUNT];
static volatile uint8_t state_initialized[ENCODER_MOTOR_COUNT];
static volatile uint16_t encoder_count[ENCODER_MOTOR_COUNT];

static uint8_t Encoder_ReadState(uint32_t motor) {
  uint32_t state = 0U;

  /* Đóng gói mức logic A vào bit 1 và B vào bit 0 để tra bảng chuyển trạng thái. */
  if (motor == ENCODER_LEFT) {
    if ((EN_LEFT_A_Port->IDR & EN_LEFT_A) != 0U) {
      state |= 2U;
    }
    if ((EN_LEFT_B_Port->IDR & EN_LEFT_B) != 0U) {
      state |= 1U;
    }
  } else {
    if ((EN_RIGHT_A_Port->IDR & EN_RIGHT_A) != 0U) {
      state |= 2U;
    }
    if ((EN_RIGHT_B_Port->IDR & EN_RIGHT_B) != 0U) {
      state |= 1U;
    }
  }

  return (uint8_t)state;
}

HAL_StatusTypeDef Encoder_Init(void) {
  uint32_t primask = __get_PRIMASK();

  /*
   * Khóa ngắt trong lúc đồng bộ trạng thái ban đầu và xóa count.
   * Lưu PRIMASK để chỉ bật lại ngắt nếu trước đó chúng đang được bật.
   */
  __disable_irq();
  for (uint32_t i = 0U; i < ENCODER_MOTOR_COUNT; i++) {
    previous_state[i] = Encoder_ReadState(i);
    state_initialized[i] = 1U;
    encoder_count[i] = 0U;
  }

  if (primask == 0U) {
    __enable_irq();
  }

  return HAL_OK;
}

HAL_StatusTypeDef Encoder_GetSnapshot(Encoder_Snapshot *snapshot) {
  uint32_t primask;

  if (snapshot == NULL) {
    return HAL_ERROR;
  }

  /* Đọc nguyên tử cả hai count: ISR không thể chen giữa hai lần đọc. */
  primask = __get_PRIMASK();
  __disable_irq();
  for (uint32_t i = 0U; i < ENCODER_MOTOR_COUNT; i++) {
    snapshot->count[i] = encoder_count[i];
  }
  if (primask == 0U) {
    __enable_irq();
  }

  return HAL_OK;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  uint32_t motor;
  uint8_t current;
  uint8_t old_state;
  int8_t delta;

  /* Xác định bánh phát sinh ngắt; sau đó đọc cả A và B để giải mã chiều quay. */
  switch (GPIO_Pin) {
    case EN_LEFT_A:
      motor = ENCODER_LEFT;
      break;
    case EN_LEFT_B:
      motor = ENCODER_LEFT;
      break;
    case EN_RIGHT_A:
      motor = ENCODER_RIGHT;
      break;
    case EN_RIGHT_B:
      motor = ENCODER_RIGHT;
      break;
    default:
      return;
  }

  if (state_initialized[motor] == 0U) {
    return;
  }

  /* Bỏ qua nếu chưa khởi tạo, không đổi trạng thái, hoặc có chuyển đổi không hợp lệ. */
  current = Encoder_ReadState(motor);
  old_state = previous_state[motor];
  if (current == old_state) {
    return;
  }
  previous_state[motor] = current;

  delta = transition_delta[(old_state << 2U) | current];
  if (delta == 0) {
    return;
  }

  /*
   * Chỉ cập nhật bộ đếm trong ISR để thời gian xử lý ngắt ngắn.
   * Bộ đếm chạy vòng trong [0, ENCODER_COUNT_MAX]; odometry sẽ tự xử lý tràn.
   */
  if (delta > 0) {
    encoder_count[motor] = encoder_count[motor] >= ENCODER_COUNT_MAX ? 0U : encoder_count[motor] + 1U;
  } else {
    encoder_count[motor] = encoder_count[motor] == 0U ? ENCODER_COUNT_MAX : encoder_count[motor] - 1U;
  }
}

#else

HAL_StatusTypeDef Encoder_Init(void) {
  /* API rỗng để phần còn lại của firmware vẫn khởi tạo được khi encoder bị tắt. */
  return HAL_OK;
}

HAL_StatusTypeDef Encoder_GetSnapshot(Encoder_Snapshot *snapshot) {
  if (snapshot == NULL) {
    return HAL_ERROR;
  }

  /* Trả snapshot 0 để phía gọi không cần nhánh riêng khi tắt encoder. */
  for (uint32_t i = 0U; i < ENCODER_MOTOR_COUNT; i++) {
    snapshot->count[i] = 0U;
  }

  return HAL_OK;
}

#endif
