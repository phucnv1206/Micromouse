#ifndef MICROMOUSE_ENCODER_H
#define MICROMOUSE_ENCODER_H

#include "stm32f4xx_hal.h"

/* Chỉ số hai encoder; thứ tự này cũng là thứ tự trong Encoder_Snapshot. */
typedef enum {
  ENCODER_LEFT,
  ENCODER_RIGHT,
  ENCODER_MOTOR_COUNT
} Encoder_Motor;

typedef struct {
  /* Bộ đếm quadrature dạng vòng, mỗi bánh có giá trị từ 0 đến ENCODER_COUNT_MAX. */
  uint16_t count[ENCODER_MOTOR_COUNT];
} Encoder_Snapshot;

/*
 * Khởi tạo trạng thái quadrature từ mức logic hiện tại trên hai pha A/B,
 * đồng thời đặt bộ đếm của mỗi bánh về 0.
 */
HAL_StatusTypeDef Encoder_Init(void);
/*
 * Sao chép bộ đếm trái/phải thành một snapshot nhất quán, không để ngắt EXTI
 * thay đổi bộ đếm giữa lúc đọc. Nếu ENCODER_ENABLED=0 thì snapshot bằng 0.
 * Trả HAL_ERROR nếu con trỏ snapshot là NULL.
 */
HAL_StatusTypeDef Encoder_GetSnapshot(Encoder_Snapshot *snapshot);

#endif
