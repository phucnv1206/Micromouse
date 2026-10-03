#ifndef MICROMOUSE_BUTTON_H
#define MICROMOUSE_BUTTON_H

#include <stdbool.h>
#include "stm32f4xx_hal.h"

typedef struct {
  /* Trả true đúng một lần cho mỗi click/hold chưa được đọc, sau đó xóa sự kiện. */
  bool (*click)(void);
  bool (*hold)(void);
} Button;

extern Button b1;
extern Button b2;

/* Khởi tạo trạng thái nút; GPIO phải được cấu hình trước bằng Config_Init(). */
void Button_Init(void);
/* Cập nhật chống dội và nhận diện sự kiện; cần được gọi thường xuyên trong main. */
void Button_Update(void);

#endif
