#ifndef MICROMOUSE_LED_H
#define MICROMOUSE_LED_H

#include "stm32f4xx_hal.h"

/* Số LED do module điều khiển; Led_SetPattern nhận số thứ tự từ 1 đến 3. */
#define LED_COUNT 3U

/* Xóa trạng thái pattern và tắt cả ba LED. */
void Led_Init(void);
/*
 * Đặt pattern nhấp nháy cho LED số led_number:
 * count là số lần sáng trong một chu kỳ; on_ms/off_ms là thời gian sáng/tắt
 * mỗi nhịp; pause_ms là thời gian nghỉ sau khi hoàn tất count lần nháy.
 * count=0 hoặc on_ms=0 sẽ tắt LED; off_ms=0 làm LED sáng liên tục.
 * LED1 active-low được xử lý nội bộ, nên API có cùng logic cho cả ba LED.
 * Trả HAL_ERROR nếu số LED nằm ngoài 1..LED_COUNT.
 */
HAL_StatusTypeDef Led_SetPattern(uint8_t led_number, uint16_t count, uint16_t on_ms, uint16_t off_ms, uint16_t pause_ms);
/* Cập nhật output GPIO theo thời gian hiện tại; gọi lặp thường xuyên, không chặn. */
void Led_Update(void);

#endif
