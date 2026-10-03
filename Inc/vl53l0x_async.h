#ifndef MICROMOUSE_VL53L0X_ASYNC_H
#define MICROMOUSE_VL53L0X_ASYNC_H

#include <stdbool.h>
#include "stm32f4xx_hal.h"

/*
 * Bắt đầu luân phiên polling ba cảm biến đã được TofSensors_Init() khởi tạo.
 * Cảm biến phải đang đo liên tục; hàm không tự khởi tạo hoặc cấu hình I2C.
 */
HAL_StatusTypeDef Vl53l0xAsync_Start(void);
/*
 * Gọi thường xuyên trong main loop. Hàm chỉ khởi động/kiểm tra giao dịch DMA,
 * không chờ phép đo hoặc bus I2C; ba cảm biến được thăm dò tuần tự trên bus chung.
 */
void Vl53l0xAsync_Process(void);
/*
 * Lấy khoảng cách hợp lệ mới nhất đã lọc của từng cảm biến đúng một lần.
 * Trả true khi *distance được cập nhật; khoảng cách có đơn vị mm.
 */
bool getDisLeft(uint16_t *dis_left);
bool getDisMid(uint16_t *dis_mid);
bool getDisRight(uint16_t *dis_right);
/* Trạng thái HAL gần nhất; lỗi vẫn có thể được đọc trong khi scheduler tiếp tục. */
HAL_StatusTypeDef Vl53l0xAsync_GetStatus(void);
/* Dừng polling mà không chờ; trả HAL_BUSY nếu một giao dịch DMA còn hoạt động. */
HAL_StatusTypeDef Vl53l0xAsync_Stop(void);

#endif
