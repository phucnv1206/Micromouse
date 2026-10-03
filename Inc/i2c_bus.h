#ifndef MICROMOUSE_I2C_BUS_H
#define MICROMOUSE_I2C_BUS_H

#include "stm32f4xx_hal.h"

/*
 * Lớp giao tiếp chung cho I2C1; địa chỉ thiết bị truyền vào luôn là địa chỉ 7-bit
 * chưa dịch. Các hàm truyền/nhận và đọc/ghi thanh ghi khởi động DMA bất đồng bộ:
 * HAL_OK nghĩa là đã khởi động, không có nghĩa là giao dịch đã hoàn tất.
 * Bộ đệm phải còn hợp lệ và không được sửa/đọc dữ liệu nhận cho đến khi
 * I2cBus_GetTransferStatus() trả HAL_OK hoặc báo lỗi. Không gọi các hàm này từ ISR.
 */
/* Kiểm tra I2C1 đã được khởi tạo bởi Config_Init(); không dò thiết bị khi khởi động. */
HAL_StatusTypeDef I2cBus_Init(void);
/* Dò ACK đồng bộ; hàm này blocking tối đa I2C_TIMEOUT_MS. */
HAL_StatusTypeDef I2cBus_IsDeviceReady(uint8_t address, uint32_t trials);
/* Bắt đầu gửi/nhận dữ liệu thô bằng DMA tới địa chỉ 7-bit chưa dịch bit. */
HAL_StatusTypeDef I2cBus_Transmit(uint8_t address, const uint8_t *data, uint16_t size);
HAL_StatusTypeDef I2cBus_Receive(uint8_t address, uint8_t *data, uint16_t size);
/*
 * Bắt đầu đọc/ghi vùng nhớ hoặc thanh ghi bằng DMA;
 * regSize là I2C_MEMADD_SIZE_8BIT/16BIT.
 */
HAL_StatusTypeDef I2cBus_MemRead(uint8_t address, uint16_t reg, uint16_t regSize, uint8_t *data, uint16_t size);
HAL_StatusTypeDef I2cBus_MemWrite(uint8_t address, uint16_t reg, uint16_t regSize, const uint8_t *data, uint16_t size);
/* Trả HAL_BUSY khi DMA đang chạy; HAL_OK khi xong; HAL_ERROR/TIMEOUT khi có lỗi. */
HAL_StatusTypeDef I2cBus_GetTransferStatus(void);
/* Trả về mã lỗi chi tiết gần nhất do HAL I2C ghi nhận. */
uint32_t I2cBus_GetErrorCode(void);

#endif
