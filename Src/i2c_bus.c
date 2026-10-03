#include "main.h"
#include "i2c_bus.h"

extern I2C_HandleTypeDef hi2c1;

static volatile HAL_StatusTypeDef transferStatus = HAL_OK;

/*
 * HAL nhận địa chỉ thiết bị ở định dạng 8-bit (địa chỉ 7-bit dịch trái một bit).
 * API của module nhận địa chỉ 7-bit để người dùng không phải tự dịch hoặc
 * nhầm lẫn địa chỉ đọc/ghi; bit R/W sẽ do HAL điều khiển.
 */
static uint16_t I2cBus_HalAddress(uint8_t address) {
  return (uint16_t)address << 1U;
}

/*
 * Xác thực các tham số dùng chung trước khi gọi HAL.
 * Không gọi bus với địa chỉ ngoài miền 7-bit, bộ đệm NULL hoặc độ dài bằng 0.
 */
static HAL_StatusTypeDef I2cBus_ValidateTransfer(uint8_t address, const uint8_t *data, uint16_t size) {
  if (address > 0x7FU || data == NULL || size == 0U) {
    return HAL_ERROR;
  }

  return HAL_OK;
}

HAL_StatusTypeDef I2cBus_Init(void) {
  /* Chỉ bật clock/peripheral và cấu hình PB6/PB7 khi ứng dụng chủ động gọi. */
  if (hi2c1.Instance == I2C1 && HAL_I2C_GetState(&hi2c1) == HAL_I2C_STATE_READY) {
    return HAL_OK;
  }

  transferStatus = Config_I2C1_Init();
  return transferStatus;
}

HAL_StatusTypeDef I2cBus_IsDeviceReady(uint8_t address, uint32_t trials) {
  /* HAL gửi yêu cầu dò và xác nhận thiết bị bằng phản hồi ACK. */
  if (address > 0x7FU || trials == 0U) {
    return HAL_ERROR;
  }

  return HAL_I2C_IsDeviceReady(&hi2c1, I2cBus_HalAddress(address), trials, I2C_TIMEOUT_MS);
}

/* Bắt đầu gửi một gói byte bằng DMA, không diễn giải nội dung theo giao thức thiết bị. */
HAL_StatusTypeDef I2cBus_Transmit(uint8_t address, const uint8_t *data, uint16_t size) {
  HAL_StatusTypeDef status = I2cBus_ValidateTransfer(address, data, size);

  if (status != HAL_OK) {
    return status;
  }

  transferStatus = HAL_BUSY;
  status = HAL_I2C_Master_Transmit_DMA(&hi2c1, I2cBus_HalAddress(address), (uint8_t *)data, size);
  if (status != HAL_OK) {
    transferStatus = status;
  }

  return status;
}

/* Bắt đầu nhận một gói byte bằng DMA vào bộ đệm do bên gọi cấp phát. */
HAL_StatusTypeDef I2cBus_Receive(uint8_t address, uint8_t *data, uint16_t size) {
  HAL_StatusTypeDef status = I2cBus_ValidateTransfer(address, data, size);

  if (status != HAL_OK) {
    return status;
  }

  transferStatus = HAL_BUSY;
  status = HAL_I2C_Master_Receive_DMA(&hi2c1, I2cBus_HalAddress(address), data, size);
  if (status != HAL_OK) {
    transferStatus = status;
  }

  return status;
}

/* Bắt đầu đọc một hoặc nhiều byte từ thanh ghi/vùng nhớ bằng DMA. */
HAL_StatusTypeDef I2cBus_MemRead(uint8_t address, uint16_t reg, uint16_t regSize, uint8_t *data, uint16_t size) {
  HAL_StatusTypeDef status = I2cBus_ValidateTransfer(address, data, size);

  if (status != HAL_OK || (regSize != I2C_MEMADD_SIZE_8BIT && regSize != I2C_MEMADD_SIZE_16BIT)) {
    return HAL_ERROR;
  }

  transferStatus = HAL_BUSY;
  status = HAL_I2C_Mem_Read_DMA(&hi2c1, I2cBus_HalAddress(address), reg, regSize, data, size);
  if (status != HAL_OK) {
    transferStatus = status;
  }

  return status;
}

/* Bắt đầu ghi một hoặc nhiều byte vào thanh ghi/vùng nhớ bằng DMA. */
HAL_StatusTypeDef I2cBus_MemWrite(uint8_t address, uint16_t reg, uint16_t regSize, const uint8_t *data, uint16_t size) {
  HAL_StatusTypeDef status = I2cBus_ValidateTransfer(address, data, size);

  if (status != HAL_OK || (regSize != I2C_MEMADD_SIZE_8BIT && regSize != I2C_MEMADD_SIZE_16BIT)) {
    return HAL_ERROR;
  }

  transferStatus = HAL_BUSY;
  status = HAL_I2C_Mem_Write_DMA(&hi2c1, I2cBus_HalAddress(address), reg, regSize, (uint8_t *)data, size);
  if (status != HAL_OK) {
    transferStatus = status;
  }

  return status;
}

HAL_StatusTypeDef I2cBus_GetTransferStatus(void) {
  return transferStatus;
}

uint32_t I2cBus_GetErrorCode(void) {
  return HAL_I2C_GetError(&hi2c1);
}

/* Callback HAL khi một giao dịch DMA đọc/ghi dữ liệu thô hoàn tất. */
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c) {
  if (hi2c->Instance == I2C1) {
    transferStatus = HAL_OK;
  }
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c) {
  if (hi2c->Instance == I2C1) {
    transferStatus = HAL_OK;
  }
}

/* Callback HAL khi một giao dịch DMA đọc/ghi thanh ghi hoàn tất. */
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c) {
  if (hi2c->Instance == I2C1) {
    transferStatus = HAL_OK;
  }
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
  if (hi2c->Instance == I2C1) {
    transferStatus = HAL_OK;
  }
}

/* Lưu trạng thái lỗi HAL; mã lỗi chi tiết có thể đọc qua I2cBus_GetErrorCode(). */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
  if (hi2c->Instance == I2C1) {
    transferStatus = HAL_ERROR;
  }
}
