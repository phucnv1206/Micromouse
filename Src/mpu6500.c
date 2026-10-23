#include "main.h"
#include "mpu6500.h"
#include "config.h"

#if MPU6500_ENABLED

/*
 * Thanh ghi MPU6500 sử dụng trong module.
 */
#define MPU6500_REG_PWR_MGMT_1        0x6BU  /* Bit 7: H_RESET, Bit 6: SLEEP, Bit 2:0: CLKSEL */
#define MPU6500_REG_PWR_MGMT_2        0x6CU  /* Bit 5:0: Enable/disable gyro và accel các trục */
#define MPU6500_REG_CONFIG            0x1AU  /* DLPF (Digital Low Pass Filter) */
#define MPU6500_REG_GYRO_CONFIG       0x1BU  /* Full-scale range gyroscope (FS_SEL) */
#define MPU6500_REG_GYRO_XOUT_H       0x43U  /* Burst read 6 byte từ XOUT_H đến ZOUT_L */
#define MPU6500_REG_GYRO_ZOUT_H       0x47U  /* Byte cao GYRO_ZOUT */
#define MPU6500_REG_WHO_AM_I          0x75U  /* Thanh ghi định danh: MPU6500 trả về 0x70 (112) */

/*
 * Hệ số nhạy theo dải đo full-scale của con quay hồi chuyển:
 *   ±250 dps:  131.0 LSB/(°/s)
 *   ±500 dps:   65.5 LSB/(°/s)
 *   ±1000 dps:  32.8 LSB/(°/s)
 *   ±2000 dps:  16.4 LSB/(°/s)
 */
#if   MPU6500_GYRO_FULL_SCALE_DPS == 250
  #define MPU6500_GYRO_CONFIG_FS_SEL 0x00U
  #define MPU6500_GYRO_SENSITIVITY   131.0f
#elif MPU6500_GYRO_FULL_SCALE_DPS == 500
  #define MPU6500_GYRO_CONFIG_FS_SEL 0x08U
  #define MPU6500_GYRO_SENSITIVITY   65.5f
#elif MPU6500_GYRO_FULL_SCALE_DPS == 1000
  #define MPU6500_GYRO_CONFIG_FS_SEL 0x10U
  #define MPU6500_GYRO_SENSITIVITY   32.8f
#elif MPU6500_GYRO_FULL_SCALE_DPS == 2000
  #define MPU6500_GYRO_CONFIG_FS_SEL 0x18U
  #define MPU6500_GYRO_SENSITIVITY   16.4f
#else
  #error "MPU6500_GYRO_FULL_SCALE_DPS must be 250, 500, 1000, or 2000"
#endif

/* Hệ số lọc thông thấp IIR: y[n] = alpha * x[n] + (1 - alpha) * y[n-1] */
#define MPU6500_LOWPASS_ALPHA   ((float)MPU6500_GYRO_LOWPASS_ALPHA_PERCENT / 100.0f)
#define MPU6500_HAL_ADDR        ((uint16_t)MPU6500_I2C_ADDRESS << 1U)
#define MPU6500_I2C_TIMEOUT_MS  5U

extern I2C_HandleTypeDef hi2c2;

static float gyroZ_filtered = 0.0f;
static uint8_t mpu6500_initialized = 0U;

static int16_t mpu_init_step = 0;
static uint32_t mpu_read_count = 0;
static uint32_t mpu_last_hal_error = 0;

static HAL_StatusTypeDef Mpu6500_WriteReg(uint8_t reg, uint8_t value, uint32_t timeout) {
  return HAL_I2C_Mem_Write(&hi2c2, MPU6500_HAL_ADDR,
                            reg, I2C_MEMADD_SIZE_8BIT,
                            &value, 1U, timeout);
}

static HAL_StatusTypeDef Mpu6500_ReadRegs(uint8_t reg, uint8_t *data, uint16_t size, uint32_t timeout) {
  return HAL_I2C_Mem_Read(&hi2c2, MPU6500_HAL_ADDR,
                           reg, I2C_MEMADD_SIZE_8BIT,
                           data, size, timeout);
}

HAL_StatusTypeDef Mpu6500_Init(void) {
  uint8_t who_am_i = 0U;
  HAL_StatusTypeDef status;
  mpu_init_step = 0;
  mpu_last_hal_error = 0;

  /* Khởi tạo I2C2 (PB10=SCL, PB3=SDA) - đã bao gồm lệnh tắt SWV/SWO trên PB3 */
  if (Config_I2C2_Init() != HAL_OK) {
    mpu_init_step = -1;
    mpu_last_hal_error = HAL_I2C_GetError(&hi2c2);
    return HAL_ERROR;
  }

  /* Đợi nguồn cấp và cảm biến ổn định */
  HAL_Delay(30U);

  /*
   * 1. Reset mềm thiết bị (H_RESET bit 7 của PWR_MGMT_1):
   * Đưa toàn bộ thanh ghi MPU6500 về trạng thái mặc định.
   */
  Mpu6500_WriteReg(MPU6500_REG_PWR_MGMT_1, 0x80U, 50U);
  HAL_Delay(50U);

  /*
   * 2. Đánh thức MPU6500 và chọn nguồn xung nhịp tốt nhất (Auto PLL):
   * Ghi 0x01 vào PWR_MGMT_1 (SLEEP = 0, CLKSEL = 1). Thử tối đa 5 lần.
   */
  status = HAL_ERROR;
  for (uint8_t retry = 0U; retry < 5U; retry++) {
    status = Mpu6500_WriteReg(MPU6500_REG_PWR_MGMT_1, 0x01U, 50U);
    if (status == HAL_OK) {
      break;
    }
    HAL_Delay(10U);
  }

  if (status != HAL_OK) {
    mpu_init_step = -2;
    mpu_last_hal_error = HAL_I2C_GetError(&hi2c2);
    return HAL_ERROR;
  }

  /* Đợi 20ms cho PLL nội ổn định */
  HAL_Delay(20U);

  /* 3. Bật toàn bộ 3 trục Gyro và Accel (PWR_MGMT_2 = 0x00) */
  Mpu6500_WriteReg(MPU6500_REG_PWR_MGMT_2, 0x00U, 50U);

  /*
   * 4. Đọc thanh ghi WHO_AM_I (0x75):
   * MPU6500 sẽ trả về giá trị 0x70 (112).
   */
  if (Mpu6500_ReadRegs(MPU6500_REG_WHO_AM_I, &who_am_i, 1U, 50U) != HAL_OK) {
    mpu_init_step = -3;
    mpu_last_hal_error = HAL_I2C_GetError(&hi2c2);
    return HAL_ERROR;
  }

  /*
   * 5. Cấu hình Digital Low-Pass Filter (DLPF) trong thanh ghi CONFIG (0x1A):
   * Ghi 0x03: Gyro bandwidth 41 Hz, delay 5.9 ms, sample rate 1 kHz.
   */
  Mpu6500_WriteReg(MPU6500_REG_CONFIG, 0x03U, 50U);

  /* 6. Cấu hình dải đo con quay hồi chuyển trong GYRO_CONFIG (0x1B) */
  if (Mpu6500_WriteReg(MPU6500_REG_GYRO_CONFIG, MPU6500_GYRO_CONFIG_FS_SEL, 50U) != HAL_OK) {
    mpu_init_step = -4;
    mpu_last_hal_error = HAL_I2C_GetError(&hi2c2);
    return HAL_ERROR;
  }

  gyroZ_filtered = 0.0f;
  mpu_read_count = 0;
  mpu_init_step = (int16_t)who_am_i; /* Thành công: trả về mã WHO_AM_I (112 = 0x70) */
  mpu6500_initialized = 1U;
  return HAL_OK;
}

HAL_StatusTypeDef Mpu6500_Update(void) {
  uint8_t raw[6];
  int16_t raw_gyro_z;
  float gyroZ_raw_dps;

  if (mpu6500_initialized == 0U) {
    return HAL_ERROR;
  }

  /*
   * Đọc burst 6 byte từ GYRO_XOUT_H (0x43) đến GYRO_ZOUT_L (0x48).
   * Byte 4 và 5 tương ứng với trục Z: raw[4] = ZOUT_H, raw[5] = ZOUT_L.
   */
  if (Mpu6500_ReadRegs(MPU6500_REG_GYRO_XOUT_H, raw, 6U, MPU6500_I2C_TIMEOUT_MS) != HAL_OK) {
    mpu_last_hal_error = HAL_I2C_GetError(&hi2c2);
    return HAL_ERROR;
  }

  /* Ghép byte 4 và 5 thành số nguyên 16-bit có dấu */
  raw_gyro_z = (int16_t)(((uint16_t)raw[4] << 8U) | (uint16_t)raw[5]);

  /* Quy đổi sang °/s theo hệ số nhạy của dải đo đã chọn */
  gyroZ_raw_dps = (float)raw_gyro_z / MPU6500_GYRO_SENSITIVITY;

  /* Bộ lọc thông thấp IIR bậc một */
  gyroZ_filtered = MPU6500_LOWPASS_ALPHA * gyroZ_raw_dps
                 + (1.0f - MPU6500_LOWPASS_ALPHA) * gyroZ_filtered;

  mpu_read_count++;
  return HAL_OK;
}

float Mpu6500_GetGyroZ(void) {
  return gyroZ_filtered;
}

int16_t Mpu6500_GetInitStep(void) {
  return mpu_init_step;
}

uint32_t Mpu6500_GetReadCount(void) {
  return mpu_read_count;
}

uint32_t Mpu6500_GetLastHalError(void) {
  return mpu_last_hal_error;
}

#else /* MPU6500_ENABLED == 0 */

HAL_StatusTypeDef Mpu6500_Init(void) {
  return HAL_OK;
}

HAL_StatusTypeDef Mpu6500_Update(void) {
  return HAL_OK;
}

float Mpu6500_GetGyroZ(void) {
  return 0.0f;
}

int16_t Mpu6500_GetInitStep(void) {
  return 0;
}

uint32_t Mpu6500_GetReadCount(void) {
  return 0;
}

uint32_t Mpu6500_GetLastHalError(void) {
  return 0;
}

#endif /* MPU6500_ENABLED */

