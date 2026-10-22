#include "main.h"
#include "mpu6050.h"
#include "config.h"

#if MPU6050_ENABLED

/*
 * Thanh ghi MPU6050 dùng trong module này.
 * Chỉ khai báo các thanh ghi cần thiết để tránh phụ thuộc vào thư viện ngoài.
 */
#define MPU6050_REG_PWR_MGMT_1   0x6BU  /* Power management; bit SLEEP mặc định = 1. */
#define MPU6050_REG_GYRO_CONFIG  0x1BU  /* Cấu hình full-scale range của gyroscope.  */
#define MPU6050_REG_GYRO_ZOUT_H  0x47U  /* Byte cao GYRO_ZOUT; byte thấp tiếp liền.  */
#define MPU6050_REG_WHO_AM_I     0x75U  /* Trả 0x68 (hoặc 0x72 tùy phiên bản chip). */

/*
 * Chọn hệ số nhạy dựa trên MPU6050_GYRO_FULL_SCALE_DPS.
 * Bảng: 250 dps → 131.0, 500 → 65.5, 1000 → 32.8, 2000 → 16.4 LSB/(°/s).
 */
#if   MPU6050_GYRO_FULL_SCALE_DPS == 250
  #define MPU6050_GYRO_CONFIG_FS_SEL 0x00U
  #define MPU6050_GYRO_SENSITIVITY   131.0f
#elif MPU6050_GYRO_FULL_SCALE_DPS == 500
  #define MPU6050_GYRO_CONFIG_FS_SEL 0x08U
  #define MPU6050_GYRO_SENSITIVITY   65.5f
#elif MPU6050_GYRO_FULL_SCALE_DPS == 1000
  #define MPU6050_GYRO_CONFIG_FS_SEL 0x10U
  #define MPU6050_GYRO_SENSITIVITY   32.8f
#elif MPU6050_GYRO_FULL_SCALE_DPS == 2000
  #define MPU6050_GYRO_CONFIG_FS_SEL 0x18U
  #define MPU6050_GYRO_SENSITIVITY   16.4f
#else
  #error "MPU6050_GYRO_FULL_SCALE_DPS must be 250, 500, 1000, or 2000"
#endif

/*
 * Hằng số bộ lọc thông thấp IIR bậc một:
 *   y[n] = alpha * x[n] + (1 - alpha) * y[n-1]
 * alpha = MPU6050_GYRO_LOWPASS_ALPHA_PERCENT / 100.0f.
 * Giá trị nhỏ → làm mượt mạnh, phản hồi chậm; giá trị lớn → phản hồi nhanh.
 */
#define MPU6050_LOWPASS_ALPHA  ((float)MPU6050_GYRO_LOWPASS_ALPHA_PERCENT / 100.0f)

/*
 * MPU6050 giao tiếp qua I2C2 (PB10=SCL, PB11=SDA) — bus riêng biệt hoàn toàn
 * với I2C1 mà ToF VL53L0X/L1X đang dùng. Không cần arbiter hay spinwait.
 * HAL I2C nhận địa chỉ 8-bit = địa chỉ 7-bit << 1.
 */
#define MPU6050_HAL_ADDR  ((uint16_t)MPU6050_I2C_ADDRESS << 1U)

/* Timeout cho mọi giao dịch I2C blocking của module này. */
#define MPU6050_I2C_TIMEOUT_MS  I2C_TIMEOUT_MS

/* hi2c2 được khai báo và khởi tạo trong config.c. */
extern I2C_HandleTypeDef hi2c2;

/* Giá trị gyro Z đã lọc low-pass, đơn vị °/s; khởi tạo 0 khi chưa có mẫu. */
static float gyroZ_filtered = 0.0f;
/* Cờ bảo vệ: chỉ cho phép Mpu6050_Update khi init đã thành công. */
static uint8_t mpu6050_initialized = 0U;

/* Ghi một byte vào thanh ghi 8-bit của MPU6050 qua I2C2 blocking. */
static HAL_StatusTypeDef Mpu6050_WriteReg(uint8_t reg, uint8_t value) {
  return HAL_I2C_Mem_Write(&hi2c2, MPU6050_HAL_ADDR,
                            reg, I2C_MEMADD_SIZE_8BIT,
                            &value, 1U, MPU6050_I2C_TIMEOUT_MS);
}

/* Đọc nhiều byte liên tiếp từ thanh ghi của MPU6050 qua I2C2 blocking. */
static HAL_StatusTypeDef Mpu6050_ReadRegs(uint8_t reg, uint8_t *data, uint16_t size) {
  return HAL_I2C_Mem_Read(&hi2c2, MPU6050_HAL_ADDR,
                           reg, I2C_MEMADD_SIZE_8BIT,
                           data, size, MPU6050_I2C_TIMEOUT_MS);
}

HAL_StatusTypeDef Mpu6050_Init(void) {
  uint8_t who_am_i = 0U;

  /* Khởi tạo I2C2 (PB10/PB11) — bus riêng dành cho MPU6050. */
  if (Config_I2C2_Init() != HAL_OK) {
    return HAL_ERROR;
  }

  /*
   * Kiểm tra thiết bị phản hồi trước khi ghi cấu hình.
   * WHO_AM_I của MPU6050 thường là 0x68; một số phiên bản trả 0x72.
   * So sánh linh hoạt: chỉ từ chối nếu giao tiếp hoàn toàn thất bại.
   */
  if (Mpu6050_ReadRegs(MPU6050_REG_WHO_AM_I, &who_am_i, 1U) != HAL_OK) {
    return HAL_ERROR;
  }

  /* Thức MPU6050: xóa bit SLEEP (bit 6) trong PWR_MGMT_1; chọn xung nhịp nội. */
  if (Mpu6050_WriteReg(MPU6050_REG_PWR_MGMT_1, 0x00U) != HAL_OK) {
    return HAL_ERROR;
  }

  /* HAL_Delay nhỏ để oscillator nội ổn định sau khi thức. */
  HAL_Delay(10U);

  /* Đặt full-scale gyro range theo cấu hình; FS_SEL nằm ở bit 4:3 của GYRO_CONFIG. */
  if (Mpu6050_WriteReg(MPU6050_REG_GYRO_CONFIG, MPU6050_GYRO_CONFIG_FS_SEL) != HAL_OK) {
    return HAL_ERROR;
  }

  gyroZ_filtered = 0.0f;
  mpu6050_initialized = 1U;
  return HAL_OK;
}

HAL_StatusTypeDef Mpu6050_Update(void) {
  uint8_t raw[2];
  int16_t raw_gyro_z;
  float gyroZ_raw_dps;

  if (mpu6050_initialized == 0U) {
    return HAL_ERROR;
  }

  /*
   * Đọc hai byte GYRO_ZOUT_H và GYRO_ZOUT_L liên tiếp trong một giao dịch.
   * MPU6050 tự động tăng địa chỉ thanh ghi khi đọc nhiều byte.
   * Dùng I2C2 riêng → không block I2C1 của ToF, không cần spinwait.
   */
  if (Mpu6050_ReadRegs(MPU6050_REG_GYRO_ZOUT_H, raw, 2U) != HAL_OK) {
    return HAL_ERROR;
  }

  /* Ghép hai byte thành số nguyên có dấu 16-bit (big-endian). */
  raw_gyro_z = (int16_t)(((uint16_t)raw[0] << 8U) | (uint16_t)raw[1]);

  /* Quy đổi sang °/s theo hệ số nhạy của full-scale range đã chọn. */
  gyroZ_raw_dps = (float)raw_gyro_z / MPU6050_GYRO_SENSITIVITY;

  /*
   * Bộ lọc IIR bậc một (low-pass):
   *   y[n] = alpha * x[n] + (1 - alpha) * y[n-1]
   * Ở tần số cập nhật 1000 Hz, hằng số thời gian xấp xỉ:
   *   tau ≈ (1 - alpha) / (alpha * f_update)
   * Với alpha = 0.20 và f = 1000 Hz: tau ≈ 4 ms.
   */
  gyroZ_filtered = MPU6050_LOWPASS_ALPHA * gyroZ_raw_dps
                 + (1.0f - MPU6050_LOWPASS_ALPHA) * gyroZ_filtered;

  return HAL_OK;
}

float Mpu6050_GetGyroZ(void) {
  return gyroZ_filtered;
}

#else /* MPU6050_ENABLED == 0 */

/*
 * Stub API khi cảm biến bị tắt qua cấu hình.
 * Phần còn lại firmware dùng getGyro() mà không cần kiểm tra #if.
 */

HAL_StatusTypeDef Mpu6050_Init(void) {
  return HAL_OK;
}

HAL_StatusTypeDef Mpu6050_Update(void) {
  return HAL_OK;
}

float Mpu6050_GetGyroZ(void) {
  /* Trả 0.0f theo đặc tả: khi tắt cảm biến, getGyro() trả 0. */
  return 0.0f;
}

#endif /* MPU6050_ENABLED */

