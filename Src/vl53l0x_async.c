#include "main.h"
#include "vl53l0x_async.h"
#include "config.h"
#include "i2c_bus.h"
#include "tof_sensors.h"

#define VL53L0X_REG_RESULT_DISTANCE 0x1EU
#define VL53L0X_INTERRUPT_CLEAR_VALUE 0x01U
#define VL53L0X_INTERRUPT_STATUS_MASK 0x07U
#define VL53L0X_ASYNC_POLL_INTERVAL_MS 5U
#define VL53L0X_RANGE_RESULT_SIZE 12U

typedef enum {
  VL53L0X_ASYNC_STOPPED,
  VL53L0X_ASYNC_IDLE,
  VL53L0X_ASYNC_READING_STATUS,
  VL53L0X_ASYNC_READING_RANGE,
  VL53L0X_ASYNC_CLEARING_INTERRUPT
} Vl53l0xAsync_State;

typedef struct {
  uint8_t address;
  uint16_t *distance;
  uint8_t *newData;
  uint8_t *filterInitialized;
} Vl53l0xAsync_Sensor;

static uint16_t distanceRight;
static uint16_t distanceMid;
static uint16_t distanceLeft;
static uint8_t rightNewData;
static uint8_t midNewData;
static uint8_t leftNewData;
static uint8_t rightFilterInitialized;
static uint8_t midFilterInitialized;
static uint8_t leftFilterInitialized;
static Vl53l0xAsync_Sensor sensors[TOF_SENSOR_COUNT] = {
  {TOF_SENSOR_RIGHT_I2C_ADDRESS, &distanceRight, &rightNewData, &rightFilterInitialized},
  {TOF_SENSOR_MID_I2C_ADDRESS, &distanceMid, &midNewData, &midFilterInitialized},
  {TOF_SENSOR_LEFT_I2C_ADDRESS, &distanceLeft, &leftNewData, &leftFilterInitialized}
};

static Vl53l0xAsync_State state = VL53L0X_ASYNC_STOPPED;
static uint8_t currentSensor;
static uint8_t interruptStatus;
static uint8_t rangeData[VL53L0X_RANGE_RESULT_SIZE];
static uint8_t interruptClearValue = VL53L0X_INTERRUPT_CLEAR_VALUE;
static uint16_t completedDistance;
static uint8_t completedRangeStatus;
static uint32_t nextPollTick;
static HAL_StatusTypeDef lastStatus = HAL_OK;

static HAL_StatusTypeDef Vl53l0xAsync_StartRead(uint16_t reg, uint8_t *data, uint16_t size) {
  HAL_StatusTypeDef status = I2cBus_MemRead(sensors[currentSensor].address, reg, I2C_MEMADD_SIZE_8BIT, data, size);

  if (status != HAL_OK) {
    lastStatus = status;
  }
  return status;
}

static HAL_StatusTypeDef Vl53l0xAsync_StartWrite(uint16_t reg, const uint8_t *data, uint16_t size) {
  HAL_StatusTypeDef status = I2cBus_MemWrite(sensors[currentSensor].address, reg, I2C_MEMADD_SIZE_8BIT, data, size);

  if (status != HAL_OK) {
    lastStatus = status;
  }
  return status;
}

static uint8_t Vl53l0xAsync_TransferFinished(void) {
  HAL_StatusTypeDef status = I2cBus_GetTransferStatus();

  if (status == HAL_BUSY) {
    return 0U;
  }
  if (status != HAL_OK) {
    lastStatus = status;
    return 2U;
  }

  return 1U;
}

static void Vl53l0xAsync_AdvanceSensor(uint32_t now) {
  currentSensor = (uint8_t)((currentSensor + 1U) % TOF_SENSOR_COUNT);
  if (currentSensor == 0U) {
    nextPollTick = now + VL53L0X_ASYNC_POLL_INTERVAL_MS;
  }
  state = VL53L0X_ASYNC_IDLE;
}

HAL_StatusTypeDef Vl53l0xAsync_Start(void) {
  for (uint32_t i = 0U; i < TOF_SENSOR_COUNT; i++) {
    if (TofSensors_GetDevice((TofXshut_Sensor)i) == NULL) {
      lastStatus = HAL_ERROR;
      return HAL_ERROR;
    }
  }

  if (I2cBus_GetTransferStatus() == HAL_BUSY) {
    lastStatus = HAL_BUSY;
    return HAL_BUSY;
  }

  distanceRight = 0U;
  distanceMid = 0U;
  distanceLeft = 0U;
  rightNewData = 0U;
  midNewData = 0U;
  leftNewData = 0U;
  rightFilterInitialized = 0U;
  midFilterInitialized = 0U;
  leftFilterInitialized = 0U;
  currentSensor = 0U;
  nextPollTick = HAL_GetTick();
  lastStatus = HAL_OK;
  state = VL53L0X_ASYNC_IDLE;
  return HAL_OK;
}

/*
 * Thực hiện tối đa một bước nhỏ của state machine mỗi lần gọi:
 * thăm dò status, đọc kết quả hoặc xóa cờ ngắt đều dùng DMA, không chờ I2C.
 * Mỗi địa chỉ được xử lý tuần tự trên bus; cảm biến vẫn tự đo song song.
 */
void Vl53l0xAsync_Process(void) {
  uint32_t now = HAL_GetTick();
  uint8_t transferResult;

  switch (state) {
    case VL53L0X_ASYNC_IDLE:
      if ((int32_t)(now - nextPollTick) >= 0) {
        if (Vl53l0xAsync_StartRead(VL53L0X_REG_RESULT_INTERRUPT_STATUS, &interruptStatus, 1U) == HAL_OK) {
          state = VL53L0X_ASYNC_READING_STATUS;
        } else {
          Vl53l0xAsync_AdvanceSensor(now);
        }
      }
      break;

    case VL53L0X_ASYNC_READING_STATUS:
      transferResult = Vl53l0xAsync_TransferFinished();
      if (transferResult == 1U) {
        if ((interruptStatus & VL53L0X_INTERRUPT_STATUS_MASK) != 0U) {
          if (Vl53l0xAsync_StartRead(VL53L0X_REG_RESULT_RANGE_STATUS, rangeData, VL53L0X_RANGE_RESULT_SIZE) == HAL_OK) {
            state = VL53L0X_ASYNC_READING_RANGE;
          } else {
            Vl53l0xAsync_AdvanceSensor(now);
          }
        } else {
          Vl53l0xAsync_AdvanceSensor(now);
        }
      } else if (transferResult == 2U) {
        Vl53l0xAsync_AdvanceSensor(now);
      }
      break;

    case VL53L0X_ASYNC_READING_RANGE:
      transferResult = Vl53l0xAsync_TransferFinished();
      if (transferResult == 1U) {
        completedRangeStatus = (uint8_t)((rangeData[0] >> 3U) & 0x0FU);
        completedDistance = (uint16_t)(((uint16_t)rangeData[VL53L0X_REG_RESULT_DISTANCE - VL53L0X_REG_RESULT_RANGE_STATUS] << 8U) |
                                        rangeData[VL53L0X_REG_RESULT_DISTANCE - VL53L0X_REG_RESULT_RANGE_STATUS + 1U]);
        if (Vl53l0xAsync_StartWrite(VL53L0X_REG_SYSTEM_INTERRUPT_CLEAR, &interruptClearValue, 1U) == HAL_OK) {
          state = VL53L0X_ASYNC_CLEARING_INTERRUPT;
        } else {
          Vl53l0xAsync_AdvanceSensor(now);
        }
      } else if (transferResult == 2U) {
        Vl53l0xAsync_AdvanceSensor(now);
      }
      break;

    case VL53L0X_ASYNC_CLEARING_INTERRUPT:
      transferResult = Vl53l0xAsync_TransferFinished();
      if (transferResult == 1U) {
        if (completedRangeStatus == 0U) {
          if (*sensors[currentSensor].filterInitialized == 0U) {
            *sensors[currentSensor].distance = completedDistance;
            *sensors[currentSensor].filterInitialized = 1U;
          } else {
            /* EMA alpha=1/2: làm mượt nhẹ nhưng vẫn phản ứng nhanh với vật cản. */
            *sensors[currentSensor].distance = (uint16_t)(((uint32_t)*sensors[currentSensor].distance + completedDistance + 1U) / 2U);
          }
          *sensors[currentSensor].newData = 1U;
        }
        lastStatus = HAL_OK;
        Vl53l0xAsync_AdvanceSensor(now);
      } else if (transferResult == 2U) {
        Vl53l0xAsync_AdvanceSensor(now);
      }
      break;

    case VL53L0X_ASYNC_STOPPED:
    default:
      break;
  }
}

static bool Vl53l0xAsync_TryGetDistance(uint16_t *distance, uint16_t *storedDistance, uint8_t *newData) {
  if (distance == NULL) {
    return false;
  }

#if !TOF_ENABLED
  (void)storedDistance;
  (void)newData;
  *distance = 0U;
  return false;
#else
  if (*newData == 0U) {
    return false;
  }

  *distance = *storedDistance;
  *newData = 0U;
  return true;
#endif
}

bool getDisLeft(uint16_t *dis_left) {
  return Vl53l0xAsync_TryGetDistance(dis_left, &distanceLeft, &leftNewData);
}

bool getDisMid(uint16_t *dis_mid) {
  return Vl53l0xAsync_TryGetDistance(dis_mid, &distanceMid, &midNewData);
}

bool getDisRight(uint16_t *dis_right) {
  return Vl53l0xAsync_TryGetDistance(dis_right, &distanceRight, &rightNewData);
}

HAL_StatusTypeDef Vl53l0xAsync_GetStatus(void) {
  return lastStatus;
}

HAL_StatusTypeDef Vl53l0xAsync_Stop(void) {
  if (I2cBus_GetTransferStatus() == HAL_BUSY) {
    lastStatus = HAL_BUSY;
    return HAL_BUSY;
  }

  state = VL53L0X_ASYNC_STOPPED;
  rightNewData = 0U;
  midNewData = 0U;
  leftNewData = 0U;
  rightFilterInitialized = 0U;
  midFilterInitialized = 0U;
  leftFilterInitialized = 0U;
  lastStatus = HAL_OK;
  return HAL_OK;
}
