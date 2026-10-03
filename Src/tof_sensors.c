#include <string.h>
#include "main.h"
#include "config.h"
#include "i2c_bus.h"
#include "led.h"
#include "tof_sensors.h"
#include "VL53L1X_api.h"

extern I2C_HandleTypeDef hi2c1;

#define VL53L0X_DEFAULT_ADDRESS_8BIT (VL53L0X_DEFAULT_I2C_ADDRESS << 1U)
#define VL53L0X_SHORT_PRE_RANGE_VCSEL_PERIOD 14U
#define VL53L0X_SHORT_FINAL_RANGE_VCSEL_PERIOD 10U
#define VL53L0X_SHORT_SIGNAL_RATE_LIMIT_MCPS 0x0000199AU

typedef struct {
  TofXshut_Sensor sensor;
  uint8_t address7Bit;
  uint32_t timingBudgetUs;
  VL53L0X_Dev_t driver;
  uint8_t initialized;
} TofSensors_Device;

static TofSensors_Device devices[TOF_SENSOR_COUNT];
static uint16_t distanceMm[TOF_SENSOR_COUNT];
static VL53L0X_Error lastError = VL53L0X_ERROR_NONE;
static VL53L1X_ERROR lastL1Error = 0;
static TofSensors_InitStage lastInitStage = TOF_INIT_STAGE_NONE;
static TofXshut_Sensor lastInitSensor = TOF_SENSOR_RIGHT;

static uint8_t TofSensors_GetLedNumber(TofXshut_Sensor sensor) {
  switch (sensor) {
    case TOF_SENSOR_LEFT:
      return 1U;
    case TOF_SENSOR_MID:
      return 2U;
    case TOF_SENSOR_RIGHT:
      return 3U;
    default:
      return 0U;
  }
}

/* Giữ mã lỗi API ST để bên gọi có thể chẩn đoán khi khởi tạo thất bại. */
static HAL_StatusTypeDef TofSensors_CheckApiStatus(VL53L0X_Error status) {
  lastError = status;
  return status == VL53L0X_ERROR_NONE ? HAL_OK : HAL_ERROR;
}

static HAL_StatusTypeDef TofSensors_ConfigureMidDevice(TofSensors_Device *device) {
  const uint16_t defaultAddress = (uint16_t)(VL53L0X_DEFAULT_I2C_ADDRESS << 1U);
  const uint16_t deviceAddress = (uint16_t)(device->address7Bit << 1U);
  uint16_t currentAddress = defaultAddress;

  lastInitStage = TOF_INIT_STAGE_L1_SENSOR_INIT;
  lastL1Error = VL53L1X_SensorInit(currentAddress);
  if (lastL1Error != 0) {
    return HAL_ERROR;
  }

  lastInitStage = TOF_INIT_STAGE_L1_SET_ADDRESS;
  lastL1Error = VL53L1X_SetI2CAddress(currentAddress, (uint8_t)deviceAddress);
  if (lastL1Error != 0) {
    return HAL_ERROR;
  }
  currentAddress = deviceAddress;

  lastInitStage = TOF_INIT_STAGE_L1_SHORT_MODE;
  lastL1Error = VL53L1X_SetDistanceMode(currentAddress, 1U);
  if (lastL1Error != 0) {
    return HAL_ERROR;
  }
  lastInitStage = TOF_INIT_STAGE_L1_TIMING_BUDGET;
  lastL1Error = VL53L1X_SetTimingBudgetInMs(currentAddress, TOF_MID_TIMING_BUDGET_MS);
  if (lastL1Error != 0) {
    return HAL_ERROR;
  }
  uint16_t timingBudgetMs = 0U;
  lastL1Error = VL53L1X_GetTimingBudgetInMs(currentAddress, &timingBudgetMs);
  if (lastL1Error != 0 || timingBudgetMs != TOF_MID_TIMING_BUDGET_MS) {
    if (lastL1Error == 0) {
      lastL1Error = 1;
    }
    return HAL_ERROR;
  }
  lastInitStage = TOF_INIT_STAGE_L1_INTER_MEASUREMENT;
  lastL1Error = VL53L1X_SetInterMeasurementInMs(currentAddress, TOF_MID_INTER_MEASUREMENT_MS);
  if (lastL1Error != 0) {
    return HAL_ERROR;
  }
  uint16_t interMeasurementMs = 0U;
  lastL1Error = VL53L1X_GetInterMeasurementInMs(currentAddress, &interMeasurementMs);
  if (lastL1Error != 0 || interMeasurementMs != TOF_MID_INTER_MEASUREMENT_MS) {
    if (lastL1Error == 0) {
      lastL1Error = 1;
    }
    return HAL_ERROR;
  }
  lastInitStage = TOF_INIT_STAGE_L1_START_RANGING;
  lastL1Error = VL53L1X_StartRanging(currentAddress);
  if (lastL1Error != 0) {
    return HAL_ERROR;
  }

  device->initialized = 1U;
  return HAL_OK;
}

static HAL_StatusTypeDef TofSensors_ConfigureDevice(TofSensors_Device *device) {
  uint8_t vhvSettings;
  uint8_t phaseCal;
  uint32_t referenceSpadCount;
  uint8_t isApertureSpads;
  VL53L0X_Error status;

  if (device->sensor == TOF_SENSOR_MID) {
    return TofSensors_ConfigureMidDevice(device);
  }

  memset(&device->driver, 0, sizeof(device->driver));
  device->driver.I2cHandle = &hi2c1;
  /* Driver ST nhận địa chỉ ở dạng 8-bit; địa chỉ cấu hình bên ngoài là 7-bit. */
  device->driver.I2cDevAddr = VL53L0X_DEFAULT_ADDRESS_8BIT;

  lastInitStage = TOF_INIT_STAGE_DATA_INIT;
  status = VL53L0X_DataInit(&device->driver);
  if (TofSensors_CheckApiStatus(status) != HAL_OK) {
    return HAL_ERROR;
  }

  lastInitStage = TOF_INIT_STAGE_SET_ADDRESS;
  status = VL53L0X_SetDeviceAddress(&device->driver, (uint8_t)(device->address7Bit << 1U));
  if (TofSensors_CheckApiStatus(status) != HAL_OK) {
    return HAL_ERROR;
  }
  device->driver.I2cDevAddr = (uint8_t)(device->address7Bit << 1U);

  /* Chạy cấu hình tĩnh, hiệu chuẩn tham chiếu/SPAD trước khi chọn chế độ đo. */
  lastInitStage = TOF_INIT_STAGE_STATIC_INIT;
  status = VL53L0X_StaticInit(&device->driver);
  if (TofSensors_CheckApiStatus(status) != HAL_OK) {
    return HAL_ERROR;
  }
  lastInitStage = TOF_INIT_STAGE_REF_CALIBRATION;
  status = VL53L0X_PerformRefCalibration(&device->driver, &vhvSettings, &phaseCal);
  if (TofSensors_CheckApiStatus(status) != HAL_OK) {
    return HAL_ERROR;
  }
  lastInitStage = TOF_INIT_STAGE_REF_SPAD;
  status = VL53L0X_PerformRefSpadManagement(&device->driver, &referenceSpadCount, &isApertureSpads);
  if (TofSensors_CheckApiStatus(status) != HAL_OK) {
    return HAL_ERROR;
  }

  /* Cấu hình short-range: VCSEL period ngắn và ngưỡng signal-rate 0,1 MCPS. */
  lastInitStage = TOF_INIT_STAGE_SHORT_PRE_RANGE;
  status = VL53L0X_SetVcselPulsePeriod(&device->driver, VL53L0X_VCSEL_PERIOD_PRE_RANGE, VL53L0X_SHORT_PRE_RANGE_VCSEL_PERIOD);
  if (TofSensors_CheckApiStatus(status) != HAL_OK) {
    return HAL_ERROR;
  }
  lastInitStage = TOF_INIT_STAGE_SHORT_FINAL_RANGE;
  status = VL53L0X_SetVcselPulsePeriod(&device->driver, VL53L0X_VCSEL_PERIOD_FINAL_RANGE, VL53L0X_SHORT_FINAL_RANGE_VCSEL_PERIOD);
  if (TofSensors_CheckApiStatus(status) != HAL_OK) {
    return HAL_ERROR;
  }
  lastInitStage = TOF_INIT_STAGE_SIGNAL_RATE;
  status = VL53L0X_SetLimitCheckValue(&device->driver, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, VL53L0X_SHORT_SIGNAL_RATE_LIMIT_MCPS);
  if (TofSensors_CheckApiStatus(status) != HAL_OK) {
    return HAL_ERROR;
  }

  lastInitStage = TOF_INIT_STAGE_SET_MODE;
  status = VL53L0X_SetDeviceMode(&device->driver, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
  if (TofSensors_CheckApiStatus(status) != HAL_OK) {
    return HAL_ERROR;
  }
  lastInitStage = TOF_INIT_STAGE_TIMING_BUDGET;
  status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(&device->driver, device->timingBudgetUs);
  if (TofSensors_CheckApiStatus(status) != HAL_OK) {
    return HAL_ERROR;
  }
  lastInitStage = TOF_INIT_STAGE_START_MEASUREMENT;
  status = VL53L0X_StartMeasurement(&device->driver);
  if (TofSensors_CheckApiStatus(status) != HAL_OK) {
    return HAL_ERROR;
  }

  device->initialized = 1U;
  return HAL_OK;
}

HAL_StatusTypeDef TofSensors_Init(void) {
  lastError = VL53L0X_ERROR_NONE;
  lastL1Error = 0;
  memset(distanceMm, 0, sizeof(distanceMm));
  lastInitStage = TOF_INIT_STAGE_NONE;
  lastInitSensor = TOF_SENSOR_RIGHT;
  for (uint8_t ledNumber = 1U; ledNumber <= 3U; ledNumber++) {
    Led_SetPattern(ledNumber, 0U, 0U, 0U, 0U);
  }
  Led_Update();

  devices[TOF_SENSOR_RIGHT].sensor = TOF_SENSOR_RIGHT;
  devices[TOF_SENSOR_RIGHT].address7Bit = TOF_SENSOR_RIGHT_I2C_ADDRESS;
  devices[TOF_SENSOR_RIGHT].timingBudgetUs = TOF_OUTER_MEASUREMENT_TIMING_BUDGET_US;
  devices[TOF_SENSOR_MID].sensor = TOF_SENSOR_MID;
  devices[TOF_SENSOR_MID].address7Bit = TOF_SENSOR_MID_I2C_ADDRESS;
  devices[TOF_SENSOR_MID].timingBudgetUs = 0U;
  devices[TOF_SENSOR_LEFT].sensor = TOF_SENSOR_LEFT;
  devices[TOF_SENSOR_LEFT].address7Bit = TOF_SENSOR_LEFT_I2C_ADDRESS;
  devices[TOF_SENSOR_LEFT].timingBudgetUs = TOF_OUTER_MEASUREMENT_TIMING_BUDGET_US;

  /* Tắt cả ba cảm biến trước, để chỉ cảm biến đang cấu hình dùng địa chỉ mặc định. */
  TofXshut_Init();
  for (uint32_t i = 0U; i < TOF_SENSOR_COUNT; i++) {
    devices[i].initialized = 0U;
  }

  HAL_Delay(TOF_XSHUT_RESET_DELAY_MS);
  lastInitStage = TOF_INIT_STAGE_I2C;
  if (I2cBus_Init() != HAL_OK) {
    lastError = VL53L0X_ERROR_CONTROL_INTERFACE;
    return HAL_ERROR;
  }

  for (uint32_t i = 0U; i < TOF_SENSOR_COUNT; i++) {
    /* Bật từng cảm biến, đợi boot, đổi địa chỉ rồi mới bật cảm biến kế tiếp. */
    lastInitSensor = devices[i].sensor;
    lastInitStage = TOF_INIT_STAGE_XSHUT;
    if (TofXshut_Set(devices[i].sensor, true) != HAL_OK) {
      lastError = VL53L0X_ERROR_CONTROL_INTERFACE;
      goto initialization_failed;
    }
    HAL_Delay(TOF_SENSOR_BOOT_DELAY_MS);
    lastInitStage = TOF_INIT_STAGE_DEVICE_READY;
    if (I2cBus_IsDeviceReady(VL53L0X_DEFAULT_I2C_ADDRESS, 3U) != HAL_OK) {
      lastError = VL53L0X_ERROR_CONTROL_INTERFACE;
      goto initialization_failed;
    }
    if (TofSensors_ConfigureDevice(&devices[i]) != HAL_OK) {
      goto initialization_failed;
    }
    Led_SetPattern(TofSensors_GetLedNumber(devices[i].sensor), 1U, 100U, 0U, 0U);
    Led_Update();
    if (i + 1U < TOF_SENSOR_COUNT) {
      HAL_Delay(TOF_SENSOR_INTER_INIT_DELAY_MS);
    }
  }

  return HAL_OK;

initialization_failed:
  /* Tắt cảm biến lỗi; giữ các cảm biến đã init và LED trạng thái của chúng. */
  TofXshut_Set(lastInitSensor, false);
  devices[lastInitSensor].initialized = 0U;
  return HAL_ERROR;
}

HAL_StatusTypeDef TofSensors_UpdateDistances(uint16_t *leftMm, uint16_t *midMm, uint16_t *rightMm) {
  VL53L0X_RangingMeasurementData_t measurement;
  VL53L0X_DEV device;
  uint8_t dataReady = 0U;
  uint8_t rangeStatus = 0U;
  uint16_t midDistance = 0U;
  VL53L0X_Error l0Status;

  if (leftMm == NULL || midMm == NULL || rightMm == NULL
      || devices[TOF_SENSOR_LEFT].initialized == 0U
      || devices[TOF_SENSOR_MID].initialized == 0U
      || devices[TOF_SENSOR_RIGHT].initialized == 0U) {
    return HAL_ERROR;
  }

  device = &devices[TOF_SENSOR_LEFT].driver;
  l0Status = VL53L0X_GetMeasurementDataReady(device, &dataReady);
  if (TofSensors_CheckApiStatus(l0Status) != HAL_OK) {
    return HAL_ERROR;
  }
  if (dataReady != 0U) {
    l0Status = VL53L0X_GetRangingMeasurementData(device, &measurement);
    if (TofSensors_CheckApiStatus(l0Status) != HAL_OK) {
      return HAL_ERROR;
    }
    l0Status = VL53L0X_ClearInterruptMask(device, 0U);
    if (TofSensors_CheckApiStatus(l0Status) != HAL_OK) {
      return HAL_ERROR;
    }
    if (measurement.RangeStatus == 0U) {
      distanceMm[TOF_SENSOR_LEFT] = measurement.RangeMilliMeter;
    }
  }

  dataReady = 0U;
  lastL1Error = VL53L1X_CheckForDataReady(
      (uint16_t)(TOF_SENSOR_MID_I2C_ADDRESS << 1U), &dataReady);
  if (lastL1Error != 0) {
    return HAL_ERROR;
  }
  if (dataReady != 0U) {
    lastL1Error = VL53L1X_GetRangeStatus(
        (uint16_t)(TOF_SENSOR_MID_I2C_ADDRESS << 1U), &rangeStatus);
    if (lastL1Error != 0) {
      return HAL_ERROR;
    }
    lastL1Error = VL53L1X_GetDistance(
        (uint16_t)(TOF_SENSOR_MID_I2C_ADDRESS << 1U), &midDistance);
    if (lastL1Error != 0) {
      return HAL_ERROR;
    }
    lastL1Error = VL53L1X_ClearInterrupt(
        (uint16_t)(TOF_SENSOR_MID_I2C_ADDRESS << 1U));
    if (lastL1Error != 0) {
      return HAL_ERROR;
    }
    if (rangeStatus == 0U) {
      distanceMm[TOF_SENSOR_MID] = midDistance;
    }
  }

  device = &devices[TOF_SENSOR_RIGHT].driver;
  dataReady = 0U;
  l0Status = VL53L0X_GetMeasurementDataReady(device, &dataReady);
  if (TofSensors_CheckApiStatus(l0Status) != HAL_OK) {
    return HAL_ERROR;
  }
  if (dataReady != 0U) {
    l0Status = VL53L0X_GetRangingMeasurementData(device, &measurement);
    if (TofSensors_CheckApiStatus(l0Status) != HAL_OK) {
      return HAL_ERROR;
    }
    l0Status = VL53L0X_ClearInterruptMask(device, 0U);
    if (TofSensors_CheckApiStatus(l0Status) != HAL_OK) {
      return HAL_ERROR;
    }
    if (measurement.RangeStatus == 0U) {
      distanceMm[TOF_SENSOR_RIGHT] = measurement.RangeMilliMeter;
    }
  }

  *leftMm = distanceMm[TOF_SENSOR_LEFT];
  *midMm = distanceMm[TOF_SENSOR_MID];
  *rightMm = distanceMm[TOF_SENSOR_RIGHT];
  return HAL_OK;
}

HAL_StatusTypeDef TofSensors_ScanI2c(uint8_t *foundMask) {
  if (foundMask == NULL) {
    return HAL_ERROR;
  }

  *foundMask = 0U;
  for (uint8_t ledNumber = 1U; ledNumber <= 3U; ledNumber++) {
    Led_SetPattern(ledNumber, 0U, 0U, 0U, 0U);
  }
  Led_Update();

  TofXshut_Init();
  HAL_Delay(TOF_XSHUT_RESET_DELAY_MS);

  for (uint32_t i = 0U; i < TOF_SENSOR_COUNT; i++) {
    TofXshut_Sensor sensor = (TofXshut_Sensor)i;
    HAL_StatusTypeDef status;

    lastInitSensor = sensor;
    for (uint32_t other = 0U; other < TOF_SENSOR_COUNT; other++) {
      if (other != i && TofXshut_Set((TofXshut_Sensor)other, false) != HAL_OK) {
        TofXshut_Init();
        return HAL_ERROR;
      }
    }
    if (TofXshut_Set(sensor, true) != HAL_OK) {
      TofXshut_Init();
      return HAL_ERROR;
    }

    HAL_Delay(TOF_SENSOR_BOOT_DELAY_MS);
    status = I2cBus_IsDeviceReady(VL53L0X_DEFAULT_I2C_ADDRESS, 3U);
    if (status == HAL_OK) {
      *foundMask |= (uint8_t)(1U << (uint32_t)sensor);
      Led_SetPattern(TofSensors_GetLedNumber(sensor), 1U, 100U, 0U, 0U);
      Led_Update();
    } else if (status != HAL_ERROR) {
      TofXshut_Init();
      return status;
    }

    if (TofXshut_Set(sensor, false) != HAL_OK) {
      TofXshut_Init();
      return HAL_ERROR;
    }
    HAL_Delay(TOF_XSHUT_RESET_DELAY_MS);
  }

  TofXshut_Init();
  return HAL_OK;
}

VL53L0X_Error TofSensors_GetLastError(void) {
  return lastError;
}

int8_t TofSensors_GetLastL1Error(void) {
  return lastL1Error;
}

TofSensors_InitStage TofSensors_GetLastInitStage(void) {
  return lastInitStage;
}

TofXshut_Sensor TofSensors_GetLastInitSensor(void) {
  return lastInitSensor;
}

VL53L0X_DEV TofSensors_GetDevice(TofXshut_Sensor sensor) {
  if ((uint32_t)sensor >= TOF_SENSOR_COUNT || sensor == TOF_SENSOR_MID
      || devices[sensor].initialized == 0U) {
    return NULL;
  }

  return &devices[sensor].driver;
}
