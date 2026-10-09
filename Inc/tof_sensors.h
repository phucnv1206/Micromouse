#ifndef MICROMOUSE_TOF_SENSORS_H
#define MICROMOUSE_TOF_SENSORS_H

#include <stdbool.h>
#include "tof_xshut.h"
#include "vl53l0x_api.h"

typedef enum {
  TOF_INIT_STAGE_NONE,
  TOF_INIT_STAGE_XSHUT,
  TOF_INIT_STAGE_I2C,
  TOF_INIT_STAGE_DEVICE_READY,
  TOF_INIT_STAGE_DATA_INIT,
  TOF_INIT_STAGE_SET_ADDRESS,
  TOF_INIT_STAGE_STATIC_INIT,
  TOF_INIT_STAGE_REF_CALIBRATION,
  TOF_INIT_STAGE_REF_SPAD,
  TOF_INIT_STAGE_SHORT_PRE_RANGE,
  TOF_INIT_STAGE_SHORT_FINAL_RANGE,
  TOF_INIT_STAGE_SIGNAL_RATE,
  TOF_INIT_STAGE_SET_MODE,
  TOF_INIT_STAGE_TIMING_BUDGET,
  TOF_INIT_STAGE_START_MEASUREMENT,
  TOF_INIT_STAGE_L1_SENSOR_INIT,
  TOF_INIT_STAGE_L1_SET_ADDRESS,
  TOF_INIT_STAGE_L1_SHORT_MODE,
  TOF_INIT_STAGE_L1_TIMING_BUDGET,
  TOF_INIT_STAGE_L1_INTER_MEASUREMENT,
  TOF_INIT_STAGE_L1_START_RANGING
} TofSensors_InitStage;

/*
 * Khởi tạo tuần tự cảm biến trái/phải VL53L0X và giữa VL53L1X, gán địa chỉ
 * riêng. Hàm này blocking do API ST thực hiện các bước khởi tạo đồng bộ.
 */
HAL_StatusTypeDef TofSensors_Init(void);
/*
 * Trả true khi đọc thành công và cung cấp khoảng cách đã lọc low-pass;
 * nếu ToF tắt, đặt các đầu ra về 0 và trả false.
 */
bool TofSensors_UpdateDistances(uint16_t *leftMm, uint16_t *midMm, uint16_t *rightMm);
/* Trả handle VL53L0X; cảm biến giữa VL53L1X không dùng handle này. */
VL53L0X_DEV TofSensors_GetDevice(TofXshut_Sensor sensor);
/* Mã lỗi gốc của API ST gần nhất; NONE nghĩa là API không báo lỗi. */
VL53L0X_Error TofSensors_GetLastError(void);
/* Mã lỗi API VL53L1X gần nhất; 0 nghĩa là API không báo lỗi. */
int8_t TofSensors_GetLastL1Error(void);
/* Giai đoạn và cảm biến cuối cùng đang khởi tạo; giữ nguyên khi init thất bại. */
TofSensors_InitStage TofSensors_GetLastInitStage(void);
TofXshut_Sensor TofSensors_GetLastInitSensor(void);

#endif
