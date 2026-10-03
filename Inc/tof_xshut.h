#ifndef MICROMOUSE_TOF_XSHUT_H
#define MICROMOUSE_TOF_XSHUT_H

#include <stdbool.h>
#include "stm32f4xx_hal.h"

typedef enum {
  /* Các nhãn là vị trí cảm biến trên xe; mỗi chân XSHUT được điều khiển độc lập. */
  TOF_SENSOR_RIGHT = 0U,
  TOF_SENSOR_MID,
  TOF_SENSOR_LEFT,
  TOF_SENSOR_COUNT
} TofXshut_Sensor;

/*
 * Khởi tạo PB13/PB14/PB15 và giữ cả ba cảm biến ở trạng thái shutdown.
 * Khi gán địa chỉ I2C sau này, bật lần lượt từng cảm biến, đổi địa chỉ của nó,
 * rồi mới bật cảm biến kế tiếp để không bị trùng địa chỉ mặc định 0x29.
 */
void TofXshut_Init(void);
/* enabled=true kéo XSHUT lên mức cao để bật cảm biến; false giữ cảm biến reset. */
HAL_StatusTypeDef TofXshut_Set(TofXshut_Sensor sensor, bool enabled);

#endif
