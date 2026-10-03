#include "main.h"
#include "tof_xshut.h"

typedef struct {
  GPIO_TypeDef *port;
  uint16_t pin;
} TofXshut_Pin;

static const TofXshut_Pin sensorPins[TOF_SENSOR_COUNT] = {
  {TOF_XSHUT_RIGHT_Port, TOF_XSHUT_RIGHT},
  {TOF_XSHUT_MID_Port, TOF_XSHUT_MID},
  {TOF_XSHUT_LEFT_Port, TOF_XSHUT_LEFT}
};

void TofXshut_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOB_CLK_ENABLE();
  for (uint32_t i = 0U; i < TOF_SENSOR_COUNT; i++) {
    HAL_GPIO_WritePin(sensorPins[i].port, sensorPins[i].pin, GPIO_PIN_RESET);
  }

  GPIO_InitStruct.Pin = TOF_XSHUT_RIGHT|TOF_XSHUT_MID|TOF_XSHUT_LEFT;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TOF_XSHUT_RIGHT_Port, &GPIO_InitStruct);
}

HAL_StatusTypeDef TofXshut_Set(TofXshut_Sensor sensor, bool enabled) {
  if ((uint32_t)sensor >= TOF_SENSOR_COUNT) {
    return HAL_ERROR;
  }

  HAL_GPIO_WritePin(sensorPins[sensor].port, sensorPins[sensor].pin, enabled ? GPIO_PIN_SET : GPIO_PIN_RESET);
  return HAL_OK;
}
