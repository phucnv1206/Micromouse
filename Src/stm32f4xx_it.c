
#include "main.h"
#include "stm32f4xx_it.h"

extern DMA_HandleTypeDef hdma_i2c1_rx;
extern DMA_HandleTypeDef hdma_i2c1_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart2;

void NMI_Handler(void)
{

   while (1)
  {
  }

}

void HardFault_Handler(void)
{

  while (1)
  {

  }
}

void MemManage_Handler(void)
{

  while (1)
  {

  }
}

void BusFault_Handler(void)
{

  while (1)
  {

  }
}

void UsageFault_Handler(void)
{

  while (1)
  {

  }
}

void SVC_Handler(void)
{

}

void DebugMon_Handler(void)
{

}

void PendSV_Handler(void)
{

}

void SysTick_Handler(void)
{

  HAL_IncTick();

}

void EXTI4_IRQHandler(void)
{

  HAL_GPIO_EXTI_IRQHandler(EN_LEFT_A);

}

void DMA1_Stream0_IRQHandler(void)
{

  HAL_DMA_IRQHandler(&hdma_i2c1_rx);

}

void I2C1_EV_IRQHandler(void)
{

  HAL_I2C_EV_IRQHandler(&hi2c1);

}

void I2C1_ER_IRQHandler(void)
{

  HAL_I2C_ER_IRQHandler(&hi2c1);

}

void DMA1_Stream5_IRQHandler(void)
{

  HAL_DMA_IRQHandler(&hdma_usart2_rx);

}

void DMA1_Stream6_IRQHandler(void)
{

  HAL_DMA_IRQHandler(&hdma_usart2_tx);

}

void EXTI9_5_IRQHandler(void)
{

  HAL_GPIO_EXTI_IRQHandler(EN_LEFT_B);

}

void USART2_IRQHandler(void)
{

  HAL_UART_IRQHandler(&huart2);

}

void EXTI15_10_IRQHandler(void)
{

  HAL_GPIO_EXTI_IRQHandler(EN_RIGHT_B);
  HAL_GPIO_EXTI_IRQHandler(EN_RIGHT_A);

}

void DMA1_Stream7_IRQHandler(void)
{

  HAL_DMA_IRQHandler(&hdma_i2c1_tx);

}
