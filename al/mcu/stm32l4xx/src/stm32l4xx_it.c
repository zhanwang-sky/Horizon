/**
  ******************************************************************************
  * @file    stm32l4xx_it.c
  * @author  Ji Chen
  * @brief   Main Interrupt Service Routines
  ******************************************************************************
  * @attention
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim7;
extern DMA_HandleTypeDef hdma_i2c1_rx;
extern DMA_HandleTypeDef hdma_i2c1_tx;

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void) {
    HAL_SYSTICK_IRQHandler();
}

/******************************************************************************/
/*                 STM32L4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32l4xxxx.s).                                             */
/******************************************************************************/

/**
  * @brief  This function handles DMA1_Channel6 interrupt request.
  * @param  None
  * @retval None
  */
void DMA1_Channel6_IRQHandler(void) {
    HAL_DMA_IRQHandler(&hdma_usart2_rx);
}

/**
  * @brief  This function handles DMA1_Channel7 interrupt request.
  * @param  None
  * @retval None
  */
void DMA1_Channel7_IRQHandler(void) {
    HAL_DMA_IRQHandler(&hdma_usart2_tx);
}

/**
  * @brief  This function handles EXTI line[9:5] interrupts.
  * @param  None
  * @retval None
  */
void EXTI9_5_IRQHandler(void) {
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_9);
}

/**
  * @brief  This function handles I2C1 event interrupt.
  * @param  None
  * @retval None
  */
void I2C1_EV_IRQHandler(void) {
    HAL_I2C_EV_IRQHandler(&hi2c1);
}

/**
  * @brief  This function handles I2C1 error interrupt.
  * @param  None
  * @retval None
  */
void I2C1_ER_IRQHandler(void) {
    HAL_I2C_ER_IRQHandler(&hi2c1);
}

/**
  * @brief  This function handles UART interrupt request.
  * @param  None
  * @retval None
  */
void USART2_IRQHandler(void) {
    HAL_UART_IRQHandler(&huart2);
}

/**
  * @brief  This function handles EXTI line[15:10] interrupts.
  * @param  None
  * @retval None
  */
void EXTI15_10_IRQHandler(void) {
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
}

/**
  * @brief  This function handles TIM7 global interrupt.
  * @param  None
  * @retval None
*/
void TIM7_IRQHandler(void) {
    HAL_TIM_IRQHandler(&htim7);
}

/**
  * @brief  This function handles DMA2 channel6 global interrupt.
  * @param  None
  * @retval None
*/
void DMA2_Channel6_IRQHandler(void) {
    HAL_DMA_IRQHandler(&hdma_i2c1_rx);
}

/**
  * @brief  This function handles DMA2 channel7 global interrupt.
  * @param  None
  * @retval None
*/
void DMA2_Channel7_IRQHandler(void) {
    HAL_DMA_IRQHandler(&hdma_i2c1_tx);
}

/******************************** END OF FILE *********************************/
