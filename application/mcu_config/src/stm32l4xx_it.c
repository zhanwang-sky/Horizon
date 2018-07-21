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
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void) {
    return;
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void) {
    /* Go to infinite loop when Hard Fault exception occurs */
    while(1);
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void) {
    /* Go to infinite loop when Memory Manage exception occurs */
    while(1);
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void) {
    /* Go to infinite loop when Bus Fault exception occurs */
    while(1);
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void) {
    /* Go to infinite loop when Usage Fault exception occurs */
    while(1);
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
/*void SVC_Handler(void) {
    return;
}*/

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void) {
    return;
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
/*void PendSV_Handler(void) {
    return;
}*/

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
/*void SysTick_Handler(void) {
    HAL_SYSTICK_IRQHandler();
}*/

/******************************************************************************/
/*                 STM32L4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32l4xxxx.s).                                             */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @brief  This function handles TIM7 global interrupt.
  * @param  None
  * @retval None
*/
void TIM7_IRQHandler(void) {
    extern TIM_HandleTypeDef htim7;

    HAL_TIM_IRQHandler(&htim7);
}

/******************************** END OF FILE *********************************/
