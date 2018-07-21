/**
  ******************************************************************************
  * @file    stm32l4xx_hal_timebase_tim.c
  * @author  Ji Chen
  * @brief   This file override the native HAL time base functions (defined as weak)
  *          the TIM time base:
  *           + Intializes the TIM peripheral to generate a Period elapsed Event each 1ms
  *           + HAL_IncTick is called inside HAL_TIM_PeriodElapsedCallback ie each 1ms
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
TIM_HandleTypeDef htim7;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  This function configures the TIM7 as a time base source.
  *         The time source is configured to have 1ms time base with a dedicated
  *         Tick interrupt priority.
  * @note   This function is called automatically at the beginning of program
  *         after reset by HAL_Init() or at any time when clock is configured,
  *         by HAL_RCC_ClockConfig().
  * @param  TickPriority: Tick interrupt priority.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority) {
    RCC_ClkInitTypeDef clkconfig;
    uint32_t           pFLatency;
    uint32_t           uwTimclock = 0;
    uint32_t           uwPrescalerValue = 0;

    /*Configure the TIM7 IRQ priority */
    HAL_NVIC_SetPriority(TIM7_IRQn, TickPriority ,0); 

    /* Enable the TIM7 global Interrupt */
    HAL_NVIC_EnableIRQ(TIM7_IRQn); 

    /* Enable TIM7 clock */
    __HAL_RCC_TIM7_CLK_ENABLE();

    /* Get clock configuration */
    HAL_RCC_GetClockConfig(&clkconfig, &pFLatency);

    /* Compute TIM7 clock */
    uwTimclock = HAL_RCC_GetPCLK1Freq();

    /* Compute the prescaler value to have TIM7 counter clock equal to 1MHz */
    uwPrescalerValue = (uint32_t) ((uwTimclock / 1000000U) - 1U);

    /* Initialize TIM7 */
    htim7.Instance = TIM7;

    /* Initialize TIMx peripheral as follow:
        + Period = [(TIM7CLK / 1000) - 1] to have a (1 / 1000) sec time base.
        + Prescaler = [(uwTimclock / 1000000) - 1] to have a 1MHz counter clock.
        + ClockDivision = 0
        + Counter direction = Up
    */
    htim7.Init.Period = 1000U - 1U;
    htim7.Init.Prescaler = uwPrescalerValue;
    htim7.Init.ClockDivision = 0U;
    htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
    if(HAL_OK == HAL_TIM_Base_Init(&htim7)) {
        /* Start the TIM time Base generation in interrupt mode */
        return HAL_TIM_Base_Start_IT(&htim7);
    }

    /* Return function status */
    return HAL_ERROR;
}

/**
  * @brief  Suspend Tick increment
  * @note   Disable the tick increment by disabling TIM7 update interrupt.
  * @param  None
  * @retval None
  */
void HAL_SuspendTick(void) {
    /* Disable TIM7 update interrupt */
    __HAL_TIM_DISABLE_IT(&htim7, TIM_IT_UPDATE);
}

/**
  * @brief  Resume Tick increment
  * @note   Enable the tick increment by enabling TIM7 update interrupt.
  * @param  None
  * @retval None
  */
void HAL_ResumeTick(void) {
    /* Enable TIM7 update interrupt */
    __HAL_TIM_ENABLE_IT(&htim7, TIM_IT_UPDATE);
}

/******************************** END OF FILE *********************************/
