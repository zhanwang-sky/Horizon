/**
  ******************************************************************************
  * @file    utilities.c
  * @author  Ji Chen
  * @brief   Provide system level ISR (systick) and FreeRTOS assertion function.
  ******************************************************************************
  * @attention
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

#include "FreeRTOS.h"
#include "task.h"

/* Function prototypes -------------------------------------------------------*/
void xPortSysTickHandler( void );

/* Functions -----------------------------------------------------------------*/
#ifdef configASSERT
void os_assert_failed(void) {
    taskDISABLE_INTERRUPTS();
    GPIOA->BRR = (uint32_t) GPIO_PIN_5;
    while(1);
}
#endif /* end of configASSERT */

/* Interrupt service routines ------------------------------------------------*/
/**
  * @brief  SYSTICK callback.
  * @param  None
  * @retval None
  */
void HAL_SYSTICK_Callback(void) {
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        xPortSysTickHandler();
    }
}

/******************************** END OF FILE *********************************/
