/**
  ******************************************************************************
  * @file    main.c
  * @author  Ji Chen
  * @brief   Main program body
  ******************************************************************************
  * @attention
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <string.h>

#include "stm32l4xx_hal.h"

#include "stm32l4xx_mcu_init.h"
#include "stm32l4xx_gen_uart.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "timers.h"

#include "inv_mpu.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
SemaphoreHandle_t xSem_i2c1RxCplt;
SemaphoreHandle_t xSem_b1Event;
SemaphoreHandle_t xSem_mpuEvent;
TimerHandle_t xTimer_blink;

/* Private function prototypes -----------------------------------------------*/
void blinkLED(void *pvParameters);
void printHello(void *pvParameters);

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void) {
    /* Initialize MCU */
    MX_MCU_Init();

    // test HAL_Delay();
    HAL_Delay(100);

    // Initialize MPU
    mpu_reset();
    mpu_set_gyro_fsr(1000);
    mpu_set_accel_fsr(4);
    mpu_set_lpf(12);
    mpu_set_sample_rate(25);
    mpu_set_int(1);

    // FreeRTOS
    gen_uart_init();
    xSem_i2c1RxCplt = xSemaphoreCreateBinary();
    xSem_b1Event = xSemaphoreCreateBinary();
    xSem_mpuEvent = xSemaphoreCreateBinary();

    xTimer_blink = xTimerCreate("blinkLED",
                                pdMS_TO_TICKS(1000),
                                pdTRUE,
                                NULL,
                                blinkLED);
    xTimerStart(xTimer_blink, 0);

    xTaskCreate(printHello,
                "printHello",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 2,
                NULL);

    /* Start the scheduler. */
    vTaskStartScheduler();

    /* If all is well, the scheduler will now be running, and the following line
       will never be reached. 
       If the following line does execute, then there was insufficient FreeRTOS
       heap memory available for the idle and/or timer tasks to be created.
       See the memory management section on the FreeRTOS web site for more
       details. */
    while(1);
}

void blinkLED(TimerHandle_t xTimer) {
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
}

void printHello(void *pvParameters) {
    static uint8_t mpuBuf[14] = { 0 };
    static uint16_t mpuData[7] = { 0 };
    static char uartTxBuf[128] = { 0 };

    while (1) {
        configASSERT(pdTRUE == xSemaphoreTake(xSem_mpuEvent, portMAX_DELAY));
        mpu_read_data_dma(mpuBuf);
        configASSERT(pdTRUE == xSemaphoreTake(xSem_i2c1RxCplt, pdMS_TO_TICKS(2)));
        mpu_convert_data(mpuBuf, mpuData);
        snprintf(uartTxBuf, sizeof(uartTxBuf),
                 "\033cG6\r\n%6hd\r\n%6hd\r\n%6hd\r\n%6hd\r\n%6hd\r\n%6hd\r\n%hu\r\n",
                 mpuData[0], mpuData[1], mpuData[2], mpuData[3], mpuData[4], mpuData[5], mpuData[6]);
        gen_uart_write(1, uartTxBuf, strlen(uartTxBuf));
    }
}

/**
  * @brief  Memory end of read transfer callback.
  * @param  hi2c I2C handle.
  * @retval None
  */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        if (I2C1 == hi2c->Instance) {
            xSemaphoreGiveFromISR(xSem_i2c1RxCplt, NULL);
        }
    }
}

/**
  * @brief  EXTI line detection callback.
  * @param  GPIO_Pin: Specifies the port pin connected to corresponding EXTI line.
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        if (GPIO_PIN_9 == GPIO_Pin) {
            xSemaphoreGiveFromISR(xSem_mpuEvent, NULL);
        } else if (GPIO_PIN_13 == GPIO_Pin) {
            xSemaphoreGiveFromISR(xSem_b1Event, NULL);
        }
    }
}

/******************************** END OF FILE *********************************/
