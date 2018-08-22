/**
  ******************************************************************************
  * @file    stm32l4xx_gen_uart.c
  * @author  Ji Chen
  * @brief   A set of APIs to interface with UART.
  ******************************************************************************
  * @attention
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* External variables --------------------------------------------------------*/
extern UART_HandleTypeDef huart2;

/* Private macro -------------------------------------------------------------*/
#define NR_UART_BUSES (1)

/* Private variables ---------------------------------------------------------*/
SemaphoreHandle_t xSem_uartTx[NR_UART_BUSES];
SemaphoreHandle_t xSem_uartTxCplt[NR_UART_BUSES];

/* Functions -----------------------------------------------------------------*/
int gen_uart_init(void) {
    int rc = 0;

    for (int i = 0; i < NR_UART_BUSES; i++) {
        if (NULL == (xSem_uartTx[i] = xSemaphoreCreateMutex())) {
            rc = -1;
            break;
        }
        if (NULL == (xSem_uartTxCplt[i] = xSemaphoreCreateBinary())) {
            rc = -1;
            break;
        }
    }

    return rc;
}

int gen_uart_write(int fd, const void *buf, size_t nbytes) {
    UART_HandleTypeDef *phuart;
    int index;
    int rc = 0;

    if ((NULL == buf)
        || ((0 == nbytes) || (nbytes > (uint16_t) 0xFFFF))) {
        return -1;
    }

    if (1 == fd) {
        phuart = &huart2;
        index = 0;
    } else {
        return -1;
    }

    xSemaphoreTake(xSem_uartTx[index], portMAX_DELAY);

    if (HAL_UART_Transmit_DMA(phuart, (uint8_t *) buf, (uint16_t) nbytes) != HAL_OK) {
        rc = -1;
        goto EXIT;
    }

    xSemaphoreTake(xSem_uartTxCplt[index], portMAX_DELAY);

    if (phuart->ErrorCode != HAL_UART_ERROR_NONE) {
        rc = -1;
    }

EXIT:
    xSemaphoreGive(xSem_uartTx[index]);

    return rc;
}

/* Interrupt service routines ------------------------------------------------*/
/**
  * @brief  Tx Transfer completed callback.
  * @param  huart UART handle.
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        if (USART2 == huart->Instance) {
            xSemaphoreGiveFromISR(xSem_uartTxCplt[0], NULL);
        }
    }
}

/******************************** END OF FILE *********************************/
