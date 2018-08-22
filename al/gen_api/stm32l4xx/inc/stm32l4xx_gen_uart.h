/**
  ******************************************************************************
  * @file    stm32l4xx_gen_uart.h
  * @author  Ji Chen
  * @brief   generic uart header file.
  ******************************************************************************
  * @attention
  ******************************************************************************
  */

#ifndef STM32L4XX_GEN_UART_H
#define STM32L4XX_GEN_UART_H

/* Function prototypes -------------------------------------------------------*/
int gen_uart_init(void);
int gen_uart_write(int fd, const void *buf, size_t nbytes);

#endif /* STM32L4XX_GEN_UART_H */

/******************************** END OF FILE *********************************/
