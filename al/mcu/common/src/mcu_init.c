/**
  ******************************************************************************
  * @file    mcu_init.c
  * @author  Ji Chen
  * @brief   Initialize MCU:
  *          configure clocks,
  *          initialize on-chip peripherals (timer, UART, I2C, ...),
  *          enable interrupts,
  *          ...
  ******************************************************************************
  * @attention
  ******************************************************************************
  */

#include "stm32l4xx_hal.h"

/* Global variables ----------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart2;

/* Functions -----------------------------------------------------------------*/
/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follows:
  *          + System Clock source = PLL (HSI)
  *          + SYSCLK(Hz)          = 80000000
  *          + HCLK(Hz)            = 80000000
  *          + AHB Prescaler       = 1
  *          + APB1 Prescaler      = 1
  *          + APB2 Prescaler      = 1
  *          + HSI Frequency(Hz)   = 16000000
  *          + PLL_M               = 2
  *          + PLL_N               = 20
  *          + PLL_P               = 7
  *          + PLL_Q               = 4
  *          + PLL_R               = 2
  *          + Flash Latency(WS)   = 4
  * @param  None
  * @retval None
  */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /* Initializes the CPU, AHB and APB busses clocks */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 2;
    RCC_OscInitStruct.PLL.PLLN = 20;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV4;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        while(1);
    }

    /* Initializes the CPU, AHB and APB busses clocks */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
        while(1);
    }

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2 | RCC_PERIPHCLK_I2C1;
    PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
    PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
        while(1);
    }

    /* Configure the main internal regulator output voltage */
    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK) {
        while(1);
    }

    /* Configure the Systick interrupt time */
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000U);

    /* Configure the Systick */
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    /* SysTick_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(SysTick_IRQn, SYSTICK_INT_PRIORITY, 0);
}

/**
  * @brief  GPIO Initiation routine
  * @param  None
  * @retval None
  */
void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct;

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* GPIOA configuration */
    /* Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    /* Configure GPIO pin: PA5(LD2-LED) */
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* GPIOC configuration */
    /* Configure GPIO pin: PC9(MPU_INT) */
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    /* Configure GPIO pin: PC13(B1-USER) */
    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* EXTI interrupt init */
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, SYSTICK_INT_PRIORITY - 1U, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, SYSTICK_INT_PRIORITY - 1U, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/**
  * @brief  DMA Initiation function
  * @param  None
  * @retval None
  */
void MX_DMA_Init(void) {
    /* DMA controller clock enable */
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();

    /* DMA1_Channel6_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, SYSTICK_INT_PRIORITY - 2U, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
    /* DMA1_Channel7_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, SYSTICK_INT_PRIORITY - 2U, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);
    /* DMA2_Channel6_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA2_Channel6_IRQn, SYSTICK_INT_PRIORITY - 2U, 0);
    HAL_NVIC_EnableIRQ(DMA2_Channel6_IRQn);
    /* DMA2_Channel7_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA2_Channel7_IRQn, SYSTICK_INT_PRIORITY - 2U, 0);
    HAL_NVIC_EnableIRQ(DMA2_Channel7_IRQn);
}

/**
  * @brief  I2C1 Initiation function
  * @param  None
  * @retval None
  */
void MX_I2C1_Init(void) {
    hi2c1.Instance = I2C1;
    hi2c1.Init.Timing = 0x00702991;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
        while(1);
    }

    /* Configure Analogue filter */
    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK) {
        while(1);
    }

    /* Configure Digital filter */
    if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) {
        while(1);
    }
}

/**
  * @brief  UART2 Initiation routine
  * @param  None
  * @retval None
  */
void MX_USART2_UART_Init(void) {
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart2) != HAL_OK) {
        while(1);
    }
}

void MX_MCU_Init(void) {
    /* STM32L4xx HAL library initialization:
       - Configure the Flash prefetch, Flash preread and Buffer caches.
       - Systick timer is configured by default as source of time base, but user
         can eventually implement his proper time base source (a general purpose
         timer for example or other time source), keeping in mind that Time base
         duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and
         handled in milliseconds basis.
       - Low Level Initialization
    */
    HAL_Init();

    /* Configure the System clock to have a frequency of 80 MHz */
    SystemClock_Config();

    /* Initialize on-chip peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_I2C1_Init();
    MX_USART2_UART_Init();

    return;
}

#ifdef USE_FULL_ASSERT
void HAL_Assert_Failed(void) {
    /* disable all interrupts */
    __disable_irq();
    /* turn off LED */
    GPIOA->BRR = (uint32_t) GPIO_PIN_5;
    /* stop here */
    while(1);
}
#endif /* end of USE_FULL_ASSERT */

/* Interrupt callbacks -------------------------------------------------------*/
/**
  * @brief  Period elapsed callback in non-blocking mode.
  * @note   This function is called when TIM7 interrupt took place, inside
  *         HAL_TIM_IRQHandler().
  *         It makes a direct call to HAL_IncTick() to increment a global
  *         variable "uwTick" used as application time base.
  * @param  htim: TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (TIM7 == htim->Instance) {
        HAL_IncTick();
    }
}

/******************************** END OF FILE *********************************/
