/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    i2c.c
  * @brief   This file provides code for the configuration
  *          of the I2C instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "i2c.h"

/* USER CODE BEGIN 0 */
#include "usart.h"
#include <string.h>
/* USER CODE END 0 */

I2C_HandleTypeDef hi2c2;

/* I2C2 init function */
void MX_I2C2_Init(void)
{
    hi2c2.Instance = I2C2;
    hi2c2.Init.ClockSpeed = 400000;
    hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c2.Init.OwnAddress1 = 0;
    hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c2.Init.OwnAddress2 = 0;
    hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c2) != HAL_OK)
    {
        Error_Handler();
    }
}

void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle)
{

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if (i2cHandle->Instance == I2C2)
    {
        /* USER CODE BEGIN I2C2_MspInit 0 */

        /* USER CODE END I2C2_MspInit 0 */

        __HAL_RCC_GPIOB_CLK_ENABLE();
        /**I2C2 GPIO Configuration
        PB10     ------> I2C2_SCL
        PB11     ------> I2C2_SDA
        */
        GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        /* I2C2 clock enable */
        __HAL_RCC_I2C2_CLK_ENABLE();
        /* USER CODE BEGIN I2C2_MspInit 1 */

        /* USER CODE END I2C2_MspInit 1 */
    }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* i2cHandle)
{

    if (i2cHandle->Instance == I2C2)
    {
        /* USER CODE BEGIN I2C2_MspDeInit 0 */

        /* USER CODE END I2C2_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_I2C2_CLK_DISABLE();

        /**I2C2 GPIO Configuration
        PB10     ------> I2C2_SCL
        PB11     ------> I2C2_SDA
        */
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10);

        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_11);

        /* USER CODE BEGIN I2C2_MspDeInit 1 */

        /* USER CODE END I2C2_MspDeInit 1 */
    }
}

/* USER CODE BEGIN 1 */

#define I2C2_FAIL_LIMIT 3u

static uint8_t i2c_fail_streak;
static uint32_t i2c_log_tick;

static void i2c_bit_delay(void)
{
    volatile uint32_t cycles = 200;
    while (cycles--) {
    }
}

static void i2c_log(const char *msg, int force)
{
    uint32_t now = HAL_GetTick();
    if (!force && i2c_log_tick != 0u && (now - i2c_log_tick) < 200u) {
        return;
    }
    i2c_log_tick = now;
    HAL_UART_Transmit(&huart1, (uint8_t *)msg, (uint16_t)strlen(msg), 20);
}

/* STM32F1 leaves I2C_SR2.BUSY set after a stuck slave or an aborted byte.
 * DeInit, clock the bus out, then re-init. Caller holds the I2C mutex. */
static void I2C2_BusRecover(void)
{
    GPIO_InitTypeDef gpio = {0};
    uint8_t pulse;

    HAL_I2C_DeInit(&hi2c2);
    __HAL_RCC_GPIOB_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_10 | GPIO_PIN_11;
    gpio.Mode = GPIO_MODE_OUTPUT_OD;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10 | GPIO_PIN_11, GPIO_PIN_SET);
    i2c_bit_delay();
    for (pulse = 0; pulse < 9u; pulse++) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
        i2c_bit_delay();
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
        i2c_bit_delay();
    }
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
    i2c_bit_delay();
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
    i2c_bit_delay();
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
    i2c_bit_delay();
    MX_I2C2_Init();
}

int I2C2_Check(HAL_StatusTypeDef status, const char *op)
{
    if (status == HAL_OK) {
        i2c_fail_streak = 0;
        return 0;
    }
    if (++i2c_fail_streak >= I2C2_FAIL_LIMIT) {
        i2c_fail_streak = 0;
        i2c_log("I2C BUS RECOVER\r\n", 1);
        I2C2_BusRecover();
        return 1;
    }
    if (op != NULL) {
        i2c_log(op, 0);
    }
    return 0;
}

/* USER CODE END 1 */
