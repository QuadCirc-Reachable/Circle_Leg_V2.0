/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_ACT_Pin GPIO_PIN_3
#define LED_ACT_GPIO_Port GPIOE
#define LASER_Pin GPIO_PIN_15
#define LASER_GPIO_Port GPIOC
#define NRST_Pin GPIO_PIN_10
#define NRST_GPIO_Port GPIOG
#define ADC_POWER_Pin GPIO_PIN_0
#define ADC_POWER_GPIO_Port GPIOC
#define ADC_5V_Pin GPIO_PIN_1
#define ADC_5V_GPIO_Port GPIOC
#define BUTTON_Pin GPIO_PIN_2
#define BUTTON_GPIO_Port GPIOC
#define BUZZER_Pin GPIO_PIN_3
#define BUZZER_GPIO_Port GPIOC
#define IMU_HEAT_Pin GPIO_PIN_2
#define IMU_HEAT_GPIO_Port GPIOF
#define UART2A_TX_Pin GPIO_PIN_2
#define UART2A_TX_GPIO_Port GPIOA
#define UART2A_RX_Pin GPIO_PIN_3
#define UART2A_RX_GPIO_Port GPIOA
#define LIS3_CS_Pin GPIO_PIN_4
#define LIS3_CS_GPIO_Port GPIOA
#define LIS3_DRY_Pin GPIO_PIN_0
#define LIS3_DRY_GPIO_Port GPIOB
#define LIS3_DRY_EXTI_IRQn EXTI0_IRQn
#define ICM_DRY_Pin GPIO_PIN_1
#define ICM_DRY_GPIO_Port GPIOB
#define ICM_DRY_EXTI_IRQn EXTI1_IRQn
#define ICM_CS_Pin GPIO_PIN_2
#define ICM_CS_GPIO_Port GPIOB
#define UART3B_RX_Pin GPIO_PIN_15
#define UART3B_RX_GPIO_Port GPIOE
#define UART3B_TX_Pin GPIO_PIN_10
#define UART3B_TX_GPIO_Port GPIOB
#define UART1C_RX_Pin GPIO_PIN_10
#define UART1C_RX_GPIO_Port GPIOA
#define UART4_DE_Pin GPIO_PIN_15
#define UART4_DE_GPIO_Port GPIOA
#define UART4_TX_Pin GPIO_PIN_10
#define UART4_TX_GPIO_Port GPIOC
#define UART4_RX_Pin GPIO_PIN_11
#define UART4_RX_GPIO_Port GPIOC
#define UART5_TX_Pin GPIO_PIN_12
#define UART5_TX_GPIO_Port GPIOC
#define UART5_RX_Pin GPIO_PIN_2
#define UART5_RX_GPIO_Port GPIOD
#define UART2B_TX_Pin GPIO_PIN_5
#define UART2B_TX_GPIO_Port GPIOD
#define UART2B_RX_Pin GPIO_PIN_6
#define UART2B_RX_GPIO_Port GPIOD
#define BOOT0_Pin GPIO_PIN_8
#define BOOT0_GPIO_Port GPIOB
#define UART1B_TX_Pin GPIO_PIN_0
#define UART1B_TX_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
