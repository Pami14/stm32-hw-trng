/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

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
#define I2C_Reset_Pin GPIO_PIN_1
#define I2C_Reset_GPIO_Port GPIOA
#define MuxOut_1_Pin GPIO_PIN_4
#define MuxOut_1_GPIO_Port GPIOC
#define MuxOut_2_Pin GPIO_PIN_5
#define MuxOut_2_GPIO_Port GPIOC
#define MuxOut_3_Pin GPIO_PIN_0
#define MuxOut_3_GPIO_Port GPIOB
#define MuxOut_4_Pin GPIO_PIN_1
#define MuxOut_4_GPIO_Port GPIOB
#define BIT_0_Pin GPIO_PIN_2
#define BIT_0_GPIO_Port GPIOB
#define BIT_1_Pin GPIO_PIN_7
#define BIT_1_GPIO_Port GPIOE
#define BIT_2_Pin GPIO_PIN_8
#define BIT_2_GPIO_Port GPIOE
#define BIT_3_Pin GPIO_PIN_9
#define BIT_3_GPIO_Port GPIOE
#define BIT_4_Pin GPIO_PIN_10
#define BIT_4_GPIO_Port GPIOE
#define BIT_5_Pin GPIO_PIN_11
#define BIT_5_GPIO_Port GPIOE
#define BIT_6_Pin GPIO_PIN_12
#define BIT_6_GPIO_Port GPIOE
#define BIT_7_Pin GPIO_PIN_13
#define BIT_7_GPIO_Port GPIOE
#define MuxOut_5_Pin GPIO_PIN_14
#define MuxOut_5_GPIO_Port GPIOE
#define MuxOut_6_Pin GPIO_PIN_15
#define MuxOut_6_GPIO_Port GPIOE
#define MuxOut_7_Pin GPIO_PIN_13
#define MuxOut_7_GPIO_Port GPIOB
#define MuxOut_8_Pin GPIO_PIN_14
#define MuxOut_8_GPIO_Port GPIOB
#define SDIO_SCK_Pin GPIO_PIN_15
#define SDIO_SCK_GPIO_Port GPIOB
#define Column_1_Pin GPIO_PIN_8
#define Column_1_GPIO_Port GPIOD
#define Column_2_Pin GPIO_PIN_9
#define Column_2_GPIO_Port GPIOD
#define Column_3_Pin GPIO_PIN_10
#define Column_3_GPIO_Port GPIOD
#define Column_4_Pin GPIO_PIN_11
#define Column_4_GPIO_Port GPIOD
#define SD_CD_Pin GPIO_PIN_6
#define SD_CD_GPIO_Port GPIOC
#define SDIO_D0_Pin GPIO_PIN_8
#define SDIO_D0_GPIO_Port GPIOC
#define SDIO_D1_Pin GPIO_PIN_9
#define SDIO_D1_GPIO_Port GPIOC
#define LCD_I2C_SCL_Pin GPIO_PIN_8
#define LCD_I2C_SCL_GPIO_Port GPIOA
#define SDIO_D2_Pin GPIO_PIN_9
#define SDIO_D2_GPIO_Port GPIOA
#define USB_DM_Pin GPIO_PIN_11
#define USB_DM_GPIO_Port GPIOA
#define USB_DP_Pin GPIO_PIN_12
#define USB_DP_GPIO_Port GPIOA
#define SDIO_D3_Pin GPIO_PIN_11
#define SDIO_D3_GPIO_Port GPIOC
#define Row_1_Pin GPIO_PIN_0
#define Row_1_GPIO_Port GPIOD
#define Row_1_EXTI_IRQn EXTI0_IRQn
#define Row_2_Pin GPIO_PIN_1
#define Row_2_GPIO_Port GPIOD
#define Row_2_EXTI_IRQn EXTI1_IRQn
#define SDIO_CMD_Pin GPIO_PIN_2
#define SDIO_CMD_GPIO_Port GPIOD
#define Row_4_Pin GPIO_PIN_3
#define Row_4_GPIO_Port GPIOD
#define Row_4_EXTI_IRQn EXTI3_IRQn
#define Row_3_Pin GPIO_PIN_6
#define Row_3_GPIO_Port GPIOD
#define Row_3_EXTI_IRQn EXTI9_5_IRQn
#define MuxEnable_N_Pin GPIO_PIN_7
#define MuxEnable_N_GPIO_Port GPIOD
#define S0_OutSel_Pin GPIO_PIN_4
#define S0_OutSel_GPIO_Port GPIOB
#define S1_OutSel_Pin GPIO_PIN_5
#define S1_OutSel_GPIO_Port GPIOB
#define Multi_I2C_SCL_Pin GPIO_PIN_6
#define Multi_I2C_SCL_GPIO_Port GPIOB
#define Multi_I2C_SDA_Pin GPIO_PIN_7
#define Multi_I2C_SDA_GPIO_Port GPIOB
#define LCD_I2C_SDA_Pin GPIO_PIN_8
#define LCD_I2C_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
