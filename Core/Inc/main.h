/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cmsis_os.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#define NUM_ACTIVE_SERVOS 2
#define MEASUREMENT_ERROR -1.0f

#define TIM_SERVO_0    	(&htim1)
#define TIM_SERVO_1   	(&htim4)
/*
#define TIM_SERVO_2	    (&htim1)
#define TIM_SERVO_3 	(&htim4)
#define TIM_SERVO_4 	(&htim4)
*/

#define TIM_CHANNEL_SERVO_0   TIM_CHANNEL_4
#define TIM_CHANNEL_SERVO_1   TIM_CHANNEL_4
/*
#define TIM_CHANNEL_SERVO_2   TIM_CHANNEL_4
#define TIM_CHANNEL_SERVO_3   TIM_CHANNEL_4
#define TIM_CHANNEL_SERVO_4   TIM_CHANNEL_2
*/

extern TIM_HandleTypeDef* servo_htim[NUM_ACTIVE_SERVOS];

extern uint32_t servo_channel[NUM_ACTIVE_SERVOS];
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim4;

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define US1_TR_Pin GPIO_PIN_3
#define US1_TR_GPIO_Port GPIOE
#define LCD_BL_CTRL_Pin GPIO_PIN_15
#define LCD_BL_CTRL_GPIO_Port GPIOG
#define US0_TR_Pin GPIO_PIN_5
#define US0_TR_GPIO_Port GPIOG
#define US2_TR_Pin GPIO_PIN_4
#define US2_TR_GPIO_Port GPIOG
#define US2_EC_Pin GPIO_PIN_6
#define US2_EC_GPIO_Port GPIOF
#define RENDER_TIME_Pin GPIO_PIN_3
#define RENDER_TIME_GPIO_Port GPIOG
#define US1_EC_Pin GPIO_PIN_8
#define US1_EC_GPIO_Port GPIOF
#define US0_EC_Pin GPIO_PIN_9
#define US0_EC_GPIO_Port GPIOF
#define SM1_Pin GPIO_PIN_15
#define SM1_GPIO_Port GPIOD
#define MCU_ACTIVE_Pin GPIO_PIN_15
#define MCU_ACTIVE_GPIO_Port GPIOB
#define FRAME_RATE_Pin GPIO_PIN_14
#define FRAME_RATE_GPIO_Port GPIOB
#define LCD_DISP_Pin GPIO_PIN_10
#define LCD_DISP_GPIO_Port GPIOD
#define RLED_Pin GPIO_PIN_2
#define RLED_GPIO_Port GPIOC
#define GLED_Pin GPIO_PIN_3
#define GLED_GPIO_Port GPIOC
#define SM0_Pin GPIO_PIN_14
#define SM0_GPIO_Port GPIOE
#define VSYNC_FREQ_Pin GPIO_PIN_0
#define VSYNC_FREQ_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
