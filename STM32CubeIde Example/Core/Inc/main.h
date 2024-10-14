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
#include "stm32f1xx_hal.h"
#include "stm32f1xx_ll_system.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_exti.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_cortex.h"
#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_utils.h"
#include "stm32f1xx_ll_pwr.h"
#include "stm32f1xx_ll_dma.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <ILI9341_STM32_Driver.h>
#include <ILI9341_GFX.h>
#include <stdio.h>
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define WIRE_SPEED_COUNTER_TIMER_US 20
#define WIRE_SPEED_COUNTER_TIMER_PRESCALER 1
#define PWM_AUTORELOAD_REG_ARR ((PWM_TIMER_US*FCLOCK_MHZ/(PWM_TIMER_PRESCALER+1))-1)
#define WIRE_SPEED_COUNTER_TIMER_ARR ((WIRE_SPEED_COUNTER_TIMER_US*FCLOCK_MHZ/(WIRE_SPEED_COUNTER_TIMER_PRESCALER+1))-1)
#define PWM_TIMER_US 1000
#define FCLOCK_MHZ 72
#define PWM_TIMER_PRESCALER 1
#define ADCS_CONV_TIMER_US 10000
#define ADCS_CONV_TIMER_PRESCALER 12
#define ADCS_CONV_TIMER_ARR ((ADCS_CONV_TIMER_US*FCLOCK_MHZ/(ADCS_CONV_TIMER_PRESCALER+1))-1)
#define OPT_SELECT_PUSH_BUTTON_Pin LL_GPIO_PIN_15
#define OPT_SELECT_PUSH_BUTTON_GPIO_Port GPIOC
#define WIRE_FEED_POT_IN_Pin LL_GPIO_PIN_0
#define WIRE_FEED_POT_IN_GPIO_Port GPIOA
#define OPT_POT_IN_Pin LL_GPIO_PIN_1
#define OPT_POT_IN_GPIO_Port GPIOA
#define PID_P_ADC_IN_Pin LL_GPIO_PIN_2
#define PID_P_ADC_IN_GPIO_Port GPIOA
#define PID_I_ADC_IN_Pin LL_GPIO_PIN_3
#define PID_I_ADC_IN_GPIO_Port GPIOA
#define PID_D_ADC_IN_Pin LL_GPIO_PIN_4
#define PID_D_ADC_IN_GPIO_Port GPIOA
#define TFT_CLK_Pin LL_GPIO_PIN_5
#define TFT_CLK_GPIO_Port GPIOA
#define MIG_BUTTON_IN_Pin LL_GPIO_PIN_6
#define MIG_BUTTON_IN_GPIO_Port GPIOA
#define TFT_MOSI_Pin LL_GPIO_PIN_7
#define TFT_MOSI_GPIO_Port GPIOA
#define TFT_DC_Pin LL_GPIO_PIN_0
#define TFT_DC_GPIO_Port GPIOB
#define TFT_RESET_Pin LL_GPIO_PIN_1
#define TFT_RESET_GPIO_Port GPIOB
#define TFT_CS_Pin LL_GPIO_PIN_10
#define TFT_CS_GPIO_Port GPIOB
#define OPTO_IN_Pin LL_GPIO_PIN_11
#define OPTO_IN_GPIO_Port GPIOB
#define PID_DIP_SW_BIT1_Pin LL_GPIO_PIN_12
#define PID_DIP_SW_BIT1_GPIO_Port GPIOB
#define OPT_SELECTOR_WIRE_TEST_Pin LL_GPIO_PIN_13
#define OPT_SELECTOR_WIRE_TEST_GPIO_Port GPIOB
#define OPT_SELECTOR_GAS_TEST_Pin LL_GPIO_PIN_14
#define OPT_SELECTOR_GAS_TEST_GPIO_Port GPIOB
#define OPT_SELECTOR_4T_Pin LL_GPIO_PIN_15
#define OPT_SELECTOR_4T_GPIO_Port GPIOB
#define OPT_SELECTOR_2T_Pin LL_GPIO_PIN_8
#define OPT_SELECTOR_2T_GPIO_Port GPIOA
#define DEBUG_OUT_Pin LL_GPIO_PIN_11
#define DEBUG_OUT_GPIO_Port GPIOA
#define PID_DIP_SW_BIT2_Pin LL_GPIO_PIN_12
#define PID_DIP_SW_BIT2_GPIO_Port GPIOA
#define DC_MOTOR_RELAY_OUT_Pin LL_GPIO_PIN_15
#define DC_MOTOR_RELAY_OUT_GPIO_Port GPIOA
#define ACTIVATE_ARC_OUT_Pin LL_GPIO_PIN_3
#define ACTIVATE_ARC_OUT_GPIO_Port GPIOB
#define GAS_SOLENOID_OUT_Pin LL_GPIO_PIN_4
#define GAS_SOLENOID_OUT_GPIO_Port GPIOB
#define EEPROM_WRITE_ENABLE_Pin LL_GPIO_PIN_5
#define EEPROM_WRITE_ENABLE_GPIO_Port GPIOB
#define PWM_OUT_Pin LL_GPIO_PIN_8
#define PWM_OUT_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

extern SPI_HandleTypeDef hspi1;

void pulse_debug_out(int n_pulses);

extern volatile uint8_t SPI_transfer_status;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
