/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32g4xx_ll_system.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_exti.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_cortex.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_utils.h"
#include "stm32g4xx_ll_pwr.h"
#include "stm32g4xx_ll_dma.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

typedef struct
{
  uint8_t IO1;
  uint8_t IO2;
  uint8_t IO3;
  uint8_t IO4;
  uint8_t IO5;
  uint8_t IO6;
  uint8_t IO7;
  uint8_t IO8;
} USR_IO_State;
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
uint64_t micros(void);
void micros_delay( uint64_t delay );
void UART2_printf( const char * format, ... );

void user_setup(void);
void user_spin(void);

USR_IO_State user_read_io(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SW3_Pin LL_GPIO_PIN_4
#define SW3_GPIO_Port GPIOA
#define SW2_Pin LL_GPIO_PIN_5
#define SW2_GPIO_Port GPIOC
#define S1_grn_Pin LL_GPIO_PIN_2
#define S1_grn_GPIO_Port GPIOB
#define S1_red_Pin LL_GPIO_PIN_10
#define S1_red_GPIO_Port GPIOB
#define SHIFT_LD_Pin LL_GPIO_PIN_12
#define SHIFT_LD_GPIO_Port GPIOB
#define BUS_EN_Pin LL_GPIO_PIN_10
#define BUS_EN_GPIO_Port GPIOA
#define SUPP_2_PG_Pin LL_GPIO_PIN_11
#define SUPP_2_PG_GPIO_Port GPIOA
#define SUPP_3_PG_Pin LL_GPIO_PIN_12
#define SUPP_3_PG_GPIO_Port GPIOA
#define BUS_PG_Pin LL_GPIO_PIN_15
#define BUS_PG_GPIO_Port GPIOA
#define BUS_CTL_Pin LL_GPIO_PIN_10
#define BUS_CTL_GPIO_Port GPIOC
#define OE_CTL_Pin LL_GPIO_PIN_11
#define OE_CTL_GPIO_Port GPIOC
#define DEMUX_OE_Pin LL_GPIO_PIN_12
#define DEMUX_OE_GPIO_Port GPIOC
#define DEMUX_S0_CTL_Pin LL_GPIO_PIN_3
#define DEMUX_S0_CTL_GPIO_Port GPIOB
#define S3_red_Pin LL_GPIO_PIN_4
#define S3_red_GPIO_Port GPIOB
#define S3_grn_Pin LL_GPIO_PIN_5
#define S3_grn_GPIO_Port GPIOB
#define S2_red_Pin LL_GPIO_PIN_6
#define S2_red_GPIO_Port GPIOB
#define S2_grn_Pin LL_GPIO_PIN_7
#define S2_grn_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
