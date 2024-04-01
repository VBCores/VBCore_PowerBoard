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
typedef enum {
    USER = 1,
    WARNING = 2,
    ALARM = 3
} buzzer_mutex_priorities;

typedef struct
{
    float raw;
    float LPF;
    float HPF;
    uint8_t charged;
    uint8_t attached;

    uint32_t PG_pin;
    GPIO_TypeDef * PG_port;
} input_src_stat;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
extern uint8_t buzzer_mutex;
extern uint64_t buzzer_pulse_stamp;
extern uint64_t buzzer_period_stamp;
extern input_src_stat *prime_VIN;
extern input_src_stat VIN1;
extern input_src_stat VIN2;
extern input_src_stat VIN3;
extern uint32_t ADC1_buf[6];
extern uint8_t emergency_stat;
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
uint64_t micros_64(void);
void micros_delay( uint64_t delay );
void UART2_printf( const char * format, ... );

void user_setup(void);
void user_spin(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define S3_red_Pin LL_GPIO_PIN_4
#define S3_red_GPIO_Port GPIOA
#define BUS_CTL_Pin LL_GPIO_PIN_5
#define BUS_CTL_GPIO_Port GPIOA
#define SW2_Pin LL_GPIO_PIN_6
#define SW2_GPIO_Port GPIOA
#define S2_red_Pin LL_GPIO_PIN_7
#define S2_red_GPIO_Port GPIOA
#define S1_red_Pin LL_GPIO_PIN_5
#define S1_red_GPIO_Port GPIOC
#define GPIO1_Pin LL_GPIO_PIN_0
#define GPIO1_GPIO_Port GPIOB
#define GPIO2_Pin LL_GPIO_PIN_1
#define GPIO2_GPIO_Port GPIOB
#define GPIO3_Pin LL_GPIO_PIN_2
#define GPIO3_GPIO_Port GPIOB
#define GPIO4_Pin LL_GPIO_PIN_10
#define GPIO4_GPIO_Port GPIOB
#define OE_CTL_Pin LL_GPIO_PIN_12
#define OE_CTL_GPIO_Port GPIOB
#define DEMUX_OE_Pin LL_GPIO_PIN_13
#define DEMUX_OE_GPIO_Port GPIOB
#define SW3_Pin LL_GPIO_PIN_14
#define SW3_GPIO_Port GPIOB
#define DEMUX_S0_CTL_Pin LL_GPIO_PIN_15
#define DEMUX_S0_CTL_GPIO_Port GPIOB
#define BEEPER_Pin LL_GPIO_PIN_6
#define BEEPER_GPIO_Port GPIOC
#define LED_R_Pin LL_GPIO_PIN_7
#define LED_R_GPIO_Port GPIOC
#define LED_G_Pin LL_GPIO_PIN_8
#define LED_G_GPIO_Port GPIOC
#define LED_B_Pin LL_GPIO_PIN_9
#define LED_B_GPIO_Port GPIOC
#define BUS_EN_Pin LL_GPIO_PIN_10
#define BUS_EN_GPIO_Port GPIOA
#define SUPP_2_PG_Pin LL_GPIO_PIN_11
#define SUPP_2_PG_GPIO_Port GPIOA
#define SUPP_3_PG_Pin LL_GPIO_PIN_12
#define SUPP_3_PG_GPIO_Port GPIOA
#define BUS_PG_Pin LL_GPIO_PIN_10
#define BUS_PG_GPIO_Port GPIOC
#define S3_grn_Pin LL_GPIO_PIN_4
#define S3_grn_GPIO_Port GPIOB
#define S2_grn_Pin LL_GPIO_PIN_5
#define S2_grn_GPIO_Port GPIOB
#define PC_CTL_Pin LL_GPIO_PIN_6
#define PC_CTL_GPIO_Port GPIOB
#define S1_grn_Pin LL_GPIO_PIN_7
#define S1_grn_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
