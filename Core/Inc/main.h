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

/**
  * @brief  State of user IOs.
  * @note   "1" = floating
  *         "0" = connected to ground
  * @note   call user_read_io() to get current state
  */
typedef struct
{
  uint8_t state[8];
} USR_IO_State;

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

typedef enum {
    USER = 1,
    WARNING = 2,
    ALARM = 3
} buzzer_mutex_priorities ;

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
#define ADC_buf_size 5
extern uint32_t ADC1_buf[ADC_buf_size];

extern input_src_stat *prime_VIN;
extern input_src_stat VIN1;
extern input_src_stat VIN2;
extern input_src_stat VIN3;

extern uint8_t pc_stat; // PC power bus status. RO
extern uint8_t bus_stat; // main power bus status. RO
extern uint8_t emergency_stat;

extern uint8_t buzzer_mutex;
extern uint64_t buzzer_pulse_stamp;
extern uint64_t buzzer_period_stamp;

uint64_t micros_64(void);
void micros_delay( uint64_t delay );
void UART2_printf( const char * format, ... );

void user_setup(void);
void user_spin(void);

/**
  * @brief  Read state of user IOs all at once via the SPI in blocking mode
  * @retval current state of user IOs
  */
USR_IO_State user_read_io(void);

/**
  * @brief  Set PWM duty-cycle for selested user IO
  * @param  usr_io Desired channel number
  * @param  value Desired PWM value. Values range is [0:255}
  * @retval Error code. "0" if succeeded
  */
uint8_t user_write_io(uint8_t usr_io, uint8_t value);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SW3_Pin LL_GPIO_PIN_4
#define SW3_GPIO_Port GPIOA
#define HPBRD_PC_CTL_Pin LL_GPIO_PIN_5
#define HPBRD_PC_CTL_GPIO_Port GPIOA
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
