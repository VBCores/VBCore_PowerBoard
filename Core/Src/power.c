#include <stdio.h>
#include <stdarg.h>
#include "main.h"
#include "power.h"
#include "battery_config.h"

//#define curr_sns_inver

// private power control parameters
const uint32_t bus_start_timeout = 30000u; // timeout for bus output reaching PowerGood status, microseconds
const uint8_t emergency_start_threshold = 3u; // number of attempts to start the bus before an emergency shutdown

// user defined power source parameters
const float uvlo_level = undervoltage_lockout_voltage; // battery discharged voltage level, Volts
const float uvlo_hyst = undervoltage_lockout_hysteresis; // battery discharged hysteresis, Volts
const float src_charged_level = charged_battery_voltage; // battery charged voltage level, Volts
const float nom_chrg_curr = nominal_charge_current;

// user accessible power control parameters
extern uint8_t prime;
extern uint8_t default_prime;
extern uint8_t pc_enable;
extern uint8_t bus_enable;

// user accessible power control status
extern uint8_t emergency_stat;
extern uint8_t pc_stat;
extern uint8_t bus_stat;

// configuration structs for peripherals
extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;

extern IWDG_HandleTypeDef hiwdg;

extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim15;
extern TIM_HandleTypeDef htim16;

extern IWDG_HandleTypeDef hiwdg;

// ADC related stuff
#define ADC_buf_size 5
uint32_t ADC1_buf[ADC_buf_size] = {0};

// private(?) variables
input_src_stat VIN1;
input_src_stat VIN2 = { .PG_pin = SUPP_2_PG_Pin, .PG_port = SUPP_2_PG_GPIO_Port};
input_src_stat VIN3 = { .PG_pin = SUPP_3_PG_Pin, .PG_port = SUPP_3_PG_GPIO_Port};

input_src_stat CHRG;

input_src_stat *prime_VIN;

input_src_stat *prime_VOUT = NULL;


#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma optimize s=none
static void __micros_delay( uint64_t delay )
#elif defined ( __GNUC__ ) /*!< GNU Compiler */
static void __attribute__((optimize("O0"))) __micros_delay( uint64_t delay )
#endif
{
  uint64_t timestamp = micros();
  while( micros() < timestamp + delay )
  {
    HAL_IWDG_Refresh(&hiwdg);
  }
}

void power_setup(void)
{
  __micros_delay(300000); // prevents MOSFETs torture if board is forced to restart frequently

  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED); // apply factory ADC calibration settings
  HAL_ADC_Start_DMA(&hadc1, ADC1_buf, ADC_buf_size); // enable ADC and bind DMA to the output buffer
  
  HAL_TIM_Base_Start_IT(&htim6); // processes ADC values in TIM6 interrupt
  
  //LL_GPIO_SetOutputPin(PC_CTL_GPIO_Port, PC_CTL_Pin); // enable PC bus    
  
  __micros_delay(100000);
  HAL_TIM_Base_Start_IT(&htim16); // processes power control logic in TIM16 interrupt
  
  UART2_printf( "UVLO=%4.1f HYST=%4.1f FULL_CHRG=%4.1f CHRG_CURR=%4.1f\r\n", uvlo_level, uvlo_hyst, src_charged_level, nom_chrg_curr);
}

static inline float adc_voltage(uint8_t channel)
{
  return 3.3f*(float)ADC1_buf[channel] / ( 4096.0f );
}

static inline float get_actual_current(void)
{
  float ACS_output_v = adc_voltage(1) - 1.65f;
  
  #ifdef curr_sns_inver
  ACS_output_v *= -1.0f;
  #endif
  
  // ACS725LLCTR-50AB Sensitivity = 26.4 mV/A
  return ACS_output_v / 0.0264f;
}

static uint8_t check_battery_input( input_src_stat * VIN )
{
  if( VIN->HPF < -5.0f )
  {
    __micros_delay( 1000 );
    
    if( get_actual_current() < 0.4f ) // exclude transients. the source is disconnected only if current ~0A
    {
      VIN->attached = 0;
      VIN->LPF = 0.0f;
    }
  }
  else if( VIN->HPF > 5.0f )
  {
    VIN->attached = 1;
    VIN->LPF = VIN->raw;
  }

  if( VIN->LPF < uvlo_level )
  {
    VIN->charged = 0;
  }
  else if( VIN->LPF > uvlo_level + uvlo_hyst )
  {
    VIN->charged = 1;
  }
    
  return 0;
}

static void check_buttons(void)
{
  if( !LL_GPIO_IsInputPinSet( SW2_GPIO_Port, SW2_Pin) )
  {
    if( !LL_GPIO_IsInputPinSet( SW3_GPIO_Port, SW3_Pin) )
    {
      // user is noodle
    }
    else
    {
      prime = 0;
    }
  }
  else
  {
    if( !LL_GPIO_IsInputPinSet( SW3_GPIO_Port, SW3_Pin) )
    {
      prime = 1;
    }
    else
    {
      // do nothing
    }
  }  
}

extern uint8_t buzzer_mutex;
extern uint64_t buzzer_pulse_stamp;
extern uint64_t buzzer_period_stamp;

void power_control(void)
{
  static uint8_t start_fail_cnt = 0; // counter of failed start-up sequences
  
  // emergency shutdown if output did not start too many times
  if( start_fail_cnt >= emergency_start_threshold )
  {
    LL_GPIO_SetOutputPin(OE_CTL_GPIO_Port, OE_CTL_Pin); // reset channels output control
    LL_GPIO_ResetOutputPin(BUS_CTL_GPIO_Port, BUS_CTL_Pin); // disable power bus    
    
    UART2_printf( "The bus did not reach PG status after %d attempts!\r\n", emergency_start_threshold);
    Error_Handler();
  }
  
  check_battery_input( &VIN2 );
  check_battery_input( &VIN3 );
  
  // Check buttons and select prime
  check_buttons();
  
  if( 16.0f*adc_voltage(2) > 2.0f && LL_GPIO_IsInputPinSet( DEMUX_OE_GPIO_Port, DEMUX_OE_Pin ) )
  {
    VIN1.attached = 1;
    
    // the batteries are disconnected automatically, but control signal needs to be disabled for 
    // the other software parts to work
    LL_GPIO_SetOutputPin(OE_CTL_GPIO_Port, OE_CTL_Pin);
    
    uint64_t timestamp = 0;
    
    if( bus_enable )
    {
      if( !LL_GPIO_IsOutputPinSet(BUS_CTL_GPIO_Port, BUS_CTL_Pin) )
      {
        LL_GPIO_SetOutputPin(BUS_CTL_GPIO_Port, BUS_CTL_Pin); // enable power bus
        __micros_delay(2);
      }
      
      if( !LL_GPIO_IsInputPinSet( BUS_EN_GPIO_Port, BUS_EN_Pin) && LL_GPIO_IsOutputPinSet(BUS_CTL_GPIO_Port, BUS_CTL_Pin) )
      {
        emergency_stat = 1;
        LL_GPIO_ResetOutputPin(BUS_CTL_GPIO_Port, BUS_CTL_Pin); // disable power bus
      }
      else
      {
        emergency_stat = 0;
        
        timestamp = micros();
        while( LL_GPIO_IsInputPinSet( BUS_PG_GPIO_Port, BUS_PG_Pin) ) // wait until PG pin goes low ( PG = OK )
        {
          if( micros() > timestamp + bus_start_timeout ) // the bus didnt reach PG status in allotted time = abort operation
          {
            LL_GPIO_SetOutputPin(OE_CTL_GPIO_Port, OE_CTL_Pin); // reset channels output control
            LL_GPIO_ResetOutputPin(BUS_CTL_GPIO_Port, BUS_CTL_Pin); // disable power bus
            
            __micros_delay( 1000000 ); // wait for 1s before trying to enable bus again to prevent MOSFET damage
            
            start_fail_cnt++;
            
            return ;
          }
          
          HAL_IWDG_Refresh(&hiwdg);
        }
      }   
    }
    else
    {
      LL_GPIO_ResetOutputPin(BUS_CTL_GPIO_Port, BUS_CTL_Pin); // disable power bus
      return ;
    }
    
    return ;
  }
  else
  {
    VIN1.attached = 0;
  }
  
#ifdef auto_prime_selection
  if( VIN2.charged && VIN3.charged )
  {
    prime = default_prime;
  }
#endif
  
  if( CHRG.raw > 3.0f ) 
  {
    // we're in charging mode
    LL_GPIO_ResetOutputPin(BUS_CTL_GPIO_Port, BUS_CTL_Pin); // disable power bus    

    if( prime_VOUT == NULL )
    {
      LL_GPIO_SetOutputPin(OE_CTL_GPIO_Port, OE_CTL_Pin); // disable input channels
      
      __micros_delay( 50000 );

      if( ( VIN2.raw > 10.0f ) && ( VIN2.raw < src_charged_level ) )
      {
        prime_VOUT = &VIN2;
        
        LL_GPIO_SetOutputPin(DEMUX_S0_CTL_GPIO_Port, DEMUX_S0_CTL_Pin);
        LL_GPIO_ResetOutputPin(OE_CTL_GPIO_Port, OE_CTL_Pin); // set channels output control
        
        __micros_delay( 50000 );
      }
      else if( ( VIN3.raw > 10.0f ) && ( VIN3.raw < src_charged_level ) )
      {
        prime_VOUT = &VIN3;

        LL_GPIO_ResetOutputPin(DEMUX_S0_CTL_GPIO_Port, DEMUX_S0_CTL_Pin);
        LL_GPIO_ResetOutputPin(OE_CTL_GPIO_Port, OE_CTL_Pin); // set channels output control

        __micros_delay( 50000 );
      }
      else
      {
        return ;
      }
    }
    
    static float my_current = 0.0f;
    
    my_current = my_current - (0.1f * (my_current - get_actual_current())); 
    
    if( my_current < ( -1.3f * nom_chrg_curr ) )
    {
      // charging overcurrent event
      LL_GPIO_SetOutputPin(OE_CTL_GPIO_Port, OE_CTL_Pin); // reset channels output control
      LL_GPIO_ResetOutputPin(BUS_CTL_GPIO_Port, BUS_CTL_Pin); // disable power bus    
    
      UART2_printf( "Charge  overcurrent event!\r\n");
      Error_Handler();
    }
    
    if( get_actual_current() > ( -0.075f * nom_chrg_curr ))
    {
      // battery takes no current = battery is charged
      prime_VOUT = NULL;
    }
  }
  else
  {
    prime_VOUT = NULL;
    
    // select suitable power source based on availability and user request
    if( prime == 0 )
    {
      if( VIN2.charged )
      {
        prime_VIN = &VIN2;
      }
      else if( VIN3.charged )
      {
        prime = 1; // switch ptime channel to VIN3
        prime_VIN = &VIN3;
      }
      else
      {
        prime_VIN = NULL;
      }
    }
    else
    {
      if( VIN3.charged )
      {
        prime_VIN = &VIN3;
      }
      else if( VIN2.charged )
      {
        prime = 0; // switch ptime channel to VIN2
        prime_VIN = &VIN2;
      }
      else
      {
        prime_VIN = NULL;
      }    
    }

    // switch power rails
    if( prime_VIN != NULL )
    {
      uint64_t timestamp = 0;
      
      if( pc_enable )
      {
        // select desired channel on multiplexer
        if( !prime )
        {
          LL_GPIO_SetOutputPin(DEMUX_S0_CTL_GPIO_Port, DEMUX_S0_CTL_Pin);
        }
        else
        {
          LL_GPIO_ResetOutputPin(DEMUX_S0_CTL_GPIO_Port, DEMUX_S0_CTL_Pin);
        }
        
        LL_GPIO_ResetOutputPin(OE_CTL_GPIO_Port, OE_CTL_Pin); // set channels output control
        
        timestamp = micros();
        while( LL_GPIO_IsInputPinSet( prime_VIN->PG_port, prime_VIN->PG_pin) ) // wait until PG pin goes low ( PG = OK )
        {
          if( micros() > timestamp + 10000 ) // the bus didnt reach PG status in 10 ms = abort operation
          {
            LL_GPIO_SetOutputPin(OE_CTL_GPIO_Port, OE_CTL_Pin); // reset channels output control
            LL_GPIO_ResetOutputPin(BUS_CTL_GPIO_Port, BUS_CTL_Pin); // disable power bus
            
            __micros_delay( 1000000 ); // wait for 1s before trying to enable bus again to prevent MOSFET damage
            
            start_fail_cnt++;

            return ;
          }
          
          HAL_IWDG_Refresh(&hiwdg);
        }
      }
      else
      {
        LL_GPIO_SetOutputPin(OE_CTL_GPIO_Port, OE_CTL_Pin); // reset channels output control
        LL_GPIO_ResetOutputPin(BUS_CTL_GPIO_Port, BUS_CTL_Pin); // disable power bus
        return ;
      }
      
      if( bus_enable )
      {
        if( !LL_GPIO_IsOutputPinSet(BUS_CTL_GPIO_Port, BUS_CTL_Pin) )
        {
          LL_GPIO_SetOutputPin(BUS_CTL_GPIO_Port, BUS_CTL_Pin); // enable power bus
          __micros_delay(2);
        }
        
        if( !LL_GPIO_IsInputPinSet( BUS_EN_GPIO_Port, BUS_EN_Pin) && LL_GPIO_IsOutputPinSet(BUS_CTL_GPIO_Port, BUS_CTL_Pin) )
        {
          emergency_stat = 1;
          LL_GPIO_ResetOutputPin(BUS_CTL_GPIO_Port, BUS_CTL_Pin); // disable power bus
        }
        else
        {
          emergency_stat = 0;
          
          timestamp = micros();
          while( LL_GPIO_IsInputPinSet( BUS_PG_GPIO_Port, BUS_PG_Pin) ) // wait until PG pin goes low ( PG = OK )
          {
            if( micros() > timestamp + bus_start_timeout ) // the bus didnt reach PG status in allotted time = abort operation
            {
              LL_GPIO_SetOutputPin(OE_CTL_GPIO_Port, OE_CTL_Pin); // reset channels output control
              LL_GPIO_ResetOutputPin(BUS_CTL_GPIO_Port, BUS_CTL_Pin); // disable power bus
              
              __micros_delay( 1000000 ); // wait for 1s before trying to enable bus again to prevent MOSFET damage
              
              start_fail_cnt++;
              
              return ;
            }
            
            HAL_IWDG_Refresh(&hiwdg);
          }
        }   
      }
      else
      {
        LL_GPIO_ResetOutputPin(BUS_CTL_GPIO_Port, BUS_CTL_Pin); // disable power bus
        return ;
      }
    }
    else
    {
      // all dead, disable output
      
      LL_GPIO_SetOutputPin(OE_CTL_GPIO_Port, OE_CTL_Pin); // reset channels output control
      
      if( LL_GPIO_IsOutputPinSet(BUS_CTL_GPIO_Port, BUS_CTL_Pin) )
      {
        LL_GPIO_ResetOutputPin(BUS_CTL_GPIO_Port, BUS_CTL_Pin); // disable power bus     
        __micros_delay(200000);
      }    
      
      if( buzzer_mutex < ALARM )
      {
        buzzer_mutex = ALARM;
        buzzer_pulse_stamp = micros() + 1000000u;
        buzzer_period_stamp = micros() + 2000000u;
      }    
    } 
  }
}

static void indication(void)
{
  // warning is needed on battery power only
  if( !VIN1.attached )
  {
    if( prime_VIN->LPF < uvlo_level + uvlo_hyst )
    {
      if( buzzer_mutex < WARNING )
      {
        buzzer_mutex = WARNING;
        buzzer_pulse_stamp = micros() + 300000u;
        buzzer_period_stamp = micros() + 1800000u;
      }
    }
  }
  
  if( buzzer_mutex )
  {
    if( micros() < buzzer_pulse_stamp )
    {
      HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_2);
    }
    else if( micros() < buzzer_period_stamp )
    {
      HAL_TIM_PWM_Stop(&htim15, TIM_CHANNEL_2);
    }
    else
    {
      buzzer_mutex = 0;
    }
  }

  if( VIN1.attached ) { LL_GPIO_SetOutputPin(S1_red_GPIO_Port, S1_red_Pin);
                        LL_GPIO_SetOutputPin(S1_grn_GPIO_Port, S1_grn_Pin);   }
  else                { LL_GPIO_ResetOutputPin(S1_red_GPIO_Port, S1_red_Pin);
                        LL_GPIO_ResetOutputPin(S1_grn_GPIO_Port, S1_grn_Pin); }
  
  if( VIN2.attached ) { LL_GPIO_SetOutputPin(S2_red_GPIO_Port, S2_red_Pin);   }
  else                { LL_GPIO_ResetOutputPin(S2_red_GPIO_Port, S2_red_Pin); }
  
  if( VIN2.charged && pc_stat && LL_GPIO_IsOutputPinSet(DEMUX_S0_CTL_GPIO_Port, DEMUX_S0_CTL_Pin) )  { LL_GPIO_SetOutputPin(S2_grn_GPIO_Port, S2_grn_Pin);   }
  else                { LL_GPIO_ResetOutputPin(S2_grn_GPIO_Port, S2_grn_Pin); }    
  
  if( VIN3.attached ) { LL_GPIO_SetOutputPin(S3_red_GPIO_Port, S3_red_Pin);   }
  else                { LL_GPIO_ResetOutputPin(S3_red_GPIO_Port, S3_red_Pin); }
  
  if( VIN3.charged && pc_stat && !LL_GPIO_IsOutputPinSet(DEMUX_S0_CTL_GPIO_Port, DEMUX_S0_CTL_Pin) )  { LL_GPIO_SetOutputPin(S3_grn_GPIO_Port, S3_grn_Pin);   }
  else                { LL_GPIO_ResetOutputPin(S3_grn_GPIO_Port, S3_grn_Pin); }     
}

void power_adc_callback(void)
{
  VIN1.raw = 16.0f*adc_voltage(2);
  VIN1.LPF = VIN1.LPF - (0.005f * (VIN1.LPF - VIN1.raw));
  VIN1.HPF = VIN1.raw - VIN1.LPF;

  VIN2.raw = 16.0f*adc_voltage(3);
  VIN2.LPF = VIN2.LPF - (0.005f * (VIN2.LPF - VIN2.raw));
  VIN2.HPF = VIN2.raw - VIN2.LPF;

  VIN3.raw = 16.0f*adc_voltage(4);
  VIN3.LPF = VIN3.LPF - (0.005f * (VIN3.LPF - VIN3.raw));
  VIN3.HPF = VIN3.raw - VIN3.LPF;    

  CHRG.raw = 16.0f*adc_voltage(0);
  CHRG.LPF = CHRG.LPF - (0.005f * (CHRG.LPF - CHRG.raw));
  CHRG.HPF = CHRG.raw - CHRG.LPF;    

  return ;
}

void power_ctl_callback(void)
{
  power_control();

  pc_stat = !LL_GPIO_IsOutputPinSet(OE_CTL_GPIO_Port, OE_CTL_Pin);
  bus_stat = LL_GPIO_IsOutputPinSet(BUS_CTL_GPIO_Port, BUS_CTL_Pin);

  indication();

  HAL_IWDG_Refresh(&hiwdg);  
}