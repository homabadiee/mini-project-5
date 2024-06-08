/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f3xx_it.c
  * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f3xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
typedef struct
{
    uint16_t frequency;
    uint16_t duration;
} Tone;
/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
//#define ARRAY_LENGTH(arr) (sizeof(arr) / sizeof((arr)[0]))

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
//
//extern TIM_HandleTypeDef htim2;
//TIM_HandleTypeDef *pwm_timer = &htim2;	// Point to PWM Timer configured in CubeMX
//uint32_t pwm_channel = TIM_CHANNEL_2;   // Select configured PWM channel number
//
//const Tone *volatile melody_ptr;
//volatile uint16_t melody_tone_count;
//volatile uint16_t current_tone_number;
//volatile uint32_t current_tone_end;
//volatile uint16_t volume = 50;          // (0 - 1000)
//volatile uint32_t last_button_press;

//const Tone super_mario_bros[] = {
//	{2637,306}, // E7 x2
//	{   0,153}, // x3 <-- Silence
//	{2637,153}, // E7
//	{   0,153}, // x3
//	{2093,153}, // C7
//	{2637,153}, // E7
//	{   0,153}, // x3
//	{3136,153}, // G7
//	{   0,459}, // x3
//	{1586,153}, // G6
//	{   0,459}, // x3
//
//	{2093,153}, // C7
//	{   0,306}, // x2
//	{1586,153}, // G6
//	{   0,306}, // x2
//	{1319,153}, // E6
//	{   0,306}, // x2
//	{1760,153}, // A6
//	{   0,153}, // x1
//	{1976,153}, // B6
//	{   0,153}, // x1
//	{1865,153}, // AS6
//	{1760,153}, // A6
//	{   0,153}, // x1
//
//	{1586,204}, // G6
//	{2637,204}, // E7
//	{3136,204}, // G7
//	{3520,153}, // A7
//	{   0,153}, // x1
//	{2794,153}, // F7
//	{3136,153}, // G7
//	{   0,153}, // x1
//	{2637,153}, // E7
//	{   0,153}, // x1
//	{2093,153}, // C7
//	{2349,153}, // D7
//	{1976,153}, // B6
//	{   0,306}, // x2
//
//	{2093,153}, // C7
//	{   0,306}, // x2
//	{1586,153}, // G6
//	{   0,306}, // x2
//	{1319,153}, // E6
//	{   0,306}, // x2
//	{1760,153}, // A6
//	{   0,153}, // x1
//	{1976,153}, // B6
//	{   0,153}, // x1
//	{1865,153}, // AS6
//	{1760,153}, // A6
//	{   0,153}, // x1
//
//	{1586,204}, // G6
//	{2637,204}, // E7
//	{3136,204}, // G7
//	{3520,153}, // A7
//	{   0,153}, // x1
//	{2794,153}, // F7
//	{3136,153}, // G7
//	{   0,153}, // x1
//	{2637,153}, // E7
//	{   0,153}, // x1
//	{2093,153}, // C7
//	{2349,153}, // D7
//	{1976,153}, // B6
//
//	{   0,  0}	// <-- Disable PWM
//};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//void PWM_Start()
//{
//    HAL_TIM_PWM_Start(pwm_timer, pwm_channel);
//}
//
//void PWM_Change_Tone(uint16_t pwm_freq, uint16_t volume) // pwm_freq (1 - 20000), volume (0 - 1000)
//{
//    if (pwm_freq == 0 || pwm_freq > 20000)
//    {
//        __HAL_TIM_SET_COMPARE(pwm_timer, pwm_channel, 0);
//    }
//    else
//    {
//        const uint32_t internal_clock_freq = HAL_RCC_GetSysClockFreq();
//        const uint16_t prescaler = 1 + internal_clock_freq / pwm_freq / 60000;
//        const uint32_t timer_clock = internal_clock_freq / prescaler;
//        const uint32_t period_cycles = timer_clock / pwm_freq;
//        const uint32_t pulse_width = volume * period_cycles / 1000 / 2;
//
//        pwm_timer->Instance->PSC = prescaler - 1;
//        pwm_timer->Instance->ARR = period_cycles - 1;
//        pwm_timer->Instance->EGR = TIM_EGR_UG;
//        __HAL_TIM_SET_COMPARE(pwm_timer, pwm_channel, pulse_width); // pwm_timer->Instance->CCR2 = pulse_width;
//    }
//}
//
//void Change_Melody(const Tone *melody, uint16_t tone_count)
//{
//    melody_ptr = melody;
//    melody_tone_count = tone_count;
//    current_tone_number = 0;
//}
//
//void Update_Melody()
//{
//    if ((HAL_GetTick() > current_tone_end) && (current_tone_number < melody_tone_count))
//    {
//        const Tone active_tone = *(melody_ptr + current_tone_number);
//        PWM_Change_Tone(active_tone.frequency, volume);
//        current_tone_end = HAL_GetTick() + active_tone.duration;
//        current_tone_number++;
//    }
//}


/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart1;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {

  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F3xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f3xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt / USART1 wake-up interrupt through EXTI line 25.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
