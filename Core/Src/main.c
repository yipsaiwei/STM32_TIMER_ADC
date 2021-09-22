/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include  "Gpio.h"
#include  "Tim.h"
#include  "Nvic.h"
#include  "Adc.h"
#include  "Usart.h"
#include  <stdlib.h>
#include  <string.h>
#include  <stdio.h>
#include  "retarget.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
  volatile  int adcValue = 0;
  int adcCounter = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void timerConfigureExperiment1PWM(TimReg *timer);
void timerConfigureExp2ForceInactiveActive(TimReg *timer);
void  timerConfigurationExp3Toggle(TimReg  *timer);
void  timerExp4TriggerAdcWithTimer();
void  usartConfiguration(UsartReg *usart, UsartConfig config, int baudrate);
void  sendToUsart(UsartReg  *usart, char  *msg);
void  readAndPrintAdc(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  RetargetInit(usart2);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */
  //timerConfigureExp2ForceInactiveActive(tim4);
  timerExp4TriggerAdcWithTimer();
  //timerConfigureExperiment1PWM(tim4);
  //timerConfigurationExp3Toggle(tim4);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    readAndPrintAdc();
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void timerConfigureExperiment1PWM(TimReg *timer){
  rccUnresetAndEnableTimer(RCC_TIM4);

  rccUnresetAndEnableGpio(RCC_GPIOB);
  gpioConfigurePin(gpioB, 6, GPIO_ALT_FUNC | AF_2 );          //BUTTON CHANNEL 1
  gpioConfigurePin(gpioB, 8, GPIO_ALT_FUNC | GPIO_FAST_SPEED | GPIO_PUSH_PULL | AF_2 ); //LED CHANNEL 3

  timerEventGenerationConfig(timer, UPDATE_GEN);
  timerCaptureCompareConfig(timer, 3, CC3_OUT_EN | PRELOAD_EN | PWN_MOD_1);
  timerCaptureCompareConfig(timer, 1, SEL_IN_TI1 | CC1_CAP_EN);
  setFrequency(timer, 1, 65000);
  timSetOutCompareReg(tim4, 3, 52000); //80% PWM should be 52000
  //timSetOutCompareReg(timer, 3, 13000); //20% PWM should be 13000
  timerBasicConfig(timer, AUTO_RELOAD_PRELOAD_EN
                          | FILTERED_TIM_IN_1 | GATED_MODE
                          |COUNTER_EN);
}

void timerConfigureExp2ForceInactiveActive(TimReg *timer){
  rccUnresetAndEnableTimer(RCC_TIM4);

  rccUnresetAndEnableGpio(RCC_GPIOB);
  gpioConfigurePin(gpioB, 6, GPIO_ALT_FUNC | AF_2 );          //BUTTON CHANNEL 1
  gpioConfigurePin(gpioB, 8, GPIO_ALT_FUNC | GPIO_FAST_SPEED | GPIO_PUSH_PULL | AF_2 ); //LED CHANNEL 3

  while(1){
    if(gpioReadPin(gpioB, 6)){
      timerCaptureCompareConfig(timer, 3, CC3_OUT_EN | FORCE_ACT_LVL);
    }else{
      timerCaptureCompareConfig(timer, 3, CC3_OUT_EN | FORCE_INACT_LVL);
    }
  }

}

void  timerConfigurationExp3Toggle(TimReg  *timer){
  rccUnresetAndEnableTimer(RCC_TIM4);

  rccUnresetAndEnableGpio(RCC_GPIOB);
  gpioConfigurePin(gpioB, 8, GPIO_ALT_FUNC | GPIO_FAST_SPEED | GPIO_PUSH_PULL | AF_2 ); //LED CHANNEL 3

  timerCaptureCompareConfig(timer, 3, CC3_OUT_EN | TOGGLE);
  //timer->CNT = 0;
  nvicEnableIrq(30);

  //To achieve 1Hz, toggle rate should be 2Hz (Toggle once every 0.5s)
  setFrequency(timer, 2, 65000);

  timSetOutCompareReg(timer, 3, 65000);

  timerBasicConfig(timer,  CC3IE_EN
                          |COUNTER_EN);

}


void  TIM4_IRQHandler(void){
  volatile  static  int i = 1;
  if(tim4->SR & TIM_UIF)
    tim4->SR &= ~TIM_UIF;
  tim4->SR = 0;
  timSetOutCompareReg(tim4, 3, 65000);
  if(i == 56){
    i = 0;
    //To achieve 1Hz, toggle rate should be 2Hz (Toggle once every 0.5s)
    setFrequency(tim4, 2, 65000);
    timSetOutCompareReg(tim4, 3, 65000);
  }else if(i == 8){
    //To achieve 2Hz, toggle rate should be 4Hz (Toggle once every 0.25s)
    setFrequency(tim4, 4, 65000);
    timSetOutCompareReg(tim4, 3, 65000);
  }else if(i == 24){
    //To achieve 4Hz, toggle rate should be 8Hz (Toggle once every 0.125s)
    setFrequency(tim4, 8, 65000);
    timSetOutCompareReg(tim4, 3, 65000);
  }
  i++;
}

void  timerExp4TriggerAdcWithTimer(){
  int channel[] = {0};
  rccUnresetAndEnableAdc(RCC_ADC1);
  rccUnresetAndEnableGpio(RCC_GPIOA);
  rccUnresetAndEnableUsart(RCC_USART2);
  rccUnresetAndEnableTimer(RCC_TIM4);

  gpioConfigurePin(gpioA, 0, GPIO_ANALOG_IN | GPIO_PUSH_PULL );  //       (PA0)

  gpioConfigurePin(gpioA, 2, GPIO_ALT_FUNC | GPIO_FAST_SPEED | AF_7);  //USART2 (Rx)
  gpioConfigurePin(gpioA, 3, GPIO_ALT_FUNC | GPIO_FAST_SPEED | AF_7);  //       (Tx)


  usartConfiguration(usart2, USART_TX_EN | USART_RX_EN | USART_9_BIT | USART_ODD_PARITY
                          | USART_2_STOPBIT
                          | USART_EN,
                          115200);

  timerEventGenerationConfig(tim4, UPDATE_GEN);

  timerCaptureCompareConfig(tim4, 4, CC4_OUT_EN | PWN_MOD_1);

  setFrequency(tim4, 2000, 500);

  timSetOutCompareReg(tim4, 4, 250);

  adcSetChannelSequence(adc1, channel, 1);

  nvicEnableIrq(18);

  adcConfiguration(adc1, ADC_CONVERTER_ON | REGULAR_CHN_START_CONVERT
                        | REGULAR_EXT_EVENT_SEL_TIM_4_CC4 | REGULAR_TRIG_DETECTION_RISING_EDGE | EOC_INT_EN | SET_EOC_BIT_AFTER_REGULAR_CONVERSION_ENDED);
  timerBasicConfig(tim4,  COUNTER_EN );
}

void  ADC_IRQHandler(void){
  //static  int i = 0;
  static  int convertedValues = 0;
  //while(!(adc1->SR && 0x2));
  convertedValues += adc1->DR;
  adcCounter++;
  if(adcCounter == 400){
    adcValue = convertedValues / 400;
    adcCounter = 0;
    convertedValues = 0;
    //sendToUsart(usart2, convertIntToAscii(adcValue));
  }

}

void  readAndPrintAdc(void){
  static int previousValue = 0;
  float voltage;
  if(adcCounter == 0){
    voltage = (3.3/4096.0) * adcValue;
    printf("ADC value : %d, voltage = %f\n\r", adcValue, voltage);
    adcValue = previousValue;
  }
}

void  sendToUsart(UsartReg  *usart, char  *msg){
  for(int  i = 0; msg[i] != '\0'; i++){
    while(!isTDREmpty(usart));
    writeToDataRegister(usart, msg[i]);
    while(!isTransmissionComplete(usart));
    }
  free(msg);
}

void  usartConfiguration(UsartReg *usart, UsartConfig config, int baudrate){
  usartSetBaudRate(usart, baudrate);
  usartConfigure(usart,config);
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
