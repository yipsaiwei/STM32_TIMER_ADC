/*
 * Tim.c
 *
 *  Created on: Aug 2, 2021
 *      Author: Yip Sai Wei
 */

#include  "Tim.h"

void  timerBasicConfig(TimReg *timer, BasicTimConfig  config){
  timer->CR1 &= TIM_16_BIT_MASK;
  timer->CR1 |= (config & 0x1FF);

  timer->CR2 &= TIM_16_BIT_MASK;
  timer->CR2 |= (config >> 16) & 0x7FFF;

  timer->SMCR &= TIM_16_BIT_MASK;
  timer->SMCR |= (config >> 32) & 0xFFFF;

  timer->DIER &= TIM_16_BIT_MASK;
  timer->DIER |= (config >> 48) &0x7FFF;
}

void  timerEventGenerationConfig(TimReg *timer, SpecificTimConfig1 config){
  timer->EGR &= TIM_8_bit_MASK;
  timer->EGR |= (config & 0xF);
}

void  timerCaptureCompareConfig(TimReg *timer, int  channel, SpecificTim1Config2  config){
  /*
  timer->CCMR1 &= TIM_16_BIT_MASK;
  timer->CCMR1 |= (config & 0xFFFF);

  timer->CCMR2 &= TIM_16_BIT_MASK;
  timer->CCMR2 |= (config >> 16) & 0xFFFF;
  */
  if(channel == 1 || channel == 2){
    if(channel == 1){
      timer->CCMR1 &= TIM_CHANNEL_MASK_LOWER_8;
      timer->CCMR1 |= (config & 0xFF);
    }else{
      timer->CCMR1 &= TIM_CHANNEL_MASK_HIGHER_8;
      timer->CCMR1 |= (config & 0xFF) << 8;
    }
  }else{
    if(channel == 3){
      timer->CCMR2 &= TIM_CHANNEL_MASK_LOWER_8;
      timer->CCMR2 |= (config & 0xFF);
    }else{
      timer->CCMR2 &= TIM_CHANNEL_MASK_HIGHER_8;
      timer->CCMR2 |= (config & 0xFF) << 8;
    }
  }
  timer->CCER &= TIM_16_BIT_MASK;
  timer->CCER |= (config >> 32) & 0x3FFF;
}

void  timSetPrescalerReg(TimReg *timer, uint32_t  value){
  timer->PSC = value;
}

void  timSetARRReg(TimReg *timer, uint32_t  value){
  timer->ARR = value;
}

void  setFrequency(TimReg *timer, long double  desiredFrequency, uint32_t  arrValue){
  uint32_t timerFrequency = HAL_RCC_GetHCLKFreq();
  timSetARRReg(timer, arrValue);
  timSetPrescalerReg(timer, ((long double)timerFrequency / (arrValue * desiredFrequency)) - 1);
}
