/*
 * Tim1.h
 *
 *  Created on: Jul 31, 2021
 *      Author: Yip Sai Wei
 */

#ifndef INC_TIM_H_
#define INC_TIM_H_

#include  <stdint.h>
#include  "IO.h"
#include  "Rcc.h"
#include "stm32f4xx_hal.h"
#include <BaseAddr/BaseAddress.h>

#define TIM_UIF (1 << 0)

#define timSetOutCompareReg(tim, chn, val)                       \
  ((_IO_  uint32_t  *)&(tim)->CCR)[(chn) - 1] = (val)

#define timGetInCaptureReg(tim, chn, val)                        \
  ((_IO_  uint32_t  *)&(tim)->ccr1)[(chn) - 1]

#define tim1                ((TimReg*)TIM1BaseAddress)
#define tim2                ((TimReg*)TIM2BaseAddress)
#define tim3                ((TimReg*)TIM3BaseAddress)
#define tim4                ((TimReg*)TIM4BaseAddress)

#define TIM_16_BIT_MASK             ~(0xFF)
#define TIM_8_bit_MASK              ~(0xF)
#define TIM_CHANNEL_MASK_LOWER_8    ~(0x00FF)
#define TIM_CHANNEL_MASK_HIGHER_8   ~(0xFF00)

typedef struct  TimReg_t  TimReg;
struct  TimReg_t {
  _IO_ uint32_t CR1;
  _IO_ uint32_t CR2;
  _IO_ uint32_t SMCR;
  _IO_ uint32_t DIER;
  _IO_ uint32_t SR;
  _IO_ uint32_t EGR;
  _IO_ uint32_t CCMR1;
  _IO_ uint32_t CCMR2;
  _IO_ uint32_t CCER;
  _IO_ uint32_t CNT;
  _IO_ uint32_t PSC;
  _IO_ uint32_t ARR;
  _IO_ uint32_t RCR;
  _IO_ uint32_t CCR[4];
 // _IO_ uint32_t CCR2;
  //_IO_ uint32_t CCR3;
  _IO_ uint32_t CCR4;
  //_IO_ uint32_t BDTR;
  _IO_ uint32_t DCR;
  _IO_ uint32_t DMAR;
  _IO_ uint32_t OR;
};

typedef enum{
  //CR1
  COUNTER_EN = 1,
  UPDATE_DIS = 1 << 1, UPDATE_EN = 0 <<1,
  OV_UV_UPDATE_REQUEST_ONLY = 1 << 2,
  ONE_PULSE_MODE = 1 << 3,
  DOWNCOUNTER = 1 << 4, UPCOUNTER = 0 << 4,
  CENTER_ALLIGNED_MODE = 0 << 5, CENTER_ALLIGNED_MODE1 = 1 << 5,
  CENTER_ALLIGNED_MODE2 = 2 << 5, CENTER_ALLIGNED_MODE3 = 3 << 5,
  AUTO_RELOAD_PRELOAD_EN = 1 << 7,
  CLK_DIVISION_RATIO_1 = 0 << 8, CLK_DIVISION_RATIO_2 = 1 << 8,
  CLK_DIVISION_RATIO_4 = 2 << 8,

  //CR2
  PRELOAD_CCxE_CCxNE_OCxM = 1 << (0 + 16),
  UPDATE_BY_COMG_TRGI_RISING = 1 << (2 + 16),
  SEND_REQUEST_UPDATE_EVENT = 1 << (3 + 16),
  SEND_REQUEST_CCx_EVENT = 0 << (3 + 16),
  MASTER_RESET = 0 << (4 + 16),
  MASTER_ENABLE = 1 << (4 + 16),
  MASTER_UPDATE = 2 << (4 + 16),
  MASTER_COMPARE_PULSE = 3 << (4 + 16),
  MASTER_COMPARE_OC1REF = 4 << (4 + 16),
  MASTER_COMPARE_OC2REF = 5 << (4 + 16),
  MASTER_COMPARE_OC3REF = 6 << (4 + 16),
  MASTER_COMPARE_OC4REF = 7 << (4 + 16),
  TI1_CONNECT_CH1_CH2_CH3 = 1 << (7 + 16),
  SET_OC1_WHEN_MOE_RESETTED = 1 << (8 + 16),
  SET_OC1N_AFTER_DEADTIMME_WHEN_MOE_RESETTED = 1 << (9 + 16),
  SET_OC2_WHEN_MOE_RESETTED = 1 << (10 + 16),
  SET_OC2N_AFTER_DEADTIMME_WHEN_MOE_RESETTED = 1 << (11 + 16),
  SET_OC3_WHEN_MOE_RESETTED = 1 << (12 + 16),
  SET_OC3N_AFTER_DEADTIMME_WHEN_MOE_RESETTED = 1 << (13 + 16),
  SET_OC4_WHEN_MOE_RESETTED = 1 << (14 + 16),

  //SMCR
  SLAVE_MODE_DISABLED = 0LL << (0 + 32),
  ENCODER_MODE_1 = 0LL << (0 + 32),
  ENCODER_MODE_2 = 0LL << (0 + 32),
  ENCODER_MODE_3 = 0LL << (0 + 32),
  RESET_MODE = 4LL << (0 + 32),
  GATED_MODE = 5LL << (0 + 32),
  TRIGGER_SEL_MODE = 6LL << (0 + 32),
  EXT_CLK_MODE = 7LL << (0 + 32),
  INT_TRIGGER_0 = 0LL << (4 + 32),
  INT_TRIGGER_1 = 1LL << (4 + 32),
  INT_TRIGGER_2 = 2LL << (4 + 32),
  INT_TRIGGER_3 = 3LL << (4 + 32),
  TI1_EDGE_DETECTOR = 4LL << (4 + 32),
  FILTERED_TIM_IN_1 = 5LL << (4 + 32),
  FILTERED_TIM_IN_2 = 6LL << (4 + 32),
  EXT_TRIG_IN = 7LL << (4 + 32),
  DELAY_TRIG_IN_EVENT = 1LL << (7 + 32),
  NO_FILTER_FDTS_INT_2N = 0LL << (8 + 32),
  FCK_INT_2N = 1LL << (8 + 32),
  FCK_INT_4N = 2LL << (8 + 32),
  FCK_INT_8N = 3LL << (8 + 32),
  FDTS_DIV_2_6N = 4LL << (8 + 32),
  FDTS_DIV_2_8N = 5LL << (8 + 32),
  FDTS_DIV_4_6N = 6LL << (8 + 32),
  FDTS_DIV_4_8N = 7LL << (8 + 32),
  FDTS_DIV_8_6N = 8LL << (8 + 32),
  FDTS_DIV_8_8N = 9LL << (8 + 32),
  FDTS_DIV_16_5N = 10LL << (8 + 32),
  FDTS_DIV_16_6N = 11LL << (8 + 32),
  FDTS_DIV_16_8N = 12LL << (8 + 32),
  FDTS_DIV_32_5N = 13LL << (8 + 32),
  FDTS_DIV_32_6N = 14LL << (8 + 32),
  FDTS_DIV_32_8N = 15LL << (8 + 32),
  EXT_PRESCALER_OFF = 0LL << (12 + 32),
  EXT_PRESCALER_ETRP_DIV_2 = 1LL << (12 + 32),
  EXT_PRESCALER_ETRP_DIV_4 = 2LL << (12 + 32),
  EXT_PRESCALER_ETRP_DIV_8 = 3LL << (12 + 32),
  EXT_CLK_EN = 1LL << (14 + 32),
  EXT_TRIG_POL_INVERTED = 1LL << (15 + 32),

  //DIER
  UIE_EN = 1LL << (0 + 48),
  CC1IE_EN = 1LL << (1 + 48),
  CC2IE_EN = 1LL << (2 + 48),
  CC3IE_EN = 1LL << (3 + 48),
  CC4IE_EN = 1LL << (4 + 48),
  COMIE_EN = 1LL << (5 + 48),
  TIE_EN = 1LL << (6 + 48),
  BIE_EN = 1LL << (7 + 48),
  UDE_EN = 1LL << (8 + 48),
  CC1DE_EN = 1LL << (9 + 48),
  CC2DE_EN = 1LL << (10 + 48),
  CC3DE_EN = 1LL << (11 + 48),
  CC4DE_EN = 1LL << (12 + 48),
  COMDE_EN = 1LL << (13 + 48),
  TDE_EN = 1LL << (14 + 48),
}BasicTimConfig;

typedef enum{
  //EGR
  UPDATE_GEN = 1,
  CC1_GEN = 1 << 1,
  CC2_GEN = 1 << 2,
  CC3_GEN = 1 << 3,
  CC4_GEN = 1 << 4,
  COM_GEN = 1 << 5,
  TRIG_GEN = 1 << 6,
  BK_GEN = 1 << 7,
}SpecificTimConfig1;

typedef enum{
  //CCMR
  SEL_OUT = 0,
  SEL_IN_TI1 = 1,
  SEL_IN_TI2 = 2,
  SEL_IN_TRC = 3,

  FAST_EN = 1 << 2,
  PRELOAD_EN = 1 << 3,
  FROZEN = 0 << 4,
  CH1_ACT_LVL = 1 << 4,
  CH1_INACT_LVL = 2 << 4,
  TOGGLE = 3 << 4,
  FORCE_INACT_LVL = 4 << 4,
  FORCE_ACT_LVL = 5 << 4,
  PWN_MOD_1 = 6 << 4,
  PWM_MOD_2 = 7 << 4,
  OC_CLR_EN = 1 << 7,

  IN_CAP_NO_PRESCALER = 0 << 2,
  IN_CAP_PRESCALER_2 = 1 << 2,
  IN_CAP_PRESCALER_4 = 2 << 2,
  IN_CAP_PRESCALER_8 = 3 << 2,
  CC_FCK_INT_2N = 1 << 4,
  CC_FCK_INT_4N = 2 << 4,
  CC_FCK_INT_8N = 3 << 4,
  CC_FDTS_DIV_2_6N = 4 << 4,
  CC_FDTS_DIV_2_8N = 5 << 4,
  CC_FDTS_DIV_4_6N = 6 << 4,
  CC_FDTS_DIV_4_8N = 7 << 4,
  CC_FDTS_DIV_8_6N = 8 << 4,
  CC_FDTS_DIV_8_8N = 9 << 4,
  CC_FDTS_DIV_16_5N = 10 << 4,
  CC_FDTS_DIV_16_6N = 11 << 4,
  CC_FDTS_DIV_16_8N = 12 << 4,
  CC_FDTS_DIV_32_5N = 13 << 4,
  CC_FDTS_DIV_32_6N = 14 << 4,
  CC_FDTS_DIV_32_8N = 15 << 4,

  //CCER
  CC1N_OUT_EN = 1LL << (2 + 32),
  CC1NP_ACT_LOW = 1LL << (3 + 32),
  CC2N_OUT_EN = 1LL << (6 + 32),
  CC2NP_ACT_LOW = 1LL << (7 + 32),
  CC3N_OUT_EN = 1LL << (10 + 32),
  CC3NP_ACT_LOW = 1LL << (11 + 32),
  //-----------------OUTPUT---------------
  CC1_OUT_EN = 1LL << (0 + 32),
  CC1P_ACT_LOW = 1LL << (1 + 32),
  CC2_OUT_EN = 1LL << (4 + 32),
  CC2P_ACT_LOW = 1LL << (5 + 32),
  CC3_OUT_EN = 1LL << (8 + 32),
  CC3P_ACT_LOW = 1LL << (9 + 32),
  CC4_OUT_EN = 1LL << (12 + 32),
  CC4P_ACT_LOW = 1LL << (13 + 32),
  //----------------INPUT-----------------
  CC1_CAP_EN = 1LL << (0 + 32),
  CC1P_IN_SET = 1LL << (1 + 32),
  CC2_CAP_EN = 1LL << (4 + 32),
  CC2P_IN_SET = 1LL << (5 + 32),
  CC3_CAP_EN = 1LL << (8 + 32),
  CC3P_IN_SET = 1LL << (9 + 32),
  CC4_CAP_EN = 1LL << (12 + 32),
  CC4P_IN_SET = 1LL << (13 + 32),
}SpecificTim1Config2;

void  timerBasicConfig(TimReg *timer, BasicTimConfig  config);
void  timerEventGenerationConfig(TimReg *timer, SpecificTimConfig1 config);
void  timerCaptureCompareConfig(TimReg *timer, int  channel, SpecificTim1Config2  config);
void  timSetPrescalerReg(TimReg *timer, uint32_t  value);
void  timSetARRReg(TimReg *timer, uint32_t  value);
void timerConfigure(TimReg *timer);
void  setFrequency(TimReg *timer, long  double  desiredFrequency, uint32_t  arrValue);
#endif /* INC_TIM1_H_ */
