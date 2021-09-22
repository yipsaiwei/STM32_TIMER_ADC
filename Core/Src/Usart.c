/*
 * Usart.c
 *
 *  Created on: Jul 17, 2021
 *      Author: Yip Sai Wei
 */

#include  "Usart.h"


void  usartSetBaudRate(UsartReg *usart, int baudrate){
  uint32_t  apbFreq = returnUsartFrequency(usart);
  int OVER8 = (usart->CR1 >> 15) & 1;
  double USARTDIV = (double) apbFreq / (baudrate * (8 * (2-OVER8)));
  int mantissa = USARTDIV;
  double decimalPoint = USARTDIV - mantissa;
  int fraction;
  if(OVER8)
    fraction = decimalPoint * 8;
  else
    fraction = decimalPoint * 16;

  usart->BRR = (mantissa << 4) | fraction;
}

/*
void  usartConfigureCR1(UsartReg *usart, int config){
  usart->CR1 = config;
}

void  usartConfigureCR2(UsartReg *usart, int config){
  usart->CR2 = config;
}

void  usartConfigureCR3(UsartReg *usart, int config){
  usart->CR3 = config;
}
*/

void  usartConfigure(UsartReg *usart, UsartConfig config){
 usart->CR1 &= USART_CR_MASK;
 usart->CR1 |= (config & 0xFFFF);

 usart->CR2 &= USART_CR_MASK;
 usart->CR2 |= (config & 0xFFFFFFFF) >> 16;

 usart->CR3 &= USART_CR_MASK;
 usart->CR3 |= config >> 32;
}

void  writeToDataRegister(UsartReg *usart, uint8_t  data){
 usart->DR &= USART_DR_MASK;
 usart->DR = data;
}

int isTransmissionComplete(UsartReg *usart){
 return (usart->SR & (1 << 6));
}

int isTDREmpty(UsartReg *usart){
 return (usart->SR & (1 << 7));
}

int isRDRAvailableForRead(UsartReg *usart){
  return  (usart->SR & (1 <<5));
}

int returnRDRValue(UsartReg *usart){
  return  usart->DR;
}

uint32_t returnUsartFrequency(UsartReg *usart){
  if(usart == usart1 || usart == usart6)
    return  HAL_RCC_GetPCLK2Freq();
  else
    return  HAL_RCC_GetPCLK1Freq();
}

Status  usartTransmit(UsartReg  *usart, char  *msg){
  for(int  i = 0; msg[i] != '\0'; i++){
    while(!isTDREmpty(usart));
    writeToDataRegister(usart, msg[i]);
    while(!isTransmissionComplete(usart));
    }
  return  OK;
}

Status readFromUsart(UsartReg  *usart, char  *msg){
  char  *charptr = msg;
  do{
    while(!isRDRAvailableForRead(usart));
    charptr = (char *)returnRDRValue(usart);
    charptr++;
  }while(*msg != '\0');
  return  OK;
}
