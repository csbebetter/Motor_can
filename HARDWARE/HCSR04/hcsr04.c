//#include "stm32f4xx.h"
//#include "usart.h"
//#include "delay.h"

//void GPIO_Configuration(void)//ÉèÖÃËùÐèÒªµÄÒý½Å¹ØÏµ
//{
//  GPIO_InitTypeDef  GPIO_InitStructure;
//  
//  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);  //ÔÊÐíÊ¹ÓÃF×éµÄGPIO¿Ú
//  /*echo*/
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;//Ö¸¶¨echoµÄÒý½Å
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//Ö¸¶¨Îªin
//  GPIO_Init(GPIOC, &GPIO_InitStructure);
//  /*trig*/
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;//Ö¸¶¨trigµÄÒý½Å
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//Ö¸¶¨Îªout
//  GPIO_Init(GPIOC, &GPIO_InitStructure);
// 
//}
// 
//void TIM2_Configuration(u16 arr, u16 psc)
//{
//  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructrue;
//  NVIC_InitTypeDef NVIC_InitStructure;
//  
//  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);//ÔÊÐíÊ¹ÓÃTIM2
//  TIM_DeInit(TIM2);//¸´Î»TIM2µÄËùÓÐ¼Ä´æÆ÷
//  
//  TIM_TimeBaseStructrue.TIM_Period = arr;  //¼ÆÊýÖÜÆÚ                 
//  TIM_TimeBaseStructrue.TIM_Prescaler = psc;   //·ÖÆµÖµ             
//  TIM_TimeBaseStructrue.TIM_ClockDivision = TIM_CKD_DIV1;//²»ÖªµÀÕâÁ©ÊÇÊ²Ã´
//  TIM_TimeBaseStructrue.TIM_CounterMode = TIM_CounterMode_Up;
//  
//  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructrue);//³õÊ¼»¯
//  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
//  
//  NVIC_InitStructure.NVIC_IRQChannel=TIM2_IRQn;             
//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x00; 
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x01;       
//  NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
//  
//  NVIC_Init(&NVIC_InitStructure);                           
//  
//  TIM_Cmd(TIM2, DISABLE); 
//}

//u16 TIM2_Flag;
//int get_Diatance()
//{
//  int distance = 0;
//  u16 TIM = 0;
//  TIM_Cmd(TIM2, ENABLE);           
//  
//  GPIO_SetBits(GPIOC, GPIO_Pin_6);  //trigÒý½ÅµçÆ½À­¸ß
//  delay_us(300);
//  GPIO_ResetBits(GPIOC, GPIO_Pin_6); //À­µÍ
// 
//  while((!GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_5))&&TIM2_Flag==0);  //¶ÁÈ¡Òý½ÅµçÆ½
//  TIM2->CNT = 0;              
//  while(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_5)&&TIM2_Flag==0);    
//  TIM_Cmd(TIM2, DISABLE);    
//  
//  if(TIM2_Flag==1)
//    TIM2_Flag = 0;
//  
//  TIM = TIM_GetCounter(TIM2);
//  distance = TIM*0.85;
//  return distance;
//}

//void TIM2_IRQHandler(void)
//{
//  if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
//  {
//    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
//    TIM2_Flag=1;
//  }
//}

//PD
#include "stm32f4xx.h"
#include "usart.h"
#include "delay.h"

void GPIO_Configuration3()//ÉèÖÃËùÐèÒªµÄÒý½Å¹ØÏµ
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);  //ÔÊÐíÊ¹ÓÃF×éµÄGPIO¿Ú
  /*echo*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;//Ö¸¶¨echoµÄÒý½Å
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//Ö¸¶¨Îªin
  GPIO_Init(GPIOE, &GPIO_InitStructure);
  /*trig*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;//Ö¸¶¨trigµÄÒý½Å
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//Ö¸¶¨Îªout
  GPIO_Init(GPIOE, &GPIO_InitStructure);
 
}
 
void TIM2_Configuration(u16 arr, u16 psc)
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructrue;
  NVIC_InitTypeDef NVIC_InitStructure;
  
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);//ÔÊÐíÊ¹ÓÃTIM2
  TIM_DeInit(TIM2);//¸´Î»TIM2µÄËùÓÐ¼Ä´æÆ÷
  
  TIM_TimeBaseStructrue.TIM_Period = arr;  //¼ÆÊýÖÜÆÚ                 
  TIM_TimeBaseStructrue.TIM_Prescaler = psc;   //·ÖÆµÖµ             
  TIM_TimeBaseStructrue.TIM_ClockDivision = TIM_CKD_DIV1;//²»ÖªµÀÕâÁ©ÊÇÊ²Ã´
  TIM_TimeBaseStructrue.TIM_CounterMode = TIM_CounterMode_Up;
  
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructrue);//³õÊ¼»¯
  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
  
  NVIC_InitStructure.NVIC_IRQChannel=TIM2_IRQn;             
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x00; 
  NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x01;       
  NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
  
  NVIC_Init(&NVIC_InitStructure);                           
  
  TIM_Cmd(TIM2, DISABLE); 
}

u16 TIM2_Flag;
int get_Diatance()
{
  int distance = 0;
  u16 TIM = 0;
  TIM_Cmd(TIM2, ENABLE);           
  
  GPIO_SetBits(GPIOE, GPIO_Pin_8);  //trigÒý½ÅµçÆ½À­¸ß
  delay_us(300);
  GPIO_ResetBits(GPIOE, GPIO_Pin_8); //À­µÍ
 
  while((!GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_7))&&TIM2_Flag==0);  //¶ÁÈ¡Òý½ÅµçÆ½
  TIM2->CNT = 0;              
  while(GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_7)&&TIM2_Flag==0);    
  TIM_Cmd(TIM2, DISABLE);    
  
  if(TIM2_Flag==1)
    TIM2_Flag = 0;
  
  TIM = TIM_GetCounter(TIM2);
  distance = TIM*0.85;
  return distance;
}

void TIM2_IRQHandler(void)
{
  if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
  {
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    TIM2_Flag=1;
  }
}