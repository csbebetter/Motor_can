#include "can.h"
#include "motor.h"

extern volatile MotorTypeDef Motor_1, Motor_2,Motor_3, Motor_4;
extern volatile MotorTypeDef Motor_5;

void CAN1_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure; 
	CAN_InitTypeDef CAN_InitStructure;
	CAN_FilterInitTypeDef CAN_FilterInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);	               											 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11| GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;			
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;			
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;		
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;			
	GPIO_Init(GPIOA, &GPIO_InitStructure);					
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource11,GPIO_AF_CAN1);	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource12,GPIO_AF_CAN1);	
	
	CAN_DeInit(CAN1);
	CAN_StructInit(&CAN_InitStructure);
	
	CAN_InitStructure.CAN_TTCM= DISABLE;		  
	CAN_InitStructure.CAN_ABOM= DISABLE;		
	CAN_InitStructure.CAN_AWUM= DISABLE;		
	CAN_InitStructure.CAN_NART= ENABLE;			 
	CAN_InitStructure.CAN_RFLM= DISABLE;		
	CAN_InitStructure.CAN_TXFP= DISABLE;		
	CAN_InitStructure.CAN_Mode= CAN_Mode_Normal;	
	CAN_InitStructure.CAN_SJW= CAN_SJW_1tq;			
	CAN_InitStructure.CAN_BS1= CAN_BS1_9tq;			
	CAN_InitStructure.CAN_BS2= CAN_BS2_4tq;			
	CAN_InitStructure.CAN_Prescaler= 3;				
	CAN_Init(CAN1, &CAN_InitStructure);		
	
	CAN_FilterInitStructure.CAN_FilterNumber= 0;							
	CAN_FilterInitStructure.CAN_FilterMode= CAN_FilterMode_IdMask; 
	CAN_FilterInitStructure.CAN_FilterScale= CAN_FilterScale_32bit;		
	CAN_FilterInitStructure.CAN_FilterIdHigh= 0x0000;
	CAN_FilterInitStructure.CAN_FilterIdLow= 0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh= 0x0000;	 
	CAN_FilterInitStructure.CAN_FilterMaskIdLow= 0x0000;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment= CAN_Filter_FIFO0;	
	CAN_FilterInitStructure.CAN_FilterActivation= ENABLE;
	CAN_FilterInit(&CAN_FilterInitStructure);	

	CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);	
	CAN_ITConfig(CAN1,CAN_IT_TME,ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;		
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;				
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = CAN1_TX_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;		
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void CAN1_TX_IRQHandler(void)
{
	if(CAN_GetITStatus(CAN1,CAN_IT_TME)!= RESET)
	{
		CAN_ClearITPendingBit(CAN1,CAN_IT_TME);
	}
}

/* CAN1 RX IRQ */
void CAN1_RX0_IRQHandler(void)
{
	CanRxMsg RxMessage;

	CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
	if(RxMessage.StdId == Motor_1_ID)
	{    
		Motor_1.CurrentMeasure=(float)(short)(RxMessage.Data[4]<<8 | RxMessage.Data[5]);
		Motor_1.SpeedMeasure =((float)(short)(RxMessage.Data[2]<<8 | RxMessage.Data[3]))/19.0;//*187.0/3591;   
		if(Motor_1.PosPre == 0 && Motor_1.PosNow == 0 )
			Motor_1.PosPre = Motor_1.PosNow = (short)(RxMessage.Data[0]<<8 | RxMessage.Data[1]);
		else
		{
			Motor_1.PosPre = Motor_1.PosNow;
			Motor_1.PosNow =  (short)(RxMessage.Data[0]<<8 | RxMessage.Data[1]);
		}
		Motor_1.PositionMeasure += Get_RM3508_Distance(Motor_1);
	}
	else if(RxMessage.StdId == Motor_2_ID)
	{
		Motor_2.CurrentMeasure=(float)(short)(RxMessage.Data[4]<<8 | RxMessage.Data[5]);
		Motor_2.SpeedMeasure =((float)(short)(RxMessage.Data[2]<<8 | RxMessage.Data[3]))/19.0;//*187.0/3591;   
		if(Motor_2.PosPre == 0 && Motor_2.PosNow == 0 )
			Motor_2.PosPre = Motor_2.PosNow = (short)(RxMessage.Data[0]<<8 | RxMessage.Data[1]);
		else
		{
			Motor_2.PosPre = Motor_2.PosNow;
			Motor_2.PosNow =  (short)(RxMessage.Data[0]<<8 | RxMessage.Data[1]);
		}
		Motor_2.PositionMeasure += Get_RM3508_Distance(Motor_2);
	}
	else if(RxMessage.StdId == Motor_3_ID)
	{
        
		Motor_3.CurrentMeasure=(float)(short)(RxMessage.Data[4]<<8 | RxMessage.Data[5]);
		Motor_3.SpeedMeasure =((float)(short)(RxMessage.Data[2]<<8 | RxMessage.Data[3]))/19.0;//*187.0/3591;   
		if(Motor_3.PosPre == 0 && Motor_3.PosNow == 0 )
			Motor_3.PosPre = Motor_3.PosNow = (short)(RxMessage.Data[0]<<8 | RxMessage.Data[1]);
		else
		{
			Motor_3.PosPre = Motor_3.PosNow;
			Motor_3.PosNow =  (short)(RxMessage.Data[0]<<8 | RxMessage.Data[1]);
		}
		Motor_3.PositionMeasure += Get_RM3508_Distance(Motor_3);
	}
	
		else if(RxMessage.StdId == Motor_4_ID)
	{
        
		Motor_4.CurrentMeasure=(float)(short)(RxMessage.Data[4]<<8 | RxMessage.Data[5]);
		Motor_4.SpeedMeasure =((float)(short)(RxMessage.Data[2]<<8 | RxMessage.Data[3]))/19.0;//*187.0/3591;   
		if(Motor_4.PosPre == 0 && Motor_4.PosNow == 0 )
			Motor_4.PosPre = Motor_4.PosNow = (short)(RxMessage.Data[0]<<8 | RxMessage.Data[1]);
		else
		{
			Motor_4.PosPre = Motor_4.PosNow;
			Motor_4.PosNow =  (short)(RxMessage.Data[0]<<8 | RxMessage.Data[1]);
		}
		Motor_4.PositionMeasure += Get_RM3508_Distance(Motor_4);
	}
    else if(RxMessage.StdId == Motor_5_ID)
	{ 
        
		Motor_5.CurrentMeasure=(float)(short)(RxMessage.Data[4]<<8 | RxMessage.Data[5]);
		Motor_5.SpeedMeasure =((float)(short)(RxMessage.Data[2]<<8 | RxMessage.Data[3]))/19.0;//*187.0/3591;   
		if(Motor_5.PosPre == 0 && Motor_5.PosNow == 0 )
			Motor_5.PosPre = Motor_5.PosNow = (short)(RxMessage.Data[0]<<8 | RxMessage.Data[1]);
		else
		{
			Motor_5.PosPre = Motor_5.PosNow;
			Motor_5.PosNow =  (short)(RxMessage.Data[0]<<8 | RxMessage.Data[1]);
		}
		Motor_5.PositionMeasure += Get_RM3508_Distance(Motor_5);
	}
}

void Motor_Set_Current(signed short int i1, signed short int i2, signed short int i3, signed short int i4)
{
	CanTxMsg tx_message;
	tx_message.IDE = CAN_Id_Standard;
	tx_message.RTR = CAN_RTR_DATA;
	tx_message.DLC = 0x08;
	tx_message.StdId = 0x200;
	
	tx_message.Data[0] = i1 >> 8;
	tx_message.Data[1] = i1;
	tx_message.Data[2] = i2 >> 8;
	tx_message.Data[3] = i2;
	tx_message.Data[4] = i3 >> 8;
	tx_message.Data[5] = i3;
	tx_message.Data[6] = i4 >> 8;
	tx_message.Data[7] = i4;
	
	CAN_Transmit(CAN1,&tx_message);
}

void Lift_Motor_Set_Current(signed short int i5)
{
	CanTxMsg tx_message;
	tx_message.IDE = CAN_Id_Standard;
	tx_message.RTR = CAN_RTR_DATA;
	tx_message.DLC = 0x02;
	tx_message.StdId = 0x1FF;
	
	tx_message.Data[0] = i5 >> 8;
	tx_message.Data[1] = i5;
	
	CAN_Transmit(CAN1,&tx_message);
}
