#include "sys.h"
#include "usart.h"	
#include "delay.h"
////////////////////////////////////////////////////////////////////////////////// 	 
//���ʹ��ucos,����������ͷ�ļ�����.
#if SYSTEM_SUPPORT_OS
#include "includes.h"					//ucos ʹ��	  
#endif

//�������´���,֧��printf����,������Ҫѡ��use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{ 	
	while((USART1->SR&0X40)==0);//ѭ������,ֱ���������   
	USART1->DR = (u8) ch;      
	return ch;
}
#endif
 
#if EN_USART1_RX   //���ʹ���˽���
//����1�жϷ������
//ע��,��ȡUSARTx->SR�ܱ���Ī������Ĵ���   	
u8 USART_RX_BUF[USART_REC_LEN];     //���ջ���,���USART_REC_LEN���ֽ�.
//����״̬
//bit15��	������ɱ�־
//bit14��	���յ�0x0d
//bit13~0��	���յ�����Ч�ֽ���Ŀ
u16 USART_RX_STA=0;       //����״̬���	

//��ʼ��IO ����1 
//bound:������
void uart_init(u32 bound){
   //GPIO�˿�����
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //ʹ��GPIOAʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//ʹ��USART1ʱ��
 
	//����1��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //GPIOA9����ΪUSART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //GPIOA10����ΪUSART1
	
	//USART1�˿�����
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; //GPIOA9��GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOA,&GPIO_InitStructure); //��ʼ��PA9��PA10

   //USART1 ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
  USART_Init(USART1, &USART_InitStructure); //��ʼ������1
	
  USART_Cmd(USART1, ENABLE);  //ʹ�ܴ���1 
	
	//USART_ClearFlag(USART1, USART_FLAG_TC);
	
#if EN_USART1_RX	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//��������ж�

	//Usart1 NVIC ����
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//����1�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����

#endif
	
}

u16 USART_RX_STA0[16] = { 0 };       //����״̬���	  
u8 Num3 = 0;              //�������ݵĵ�ǰλ��
u16 USART_RX_STA1[16] = { 0 };       //����״̬���	  
u8 Num2 = 0;              //�������ݵĵ�ǰλ��

void uart3_init(u32 bound)
{
   //GPIO????
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE); //??GPIOB??
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);//??USART3??
 
	//??3????????
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3); //GPIOA10???USART3
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3); //GPIOA11???USART3
	
	//USART3????
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_10; //GPIOA11?GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//????
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//??50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //??????
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //??
	GPIO_Init(GPIOB,&GPIO_InitStructure); //???
 
   //USART3 ?????
	USART_InitStructure.USART_BaudRate = bound;//?????
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//???8?????
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//?????
	USART_InitStructure.USART_Parity = USART_Parity_No;//??????
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//????????
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//????
	USART_Init(USART3, &USART_InitStructure); //?????1
		
	USART_Cmd(USART3, ENABLE);  //????3
	
	//USART_ClearFlag(USART1, USART_FLAG_TC);
	
//#if EN_USART1_RX	
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//??????
 
	//Usart1 NVIC ??
   NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;//??1????
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;//?????3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//????3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ????
	NVIC_Init(&NVIC_InitStructure);	//??????????VIC????
 
//#endif
	
}

void uart2_init(u32 bound)
{
   //GPIO????
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //??GPIOB??
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);//??USART3??
 
	//??3????????
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2); //GPIOA10???USART3
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2); //GPIOA11???USART3
	
	//USART3????
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_2; //GPIOA11?GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//????
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//??50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //??????
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //??
	GPIO_Init(GPIOA,&GPIO_InitStructure); //???
 
   //USART3 ?????
	USART_InitStructure.USART_BaudRate = bound;//?????
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//???8?????
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//?????
	USART_InitStructure.USART_Parity = USART_Parity_No;//??????
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//????????
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//????
	USART_Init(USART2, &USART_InitStructure); //?????1
		
	USART_Cmd(USART2, ENABLE);  //????3
	
	//USART_ClearFlag(USART1, USART_FLAG_TC);
	
//#if EN_USART1_RX	
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//??????
 
	//Usart1 NVIC ??
   NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;//??1????
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;//?????3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//????3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ????
	NVIC_Init(&NVIC_InitStructure);	//??????????VIC????
 
//#endif
	
}

void Read_LaserDis_Usart3(unsigned char ID, unsigned int *Data)
{
	
	
	unsigned char y=0;
	unsigned int Receive_data[3] = { 0 };       //���ݻ�����
	//Num=0;
	
	while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
	USART_SendData(USART3, 0x57);																//������ʼ�ź�
	while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);	

	while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
	USART_SendData(USART3, ID);																  //IDģ����
	while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
	delay_ms(20);
	
	while(1)
  {
		if(USART_RX_STA0[0] != 0x75) { Num3 = 0; } 
		if(Num3 == 8)
		{
			Num3 = 0;
			if(USART_RX_STA0[7] == 0x07)  //�ж�֡β0x07,���߲���ֵ
			{
				Receive_data[0] = USART_RX_STA0[1];
				Receive_data[0] <<= 8;
				Receive_data[0] |= USART_RX_STA0[2];        
				*Data = Receive_data[0];          //����
				
				Receive_data[1] = USART_RX_STA0[3];
				Receive_data[1] <<= 8;
				Receive_data[1] |= USART_RX_STA0[4];
				*(Data+1) = Receive_data[1];          //��������
				
				Receive_data[2] = USART_RX_STA0[5];
				Receive_data[2] <<= 8;
				Receive_data[2] |= USART_RX_STA0[6];
				*(Data+2) = Receive_data[2];         //������ǿ        
				
				break;
			}        
			break;
		}
    else
    {
      delay_ms(1);y++;
      if(y==10) { Num3 = 0;break; }
    }
  }
}

void Read_LaserDis_Usart2(unsigned char ID, unsigned int *Data)
{
	
	
	unsigned char y=0;
	unsigned int Receive_data[3] = { 0 };       //���ݻ�����
	//Num=0;
	
	while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
	USART_SendData(USART2, 0x57);																//������ʼ�ź�
	while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);	

	while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
	USART_SendData(USART2, ID);																  //IDģ����
	while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
	delay_ms(200);
	
	while(1)
  {
		if(USART_RX_STA1[0] != 0x75) { Num2 = 0; } 
		if(Num2 == 8)
		{
			Num2 = 0;
			if(USART_RX_STA1[7] == 0x07)  //�ж�֡β0x07,���߲���ֵ
			{
				Receive_data[0] = USART_RX_STA1[1];
				Receive_data[0] <<= 8;
				Receive_data[0] |= USART_RX_STA1[2];        
				*Data = Receive_data[0];          //����
				
				Receive_data[1] = USART_RX_STA1[3];
				Receive_data[1] <<= 8;
				Receive_data[1] |= USART_RX_STA1[4];
				*(Data+1) = Receive_data[1];          //��������
				
				Receive_data[2] = USART_RX_STA1[5];
				Receive_data[2] <<= 8;
				Receive_data[2] |= USART_RX_STA1[6];
				*(Data+2) = Receive_data[2];         //������ǿ        
				
				break;
			}        
			break;
		}
    else
    {
      delay_ms(1);y++;
      if(y==10) { Num2 = 0;break; }
    }
  }
}

/*********************************************************************
 *  �������ƣ�Set_LaserDis
 *  �������ܣ����ù��ܲ���
 *  ��    �Σ�ID: ģ����,Fun: ������,Par: ����,
 *  ��    ������
 *  ��    ע����
 ********************************************************************/
void Set_LaserDis_Usart3(unsigned char ID, unsigned char Fun,unsigned char Par)	       
{	
 ///////////////////////////���ù��ܲ���///////////////////////////////	
	while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
	USART_SendData(USART3, 0x4C);																//������ʼ�ź�
	while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);	
	
	while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
	USART_SendData(USART3, ID);																  //IDģ����
	while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
	
	while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
	USART_SendData(USART3, Fun);																//������
	while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);	
	
	while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
	USART_SendData(USART3, Par);																//����
	while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);

///////////////////////////���ù��ܲ���///////////////////////////////		
}

void Set_LaserDis_Usart2(unsigned char ID, unsigned char Fun,unsigned char Par)	       
{	
 ///////////////////////////���ù��ܲ���///////////////////////////////	
	while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
	USART_SendData(USART2, 0x4C);																//������ʼ�ź�
	while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);	
	
	while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
	USART_SendData(USART2, ID);																  //IDģ����
	while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
	
	while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
	USART_SendData(USART2, Fun);																//������
	while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);	
	
	while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
	USART_SendData(USART2, Par);																//����
	while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);

///////////////////////////���ù��ܲ���///////////////////////////////		
}

void USART1_IRQHandler(void)                	//����1�жϷ������
{
	u8 Res;
#if SYSTEM_SUPPORT_OS 		//���SYSTEM_SUPPORT_OSΪ�棬����Ҫ֧��OS.
	OSIntEnter();    
#endif
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //�����ж�(���յ������ݱ�����0x0d 0x0a��β)
	{
		Res =USART_ReceiveData(USART1);//(USART1->DR);	//��ȡ���յ�������
		
		if((USART_RX_STA&0x8000)==0)//����δ���
		{
			if(USART_RX_STA&0x4000)//���յ���0x0d
			{
				if(Res!=0x0a)USART_RX_STA=0;//���մ���,���¿�ʼ
				else USART_RX_STA|=0x8000;	//��������� 
			}
			else //��û�յ�0X0D
			{	
				if(Res==0x0d)USART_RX_STA|=0x4000;
				else
				{
					USART_RX_BUF[USART_RX_STA&0X3FFF]=Res ;
					USART_RX_STA++;
					if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;//�������ݴ���,���¿�ʼ����	  
				}		 
			}
		}   		 
  } 
#if SYSTEM_SUPPORT_OS 	//���SYSTEM_SUPPORT_OSΪ�棬����Ҫ֧��OS.
	OSIntExit();  											 
#endif
} 

void USART3_IRQHandler(void)
{
	if(USART_GetITStatus(USART3,USART_IT_RXNE)!=RESET)
	{		
		USART_RX_STA0[Num3++] = USART_ReceiveData(USART3);	
	}

}

void USART2_IRQHandler(void)
{
	if(USART_GetITStatus(USART2,USART_IT_RXNE)!=RESET)
	{		
		USART_RX_STA1[Num2++] = USART_ReceiveData(USART2);	
	}

}

#endif	

 



