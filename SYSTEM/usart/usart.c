#include "sys.h"
#include "usart.h"	
#include "delay.h"
////////////////////////////////////////////////////////////////////////////////// 	 
//如果使用ucos,则包括下面的头文件即可.
#if SYSTEM_SUPPORT_OS
#include "includes.h"					//ucos 使用	  
#endif

//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{ 	
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
	USART1->DR = (u8) ch;      
	return ch;
}
#endif
 
#if EN_USART1_RX   //如果使能了接收
//串口1中断服务程序
//注意,读取USARTx->SR能避免莫名其妙的错误   	
u8 USART_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
//接收状态
//bit15，	接收完成标志
//bit14，	接收到0x0d
//bit13~0，	接收到的有效字节数目
u16 USART_RX_STA=0;       //接收状态标记	

//初始化IO 串口1 
//bound:波特率
void uart_init(u32 bound){
   //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //使能GPIOA时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//使能USART1时钟
 
	//串口1对应引脚复用映射
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //GPIOA9复用为USART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //GPIOA10复用为USART1
	
	//USART1端口配置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; //GPIOA9与GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化PA9，PA10

   //USART1 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART1, &USART_InitStructure); //初始化串口1
	
  USART_Cmd(USART1, ENABLE);  //使能串口1 
	
	//USART_ClearFlag(USART1, USART_FLAG_TC);
	
#if EN_USART1_RX	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启相关中断

	//Usart1 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//串口1中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、

#endif
	
}

u16 USART_RX_STA0[16] = { 0 };       //接收状态标记	  
u8 Num3 = 0;              //接收数据的当前位置
u16 USART_RX_STA1[16] = { 0 };       //接收状态标记	  
u8 Num2 = 0;              //接收数据的当前位置

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
	unsigned int Receive_data[3] = { 0 };       //数据缓存区
	//Num=0;
	
	while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
	USART_SendData(USART3, 0x57);																//命令起始信号
	while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);	

	while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
	USART_SendData(USART3, ID);																  //ID模块编号
	while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
	delay_ms(20);
	
	while(1)
  {
		if(USART_RX_STA0[0] != 0x75) { Num3 = 0; } 
		if(Num3 == 8)
		{
			Num3 = 0;
			if(USART_RX_STA0[7] == 0x07)  //判断帧尾0x07,否者不赋值
			{
				Receive_data[0] = USART_RX_STA0[1];
				Receive_data[0] <<= 8;
				Receive_data[0] |= USART_RX_STA0[2];        
				*Data = Receive_data[0];          //距离
				
				Receive_data[1] = USART_RX_STA0[3];
				Receive_data[1] <<= 8;
				Receive_data[1] |= USART_RX_STA0[4];
				*(Data+1) = Receive_data[1];          //环境质量
				
				Receive_data[2] = USART_RX_STA0[5];
				Receive_data[2] <<= 8;
				Receive_data[2] |= USART_RX_STA0[6];
				*(Data+2) = Receive_data[2];         //环境光强        
				
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
	unsigned int Receive_data[3] = { 0 };       //数据缓存区
	//Num=0;
	
	while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
	USART_SendData(USART2, 0x57);																//命令起始信号
	while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);	

	while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
	USART_SendData(USART2, ID);																  //ID模块编号
	while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
	delay_ms(200);
	
	while(1)
  {
		if(USART_RX_STA1[0] != 0x75) { Num2 = 0; } 
		if(Num2 == 8)
		{
			Num2 = 0;
			if(USART_RX_STA1[7] == 0x07)  //判断帧尾0x07,否者不赋值
			{
				Receive_data[0] = USART_RX_STA1[1];
				Receive_data[0] <<= 8;
				Receive_data[0] |= USART_RX_STA1[2];        
				*Data = Receive_data[0];          //距离
				
				Receive_data[1] = USART_RX_STA1[3];
				Receive_data[1] <<= 8;
				Receive_data[1] |= USART_RX_STA1[4];
				*(Data+1) = Receive_data[1];          //环境质量
				
				Receive_data[2] = USART_RX_STA1[5];
				Receive_data[2] <<= 8;
				Receive_data[2] |= USART_RX_STA1[6];
				*(Data+2) = Receive_data[2];         //环境光强        
				
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
 *  函数名称：Set_LaserDis
 *  函数功能：设置功能参数
 *  形    参：ID: 模块编号,Fun: 功能项,Par: 参数,
 *  输    出：无
 *  备    注：无
 ********************************************************************/
void Set_LaserDis_Usart3(unsigned char ID, unsigned char Fun,unsigned char Par)	       
{	
 ///////////////////////////设置功能参数///////////////////////////////	
	while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
	USART_SendData(USART3, 0x4C);																//命令起始信号
	while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);	
	
	while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
	USART_SendData(USART3, ID);																  //ID模块编号
	while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
	
	while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
	USART_SendData(USART3, Fun);																//功能项
	while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);	
	
	while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
	USART_SendData(USART3, Par);																//参数
	while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);

///////////////////////////设置功能参数///////////////////////////////		
}

void Set_LaserDis_Usart2(unsigned char ID, unsigned char Fun,unsigned char Par)	       
{	
 ///////////////////////////设置功能参数///////////////////////////////	
	while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
	USART_SendData(USART2, 0x4C);																//命令起始信号
	while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);	
	
	while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
	USART_SendData(USART2, ID);																  //ID模块编号
	while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
	
	while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
	USART_SendData(USART2, Fun);																//功能项
	while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);	
	
	while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
	USART_SendData(USART2, Par);																//参数
	while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);

///////////////////////////设置功能参数///////////////////////////////		
}

void USART1_IRQHandler(void)                	//串口1中断服务程序
{
	u8 Res;
#if SYSTEM_SUPPORT_OS 		//如果SYSTEM_SUPPORT_OS为真，则需要支持OS.
	OSIntEnter();    
#endif
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
	{
		Res =USART_ReceiveData(USART1);//(USART1->DR);	//读取接收到的数据
		
		if((USART_RX_STA&0x8000)==0)//接收未完成
		{
			if(USART_RX_STA&0x4000)//接收到了0x0d
			{
				if(Res!=0x0a)USART_RX_STA=0;//接收错误,重新开始
				else USART_RX_STA|=0x8000;	//接收完成了 
			}
			else //还没收到0X0D
			{	
				if(Res==0x0d)USART_RX_STA|=0x4000;
				else
				{
					USART_RX_BUF[USART_RX_STA&0X3FFF]=Res ;
					USART_RX_STA++;
					if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;//接收数据错误,重新开始接收	  
				}		 
			}
		}   		 
  } 
#if SYSTEM_SUPPORT_OS 	//如果SYSTEM_SUPPORT_OS为真，则需要支持OS.
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

 



