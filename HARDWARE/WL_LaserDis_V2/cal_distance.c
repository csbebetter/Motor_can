#include "cal_distance.h"
//接线：PB10号口接RXD  11口接TXD
//      PA3 接TXD      2接RXD

void GPIO_Configuration(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);  
  /*echo*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;//指定echo的引脚
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//指定为in
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
}


int cal_distance1()  //下面模块
{
	unsigned int Temp_Data1[3] = { 0 };       //数据缓存区
	unsigned int reading_count1 = 11 ; //记录读数次数
	unsigned int last_time1_max=0 , last_time1_min=999999999;
	unsigned int i1=0;
	unsigned int sum1=0;
	unsigned int distance1=0;
	int flag1=0;
	
	while(i1<=reading_count1)
	{
		i1++;
		if(i1<=2 || i1>=10)	continue;
		delay_ms(10);
		Read_LaserDis_Usart3(0x00, Temp_Data1);  //读取
		sum1+=Temp_Data1[0];
		if(Temp_Data1[0]>last_time1_max)	last_time1_max=Temp_Data1[0];
		if(Temp_Data1[0]<last_time1_min)	last_time1_min=Temp_Data1[0];
		delay_ms(10);
	}
	distance1 = (sum1-last_time1_max-last_time1_min)/5;
	if(distance1 >= 600)
	{
		//此时没有箱子
		flag1 = 0;
	}
	if(distance1 >= 400 && distance1 <= 440 )
	{
		//此处有一个纸箱，纸箱横放
		flag1 = 1;
	}
	if(distance1 >= 340 && distance1 <= 380)
	{
		//此处有一个纸箱，纸箱纵放
		flag1 = 2;
	}
	
	return flag1;
}

int cal_distance2(int last_flag) //上面模块
{
	unsigned int Temp_Data2[3] = { 0 };       //数据缓存
	unsigned int reading_count2 = 11 ; //记录读数次数
	unsigned int last_time2_max=0 , last_time2_min=999999999;
	unsigned int i2=0;
	unsigned int sum2=0;
	unsigned int distance2=0;
	int flag2=0;
	
	if(last_flag != 0)
	{
		while(i2<=reading_count2)
		{
			i2++;
			if(i2<=2 || i2>=10)	continue;
			delay_ms(10);
			Read_LaserDis_Usart2(0x00, Temp_Data2);  //读取
			sum2+=Temp_Data2[0];
			if(Temp_Data2[0]>last_time2_max)	last_time2_max=Temp_Data2[0];
			if(Temp_Data2[0]<last_time2_min)	last_time2_min=Temp_Data2[0];
			delay_ms(10);
		}
		distance2 = (sum2-last_time2_max-last_time2_min)/5;
		if(distance2 >= 210 && distance2 <= 250)
		{
			//此处有一个纸箱，纸箱横放
			flag2 = 1;
		}
		if(distance2 >= 160 && distance2 <= 200)
		{
			//此处有一个纸箱，纸箱纵放
			flag2 = 2;
		}
	}
	
	return flag2;
	
}

int cal_distance3()
{
	volatile int jug=-1;
	int i3=0;
	int flag3=-1;
	//delay_init(168); 
	//uart_init(115200);
	GPIO_Configuration();
	
	while(i3<=11)
	{
		i3++;
		if(i3 == 9)	jug=GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_8);// 变量返回0表示传感器的灯是亮的  返回1表示传感器灯是灭的
		delay_ms(5);
	}
	if(jug == 0)
	{
		flag3=0;
	}
	if(jug == 1)
	{
		flag3=3;
	}
	return flag3;
	
}
