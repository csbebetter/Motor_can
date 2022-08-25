#include "cal_distance.h"
#include "motor.h"
//接线：PB10号口接RXD  11口接TXD
//      PA3 接TXD      2接RXD
extern int i;

int cal_distance1()  //下面模块
{
	unsigned int Temp_Data1[3] = { 0 };       //数据缓存区
	unsigned int reading_count1 = 6 ; //记录读数次数
	unsigned int last_time1_max=0 , last_time1_min=999999999;
	unsigned int i1=0;
	unsigned int sum1=0;
	unsigned int distance1=0;
	int flag1=0;
	int check1 = 0;
	
	while(check1 == 0)
	{
		i1=0;
		sum1=0;
		last_time1_max=0;
		last_time1_min=999999999;
		distance1=0;
		flag1=0;
		
	for(i=0;i<20;i++){
		MotorSpeedExpected(0, 0, 0, 0, 0);
		delay_ms(2);
	}
	
	while(i1<=reading_count1)
	{
		i1++;
		if(i1<=2)	continue;
		delay_us(500);
		MotorSpeedExpected(0, 0, 0, 0, 0);
		Read_LaserDis_Usart3(0x00, Temp_Data1);  //读取
		sum1+=Temp_Data1[0];
		if(Temp_Data1[0]>last_time1_max)	last_time1_max=Temp_Data1[0];
		if(Temp_Data1[0]<last_time1_min)	last_time1_min=Temp_Data1[0];
		MotorSpeedExpected(0, 0, 0, 0, 0);
//		delay_ms(1);
	}
	distance1 = (sum1-last_time1_max-last_time1_min)/3;
	if(distance1 >= 650)
	{
		//此时没有箱子
		flag1 = 0;
		check1 = 1;
	}
	else if(distance1 >= 431 && distance1 <= 500)
	{
		//此处有一个纸箱，纸箱横放
		flag1 = 1;
		check1 = 1;
	}
	else if(distance1 >= 360 && distance1 <= 430)
	{
		//此处有一个纸箱，纸箱纵放
		flag1 = 2;
		check1 = 1;
	}
	else check1 = 0;
	
	}
	return flag1;
}

int cal_distance2(int last_flag) //上面模块
{
	unsigned int Temp_Data2[3] = { 0 };       //数据缓存
	unsigned int reading_count2 = 6 ; //记录读数次数
	unsigned int last_time2_max=0 , last_time2_min=999999999;
	unsigned int i2=0;
	unsigned int sum2=0;
	unsigned int distance2=0;
	int flag2=0;
	int check2 = 0;
	
	while(check2 == 0)
	{
		i2=0;
		sum2=0;
		last_time2_max=0;
		last_time2_min=999999999;
		distance2=0;
		flag2=0;
		
	for(i=0;i<20;i++){
		MotorSpeedExpected(0, 0, 0, 0, 0);
		delay_ms(2);
	}
	
	if(last_flag != 0)
	{
		while(i2<=reading_count2)
		{
			i2++;
 			if(i2<=2)	continue;
			delay_us(500);
			MotorSpeedExpected(0, 0, 0, 0, 0);
			Read_LaserDis_Usart2(0x00, Temp_Data2);  //读取
			sum2+=Temp_Data2[0];
			if(Temp_Data2[0]>last_time2_max)	last_time2_max=Temp_Data2[0];
			if(Temp_Data2[0]<last_time2_min)	last_time2_min=Temp_Data2[0];
			MotorSpeedExpected(0, 0, 0, 0, 0);
//			delay_ms(1);
		}
		distance2 = (sum2-last_time2_max-last_time2_min)/3;;
		if(distance2 >= 302 && distance2 <= 380)
		{
			//此处有一个纸箱，纸箱横放
			flag2 = 1;
			check2 = 1;
		}
		else if(distance2 >= 235 && distance2 <= 301)
		{
			//此处有一个纸箱，纸箱纵放
			flag2 = 2;
			check2 = 1;
		}
		else if(distance2 >= 650)
		{
			flag2 = 0;
			check2 = 1;
		}
		else check2 = 0;
	}
	else check2 = 1;
	}
	
	return flag2;
	
}



int cal_distance3() //2à???￡?é
{
	unsigned int Temp_Data3[3] = { 0 };       //êy?Y?o′?
	unsigned int reading_count3 = 4 ; //?????áêy′?êy
	unsigned int last_time3_max=0 , last_time3_min=999999999;
	unsigned int i3=0;
	unsigned int sum3=0;
	unsigned int distance3=0;
	int flag3=0;
	int checkdis = 0;
	
	//red
	GPIO_InitTypeDef  GPIO_InitStructure1;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);//ê1?üGPIOFê±?ó

  //GPIOF9,F103?ê??ˉéè??
  GPIO_InitStructure1.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;//LED0oíLED1??ó|IO?ú
  GPIO_InitStructure1.GPIO_Mode = GPIO_Mode_OUT;//??í¨ê?3??￡ê?
  GPIO_InitStructure1.GPIO_OType = GPIO_OType_PP;//í?íìê?3?
  GPIO_InitStructure1.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure1.GPIO_PuPd = GPIO_PuPd_UP;//é?à-
  GPIO_Init(GPIOD, &GPIO_InitStructure1);//3?ê??ˉGPIO
	
	GPIO_ResetBits(GPIOD,GPIO_Pin_8 | GPIO_Pin_9);//GPIOF9,F10éè????￡?μ??e
	
	//green
	GPIO_InitTypeDef  GPIO_InitStructure2;

  //GPIOF9,F103?ê??ˉéè??
  GPIO_InitStructure2.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;//LED0oíLED1??ó|IO?ú
  GPIO_InitStructure2.GPIO_Mode = GPIO_Mode_OUT;//??í¨ê?3??￡ê?
  GPIO_InitStructure2.GPIO_OType = GPIO_OType_PP;//í?íìê?3?
  GPIO_InitStructure2.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure2.GPIO_PuPd = GPIO_PuPd_UP;//é?à-
  GPIO_Init(GPIOD, &GPIO_InitStructure2);//3?ê??ˉGPIO
	
	GPIO_ResetBits(GPIOD,GPIO_Pin_10 | GPIO_Pin_11);//GPIOF9,F10éè????￡?μ??e
	GPIO_SetBits(GPIOD,GPIO_Pin_8);
	GPIO_SetBits(GPIOD,GPIO_Pin_10);
	
	while(checkdis == 0)
	{
	last_time3_max=0;
	last_time3_min=999999999;
	i3=0;
	sum3=0;
	distance3=0;
		while(i3<=reading_count3)
		{
			i3++;
			if(i3<=2)	continue;
			delay_us(500);
			Read_LaserDis_Usart6(0x00, Temp_Data3);  //?áè?
			sum3+=Temp_Data3[0];
			if(Temp_Data3[0]>last_time3_max)	last_time3_max=Temp_Data3[0];
			if(Temp_Data3[0]<last_time3_min)	last_time3_min=Temp_Data3[0];
			delay_ms(1);
		}
		distance3 = sum3-last_time3_min-last_time3_max;
		if(distance3 >= 250 && distance3 <=900 )
		{
			//óD????
			flag3 = 3;
			GPIO_SetBits(GPIOD,GPIO_Pin_8);//GPIOF9,F10éè????￡?μ??e
			GPIO_ResetBits(GPIOD,GPIO_Pin_10);
			checkdis = 1;
			
		}
		else if(distance3>=10)
		{
			flag3 =0;
			GPIO_SetBits(GPIOD,GPIO_Pin_10);//GPIOF9,F10éè????￡?μ??e
			GPIO_ResetBits(GPIOD,GPIO_Pin_8);
			checkdis = 1;
		}
		else 
		{
			checkdis = 0;
			GPIO_ResetBits(GPIOD,GPIO_Pin_8);
			GPIO_ResetBits(GPIOD,GPIO_Pin_10);
		}
	}
	return flag3;
	
}
