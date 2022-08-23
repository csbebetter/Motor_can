#include "cal_distance.h"
#include "motor.h"
//���ߣ�PB10�ſڽ�RXD  11�ڽ�TXD
//      PA3 ��TXD      2��RXD
extern int i;

int cal_distance1()  //����ģ��
{
	unsigned int Temp_Data1[3] = { 0 };       //���ݻ�����
	unsigned int reading_count1 = 6 ; //��¼��������
	unsigned int last_time1_max=0 , last_time1_min=999999999;
	unsigned int i1=0;
	unsigned int sum1=0;
	unsigned int distance1=0;
	int flag1=0;
	 
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
		Read_LaserDis_Usart3(0x00, Temp_Data1);  //��ȡ
		sum1+=Temp_Data1[0];
		if(Temp_Data1[0]>last_time1_max)	last_time1_max=Temp_Data1[0];
		if(Temp_Data1[0]<last_time1_min)	last_time1_min=Temp_Data1[0];
		MotorSpeedExpected(0, 0, 0, 0, 0);
//		delay_ms(1);
	}
	distance1 = (sum1-last_time1_max-last_time1_min)/3;
	if(distance1 >= 650)
	{
		//��ʱû������
		flag1 = 0;
	}
	if(distance1 >= 436 && distance1 <= 500)
	{
		//�˴���һ��ֽ�䣬ֽ����
		flag1 = 1;
	}
	if(distance1 >= 360 && distance1 <= 435)
	{
		//�˴���һ��ֽ�䣬ֽ���ݷ�
		flag1 = 2;
	}
	
	return flag1;
}

int cal_distance2(int last_flag) //����ģ��
{
	unsigned int Temp_Data2[3] = { 0 };       //���ݻ���
	unsigned int reading_count2 = 6 ; //��¼��������
	unsigned int last_time2_max=0 , last_time2_min=999999999;
	unsigned int i2=0;
	unsigned int sum2=0;
	unsigned int distance2=0;
	int flag2=0;
	
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
			Read_LaserDis_Usart2(0x00, Temp_Data2);  //��ȡ
			sum2+=Temp_Data2[0];
			if(Temp_Data2[0]>last_time2_max)	last_time2_max=Temp_Data2[0];
			if(Temp_Data2[0]<last_time2_min)	last_time2_min=Temp_Data2[0];
			MotorSpeedExpected(0, 0, 0, 0, 0);
//			delay_ms(1);
		}
		distance2 = (sum2-last_time2_max-last_time2_min)/3;;
		if(distance2 >= 303 && distance2 <= 380)
		{
			//�˴���һ��ֽ�䣬ֽ����
			flag2 = 1;
		}
		if(distance2 >= 225 && distance2 <= 302)
		{
			//�˴���һ��ֽ�䣬ֽ���ݷ�
			flag2 = 2;
		}
	}
	
	return flag2;
	
}

int cal_distance3() //����ģ��
{
	unsigned int Temp_Data3[3] = { 0 };       //���ݻ���
	unsigned int reading_count3 = 4 ; //��¼��������
	unsigned int last_time3_max=0 , last_time3_min=999999999;
	unsigned int i3=0;
	unsigned int sum3=0;
	unsigned int distance3=0;
	int flag3=0;
	
		while(i3<=reading_count3)
		{
			i3++;
			if(i3<=2)	continue;
			delay_us(500);
			Read_LaserDis_Usart6(0x00, Temp_Data3);  //��ȡ
			sum3+=Temp_Data3[0];
			if(Temp_Data3[0]>last_time3_max)	last_time3_max=Temp_Data3[0];
			if(Temp_Data3[0]<last_time3_min)	last_time3_min=Temp_Data3[0];
			delay_ms(1);
		}
		distance3 = sum3-last_time3_min-last_time3_max;
		if(distance3 >= 250 && distance3 <=900 )
		{
			//��ֽ��
			flag3 = 3;
		}
		else flag3 =0;
	
	return flag3;
	
}
