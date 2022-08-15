#include "cal_distance.h"
//���ߣ�PB10�ſڽ�RXD  11�ڽ�TXD
//      PA3 ��TXD      2��RXD


int cal_distance1()  //����ģ��
{
	unsigned int Temp_Data1[3] = { 0 };       //���ݻ�����
	unsigned int reading_count1 = 11 ; //��¼��������
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
		Read_LaserDis_Usart3(0x00, Temp_Data1);  //��ȡ
		sum1+=Temp_Data1[0];
		if(Temp_Data1[0]>last_time1_max)	last_time1_max=Temp_Data1[0];
		if(Temp_Data1[0]<last_time1_min)	last_time1_min=Temp_Data1[0];
		delay_ms(10);
	}
	distance1 = (sum1-last_time1_max-last_time1_min)/5;
	if(distance1 >= 3000)
	{
		//��ʱû������
		flag1 = 0;
	}
	if(distance1 >= 125 && distance1 <= 155 )
	{
		//�˴���һ��ֽ�䣬ֽ����
		flag1 = 1;
	}
	if(distance1 >= 65 && distance1 <= 95)
	{
		//�˴���һ��ֽ�䣬ֽ���ݷ�
		flag1 = 2;
	}
	
	return flag1;
}

int cal_distance2(int last_flag) //����ģ��
{
	unsigned int Temp_Data2[3] = { 0 };       //���ݻ���
	unsigned int reading_count2 = 11 ; //��¼��������
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
			Read_LaserDis_Usart2(0x00, Temp_Data2);  //��ȡ
			sum2+=Temp_Data2[0];
			if(Temp_Data2[0]>last_time2_max)	last_time2_max=Temp_Data2[0];
			if(Temp_Data2[0]<last_time2_min)	last_time2_min=Temp_Data2[0];
			delay_ms(10);
		}
		distance2 = (sum2-last_time2_max-last_time2_min)/5;
		if(distance2 >= 125 && distance2 <= 155 )
		{
			//�˴���һ��ֽ�䣬ֽ����
			flag2 = 1;
		}
		if(distance2 >= 65 && distance2 <= 95)
		{
			//�˴���һ��ֽ�䣬ֽ���ݷ�
			flag2 = 2;
		}
	}
	
	return flag2;
	
}
