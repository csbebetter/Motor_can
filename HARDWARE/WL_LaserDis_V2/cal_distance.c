#include "cal_distance.h"
//���ߣ�PB10�ſڽ�RXD  11�ڽ�TXD
//      PA3 ��TXD      2��RXD
extern u16 TIM2_Flag;


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
	if(distance1 >= 600)
	{
		//��ʱû������
		flag1 = 0;
	}
	if(distance1 >= 435 && distance1 <= 480)
	{
		//�˴���һ��ֽ�䣬ֽ����
		flag1 = 1;
	}
	if(distance1 >= 375 && distance1 <= 420)
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
		if(distance2 >= 280 && distance2 <= 320)
		{
			//�˴���һ��ֽ�䣬ֽ����
			flag2 = 1;
		}
		if(distance2 >= 235 && distance2 <= 275)
		{
			//�˴���һ��ֽ�䣬ֽ���ݷ�
			flag2 = 2;
		}
	}
	
	return flag2;
	
}

int cal_distance3()
{
	int diatance_Data;
	int sum=0;
	int maxx=0,minn=999999999;
	int k=0;
	int flag3=-1;
	int threshold_value=10;
    u16 q;
    u16 b;
    u16 s;
    u16 g;
 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //�ж����ȼ�����
  
    GPIO_Configuration3();
    TIM2_Configuration(5000,419);  
	
  while (k<11)
  {
	k++;
    diatance_Data = get_Diatance();  
    q = diatance_Data/1000;
    b = diatance_Data/100%10;
    s = diatance_Data/10%10;
    g = diatance_Data%10;
    diatance_Data = q*1000+b*100+s*10+g;
    //if(diatance_Data>maxx)	maxx=diatance_Data;
	if(diatance_Data<minn)	minn=diatance_Data;
	//sum+=diatance_Data;
    delay_us(500);
	}
	
	diatance_Data=minn;
	if(diatance_Data >= threshold_value) flag3=0;//��ʱ������ֵ  ��ʾ�˴�û������
	else flag3=3;
	return flag3;
}
