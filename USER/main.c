#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "motor.h"
#include "can.h"
#include "routeplan.h"
#include "led.h"
#include "key.h"
#include "MyServo.h"
#include "track.h"
#include "cal_distance.h"

//5��ģʽ
#define MOBILE_MODE 1
#define TRACK_MODE 2
#define LIFT_MODE 3
#define DROP_MODE 4
#define STOP_MODE 5

//////////////
//������track����
extern float motor_Position[3];
extern u8 currentRobotState;
extern u8 lastRobotState;
/////////////

	/*         
						(start)    
Sbox_state[3][2]		0��1��2    motor_park_space_start       motor_Location = 0
						*******
						*******     ���� mobile_box_state��ţ������˶�״̬
						*******
Ebox_state[3]			0��1��2    motor_park_space_end         motor_Location = 1
						( end )    
	*/
int Sbox_state[3][2]; //Start ��ʼλ�� 0��1��2��ţ����ڷ�״̬��0��δ��ţ���䣬1����ţ�����ţ�2����ţ�����ݷ�,3������ţ���䣬����֪���ڷŷ�ʽ��-1������ţ���䵫�Ѵ������
int Ebox_state[3]; //End ����λ�� 0��1��2��ţ����ڷ�״̬�� 0�����ã�1���ѷ���ţ������
int mobile_box_state = 0; //�˶�������ţ������˶�״̬��1����ţ�����ţ�2����ţ�����ݷ�
int motor_park_space_start = 1; //С��Ŀ�����λ�õ�3��λ��0��1��2
int motor_park_space_end = 0;   //С��Ŀ���յ�λ�õ�3��λ��0��1��2
int motor_Location = 0; //0��С��λ����ʼ����1��С��λ�ڷ��ô�


//ѡ��ţ�����ӷ���λ��
int ChooseEndPoint(int i){
	int Ebox_state_sum = 0;
	for(i=0;i<3;i++) Ebox_state_sum += Ebox_state[i];
	
	if(Ebox_state_sum == 2) //���ֻʣһ���գ�����ѡ��
	{
		for(i=0;i<3;i++){
			if(Ebox_state[i]==0) return i;
		}
	}
	if(Ebox_state_sum == 1)// �������������λ��
	{
		if(i == 1) //���
		{
			if(Ebox_state[1] == 1){
				return 0;
			}
			else {
				return 1;
			}
		}
		if(i == 2)//�ݷ�
		{
			if(Ebox_state[0] == 1){
				return 2; //0��������ѡ��2��λ
			}
			else {
				return 0; //0��δ����ѡ�����λ
			}
		}
	}
	if(Ebox_state_sum == 0)//��������û�п�ʼ������Ʒ
	{
		if(i == 1) //���
		{
			return 1; //�����м�
		}
		if(i == 2)//�ݷ�
		{
			return 0; //����0��λ��
		}
	}
}

//ѡ��С�����ص����λ��
int ChooseStartPoint(){
	if(Sbox_state[1][0] >0){
		return 1;
	}
	else{
		if(Sbox_state[0][0] >0){
			return 0;
		}
		else{
			if(Sbox_state[2][0] >0){
				return 2;
			}
			else{
				return -1; //�������
			}
		}
	}

}
	
	
	
int main(void){
	//Parametereinstellung
	int Control_Mode;
	LED0 = 1;
	LED1 = 1;
	
	u32 keepCommCnt = 0;
	int j = 0;
	
	//Initialisierung
	uart_init(115200);
	uart2_init(115200);
	uart3_init(115200);
	delay_init(168);
	LED_Init();
	KEY_Init();
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);    
	GPIOCLKInit();
	RedRayInit();
	Reset_motors();
	PidInit();
	CAN1_Configuration();
	stateInit();
	u8 trackModeState = 0;
	
	
	/*�������������������������������������������������������ţ����״̬�� ��������λ����������м�û��ţ���䣬����λ��λ��0������������������������������������������������������������������������*/
	Sbox_state[1][0] = cal_distance1();
	//Sbox_state[0][0] = cal_distance3();
	if(Sbox_state[1][0] == 0){
		Sbox_state[2][0] = 3;
		CoordinatePositionMovement(0.0f, 0.0f, CTransX(-500.00f, 00.00f), CTransY(-500.00f, 00.00f)); //����м�û�У������Ƶ�λ��0��
		motor_park_space_start = 0;//����С����״̬λ����Ϊ����0λ��
	}
	else{ //����С������λ
		if(Sbox_state[0][0] == 0){
			Sbox_state[2][0] = 3;
		}
		else{
			Sbox_state[2][0] = 0;
		}
	}
	
	Control_Mode = LIFT_MODE;

	
	
	/*                   ------------------------------------------------------------------------<--------------------------------<
                     	v                                                                         |                                |
	 LIFT_MODE -> MOBILE_MODE -> TRACK_MODE (if motor_Location == 0) -> LIFT_MODE -> MOBILE_MODE -^                                |
											(if motor_Location == 0) -> DROP_MODE (if motor_park_space_start��=-1) -> MOBILE_MODE -^
																				  (if motor_park_space_start ==-1) -> STOP_MODE
	*/
	
	while(1){
		switch(Control_Mode){
			case MOBILE_MODE:
				/*�켣�滮*/
				for (j=0; j<3; j++) motor_Position[j]=0;
				void routeplan(int motor_park_space_strat, int motor_park_space_end, int motor_Location);
				Control_Mode = TRACK_MODE;
				//ת��С������ʼλ��״̬����
				if(motor_Location == 0){motor_Location = 1;}else{motor_Location = 0;}
				break;
			
			case TRACK_MODE:
				trackModeState = startTrack();
				if(trackModeState){
					stateInit();
					//���С��λ����ʼλ�ã���̧��;���С��λ�ڽ�βλ�ã������²���
					if(motor_Location == 0){Control_Mode=LIFT_MODE;}else{Control_Mode=DROP_MODE;}
					
				}
				break;
				
			case LIFT_MODE:
				Sbox_state[motor_park_space_start][0] = cal_distance1();
				Sbox_state[motor_park_space_start][1] = cal_distance2(Sbox_state[motor_park_space_start][0]);
				if(Sbox_state[motor_park_space_start][0] == 0 || Sbox_state[motor_park_space_start][0] == -1){
					break;
				}
				else if(Sbox_state[motor_park_space_start][0] > 0){
					if(Sbox_state[motor_park_space_start][1] >0){
						if(Sbox_state[motor_park_space_start][1] == 1){
//							Lift_Drop_box(860);
//							CoordinatePositionMovement(0.0f, 0.0f, CTransX(250.00f, 00.00f), CTransY(250.00f, 00.00f));
//							Lift_Drop_box(60);
//							CoordinatePositionMovement(CTransX(250.00f, 00.00f), CTransY(250.00f, 00.00f),0.0f,0.0f);
						}
						if(Sbox_state[motor_park_space_start][1] == 2){
//							Lift_Drop_box(860);
//							CoordinatePositionMovement(0.0f, 0.0f, CTransX(250.00f, 00.00f), CTransY(250.00f, 00.00f));
//							Lift_Drop_box(60);
//							CoordinatePositionMovement(CTransX(250.00f, 00.00f), CTransY(250.00f, 00.00f),0.0f,0.0f);
						}
						
						motor_park_space_end = ChooseEndPoint(Sbox_state[motor_park_space_start][1]);
						Sbox_state[motor_park_space_start][1] = -1;
					}
					else{
						if(Sbox_state[motor_park_space_start][0] == 1){
//							Lift_Drop_box(860);
//							CoordinatePositionMovement(0.0f, 0.0f, CTransX(250.00f, 00.00f), CTransY(250.00f, 00.00f));
//							Lift_Drop_box(60);
//							CoordinatePositionMovement(CTransX(250.00f, 00.00f), CTransY(250.00f, 00.00f),0.0f,0.0f);
						}
						if(Sbox_state[motor_park_space_start][0] == 2){
//							Lift_Drop_box(860);
//							CoordinatePositionMovement(0.0f, 0.0f, CTransX(250.00f, 00.00f), CTransY(250.00f, 00.00f));
//							Lift_Drop_box(60);
//							CoordinatePositionMovement(CTransX(250.00f, 00.00f), CTransY(250.00f, 00.00f),0.0f,0.0f);
						}
						mobile_box_state = Sbox_state[motor_park_space_start][1];
						motor_park_space_end = ChooseEndPoint(Sbox_state[motor_park_space_start][0]);
						Sbox_state[motor_park_space_start][0] = -1;
					}
				}

				Control_Mode = MOBILE_MODE;
				break;
				
			case DROP_MODE:
				if(motor_park_space_end == 1){
					//�����м�λ��
					if(mobile_box_state == 1)
					{
						//����Ҫ����λ��
					}
					else
					{
						//��Ҫλ�õ���
					}
					
				}
				else{
					//�������ߵ�λ��
					if(mobile_box_state == 2)
					{
						//����Ҫ����λ��
					}
					else
					{
						//��Ҫλ�õ���
					}
				}
				motor_park_space_start = ChooseStartPoint();
				if(motor_park_space_start>=0){Control_Mode = MOBILE_MODE;}
				else{Control_Mode = STOP_MODE;}
				break;
				
			case STOP_MODE:
				MotorSpeedExpected(0, 0, 0, 0, 0);
				delay_ms(1);
			
			default:
				break;
		
		}
	}
	
}
