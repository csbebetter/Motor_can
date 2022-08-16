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
//������trackѭ������
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
int motor_Location = 0; //s or e state    0��С��λ����ʼ����1��С��λ�ڷ��ô�


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
	Servo_Init();
	u8 trackModeState = 0;
	
	

	//CoordinatePositionMovement(0.0f, 0.0f, CTransX(400.00f, 00.00f), CTransY(400.00f, 00.00f));
	//Lift_Drop_box(2160.0f);
	Servo_Angle_Set(180);
	//CoordinatePositionMovement(CTransX(400.00f, 00.00f), CTransY(400.00f, 00.00f),0.0f,0.0f);
	
//	CoordinatePositionMovement(motor_Position[0], motor_Position[1], CTransX(0.00f, -500.00f), CTransY(0.00f, -500.00f));
//	CoordinatePositionMovement(motor_Position[0], motor_Position[1], CTransX(-1000.00f, -500.00f), CTransY(-1000.00f, -500.00f));
//	CoordinatePositionMovement(motor_Position[0], motor_Position[1], CTransX(-1000.00f, 500.00f), CTransY(-1000.00f, 500.00f));
//	CoordinatePositionMovement(motor_Position[0], motor_Position[1], CTransX(-2000.00f, 500.00f), CTransY(-2000.00f, 500.00f));
//	CoordinatePositionMovement(motor_Position[0], motor_Position[1], CTransX(-2000.00f, -100.00f), CTransY(-2000.00f, -100.00f));
//	AngularRotationMovement(motor_Position[2], 180.0f);
	
/*	
	runStop();
	while(1){
		
			if(needForward()){
				lastRobotState = currentRobotState;
				currentRobotState = COMM_FORWARD;
			}
			else if(needLeft()){
				lastRobotState = currentRobotState;
				currentRobotState = COMM_LEFT;
			}
			else if(needRight()){
				lastRobotState = currentRobotState;
				currentRobotState = COMM_RIGHT;
			}
			else if(reachTarget(4) || wayAllWhite()){
				lastRobotState = currentRobotState;
				currentRobotState = COMM_STOP;
			}
			if(currentRobotState == COMM_STOP && wayAllWhite()){
				if(lastRobotState == COMM_LEFT){
					currentRobotState = COMM_LEFT;
					++keepCommCnt;
				}
				if(lastRobotState == COMM_RIGHT){
					currentRobotState = COMM_RIGHT;
					++keepCommCnt;
				}
				if(lastRobotState == COMM_FORWARD){
					if(reachTarget(4)){
						currentRobotState = COMM_STOP;
					}
					else{
						currentRobotState = COMM_FORWARD;
						++keepCommCnt;
					}
				}
			}
			if(keepCommCnt > 1000){
				keepCommCnt =0;
				currentRobotState = COMM_STOP;
			}
			
			switch(currentRobotState){
				case COMM_FORWARD:{
					runFront();
					break;
				}
				case COMM_LEFT:{
					runLeft();
					break;
				}
				case COMM_RIGHT:{
					runRight();
					break;
				}
				case COMM_STOP:{
					runStop();
					break;
				}
				default:{
					delay_ms(10);
					break;
				}
			}
		}

	while(1){
		if(SEARCH_OUT_IO_3 == WHITE_AREA){
			LED0 = 0;
		}
		else{
			LED0 = 1;
		}
		if(SEARCH_OUT_IO_4 == WHITE_AREA){
			LED1 = 0;
		}
		else{
			LED1 = 1;
		}
	}
	
	while(1){
		MotorSpeedExpected(0, 0, 0, 0, 0);
		delay_ms(1);
	
	}
	
	*/
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
//	
//	/*����������������������������������������������������V ���ţ����״̬�� ��������λ���������λ��1û��ţ���䣬����λ��λ��0 V������������������������������������������������������������������������*/
//	Sbox_state[1][0] = cal_distance1();
//	Sbox_state[0][0] = cal_distance3();
//	if(Sbox_state[1][0] == 0){
//		Sbox_state[2][0] = 3;
//		CoordinatePositionMovement(0.0f, 0.0f, CTransX(-500.00f, 00.00f), CTransY(-500.00f, 00.00f)); //����м�û�У������Ƶ�λ��0��
//		motor_park_space_start = 0;//����С����״̬λ����Ϊ����0λ��
//	}
//	else{ //����С������λ
//		if(Sbox_state[0][0] == 0){Sbox_state[2][0] = 3;} else{Sbox_state[2][0] = 0;}
//	}
//	/*����������������������������������������������������^ ���ţ����״̬�� ��������λ���������λ��1û��ţ���䣬����λ��λ��0 ^������������������������������������������������������������������������*/
//	Control_Mode = LIFT_MODE;

//	
//	
//	/*Program flow ideas��
//                       	 ------------------------------------------------------------------------<--------------------------------<
//                     	v                                                                         |                                |
//	 LIFT_MODE -> MOBILE_MODE -> TRACK_MODE (if motor_Location == 0) -> LIFT_MODE -> MOBILE_MODE -^                                |
//											(if motor_Location == 0) -> DROP_MODE (if motor_park_space_start��=-1) -> MOBILE_MODE -^
//																				  (if motor_park_space_start ==-1) -> STOP_MODE
//	*/
//	
//	while(1){
//		switch(Control_Mode){
//			case MOBILE_MODE:
//				/*�켣�滮*/
//				for (j=0; j<3; j++) motor_Position[j]=0;
//				void routeplan(int motor_park_space_strat, int motor_park_space_end, int motor_Location);
//				//ת��С����s or e״̬
//				if(motor_Location == 0){motor_Location = 1;}else{motor_Location = 0;}
//				//-> TRACK_MODE
//				Control_Mode = TRACK_MODE;
//				break;
//			
//			case TRACK_MODE:
//				trackModeState = startTrack();
//				if(trackModeState){
//					stateInit();
//					//���С��λ����ʼλ�ã���̧��;���С��λ�ڽ�βλ�ã������²���
//					if(motor_Location == 0){Control_Mode=LIFT_MODE;}else{Control_Mode=DROP_MODE;}
//				}
//				break;
//				
//			case LIFT_MODE:
//				Sbox_state[motor_park_space_start][0] = cal_distance1();
//				Sbox_state[motor_park_space_start][1] = cal_distance2(Sbox_state[motor_park_space_start][0]);
//				if(Sbox_state[motor_park_space_start][0] == 0 || Sbox_state[motor_park_space_start][0] == -1) //With path planning, this situation does not occur
//				{
//					Control_Mode = STOP_MODE;
//					printf("\r\n ERROR: You plan a bad route, here is noting! Rewrite your code!  \r\n");
//					break;
//				}
//				else if(Sbox_state[motor_park_space_start][0] > 0){
//					if(Sbox_state[motor_park_space_start][1] >0){
//						liftplan(1,Sbox_state[motor_park_space_start][1]);
//						mobile_box_state = Sbox_state[motor_park_space_start][1]; //milk box state
//						motor_park_space_end = ChooseEndPoint(Sbox_state[motor_park_space_start][1]);
//						Sbox_state[motor_park_space_start][1] = -1; //update state
//					}
//					else{
//						liftplan(0,Sbox_state[motor_park_space_start][1]);
//						mobile_box_state = Sbox_state[motor_park_space_start][0]; //milk box state
//						motor_park_space_end = ChooseEndPoint(Sbox_state[motor_park_space_start][0]);
//						Sbox_state[motor_park_space_start][0] = -1; //update state
//					}
//				}

//				Control_Mode = MOBILE_MODE;
//				break;
//				
//			case DROP_MODE:
//				dropplan(motor_park_space_end,mobile_box_state);
//				motor_park_space_start = ChooseStartPoint();
//				if(motor_park_space_start>=0){Control_Mode = MOBILE_MODE;} // mission in progress
//				else{Control_Mode = STOP_MODE;} //mission over
//				break;
//				
//			case STOP_MODE:
//				// just stop!
//				MotorSpeedExpected(0, 0, 0, 0, 0);
//				delay_ms(1);
//			
//			default:
//				Control_Mode = STOP_MODE;
//				break;
//		
//		}
//	}
}

