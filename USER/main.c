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

//5种模式
#define MOBILE_MODE 1
#define TRACK_MODE 2
#define LIFT_MODE 3
#define DROP_MODE 4
#define STOP_MODE 5

//////////////
//仅用于track操作
extern float motor_Position[3];
extern u8 currentRobotState;
extern u8 lastRobotState;
/////////////

	/*         
						(start)    
Sbox_state[3][2]		0、1、2    motor_park_space_start       motor_Location = 0
						*******
						*******     —— mobile_box_state：牛奶箱的运动状态
						*******
Ebox_state[3]			0、1、2    motor_park_space_end         motor_Location = 1
						( end )    
	*/
int Sbox_state[3][2]; //Start 起始位置 0，1，2的牛奶箱摆放状态。0：未放牛奶箱，1：放牛奶箱横放，2：放牛奶箱纵放,3：放置牛奶箱，但不知道摆放方式，-1：放有牛奶箱但已处理完成
int Ebox_state[3]; //End 结束位置 0，1，2的牛奶箱摆放状态。 0：闲置；1：已放置牛奶箱子
int mobile_box_state = 0; //运动过程中牛奶箱的运动状态。1：放牛奶箱横放，2：放牛奶箱纵放
int motor_park_space_start = 1; //小车目标起点位置的3个位置0，1，2
int motor_park_space_end = 0;   //小车目标终点位置的3个位置0，1，2
int motor_Location = 0; //0：小车位于起始处，1：小车位于放置处


//选择牛奶箱子放置位置
int ChooseEndPoint(int i){
	int Ebox_state_sum = 0;
	for(i=0;i<3;i++) Ebox_state_sum += Ebox_state[i];
	
	if(Ebox_state_sum == 2) //如果只剩一个空，则被迫选择
	{
		for(i=0;i<3;i++){
			if(Ebox_state[i]==0) return i;
		}
	}
	if(Ebox_state_sum == 1)// 如果有两个空闲位置
	{
		if(i == 1) //横放
		{
			if(Ebox_state[1] == 1){
				return 0;
			}
			else {
				return 1;
			}
		}
		if(i == 2)//纵放
		{
			if(Ebox_state[0] == 1){
				return 2; //0处放置则选择2号位
			}
			else {
				return 0; //0号未放置选择零号位
			}
		}
	}
	if(Ebox_state_sum == 0)//放置区还没有开始放置物品
	{
		if(i == 1) //横放
		{
			return 1; //放置中间
		}
		if(i == 2)//纵放
		{
			return 0; //放置0号位置
		}
	}
}

//选择小车返回的起点位置
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
				return -1; //任务结束
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
	
	
	/*——————————————————————————检测牛奶箱状态； 并进行移位操作，如果中间没有牛奶箱，则移位到位置0————————————————————————————————————*/
	Sbox_state[1][0] = cal_distance1();
	//Sbox_state[0][0] = cal_distance3();
	if(Sbox_state[1][0] == 0){
		Sbox_state[2][0] = 3;
		CoordinatePositionMovement(0.0f, 0.0f, CTransX(-500.00f, 00.00f), CTransY(-500.00f, 00.00f)); //如果中间没有，向左移到位置0。
		motor_park_space_start = 0;//并将小车的状态位设置为处于0位置
	}
	else{ //否则小车不移位
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
											(if motor_Location == 0) -> DROP_MODE (if motor_park_space_start！=-1) -> MOBILE_MODE -^
																				  (if motor_park_space_start ==-1) -> STOP_MODE
	*/
	
	while(1){
		switch(Control_Mode){
			case MOBILE_MODE:
				/*轨迹规划*/
				for (j=0; j<3; j++) motor_Position[j]=0;
				void routeplan(int motor_park_space_strat, int motor_park_space_end, int motor_Location);
				Control_Mode = TRACK_MODE;
				//转换小车的起始位置状态坐标
				if(motor_Location == 0){motor_Location = 1;}else{motor_Location = 0;}
				break;
			
			case TRACK_MODE:
				trackModeState = startTrack();
				if(trackModeState){
					stateInit();
					//如果小车位于起始位置，做抬升;如果小车位于结尾位置，做放下操作
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
					//放置中间位置
					if(mobile_box_state == 1)
					{
						//不需要调整位置
					}
					else
					{
						//需要位置调整
					}
					
				}
				else{
					//放置两边的位置
					if(mobile_box_state == 2)
					{
						//不需要调整位置
					}
					else
					{
						//需要位置调整
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

