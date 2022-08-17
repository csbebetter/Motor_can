#include "routeplan.h"

//5个电机的结构体
extern volatile MotorTypeDef Motor_1, Motor_2,Motor_3, Motor_4;
extern volatile MotorTypeDef Motor_5;

float motor_Position[3] = {0, 0, 0}; //coordinate x; coordinate y; Rotation angle(Horizontal counterclockwise is positive)

float motor_circle_angle[5] = {0,0,0,0,0};   //Relative rotation angle of the universal wheel 电机旋转的相对角度
float motor_Position_angle[5] = {0,0,0,0,0}; //Absolute rotation angle of the universal wheel 电机旋转的绝对角度
int i;

/*————————————————————————————————————————————轨迹规划代码↓————————————————————————————————————————————*/
/*————————————————————————————————————————————轨迹规划代码↓————————————————————————————————————————————*/
/*————————————————————————————————————————————轨迹规划代码↓————————————————————————————————————————————*/
/*————————————————————————————————————————————轨迹规划代码↓————————————————————————————————————————————*/
void CoordinatePositionMovement(float CurrentX, float CurrentY, float ExpectedX, float ExpectedY){
	float linear_speed[5];
	float errX = (ExpectedX - CurrentX)*LINEAR_ERR;
	float errY = (ExpectedY - CurrentY)*LINEAR_ERR;
	float flag = 0.0;   //实现小车运动的缓慢加速
	//小车出发时的位置。
	motor_Position_angle[0] = Motor_1.PositionMeasure;
	motor_Position_angle[1] = Motor_2.PositionMeasure;
	motor_Position_angle[2] = Motor_3.PositionMeasure;
	motor_Position_angle[3] = Motor_4.PositionMeasure;
	
	//Straight
	motor_circle_angle[0] =  errX/(PI*WHEEL_DIAMETER)*360;
	motor_circle_angle[1] = -errY/(PI*WHEEL_DIAMETER)*360;
	motor_circle_angle[2] = -motor_circle_angle[0];
	motor_circle_angle[3] = -motor_circle_angle[1];
	
	for(i=0;i<4;i++){
		if(motor_circle_angle[i]<0){
			linear_speed[i] = -LINEAR_SPEED;
		}
		else{
			linear_speed[i] =  LINEAR_SPEED;
		}
	}
	
	do{
		if(flag < LINEAR_SPEED){
			MotorSpeedExpected( sign(linear_speed[0])*flag*fabs(errX)/(fabs(errX)+fabs(errY)),
								sign(linear_speed[1])*flag*fabs(errY)/(fabs(errX)+fabs(errY)),
								sign(linear_speed[2])*flag*fabs(errX)/(fabs(errX)+fabs(errY)),
								sign(linear_speed[3])*flag*fabs(errY)/(fabs(errX)+fabs(errY)),
								0);
			flag += 0.3f ;
		}
		else{
			MotorSpeedExpected( linear_speed[0]*fabs(errX)/(fabs(errX)+fabs(errY)),
								linear_speed[1]*fabs(errY)/(fabs(errX)+fabs(errY)),
								linear_speed[2]*fabs(errX)/(fabs(errX)+fabs(errY)),
								linear_speed[3]*fabs(errY)/(fabs(errX)+fabs(errY)),
								0);
		}
		delay_ms(2);
	}while(!(stopCspdJudging(CSPDERR*4,-1)));
		
  //实现小车缓慢减速
	do{
		MotorSpeedExpected( sign(linear_speed[0])*flag*fabs(errX)/(fabs(errX)+fabs(errY)),
							sign(linear_speed[1])*flag*fabs(errY)/(fabs(errX)+fabs(errY)),
							sign(linear_speed[2])*flag*fabs(errX)/(fabs(errX)+fabs(errY)),
							sign(linear_speed[3])*flag*fabs(errY)/(fabs(errX)+fabs(errY)),
							0);
		if(flag>50)flag -= 0.3f;
		delay_ms(2);
	}while(!(stopCspdJudging(CSPDERR,-1)));

	motor_Position[0] = ExpectedX;
	motor_Position[1] = ExpectedY;
}

void AngularRotationMovement(float CurrentA, float ExpectedA){
	float flag = 0.0;
	float spinning_speed[5];
	float errA = (ExpectedA - CurrentA)*SPINNING_ERR;
	float Motor_rotation_angle = errA*PI/180*MOTOR_DIAMETER/2/(PI*WHEEL_DIAMETER)*360;
	//小车出发时的位置
	motor_Position_angle[0] = Motor_1.PositionMeasure;
	motor_Position_angle[1] = Motor_2.PositionMeasure;
	motor_Position_angle[2] = Motor_3.PositionMeasure;
	motor_Position_angle[3] = Motor_4.PositionMeasure;
	
	for(i=0;i<4;i++){
		motor_circle_angle[i] = - Motor_rotation_angle;
	}

	for(i=0;i<4;i++){
		if(motor_circle_angle[i]<0){
			spinning_speed[i] = -SPINNING_SPEED;
		}
		else{
			spinning_speed[i] =  SPINNING_SPEED;
		}
	}
	//
	do{
		if(flag < LINEAR_SPEED){
			MotorSpeedExpected( sign(spinning_speed[0])*flag,
								sign(spinning_speed[1])*flag,
								sign(spinning_speed[2])*flag,
								sign(spinning_speed[3])*flag,
								0);
			flag += 0.3f ;
		}
		else{
			MotorSpeedExpected(spinning_speed[0],spinning_speed[1],spinning_speed[2],spinning_speed[3],0);
			delay_ms(2);
		}
	}while(!(stopCspdJudging(CSPDERR, 0)));
	motor_Position[2] = ExpectedA;
}


//+ 向下
//- 向上
void Lift_Drop_box(float s){
	int m1 = Motor_1.PositionMeasure;
	int m2 = Motor_2.PositionMeasure;
	int m3 = Motor_3.PositionMeasure;
	int m4 = Motor_4.PositionMeasure;
	int m5 = Motor_5.PositionMeasure;
	while(!(fabs(Motor_5.PositionMeasure - s )<10)){
		MotorPositionExpected(m1,m2,m3,m4,s);
		delay_ms(2);
	}
	
}

void routeplan(int s, int e, int startORend){
	/*
	             (eye)
	(start)     (start)
	0、1、2      2、1、0
	*******
	*******  =>
	*******
	0、1、2      2、1、0
	( end )     ( end )
	（eye）
	
	*/

	if(startORend == 1){//初始位置位于终点,s,e交换，且2 => 0, 0 => 2 
		//swap
		int temp = s;
		s = e;
		e = temp;
		//2=>0, 0=>2 
		if(e!=1){if(e==0){e=2;}else{e=0;}}
		if(s!=1){if(s==0){s=2;}else{s=0;}}
	}
	
	if(s==1&&e==1){
	CoordinatePositionMovement(motor_Position[0], motor_Position[1], CTransX(0.00f, -500.00f), CTransY(0.00f, -500.00f));
	CoordinatePositionMovement(motor_Position[0], motor_Position[1], CTransX(-1250.00f, -500.00f), CTransY(-1250.00f, -500.00f));
	CoordinatePositionMovement(motor_Position[0], motor_Position[1], CTransX(-1250.00f, 500.00f), CTransY(-1250.00f, 500.00f));
	CoordinatePositionMovement(motor_Position[0], motor_Position[1], CTransX(-2250.00f, 500.00f), CTransY(-2250.00f, 500.00f));
	CoordinatePositionMovement(motor_Position[0], motor_Position[1], CTransX(-2250.00f, 00.00f), CTransY(-2250.00f, 00.00f));
	AngularRotationMovement(motor_Position[2], 180.0f);
	}

	if(s==1&&e==0){
	CoordinatePositionMovement(motor_Position[0], motor_Position[1], CTransX(0.00f, -500.00f), CTransY(0.00f, -500.00f));
	CoordinatePositionMovement(motor_Position[0], motor_Position[1], CTransX(-1250.00f, -500.00f), CTransY(-1250.00f, -500.00f));
	CoordinatePositionMovement(motor_Position[0], motor_Position[1], CTransX(-1250.00f, 500.00f), CTransY(-1250.00f, 500.00f));
	CoordinatePositionMovement(motor_Position[0], motor_Position[1], CTransX(-2250.00f, 500.00f), CTransY(-2250.00f, 500.00f));
	AngularRotationMovement(motor_Position[2], 180.0f);
	}

	if(s==1&&e==2){
	CoordinatePositionMovement(motor_Position[0], motor_Position[1], CTransX(0.00f, 500.00f), CTransY(0.00f, 500.00f));
	CoordinatePositionMovement(motor_Position[0], motor_Position[1], CTransX(-1250.00f, 500.00f), CTransY(-1250.00f, 500.00f));
	CoordinatePositionMovement(motor_Position[0], motor_Position[1], CTransX(-1250.00f, -500.00f), CTransY(-1250.00f, -500.00f));
	CoordinatePositionMovement(motor_Position[0], motor_Position[1], CTransX(-2250.00f, -500.00f), CTransY(-2250.00f, -500.00f));
	AngularRotationMovement(motor_Position[2], 180.0f);
	}

	if(s==2&&e==0){
	CoordinatePositionMovement(0.00f, -500.00f, CTransX(-1250.00f, -500.00f), CTransY(-1250.00f, -500.00f));
	CoordinatePositionMovement(motor_Position[0], motor_Position[1], CTransX(-1250.00f, 500.00f), CTransY(-1250.00f, 500.00f));
	CoordinatePositionMovement(motor_Position[0], motor_Position[1], CTransX(-2250.00f, 500.00f), CTransY(-2250.00f, 500.00f));
	AngularRotationMovement(motor_Position[2], 180.0f);
	}

	if(s==2&&e==1){
	CoordinatePositionMovement(0.00f, -500.00f, CTransX(-1250.00f, -500.00f), CTransY(-1250.00f, -500.00f));
	CoordinatePositionMovement(motor_Position[0], motor_Position[1], CTransX(-1250.00f, 500.00f), CTransY(-1250.00f, 500.00f));
	CoordinatePositionMovement(motor_Position[0], motor_Position[1], CTransX(-2250.00f, 500.00f), CTransY(-2250.00f, 500.00f));
	CoordinatePositionMovement(motor_Position[0], motor_Position[1], CTransX(-2250.00f, 00.00f), CTransY(-2250.00f, 00.00f));
	AngularRotationMovement(motor_Position[2], 180.0f);
	}

	if(s==2&&e==2){
	CoordinatePositionMovement(0.00f, -500.00f, CTransX(-1250.00f, -500.00f), CTransY(-1250.00f, -500.00f));
	CoordinatePositionMovement(motor_Position[0], motor_Position[1], CTransX(-1250.00f, 500.00f), CTransY(-1250.00f, 500.00f));
	CoordinatePositionMovement(motor_Position[0], motor_Position[1], CTransX(-2250.00f, 500.00f), CTransY(-2250.00f, 500.00f));
	CoordinatePositionMovement(motor_Position[0], motor_Position[1], CTransX(-2250.00f, -500.00f), CTransY(-2250.00f, -500.00f));
	AngularRotationMovement(motor_Position[2], 180.0f);
	}

	if(s==0&&e==2){
	CoordinatePositionMovement(0.00f, 500.00f, CTransX(-1250.00f, 500.00f), CTransY(-1250.00f, 500.00f));
	CoordinatePositionMovement(motor_Position[0], motor_Position[1], CTransX(-1250.00f, -500.00f), CTransY(-1250.00f, -500.00f));
	CoordinatePositionMovement(motor_Position[0], motor_Position[1], CTransX(-2250.00f, -500.00f), CTransY(-2250.00f, -500.00f));
	AngularRotationMovement(motor_Position[2], 180.0f);
	}

	if(s==0&&e==1){
	CoordinatePositionMovement(0.00f, 500.00f, CTransX(-1250.00f, 500.00f), CTransY(-1250.00f, 500.00f));
	CoordinatePositionMovement(motor_Position[0], motor_Position[1], CTransX(-1250.00f, -500.00f), CTransY(-1250.00f, -500.00f));
	CoordinatePositionMovement(motor_Position[0], motor_Position[1], CTransX(-2250.00f, -500.00f), CTransY(-2250.00f, -500.00f));
	CoordinatePositionMovement(motor_Position[0], motor_Position[1], CTransX(-2250.00f, 00.00f), CTransY(-2250.00f, 00.00f));
	AngularRotationMovement(motor_Position[2], 180.0f);
	}

	if(s==0&&e==0){
	CoordinatePositionMovement(0.00f, 500.00f, CTransX(-1250.00f, 500.00f), CTransY(-1250.00f, 500.00f));
	CoordinatePositionMovement(motor_Position[0], motor_Position[1], CTransX(-1250.00f, -500.00f), CTransY(-1250.00f, -500.00f));
	CoordinatePositionMovement(motor_Position[0], motor_Position[1], CTransX(-2250.00f, -500.00f), CTransY(-2250.00f, -500.00f));
	CoordinatePositionMovement(motor_Position[0], motor_Position[1], CTransX(-2250.00f, 500.00f), CTransY(-2250.00f, 500.00f));
	AngularRotationMovement(motor_Position[2], 180.0f);
	}
}


void liftplan(int upordown, int state){
	
	if(upordown == 1 && state == 1)//up -
	{
		Lift_Drop_box(-4750);
		CoordinatePositionMovement(0, 0, CTransX(360.00f, 0.00f), CTransY(360.00f,0.00f));
		Lift_Drop_box(-5400);
		CoordinatePositionMovement(CTransX(360.00f, 0.00f), CTransY(360.00f,0.00f),0,0);
		Lift_Drop_box(-3000);
	}
	if(upordown == 1 && state == 2)//up |
	{
	
	}
	if(upordown == 0 && state == 1)//down -
	{
		Lift_Drop_box(-360);
		CoordinatePositionMovement(0, 0, CTransX(360.00f, 0.00f), CTransY(360.00f,0.00f));
		Lift_Drop_box(-3000);
		CoordinatePositionMovement(CTransX(360.00f, 0.00f), CTransY(360.00f,0.00f),0,0);

	}
	if(upordown == 0 && state == 2)//down |
	{
	
	}
}
	

	
void dropplan(int e, int state){
	if(e == 1){
		//放置中间位置
		if(state == 1)
		{
			//不需要调整位置
			CoordinatePositionMovement(0, 0, CTransX(360.00f, 0.00f), CTransY(360.00f,0.00f));
			Lift_Drop_box(-360);
			CoordinatePositionMovement(CTransX(360.00f, 0.00f), CTransY(360.00f,0.00f),0,0);
			
		}
		else
		{
			//需要位置调整
		}
		
	}
	else{
		//放置两边的位置
		if(state == 2)
		{
			//不需要调整位置
			CoordinatePositionMovement(0, 0, CTransX(360.00f, 0.00f), CTransY(360.00f,0.00f));
			Lift_Drop_box(-360);
			CoordinatePositionMovement(CTransX(360.00f, 0.00f), CTransY(360.00f,0.00f),0,0);
		}
		else
		{
			//需要位置调整
		}
	}
}




//Judging the speed loop control stop
bool stopCspdJudging(float cspderr, int x){
	int stopj = -3;
	float err1 = Motor_1.PositionMeasure - motor_Position_angle[0] - motor_circle_angle[0];
	float err2 = Motor_2.PositionMeasure - motor_Position_angle[1] - motor_circle_angle[1];
	float err3 = Motor_3.PositionMeasure - motor_Position_angle[2] - motor_circle_angle[2];
	float err4 = Motor_4.PositionMeasure - motor_Position_angle[3] - motor_circle_angle[3];
	if(err1<cspderr && err1>-cspderr){stopj+=1;}
	if(err2<cspderr && err2>-cspderr){stopj+=1;}
	if(err3<cspderr && err3>-cspderr){stopj+=1;}
	if(err4<cspderr && err4>-cspderr){stopj+=1;}
	if(stopj>x){return true;}
	else{return false;}
}

bool stopCposJudging(float cposerr,int x){
	int stopj = -3;
	if(Motor_1.SpeedMeasure<cposerr && Motor_1.SpeedMeasure>-cposerr){stopj+=1;}
	if(Motor_2.SpeedMeasure<cposerr && Motor_2.SpeedMeasure>-cposerr){stopj+=1;}
	if(Motor_3.SpeedMeasure<cposerr && Motor_3.SpeedMeasure>-cposerr){stopj+=1;}
	if(Motor_4.SpeedMeasure<cposerr && Motor_4.SpeedMeasure>-cposerr){stopj+=1;}
	if(stopj>x){return true;}
	else{return false;}
}


float CTransX(float x,float y){
	return (x*cos(theta)+y*sin(theta));
}

float CTransY(float x,float y){
	return (y*cos(theta)-x*sin(theta));
}

int sign(float a){
	if (a == 0) return 0;
	else if (a > 0)  return 1;
	else return -1;
}
