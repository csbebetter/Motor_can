#include "routeplan.h"

//5倖窮字議潤更悶
extern volatile MotorTypeDef Motor_1, Motor_2,Motor_3, Motor_4;
extern volatile MotorTypeDef Motor_5;

float motor_Position[3] = {0, 0, 0}; //coordinate x; coordinate y; Rotation angle(Horizontal counterclockwise is positive)

float motor_circle_angle[5] = {0,0,0,0,0};   //Relative rotation angle of the universal wheel 窮字傴廬議�犇埆廼�
float motor_Position_angle[5] = {0,0,0,0,0}; //Absolute rotation angle of the universal wheel 窮字傴廬議蒸斤叔業
int i;

/*！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！轟治号皿旗鷹◎！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！*/
/*！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！轟治号皿旗鷹◎！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！*/
/*！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！轟治号皿旗鷹◎！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！*/
/*！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！轟治号皿旗鷹◎！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！*/
void CoordinatePositionMovement(float CurrentX, float CurrentY, float ExpectedX, float ExpectedY){
	float linear_speed[5];
	float errX = (ExpectedX - CurrentX)*LINEAR_ERR;
	float errY = (ExpectedY - CurrentY)*LINEAR_ERR;
	float flag = 0.0;   //糞�嶄ヽ虐剖�議産蛸紗堀
	//弌概竃窟扮議了崔。
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
			flag += 0.1f ;
		}
		else{
			MotorSpeedExpected( linear_speed[0]*fabs(errX)/(fabs(errX)+fabs(errY)),
								linear_speed[1]*fabs(errY)/(fabs(errX)+fabs(errY)),
								linear_speed[2]*fabs(errX)/(fabs(errX)+fabs(errY)),
								linear_speed[3]*fabs(errY)/(fabs(errX)+fabs(errY)),
								0);
		}
		delay_ms(2);
	}while(!(stopCspdJudging(CSPDERR*10,-1)));
	
	if (flag<50){
		flag = 50;
	}
  //糞�嶄ヽ技座�受堀
	do{
		MotorSpeedExpected( sign(linear_speed[0])*flag*fabs(errX)/(fabs(errX)+fabs(errY)),
							sign(linear_speed[1])*flag*fabs(errY)/(fabs(errX)+fabs(errY)),
							sign(linear_speed[2])*flag*fabs(errX)/(fabs(errX)+fabs(errY)),
							sign(linear_speed[3])*flag*fabs(errY)/(fabs(errX)+fabs(errY)),
							0);
		if(flag>50)flag -= 0.5f;
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
	//弌概竃窟扮議了崔
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
			flag += 0.2f ;
		}
		else{
			MotorSpeedExpected(spinning_speed[0],spinning_speed[1],spinning_speed[2],spinning_speed[3],0);
			delay_ms(2);
		}
	}while(!(stopCspdJudging(CSPDERR, 0)));
	motor_Position[2] = ExpectedA;
}


//+ �鯱�
//- �鯢�
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
	��eye��
	
	*/
	
	//肇
	if(startORend == 0){
		
		if(s==1&&e==0){
			CoordinatePositionMovement(motor_Position[0], motor_Position[1], CTransX(0.00f, -550.00f), CTransY(0.00f, -550.00f));
			CoordinatePositionMovement(motor_Position[0], motor_Position[1], CTransX(-1200.00f, -550.00f), CTransY(-1200.00f, -550.00f));
			CoordinatePositionMovement(motor_Position[0], motor_Position[1], CTransX(-1200.00f, 600.00f), CTransY(-1200.00f, 600.00f));
			CoordinatePositionMovement(motor_Position[0], motor_Position[1], CTransX(-1900.00f, 600.00f), CTransY(-1900.00f, 600.00f));
			AngularRotationMovement(motor_Position[2], 180.0f);
		}
		
		if(s==1&&e==1){
			CoordinatePositionMovement(motor_Position[0], motor_Position[1], CTransX(0.00f, -550.00f), CTransY(0.00f, -550.00f));
			CoordinatePositionMovement(motor_Position[0], motor_Position[1], CTransX(-1200.00f, -550.00f), CTransY(-1200.00f, -550.00f));
			CoordinatePositionMovement(motor_Position[0], motor_Position[1], CTransX(-1200.00f, 600.00f), CTransY(-1200.00f, 600.00f));
			CoordinatePositionMovement(motor_Position[0], motor_Position[1], CTransX(-1900.00f, 600.00f), CTransY(-1900.00f, 600.00f));
			AngularRotationMovement(motor_Position[2], 180.0f);
			CoordinatePositionMovement(0, 0, CTransX(00.00f, 523.00f), CTransY(00.00f, 523.00f));
		}


		if(s==1&&e==2){
			CoordinatePositionMovement(motor_Position[0], motor_Position[1], CTransX(0.00f, -550.00f), CTransY(0.00f, -550.00f));
			CoordinatePositionMovement(motor_Position[0], motor_Position[1], CTransX(-1200.00f, -550.00f), CTransY(-1200.00f, -550.00f));
			CoordinatePositionMovement(motor_Position[0], motor_Position[1], CTransX(-1200.00f, 600.00f), CTransY(-1200.00f, 600.00f));
			CoordinatePositionMovement(motor_Position[0], motor_Position[1], CTransX(-1900.00f, 600.00f), CTransY(-1900.00f, 600.00f));
			AngularRotationMovement(motor_Position[2], 180.0f);
			CoordinatePositionMovement(0, 0, CTransX(00.00f, 1100.00f), CTransY(00.00f, 1100.00f));
		}

		if(s==2&&e==0){
			CoordinatePositionMovement(CTransX(0.00f, -550.00f), CTransY(0.00f, -550.00f), CTransX(-1200.00f, -550.00f), CTransY(-1200.00f, -550.00f));
			CoordinatePositionMovement(motor_Position[0], motor_Position[1], CTransX(-1200.00f, 600.00f), CTransY(-1200.00f, 600.00f));
			CoordinatePositionMovement(motor_Position[0], motor_Position[1], CTransX(-1900.00f, 600.00f), CTransY(-1900.00f, 600.00f));
			AngularRotationMovement(motor_Position[2], 180.0f);
		}

		if(s==2&&e==1){
			CoordinatePositionMovement(CTransX(0.00f, -550.00f), CTransY(0.00f, -550.00f), CTransX(-1200.00f, -550.00f), CTransY(-1200.00f, -550.00f));
			CoordinatePositionMovement(motor_Position[0], motor_Position[1], CTransX(-1200.00f, 600.00f), CTransY(-1200.00f, 600.00f));
			CoordinatePositionMovement(motor_Position[0], motor_Position[1], CTransX(-1900.00f, 600.00f), CTransY(-1900.00f, 600.00f));
			AngularRotationMovement(motor_Position[2], 180.0f);
			CoordinatePositionMovement(0, 0, CTransX(00.00f, 523.00f), CTransY(00.00f, 523.00f));
		}

		if(s==2&&e==2){
			CoordinatePositionMovement(CTransX(0.00f, -550.00f), CTransY(0.00f, -550.00f), CTransX(-1200.00f, -550.00f), CTransY(-1200.00f, -550.00f));
			CoordinatePositionMovement(motor_Position[0], motor_Position[1], CTransX(-1200.00f, 600.00f), CTransY(-1200.00f, 600.00f));
			CoordinatePositionMovement(motor_Position[0], motor_Position[1], CTransX(-1900.00f, 600.00f), CTransY(-1900.00f, 600.00f));
			AngularRotationMovement(motor_Position[2], 180.0f);
			CoordinatePositionMovement(0, 0, CTransX(00.00f, 1100.00f), CTransY(00.00f, 1100.00f));
		}

		if(s==0&&e==0){
			CoordinatePositionMovement(CTransX(0.00f, 550.00f), CTransY(0.00f, 550.00f), CTransX(0.00f, -550.00f), CTransY(0.00f, -550.00f));
			CoordinatePositionMovement(CTransX(0.00f, -550.00f), CTransY(0.00f, -550.00f), CTransX(-1200.00f, -550.00f), CTransY(-1200.00f, -550.00f));
			CoordinatePositionMovement(motor_Position[0], motor_Position[1], CTransX(-1200.00f, 600.00f), CTransY(-1200.00f, 600.00f));
			CoordinatePositionMovement(motor_Position[0], motor_Position[1], CTransX(-1900.00f, 600.00f), CTransY(-1900.00f, 600.00f));
			AngularRotationMovement(motor_Position[2], 180.0f);
		}
		if(s==0&&e==1){
			CoordinatePositionMovement(CTransX(0.00f, 550.00f), CTransY(0.00f, 550.00f), CTransX(0.00f, -550.00f), CTransY(0.00f, -550.00f));
			CoordinatePositionMovement(CTransX(0.00f, -550.00f), CTransY(0.00f, -550.00f), CTransX(-1200.00f, -550.00f), CTransY(-1200.00f, -550.00f));
			CoordinatePositionMovement(motor_Position[0], motor_Position[1], CTransX(-1200.00f, 600.00f), CTransY(-1200.00f, 600.00f));
			CoordinatePositionMovement(motor_Position[0], motor_Position[1], CTransX(-1900.00f, 600.00f), CTransY(-1900.00f, 600.00f));
			AngularRotationMovement(motor_Position[2], 180.0f);
			CoordinatePositionMovement(0, 0, CTransX(00.00f, 523.00f), CTransY(00.00f, 523.00f));
		}
		if(s==0&&e==2){
			CoordinatePositionMovement(CTransX(0.00f, 550.00f), CTransY(0.00f, 550.00f), CTransX(0.00f, -550.00f), CTransY(0.00f, -550.00f));
			CoordinatePositionMovement(CTransX(0.00f, -550.00f), CTransY(0.00f, -550.00f), CTransX(-1200.00f, -550.00f), CTransY(-1200.00f, -550.00f));
			CoordinatePositionMovement(motor_Position[0], motor_Position[1], CTransX(-1200.00f, 600.00f), CTransY(-1200.00f, 600.00f));
			CoordinatePositionMovement(motor_Position[0], motor_Position[1], CTransX(-1900.00f, 600.00f), CTransY(-1900.00f, 600.00f));
			AngularRotationMovement(motor_Position[2], 180.0f);
			CoordinatePositionMovement(0, 0, CTransX(00.00f, 1100.00f), CTransY(00.00f, 1100.00f));
		}
	}
	//指
	if(startORend == 1){
		
		if(s==1&&e==0){
			CoordinatePositionMovement(motor_Position[0], motor_Position[1], CTransX(-1800.00f, 0.00f), CTransY(-1800.00f, 00.00f));
			AngularRotationMovement(motor_Position[2], -180.0f);
			CoordinatePositionMovement(0, 0, CTransX(00.00f, -510.00f), CTransY(00.00f, -510.00f));
		}
		
		if(s==1&&e==1){
			CoordinatePositionMovement(motor_Position[0], motor_Position[1], CTransX(0.00f, -519.00f), CTransY(0.00f, -519.00f));
			CoordinatePositionMovement(motor_Position[0], motor_Position[1], CTransX(-1800.00f, -519.00f), CTransY(-1800.00f, -519.00f));
			AngularRotationMovement(motor_Position[2], -180.0f);
			CoordinatePositionMovement(0, 0, CTransX(00.00f, -510.00f), CTransY(00.00f, -510.00f));
		}

		if(s==1&&e==2){
			CoordinatePositionMovement(motor_Position[0], motor_Position[1], CTransX(0.00f, -1000.00f), CTransY(0.00f, -1000.00f));
			CoordinatePositionMovement(motor_Position[0], motor_Position[1], CTransX(-1800.00f, -1000.00f), CTransY(-1800.00f, -1000.00f));
			AngularRotationMovement(motor_Position[2], -180.0f);
			CoordinatePositionMovement(0, 0, CTransX(00.00f, -510.00f), CTransY(00.00f, -510.00f));
		}

		if(s==2&&e==0){
			CoordinatePositionMovement(motor_Position[0], motor_Position[1], CTransX(0.00f, 1100.00f), CTransY(0.00f, 1100.00f));
			CoordinatePositionMovement(motor_Position[0], motor_Position[1], CTransX(-1800.00f, 1100.00f), CTransY(-1800.00f, 1100.00f));
			AngularRotationMovement(motor_Position[2], 180.0f);
		}

		if(s==2&&e==1){
			CoordinatePositionMovement(motor_Position[0], motor_Position[1], CTransX(0.00f, 577.00f), CTransY(0.00f, 577.00f));
			CoordinatePositionMovement(motor_Position[0], motor_Position[1], CTransX(-1800.00f, 577.00f), CTransY(-1800.00f, 577.00f));
			AngularRotationMovement(motor_Position[2], 180.0f);
		}

		if(s==2&&e==2){
			CoordinatePositionMovement(motor_Position[0], motor_Position[1], CTransX(-1800.00f, 0.00f), CTransY(-1800.00f, 0.00f));
			AngularRotationMovement(motor_Position[2], 180.0f);
		}

		if(s==0&&e==0){
			CoordinatePositionMovement(motor_Position[0], motor_Position[1], CTransX(-1800.00f, 0.00f), CTransY(-1800.00f, 0.00f));
			AngularRotationMovement(motor_Position[2], -180.0f);
			
		}

		if(s==0&&e==1){
			CoordinatePositionMovement(motor_Position[0], motor_Position[1], CTransX(0.00f, -519.00f), CTransY(0.00f, -519.00f));
			CoordinatePositionMovement(motor_Position[0], motor_Position[1], CTransX(-1800.00f, -519.00f), CTransY(-1800.00f, -519.00f));
			AngularRotationMovement(motor_Position[2], -180.0f);
		}

		if(s==0&&e==2){
			CoordinatePositionMovement(motor_Position[0], motor_Position[1], CTransX(0.00f, -1000.00f), CTransY(0.00f, -1000.00f));
			CoordinatePositionMovement(motor_Position[0], motor_Position[1], CTransX(-1800.00f, -1100.00f), CTransY(-1800.00f, -1100.00f));
			AngularRotationMovement(motor_Position[2], -180.0f);
		}
	}
}


void liftplan(int upordown, int state){
	
	if(upordown == 1 && state == 1)//up -
	{
		Lift_Drop_box(-4900);
		CoordinatePositionMovement(0, 0, CTransX(360.00f, 0.00f), CTransY(360.00f,0.00f));
		Lift_Drop_box(-5800);
		CoordinatePositionMovement(CTransX(370.00f, 0.00f), CTransY(370.00f,0.00f),0,0);
		Lift_Drop_box(-3000);
	}
	if(upordown == 1 && state == 2)//up |
	{
		CoordinatePositionMovement(0, 0, CTransX(0.00f, 250.00f), CTransY(0.00f,250.00f));
		Clockwise;
		Lift_Drop_box(-4900);
		CoordinatePositionMovement(CTransX(0.00f, 250.00f), CTransY(0.00f,250.00f),CTransX(380.00f, 250.00f), CTransY(380.00f,250.00f));
		CoordinatePositionMovement(CTransX(380.00f, 250.00f), CTransY(380.00f,250.00f), CTransX(380.00f, 30.00f), CTransY(380.00f,30.00f));
		Lift_Drop_box(-5800);
		CoordinatePositionMovement(CTransX(380.00f, 30.00f), CTransY(380.00f,30.00f),CTransX(0.00f, 30.00f), CTransY(0.00f,30.00f));
		Lift_Drop_box(-3000);
		//CoordinatePositionMovement(CTransX(0.00f, 30.00f), CTransY(0.00f,30.00f),CTransX(0.00f, -30.00f), CTransY(0.00f,-30.00f));

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
		CoordinatePositionMovement(0, 0, CTransX(0.00f, 250.00f), CTransY(0.00f,250.00f));
		Clockwise;
		Lift_Drop_box(-360);
		CoordinatePositionMovement(CTransX(0.00f, 250.00f), CTransY(0.00f,250.00f),CTransX(380.00f, 250.00f), CTransY(380.00f,250.00f));
		CoordinatePositionMovement(CTransX(380.00f, 250.00f), CTransY(380.00f,250.00f), CTransX(380.00f, 30.00f), CTransY(380.00f,30.00f));
		Lift_Drop_box(-3000);
		CoordinatePositionMovement(CTransX(380.00f, 30.00f), CTransY(380.00f,30.00f),CTransX(0.00f, 30.00f), CTransY(0.00f,30.00f));
		//CoordinatePositionMovement(CTransX(0.00f, 30.00f), CTransY(0.00f,30.00f),CTransX(0.00f, -30.00f), CTransY(0.00f,-30.00f));
	}
}
	

	
void dropplan(int e, int state){
	if(e == 1){
		//慧崔嶄寂了崔
		if(state == 1)
		{
			Lift_Drop_box(-360);
			CoordinatePositionMovement(CTransX(300.00f, 0.00f), CTransY(300.00f,0.00f),0,0);
			Lift_Drop_box(0);
			
		}
		else
		{
			//俶勣了崔距屁
			MoveToDefault;

			Lift_Drop_box(-360);
			CoordinatePositionMovement(CTransX(300.00f, 0.00f), CTransY(300.00f,0.00f),0,0);
			Lift_Drop_box(0);
		}
		
	}
	else{
		//慧崔曾円議了崔
		if(state == 2)
		{
			//音俶勣距屁了崔
			Lift_Drop_box(-360);
			CoordinatePositionMovement(0, 0, CTransX(0.00f, 200.00f), CTransY(0.00f,200.00f));
			CoordinatePositionMovement(CTransX(0.00f, 200.00f), CTransY(0.00f,200.00f),CTransX(-300.00f, 200.00f), CTransY(-300.00f,200.00f));
			CoordinatePositionMovement(CTransX(-300.00f, 200.00f), CTransY(-300.00f,200.00f),CTransX(-300.00f, 00.00f), CTransY(-300.00f,00.00f));
			Lift_Drop_box(0);
			Servo_Init();
		}
		else
		{
			//俶勣了崔距屁
			Clockwise;
			Lift_Drop_box(-360);
			CoordinatePositionMovement(0, 0, CTransX(0.00f, 200.00f), CTransY(0.00f,200.00f));
			CoordinatePositionMovement(CTransX(0.00f, 200.00f), CTransY(0.00f,200.00f),CTransX(-300.00f, 200.00f), CTransY(-300.00f,200.00f));
			CoordinatePositionMovement(CTransX(-300.00f, 200.00f), CTransY(-300.00f,200.00f),CTransX(-300.00f, 00.00f), CTransY(-300.00f,00.00f));
			Lift_Drop_box(0);
			Servo_Init();
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
