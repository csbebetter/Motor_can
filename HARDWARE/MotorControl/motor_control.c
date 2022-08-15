#include "motor.h"

/*****PID*****/
//电机状态参数
volatile MotorTypeDef Motor_1, Motor_2,Motor_3, Motor_4;
volatile MotorTypeDef Motor_5;

/*********PID*******/
ClassicPidStructTypedef MotorPositionPid1,MotorSpeedPid1,MotorCurrentPid1,MotorPositionPid2,MotorSpeedPid2,MotorCurrentPid2;
ClassicPidStructTypedef MotorPositionPid3,MotorSpeedPid3,MotorCurrentPid3,MotorPositionPid4,MotorSpeedPid4,MotorCurrentPid4;
ClassicPidStructTypedef MotorPositionPid5,MotorSpeedPid5,MotorCurrentPid5;

float Myabs (float input){
	if (input<0)
	{
		input = -input;
		
	}
	return input;
}

float Get_RM3508_Distance(MotorTypeDef Motor)
{
		int Distance = Motor.PosNow - Motor.PosPre;
		if(Myabs(Distance) > 4000)
			Distance = Distance - Distance/Myabs(Distance) * 8192;
		return ((float)Distance*360.0f/19.0f/8192.0f);		
}
 //重置电机初始角
void Reset_motors(void){
	Motor_1.PositionMeasure = 0;
	Motor_2.PositionMeasure = 0;
	Motor_3.PositionMeasure = 0;
	Motor_4.PositionMeasure = 0;
	Motor_5.PositionMeasure = 0;
	Motor_1.State = PIDSPEED;
	Motor_2.State = PIDSPEED;
	Motor_3.State = PIDSPEED;
	Motor_4.State = PIDSPEED;
	Motor_5.State = PIDSPEED;
	Motor_1.SpeedExpected = 0;
 	Motor_2.SpeedExpected = 0;
	Motor_3.SpeedExpected = 0;
	Motor_4.SpeedExpected = 0;
	Motor_5.SpeedExpected = 0;
}

void MotorCurrentPidInit(void)
{
	MotorCurrentPid1.Kp = 1.3;
	MotorCurrentPid1.Ki = 0.03; 
	MotorCurrentPid1.Kd = 0;
	MotorCurrentPid1.LimitOutput = 16000;
	MotorCurrentPid1.LimitIntegral = 3800;
	MotorCurrentPid1.Integral = 0;
	MotorCurrentPid1.PreError = 0;
	MotorCurrentPid1.SectionFlag = 0;

	MotorCurrentPid2.Kp = 1.3;
	MotorCurrentPid2.Ki = 0.03; 
	MotorCurrentPid2.Kd = 0;
	MotorCurrentPid2.LimitOutput = 16000;
	MotorCurrentPid2.LimitIntegral = 3800;
	MotorCurrentPid2.Integral = 0;
	MotorCurrentPid2.PreError = 0;
	MotorCurrentPid2.SectionFlag = 0;
	
	MotorCurrentPid3.Kp = 1.3;//1.3
	MotorCurrentPid3.Ki = 0.03; 
	MotorCurrentPid3.Kd = 0;
	MotorCurrentPid3.LimitOutput = 16000;
	MotorCurrentPid3.LimitIntegral = 3800;
	MotorCurrentPid3.Integral = 0;
	MotorCurrentPid3.PreError = 0;
	MotorCurrentPid3.SectionFlag = 0;

	MotorCurrentPid4.Kp = 1.3;//1.3
	MotorCurrentPid4.Ki = 0.03; 
	MotorCurrentPid4.Kd = 0;
	MotorCurrentPid4.LimitOutput = 16000;
	MotorCurrentPid4.LimitIntegral = 3800;
	MotorCurrentPid4.Integral = 0;
	MotorCurrentPid4.PreError = 0;
	MotorCurrentPid4.SectionFlag = 0;
	
	MotorCurrentPid5.Kp = 1.3;
	MotorCurrentPid5.Ki = 0.03; 
	MotorCurrentPid5.Kd = 0;
	MotorCurrentPid5.LimitOutput = 16000;
	MotorCurrentPid5.LimitIntegral = 3800;
	MotorCurrentPid5.Integral = 0;
	MotorCurrentPid5.PreError = 0;
	MotorCurrentPid5.SectionFlag = 0;

}


void MotorSpeedPidInit(void)
{
	MotorSpeedPid1.Kp = 250.0f;  //250.0f
	MotorSpeedPid1.Ki =5.0f;     
	MotorSpeedPid1.Kd = 0.0f;
	MotorSpeedPid1.LimitOutput = 16000.0f;
	MotorSpeedPid1.LimitIntegral = 5000.0f;
	MotorSpeedPid1.Integral = 0;
	MotorSpeedPid1.PreError = 0;
	MotorSpeedPid1.SectionFlag = 0;
	MotorSpeedPid1.ErrorLine = 1000;
	MotorSpeedPid1.KpMax = 400;
	
	MotorSpeedPid2.Kp = 250.0f;  
	MotorSpeedPid2.Ki =5.0f;     
	MotorSpeedPid2.Kd = 0.0f;
	MotorSpeedPid2.LimitOutput = 16000.0f;
	MotorSpeedPid2.LimitIntegral = 5000.0f;
	MotorSpeedPid2.Integral = 0;
	MotorSpeedPid2.PreError = 0;
	MotorSpeedPid2.SectionFlag = 0;
	MotorSpeedPid2.ErrorLine = 1000;
	MotorSpeedPid2.KpMax = 400;
	
	MotorSpeedPid3.Kp = 250.0f;  
	MotorSpeedPid3.Ki =5.0f;     
	MotorSpeedPid3.Kd = 0.0f;
	MotorSpeedPid3.LimitOutput = 16000.0f;
	MotorSpeedPid3.LimitIntegral = 5000.0f;
	MotorSpeedPid3.Integral = 0;
	MotorSpeedPid3.PreError = 0;
	MotorSpeedPid3.SectionFlag = 0;
	MotorSpeedPid3.ErrorLine = 1000;
	MotorSpeedPid3.KpMax = 400;
	
	MotorSpeedPid4.Kp = 250.0f;  
	MotorSpeedPid4.Ki =5.0f;     
	MotorSpeedPid4.Kd = 0.0f;
	MotorSpeedPid4.LimitOutput = 16000.0f;
	MotorSpeedPid4.LimitIntegral = 5000.0f;
	MotorSpeedPid4.Integral = 0;
	MotorSpeedPid4.PreError = 0;
	MotorSpeedPid4.SectionFlag = 0;
	MotorSpeedPid4.ErrorLine = 1000;
	MotorSpeedPid4.KpMax = 400;
	
	MotorSpeedPid5.Kp = 220.0f;  //220.of
	MotorSpeedPid5.Ki =5.0f;     
	MotorSpeedPid5.Kd = 0.0f;
	MotorSpeedPid5.LimitOutput = 16000.0f;
	MotorSpeedPid5.LimitIntegral = 5000.0f;
	MotorSpeedPid5.Integral = 0;
	MotorSpeedPid5.PreError = 0;
	MotorSpeedPid5.SectionFlag = 0;
	MotorSpeedPid5.ErrorLine = 1000;
	MotorSpeedPid5.KpMax = 400;
}

void MotorPositionPidInit(void)
{
  MotorPositionPid1.Kp = 0.41;   //2.5
	MotorPositionPid1.Ki = 0;
	MotorPositionPid1.Kd = 0.0001;
	MotorPositionPid1.LimitOutput = 16000;// 
	MotorPositionPid1.LimitIntegral = 2000;
	MotorPositionPid1.Integral = 0;
	MotorPositionPid1.PreError = 0;
	MotorPositionPid1.SectionFlag = 0;
	MotorPositionPid1.ErrorLine = 20;
	MotorPositionPid1.KpMax = 50;
	
	MotorPositionPid2.Kp = 0.41;  //2.5
	MotorPositionPid2.Ki = 0;
	MotorPositionPid2.Kd = 0.0001;
	MotorPositionPid2.LimitOutput = 16000;// 
	MotorPositionPid2.LimitIntegral = 2000;
	MotorPositionPid2.Integral = 0;
	MotorPositionPid2.PreError = 0;
	MotorPositionPid2.SectionFlag = 0;
	MotorPositionPid2.ErrorLine = 20;
	MotorPositionPid2.KpMax = 50;
	
	MotorPositionPid3.Kp = 0.41;   //2.5
	MotorPositionPid3.Ki = 0.00;
	MotorPositionPid3.Kd = 0.0001;
	MotorPositionPid3.LimitOutput = 16000;// 
	MotorPositionPid3.LimitIntegral = 2000;
	MotorPositionPid3.Integral = 0;
	MotorPositionPid3.PreError = 0;
	MotorPositionPid3.SectionFlag = 0;
	MotorPositionPid3.ErrorLine = 20;
	MotorPositionPid3.KpMax = 50;
	
	MotorPositionPid4.Kp = 0.41;  //2.5
	MotorPositionPid4.Ki = 0;
	MotorPositionPid4.Kd = 0.0001;
	MotorPositionPid4.LimitOutput = 16000;// 
	MotorPositionPid4.LimitIntegral = 2000;
	MotorPositionPid4.Integral = 0;
	MotorPositionPid4.PreError = 0;
	MotorPositionPid4.SectionFlag = 0;
	MotorPositionPid4.ErrorLine = 20;
	MotorPositionPid4.KpMax = 50;
	
	MotorPositionPid5.Kp = 1;   //2.5
	MotorPositionPid5.Ki = 0;
	MotorPositionPid5.Kd = 0.01;
	MotorPositionPid5.LimitOutput = 16000;// 
	MotorPositionPid5.LimitIntegral = 2000;
	MotorPositionPid5.Integral = 0;
	MotorPositionPid5.PreError = 0;
	MotorPositionPid5.SectionFlag = 0;
	MotorPositionPid5.ErrorLine = 20;
	MotorPositionPid5.KpMax = 50;
}

	
void PidInit(void)
{
	MotorCurrentPidInit();
	MotorSpeedPidInit();	  
	MotorPositionPidInit();
}
	

float ClassicPidRegulate(float Reference, float PresentFeedback,ClassicPidStructTypedef *PID_Struct)
{
	float error;
	float error_inc;
	float pTerm;
	float iTerm;
	float dTerm;
	float dwAux;
	float output;
	/*error computation*/
	error = Reference - PresentFeedback;
	
	/*proportional term computation*/

	if(PID_Struct->SectionFlag == 1)
	{
		if( Myabs(error) >=  PID_Struct->ErrorLine )
			pTerm = error * PID_Struct->KpMax;
		else
			pTerm = error * PID_Struct->Kp;
	}
	else
			pTerm = error * PID_Struct->Kp;
	
	/*Integral term computation*/
	
	iTerm = error * PID_Struct->Ki;
	
	dwAux = PID_Struct->Integral + iTerm;
	/*limit integral*/
	if (dwAux > PID_Struct->LimitIntegral)
	{
		PID_Struct->Integral = PID_Struct->LimitIntegral;
	} else if (dwAux < -1*PID_Struct->LimitIntegral)
	{
		PID_Struct->Integral = -1*PID_Struct->LimitIntegral;
	} else
	{
	  PID_Struct->Integral = dwAux;
	}
	/*differential term computation*/
	
	error_inc = error - PID_Struct->PreError;
	dTerm = error_inc * PID_Struct->Kd;
	PID_Struct->PreError = error;

	output = pTerm + PID_Struct->Integral + dTerm;

	/*limit output*/
	if (output >= PID_Struct->LimitOutput)
	{
		return (PID_Struct->LimitOutput);
	} else if (output < -1.0f*PID_Struct->LimitOutput)
	{
		return (-1.0f*PID_Struct->LimitOutput);
	} else
	{
		return output;
	}
}


void motor_driver(void)   //驱动电机运动，下达PID电流指令，完成PID计算
{
	switch(Motor_1.State)
		{
			case PIDPOSITION:
				Motor_1.SpeedExpected =  ClassicPidRegulate(Motor_1.PositionExpected,Motor_1.PositionMeasure,&MotorPositionPid1);		
			case PIDSPEED:						
				Motor_1.CurrentExpected = ClassicPidRegulate(Motor_1.SpeedExpected,Motor_1.SpeedMeasure,&MotorSpeedPid1);
			case MOTOR_CURRENT: 					 
				Motor_1.PWM = ClassicPidRegulate(Motor_1.CurrentExpected,Motor_1.CurrentMeasure,&MotorCurrentPid1);					
				break; 
			default:
				break;
		}
		switch(Motor_2.State)
		{
			case PIDPOSITION:				
				Motor_2.SpeedExpected =  ClassicPidRegulate(Motor_2.PositionExpected,Motor_2.PositionMeasure,&MotorPositionPid2);
			case PIDSPEED:						
				Motor_2.CurrentExpected = ClassicPidRegulate(Motor_2.SpeedExpected,Motor_2.SpeedMeasure,&MotorSpeedPid2);
			case MOTOR_CURRENT: 					 
				Motor_2.PWM = ClassicPidRegulate(Motor_2.CurrentExpected,Motor_2.CurrentMeasure,&MotorCurrentPid2);					
				break; 
			default:
				break;
		}
		switch(Motor_3.State)
		{
			case PIDPOSITION:				
				Motor_3.SpeedExpected =  ClassicPidRegulate(Motor_3.PositionExpected,Motor_3.PositionMeasure,&MotorPositionPid3);
			case PIDSPEED:						
				Motor_3.CurrentExpected = ClassicPidRegulate(Motor_3.SpeedExpected,Motor_3.SpeedMeasure,&MotorSpeedPid3);
            case MOTOR_CURRENT: 					 
			    Motor_3.PWM = ClassicPidRegulate(Motor_3.CurrentExpected,Motor_3.CurrentMeasure,&MotorCurrentPid3);					
			    break; 
			default:
				break;
		}
		switch(Motor_4.State)
		{
			case PIDPOSITION:				
				Motor_4.SpeedExpected =  ClassicPidRegulate(Motor_4.PositionExpected,Motor_4.PositionMeasure,&MotorPositionPid4);
			case PIDSPEED:						
				Motor_4.CurrentExpected = ClassicPidRegulate(Motor_4.SpeedExpected,Motor_4.SpeedMeasure,&MotorSpeedPid4);
			case MOTOR_CURRENT: 					 
				Motor_4.PWM = ClassicPidRegulate(Motor_4.CurrentExpected,Motor_4.CurrentMeasure,&MotorCurrentPid4);					
				break; 
			default:
				break;
		}
		switch(Motor_5.State)
		{
			case PIDPOSITION:				
				Motor_5.SpeedExpected =  ClassicPidRegulate(Motor_5.PositionExpected,Motor_5.PositionMeasure,&MotorPositionPid5);
			case PIDSPEED:						
				Motor_5.CurrentExpected = ClassicPidRegulate(Motor_5.SpeedExpected,Motor_5.SpeedMeasure,&MotorSpeedPid5);
			case MOTOR_CURRENT: 					 
				Motor_5.PWM = ClassicPidRegulate(Motor_5.CurrentExpected,Motor_5.CurrentMeasure,&MotorCurrentPid5);					
				break; 
			default:
				break;
		}
		Motor_Set_Current(Motor_1.PWM,Motor_2.PWM,Motor_3.PWM,Motor_4.PWM);
		Lift_Motor_Set_Current(Motor_5.PWM);
		//printf("\r\n Motor1: %.3f; Motor2: %.3f; Motor3: %.3f; Motor4: %.3f; \r\n",Motor_1.PositionMeasure, Motor_2.PositionMeasure,Motor_3.PositionMeasure,Motor_4.PositionMeasure);
}

void MotorSpeedExpected(float Spe1,float Spe2,float Spe3,float Spe4,float Spe5){
	Motor_1.State = PIDSPEED;
	Motor_2.State = PIDSPEED;
	Motor_3.State = PIDSPEED;
	Motor_4.State = PIDSPEED;
	Motor_5.State = PIDSPEED;
	Motor_1.SpeedExpected = Spe1;
 	Motor_2.SpeedExpected = Spe2;
	Motor_3.SpeedExpected = Spe3;
	Motor_4.SpeedExpected = Spe4;
	Motor_5.SpeedExpected = Spe5;
	motor_driver();
}

void MotorPositionExpected(float Pos1,float Pos2,float Pos3,float Pos4,float Pos5){
	Motor_1.State = PIDPOSITION;
	Motor_2.State = PIDPOSITION;
	Motor_3.State = PIDPOSITION;
	Motor_4.State = PIDPOSITION;
	Motor_5.State = PIDPOSITION;
	Motor_1.PositionExpected = Pos1;
 	Motor_2.PositionExpected = Pos2;
	Motor_3.PositionExpected = Pos3;
	Motor_4.PositionExpected = Pos4;
	Motor_5.PositionExpected = Pos5;
	motor_driver();
}




