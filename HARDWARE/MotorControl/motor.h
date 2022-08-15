#ifndef __MOTOR_CONTROL_H
#define __MOTOR_CONTROL_H

#include "stm32f4xx.h"
#include "sys.h"
#include "usart.h"
#include <stdbool.h>

#define Motor_1_ID	0x201 
#define Motor_2_ID	0x202 
#define Motor_3_ID	0x203 
#define Motor_4_ID	0x204 
#define Motor_5_ID	0x205

typedef enum 
{
	ClassicPIDType      = 0,
	FuzzyPIDType        = 1,
	ProPIDType          = 2,
}PidTypeEnumTypedef;

extern float (*PidFcnProp[])(float,float,void*);

typedef  enum
{
	IDLE,
	PIDSPEED,
	PIDPOSITION,
	MOTOR_ERROR,
	MOTOR_PWM,   //PWM 开环模式
	MOTOR_CURRENT,
	MOTOR_BUFFER,
}DriverState;//系统状态



typedef struct 
{
	float PositionExpected;
	float	PositionMeasure;
	float SpeedExpected;
	float SpeedMeasure;

	float CurrentExpected;
	float CurrentMeasure;
	
	short PosPre;
	short PosNow;
	
	int32_t 	PWM;
  DriverState    State;	
}MotorTypeDef;

/***PID参数结构***/
typedef  struct 
{
	float Kp;
	float Ki;
	float Kd;
	float LimitOutput;  //  Limit for Output limitation
	float LimitIntegral; //Lower Limit for Integral term limitation
	float Integral; //积分项,存储积分误差×KI
	float PreError;
	int SectionFlag;
	float KpMax;
	float ErrorLine;
}ClassicPidStructTypedef;



extern void Motor_Set_Current(signed short int i1, signed short int i2, signed short int i3, signed short int i4);
extern void Lift_Motor_Set_Current(signed short int i5);
float Myabs(float input);
float Get_RM3508_Distance(MotorTypeDef Motor);
void MotorCurrentPidInit(void);
void MotorPositionPidInit(void);
void MotorSpeedPidInit(void);
void  PidInit(void); 
void Reset_motors(void);
void MotorSpeedExpected(float Spe1,float Spe2,float Spe3,float Spe4,float Spe5);
void MotorPositionExpected(float Spe1,float Spe2,float Spe3,float Spe4,float Spe5);
float ClassicPidRegulate(float Reference, float PresentFeedback,ClassicPidStructTypedef *PidStruct);

#endif
