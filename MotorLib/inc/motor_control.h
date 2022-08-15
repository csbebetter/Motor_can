#ifndef __MOTOR_CONTROL_H
#define __MOTOR_CONTROL_H

#include "math.h"
#include "stm32f4xx.h"
#include "arm_math.h"
#include <stdbool.h>

#define Calibration_Pos 121.785


#define Motor1_ClimaxPos   26.45 
#define Motor2_ClimaxPos   15.93 
#define Motor3_ClimaxPos   26.45
#define Motor4_ClimaxPos   15.93
#define Motor5_ClimaxPos  -26.45
#define Motor6_ClimaxPos  -15.93
#define Motor7_ClimaxPos  -26.45 
#define Motor8_ClimaxPos  -15.93


#define Motor1_Withdraw   66.0703//61.6282//61.3577   
#define Motor2_Withdraw   67.2933//54.5692//46.6206
#define Motor3_Withdraw   43.2201//66.0703//61.6282//61.3577   
#define Motor4_Withdraw   66.2925//67.2933//54.5692//46.6206 
#define Motor5_Withdraw   -66.0703//61.6282//61.3577   
#define Motor6_Withdraw   -67.2933//54.5692//46.6206 
#define Motor7_Withdraw   -43.2201//66.0703//61.6282//61.3577    
#define Motor8_Withdraw   -66.2925//67.2933//54.5692//46.6206 

#define Motor1_LandPos   53.51//81.5479
#define Motor2_LandPos   36.81//59.5449 
#define Motor3_LandPos   53.51//81.5479
#define Motor4_LandPos   36.81//59.5449 
#define Motor5_LandPos   -53.51//81.5479
#define Motor6_LandPos   -36.81//59.5449
#define Motor7_LandPos   -26.45//-53.51//81.5479
#define Motor8_LandPos   -15.93//-36.81//59.5449
typedef enum 
{
	ClassicPIDType      = 0,
	FuzzyPIDType        = 1,
	ProPIDType          = 2,
}PidTypeEnumTypedef;

typedef enum 
{
	POS     = 0,
	SPEED   = 1,
	CURRENT = 2,
}PID_INDEX_Typedef;

typedef enum
{
	NB = 0,
	NM = 1,
	NS = 2,
	ZO = 3,
	PS = 4,
	PM = 5,
	PB = 6
}FuzzyDomain;

extern PidTypeEnumTypedef PosSpeedPidType[3];
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

typedef  enum
{
	DISABLE1,
	STAND,
	STRAIGHT,
	CALIBRATE,
	FLYJUMP,
	STAIR_FIRST,
	STAIR_FIRST_HOLD,
	STAIR_SECOND,
	STAIR_SECOND_HOLD,
	STAIR_THIRD,
	STAIR_FOURTH,
	STAIR_FIFTH,
	UPSLOPE,
	DOWNSLOPE,
	RIGHTROTATE,
	LEFTROTATE,
	STAIR_RIGHTROTATE,
	STAIR_LEFTROTATE,
	NORMALJUMP,
	UPSTAIR_FIX,
	TITA,
	LITTLEJUMP,
}LegState;//系统状态



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


/***fuzzy PID参数结构***/
typedef  struct 
{
	float Kp;
	float Ki;
	float Kd;
	float PreError;
	float Integral; //积分项,存储积分误差×KI
	float LimitOutput;  //  Limit for Output limitation
	float LimitIntegral; //Lower Limit for Integral term limitation
	
	float Prime_Kp;
	float Prime_Ki;
	float Prime_Kd;

	float Kp_Proportion;
	float Ki_Proportion;
	float Kd_Proportion;
}FuzzyPidStructTypedef;


/***专家PID结构体***/
typedef  struct 
{
	float Kp;
	float Ki;
	float Kd;
	float deltaU;
	float Uk;
	float Uk_1;
	float Umax;
	float I_item;

	float emax;
	float emid;
	float emin;
	float emk;

	float deltaEk;
	float deltaEk_1;

	float ek;
	float ek_1;
	float ek_2;

	float K1;
	float K2;
	float epusenne;

	float zerolim;
	unsigned char condition;
         
}ProPidStructTypedef;


/*********PID*******/
extern ClassicPidStructTypedef MotorPositionPid1,MotorSpeedPid1,MotorCurrentPid1,MotorPositionPid2,MotorSpeedPid2,MotorCurrentPid2;
extern ClassicPidStructTypedef MotorPositionPid3,MotorSpeedPid3,MotorCurrentPid3,MotorPositionPid4,MotorSpeedPid4,MotorCurrentPid4;

extern ClassicPidStructTypedef MotorPositionPid5,MotorSpeedPid5,MotorCurrentPid5,MotorPositionPid6,MotorSpeedPid6,MotorCurrentPid6;
extern ClassicPidStructTypedef MotorPositionPid7,MotorSpeedPid7,MotorCurrentPid7,MotorPositionPid8,MotorSpeedPid8,MotorCurrentPid8;

extern ClassicPidStructTypedef PoseRollPID,PosePitchPID,PoseYawPID;


extern ProPidStructTypedef     MotorProPositionPid,MotorProSpeedPid,MotorProCurrentPid;
extern FuzzyPidStructTypedef   MotorFuzzyPositionPid,MotorFuzzySpeedPid,MotorFuzzyCurrentPid;

void  PidInit(void);
void  ProPidStructInit(ProPidStructTypedef *PidStruct);
float ProPidRegulate(float Reference, float Present, ProPidStructTypedef *PidStruct);
void  FuzzyPidStructInit(FuzzyPidStructTypedef *PidStruct);
float FuzzyPidRegulate(float Reference, float PresentFeedback,FuzzyPidStructTypedef *PidStruct);
void  FuzzyPidCalculate(float Reference,float PresentFeedback,FuzzyPidStructTypedef* PidStruct);
float InTriangularFuzzy(float y,float min,float mid,float max);
float TriangularFuzzy(float x,float min,float mid,float max);
float InTrapezoidFuzzy(float y,float min,float max,bool dir);
float TrapezoidFuzzy(float x,float min,float max,bool dir);  
float ClassicPidRegulate(float Reference, float PresentFeedback,ClassicPidStructTypedef *PidStruct);
#endif
