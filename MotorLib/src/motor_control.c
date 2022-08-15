#include "motor_control.h"
#include "MyType.h"

/*****PID*****/
ClassicPidStructTypedef MotorPositionPid1,MotorSpeedPid1,MotorCurrentPid1,MotorPositionPid2,MotorSpeedPid2,MotorCurrentPid2;
ClassicPidStructTypedef MotorPositionPid3,MotorSpeedPid3,MotorCurrentPid3,MotorPositionPid4,MotorSpeedPid4,MotorCurrentPid4;

ClassicPidStructTypedef MotorPositionPid5,MotorSpeedPid5,MotorCurrentPid5,MotorPositionPid6,MotorSpeedPid6,MotorCurrentPid6;
ClassicPidStructTypedef MotorPositionPid7,MotorSpeedPid7,MotorCurrentPid7,MotorPositionPid8,MotorSpeedPid8,MotorCurrentPid8;

ClassicPidStructTypedef PoseRollPID, PosePitchPID, PoseYawPID;

ProPidStructTypedef     MotorProPositionPid,MotorProSpeedPid,MotorProCurrentPid;
FuzzyPidStructTypedef   MotorFuzzyPositionPid,MotorFuzzySpeedPid,MotorFuzzyCurrentPid;

//电机状态参数
volatile MotorTypeDef Motor;
volatile MotorTypeDef Motor_1, Motor_2,Motor_3, Motor_4;
volatile MotorTypeDef Motor_5, Motor_6,Motor_7, Motor_8;

volatile LegState leg_state;
volatile bool Gait = 1;	//0为WALK，1为trot
volatile bool StartPosFlag = 0;
volatile bool LeftRotateFlag=0;
volatile bool StairLeftRotateFlag=0;
volatile bool RightRotateFlag=0;
volatile bool StairRightRotateFlag=0;
volatile bool JumpFlag = 0;
volatile int CurrentPoint = 0;

volatile ChassisInfoTypedef MyChassis;

/*
										   电机位置示意
										  -----1(2)-----
                                            \		 /
										     \		/
											  \	   /
											   \  /
                                                 `

*/

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
	
	MotorCurrentPid3.Kp = 1.35;//1.3
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

	MotorCurrentPid6.Kp = 1.3;
	MotorCurrentPid6.Ki = 0.03; 
	MotorCurrentPid6.Kd = 0;
	MotorCurrentPid6.LimitOutput = 16000;
	MotorCurrentPid6.LimitIntegral = 3800;
	MotorCurrentPid6.Integral = 0;
	MotorCurrentPid6.PreError = 0;
	MotorCurrentPid6.SectionFlag = 0;
	
	MotorCurrentPid7.Kp = 1.3;
	MotorCurrentPid7.Ki = 0.03; 
	MotorCurrentPid7.Kd = 0;
	MotorCurrentPid7.LimitOutput = 16000;
	MotorCurrentPid7.LimitIntegral = 3800;
	MotorCurrentPid7.Integral = 0;
	MotorCurrentPid7.PreError = 0;
	MotorCurrentPid7.SectionFlag = 0;

	MotorCurrentPid8.Kp = 1.3;
	MotorCurrentPid8.Ki = 0.03; 
	MotorCurrentPid8.Kd = 0;
	MotorCurrentPid8.LimitOutput = 16000;
	MotorCurrentPid8.LimitIntegral = 3800;
	MotorCurrentPid8.Integral = 0;
	MotorCurrentPid8.PreError = 0;
	MotorCurrentPid8.SectionFlag = 0;
	
	
}

void MotorSpeedPidInit(void)
{
  MotorSpeedPid1.Kp = 220.0f;  
	MotorSpeedPid1.Ki =5.0f;     
	MotorSpeedPid1.Kd = 0.0f;
	MotorSpeedPid1.LimitOutput = 16000.0f;
	MotorSpeedPid1.LimitIntegral = 10000.0f;
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
	MotorSpeedPid3.Ki =6.0f;     
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
	
	MotorSpeedPid5.Kp = 220.0f;  
	MotorSpeedPid5.Ki =5.0f;     
	MotorSpeedPid5.Kd = 0.0f;
	MotorSpeedPid5.LimitOutput = 16000.0f;
	MotorSpeedPid5.LimitIntegral = 5000.0f;
	MotorSpeedPid5.Integral = 0;
	MotorSpeedPid5.PreError = 0;
	MotorSpeedPid5.SectionFlag = 0;
	MotorSpeedPid5.ErrorLine = 1000;
	MotorSpeedPid5.KpMax = 400;
	
	MotorSpeedPid6.Kp = 250.0f;  
	MotorSpeedPid6.Ki =5.0f;     
	MotorSpeedPid6.Kd = 0.0f;
	MotorSpeedPid6.LimitOutput = 16000.0f;
	MotorSpeedPid6.LimitIntegral = 5000.0f;
	MotorSpeedPid6.Integral = 0;
	MotorSpeedPid6.PreError = 0;
	MotorSpeedPid6.SectionFlag = 0;
	MotorSpeedPid6.ErrorLine = 1000;
	MotorSpeedPid6.KpMax = 400;
	
	MotorSpeedPid7.Kp = 220.0f;  
	MotorSpeedPid7.Ki =5.0f;     
	MotorSpeedPid7.Kd = 0.0f;
	MotorSpeedPid7.LimitOutput = 16000.0f;
	MotorSpeedPid7.LimitIntegral = 5000.0f;
	MotorSpeedPid7.Integral = 0;
	MotorSpeedPid7.PreError = 0;
	MotorSpeedPid7.SectionFlag = 0;
	MotorSpeedPid7.ErrorLine = 1000;
	MotorSpeedPid7.KpMax = 400;
	
	MotorSpeedPid8.Kp = 250.0f;  
	MotorSpeedPid8.Ki =5.0f;     
	MotorSpeedPid8.Kd = 0.0f;
	MotorSpeedPid8.LimitOutput = 16000.0f;
	MotorSpeedPid8.LimitIntegral = 5000.0f;
	MotorSpeedPid8.Integral = 0;
	MotorSpeedPid8.PreError = 0;
	MotorSpeedPid8.SectionFlag = 0;
	MotorSpeedPid8.ErrorLine = 1000;
	MotorSpeedPid8.KpMax = 400;
}

void MotorPositionPidInit(void)
{
  MotorPositionPid1.Kp = 2.3;   //2.2
	MotorPositionPid1.Ki = 0;
	MotorPositionPid1.Kd = 0.0001;
	MotorPositionPid1.LimitOutput = 16000;// 
	MotorPositionPid1.LimitIntegral = 2000;
	MotorPositionPid1.Integral = 0;
	MotorPositionPid1.PreError = 0;
	MotorPositionPid1.SectionFlag = 0;
	MotorPositionPid1.ErrorLine = 20;
	MotorPositionPid1.KpMax = 50;
	
	MotorPositionPid2.Kp = 2.3;  //2.5
	MotorPositionPid2.Ki = 0;
	MotorPositionPid2.Kd = 0.0001;
	MotorPositionPid2.LimitOutput = 16000;// 
	MotorPositionPid2.LimitIntegral = 2000;
	MotorPositionPid2.Integral = 0;
	MotorPositionPid2.PreError = 0;
	MotorPositionPid2.SectionFlag = 0;
	MotorPositionPid2.ErrorLine = 20;
	MotorPositionPid2.KpMax = 50;
	
	MotorPositionPid3.Kp = 2.5;   //2.2
	MotorPositionPid3.Ki = 0.00;
	MotorPositionPid3.Kd = 0.0001;
	MotorPositionPid3.LimitOutput = 16000;// 
	MotorPositionPid3.LimitIntegral = 2000;
	MotorPositionPid3.Integral = 0;
	MotorPositionPid3.PreError = 0;
	MotorPositionPid3.SectionFlag = 0;
	MotorPositionPid3.ErrorLine = 20;
	MotorPositionPid3.KpMax = 50;
	
	MotorPositionPid4.Kp = 2.5;  //2.5
	MotorPositionPid4.Ki = 0;
	MotorPositionPid4.Kd = 0.0001;
	MotorPositionPid4.LimitOutput = 16000;// 
	MotorPositionPid4.LimitIntegral = 2000;
	MotorPositionPid4.Integral = 0;
	MotorPositionPid4.PreError = 0;
	MotorPositionPid4.SectionFlag = 0;
	MotorPositionPid4.ErrorLine = 20;
	MotorPositionPid4.KpMax = 50;
	
	MotorPositionPid5.Kp = 2.3;   //2.2
	MotorPositionPid5.Ki = 0;
	MotorPositionPid5.Kd = 0.0001;
	MotorPositionPid5.LimitOutput = 16000;// 
	MotorPositionPid5.LimitIntegral = 2000;
	MotorPositionPid5.Integral = 0;
	MotorPositionPid5.PreError = 0;
	MotorPositionPid5.SectionFlag = 0;
	MotorPositionPid5.ErrorLine = 20;
	MotorPositionPid5.KpMax = 50;
	
	MotorPositionPid6.Kp = 2.3;  //2.5
	MotorPositionPid6.Ki = 0;
	MotorPositionPid6.Kd = 0.0001;
	MotorPositionPid6.LimitOutput = 16000;// 
	MotorPositionPid6.LimitIntegral = 2000;
	MotorPositionPid6.Integral = 0;
	MotorPositionPid6.PreError = 0;
	MotorPositionPid6.SectionFlag = 0;
	MotorPositionPid6.ErrorLine = 20;
	MotorPositionPid6.KpMax = 50;
	
	MotorPositionPid7.Kp = 2.4;   //2.2
	MotorPositionPid7.Ki = 0;
	MotorPositionPid7.Kd = 0.0001;
	MotorPositionPid7.LimitOutput = 16000;// 
	MotorPositionPid7.LimitIntegral = 2000;
	MotorPositionPid7.Integral = 0;
	MotorPositionPid7.PreError = 0;
	MotorPositionPid7.SectionFlag = 0;
	MotorPositionPid7.ErrorLine = 20;
	MotorPositionPid7.KpMax = 50;
	
	MotorPositionPid8.Kp = 2.4;  //2.5
	MotorPositionPid8.Ki = 0;
	MotorPositionPid8.Kd = 0.0001;
	MotorPositionPid8.LimitOutput = 16000;// 
	MotorPositionPid8.LimitIntegral = 2000;
	MotorPositionPid8.Integral = 0;
	MotorPositionPid8.PreError = 0;
	MotorPositionPid8.SectionFlag = 0;
	MotorPositionPid8.ErrorLine = 20;
	MotorPositionPid8.KpMax = 50;
}

void PosePidInit()
{
	PoseRollPID.Kp = 2.5;  
	PoseRollPID.Ki = 0;
	PoseRollPID.Kd = 0.0001;
	PoseRollPID.LimitOutput = 12000;// 
	PoseRollPID.LimitIntegral = 2000;
	PoseRollPID.Integral = 0;
	PoseRollPID.PreError = 0;
	PoseRollPID.SectionFlag = 0;
	PoseRollPID.ErrorLine = 20;
	PoseRollPID.KpMax = 50;
	
	PosePitchPID.Kp = 2.5;  
	PosePitchPID.Ki = 0;
	PosePitchPID.Kd = 0.0001;
	PosePitchPID.LimitOutput = 12000;// 
	PosePitchPID.LimitIntegral = 2000;
	PosePitchPID.Integral = 0;
	PosePitchPID.PreError = 0;
	PosePitchPID.SectionFlag = 0;
	PosePitchPID.ErrorLine = 20;
	PosePitchPID.KpMax = 50;
}

void PidInit(void)
{
	MotorCurrentPidInit();
	MotorSpeedPidInit();	  
  MotorPositionPidInit();
	PosePidInit();
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
		if( fabs(error) >=  PID_Struct->ErrorLine )
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

void FuzzyPidCalculate(float Reference,float PresentFeedback,FuzzyPidStructTypedef* PidStruct);

float FuzzyPidRegulate(float Reference, float PresentFeedback,FuzzyPidStructTypedef* PidStruct)
{ 
		FuzzyPidCalculate(Reference, PresentFeedback,PidStruct);
		return ClassicPidRegulate(Reference,PresentFeedback,(ClassicPidStructTypedef*)PidStruct);
}

/********************************************************************************
Fuzzy_PID Functions
1.NB 1 NM 2 NS 3 ZO 4 PS 5 PM 6 PB 7
2.Error~[-300,300],ErrorInc~[-100,100]  不同部分需要测试
3.DeltaPara[0],DeltaPara[1],DeltaPara[2]分别对应Kp,Ki,Kd的模糊输出，乘以相应
  的Proportion即可映射到其物理论域
4.TriangularFuzzy和InTriangularFuzzy为三角模糊化,和去三角模糊化函数
********************************************************************************/

#define FuzzyDomainMax 3
/* 输入映射变量,和输出映射变量(对Kp,Ki,Kd均不同，由Kp_Proportion等确定) 提供模糊控制器和外部设备的接口，使模糊控制器的输入输出量和外部匹配*/ 

#define EorrorMax  80//20//20//30//80  //通过Max与FuzzyDominMax之比得到映射系数，将Eorror和EorrorInc均映射到[-3,3]的模糊论域。计算FuzzyDomainMax/FuzzyInMax = 3/300
#define EorrorIncMax  30//10//5//10//30   //Error~[-240,240],ErrorInc~[-90,90]

void FuzzyPidStructInit(FuzzyPidStructTypedef *PidStruct)
{
		PidStruct->Kp = 0;
		PidStruct->Ki = 0 ;
		PidStruct->Kd = 0;
	
    #ifdef MecMainMotor_Up
		PidStruct->Prime_Kp = 1.85;     //Kp初始值  Kp最终输出值等于(Prime_Kp + deltaKp)
		PidStruct->Prime_Ki = 0.05;
		PidStruct->Prime_Kd = 0.18;
	  #endif
	
	  #ifdef MecAuxMotor_Up
		PidStruct->Prime_Kp = 1.30;     //Kp初始值  Kp最终输出值等于(Prime_Kp + deltaKp)
		PidStruct->Prime_Ki = 0.05;
		PidStruct->Prime_Kd = 0;
	  #endif
	
	  #ifdef MecMainMotor_Down
		PidStruct->Prime_Kp = 1.25;     //Kp初始值  Kp最终输出值等于(Prime_Kp + deltaKp)
		PidStruct->Prime_Ki = 0.03;
		PidStruct->Prime_Kd = 0.35;
	  #endif
	
	  #ifdef MecAuxMotor_Down
		PidStruct->Prime_Kp = 1.54;     //Kp初始值  Kp最终输出值等于(Prime_Kp + deltaKp)
		PidStruct->Prime_Ki = 0.01;
		PidStruct->Prime_Kd = 0;
	  #endif
		
		PidStruct->PreError = 0;
		PidStruct->Integral = 0; 			//积分项,存储积分误差×KI
		
		PidStruct->LimitOutput = 2050;   //  Limit for Output limitation
		PidStruct->LimitIntegral = 2050;  //  Lower Limit for Integral term limitation
		
		#ifdef MecMainMotor_Up
		PidStruct->Kp_Proportion = 0.45;    //
		PidStruct->Ki_Proportion = 0;
		PidStruct->Kd_Proportion = 0;
		#endif
		
		#ifdef MecAuxMotor_Up
		PidStruct->Kp_Proportion = 0;    //
		PidStruct->Ki_Proportion = 0;
		PidStruct->Kd_Proportion = 0;
		#endif
		
		#ifdef MecMainMotor_Down
		PidStruct->Kp_Proportion = 0.45;    //
		PidStruct->Ki_Proportion = 0;
		PidStruct->Kd_Proportion = 0;
		#endif
		
		#ifdef MecAuxMotor_Down
		PidStruct->Kp_Proportion = 0.42;    //
		PidStruct->Ki_Proportion = 0;
		PidStruct->Kd_Proportion = 0;
		#endif
}

const int Kp[7][7]=
	{
		{PB,PB,PM,PS,PS,ZO,ZO},
		{PB,PB,PM,PS,PS,ZO,NS},
		{PM,PM,PM,PS,ZO,NS,NS},
		{PM,PM,PS,ZO,NS,NM,NM},
		{PS,PS,ZO,NS,NS,NM,NM},
		{PS,ZO,NS,NM,NM,NM,NB},
		{ZO,ZO,NM,NM,NM,NB,NB}
	};
const int Ki[7][7]=
	{
		{NB,NB,NM,NM,NS,ZO,ZO},
		{NB,NB,NM,NS,NS,ZO,ZO},
		{NB,NM,NS,NS,ZO,PS,PS},
		{NM,NM,NS,ZO,PS,PM,PM},
		{NM,NS,ZO,PS,PS,PM,PM},
		{ZO,ZO,PS,PS,PM,PB,PB},
		{ZO,ZO,PS,PM,PM,PB,PB}
	};
const int Kd[7][7]=
	{
		{PS,NS,NB,NB,NB,NM,PS},
		{PS,NS,NB,NM,NM,NS,ZO},
		{ZO,NS,NM,NM,NS,NS,ZO},
		{ZO,NS,NS,NS,NS,NS,ZO},
		{ZO,ZO,ZO,ZO,ZO,ZO,ZO},
		{PB,NS,PS,PS,PS,PS,PB},
		{PB,PM,PM,PM,PS,PS,PB}
	};
		
void FuzzyPidCalculate(float Reference, float PresentFeedback,FuzzyPidStructTypedef *PidStruct)
{
	float error;
	float error_inc;
	float deltaP;
	float deltaI;
	float deltaD;
	float U_Of_Error[7];
	float U_Of_Error_inc[7];
	float Min_U[7][7],DeltaPara[3],Max_U;
  
	int U_Of_Max[3];
	int i=0,j=0,m=0,n=0;
	
	/*error computation*/

	error = Reference - PresentFeedback;
	if(error<-EorrorMax) 
		error = -EorrorMax;
	if(error>EorrorMax)
		error = EorrorMax;	
	
	error_inc = error - PidStruct->PreError ;
	if(error_inc<-EorrorIncMax) 
		error_inc=-EorrorIncMax;
	if(error_inc>EorrorIncMax)
		error_inc=EorrorIncMax;
	
	
	error = error*FuzzyDomainMax/EorrorMax;
	error_inc = error_inc*FuzzyDomainMax/EorrorIncMax;
	
	/*error 隶属度计算*/
	U_Of_Error[NB] = TrapezoidFuzzy (error,-3,-1, 0);
	U_Of_Error[NM] = TriangularFuzzy(error,-3,-2, 0);
	U_Of_Error[NS] = TriangularFuzzy(error,-3,-1, 1);
	U_Of_Error[ZO] = TriangularFuzzy(error,-2, 0, 2);
	U_Of_Error[PS] = TriangularFuzzy(error,-1, 1, 3);
	U_Of_Error[PM] = TriangularFuzzy(error, 0, 2, 3);
	U_Of_Error[PB] = TrapezoidFuzzy (error, 1, 3, 1);

	/*error_inc 隶属度计算*/
	
	U_Of_Error_inc[NB] = TrapezoidFuzzy (error_inc,-3,-1, 0);
	U_Of_Error_inc[NM] = TriangularFuzzy(error_inc,-3,-2, 0);
	U_Of_Error_inc[NS] = TriangularFuzzy(error_inc,-3,-1, 1);
	U_Of_Error_inc[ZO] = TriangularFuzzy(error_inc,-2, 0, 2);
	U_Of_Error_inc[PS] = TriangularFuzzy(error_inc,-1, 1, 3);
	U_Of_Error_inc[PM] = TriangularFuzzy(error_inc, 0, 2, 3);
	U_Of_Error_inc[PB] = TrapezoidFuzzy (error_inc, 1, 3, 1);
	
	/*模糊推理过程*/
	
	/*i表示行数，j表示列,求Error和ErrorInc隶属度相与最小值*/   
	for(i=0;i<7;i++)
	{
		for(j=0;j<7;j++)
		{
			Min_U[i][j] = (U_Of_Error_inc[j]<U_Of_Error[i])?U_Of_Error_inc[j]:U_Of_Error[i];	
		}
	}
	/*最大隶属度法：求隶属度相与之后的最大隶属度*/
	for(i=0;i<7;i++)
	{
		for(j=0;j<7;j++)
		{
	    if(Min_U[m][n]<Min_U[i][j])
			{
				m = i;
				n = j;
			}				
		}
	}
	Max_U = Min_U[m][n];
	U_Of_Max[0] = Kp[m][n];
	U_Of_Max[1] = Ki[m][n];
	U_Of_Max[2] = Kd[m][n];
	/*去模糊化*/
	for(i=0;i<3;i++)
	{
		switch(U_Of_Max[i])
		{
			case NB: 
			{
				DeltaPara[i] = InTrapezoidFuzzy(Max_U,-3,-1,0);
				break;
			}
			case NM: 			
			{
				DeltaPara[i] = InTriangularFuzzy(Max_U,-3,-2, 0);
				break;
			}
			case NS:
			{
				DeltaPara[i] = InTriangularFuzzy(Max_U,-3,-1, 1);
				break;
			}
			case ZO: 
			{
				DeltaPara[i] = InTriangularFuzzy(Max_U,-2, 0, 2);
				break;
			}
			case PS: 
			{
				DeltaPara[i] = InTriangularFuzzy(Max_U,-1, 1, 3);
				break;
			}
			case PM:
			{
				DeltaPara[i] = InTriangularFuzzy(Max_U, 0, 2, 3);
				break;
			}
			case PB:
			{
			  DeltaPara[i] = InTrapezoidFuzzy(Max_U, 1, 3, 1);
				break;
			}
			default: break;
		}
	}
	/*去模糊化输出乘以相应映射系数，转换到物理输出论域*/
	deltaP = (DeltaPara[0])*PidStruct->Kp_Proportion;
	deltaI = (DeltaPara[1])*PidStruct->Ki_Proportion;
	deltaD = (DeltaPara[2])*PidStruct->Kd_Proportion;
	PidStruct->Kp=PidStruct->Prime_Kp+deltaP;
	PidStruct->Ki=PidStruct->Prime_Ki+deltaI;
	PidStruct->Kd=PidStruct->Prime_Kd+deltaD;
	
}

float TriangularFuzzy(float x,float min,float mid,float max) 
{
	if(x<=min) return 0;
	else if((x>min)&&(x<=mid)) return (x-min)/(mid-min);
	else if((x>mid)&&(x<=max)) return (max-x)/(max-mid);
	else if((x>max)) return 0;
}

float InTriangularFuzzy(float y,float min,float mid,float max)  //平均值法
{
	float temp1,temp2;
	temp1 = (mid-min)*y + min;
	temp2 = max - (max-mid)*y;
	return (temp1+temp2)/2;
}

float TrapezoidFuzzy(float x,float min,float max,bool dir)   //模糊域NB取dir=0,PB处取dir=1
{
	if(x<=min) return !dir;
	else if((x>min)&&(x<=max))
	{
		if(dir) return (x-min)/(max-min);
		else return (max-x)/(max-min);
	}
	else if(x>max) return dir;
}

float InTrapezoidFuzzy(float y,float min,float max,bool dir)
{
   if(dir) return (max-min)*y+min;
	 else return max-(max-min)*y;
}



/********************************************************************************
专家PID初始化
参数说明
Professional_PID_StructTypedef *PidStruct  要初始化的专家PID
float Kp                                   比例系数
float Ki                                   积分系数
float Kd                                   微分系数
float Umax                                 最大输出
float k1                                   放大系数,k1>1
float k2                                   抑制系数,0<k2<1
float epusenne                             加入积分的定义域
float emax                                 误差非常大的定义域
float emid                                 误差中等大的定义域
float emin                                 误差较小的定义域
float zerolim                              死区定义域
********************************************************************************/
void ProPidStructInit(ProPidStructTypedef *PidStruct)
{
    PidStruct->ek=0;
    PidStruct->ek_1=0;
    PidStruct->ek_2=0;
    PidStruct->deltaEk=0;
    PidStruct->deltaEk_1=0;
    PidStruct->emk=0;
    PidStruct->Uk=0;
    PidStruct->Uk_1=0;
    PidStruct->deltaU=0;
	  PidStruct->I_item=0;
}

float ProPidRegulate(float Reference, float Present, ProPidStructTypedef *PidStruct)
{
	PidStruct->ek = Reference - Present;
	PidStruct->deltaEk = PidStruct->ek - PidStruct->ek_1;
	PidStruct->I_item=PidStruct->I_item+PidStruct->ek;
	if(PidStruct->I_item>4000)
		PidStruct->I_item=4000;
	else if(PidStruct->I_item<-4000)
		PidStruct->I_item=-4000;
		
	if (PidStruct->ek>PidStruct->emax)
	{
       PidStruct->condition=1;
			 PidStruct->Uk=PidStruct->Umax;
	}
	if(PidStruct->ek <(-PidStruct->emax))
	{
      PidStruct->condition=1;
			PidStruct->Uk=-PidStruct->Umax;
	}
	
	if (PidStruct->ek*PidStruct->deltaEk>0 || PidStruct->deltaEk==0)
	{
		//绝对值在增大或者误差是一个常值
		if (((PidStruct->ek >= PidStruct->emid)&&(PidStruct->ek <= PidStruct->emax))||((PidStruct->ek <= -PidStruct->emid)
			&& (PidStruct->ek >= -PidStruct->emax)))
		{
			//误差还比较大
			PidStruct->condition=2;
			PidStruct->Uk =PidStruct->K1*(PidStruct->Kp*PidStruct->ek  +PidStruct->Ki*PidStruct->I_item+PidStruct->Kd*(PidStruct->ek -PidStruct->ek_1));
		}
		else if (((PidStruct->ek <= PidStruct->emid)&&(PidStruct->ek>=PidStruct->emin))||((PidStruct->ek >= -PidStruct->emid)&&(PidStruct->ek<=-PidStruct->emin)))
		{
			//误差在增大但值比较小，普通调节
			PidStruct->condition=3;
			PidStruct->Uk =PidStruct->K2*(PidStruct->Kp*PidStruct->ek+PidStruct->Ki*PidStruct->I_item+PidStruct->Kd*(PidStruct->ek -PidStruct->ek_1));
		}
	}
	if ((PidStruct->ek * PidStruct->deltaEk<0)&&(PidStruct->deltaEk* PidStruct->deltaEk_1>0))
	{
		//误差在减小保持输出
		PidStruct->condition=4;
		PidStruct->Uk = PidStruct->Uk_1;
	}
	if ((PidStruct->ek*PidStruct->deltaEk<0)&&(PidStruct->deltaEk* PidStruct->deltaEk_1<0))
	{
		//误差处于极值
		PidStruct->emk = PidStruct->ek_1;
		if (((PidStruct->ek >= PidStruct->emid)&&(PidStruct->ek <= PidStruct->emax))|| ((PidStruct->ek <= -PidStruct->emid)&&(PidStruct->ek >= -PidStruct->emax)))
		{
			//误差比较大
			PidStruct->condition=5;
			PidStruct->Uk =PidStruct->K1*PidStruct->Kp*PidStruct->ek;
		}
		else if (((PidStruct->ek >= PidStruct->emin)&&(PidStruct->ek <= PidStruct->emid))||((PidStruct->ek <= -PidStruct->emin)&&(PidStruct->ek >= -PidStruct->emid)))
		{
			//误差比较小
			PidStruct->condition=6;
			PidStruct->Uk =PidStruct->K2*PidStruct->Kp*PidStruct->ek;
		}
	}
	
	if (((PidStruct->ek>= PidStruct->epusenne)&&(PidStruct->ek<=PidStruct->emin))||((PidStruct->ek<=-PidStruct->epusenne)&&(PidStruct->ek>=-PidStruct->emin)))
	{
		//误差小
		PidStruct->condition=7;
		PidStruct->Uk =PidStruct->K2*PidStruct->Kp*PidStruct->ek;
	}
	
	if ((PidStruct->ek<= PidStruct->epusenne)&&(PidStruct->ek>=-PidStruct->epusenne))
	{
		//误差达到积分区
		PidStruct->condition=8;
		PidStruct->Uk =PidStruct->K2*(PidStruct->Kp *PidStruct->ek+PidStruct->Ki*PidStruct->I_item);
	}
	
	
	if ((PidStruct->ek<PidStruct->zerolim)&&(PidStruct->ek>-PidStruct->zerolim))
	{//误差达到死区
		PidStruct->Uk = 0;
	}
	if (PidStruct->Uk > PidStruct->Umax)
		PidStruct->Uk = PidStruct->Umax;
	if (PidStruct->Uk <-PidStruct->Umax)
		PidStruct->Uk = -PidStruct->Umax;

	PidStruct->deltaEk_1=PidStruct->deltaEk;
	PidStruct->ek_1 = PidStruct->ek;
	PidStruct->Uk_1 = PidStruct->Uk;
	return PidStruct->Uk;
}
