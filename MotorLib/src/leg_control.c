#include "leg_control.h"
#include "arm_math.h"
//#include "Parallel_Leg.h"
#define pi 3.1415926 

LegTimeTypedef 			 LegTimer;

uint8_t OnlineInitFinishFlag = 0;
float MotorPrePos = 0, MotorNowPos = 0;
uint8_t	ElongFinishFlag = 0;
uint8_t UpstairStart=0;
void LegDisable()
{
	Motor_1.State = IDLE;
	Motor_2.State = IDLE;
	Motor_3.State = IDLE;
	Motor_4.State = IDLE;
	Motor_5.State = IDLE;
	Motor_6.State = IDLE;
	Motor_7.State = IDLE;
	Motor_8.State = IDLE;
}

void MotorPosExpected(float pos0,float pos1,float pos2,float pos3,float pos4,float pos5,float pos6,float pos7){
	Motor_1.State = PIDPOSITION;
	Motor_2.State = PIDPOSITION;
	Motor_3.State = PIDPOSITION;
	Motor_4.State = PIDPOSITION;
	Motor_5.State = PIDPOSITION;
	Motor_6.State = PIDPOSITION;
	Motor_7.State = PIDPOSITION;
	Motor_8.State = PIDPOSITION;
	Motor_1.PositionExpected = pos6;
 	Motor_2.PositionExpected = pos7;
	Motor_3.PositionExpected = pos0;
	Motor_4.PositionExpected = pos1;
	Motor_5.PositionExpected = -pos4;
	Motor_6.PositionExpected = -pos5;
	Motor_7.PositionExpected = -pos2;
	Motor_8.PositionExpected = -pos3;			//注意：67014523对应电调ID12345678
}

void MotorSpeedExpected(float Spe0,float Spe1,float Spe2,float Spe3,float Spe4,float Spe5,float Spe6,float Spe7){
	Motor_1.State = PIDSPEED;
	Motor_2.State = PIDSPEED;
	Motor_3.State = PIDSPEED;
	Motor_4.State = PIDSPEED;
	Motor_5.State = PIDSPEED;
	Motor_6.State = PIDSPEED;
	Motor_7.State = PIDSPEED;
	Motor_8.State = PIDSPEED;
	Motor_1.SpeedExpected = Spe6;
 	Motor_2.SpeedExpected = Spe7;
	Motor_3.SpeedExpected = Spe0;
	Motor_4.SpeedExpected = Spe1;
	Motor_5.SpeedExpected = Spe4;
	Motor_6.SpeedExpected = Spe5;
	Motor_7.SpeedExpected = Spe2;
	Motor_8.SpeedExpected = Spe3;			//注意：67014523对应电调ID12345678
}

void StandingHold(){
	MotorPositionPid1.Kp = 3.0;MotorPositionPid2.Kp = 3.0;MotorPositionPid3.Kp = 3.0;MotorPositionPid4.Kp = 3.0;
			MotorPositionPid5.Kp = 3.0;MotorPositionPid6.Kp = 3.0;MotorPositionPid7.Kp = 3.0;MotorPositionPid8.Kp = 3.0;
	MotorPosExpected(MotorStandingPos[0],MotorStandingPos[1],MotorStandingPos[2],MotorStandingPos[3],
									MotorStandingPos[4],MotorStandingPos[5],MotorStandingPos[6],MotorStandingPos[7]);
}
void Stair_First_Hold(){
  MotorPositionPid1.Kp = 3.0;MotorPositionPid2.Kp = 3.0;MotorPositionPid3.Kp = 3.0;MotorPositionPid4.Kp = 3.0;
			MotorPositionPid5.Kp = 3.0;MotorPositionPid6.Kp = 3.0;MotorPositionPid7.Kp = 3.0;MotorPositionPid8.Kp = 3.0;
	MotorPosExpected(StairFirstPos[0],StairFirstPos[1],StairFirstPos[2],StairFirstPos[3],
									StairFirstPos[4],StairFirstPos[5],StairFirstPos[6],StairFirstPos[7]);
}
void Stair_Second_Hold(){
  MotorPositionPid1.Kp = 3.0;MotorPositionPid2.Kp = 3.0;MotorPositionPid3.Kp = 3.0;MotorPositionPid4.Kp = 3.0;
			MotorPositionPid5.Kp = 3.0;MotorPositionPid6.Kp = 3.0;MotorPositionPid7.Kp = 3.0;MotorPositionPid8.Kp = 3.0;
	MotorPosExpected(StairSecondPos[0],StairSecondPos[1],StairSecondPos[2],StairSecondPos[3],
									StairSecondPos[4],StairSecondPos[5],StairSecondPos[6],StairSecondPos[7]);
}
//void SlopeHold(){
//	MotorPositionPid1.Kp = 3.0;MotorPositionPid2.Kp = 3.0;MotorPositionPid3.Kp = 3.0;MotorPositionPid4.Kp = 3.0;
//			MotorPositionPid5.Kp = 3.0;MotorPositionPid6.Kp = 3.0;MotorPositionPid7.Kp = 3.0;MotorPositionPid8.Kp = 3.0;
//	MotorPosExpected(SlopeStanding[0],SlopeStanding[1],SlopeStanding[2],SlopeStanding[3],
//									SlopeStanding[4],SlopeStanding[5],SlopeStanding[6],SlopeStanding[7]);
//}

void StartPosControl(){
//	if(Gait)	
		Trot_Straight();
//	else	Walk_Straight();
}
void Upstair_Fix(){
		OS_ERR err;
	//先调整起步姿态
	if(!StartPosFlag){
		float TargetPos[8] = {SlopeUpStanding[0], SlopeUpStanding[1], SlopeUpStanding[2], SlopeUpStanding[3], 
													SlopeUpStanding[4], SlopeUpStanding[5], SlopeUpStanding[6], SlopeUpStanding[7]};
		if(ArrivePos(TargetPos, 10)){
			StartPosFlag = 1;
			MotorPositionPid1.Kp = 3.0;MotorPositionPid2.Kp = 3.0;MotorPositionPid3.Kp = 3.0;MotorPositionPid4.Kp = 3.0;
			MotorPositionPid5.Kp = 3.0;MotorPositionPid6.Kp = 3.0;MotorPositionPid7.Kp = 3.0;MotorPositionPid8.Kp = 3.0;
			LegTimerInit();
			//MyDelayms(2000);
		}
		else	MotorPosExpected(SlopeUpStanding[0], SlopeUpStanding[1], SlopeUpStanding[2], SlopeUpStanding[3], 
													SlopeUpStanding[4], SlopeUpStanding[5], SlopeUpStanding[6], SlopeUpStanding[7]);
	}
	
	else if(StartPosFlag){	//起步姿态调整完毕
		LegTimer.NowTime = OSTimeGet(&err);	//此刻时间
		if(LegTimer.NowTime - LegTimer.PreTime >= UpStairTimeInterval){
			LegTimer.PreTime = LegTimer.NowTime;	//更新时间
			MotorPosExpected(MotorUpstair_Fix[0][CurrentPoint],MotorUpstair_Fix[1][CurrentPoint],MotorUpstair_Fix[2][CurrentPoint],MotorUpstair_Fix[3][CurrentPoint],
												MotorUpstair_Fix[4][CurrentPoint],MotorUpstair_Fix[5][CurrentPoint],MotorUpstair_Fix[6][CurrentPoint],MotorUpstair_Fix[7][CurrentPoint]);
			CurrentPoint++;
		}
		if(CurrentPoint == Trot_LENGTH){	//一个周期的最后一个点发送完毕
			CurrentPoint = 0;
		}
	}	
}
void UpSlopeControl()
{
   Slope_Up();
}
void DownSlopeControl()
{
   Slope_Down();
}
bool ArrivePos(float *ArriveGoalPos, int DAngle){
	if(fabs(Motor_1.PositionMeasure - ArriveGoalPos[6]	) < DAngle &&
			fabs(Motor_2.PositionMeasure - ArriveGoalPos[7] )	< DAngle &&
			fabs(Motor_3.PositionMeasure - ArriveGoalPos[0]	) < DAngle && 
			fabs(Motor_4.PositionMeasure - ArriveGoalPos[1] ) < DAngle &&
			fabs(-Motor_5.PositionMeasure - ArriveGoalPos[4]	) < DAngle && 
			fabs(-Motor_6.PositionMeasure - ArriveGoalPos[5] ) < DAngle &&
			fabs(-Motor_7.PositionMeasure - ArriveGoalPos[2]	) < DAngle && 
			fabs(-Motor_8.PositionMeasure - ArriveGoalPos[3] ) < DAngle)
				return 1;
		else
				return 0;
}

void UpSlope_Fh_Control(){
	OS_ERR err;
	//先调整起步姿态
	if(!StartPosFlag){
		float TargetPos[8] = {Motor_Fh_Slope[0][0], Motor_Fh_Slope[1][0], Motor_Fh_Slope[2][0], Motor_Fh_Slope[3][0], 
													Motor_Fh_Slope[4][0], Motor_Fh_Slope[5][0], Motor_Fh_Slope[6][0], Motor_Fh_Slope[7][0]};
		if(ArrivePos(TargetPos, 10)){
			StartPosFlag = 1;
			MotorPositionPid1.Kp = 6.0;MotorPositionPid2.Kp = 6.0;MotorPositionPid3.Kp = 6.0;MotorPositionPid4.Kp = 6.0;
			MotorPositionPid5.Kp = 6.0;MotorPositionPid6.Kp = 6.0;MotorPositionPid7.Kp = 6.0;MotorPositionPid8.Kp = 6.0;
			LegTimerInit();
			//MyDelayms(2000);
		}
		else	MotorPosExpected(Motor_Fh_Slope[0][0],Motor_Fh_Slope[1][0],Motor_Fh_Slope[2][0],Motor_Fh_Slope[3][0],
														Motor_Fh_Slope[4][0],Motor_Fh_Slope[5][0],Motor_Fh_Slope[6][0],Motor_Fh_Slope[7][0]);
	}
	
	else if(StartPosFlag){	//起步姿态调整完毕
		LegTimer.NowTime = OSTimeGet(&err);	//此刻时间
		if(LegTimer.NowTime - LegTimer.PreTime >= FhTimeInterval){
			LegTimer.PreTime = LegTimer.NowTime;	//更新时间
			MotorPosExpected(Motor_Fh_Slope[0][CurrentPoint],Motor_Fh_Slope[1][CurrentPoint],Motor_Fh_Slope[2][CurrentPoint],Motor_Fh_Slope[3][CurrentPoint],
												Motor_Fh_Slope[4][CurrentPoint],Motor_Fh_Slope[5][CurrentPoint],Motor_Fh_Slope[6][CurrentPoint],Motor_Fh_Slope[7][CurrentPoint]);
			CurrentPoint++;
		}
		if(CurrentPoint == Fh_Slope_LENGTH){	//一个周期的最后一个点发送完毕
			CurrentPoint = 0;
		}
	}	
}
void Slope_Up(){
	OS_ERR err;
		const int TempLength = 31;
	float MotorTemp[8][TempLength] = {
{	84.123,87.055,90.003,92.97,95.956,98.963,101.99,105.05,108.13,111.23,114.36,115.25,115.99,116.59,117.05,117.37,117.56,117.63,117.58,117.41,117.14,119.31,121.44,123.53,125.58,127.58,129.54,131.44,133.28,135.07,136.8	}	,
{	65.44,66.289,67.12,67.934,68.732,69.518,70.291,71.054,71.808,72.555,73.294,70.841,68.302,65.687,63.006,60.268,57.484,54.662,51.809,48.933,46.04,45.721,45.348,44.917,44.429,43.881,43.272,42.6,41.865,41.064,40.198	}	,
{	84.123,87.055,90.003,92.97,95.956,98.963,101.99,105.05,108.13,111.23,114.36,117.43,125.62,136.58,145.38,139.33,116.76,94.649,79.202,70.788,68.358,71.778,75.287,78.899,82.631,86.503,90.537,94.763,99.217,103.95,109.02	}	,
{	65.44,66.289,67.12,67.934,68.732,69.518,70.291,71.054,71.808,72.555,73.294,76.1,85.816,104.43,130.21,142.38,130.93,117.01,106.31,99.389,96.947,98.969,101.07,103.27,105.59,108.07,110.73,113.62,116.82,120.39,124.46	}	,
{	84.123,87.055,90.003,92.97,95.956,98.963,101.99,105.05,108.13,111.23,114.36,115.25,115.99,116.59,117.05,117.37,117.56,117.63,117.58,117.41,117.14,119.31,121.44,123.53,125.58,127.58,129.54,131.44,133.28,135.07,136.8	}	,
{	65.44,66.289,67.12,67.934,68.732,69.518,70.291,71.054,71.808,72.555,73.294,70.841,68.302,65.687,63.006,60.268,57.484,54.662,51.809,48.933,46.04,45.721,45.348,44.917,44.429,43.881,43.272,42.6,41.865,41.064,40.198	}	,
{	84.123,87.055,90.003,92.97,95.956,98.963,101.99,105.05,108.13,111.23,114.36,117.43,125.62,136.58,145.38,139.33,116.76,94.649,79.202,70.788,68.358,71.778,75.287,78.899,82.631,86.503,90.537,94.763,99.217,103.95,109.02	}	,
{	65.44,66.289,67.12,67.934,68.732,69.518,70.291,71.054,71.808,72.555,73.294,76.1,85.816,104.43,130.21,142.38,130.93,117.01,106.31,99.389,96.947,98.969,101.07,103.27,105.59,108.07,110.73,113.62,116.82,120.39,124.46	}	,
};
	
	//先调整起步姿态
	if(!StartPosFlag){

		LegTimer.NowTime = OSTimeGet(&err);	//此刻时间
		if(LegTimer.NowTime - LegTimer.PreTime >= 100){
			LegTimer.PreTime = LegTimer.NowTime;	//更新时间
			MotorPosExpected(MotorTemp[0][CurrentPoint],MotorTemp[1][CurrentPoint],MotorTemp[2][CurrentPoint],MotorTemp[3][CurrentPoint],
												MotorTemp[4][CurrentPoint],MotorTemp[5][CurrentPoint],MotorTemp[6][CurrentPoint],MotorTemp[7][CurrentPoint]);
			CurrentPoint++;
		}
		if(CurrentPoint == TempLength){
			StartPosFlag = 1;
			MotorPositionPid1.Kp = 3.0;MotorPositionPid2.Kp = 3.0;MotorPositionPid3.Kp = 3.0;MotorPositionPid4.Kp = 3.0;
			MotorPositionPid5.Kp = 3.0;MotorPositionPid6.Kp = 3.0;MotorPositionPid7.Kp = 3.0;MotorPositionPid8.Kp = 3.0;
			LegTimerInit();
			CurrentPoint = 0;
			//MyDelayms(800);
		}
//	if(!StartPosFlag){
		
//		float TargetPos[8] = {SlopeUpStanding[0], SlopeUpStanding[1], SlopeUpStanding[2], SlopeUpStanding[3], 
//													SlopeUpStanding[4], SlopeUpStanding[5], SlopeUpStanding[6], SlopeUpStanding[7]};

//		if(ArrivePos(TargetPos, 10)){
//			StartPosFlag = 1;
//			MotorPositionPid1.Kp = 6.0;MotorPositionPid2.Kp = 6.0;MotorPositionPid3.Kp = 6.0;MotorPositionPid4.Kp = 6.0;
//			MotorPositionPid5.Kp = 6.0;MotorPositionPid6.Kp = 6.0;MotorPositionPid7.Kp = 6.0;MotorPositionPid8.Kp = 6.0;
//			LegTimerInit();
//			//MyDelayms(2000);
//		}
//		else	MotorPosExpected(SlopeUpStanding[0], SlopeUpStanding[1], SlopeUpStanding[2], SlopeUpStanding[3], 
//													SlopeUpStanding[4], SlopeUpStanding[5], SlopeUpStanding[6], SlopeUpStanding[7]);
//	}
	}
	else if(StartPosFlag){	//起步姿态调整完毕
		LegTimer.NowTime = OSTimeGet(&err);	//此刻时间
		if(LegTimer.NowTime - LegTimer.PreTime >= SlopUpTimeInterval){
			LegTimer.PreTime = LegTimer.NowTime;	//更新时间
			MotorPosExpected(MotorUpSlope[0][CurrentPoint],MotorUpSlope[1][CurrentPoint],MotorUpSlope[2][CurrentPoint],MotorUpSlope[3][CurrentPoint],
												MotorUpSlope[4][CurrentPoint],MotorUpSlope[5][CurrentPoint],MotorUpSlope[6][CurrentPoint],MotorUpSlope[7][CurrentPoint]);
			CurrentPoint++;
		}
		if(CurrentPoint == Trot_LENGTH){	//一个周期的最后一个点发送完毕
			CurrentPoint = 0;
		}
	}	
}

void Slope_Down(){
	OS_ERR err;
	//先调整起步姿态
	if(!StartPosFlag){
		float TargetPos[8] = {SlopeDownStanding[0], SlopeDownStanding[1], SlopeDownStanding[2], SlopeDownStanding[3], 
													SlopeDownStanding[4], SlopeDownStanding[5], SlopeDownStanding[6], SlopeDownStanding[7]};
		if(ArrivePos(TargetPos, 10)){
			StartPosFlag = 1;
			MotorPositionPid1.Kp = 3.0;MotorPositionPid2.Kp = 3.0;MotorPositionPid3.Kp = 3.0;MotorPositionPid4.Kp = 3.0;
			MotorPositionPid5.Kp = 3.0;MotorPositionPid6.Kp = 3.0;MotorPositionPid7.Kp = 3.0;MotorPositionPid8.Kp = 3.0;
			LegTimerInit();
			//MyDelayms(2000);
		}
		else	MotorPosExpected(SlopeDownStanding[0], SlopeDownStanding[1], SlopeDownStanding[2], SlopeDownStanding[3], 
													SlopeDownStanding[4], SlopeDownStanding[5], SlopeDownStanding[6], SlopeDownStanding[7]);
	}
	
	else if(StartPosFlag){	//起步姿态调整完毕
		LegTimer.NowTime = OSTimeGet(&err);	//此刻时间
		if(LegTimer.NowTime - LegTimer.PreTime >= SlopDownTimeInterval){
			LegTimer.PreTime = LegTimer.NowTime;	//更新时间
			MotorPosExpected(MotorDownSlope[0][CurrentPoint],MotorDownSlope[1][CurrentPoint],MotorDownSlope[2][CurrentPoint],MotorDownSlope[3][CurrentPoint],
												MotorDownSlope[4][CurrentPoint],MotorDownSlope[5][CurrentPoint],MotorDownSlope[6][CurrentPoint],MotorDownSlope[7][CurrentPoint]);
			CurrentPoint++;
		}
		if(CurrentPoint == Trot_LENGTH){	//一个周期的最后一个点发送完毕
			CurrentPoint = 0;
		}
	}	
}
	
void Trot_Straight(){
	OS_ERR err;	
	
//	float TargetPos[8] = {MotorTrot[0][0], MotorTrot[1][0], MotorTrot[2][0], MotorTrot[3][0], 
//											MotorTrot[4][0], MotorTrot[5][0], MotorTrot[6][0], MotorTrot[7][0]};
	
	//摆线行至起步姿势 
	const int TempLength = 6;
	float MotorTemp[8][TempLength] = {
{	95.204,112.03,156.42,156.99,121.17,105.37	}	,
{	74.399,87.374,125.17,106.56,65.248,53.594	}	,
{	95.204,91.673,87.68,83.262,78.462,73.326	}	,
{	74.399,77.484,80.149,82.391,84.216,85.638	}	,
{	95.204,112.03,156.42,156.99,121.17,105.37	}	,
{	74.399,87.374,125.17,106.56,65.248,53.594	}	,
{	95.204,91.673,87.68,83.262,78.462,73.326	}	,
{	74.399,77.484,80.149,82.391,84.216,85.638	}	,
	};
	
	//先调整起步姿态
	if(!StartPosFlag){

		LegTimer.NowTime = OSTimeGet(&err);	//此刻时间
		if(LegTimer.NowTime - LegTimer.PreTime >= 100){
			LegTimer.PreTime = LegTimer.NowTime;	//更新时间
			MotorPosExpected(MotorTemp[0][CurrentPoint],MotorTemp[1][CurrentPoint],MotorTemp[2][CurrentPoint],MotorTemp[3][CurrentPoint],
												MotorTemp[4][CurrentPoint],MotorTemp[5][CurrentPoint],MotorTemp[6][CurrentPoint],MotorTemp[7][CurrentPoint]);
			CurrentPoint++;
		}
		if(CurrentPoint == TempLength){
			StartPosFlag = 1;
			MotorPositionPid1.Kp = 3.0;MotorPositionPid2.Kp = 3.0;MotorPositionPid3.Kp = 3.0;MotorPositionPid4.Kp = 3.0;
			MotorPositionPid5.Kp = 3.0;MotorPositionPid6.Kp = 3.0;MotorPositionPid7.Kp = 3.0;MotorPositionPid8.Kp = 3.0;
			LegTimerInit();
			CurrentPoint = 0;
			MyDelayms(500);
		}
//		if(ArrivePos(TargetPos, 5)){
//			StartPosFlag = 1;
//			MotorPositionPid1.Kp = 3.0;MotorPositionPid2.Kp = 3.0;MotorPositionPid3.Kp = 3.0;MotorPositionPid4.Kp = 3.0;
//			MotorPositionPid5.Kp = 3.0;MotorPositionPid6.Kp = 3.0;MotorPositionPid7.Kp = 3.0;MotorPositionPid8.Kp = 3.0;
//			LegTimerInit();
//			CurrentPoint = 0;
//			MyDelayms(1000);
//		}
//		else	MotorPosExpected(MotorTrot[0][0],MotorTrot[1][0],MotorTrot[2][0],MotorTrot[3][0],
//														MotorTrot[4][0],MotorTrot[5][0],MotorTrot[6][0],MotorTrot[7][0]);
	}
	
	else if(StartPosFlag){	//起步姿态调整完毕
		LegTimer.NowTime = OSTimeGet(&err);	//此刻时间
		if(LegTimer.NowTime - LegTimer.PreTime >= TrotTimeInterval){
			LegTimer.PreTime = LegTimer.NowTime;	//更新时间
			MotorPosExpected(MotorTrot[0][CurrentPoint],MotorTrot[1][CurrentPoint],MotorTrot[2][CurrentPoint],MotorTrot[3][CurrentPoint],
												MotorTrot[4][CurrentPoint],MotorTrot[5][CurrentPoint],MotorTrot[6][CurrentPoint],MotorTrot[7][CurrentPoint]);
			CurrentPoint++;
		}
		if(CurrentPoint == Trot_LENGTH){	//一个周期的最后一个点发送完毕
			CurrentPoint = 0;
		}
	}	
}
	
void Walk_Straight(){
	OS_ERR err;
	//先调整起步姿态
	if(!StartPosFlag){
		float TargetPos[8] = {MotorWalk[0][0], MotorWalk[1][0], MotorWalk[2][0], MotorWalk[3][0], 
													MotorWalk[4][0], MotorWalk[5][0], MotorWalk[6][0], MotorWalk[7][0]};
		if(ArrivePos(TargetPos, 10)){
			StartPosFlag = 1;
			MotorPositionPid1.Kp = 4.0;MotorPositionPid2.Kp = 4.0;MotorPositionPid3.Kp = 4.0;MotorPositionPid4.Kp = 4.0;
			MotorPositionPid5.Kp = 4.0;MotorPositionPid6.Kp = 4.0;MotorPositionPid7.Kp = 4.0;MotorPositionPid8.Kp = 4.0;
			LegTimerInit();
			//MyDelayms(2000);
		}
		else	MotorPosExpected(MotorWalk[0][0],MotorWalk[1][0],MotorWalk[2][0],MotorWalk[3][0],
														MotorWalk[4][0],MotorWalk[5][0],MotorWalk[6][0],MotorWalk[7][0]);
	}
	
	else if(StartPosFlag){	//起步姿态调整完毕
		LegTimer.NowTime = OSTimeGet(&err);	//此刻时间
		if(LegTimer.NowTime - LegTimer.PreTime >= WalkTimeInterval){
			LegTimer.PreTime = LegTimer.NowTime;	//更新时间
			MotorPosExpected(MotorWalk[0][CurrentPoint],MotorWalk[1][CurrentPoint],MotorWalk[2][CurrentPoint],MotorWalk[3][CurrentPoint],
												MotorWalk[4][CurrentPoint],MotorWalk[5][CurrentPoint],MotorWalk[6][CurrentPoint],MotorWalk[7][CurrentPoint]);
			CurrentPoint++;
		}
		if(CurrentPoint == Walk_LENGTH){	//一个周期的最后一个点发送完毕
			CurrentPoint = 0;
		}
	}
}

/**************************************************/

/**
  * @brief  并联腿时钟初始化
  * @param  无
  * @retval 无
*/
void LegTimerInit(void)
{
	OS_ERR err;
	LegTimer.StartTime = OSTimeGet(&err);
	LegTimer.PreTime = OSTimeGet(&err);
}


bool OnlineInitFinish(){
  MotorNowPos = Motor_1.PositionMeasure;
	if( fabs(MotorPrePos - MotorNowPos) < 0.1 )
	{OnlineInitFinishFlag ++;}
	if(OnlineInitFinishFlag > 200){
		Motor_1.PositionExpected = OnlineInitAngle;
		Motor_2.PositionExpected = OnlineInitAngle;
		Motor_3.PositionExpected = OnlineInitAngle;
		Motor_4.PositionExpected = OnlineInitAngle;
		Motor_5.PositionExpected = -OnlineInitAngle;
		Motor_6.PositionExpected = -OnlineInitAngle;
		Motor_7.PositionExpected = -OnlineInitAngle;
		Motor_8.PositionExpected = -OnlineInitAngle;
		//ID1-4给了正位置，5-8给了负位置
		return true;
	}
	MotorPrePos = MotorNowPos;
		return false;
}

void PosCaliBration(){
	MotorSpeedExpected(10,10,-10,-10,-10,-10,10,10);	//ID1-4给了正速度，5-8给了负速度
	if(OnlineInitFinish()){
		Motor_1.PositionMeasure = OnlineInitAngle;
		Motor_2.PositionMeasure = OnlineInitAngle;
		Motor_3.PositionMeasure = OnlineInitAngle;
		Motor_4.PositionMeasure = OnlineInitAngle;
		Motor_5.PositionMeasure = -OnlineInitAngle;
		Motor_6.PositionMeasure = -OnlineInitAngle;
		Motor_7.PositionMeasure = -OnlineInitAngle;
		Motor_8.PositionMeasure = -OnlineInitAngle;
		Motor_1.State = PIDPOSITION;
		Motor_2.State = PIDPOSITION;
	  Motor_3.State = PIDPOSITION;
		Motor_4.State = PIDPOSITION;
	  Motor_5.State = PIDPOSITION;
		Motor_6.State = PIDPOSITION;
	  Motor_7.State = PIDPOSITION;
		Motor_8.State = PIDPOSITION;
		MyDelayms(500);
		leg_state = STAND;
	}
}

void TITA_Straight(void){
		OS_ERR err;
	//先调整起步姿态
	if(!StartPosFlag){
		float TargetPos[8] = {MotorTITA[0][0], MotorTITA[1][0], MotorTITA[2][0], MotorTITA[3][0], 
													MotorTITA[4][0], MotorTITA[5][0], MotorTITA[6][0], MotorTITA[7][0]};
		if(ArrivePos(TargetPos, 5)){
			StartPosFlag = 1;
			MotorPositionPid1.Kp = 3.0;MotorPositionPid2.Kp = 3.0;MotorPositionPid3.Kp = 3.0;MotorPositionPid4.Kp = 3.0;
			MotorPositionPid5.Kp = 3.0;MotorPositionPid6.Kp = 3.0;MotorPositionPid7.Kp = 3.0;MotorPositionPid8.Kp = 3.0;
			LegTimerInit();
			//MyDelayms(2000);
		}
		else	MotorPosExpected(MotorTITA[0][0],MotorTITA[1][0],MotorTITA[2][0],MotorTITA[3][0],
														MotorTITA[4][0],MotorTITA[5][0],MotorTITA[6][0],MotorTITA[7][0]);
	}
	
	else if(StartPosFlag){	//起步姿态调整完毕
		LegTimer.NowTime = OSTimeGet(&err);	//此刻时间
		if(LegTimer.NowTime - LegTimer.PreTime >= TITATimeInterval){
			LegTimer.PreTime = LegTimer.NowTime;	//更新时间
			MotorPosExpected(MotorTITA[0][CurrentPoint],MotorTITA[1][CurrentPoint],MotorTITA[2][CurrentPoint],MotorTITA[3][CurrentPoint],
												MotorTITA[4][CurrentPoint],MotorTITA[5][CurrentPoint],MotorTITA[6][CurrentPoint],MotorTITA[7][CurrentPoint]);
			CurrentPoint++;
		}
		if(CurrentPoint == Trot_LENGTH){	//一个周期的最后一个点发送完毕
			CurrentPoint = 0;
		}
	}	
}

void Task_FlyJump(void){
	OS_ERR err;
	float TargetPos[8] = {MotorFlyJump[0][9], MotorFlyJump[1][9], MotorFlyJump[2][9], MotorFlyJump[3][9], 
												MotorFlyJump[4][9], MotorFlyJump[5][9], MotorFlyJump[6][9], MotorFlyJump[7][9]};
	MotorPositionPid1.Kp = 12.0;MotorPositionPid2.Kp = 12.0;MotorPositionPid3.Kp = 12.0;MotorPositionPid4.Kp = 12.0;
		MotorPositionPid5.Kp = 12.0;MotorPositionPid6.Kp = 12.0;MotorPositionPid7.Kp = 12.0;MotorPositionPid8.Kp = 12.0;
	//保证腿完全伸长
	if(!ElongFinishFlag){
		if(ArrivePos(TargetPos, 10)){
			ElongFinishFlag = 1;
			LegTimerInit();
			CurrentPoint = 10;
		}
		else	MotorPosExpected(MotorFlyJump[0][9],MotorFlyJump[1][9],MotorFlyJump[2][9],MotorFlyJump[3][9],
														MotorFlyJump[4][9],MotorFlyJump[5][9],MotorFlyJump[6][9],MotorFlyJump[7][9]);
		
	}
	else if(ElongFinishFlag){	
		LegTimer.NowTime = OSTimeGet(&err);	//此刻时间
		if(LegTimer.NowTime - LegTimer.PreTime >= JumpTimeInterval){
			LegTimer.PreTime = LegTimer.NowTime;	//更新时间
			MotorPosExpected(MotorFlyJump[0][CurrentPoint],MotorFlyJump[1][CurrentPoint],MotorFlyJump[2][CurrentPoint],MotorFlyJump[3][CurrentPoint],
												MotorFlyJump[4][CurrentPoint],MotorFlyJump[5][CurrentPoint],MotorFlyJump[6][CurrentPoint],MotorFlyJump[7][CurrentPoint]);
			CurrentPoint++;
		}
		if(CurrentPoint == FlyJump_LENGTH){	//一个周期的最后一个点发送完毕
					MotorPositionPid1.Kp = 3.0;MotorPositionPid2.Kp = 3.0;MotorPositionPid3.Kp = 3.0;MotorPositionPid4.Kp = 3.0;
      	MotorPositionPid5.Kp = 3.0;MotorPositionPid6.Kp = 3.0;MotorPositionPid7.Kp = 3.0;MotorPositionPid8.Kp = 3.0;
			
				Motor_1.PositionExpected = 300.0;
 	Motor_2.PositionExpected = -60.0;
	Motor_3.PositionExpected = 220.0;
	Motor_4.PositionExpected = 65.0;
	Motor_5.PositionExpected = -300.0;
	Motor_6.PositionExpected = 60.0;
	Motor_7.PositionExpected = -220.0;
	Motor_8.PositionExpected = -70.0;	
	float TargetPos1[8] = { Motor_1.PositionExpected,Motor_2.PositionExpected,Motor_3.PositionExpected,Motor_4.PositionExpected,
	Motor_5.PositionExpected,Motor_6.PositionExpected,Motor_7.PositionExpected,Motor_8.PositionExpected};
//	while(1){
//				if(ArrivePos(TargetPos1, 30)){	
MyDelayms(100);	
				Motor_1.PositionExpected = 205.0;
				Motor_2.PositionExpected = -165.0;
	      Motor_3.PositionExpected = -150.0;
	      Motor_4.PositionExpected = 200.0;
				Motor_5.PositionExpected = -205.0;
				Motor_6.PositionExpected = 165.0;
	      Motor_7.PositionExpected = 150.0;
	      Motor_8.PositionExpected = -200.0;	
//					break;
//						}	
//				}				
			MyDelayms(500);
			ElongFinishFlag = 0;
			JumpFlag = 0;
			StartPosFlag = 0;
//		MotorPositionPid1.Kp = 3.0;MotorPositionPid2.Kp = 3.0;MotorPositionPid3.Kp = 3.0;MotorPositionPid4.Kp = 3.0;
//		MotorPositionPid5.Kp = 3.0;MotorPositionPid6.Kp = 3.0;MotorPositionPid7.Kp = 3.0;MotorPositionPid8.Kp = 3.0;
     while(1){};
//			leg_state = STAND;
		}
	}
	
}
void Task_Jump_First(void)
{
OS_ERR err;
	float TargetPos[8] = {MotorJump_First[0][9], MotorJump_First[1][9], MotorJump_First[2][9], MotorJump_First[3][9], 
												MotorJump_First[4][9], MotorJump_First[5][9], MotorJump_First[6][9], MotorJump_First[7][9]};
	MotorPositionPid1.Kp = 5.5;MotorPositionPid2.Kp = 5.5;MotorPositionPid3.Kp = 5.5;MotorPositionPid4.Kp = 5.5;
		MotorPositionPid5.Kp = 5.5;MotorPositionPid6.Kp = 5.5;MotorPositionPid7.Kp = 5.5;MotorPositionPid8.Kp = 5.5;
	//保证腿完全伸长
	if(!ElongFinishFlag){
		if(ArrivePos(TargetPos, 10)){
			ElongFinishFlag = 1;
			LegTimerInit();
			CurrentPoint = 10;
		}
		else	MotorPosExpected(MotorJump_First[0][9],MotorJump_First[1][9],MotorJump_First[2][9],MotorJump_First[3][9],
														MotorJump_First[4][9],MotorJump_First[5][9],MotorJump_First[6][9],MotorJump_First[7][9]);
		
	}
	else if(ElongFinishFlag){	
		LegTimer.NowTime = OSTimeGet(&err);	//此刻时间
		if(LegTimer.NowTime - LegTimer.PreTime >= JumpTimeInterval){
			LegTimer.PreTime = LegTimer.NowTime;	//更新时间
			MotorPosExpected(MotorJump_First[0][CurrentPoint],MotorJump_First[1][CurrentPoint],MotorJump_First[2][CurrentPoint],MotorJump_First[3][CurrentPoint],
												MotorJump_First[4][CurrentPoint],MotorJump_First[5][CurrentPoint],MotorJump_First[6][CurrentPoint],MotorJump_First[7][CurrentPoint]);
			CurrentPoint++;
		}
		if(CurrentPoint == Jump_LENGTH_FIRST){	//一个周期的最后一个点发送完毕
			MyDelayms(500);
			ElongFinishFlag = 0;
			JumpFlag = 0;
			StartPosFlag = 0;
			leg_state = STAIR_FIRST_HOLD;
		}
  }
}


void Task_NormalJump(void)
{
OS_ERR err;
	float TargetPos[8] = {MotorJump[0][9], MotorJump[1][9], MotorJump[2][9], MotorJump[3][9], 
												MotorJump[4][9], MotorJump[5][9], MotorJump[6][9], MotorJump[7][9]};
	MotorPositionPid1.Kp = 5.5;MotorPositionPid2.Kp = 5.5;MotorPositionPid3.Kp = 5.5;MotorPositionPid4.Kp = 5.5;
		MotorPositionPid5.Kp = 5.5;MotorPositionPid6.Kp = 5.5;MotorPositionPid7.Kp = 5.5;MotorPositionPid8.Kp = 5.5;
	//保证腿完全伸长
	if(!ElongFinishFlag){
		if(ArrivePos(TargetPos, 10)){
			ElongFinishFlag = 1;
			LegTimerInit();
			CurrentPoint = 10;
		}
		else	MotorPosExpected(MotorJump[0][9],MotorJump[1][9],MotorJump[2][9],MotorJump[3][9],
														MotorJump[4][9],MotorJump[5][9],MotorJump[6][9],MotorJump[7][9]);
		
	}
	else if(ElongFinishFlag){	
		LegTimer.NowTime = OSTimeGet(&err);	//此刻时间
		if(LegTimer.NowTime - LegTimer.PreTime >= JumpTimeInterval){
			LegTimer.PreTime = LegTimer.NowTime;	//更新时间
			MotorPosExpected(MotorJump[0][CurrentPoint],MotorJump[1][CurrentPoint],MotorJump[2][CurrentPoint],MotorJump[3][CurrentPoint],
												MotorJump[4][CurrentPoint],MotorJump[5][CurrentPoint],MotorJump[6][CurrentPoint],MotorJump[7][CurrentPoint]);
			CurrentPoint++;
		}
		if(CurrentPoint == Jump_LENGTH){	//一个周期的最后一个点发送完毕
			MyDelayms(500);
			ElongFinishFlag = 0;
			JumpFlag = 0;
			StartPosFlag = 0;
			leg_state = STAND;
		}
  }
}


void Jump_Pre(void){
	OS_ERR err;
	LegTimer.NowTime = OSTimeGet(&err);	//此刻时间
	if(LegTimer.NowTime - LegTimer.PreTime >= JumpTimeInterval){
		LegTimer.PreTime = LegTimer.NowTime;	//更新时间
		MotorPosExpected(MotorJump[0][CurrentPoint],MotorJump[1][CurrentPoint],MotorJump[2][CurrentPoint],MotorJump[3][CurrentPoint],
											MotorJump[4][CurrentPoint],MotorJump[5][CurrentPoint],MotorJump[6][CurrentPoint],MotorJump[7][CurrentPoint]);
		CurrentPoint++;
	}
	if(CurrentPoint >= 6){	//点5已经发送
		float TargetPos[8] = {MotorJump[0][5], MotorJump[1][5], MotorJump[2][5], MotorJump[3][5], 
												MotorJump[4][5], MotorJump[5][5], MotorJump[6][5], MotorJump[7][5]};
		MotorPosExpected(MotorJump[0][5],MotorJump[1][5],MotorJump[2][5],MotorJump[3][5],
									 MotorJump[4][5],MotorJump[5][5],MotorJump[6][5],MotorJump[7][5]);
//		if(ArrivePos(TargetPos, 10)){
			StartPosFlag = 1;
			CurrentPoint=6;
			MyDelayms(1000);
//		}
	}
}
void Task_LittleJump(void)
{
OS_ERR err;
	float TargetPos[8] = {MotorLittleJump[0][9], MotorLittleJump[1][9], MotorLittleJump[2][9], MotorLittleJump[3][9], 
												MotorLittleJump[4][9], MotorLittleJump[5][9], MotorLittleJump[6][9], MotorLittleJump[7][9]};
	MotorPositionPid1.Kp = 5.5;MotorPositionPid2.Kp = 5.5;MotorPositionPid3.Kp = 5.5;MotorPositionPid4.Kp = 5.5;
		MotorPositionPid5.Kp = 5.5;MotorPositionPid6.Kp = 5.5;MotorPositionPid7.Kp = 5.5;MotorPositionPid8.Kp = 5.5;
	//保证腿完全伸长
	if(!ElongFinishFlag){
		if(ArrivePos(TargetPos, 10)){
			ElongFinishFlag = 1;
			LegTimerInit();
			CurrentPoint = 10;
		}
		else	MotorPosExpected(MotorLittleJump[0][9],MotorLittleJump[1][9],MotorLittleJump[2][9],MotorLittleJump[3][9],
														MotorLittleJump[4][9],MotorLittleJump[5][9],MotorLittleJump[6][9],MotorLittleJump[7][9]);
		
	}
	else if(ElongFinishFlag){	
		LegTimer.NowTime = OSTimeGet(&err);	//此刻时间
		if(LegTimer.NowTime - LegTimer.PreTime >= JumpTimeInterval){
			LegTimer.PreTime = LegTimer.NowTime;	//更新时间
			MotorPosExpected(MotorLittleJump[0][CurrentPoint],MotorLittleJump[1][CurrentPoint],MotorLittleJump[2][CurrentPoint],MotorLittleJump[3][CurrentPoint],
												MotorLittleJump[4][CurrentPoint],MotorLittleJump[5][CurrentPoint],MotorLittleJump[6][CurrentPoint],MotorLittleJump[7][CurrentPoint]);
			CurrentPoint++;
		}
		if(CurrentPoint == LittleJump_LENGTH){	//一个周期的最后一个点发送完毕
			MyDelayms(500);
			ElongFinishFlag = 0;
			JumpFlag = 0;
			StartPosFlag = 0;
			leg_state = STAND;
		}
  }
}


void LittleJump_Pre(void){
	OS_ERR err;
	LegTimer.NowTime = OSTimeGet(&err);	//此刻时间
	if(LegTimer.NowTime - LegTimer.PreTime >= JumpTimeInterval){
		LegTimer.PreTime = LegTimer.NowTime;	//更新时间
		MotorPosExpected(MotorLittleJump[0][CurrentPoint],MotorLittleJump[1][CurrentPoint],MotorLittleJump[2][CurrentPoint],MotorLittleJump[3][CurrentPoint],
											MotorLittleJump[4][CurrentPoint],MotorLittleJump[5][CurrentPoint],MotorLittleJump[6][CurrentPoint],MotorLittleJump[7][CurrentPoint]);
		CurrentPoint++;
	}
	if(CurrentPoint >= 6){	//点5已经发送
		float TargetPos[8] = {MotorLittleJump[0][5], MotorLittleJump[1][5], MotorLittleJump[2][5], MotorLittleJump[3][5], 
												MotorLittleJump[4][5], MotorLittleJump[5][5], MotorLittleJump[6][5], MotorLittleJump[7][5]};
		MotorPosExpected(MotorLittleJump[0][5],MotorLittleJump[1][5],MotorLittleJump[2][5],MotorLittleJump[3][5],
									 MotorLittleJump[4][5],MotorLittleJump[5][5],MotorLittleJump[6][5],MotorLittleJump[7][5]);
//		if(ArrivePos(TargetPos, 10)){
			StartPosFlag = 1;
			CurrentPoint=6;
			MyDelayms(1000);
//		}
	}
}


void Jump_Pre_First(void){
	OS_ERR err;
	LegTimer.NowTime = OSTimeGet(&err);	//此刻时间
	if(LegTimer.NowTime - LegTimer.PreTime >= JumpTimeInterval){
		LegTimer.PreTime = LegTimer.NowTime;	//更新时间
		MotorPosExpected(MotorJump_First[0][CurrentPoint],MotorJump_First[1][CurrentPoint],MotorJump_First[2][CurrentPoint],MotorJump_First[3][CurrentPoint],
											MotorJump_First[4][CurrentPoint],MotorJump_First[5][CurrentPoint],MotorJump_First[6][CurrentPoint],MotorJump_First[7][CurrentPoint]);
		CurrentPoint++;
	}
	if(CurrentPoint >= 6){	//点5已经发送
		float TargetPos[8] = {MotorJump_First[0][5], MotorJump_First[1][5], MotorJump_First[2][5], MotorJump_First[3][5], 
												MotorJump_First[4][5], MotorJump_First[5][5], MotorJump_First[6][5], MotorJump_First[7][5]};
		MotorPosExpected(MotorJump_First[0][5],MotorJump_First[1][5],MotorJump_First[2][5],MotorJump_First[3][5],
									 MotorJump_First[4][5],MotorJump_First[5][5],MotorJump_First[6][5],MotorJump_First[7][5]);
//		if(ArrivePos(TargetPos, 10)){
			StartPosFlag = 1;
			CurrentPoint=6;
			MyDelayms(1000);
//		}
	}
}
void Task_Jump_Second(void)
{
OS_ERR err;
	float TargetPos[8] = {MotorJump_Second[0][9], MotorJump_Second[1][9], MotorJump_Second[2][9], MotorJump_Second[3][9], 
												MotorJump_Second[4][9], MotorJump_Second[5][9], MotorJump_Second[6][9], MotorJump_Second[7][9]};
	MotorPositionPid1.Kp = 5.5;MotorPositionPid2.Kp = 5.5;MotorPositionPid3.Kp = 5.5;MotorPositionPid4.Kp = 5.5;
		MotorPositionPid5.Kp = 5.5;MotorPositionPid6.Kp = 5.5;MotorPositionPid7.Kp = 5.5;MotorPositionPid8.Kp = 5.5;
	//保证腿完全伸长
	if(!ElongFinishFlag){
		if(ArrivePos(TargetPos, 10)){
			ElongFinishFlag = 1;
			LegTimerInit();
			CurrentPoint = 10;
		}
		else	MotorPosExpected(MotorJump_Second[0][9],MotorJump_Second[1][9],MotorJump_Second[2][9],MotorJump_Second[3][9],
														MotorJump_Second[4][9],MotorJump_Second[5][9],MotorJump_Second[6][9],MotorJump_Second[7][9]);
		
	}
	else if(ElongFinishFlag){	
		LegTimer.NowTime = OSTimeGet(&err);	//此刻时间
		if(LegTimer.NowTime - LegTimer.PreTime >= JumpTimeInterval){
			LegTimer.PreTime = LegTimer.NowTime;	//更新时间
			MotorPosExpected(MotorJump_Second[0][CurrentPoint],MotorJump_Second[1][CurrentPoint],MotorJump_Second[2][CurrentPoint],MotorJump_Second[3][CurrentPoint],
												MotorJump_Second[4][CurrentPoint],MotorJump_Second[5][CurrentPoint],MotorJump_Second[6][CurrentPoint],MotorJump_Second[7][CurrentPoint]);
			CurrentPoint++;
		}
		if(CurrentPoint == Jump_LENGTH_SECOND){	//一个周期的最后一个点发送完毕
			MyDelayms(500);
			ElongFinishFlag = 0;
			JumpFlag = 0;
			StartPosFlag = 0;
			leg_state = STAIR_FIRST_HOLD;
		}
  }
}


void Jump_Pre_Second(void){
	OS_ERR err;
	LegTimer.NowTime = OSTimeGet(&err);	//此刻时间
	if(LegTimer.NowTime - LegTimer.PreTime >= JumpTimeInterval){
		LegTimer.PreTime = LegTimer.NowTime;	//更新时间
		MotorPosExpected(MotorJump_Second[0][CurrentPoint],MotorJump_Second[1][CurrentPoint],MotorJump_Second[2][CurrentPoint],MotorJump_Second[3][CurrentPoint],
											MotorJump_Second[4][CurrentPoint],MotorJump_Second[5][CurrentPoint],MotorJump_Second[6][CurrentPoint],MotorJump_Second[7][CurrentPoint]);
		CurrentPoint++;
	}
	if(CurrentPoint >= 6){	//点5已经发送
		float TargetPos[8] = {MotorJump_Second[0][5], MotorJump_Second[1][5], MotorJump_Second[2][5], MotorJump_Second[3][5], 
												MotorJump_Second[4][5], MotorJump_Second[5][5], MotorJump_Second[6][5], MotorJump_Second[7][5]};
		MotorPosExpected(MotorJump_Second[0][5],MotorJump_Second[1][5],MotorJump_Second[2][5],MotorJump_Second[3][5],
									 MotorJump_Second[4][5],MotorJump_Second[5][5],MotorJump_Second[6][5],MotorJump_Second[7][5]);
//		if(ArrivePos(TargetPos, 10)){
			StartPosFlag = 1;
			CurrentPoint=6;
			MyDelayms(1000);
//		}
	}
}

void Task_Jump_Third(void)
{
OS_ERR err;
	float TargetPos[8] = {MotorJump_Third[0][9], MotorJump_Third[1][9], MotorJump_Third[2][9], MotorJump_Third[3][9], 
												MotorJump_Third[4][9], MotorJump_Third[5][9], MotorJump_Third[6][9], MotorJump_Third[7][9]};
	MotorPositionPid1.Kp = 5.5;MotorPositionPid2.Kp = 5.5;MotorPositionPid3.Kp = 5.5;MotorPositionPid4.Kp = 5.5;
		MotorPositionPid5.Kp = 5.5;MotorPositionPid6.Kp = 5.5;MotorPositionPid7.Kp = 5.5;MotorPositionPid8.Kp = 5.5;
	//保证腿完全伸长
	if(!ElongFinishFlag){
		if(ArrivePos(TargetPos, 10)){
			ElongFinishFlag = 1;
			LegTimerInit();
			CurrentPoint = 10;
		}
		else	MotorPosExpected(MotorJump_Third[0][9],MotorJump_Third[1][9],MotorJump_Third[2][9],MotorJump_Third[3][9],
														MotorJump_Third[4][9],MotorJump_Third[5][9],MotorJump_Third[6][9],MotorJump_Third[7][9]);
		
	}
	else if(ElongFinishFlag){	
		LegTimer.NowTime = OSTimeGet(&err);	//此刻时间
		if(LegTimer.NowTime - LegTimer.PreTime >= JumpTimeInterval){
			LegTimer.PreTime = LegTimer.NowTime;	//更新时间
			MotorPosExpected(MotorJump_Third[0][CurrentPoint],MotorJump_Third[1][CurrentPoint],MotorJump_Third[2][CurrentPoint],MotorJump_Third[3][CurrentPoint],
												MotorJump_Third[4][CurrentPoint],MotorJump_Third[5][CurrentPoint],MotorJump_Third[6][CurrentPoint],MotorJump_Third[7][CurrentPoint]);
			CurrentPoint++;
		}
		if(CurrentPoint == Jump_LENGTH_THIRD){	//一个周期的最后一个点发送完毕
//			MyDelayms(500);
			ElongFinishFlag = 0;
			JumpFlag = 0;
			StartPosFlag = 0;
			leg_state = STAIR_SECOND_HOLD;
		}
  }
}
void Jump_Pre_Third(void){
	OS_ERR err;
	LegTimer.NowTime = OSTimeGet(&err);	//此刻时间
	if(LegTimer.NowTime - LegTimer.PreTime >= JumpTimeInterval){
		LegTimer.PreTime = LegTimer.NowTime;	//更新时间
		MotorPosExpected(MotorJump_Third[0][CurrentPoint],MotorJump_Third[1][CurrentPoint],MotorJump_Third[2][CurrentPoint],MotorJump_Third[3][CurrentPoint],
											MotorJump_Third[4][CurrentPoint],MotorJump_Third[5][CurrentPoint],MotorJump_Third[6][CurrentPoint],MotorJump_Third[7][CurrentPoint]);
		CurrentPoint++;
	}
	if(CurrentPoint >= 6){	//点5已经发送
		float TargetPos[8] = {MotorJump_Third[0][5], MotorJump_Third[1][5], MotorJump_Third[2][5], MotorJump_Third[3][5], 
												MotorJump_Third[4][5], MotorJump_Third[5][5], MotorJump_Third[6][5], MotorJump_Third[7][5]};
		MotorPosExpected(MotorJump_Third[0][5],MotorJump_Third[1][5],MotorJump_Third[2][5],MotorJump_Third[3][5],
									 MotorJump_Third[4][5],MotorJump_Third[5][5],MotorJump_Third[6][5],MotorJump_Third[7][5]);
//		if(ArrivePos(TargetPos, 10)){
			StartPosFlag = 1;
			CurrentPoint=6;
			MyDelayms(1000);
//		}
	}
}

void Task_Jump_Fourth(void)
{
OS_ERR err;
	float TargetPos[8] = {MotorJump_Fourth[0][9], MotorJump_Fourth[1][9], MotorJump_Fourth[2][9], MotorJump_Fourth[3][9], 
												MotorJump_Fourth[4][9], MotorJump_Fourth[5][9], MotorJump_Fourth[6][9], MotorJump_Fourth[7][9]};
	MotorPositionPid1.Kp = 5.5;MotorPositionPid2.Kp = 5.5;MotorPositionPid3.Kp = 5.5;MotorPositionPid4.Kp = 5.5;
		MotorPositionPid5.Kp = 5.5;MotorPositionPid6.Kp = 5.5;MotorPositionPid7.Kp = 5.5;MotorPositionPid8.Kp = 5.5;
	//保证腿完全伸长
	if(!ElongFinishFlag){
		if(ArrivePos(TargetPos, 10)){
			ElongFinishFlag = 1;
			LegTimerInit();
			CurrentPoint = 10;
		}
		else	MotorPosExpected(MotorJump_Fourth[0][9],MotorJump_Fourth[1][9],MotorJump_Fourth[2][9],MotorJump_Fourth[3][9],
														MotorJump_Fourth[4][9],MotorJump_Fourth[5][9],MotorJump_Fourth[6][9],MotorJump_Fourth[7][9]);
		
	}
	else if(ElongFinishFlag){	
		LegTimer.NowTime = OSTimeGet(&err);	//此刻时间
		if(LegTimer.NowTime - LegTimer.PreTime >= JumpTimeInterval){
			LegTimer.PreTime = LegTimer.NowTime;	//更新时间
			MotorPosExpected(MotorJump_Fourth[0][CurrentPoint],MotorJump_Fourth[1][CurrentPoint],MotorJump_Fourth[2][CurrentPoint],MotorJump_Fourth[3][CurrentPoint],
												MotorJump_Fourth[4][CurrentPoint],MotorJump_Fourth[5][CurrentPoint],MotorJump_Fourth[6][CurrentPoint],MotorJump_Fourth[7][CurrentPoint]);
			CurrentPoint++;
		}
		if(CurrentPoint == Jump_LENGTH_FOURTH){	//一个周期的最后一个点发送完毕
			ElongFinishFlag = 0;
			JumpFlag = 0;
			StartPosFlag = 0;
			leg_state = STAIR_SECOND_HOLD;
		}
  }
}
void Jump_Pre_Fourth(void){
	OS_ERR err;
	LegTimer.NowTime = OSTimeGet(&err);	//此刻时间
	if(LegTimer.NowTime - LegTimer.PreTime >= JumpTimeInterval){
		LegTimer.PreTime = LegTimer.NowTime;	//更新时间
		MotorPosExpected(MotorJump_Fourth[0][CurrentPoint],MotorJump_Fourth[1][CurrentPoint],MotorJump_Fourth[2][CurrentPoint],MotorJump_Fourth[3][CurrentPoint],
											MotorJump_Fourth[4][CurrentPoint],MotorJump_Fourth[5][CurrentPoint],MotorJump_Fourth[6][CurrentPoint],MotorJump_Fourth[7][CurrentPoint]);
		CurrentPoint++;
	}
	if(CurrentPoint >= 6){	//点5已经发送
		float TargetPos[8] = {MotorJump_Fourth[0][5], MotorJump_Fourth[1][5], MotorJump_Fourth[2][5], MotorJump_Fourth[3][5], 
												MotorJump_Fourth[4][5], MotorJump_Fourth[5][5], MotorJump_Fourth[6][5], MotorJump_Fourth[7][5]};
		MotorPosExpected(MotorJump_Fourth[0][5],MotorJump_Fourth[1][5],MotorJump_Fourth[2][5],MotorJump_Fourth[3][5],
									 MotorJump_Fourth[4][5],MotorJump_Fourth[5][5],MotorJump_Fourth[6][5],MotorJump_Fourth[7][5]);
//		if(ArrivePos(TargetPos, 10)){
			StartPosFlag = 1;
			CurrentPoint=6;
			MyDelayms(1000);
//		}
	}
}

void Task_Jump_Fifth(void)
{
OS_ERR err;
	float TargetPos[8] = {MotorJump_Fifth[0][9], MotorJump_Fifth[1][9], MotorJump_Fifth[2][9], MotorJump_Fifth[3][9], 
												MotorJump_Fifth[4][9], MotorJump_Fifth[5][9], MotorJump_Fifth[6][9], MotorJump_Fifth[7][9]};
	MotorPositionPid1.Kp = 5.5;MotorPositionPid2.Kp = 5.5;MotorPositionPid3.Kp = 5.5;MotorPositionPid4.Kp = 5.5;
		MotorPositionPid5.Kp = 5.5;MotorPositionPid6.Kp = 5.5;MotorPositionPid7.Kp = 5.5;MotorPositionPid8.Kp = 5.5;
	//保证腿完全伸长
	if(!ElongFinishFlag){
		if(ArrivePos(TargetPos, 10)){
			ElongFinishFlag = 1;
			LegTimerInit();
			CurrentPoint = 10;
		}
		else	MotorPosExpected(MotorJump_Fifth[0][9],MotorJump_Fifth[1][9],MotorJump_Fifth[2][9],MotorJump_Fifth[3][9],
														MotorJump_Fifth[4][9],MotorJump_Fifth[5][9],MotorJump_Fifth[6][9],MotorJump_Fifth[7][9]);
		
	}
	else if(ElongFinishFlag){	
		LegTimer.NowTime = OSTimeGet(&err);	//此刻时间
		if(LegTimer.NowTime - LegTimer.PreTime >= JumpTimeInterval){
			LegTimer.PreTime = LegTimer.NowTime;	//更新时间
			MotorPosExpected(MotorJump_Fifth[0][CurrentPoint],MotorJump_Fifth[1][CurrentPoint],MotorJump_Fifth[2][CurrentPoint],MotorJump_Fifth[3][CurrentPoint],
												MotorJump_Fifth[4][CurrentPoint],MotorJump_Fifth[5][CurrentPoint],MotorJump_Fifth[6][CurrentPoint],MotorJump_Fifth[7][CurrentPoint]);
			CurrentPoint++;
		}
		if(CurrentPoint == Jump_LENGTH_FIFTH){	//一个周期的最后一个点发送完毕
//			MyDelayms(500);
			ElongFinishFlag = 0;
			JumpFlag = 0;
			StartPosFlag = 0;
			leg_state = STAND;
		}
  }
}
void Jump_Pre_Fifth(void){
	OS_ERR err;
	LegTimer.NowTime = OSTimeGet(&err);	//此刻时间
	if(LegTimer.NowTime - LegTimer.PreTime >= JumpTimeInterval){
		LegTimer.PreTime = LegTimer.NowTime;	//更新时间
		MotorPosExpected(MotorJump_Fifth[0][CurrentPoint],MotorJump_Fifth[1][CurrentPoint],MotorJump_Fifth[2][CurrentPoint],MotorJump_Fifth[3][CurrentPoint],
											MotorJump_Fifth[4][CurrentPoint],MotorJump_Fifth[5][CurrentPoint],MotorJump_Fifth[6][CurrentPoint],MotorJump_Fifth[7][CurrentPoint]);
		CurrentPoint++;
	}
	if(CurrentPoint >= 6){	//点5已经发送
		float TargetPos[8] = {MotorJump_Fifth[0][5], MotorJump_Fifth[1][5], MotorJump_Fifth[2][5], MotorJump_Fifth[3][5], 
												MotorJump_Fifth[4][5], MotorJump_Fifth[5][5], MotorJump_Fifth[6][5], MotorJump_Fifth[7][5]};
		MotorPosExpected(MotorJump_Fifth[0][5],MotorJump_Fifth[1][5],MotorJump_Fifth[2][5],MotorJump_Fifth[3][5],
									 MotorJump_Fifth[4][5],MotorJump_Fifth[5][5],MotorJump_Fifth[6][5],MotorJump_Fifth[7][5]);
//		if(ArrivePos(TargetPos, 10)){
			StartPosFlag = 1;
			CurrentPoint=6;
			MyDelayms(1000);
//		}
	}
}
//上台阶第一极
void Upstair(void){
	OS_ERR err;
float TargetPos[8] = {MotorUpstair[0][5], MotorUpstair[1][5], MotorUpstair[2][5], MotorUpstair[3][5], 
												MotorUpstair[4][5], MotorUpstair[5][5], MotorUpstair[6][5], MotorUpstair[7][5]};
	//保证腿完全伸长
	if(!UpstairStart){
		if(ArrivePos(TargetPos, 6)){
			UpstairStart = 1;
			LegTimerInit();
			CurrentPoint = 6;
		}
		else	MotorPosExpected(MotorUpstair[0][5],MotorUpstair[1][5],MotorUpstair[2][5],MotorUpstair[3][5],
														MotorUpstair[4][5],MotorUpstair[5][5],MotorUpstair[6][5],MotorUpstair[7][5]);
		
	}
	else if(UpstairStart){	//起步姿态调整完毕
	MotorPositionPid1.Kp = 3.0;MotorPositionPid2.Kp = 3.0;MotorPositionPid3.Kp = 3.0;MotorPositionPid4.Kp = 3.0;
  MotorPositionPid5.Kp = 3.0;MotorPositionPid6.Kp = 3.0;MotorPositionPid7.Kp = 3.0;MotorPositionPid8.Kp = 3.0;

	LegTimer.NowTime = OSTimeGet(&err);	//此刻时间
	
	if(LegTimer.NowTime - LegTimer.PreTime >= JumpTimeInterval){
		LegTimer.PreTime = LegTimer.NowTime;	//更新时间
	  MotorPosExpected(MotorUpstair[0][CurrentPoint], MotorUpstair[1][CurrentPoint], MotorUpstair[2][CurrentPoint], MotorUpstair[3][CurrentPoint], 
												MotorUpstair[4][CurrentPoint], MotorUpstair[5][CurrentPoint], MotorUpstair[6][CurrentPoint], MotorUpstair[7][CurrentPoint]);
		if(CurrentPoint<Upstair_LENGTH-1)
		{
	     CurrentPoint++;
		}
	}
//	if(CurrentPoint == Upstair_LENGTH){	//一个周期的最后一个点发送完毕
//		MotorPosExpected(MotorUpstair[0][45], MotorUpstair[1][45], MotorUpstair[2][45], MotorUpstair[3][45], 
//												MotorUpstair[4][45], MotorUpstair[5][45], MotorUpstair[6][45], MotorUpstair[7][45]);
//  }
}
}

void RightRotate(void){
OS_ERR err;
	//先调整起步姿态
	if(!RightRotateFlag){
		float TargetPos[8] = {MotorRightRotate[0][0], MotorRightRotate[1][0], MotorRightRotate[2][0], MotorRightRotate[3][0], 
													MotorRightRotate[4][0], MotorRightRotate[5][0], MotorRightRotate[6][0], MotorRightRotate[7][0]};
		if(ArrivePos(TargetPos, 10)){
			RightRotateFlag = 1;
			MotorPositionPid1.Kp = 5.0;MotorPositionPid2.Kp = 5.0;MotorPositionPid3.Kp = 5.0;MotorPositionPid4.Kp = 5.0;
			MotorPositionPid5.Kp = 5.0;MotorPositionPid6.Kp = 5.0;MotorPositionPid7.Kp = 5.0;MotorPositionPid8.Kp = 5.0;
			LegTimerInit();
			//MyDelayms(2000);
		}
		else	MotorPosExpected(MotorRightRotate[0][0],MotorRightRotate[1][0],MotorRightRotate[2][0],MotorRightRotate[3][0],
														MotorRightRotate[4][0],MotorRightRotate[5][0],MotorRightRotate[6][0],MotorRightRotate[7][0]);
	}
	
	else if(RightRotateFlag){	//起步姿态调整完毕
		LegTimer.NowTime = OSTimeGet(&err);	//此刻时间
		if(LegTimer.NowTime - LegTimer.PreTime >= RotateTimeInterval){
			LegTimer.PreTime = LegTimer.NowTime;	//更新时间
			MotorPosExpected(MotorRightRotate[0][CurrentPoint],MotorRightRotate[1][CurrentPoint],MotorRightRotate[2][CurrentPoint],MotorRightRotate[3][CurrentPoint],
												MotorRightRotate[4][CurrentPoint],MotorRightRotate[5][CurrentPoint],MotorRightRotate[6][CurrentPoint],MotorRightRotate[7][CurrentPoint]);
			CurrentPoint++;
		}
		if(CurrentPoint == RightRotate_LENGTH){	//一个周期的最后一个点发送完毕
			CurrentPoint = 0;
		}
	}

}
void LeftRotate(void)
{
OS_ERR err;
	//先调整起步姿态
	if(!LeftRotateFlag){
		float TargetPos[8] = {MotorLeftRotate[0][0], MotorLeftRotate[1][0], MotorLeftRotate[2][0], MotorLeftRotate[3][0], 
													MotorLeftRotate[4][0], MotorLeftRotate[5][0], MotorLeftRotate[6][0], MotorLeftRotate[7][0]};
		if(ArrivePos(TargetPos, 10)){
			LeftRotateFlag = 1;
			MotorPositionPid1.Kp = 5.0;MotorPositionPid2.Kp = 5.0;MotorPositionPid3.Kp = 5.0;MotorPositionPid4.Kp = 5.0;
			MotorPositionPid5.Kp = 5.0;MotorPositionPid6.Kp = 5.0;MotorPositionPid7.Kp = 5.0;MotorPositionPid8.Kp = 5.0;
			LegTimerInit();
			//MyDelayms(2000);
		}
		else	MotorPosExpected(MotorLeftRotate[0][0],MotorLeftRotate[1][0],MotorLeftRotate[2][0],MotorLeftRotate[3][0],
														MotorLeftRotate[4][0],MotorLeftRotate[5][0],MotorLeftRotate[6][0],MotorLeftRotate[7][0]);
	}
	
	else if(LeftRotateFlag){	//起步姿态调整完毕
		LegTimer.NowTime = OSTimeGet(&err);	//此刻时间
		if(LegTimer.NowTime - LegTimer.PreTime >= RotateTimeInterval){
			LegTimer.PreTime = LegTimer.NowTime;	//更新时间
			MotorPosExpected(MotorLeftRotate[0][CurrentPoint],MotorLeftRotate[1][CurrentPoint],MotorLeftRotate[2][CurrentPoint],MotorLeftRotate[3][CurrentPoint],
												MotorLeftRotate[4][CurrentPoint],MotorLeftRotate[5][CurrentPoint],MotorLeftRotate[6][CurrentPoint],MotorLeftRotate[7][CurrentPoint]);
			CurrentPoint++;
		}
		if(CurrentPoint == LeftRotate_LENGTH){	//一个周期的最后一个点发送完毕
			CurrentPoint = 0;
		}
	}
}

void Stair_RightRotate(void){
OS_ERR err;
	//先调整起步姿态
	if(!StairRightRotateFlag){
		float TargetPos[8] = {MotorStairRightRotate[0][0], MotorStairRightRotate[1][0], MotorStairRightRotate[2][0], MotorStairRightRotate[3][0], 
													MotorStairRightRotate[4][0], MotorStairRightRotate[5][0], MotorStairRightRotate[6][0], MotorStairRightRotate[7][0]};
		if(ArrivePos(TargetPos, 10)){
			StairRightRotateFlag = 1;
			MotorPositionPid1.Kp = 5.0;MotorPositionPid2.Kp = 5.0;MotorPositionPid3.Kp = 5.0;MotorPositionPid4.Kp = 5.0;
			MotorPositionPid5.Kp = 5.0;MotorPositionPid6.Kp = 5.0;MotorPositionPid7.Kp = 5.0;MotorPositionPid8.Kp = 5.0;
			LegTimerInit();
			//MyDelayms(2000);
		}
		else	MotorPosExpected(MotorStairRightRotate[0][0],MotorStairRightRotate[1][0],MotorStairRightRotate[2][0],MotorStairRightRotate[3][0],
														MotorStairRightRotate[4][0],MotorStairRightRotate[5][0],MotorStairRightRotate[6][0],MotorStairRightRotate[7][0]);
	}
	
	else if(StairRightRotateFlag){	//起步姿态调整完毕
		LegTimer.NowTime = OSTimeGet(&err);	//此刻时间
		if(LegTimer.NowTime - LegTimer.PreTime >= StairRotateTimeInterval){
			LegTimer.PreTime = LegTimer.NowTime;	//更新时间
			MotorPosExpected(MotorStairRightRotate[0][CurrentPoint],MotorStairRightRotate[1][CurrentPoint],MotorStairRightRotate[2][CurrentPoint],MotorStairRightRotate[3][CurrentPoint],
												MotorStairRightRotate[4][CurrentPoint],MotorStairRightRotate[5][CurrentPoint],MotorStairRightRotate[6][CurrentPoint],MotorStairRightRotate[7][CurrentPoint]);
			CurrentPoint++;
		}
		if(CurrentPoint == StairRightRotate_LENGTH){	//一个周期的最后一个点发送完毕
			CurrentPoint = 0;
		}
	}

}

void Stair_LeftRotate(void)
{
OS_ERR err;
	//先调整起步姿态
	if(!StairLeftRotateFlag){
		float TargetPos[8] = {MotorStairLeftRotate[0][0], MotorStairLeftRotate[1][0], MotorStairLeftRotate[2][0], MotorStairLeftRotate[3][0], 
													MotorStairLeftRotate[4][0], MotorStairLeftRotate[5][0], MotorStairLeftRotate[6][0], MotorStairLeftRotate[7][0]};
		if(ArrivePos(TargetPos, 10)){
			StairLeftRotateFlag = 1;
			MotorPositionPid1.Kp = 5.0;MotorPositionPid2.Kp = 5.0;MotorPositionPid3.Kp = 5.0;MotorPositionPid4.Kp = 5.0;
			MotorPositionPid5.Kp = 5.0;MotorPositionPid6.Kp = 5.0;MotorPositionPid7.Kp = 5.0;MotorPositionPid8.Kp = 5.0;
			LegTimerInit();
			//MyDelayms(2000);
		}
		else	MotorPosExpected(MotorStairLeftRotate[0][0],MotorStairLeftRotate[1][0],MotorStairLeftRotate[2][0],MotorStairLeftRotate[3][0],
														MotorStairLeftRotate[4][0],MotorStairLeftRotate[5][0],MotorStairLeftRotate[6][0],MotorStairLeftRotate[7][0]);
	}
	
	else if(StairLeftRotateFlag){	//起步姿态调整完毕
		LegTimer.NowTime = OSTimeGet(&err);	//此刻时间
		if(LegTimer.NowTime - LegTimer.PreTime >= StairRotateTimeInterval){
			LegTimer.PreTime = LegTimer.NowTime;	//更新时间
			MotorPosExpected(MotorStairLeftRotate[0][CurrentPoint],MotorStairLeftRotate[1][CurrentPoint],MotorStairLeftRotate[2][CurrentPoint],MotorStairLeftRotate[3][CurrentPoint],
												MotorStairLeftRotate[4][CurrentPoint],MotorStairLeftRotate[5][CurrentPoint],MotorStairLeftRotate[6][CurrentPoint],MotorStairLeftRotate[7][CurrentPoint]);
			CurrentPoint++;
		}
		if(CurrentPoint == StairLeftRotate_LENGTH){	//一个周期的最后一个点发送完毕
			CurrentPoint = 0;
		}
	}
}


