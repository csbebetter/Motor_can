#ifndef __LEG_CONTROL_H
#define __LEG_CONTROL_H

#include "system_config.h"


typedef struct
{
  uint32_t StartTime;
	volatile uint32_t PreTime;
	volatile uint32_t NowTime;
}LegTimeTypedef;

void LegTimerInit(void);
void LegDisable(void);
void StandingHold(void);

void Stair_First_Hold(void);
void Stair_Second_Hold(void);
void Stair_Third_Hold(void);
void Stair_Fo_Hold(void);
void Stair_Third_Hold(void);

void StartPosControl(void);
void UpSlopeControl(void);
void UpSlope_Fh_Control(void);
void Upstair_Fix(void);

void DownSlopeControl(void);
void Trot_Straight(void);
void Slope_Up(void);
void Slope_Down(void);
void Walk_Straight(void);
bool ArrivePos(float *ArriveGoalPos, int DAngle);	//是否到达指定位姿范围
void MotorPosExpected(float pos0,float pos1,float pos2,float pos3,float pos4,float pos5,float pos6,float pos7);	//设置电机期望位置
void MotorSpeedExpected(float Spe0,float Spe1,float Spe2,float Spe3,float Spe4,float Spe5,float Spe6,float Spe7);	//设置电机期望速度

void Task_FlyJump(void);
void SlopeHold(void);
void Jump_Pre(void);
void Task_NormalJump(void);
void LittleJump_Pre(void);
void Task_LittleJump(void);


void Task_Jump_First(void);
void Jump_Pre_First(void);
void Task_Jump_Second(void);
void Jump_Pre_Second(void);
void Task_Jump_Third(void);
void Jump_Pre_Third(void);
void Task_Jump_Fourth(void);
void Jump_Pre_Fourth(void);
void Task_Jump_Fifth(void);
void Jump_Pre_Fifth(void);


void Upstair(void);
void RightRotate(void);//原地选择姿态
void LeftRotate(void);
void Stair_RightRotate(void);//原地选择姿态
void Stair_LeftRotate(void);
void TITA_Straight(void);

//void LegInit();
//void LegUp();
//void LegBack();
//void LegDown();
//void LegPosControlStart(void);
//void LandingPosControl();
//void HoldOn(void);
//void PoseControl(void);
//void StandingPose(void);
//void Stop(void);
//void Restart(void);
//void Crawl(void);
void PosCaliBration();
bool OnlineInitFinish(void);

#endif