/* Includes ------------------------------------------------------------------*/
#include "chassis.h"
#include "arm_math.h"
#include "hust_math_lib.h"
#include "MyType.h"

extern volatile ChassisInfoTypedef MyChassis;


void LegPoints_Init(void)
{
//	A.x = 0.0f;
//	A.y = 0.0f;
//	E.x = 0.0f;
//	E.y = 0.0f;
}

//float Calculate_Motor_Angle(float xx, float yy, LegPointsTypedef * A, int side)
//{
//	float x, y;
//	float afa = 0, theta = 0, fai = 0;
//	float A0, B0, C0, T;

//	theta = FastAcos(((L2 + L3) *  (L2 + L3) + xx * xx + yy * yy - L1 * L1) / (2 * (L2 + L3)* sqrtf(xx*xx + yy * yy)));
//	afa = FastTableAtan(yy / xx);
//	theta = theta * 180 / PI;
//	afa = afa * 180 / PI;
//	if (afa < 0)
//		afa = afa + 180;

//	fai = afa - theta;

//	x = xx - L3 * cos(fai * PI / 180.0f);
//	y = yy - L3 * sin(fai* PI / 180.0f);

//	A0 = (x - A->x) * (x - A->x) + (y - A->y) * (y - A->y) + L1 * L1 - L2 * L2;
//	B0 = -2 * (x - A->x) * L1;
//	C0 = -2 * (y - A->y) * L1;
//	/* A0 + B0cos(fy1) + C0sin(fy1) = 0 */
//	// T =( -C0 + -1 * sqrtf(C0*C0 + B0*B0 - A0*A0) ) / (A0 - B0)	
//	// T =( -C0 +  1 * sqrtf(C0*C0 + B0*B0 - A0*A0) ) / (A0 - B0)  

//	T = (-C0 + side * sqrtf(C0*C0 + B0 * B0 - A0 * A0)) / (A0 - B0);
//	A->fy = 2 * FastTableAtan(T) * 180 / PI;
//	return A->fy;
//}


void BezierControlPoints_Init(void)
{
}

//void BezierControlPointsArray_Init(void)
//{
//	int LegID;
//	for (LegID = 0; LegID < 4; LegID++)
//	{
//		p[LegID][0].x = -80;
//		p[LegID][0].y = 250;

//		p[LegID][1].x = -60;
//		p[LegID][1].y = 170;

//		p[LegID][2].x = 30;
//		p[LegID][2].y = 170;

//		p[LegID][3].x = 60;
//		p[LegID][3].y = 250;
//	}
//}
//void BezierControlPoints_Assign(float x, float y, ClassicPointsTypedef *point)
//{
//	point->x = x;
//	point->y = y;
//}
//void BezierRegulate(int LegID)
//{
//	int t = 0;
//	double time;
//	float temp, x, y;
//	for (t = 0; t < (LENGTH / 3); t++)
//	{
//		time = t;
//		temp = time / 10.0;
//		x = (1 - temp)*(1 - temp)*(1 - temp) * p[LegID - 1][0].x + 3 * temp*(1 - temp)*(1 - temp) * p[LegID - 1][1].x + 3 * temp*temp*(1 - temp)*p[LegID - 1][2].x + temp * temp*temp * p[LegID - 1][3].x;
//		y = (1 - temp)*(1 - temp)*(1 - temp) * p[LegID - 1][0].y + 3 * temp*(1 - temp)*(1 - temp) * p[LegID - 1][1].y + 3 * temp*temp*(1 - temp)*p[LegID - 1][2].y + temp * temp*temp * p[LegID - 1][3].y;
//		FOOTPOINT[LegID - 1][t].x = x;
//		FOOTPOINT[LegID - 1][t].y = y;
//	}
//}
//void SinRegulate(float Amplitude, int LegID)
//{
//	int t = 0;
//	float  x, y;
//	float temp;
//	double time;
//	float frequency = p[LegID - 1][3].x - p[LegID - 1][0].x;
//	for (t = (LENGTH / 3); t < LENGTH; t++)
//	{
//		time = t;
//		temp = (time - (LENGTH / 3)) / 20;
//		x = p[LegID - 1][3].x - frequency * temp;
//		y = p[LegID - 1][3].y + Amplitude * sin(PI*temp);
//		FOOTPOINT[LegID - 1][t].x = x;
//		FOOTPOINT[LegID - 1][t].y = y;
//	}
//}
//float AngleConversion(float PreAngle, int MotorID)
//{
//	float AfterConversion;
//	switch (MotorID)
//	{
//	case 1:
//		AfterConversion = -221.45 - PreAngle;
//		return AfterConversion;
//	case 2:
//		AfterConversion = 138.555 - (180 - PreAngle);
//		if (AfterConversion < 0)
//			return AfterConversion + 360;
//		return AfterConversion;
//	case 3:
//		AfterConversion = 138.555 - PreAngle;
//		return AfterConversion;
//	case 4:
//		AfterConversion = -41.445 + PreAngle;
//		if (AfterConversion > 0)
//			return AfterConversion - 360;
//		return AfterConversion;
//	case 5:
//		AfterConversion = 41.445 + 360 - PreAngle;
//		if (AfterConversion > 360)
//			return AfterConversion - 360;
//		return AfterConversion;
//	case 6:
//		AfterConversion = -(180 - 41.445 - PreAngle);
//		return AfterConversion;
//	case 7:
//		AfterConversion = -(PreAngle - 41.445);
//		if (AfterConversion > 0)
//			return AfterConversion - 360;
//		return AfterConversion;
//	case 8:
//		AfterConversion = 41.445 + 180 + PreAngle;
//		return AfterConversion;
//	default:break;
//	}
//}
//void AngleReverse(int MotorID)
//{
//	int temp;
//	float vector;
//	switch (MotorID)
//	{
//	case 1:
//		for (temp = 1; temp <= (LENGTH / 2); temp++)
//		{
//			vector = MOTOR_1_ANGLE[temp];
//			MOTOR_1_ANGLE[temp] = MOTOR_1_ANGLE[LENGTH - temp];
//			MOTOR_1_ANGLE[LENGTH - temp] = vector;
//		}
//		break;
//	case 2:
//		for (temp = 1; temp <= (LENGTH / 2); temp++)
//		{
//			vector = MOTOR_2_ANGLE[temp];
//			MOTOR_2_ANGLE[temp] = MOTOR_2_ANGLE[LENGTH - temp];
//			MOTOR_2_ANGLE[LENGTH - temp] = vector;
//		}
//		break;
//	case 3:
//		for (temp = 1; temp <= (LENGTH / 2); temp++)
//		{
//			vector = MOTOR_3_ANGLE[temp];
//			MOTOR_3_ANGLE[temp] = MOTOR_3_ANGLE[LENGTH - temp];
//			MOTOR_3_ANGLE[LENGTH - temp] = vector;
//		}
//		break;
//	case 4:
//		for (temp = 1; temp <= (LENGTH / 2); temp++)
//		{
//			vector = MOTOR_4_ANGLE[temp];
//			MOTOR_4_ANGLE[temp] = MOTOR_4_ANGLE[LENGTH - temp];
//			MOTOR_4_ANGLE[LENGTH - temp] = vector;
//		}
//		break;
//	case 5:
//		for (temp = 1; temp <= (LENGTH / 2); temp++)
//		{
//			vector = MOTOR_5_ANGLE[temp];
//			MOTOR_5_ANGLE[temp] = MOTOR_5_ANGLE[LENGTH - temp];
//			MOTOR_5_ANGLE[LENGTH - temp] = vector;
//		}
//		break;
//	case 6:
//		for (temp = 1; temp <= (LENGTH / 2); temp++)
//		{
//			vector = MOTOR_6_ANGLE[temp];
//			MOTOR_6_ANGLE[temp] = MOTOR_6_ANGLE[LENGTH - temp];
//			MOTOR_6_ANGLE[LENGTH - temp] = vector;
//		}
//		break;
//	case 7:
//		for (temp = 1; temp <= (LENGTH / 2); temp++)
//		{
//			vector = MOTOR_7_ANGLE[temp];
//			MOTOR_7_ANGLE[temp] = MOTOR_7_ANGLE[LENGTH - temp];
//			MOTOR_7_ANGLE[LENGTH - temp] = vector;
//		}
//		break;
//	case 8:
//		for (temp = 1; temp <= (LENGTH / 2); temp++)
//		{
//			vector = MOTOR_8_ANGLE[temp];
//			MOTOR_8_ANGLE[temp] = MOTOR_8_ANGLE[LENGTH - temp];
//			MOTOR_8_ANGLE[LENGTH - temp] = vector;
//		}
//		break;
//	default:break;
//	}
//}
//void Navigation(void)
//{
//	int count;
//	int temp = 0;
//	float dYaw = MyChassis.AimState.Yaw - MyChassis.CurrentState.Yaw;
//	float dPitch = MyChassis.AimState.Pitch - MyChassis.CurrentState.Pitch;
//	if (dPitch >= 0)
//		PitchUp(dPitch);
//	else if (dPitch <= 0)
//		PitchDown(dPitch);
//	BezierRegulate(1);
//	BezierRegulate(2);
//	BezierRegulate(3);
//	BezierRegulate(4);
//	SinRegulate(10, 1);
//	SinRegulate(10, 2);
//	SinRegulate(10, 3);
//	SinRegulate(10, 4);
//	for (count = 0; count < LENGTH; count++)
//	{
//		/*************** the first leg **************/
//		//cout << "/*************** the first leg **************/" << endl;
//		MOTOR_1_ANGLE[count] = Calculate_Motor_Angle(FOOTPOINT[0][count].x, FOOTPOINT[0][count].y, &A, 1);
//		MOTOR_2_ANGLE[count] = Calculate_Motor_Angle(FOOTPOINT[0][count].x, FOOTPOINT[0][count].y, &A, -1);

//		MOTOR_1_ANGLE[count] = AngleConversion_2generation(MOTOR_1_ANGLE[count], 1);
//		MOTOR_2_ANGLE[count] = AngleConversion_2generation(MOTOR_2_ANGLE[count], 2);

//		/*************** the second leg **************/
//		//cout << "/*************** the second leg **************/" << endl;
//		MOTOR_3_ANGLE[count] = Calculate_Motor_Angle(FOOTPOINT[1][count].x, FOOTPOINT[1][count].y, &A, 1);
//		MOTOR_4_ANGLE[count] = Calculate_Motor_Angle(FOOTPOINT[1][count].x, FOOTPOINT[1][count].y, &A, -1);

//		MOTOR_3_ANGLE[count] = AngleConversion_2generation(MOTOR_3_ANGLE[count], 3);
//		MOTOR_4_ANGLE[count] = AngleConversion_2generation(MOTOR_4_ANGLE[count], 4);

//		/*************** the third leg **************/
//		//cout << "/*************** the third leg **************/" << endl;
//		MOTOR_5_ANGLE[count] = Calculate_Motor_Angle(FOOTPOINT[2][count].x, FOOTPOINT[2][count].y, &A, 1);
//		MOTOR_6_ANGLE[count] = Calculate_Motor_Angle(FOOTPOINT[2][count].x, FOOTPOINT[2][count].y, &A, -1);

//		MOTOR_5_ANGLE[count] = AngleConversion_2generation(MOTOR_5_ANGLE[count], 5);
//		MOTOR_6_ANGLE[count] = AngleConversion_2generation(MOTOR_6_ANGLE[count], 6);

//		/*************** the fourth leg **************/
//		//cout << "/*************** the fourth leg **************/" << endl;
//		MOTOR_7_ANGLE[count] = Calculate_Motor_Angle(FOOTPOINT[3][count].x, FOOTPOINT[3][count].y, &A, 1);
//		MOTOR_8_ANGLE[count] = Calculate_Motor_Angle(FOOTPOINT[3][count].x, FOOTPOINT[3][count].y, &A, -1);

//		MOTOR_7_ANGLE[count] = AngleConversion_2generation(MOTOR_7_ANGLE[count], 7);
//		MOTOR_8_ANGLE[count] = AngleConversion_2generation(MOTOR_8_ANGLE[count], 8);
//	}
//	//ChangeOrder(3);
//	//ChangeOrder(4);
//	//ChangeOrder(7);
//	//ChangeOrder(8);
//}
//void ChangeOrder(int MotorID)
//{
//	int temp = 0;
//	float vector = 0;
//	switch (MotorID)
//	{
//	case 1:
//		for (temp = (LENGTH / 3); temp < (2*LENGTH / 3); temp++)
//		{
//			vector = MOTOR_1_ANGLE[temp];
//			MOTOR_1_ANGLE[temp] = MOTOR_1_ANGLE[temp + (LENGTH / 3)];
//			MOTOR_1_ANGLE[temp + (LENGTH / 3)] = vector;
//		}
//		for (temp = 0; temp < (LENGTH / 3); temp++)
//		{
//			vector = MOTOR_1_ANGLE[temp];
//			MOTOR_1_ANGLE[temp] = MOTOR_1_ANGLE[temp + (2*LENGTH / 3)];
//			MOTOR_1_ANGLE[temp + (2*LENGTH / 3)] = vector;
//		}
//		break;
//	case 2:
//		for (temp = (LENGTH / 3); temp < (2 * LENGTH / 3); temp++)
//		{
//			vector = MOTOR_2_ANGLE[temp];
//			MOTOR_2_ANGLE[temp] = MOTOR_2_ANGLE[temp + (LENGTH / 3)];
//			MOTOR_2_ANGLE[temp + (LENGTH / 3)] = vector;
//		}
//		for (temp = 0; temp < (LENGTH / 3); temp++)
//		{
//			vector = MOTOR_2_ANGLE[temp];
//			MOTOR_2_ANGLE[temp] = MOTOR_2_ANGLE[temp + (2 * LENGTH / 3)];
//			MOTOR_2_ANGLE[temp + (2 * LENGTH / 3)] = vector;
//		}
//		break;

//	case 3:
//		for (temp = (LENGTH / 3); temp < (2 * LENGTH / 3); temp++)
//		{
//			vector = MOTOR_3_ANGLE[temp];
//			MOTOR_3_ANGLE[temp] = MOTOR_3_ANGLE[temp + (LENGTH / 3)];
//			MOTOR_3_ANGLE[temp + (LENGTH / 3)] = vector;
//		}
//		for (temp = 0; temp < (LENGTH / 3); temp++)
//		{
//			vector = MOTOR_3_ANGLE[temp];
//			MOTOR_3_ANGLE[temp] = MOTOR_3_ANGLE[temp + (2 * LENGTH / 3)];
//			MOTOR_3_ANGLE[temp + (2 * LENGTH / 3)] = vector;
//		}
//		break;
//	case 4:
//		for (temp = (LENGTH / 3); temp < (2 * LENGTH / 3); temp++)
//		{
//			vector = MOTOR_4_ANGLE[temp];
//			MOTOR_4_ANGLE[temp] = MOTOR_4_ANGLE[temp + (LENGTH / 3)];
//			MOTOR_4_ANGLE[temp + (LENGTH / 3)] = vector;
//		}
//		for (temp = 0; temp < (LENGTH / 3); temp++)
//		{
//			vector = MOTOR_4_ANGLE[temp];
//			MOTOR_4_ANGLE[temp] = MOTOR_4_ANGLE[temp + (2 * LENGTH / 3)];
//			MOTOR_4_ANGLE[temp + (2 * LENGTH / 3)] = vector;
//		}
//		break;

//	case 5:
//		for (temp = (LENGTH / 3); temp < (2 * LENGTH / 3); temp++)
//		{
//			vector = MOTOR_5_ANGLE[temp];
//			MOTOR_5_ANGLE[temp] = MOTOR_5_ANGLE[temp + (LENGTH / 3)];
//			MOTOR_5_ANGLE[temp + (LENGTH / 3)] = vector;
//		}
//		for (temp = 0; temp < (LENGTH / 3); temp++)
//		{
//			vector = MOTOR_5_ANGLE[temp];
//			MOTOR_5_ANGLE[temp] = MOTOR_5_ANGLE[temp + (2 * LENGTH / 3)];
//			MOTOR_5_ANGLE[temp + (2 * LENGTH / 3)] = vector;
//		}
//		break;
//	case 6:
//		for (temp = (LENGTH / 3); temp < (2 * LENGTH / 3); temp++)
//		{
//			vector = MOTOR_6_ANGLE[temp];
//			MOTOR_6_ANGLE[temp] = MOTOR_6_ANGLE[temp + (LENGTH / 3)];
//			MOTOR_6_ANGLE[temp + (LENGTH / 3)] = vector;
//		}
//		for (temp = 0; temp < (LENGTH / 3); temp++)
//		{
//			vector = MOTOR_6_ANGLE[temp];
//			MOTOR_6_ANGLE[temp] = MOTOR_6_ANGLE[temp + (2 * LENGTH / 3)];
//			MOTOR_6_ANGLE[temp + (2 * LENGTH / 3)] = vector;
//		}
//		break;

//	case 7:
//		for (temp = (LENGTH / 3); temp < (2 * LENGTH / 3); temp++)
//		{
//			vector = MOTOR_7_ANGLE[temp];
//			MOTOR_7_ANGLE[temp] = MOTOR_7_ANGLE[temp + (LENGTH / 3)];
//			MOTOR_7_ANGLE[temp + (LENGTH / 3)] = vector;
//		}
//		for (temp = 0; temp < (LENGTH / 3); temp++)
//		{
//			vector = MOTOR_7_ANGLE[temp];
//			MOTOR_7_ANGLE[temp] = MOTOR_7_ANGLE[temp + (2 * LENGTH / 3)];
//			MOTOR_7_ANGLE[temp + (2 * LENGTH / 3)] = vector;
//		}
//		break;
//	case 8:
//		for (temp = (LENGTH / 3); temp < (2 * LENGTH / 3); temp++)
//		{
//			vector = MOTOR_8_ANGLE[temp];
//			MOTOR_8_ANGLE[temp] = MOTOR_8_ANGLE[temp + (LENGTH / 3)];
//			MOTOR_8_ANGLE[temp + (LENGTH / 3)] = vector;
//		}
//		for (temp = 0; temp < (LENGTH / 3); temp++)
//		{
//			vector = MOTOR_8_ANGLE[temp];
//			MOTOR_8_ANGLE[temp] = MOTOR_8_ANGLE[temp + (2 * LENGTH / 3)];
//			MOTOR_8_ANGLE[temp + (2 * LENGTH / 3)] = vector;
//		}
//		break;

//	default:break;
//	}
//}
//void RightTurn(float yaw)
//{
//	float deltaYaw = TURNINGFACTOR * yaw;
//	BezierControlPoints_Assign(-80 + fabs(deltaYaw), 250, &p[0][0]);
//	BezierControlPoints_Assign(60 - fabs(deltaYaw), 250, &p[0][3]);

//	BezierControlPoints_Assign(-80 + fabs(deltaYaw), 250, &p[1][0]);
//	BezierControlPoints_Assign(60 - fabs(deltaYaw), 250, &p[1][3]);

//	BezierControlPoints_Assign(-80 - fabs(deltaYaw), 250, &p[2][0]);
//	BezierControlPoints_Assign(60 + fabs(deltaYaw), 250, &p[2][3]);

//	BezierControlPoints_Assign(-80 - fabs(deltaYaw), 250, &p[3][0]);
//	BezierControlPoints_Assign(60 + fabs(deltaYaw), 250, &p[3][3]);
//}
//void LeftTurn(float yaw)
//{
//	float deltaYaw = TURNINGFACTOR * yaw;
//	BezierControlPoints_Assign(-80 - fabs(deltaYaw), 250, &p[0][0]);
//	BezierControlPoints_Assign(60 + fabs(deltaYaw), 250, &p[0][3]);

//	BezierControlPoints_Assign(-80 - fabs(deltaYaw), 250, &p[1][0]);
//	BezierControlPoints_Assign(60 + fabs(deltaYaw), 250, &p[1][3]);

//	BezierControlPoints_Assign(-80 + fabs(deltaYaw), 250, &p[2][0]);
//	BezierControlPoints_Assign(60 - fabs(deltaYaw), 250, &p[2][3]);

//	BezierControlPoints_Assign(-80 + fabs(deltaYaw), 250, &p[3][0]);
//	BezierControlPoints_Assign(60 - fabs(deltaYaw), 250, &p[3][3]);
//}

//float AngleConversion_2generation(float PreAngle, int MotorID)   //input MotorID to choose motor for conversion, e.g 1:motor_1
//{
//	float AfterConversion;
//	switch (MotorID)
//	{
//	case 1:
//		AfterConversion = -(180 - 68.58 - PreAngle);
//		if (AfterConversion < -200)
//			return AfterConversion + 360;
//		return AfterConversion;
//	case 2:
//		AfterConversion = (68.58 - PreAngle);
//		return AfterConversion;
//	case 3:
//		AfterConversion = -(180 - 68.58 - PreAngle);
//		if (AfterConversion < -200)
//			return AfterConversion + 360;
//		return AfterConversion;
//	case 4:
//		AfterConversion = (68.58 - PreAngle);
//		return AfterConversion;
//	case 5:
//		AfterConversion = (180 - 68.58 - PreAngle);
//		if (AfterConversion > 200)
//			return AfterConversion - 360;
//		return AfterConversion;
//	case 6:
//		AfterConversion = -(68.58 - PreAngle);
//		return AfterConversion;
//	case 7:
//		AfterConversion = (180 - 68.58 - PreAngle);
//		if (AfterConversion > 200)
//			return AfterConversion - 360;
//		return AfterConversion;
//	case 8:
//		AfterConversion = -(68.58 - PreAngle);
//		return AfterConversion;
//	default:break;
//	}
//}
//void PitchUp(float pitch)
//{
//	float deltaPitch = PITCHFACTOR * pitch;

//	BezierControlPoints_Assign(-80 , 270+ fabs(deltaPitch), &p[0][0]);
//	BezierControlPoints_Assign(80  , 270+ fabs(deltaPitch), &p[0][3]);
//	
//	BezierControlPoints_Assign(-80 , 270- fabs(deltaPitch), &p[1][0]);
//	BezierControlPoints_Assign(80  , 270- fabs(deltaPitch), &p[1][3]);
//	
//	BezierControlPoints_Assign(-80 , 270+ fabs(deltaPitch), &p[2][0]);
//	BezierControlPoints_Assign(80  , 270+ fabs(deltaPitch), &p[2][3]);
//	
//	BezierControlPoints_Assign(-80 , 270- fabs(deltaPitch), &p[3][0]);
//	BezierControlPoints_Assign(80  , 270- fabs(deltaPitch), &p[3][3]);
//}	
//void PitchDown(float pitch)
//{
//	float deltaPitch = PITCHFACTOR * pitch;
//	BezierControlPoints_Assign(-80 , 270- fabs(deltaPitch), &p[0][0]);
//	BezierControlPoints_Assign(80  , 270- fabs(deltaPitch), &p[0][3]);
//	
//	BezierControlPoints_Assign(-80 , 270+ fabs(deltaPitch), &p[1][0]);
//	BezierControlPoints_Assign(80  , 270+ fabs(deltaPitch), &p[1][3]);

//	BezierControlPoints_Assign(-80 , 270- fabs(deltaPitch), &p[2][0]);
//	BezierControlPoints_Assign(80  , 270- fabs(deltaPitch), &p[2][3]);
//	
//	BezierControlPoints_Assign(-80 , 270+ fabs(deltaPitch), &p[3][0]);
//	BezierControlPoints_Assign(80  , 270+ fabs(deltaPitch), &p[3][3]);
//}
//	
//	void chassisOneCircle(void)
//	{
//		Navigation();
//	}
/******************* (C) COPYRIGHT 2020 HUST-Robocon-Team *****END OF FILE****/
