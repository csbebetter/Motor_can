#include "Parallel_Leg.h"

void DeflectToLeft(float DS){
	S0 -= DS;
	S1 += DS;
	S2 += DS;
	S3 -= DS;
	OnlineRegulate();
	Up_Slope_Regulate();
}

void DeflectToRight(float DS){
	S0 += DS;
	S1 -= DS;
	S2 -= DS;
	S3 += DS;
	OnlineRegulate();
	Up_Slope_Regulate();
}

void OnlineRegulate(void){
	for (int count = 0; count < Trot_LENGTH / 2; count++) {
		float t = count * walk_delay;
		FOOTPOINT_X[0][count] = S0 * (t / Tm - 0.5 / PI * sin(2 * PI * t / Tm));
		FOOTPOINT_Y[0][count] = H * (0.5 - 0.5 * cos(2 * PI * t / Tm));
		FOOTPOINT_X[1][count] = S1 * (1 - t / Tm + 0.5 / PI * sin(2 * PI * t / Tm));
		FOOTPOINT_Y[1][count] = -HB * (0.5 - 0.5 * cos(2 * PI * t / Tm));
		FOOTPOINT_X[2][count] = S2 * (t / Tm - 0.5 / PI * sin(2 * PI * t / Tm));
		FOOTPOINT_Y[2][count] = H * (0.5 - 0.5 * cos(2 * PI * t / Tm));
		FOOTPOINT_X[3][count] = S3 * (1 - t / Tm + 0.5 / PI * sin(2 * PI * t / Tm));
		FOOTPOINT_Y[3][count] = -HB * (0.5 - 0.5 * cos(2 * PI * t / Tm));
	}
	for (int count = 0; count < Trot_LENGTH / 2; count++) {
		float t = count * walk_delay;
		FOOTPOINT_X[0][count + Trot_LENGTH / 2] = S0 * (1 - t / Tm + 0.5 / PI * sin(2 * PI * t / Tm));
		FOOTPOINT_Y[0][count + Trot_LENGTH / 2] = -HB * (0.5 - 0.5 * cos(2 * PI * t / Tm));
		FOOTPOINT_X[1][count + Trot_LENGTH / 2] = S1 * (t / Tm - 0.5 / PI * sin(2 * PI * t / Tm));
		FOOTPOINT_Y[1][count + Trot_LENGTH / 2] = H * (0.5 - 0.5 * cos(2 * PI * t / Tm));
		FOOTPOINT_X[2][count + Trot_LENGTH / 2] = S2 * (1 - t / Tm + 0.5 / PI * sin(2 * PI * t / Tm));
		FOOTPOINT_Y[2][count + Trot_LENGTH / 2] = -HB * (0.5 - 0.5 * cos(2 * PI * t / Tm));
		FOOTPOINT_X[3][count + Trot_LENGTH / 2] = S3 * (t / Tm - 0.5 / PI * sin(2 * PI * t / Tm));
		FOOTPOINT_Y[3][count + Trot_LENGTH / 2] = H * (0.5 - 0.5 * cos(2 * PI * t / Tm));
	}
	float xF, yF, virtual_L, rodAF_angel, ABF_angel;
	float xB, yB, xC, yC, BAC_angel, rodAC_angel, L_AF;
	L_AF = sqrt(pow(L1, 2) + pow((L2 + L4), 2) - (pow(L1, 2) + pow(L2, 2) - pow(L3, 2)) * (L2 + L4) / L2);
	for (int count = 0; count < Trot_LENGTH; count++) {
		xF = FOOTPOINT_X[0][count] - S0 / 2;//F点坐标
		yF = FOOTPOINT_Y[0][count] - L_AF;
		virtual_L = pow(pow(xF,2) + pow(yF,2),0.5);
		rodAF_angel = asin(xF / virtual_L); // AF与竖直方向夹角（逆时针为正）
		ABF_angel = acos((pow(virtual_L,2) + pow(L1,2) - pow(L2 + L4,2)) / 2 / L1 / virtual_L); //AF与AB夹角
		MOTOR_ANGLE[0][count] = ABF_angel - rodAF_angel;
		xB = -L1 * sin(MOTOR_ANGLE[0][count]);//B点坐标
		yB = -L1 * cos(MOTOR_ANGLE[0][count]);
		xC = (L4 * xB + L2 * xF) / (L2 + L4);//C点坐标
		yC = (L4 * yB + L2 * yF) / (L2 + L4);
		BAC_angel = acos((xB * xC + yB * yC) / sqrt(pow(xB,2) + pow(yB,2)) / sqrt(pow(xC,2) + pow(yC,2)));
		rodAC_angel = atan(- xC / yC); // AC与竖直方向夹角（逆时针为正）
		MOTOR_ANGLE[1][count] = BAC_angel + rodAC_angel;
	
		xF = FOOTPOINT_X[1][count] - S1 / 2;
		yF = FOOTPOINT_Y[1][count] - L_AF;
		virtual_L = pow(pow(xF, 2) + pow(yF, 2), 0.5);
		rodAF_angel = asin(xF / virtual_L); // AF与竖直方向夹角（逆时针为正）
		ABF_angel = acos((pow(virtual_L, 2) + pow(L1, 2) - pow(L2 + L4, 2)) / 2 / L1 / virtual_L); //AF与AB夹角
		MOTOR_ANGLE[2][count] = ABF_angel - rodAF_angel;
		xB = -L1 * sin(MOTOR_ANGLE[2][count]);//B点坐标
		yB = -L1 * cos(MOTOR_ANGLE[2][count]);
		xC = (L4 * xB + L2 * xF) / (L2 + L4);//C点坐标
		yC = (L4 * yB + L2 * yF) / (L2 + L4);
		BAC_angel = acos((xB * xC + yB * yC) / sqrt(pow(xB, 2) + pow(yB, 2)) / sqrt(pow(xC, 2) + pow(yC, 2)));
		rodAC_angel = atan(-xC / yC); // AC与竖直方向夹角（逆时针为正）
		MOTOR_ANGLE[3][count] = BAC_angel + rodAC_angel;

		xF = FOOTPOINT_X[2][count] - S2 / 2;
		yF = FOOTPOINT_Y[2][count] - L_AF;
		virtual_L = pow(pow(xF, 2) + pow(yF, 2), 0.5);
		rodAF_angel = asin(xF / virtual_L); // AF与竖直方向夹角（逆时针为正）
		ABF_angel = acos((pow(virtual_L, 2) + pow(L1, 2) - pow(L2 + L4, 2)) / 2 / L1 / virtual_L); //AF与AB夹角
		MOTOR_ANGLE[4][count] = ABF_angel - rodAF_angel;
		xB = -L1 * sin(MOTOR_ANGLE[4][count]);//B点坐标
		yB = -L1 * cos(MOTOR_ANGLE[4][count]);
		xC = (L4 * xB + L2 * xF) / (L2 + L4);//C点坐标
		yC = (L4 * yB + L2 * yF) / (L2 + L4);
		BAC_angel = acos((xB * xC + yB * yC) / sqrt(pow(xB, 2) + pow(yB, 2)) / sqrt(pow(xC, 2) + pow(yC, 2)));
		rodAC_angel = atan(-xC / yC); // AC与竖直方向夹角（逆时针为正）
		MOTOR_ANGLE[5][count] = BAC_angel + rodAC_angel;

		xF = FOOTPOINT_X[3][count] - S3 / 2;
		yF = FOOTPOINT_Y[3][count] - L_AF;
		virtual_L = pow(pow(xF, 2) + pow(yF, 2), 0.5);
		rodAF_angel = asin(xF / virtual_L); // AF与竖直方向夹角（逆时针为正）
		ABF_angel = acos((pow(virtual_L, 2) + pow(L1, 2) - pow(L2 + L4, 2)) / 2 / L1 / virtual_L); //AF与AB夹角
		MOTOR_ANGLE[6][count] = ABF_angel - rodAF_angel;
		xB = -L1 * sin(MOTOR_ANGLE[6][count]);//B点坐标
		yB = -L1 * cos(MOTOR_ANGLE[6][count]);
		xC = (L4 * xB + L2 * xF) / (L2 + L4);//C点坐标
		yC = (L4 * yB + L2 * yF) / (L2 + L4);
		BAC_angel = acos((xB * xC + yB * yC) / sqrt(pow(xB, 2) + pow(yB, 2)) / sqrt(pow(xC, 2) + pow(yC, 2)));
		rodAC_angel = atan(-xC / yC); // AC与竖直方向夹角（逆时针为正）
		MOTOR_ANGLE[7][count] = BAC_angel + rodAC_angel;
		
		//弧度制 -> 角度制
		for(int i = 0; i < Trot_LENGTH; i++){
			MOTOR_ANGLE[i][count] = MOTOR_ANGLE[i][count] * 180 / PI;
		}
	}
	//传递电机角
	for(int i = 0; i < 8; i++){
		for(int j = 0; j < Trot_LENGTH; j++) {
			MotorTrot[i][j] = MOTOR_ANGLE[i][j];
		}
	}
}

void TITA_Regulate(float SS,float SH,float SHB){
	for (int count = 0; count < Trot_LENGTH / 2; count++) {
		float t = count * walk_delay;
		FOOTPOINT_X[0][count] = SS * (t / Tm - 0.5 / PI * sin(2 * PI * t / Tm));
		FOOTPOINT_Y[0][count] = SH * (0.5 - 0.5 * cos(2 * PI * t / Tm));
		FOOTPOINT_X[1][count] = SS * (1 - t / Tm + 0.5 / PI * sin(2 * PI * t / Tm));
		FOOTPOINT_Y[1][count] = -SHB * (0.5 - 0.5 * cos(2 * PI * t / Tm));
		FOOTPOINT_X[2][count] = SS * (t / Tm - 0.5 / PI * sin(2 * PI * t / Tm));
		FOOTPOINT_Y[2][count] = SH * (0.5 - 0.5 * cos(2 * PI * t / Tm));
		FOOTPOINT_X[3][count] = SS * (1 - t / Tm + 0.5 / PI * sin(2 * PI * t / Tm));
		FOOTPOINT_Y[3][count] = -SHB * (0.5 - 0.5 * cos(2 * PI * t / Tm));
	}
	for (int count = 0; count < Trot_LENGTH / 2; count++) {
		float t = count * walk_delay;
		FOOTPOINT_X[0][count + Trot_LENGTH / 2] = SS * (1 - t / Tm + 0.5 / PI * sin(2 * PI * t / Tm));
		FOOTPOINT_Y[0][count + Trot_LENGTH / 2] = -SHB * (0.5 - 0.5 * cos(2 * PI * t / Tm));
		FOOTPOINT_X[1][count + Trot_LENGTH / 2] = SS * (t / Tm - 0.5 / PI * sin(2 * PI * t / Tm));
		FOOTPOINT_Y[1][count + Trot_LENGTH / 2] = SH * (0.5 - 0.5 * cos(2 * PI * t / Tm));
		FOOTPOINT_X[2][count + Trot_LENGTH / 2] = SS * (1 - t / Tm + 0.5 / PI * sin(2 * PI * t / Tm));
		FOOTPOINT_Y[2][count + Trot_LENGTH / 2] = -SHB * (0.5 - 0.5 * cos(2 * PI * t / Tm));
		FOOTPOINT_X[3][count + Trot_LENGTH / 2] = SS * (t / Tm - 0.5 / PI * sin(2 * PI * t / Tm));
		FOOTPOINT_Y[3][count + Trot_LENGTH / 2] = SH * (0.5 - 0.5 * cos(2 * PI * t / Tm));
	}
	float xF, yF, virtual_L, rodAF_angel, ABF_angel;
	float xB, yB, xC, yC, BAC_angel, rodAC_angel, L_AF;
	L_AF = sqrt(pow(L1, 2) + pow((L2 + L4), 2) - (pow(L1, 2) + pow(L2, 2) - pow(L3, 2)) * (L2 + L4) / L2);
	for (int count = 0; count < Trot_LENGTH; count++) {
		xF = FOOTPOINT_X[0][count] - SS / 2;//F点坐标
		yF = FOOTPOINT_Y[0][count] - L_AF;
		virtual_L = pow(pow(xF,2) + pow(yF,2),0.5);
		rodAF_angel = asin(xF / virtual_L); // AF与竖直方向夹角（逆时针为正）
		ABF_angel = acos((pow(virtual_L,2) + pow(L1,2) - pow(L2 + L4,2)) / 2 / L1 / virtual_L); //AF与AB夹角
		MOTOR_ANGLE[0][count] = ABF_angel - rodAF_angel;
		xB = -L1 * sin(MOTOR_ANGLE[0][count]);//B点坐标
		yB = -L1 * cos(MOTOR_ANGLE[0][count]);
		xC = (L4 * xB + L2 * xF) / (L2 + L4);//C点坐标
		yC = (L4 * yB + L2 * yF) / (L2 + L4);
		BAC_angel = acos((xB * xC + yB * yC) / sqrt(pow(xB,2) + pow(yB,2)) / sqrt(pow(xC,2) + pow(yC,2)));
		rodAC_angel = atan(- xC / yC); // AC与竖直方向夹角（逆时针为正）
		MOTOR_ANGLE[1][count] = BAC_angel + rodAC_angel;
	
		xF = FOOTPOINT_X[1][count] - SS / 2;
		yF = FOOTPOINT_Y[1][count] - L_AF;
		virtual_L = pow(pow(xF, 2) + pow(yF, 2), 0.5);
		rodAF_angel = asin(xF / virtual_L); // AF与竖直方向夹角（逆时针为正）
		ABF_angel = acos((pow(virtual_L, 2) + pow(L1, 2) - pow(L2 + L4, 2)) / 2 / L1 / virtual_L); //AF与AB夹角
		MOTOR_ANGLE[2][count] = ABF_angel - rodAF_angel;
		xB = -L1 * sin(MOTOR_ANGLE[2][count]);//B点坐标
		yB = -L1 * cos(MOTOR_ANGLE[2][count]);
		xC = (L4 * xB + L2 * xF) / (L2 + L4);//C点坐标
		yC = (L4 * yB + L2 * yF) / (L2 + L4);
		BAC_angel = acos((xB * xC + yB * yC) / sqrt(pow(xB, 2) + pow(yB, 2)) / sqrt(pow(xC, 2) + pow(yC, 2)));
		rodAC_angel = atan(-xC / yC); // AC与竖直方向夹角（逆时针为正）
		MOTOR_ANGLE[3][count] = BAC_angel + rodAC_angel;

		xF = FOOTPOINT_X[2][count] - SS / 2;
		yF = FOOTPOINT_Y[2][count] - L_AF;
		virtual_L = pow(pow(xF, 2) + pow(yF, 2), 0.5);
		rodAF_angel = asin(xF / virtual_L); // AF与竖直方向夹角（逆时针为正）
		ABF_angel = acos((pow(virtual_L, 2) + pow(L1, 2) - pow(L2 + L4, 2)) / 2 / L1 / virtual_L); //AF与AB夹角
		MOTOR_ANGLE[4][count] = ABF_angel - rodAF_angel;
		xB = -L1 * sin(MOTOR_ANGLE[4][count]);//B点坐标
		yB = -L1 * cos(MOTOR_ANGLE[4][count]);
		xC = (L4 * xB + L2 * xF) / (L2 + L4);//C点坐标
		yC = (L4 * yB + L2 * yF) / (L2 + L4);
		BAC_angel = acos((xB * xC + yB * yC) / sqrt(pow(xB, 2) + pow(yB, 2)) / sqrt(pow(xC, 2) + pow(yC, 2)));
		rodAC_angel = atan(-xC / yC); // AC与竖直方向夹角（逆时针为正）
		MOTOR_ANGLE[5][count] = BAC_angel + rodAC_angel;

		xF = FOOTPOINT_X[3][count] - SS / 2;
		yF = FOOTPOINT_Y[3][count] - L_AF;
		virtual_L = pow(pow(xF, 2) + pow(yF, 2), 0.5);
		rodAF_angel = asin(xF / virtual_L); // AF与竖直方向夹角（逆时针为正）
		ABF_angel = acos((pow(virtual_L, 2) + pow(L1, 2) - pow(L2 + L4, 2)) / 2 / L1 / virtual_L); //AF与AB夹角
		MOTOR_ANGLE[6][count] = ABF_angel - rodAF_angel;
		xB = -L1 * sin(MOTOR_ANGLE[6][count]);//B点坐标
		yB = -L1 * cos(MOTOR_ANGLE[6][count]);
		xC = (L4 * xB + L2 * xF) / (L2 + L4);//C点坐标
		yC = (L4 * yB + L2 * yF) / (L2 + L4);
		BAC_angel = acos((xB * xC + yB * yC) / sqrt(pow(xB, 2) + pow(yB, 2)) / sqrt(pow(xC, 2) + pow(yC, 2)));
		rodAC_angel = atan(-xC / yC); // AC与竖直方向夹角（逆时针为正）
		MOTOR_ANGLE[7][count] = BAC_angel + rodAC_angel;
		
		//弧度制 -> 角度制
		for(int i = 0; i < Trot_LENGTH; i++){
			MOTOR_ANGLE[i][count] = MOTOR_ANGLE[i][count] * 180 / PI;
		}
	}
	//传递电机角
	for(int i = 0; i < 8; i++){
		for(int j = 0; j < Trot_LENGTH; j++) {
			MotorTITA[i][j] = MOTOR_ANGLE[i][j];
		}
	}
}

void WalkRegulate(void){
	//计算ZMP
	float I1_X, I1_Y, I2_X, I2_Y, a, b;
	I1_X = half_width * SW / (half_length * 4 - SW);
	I1_Y = half_length * SW / (half_length * 4 - SW);
	a = sqrt(pow((I1_X + half_width), 2) + pow((I1_Y + half_length), 2));
	b = sqrt(pow((I1_X + half_width), 2) + pow((I1_Y - half_length), 2));
	YM1 = (half_length * a - half_length * b + half_length * 2 * I1_Y) / (a + b + half_length * 2);
	I2_X = half_width * SW / (SW - half_length * 4);
	I2_Y = (SW - 2 * half_length) / (2 * (SW - half_length * 4));
	a = sqrt(pow((I2_X - half_width), 2) + pow((I2_Y - (2 * half_length + SW) / 2), 2));
	b = sqrt(pow((I2_X - half_width), 2) + pow((I2_Y - (SW - 2 * half_length) / 2), 2));
	YM2 = ((SW - 2 * half_length) / 2 * a + (2 * half_length + SW) / 2 * b + half_length * 2 * I2_Y) / (a + b + half_length * 2);
	//YM1 = YM1 + YM1_ADD;  //补偿量 用于调试
	//YM2 = YM2 + YM2_ADD;  
	YM1 = YM1_ADD;  //补偿量 用于调试
	YM2 = YM2_ADD;  
	
	float dt = Tm / (LENGTH - 1);
	for (int count = 0; count < Support1; count++) {	//support1
		FOOTPOINT_XW[0][count] = (float)-count * (YM1 / Support1);
		FOOTPOINT_YW[0][count] = 0;
		FOOTPOINT_XW[1][count] = (float)-count * (YM1 / Support1);
		FOOTPOINT_YW[1][count] = 0;
		FOOTPOINT_XW[2][count] = (float)-count * (YM1 / Support1);
		FOOTPOINT_YW[2][count] = 0;
		FOOTPOINT_XW[3][count] = (float)-count * (YM1 / Support1);
		FOOTPOINT_YW[3][count] = 0;
	}
	for (int count = Support1; count < (Support1 + LENGTH); count++) {	//first step
		float t = (count - Support1) * dt;
		FOOTPOINT_XW[0][count] = FOOTPOINT_XW[0][Support1 - 1];
		FOOTPOINT_YW[0][count] = FOOTPOINT_YW[0][Support1 - 1] + HJ;
		FOOTPOINT_XW[1][count] = FOOTPOINT_XW[1][Support1 - 1];
		FOOTPOINT_YW[1][count] = FOOTPOINT_YW[1][Support1 - 1];
		FOOTPOINT_XW[2][count] = SW * (t / Tm - 0.5 / PI * sin(2 * PI * t / Tm)) - YM1;
		FOOTPOINT_YW[2][count] = HW * (0.5 - 0.5 * cos(2 * PI * t / Tm));
		FOOTPOINT_XW[3][count] = FOOTPOINT_XW[3][Support1 - 1];
		FOOTPOINT_YW[3][count] = FOOTPOINT_YW[3][Support1 - 1] + HJ;
	}
	for (int count = (Support1 + LENGTH); count < (Support1 + 2 * LENGTH); count++) {	//second step
		float t = (count - (Support1 + LENGTH)) * dt;
		FOOTPOINT_XW[0][count] = FOOTPOINT_XW[0][Support1 - 1 + LENGTH];
		FOOTPOINT_YW[0][count] = FOOTPOINT_YW[0][Support1 - 1 + LENGTH];
		FOOTPOINT_XW[1][count] = SW * (t / Tm - 0.5 / PI * sin(2 * PI * t / Tm)) - YM1;
		FOOTPOINT_YW[1][count] = HW * (0.5 - 0.5 * cos(2 * PI * t / Tm));
		FOOTPOINT_XW[2][count] = FOOTPOINT_XW[2][Support1 - 1 + LENGTH];
		FOOTPOINT_YW[2][count] = FOOTPOINT_YW[2][Support1 - 1 + LENGTH];
		FOOTPOINT_XW[3][count] = FOOTPOINT_XW[3][Support1 - 1 + LENGTH];
		FOOTPOINT_YW[3][count] = FOOTPOINT_YW[3][Support1 - 1 + LENGTH];
	}
	for (int count = (Support1 + 2 * LENGTH); count < (Support13 + 2 * LENGTH); count++) { // support3
		FOOTPOINT_XW[0][count] = FOOTPOINT_XW[0][Support1 - 1 + 2 * LENGTH] - (float)(count - (Support1 - 1 + 2 * LENGTH)) * YM2 / (Support13 - Support1);
		FOOTPOINT_YW[0][count] = FOOTPOINT_YW[0][Support1 - 1 + 2 * LENGTH] - HJ;
		FOOTPOINT_XW[1][count] = FOOTPOINT_XW[1][Support1 - 1 + 2 * LENGTH] - (float)(count - (Support1 - 1 + 2 * LENGTH)) * YM2 / (Support13 - Support1);
		FOOTPOINT_YW[1][count] = FOOTPOINT_YW[1][Support1 - 1 + 2 * LENGTH];
		FOOTPOINT_XW[2][count] = FOOTPOINT_XW[2][Support1 - 1 + 2 * LENGTH] - (float)(count - (Support1 - 1 + 2 * LENGTH)) * YM2 / (Support13 - Support1);
		FOOTPOINT_YW[2][count] = FOOTPOINT_YW[2][Support1 - 1 + 2 * LENGTH];
		FOOTPOINT_XW[3][count] = FOOTPOINT_XW[3][Support1 - 1 + 2 * LENGTH] - (float)(count - (Support1 - 1 + 2 * LENGTH)) * YM2 / (Support13 - Support1);
		FOOTPOINT_YW[3][count] = FOOTPOINT_YW[3][Support1 - 1 + 2 * LENGTH] - HJ;
	}
	for (int count = (Support13 + 2 * LENGTH); count < (Support13 + 3 * LENGTH); count++) {	// third step
		float t = (count - (Support13 + 2 * LENGTH)) * dt;
		FOOTPOINT_XW[0][count] = FOOTPOINT_XW[0][Support13 - 1 + 2 * LENGTH];
		FOOTPOINT_YW[0][count] = FOOTPOINT_YW[0][Support13 - 1 + 2 * LENGTH];
		FOOTPOINT_XW[1][count] = FOOTPOINT_XW[1][Support13 - 1 + 2 * LENGTH];
		FOOTPOINT_YW[1][count] = FOOTPOINT_YW[1][Support13 - 1 + 2 * LENGTH] + HJ;
		FOOTPOINT_XW[2][count] = FOOTPOINT_XW[2][Support13 - 1 + 2 * LENGTH];
		FOOTPOINT_YW[2][count] = FOOTPOINT_YW[2][Support13 - 1 + 2 * LENGTH] + HJ;
		FOOTPOINT_XW[3][count] = SW * (t / Tm - 0.5 / PI * sin(2 * PI * t / Tm)) - YM1 - YM2;
		FOOTPOINT_YW[3][count] = HW * (0.5 - 0.5 * cos(2 * PI * t / Tm));
	}
	for (int count = (Support13 + 3 * LENGTH); count < (Support13 + 4 * LENGTH); count++) {	// forth step
		float t = (count - (Support13 + 3 * LENGTH)) * dt;
		FOOTPOINT_XW[0][count] = SW * (t / Tm - 0.5 / PI * sin(2 * PI * t / Tm)) - YM1 - YM2;
		FOOTPOINT_YW[0][count] = HW * (0.5 - 0.5 * cos(2 * PI * t / Tm));
		FOOTPOINT_XW[1][count] = FOOTPOINT_XW[1][Support13 - 1 + 3 * LENGTH];
		FOOTPOINT_YW[1][count] = FOOTPOINT_YW[1][Support13 - 1 + 3 * LENGTH];
		FOOTPOINT_XW[2][count] = FOOTPOINT_XW[2][Support13 - 1 + 3 * LENGTH];
		FOOTPOINT_YW[2][count] = FOOTPOINT_YW[2][Support13 - 1 + 3 * LENGTH];
		FOOTPOINT_XW[3][count] = FOOTPOINT_XW[3][Support13 - 1 + 3 * LENGTH];
		FOOTPOINT_YW[3][count] = FOOTPOINT_YW[3][Support13 - 1 + 3 * LENGTH];
	}
	for (int count = (Support13 + 4 * LENGTH); count < (Support123 + 4 * LENGTH); count++) {	// support2
		FOOTPOINT_XW[0][count] = (S - YM1 - YM2) * (1 - (float)(count - (Support13 - 1 + 4 * LENGTH)) / (Support123 - Support13));
		FOOTPOINT_YW[0][count] = 0;
		FOOTPOINT_XW[1][count] = (S - YM1 - YM2) * (1 - (float)(count - (Support13 - 1 + 4 * LENGTH)) / (Support123 - Support13));
		FOOTPOINT_YW[1][count] = 0;
		FOOTPOINT_XW[2][count] = (S - YM1 - YM2) * (1 - (float)(count - (Support13 - 1 + 4 * LENGTH)) / (Support123 - Support13));
		FOOTPOINT_YW[2][count] = 0;
		FOOTPOINT_XW[3][count] = (S - YM1 - YM2) * (1 - (float)(count - (Support13 - 1 + 4 * LENGTH)) / (Support123 - Support13));
		FOOTPOINT_YW[3][count] = 0;
	}
	
	float xF, yF, virtual_L, rodAF_angel, ABF_angel;
	float xB, yB, xC, yC, BAC_angel, rodAC_angel, L_AF;
	L_AF = sqrt(pow(L1, 2) + pow((L2 + L4), 2) - (pow(L1, 2) + pow(L2, 2) - pow(L3W, 2)) * (L2 + L4) / L2);
	for (int count = 0; count < Walk_LENGTH; count++) {
		xF = FOOTPOINT_XW[0][count];//F??????
		yF = FOOTPOINT_YW[0][count] - L_AF;
		virtual_L = pow(pow(xF, 2) + pow(yF, 2), 0.5);
		rodAF_angel = asin(xF / virtual_L); // AF??????????н????????????
		ABF_angel = acos((pow(virtual_L, 2) + pow(L1, 2) - pow(L2 + L4, 2)) / 2 / L1 / virtual_L); //AF??AB?н?
		MOTOR_ANGLEW[0][count] = ABF_angel - rodAF_angel;
		xB = -L1 * sin(MOTOR_ANGLEW[0][count]);//B??????
		yB = -L1 * cos(MOTOR_ANGLEW[0][count]);
		xC = (L4 * xB + L2 * xF) / (L2 + L4);//C??????
		yC = (L4 * yB + L2 * yF) / (L2 + L4);
		BAC_angel = acos((xB * xC + yB * yC) / sqrt(pow(xB, 2) + pow(yB, 2)) / sqrt(pow(xC, 2) + pow(yC, 2)));
		rodAC_angel = atan(-xC / yC); // AC??????????н????????????
		MOTOR_ANGLEW[1][count] = BAC_angel + rodAC_angel;

		xF = FOOTPOINT_XW[1][count];
		yF = FOOTPOINT_YW[1][count] - L_AF;
		virtual_L = pow(pow(xF, 2) + pow(yF, 2), 0.5);
		rodAF_angel = asin(xF / virtual_L); // AF??????????н????????????
		ABF_angel = acos((pow(virtual_L, 2) + pow(L1, 2) - pow(L2 + L4, 2)) / 2 / L1 / virtual_L); //AF??AB?н?
		MOTOR_ANGLEW[2][count] = ABF_angel - rodAF_angel;
		xB = -L1 * sin(MOTOR_ANGLEW[2][count]);//B??????
		yB = -L1 * cos(MOTOR_ANGLEW[2][count]);
		xC = (L4 * xB + L2 * xF) / (L2 + L4);//C??????
		yC = (L4 * yB + L2 * yF) / (L2 + L4);
		BAC_angel = acos((xB * xC + yB * yC) / sqrt(pow(xB, 2) + pow(yB, 2)) / sqrt(pow(xC, 2) + pow(yC, 2)));
		rodAC_angel = atan(-xC / yC); // AC??????????н????????????
		MOTOR_ANGLEW[3][count] = BAC_angel + rodAC_angel;

		xF = FOOTPOINT_XW[2][count];
		yF = FOOTPOINT_YW[2][count] - L_AF;
		virtual_L = pow(pow(xF, 2) + pow(yF, 2), 0.5);
		rodAF_angel = asin(xF / virtual_L); // AF??????????н????????????
		ABF_angel = acos((pow(virtual_L, 2) + pow(L1, 2) - pow(L2 + L4, 2)) / 2 / L1 / virtual_L); //AF??AB?н?
		MOTOR_ANGLEW[4][count] = ABF_angel - rodAF_angel;
		xB = -L1 * sin(MOTOR_ANGLEW[4][count]);//B??????
		yB = -L1 * cos(MOTOR_ANGLEW[4][count]);
		xC = (L4 * xB + L2 * xF) / (L2 + L4);//C??????
		yC = (L4 * yB + L2 * yF) / (L2 + L4);
		BAC_angel = acos((xB * xC + yB * yC) / sqrt(pow(xB, 2) + pow(yB, 2)) / sqrt(pow(xC, 2) + pow(yC, 2)));
		rodAC_angel = atan(-xC / yC); // AC??????????н????????????
		MOTOR_ANGLEW[5][count] = BAC_angel + rodAC_angel;

		xF = FOOTPOINT_XW[3][count];
		yF = FOOTPOINT_YW[3][count] - L_AF;
		virtual_L = pow(pow(xF, 2) + pow(yF, 2), 0.5);
		rodAF_angel = asin(xF / virtual_L); // AF??????????н????????????
		ABF_angel = acos((pow(virtual_L, 2) + pow(L1, 2) - pow(L2 + L4, 2)) / 2 / L1 / virtual_L); //AF??AB?н?
		MOTOR_ANGLEW[6][count] = ABF_angel - rodAF_angel;
		xB = -L1 * sin(MOTOR_ANGLEW[6][count]);//B??????
		yB = -L1 * cos(MOTOR_ANGLEW[6][count]);
		xC = (L4 * xB + L2 * xF) / (L2 + L4);//C??????
		yC = (L4 * yB + L2 * yF) / (L2 + L4);
		BAC_angel = acos((xB * xC + yB * yC) / sqrt(pow(xB, 2) + pow(yB, 2)) / sqrt(pow(xC, 2) + pow(yC, 2)));
		rodAC_angel = atan(-xC / yC); // AC??????????н????????????
		MOTOR_ANGLEW[7][count] = BAC_angel + rodAC_angel;
	
		//弧度制 -> 角度制
		for(int i = 0; i < Walk_LENGTH; i++){
			MOTOR_ANGLEW[i][count] = MOTOR_ANGLEW[i][count] * 180 / PI;
		}
	}
	//传递电机角
	for(int i = 0; i < 8; i++){
		for(int j = 0; j < Walk_LENGTH; j++) {
			MotorWalk[i][j] = MOTOR_ANGLEW[i][j];
		}
	}
	
}
void Slope_Regulate(float L3_Front,float L3_Back,float H_Front,float H_Back){
	for (int count = 0; count < Trot_LENGTH / 2; count++) {
		float t = count * walk_delay;
		FOOTPOINT_X[0][count] = S0 * (t / Tm - 0.5 / PI * sin(2 * PI * t / Tm));
		FOOTPOINT_Y[0][count] = H_Front * (0.5 - 0.5 * cos(2 * PI * t / Tm));
		FOOTPOINT_X[1][count] = S1 * (1 - t / Tm + 0.5 / PI * sin(2 * PI * t / Tm));
		FOOTPOINT_Y[1][count] = -HB * (0.5 - 0.5 * cos(2 * PI * t / Tm));
		FOOTPOINT_X[2][count] = S2 * (t / Tm - 0.5 / PI * sin(2 * PI * t / Tm));
		FOOTPOINT_Y[2][count] = H_Back * (0.5 - 0.5 * cos(2 * PI * t / Tm));
		FOOTPOINT_X[3][count] = S3 * (1 - t / Tm + 0.5 / PI * sin(2 * PI * t / Tm));
		FOOTPOINT_Y[3][count] = -HB * (0.5 - 0.5 * cos(2 * PI * t / Tm));
	}
	for (int count = 0; count < Trot_LENGTH / 2; count++) {
		float t = count * walk_delay;
		FOOTPOINT_X[0][count + Trot_LENGTH / 2] = S0 * (1 - t / Tm + 0.5 / PI * sin(2 * PI * t / Tm));
		FOOTPOINT_Y[0][count + Trot_LENGTH / 2] = -HB * (0.5 - 0.5 * cos(2 * PI * t / Tm));
		FOOTPOINT_X[1][count + Trot_LENGTH / 2] = S1 * (t / Tm - 0.5 / PI * sin(2 * PI * t / Tm));
		FOOTPOINT_Y[1][count + Trot_LENGTH / 2] = H_Front * (0.5 - 0.5 * cos(2 * PI * t / Tm));
		FOOTPOINT_X[2][count + Trot_LENGTH / 2] = S2 * (1 - t / Tm + 0.5 / PI * sin(2 * PI * t / Tm));
		FOOTPOINT_Y[2][count + Trot_LENGTH / 2] = -HB * (0.5 - 0.5 * cos(2 * PI * t / Tm));
		FOOTPOINT_X[3][count + Trot_LENGTH / 2] = S3 * (t / Tm - 0.5 / PI * sin(2 * PI * t / Tm));
		FOOTPOINT_Y[3][count + Trot_LENGTH / 2] = H_Back * (0.5 - 0.5 * cos(2 * PI * t / Tm));
	}
	float xF, yF_Front, yF_Back,virtual_L_Front, virtual_L_Back,rodAF_angel_Front,rodAF_angel_Back, ABF_angel_Front,ABF_angel_Back;
	float xB, yB, xC, yC, BAC_angel, rodAC_angel, L_AF_Front,L_AF_Back;
	L_AF_Front = sqrt(pow(L1, 2) + pow((L2 + L4), 2) - (pow(L1, 2) + pow(L2, 2) - pow(L3_Front, 2)) * (L2 + L4) / L2);
  L_AF_Back   =  sqrt(pow(L1, 2) + pow((L2 + L4), 2) - (pow(L1, 2) + pow(L2, 2) - pow(L3_Back, 2)) * (L2 + L4) / L2);
	for (int count = 0; count < Trot_LENGTH; count++) {
		xF = FOOTPOINT_X[0][count] - S0 / 2;//F点坐标
		yF_Front = FOOTPOINT_Y[0][count] - L_AF_Front;
		virtual_L_Front = pow(pow(xF,2) + pow(yF_Front,2),0.5);	
		rodAF_angel_Front = asin(xF / virtual_L_Front); // AF与竖直方向夹角（逆时针为正）		
		ABF_angel_Front = acos((pow(virtual_L_Front,2) + pow(L1,2) - pow(L2 + L4,2)) / 2 / L1 / virtual_L_Front); //AF与AB夹角
		MOTOR_ANGLE[0][count] = ABF_angel_Front - rodAF_angel_Front;
		xB = -L1 * sin(MOTOR_ANGLE[0][count]);//B点坐标
		yB = -L1 * cos(MOTOR_ANGLE[0][count]);
		xC = (L4 * xB + L2 * xF) / (L2 + L4);//C点坐标
		yC = (L4 * yB + L2 * yF_Front) / (L2 + L4);
		BAC_angel = acos((xB * xC + yB * yC) / sqrt(pow(xB,2) + pow(yB,2)) / sqrt(pow(xC,2) + pow(yC,2)));
		rodAC_angel = atan(- xC / yC); // AC与竖直方向夹角（逆时针为正）
		MOTOR_ANGLE[1][count] = BAC_angel + rodAC_angel;
	
		xF = FOOTPOINT_X[1][count] - S1 / 2;
		yF_Front = FOOTPOINT_Y[1][count] - L_AF_Front;
		virtual_L_Front = pow(pow(xF, 2) + pow(yF_Front, 2), 0.5);
		rodAF_angel_Front = asin(xF / virtual_L_Front); // AF与竖直方向夹角（逆时针为正）
		ABF_angel_Front = acos((pow(virtual_L_Front, 2) + pow(L1, 2) - pow(L2 + L4, 2)) / 2 / L1 / virtual_L_Front); //AF与AB夹角
		MOTOR_ANGLE[2][count] = ABF_angel_Front - rodAF_angel_Front;
		xB = -L1 * sin(MOTOR_ANGLE[2][count]);//B点坐标
		yB = -L1 * cos(MOTOR_ANGLE[2][count]);
		xC = (L4 * xB + L2 * xF) / (L2 + L4);//C点坐标
		yC = (L4 * yB + L2 * yF_Front) / (L2 + L4);
		BAC_angel = acos((xB * xC + yB * yC) / sqrt(pow(xB, 2) + pow(yB, 2)) / sqrt(pow(xC, 2) + pow(yC, 2)));
		rodAC_angel = atan(-xC / yC); // AC与竖直方向夹角（逆时针为正）
		MOTOR_ANGLE[3][count] = BAC_angel + rodAC_angel;

		xF = FOOTPOINT_X[2][count] - S2 / 2;
		yF_Back = FOOTPOINT_Y[2][count] - L_AF_Back;
		virtual_L_Back = pow(pow(xF, 2) + pow(yF_Back, 2), 0.5);
		rodAF_angel_Back = asin(xF / virtual_L_Back); // AF与竖直方向夹角（逆时针为正）
		ABF_angel_Back = acos((pow(virtual_L_Back, 2) + pow(L1, 2) - pow(L2 + L4, 2)) / 2 / L1 / virtual_L_Back); //AF与AB夹角
		MOTOR_ANGLE[4][count] = ABF_angel_Back - rodAF_angel_Back;
		xB = -L1 * sin(MOTOR_ANGLE[4][count]);//B点坐标
		yB = -L1 * cos(MOTOR_ANGLE[4][count]);
		xC = (L4 * xB + L2 * xF) / (L2 + L4);//C点坐标
		yC = (L4 * yB + L2 * yF_Back ) / (L2 + L4);
		BAC_angel = acos((xB * xC + yB * yC) / sqrt(pow(xB, 2) + pow(yB, 2)) / sqrt(pow(xC, 2) + pow(yC, 2)));
		rodAC_angel = atan(-xC / yC); // AC与竖直方向夹角（逆时针为正）
		MOTOR_ANGLE[5][count] = BAC_angel + rodAC_angel;

		xF = FOOTPOINT_X[3][count] - S3 / 2;
		yF_Back  = FOOTPOINT_Y[3][count] - L_AF_Back;
		virtual_L_Back = pow(pow(xF, 2) + pow(yF_Back, 2), 0.5);
		rodAF_angel_Back = asin(xF / virtual_L_Back); // AF与竖直方向夹角（逆时针为正）
		ABF_angel_Back = acos((pow(virtual_L_Back, 2) + pow(L1, 2) - pow(L2 + L4, 2)) / 2 / L1 / virtual_L_Back); //AF与AB夹角
		MOTOR_ANGLE[6][count] = ABF_angel_Back - rodAF_angel_Back;
		xB = -L1 * sin(MOTOR_ANGLE[6][count]);//B点坐标
		yB = -L1 * cos(MOTOR_ANGLE[6][count]);
		xC = (L4 * xB + L2 * xF) / (L2 + L4);//C点坐标
		yC = (L4 * yB + L2 * yF_Back ) / (L2 + L4);
		BAC_angel = acos((xB * xC + yB * yC) / sqrt(pow(xB, 2) + pow(yB, 2)) / sqrt(pow(xC, 2) + pow(yC, 2)));
		rodAC_angel = atan(-xC / yC); // AC与竖直方向夹角（逆时针为正）
		MOTOR_ANGLE[7][count] = BAC_angel + rodAC_angel;
		
		//弧度制 -> 角度制
		for(int i = 0; i < 8; i++){
			MOTOR_ANGLE[i][count] = MOTOR_ANGLE[i][count] * 180 / PI;
		}
	}
	//传递电机角
	for(int i = 0; i < 8; i++){
		for(int j = 0; j < Trot_LENGTH; j++) {
			MotorSlope[i][j] = MOTOR_ANGLE[i][j];
		}
	}
}


void Up_Slope_Regulate(void)
{
//  Slope_Regulate(180,280,75,75);
//	for(int i = 0; i < 8; i++){
//		for(int j = 0; j < Trot_LENGTH; j++) {
//			MotorUpSlope[i][j] = MotorSlope[i][j];
//		}
//	}
	float s = 150;
	float h = 10;
	float L33 = 120;
	for (int count = 0; count < Trot_LENGTH / 2; count++) {
		float t = count * walk_delay;
		FOOTPOINT_X[0][count] = S0 * (t / Tm - 0.5 / PI * sin(2 * PI * t / Tm));
		FOOTPOINT_Y[0][count] = h * (0.5 - 0.5 * cos(2 * PI * t / Tm));
		FOOTPOINT_X[1][count] = S1 * (1 - t / Tm + 0.5 / PI * sin(2 * PI * t / Tm));
		FOOTPOINT_Y[1][count] = -HB * (0.5 - 0.5 * cos(2 * PI * t / Tm));
		FOOTPOINT_X[2][count] = S2 * (t / Tm - 0.5 / PI * sin(2 * PI * t / Tm));
		FOOTPOINT_Y[2][count] = h * (0.5 - 0.5 * cos(2 * PI * t / Tm));
		FOOTPOINT_X[3][count] = S3 * (1 - t / Tm + 0.5 / PI * sin(2 * PI * t / Tm));
		FOOTPOINT_Y[3][count] = -HB * (0.5 - 0.5 * cos(2 * PI * t / Tm));
	}
	for (int count = 0; count < Trot_LENGTH / 2; count++) {
		float t = count * walk_delay;
		FOOTPOINT_X[0][count + Trot_LENGTH / 2] = S0 * (1 - t / Tm + 0.5 / PI * sin(2 * PI * t / Tm));
		FOOTPOINT_Y[0][count + Trot_LENGTH / 2] = -HB * (0.5 - 0.5 * cos(2 * PI * t / Tm));
		FOOTPOINT_X[1][count + Trot_LENGTH / 2] = S1 * (t / Tm - 0.5 / PI * sin(2 * PI * t / Tm));
		FOOTPOINT_Y[1][count + Trot_LENGTH / 2] = h * (0.5 - 0.5 * cos(2 * PI * t / Tm));
		FOOTPOINT_X[2][count + Trot_LENGTH / 2] = S2 * (1 - t / Tm + 0.5 / PI * sin(2 * PI * t / Tm));
		FOOTPOINT_Y[2][count + Trot_LENGTH / 2] = -HB * (0.5 - 0.5 * cos(2 * PI * t / Tm));
		FOOTPOINT_X[3][count + Trot_LENGTH / 2] = S3 * (t / Tm - 0.5 / PI * sin(2 * PI * t / Tm));
		FOOTPOINT_Y[3][count + Trot_LENGTH / 2] = h * (0.5 - 0.5 * cos(2 * PI * t / Tm));
	}
	float xF, yF, virtual_L, rodAF_angel, ABF_angel;
	float xB, yB, xC, yC, BAC_angel, rodAC_angel, L_AF;
	L_AF = sqrt(pow(L1, 2) + pow((L2 + L4), 2) - (pow(L1, 2) + pow(L2, 2) - pow(L33, 2)) * (L2 + L4) / L2);
	for (int count = 0; count < Trot_LENGTH; count++) {
		xF = FOOTPOINT_X[0][count] - S0 / 2 - 80;//F点坐标
		yF = FOOTPOINT_Y[0][count] - L_AF;
		virtual_L = pow(pow(xF,2) + pow(yF,2),0.5);
		rodAF_angel = asin(xF / virtual_L); // AF与竖直方向夹角（逆时针为正）
		ABF_angel = acos((pow(virtual_L,2) + pow(L1,2) - pow(L2 + L4,2)) / 2 / L1 / virtual_L); //AF与AB夹角
		MOTOR_ANGLE[0][count] = ABF_angel - rodAF_angel;
		xB = -L1 * sin(MOTOR_ANGLE[0][count]);//B点坐标
		yB = -L1 * cos(MOTOR_ANGLE[0][count]);
		xC = (L4 * xB + L2 * xF) / (L2 + L4);//C点坐标
		yC = (L4 * yB + L2 * yF) / (L2 + L4);
		BAC_angel = acos((xB * xC + yB * yC) / sqrt(pow(xB,2) + pow(yB,2)) / sqrt(pow(xC,2) + pow(yC,2)));
		rodAC_angel = atan(- xC / yC); // AC与竖直方向夹角（逆时针为正）
		MOTOR_ANGLE[1][count] = BAC_angel + rodAC_angel;
	
		xF = FOOTPOINT_X[1][count] - S1/ 2 - 80;
		yF = FOOTPOINT_Y[1][count] - L_AF;
		virtual_L = pow(pow(xF, 2) + pow(yF, 2), 0.5);
		rodAF_angel = asin(xF / virtual_L); // AF与竖直方向夹角（逆时针为正）
		ABF_angel = acos((pow(virtual_L, 2) + pow(L1, 2) - pow(L2 + L4, 2)) / 2 / L1 / virtual_L); //AF与AB夹角
		MOTOR_ANGLE[2][count] = ABF_angel - rodAF_angel;
		xB = -L1 * sin(MOTOR_ANGLE[2][count]);//B点坐标
		yB = -L1 * cos(MOTOR_ANGLE[2][count]);
		xC = (L4 * xB + L2 * xF) / (L2 + L4);//C点坐标
		yC = (L4 * yB + L2 * yF) / (L2 + L4);
		BAC_angel = acos((xB * xC + yB * yC) / sqrt(pow(xB, 2) + pow(yB, 2)) / sqrt(pow(xC, 2) + pow(yC, 2)));
		rodAC_angel = atan(-xC / yC); // AC与竖直方向夹角（逆时针为正）
		MOTOR_ANGLE[3][count] = BAC_angel + rodAC_angel;

		xF = FOOTPOINT_X[2][count] - S2 / 2 - 80;
		yF = FOOTPOINT_Y[2][count] - L_AF;
		virtual_L = pow(pow(xF, 2) + pow(yF, 2), 0.5);
		rodAF_angel = asin(xF / virtual_L); // AF与竖直方向夹角（逆时针为正）
		ABF_angel = acos((pow(virtual_L, 2) + pow(L1, 2) - pow(L2 + L4, 2)) / 2 / L1 / virtual_L); //AF与AB夹角
		MOTOR_ANGLE[4][count] = ABF_angel - rodAF_angel;
		xB = -L1 * sin(MOTOR_ANGLE[4][count]);//B点坐标
		yB = -L1 * cos(MOTOR_ANGLE[4][count]);
		xC = (L4 * xB + L2 * xF) / (L2 + L4);//C点坐标
		yC = (L4 * yB + L2 * yF) / (L2 + L4);
		BAC_angel = acos((xB * xC + yB * yC) / sqrt(pow(xB, 2) + pow(yB, 2)) / sqrt(pow(xC, 2) + pow(yC, 2)));
		rodAC_angel = atan(-xC / yC); // AC与竖直方向夹角（逆时针为正）
		MOTOR_ANGLE[5][count] = BAC_angel + rodAC_angel;

		xF = FOOTPOINT_X[3][count] - S3 / 2 - 80;
		yF = FOOTPOINT_Y[3][count] - L_AF;
		virtual_L = pow(pow(xF, 2) + pow(yF, 2), 0.5);
		rodAF_angel = asin(xF / virtual_L); // AF与竖直方向夹角（逆时针为正）
		ABF_angel = acos((pow(virtual_L, 2) + pow(L1, 2) - pow(L2 + L4, 2)) / 2 / L1 / virtual_L); //AF与AB夹角
		MOTOR_ANGLE[6][count] = ABF_angel - rodAF_angel;
		xB = -L1 * sin(MOTOR_ANGLE[6][count]);//B点坐标
		yB = -L1 * cos(MOTOR_ANGLE[6][count]);
		xC = (L4 * xB + L2 * xF) / (L2 + L4);//C点坐标
		yC = (L4 * yB + L2 * yF) / (L2 + L4);
		BAC_angel = acos((xB * xC + yB * yC) / sqrt(pow(xB, 2) + pow(yB, 2)) / sqrt(pow(xC, 2) + pow(yC, 2)));
		rodAC_angel = atan(-xC / yC); // AC与竖直方向夹角（逆时针为正）
		MOTOR_ANGLE[7][count] = BAC_angel + rodAC_angel;
		
		//弧度制 -> 角度制
		for(int i = 0; i < Trot_LENGTH; i++){
			MOTOR_ANGLE[i][count] = MOTOR_ANGLE[i][count] * 180 / PI;
		}
	}
	//传递电机角
	for(int i = 0; i < 8; i++){
		for(int j = 0; j < Trot_LENGTH; j++) {
			MotorUpSlope[i][j] = MOTOR_ANGLE[i][j];
		}
	}
}
void Down_Slope_Regulate(void)
{
  Slope_Regulate(270,180,75,75);
	for(int i = 0; i < 8; i++){
		for(int j = 0; j < Trot_LENGTH; j++) {
			MotorDownSlope[i][j] = MotorSlope[i][j];
		}
	}
}

void Upstair_Regulate(float L3_Front,float L3_Back,float H_Front,float H_Back,float S_Stair){
	for (int count = 0; count < Trot_LENGTH / 2; count++) {
		float t = count * walk_delay;
		FOOTPOINT_X[0][count] = S_Stair * (t / Tm - 0.5 / PI * sin(2 * PI * t / Tm));
		FOOTPOINT_Y[0][count] = H_Front * (0.5 - 0.5 * cos(2 * PI * t / Tm));
		FOOTPOINT_X[1][count] = S_Stair * (1 - t / Tm + 0.5 / PI * sin(2 * PI * t / Tm));
		FOOTPOINT_Y[1][count] = -HB * (0.5 - 0.5 * cos(2 * PI * t / Tm));
		FOOTPOINT_X[2][count] = S_Stair * (t / Tm - 0.5 / PI * sin(2 * PI * t / Tm));
		FOOTPOINT_Y[2][count] = H_Back * (0.5 - 0.5 * cos(2 * PI * t / Tm));
		FOOTPOINT_X[3][count] = S_Stair * (1 - t / Tm + 0.5 / PI * sin(2 * PI * t / Tm));
		FOOTPOINT_Y[3][count] = -HB * (0.5 - 0.5 * cos(2 * PI * t / Tm));
	}
	for (int count = 0; count < Trot_LENGTH / 2; count++) {
		float t = count * walk_delay;
		FOOTPOINT_X[0][count + Trot_LENGTH / 2] = S_Stair * (1 - t / Tm + 0.5 / PI * sin(2 * PI * t / Tm));
		FOOTPOINT_Y[0][count + Trot_LENGTH / 2] = -HB * (0.5 - 0.5 * cos(2 * PI * t / Tm));
		FOOTPOINT_X[1][count + Trot_LENGTH / 2] = S_Stair * (t / Tm - 0.5 / PI * sin(2 * PI * t / Tm));
		FOOTPOINT_Y[1][count + Trot_LENGTH / 2] = H_Front * (0.5 - 0.5 * cos(2 * PI * t / Tm));
		FOOTPOINT_X[2][count + Trot_LENGTH / 2] = S_Stair * (1 - t / Tm + 0.5 / PI * sin(2 * PI * t / Tm));
		FOOTPOINT_Y[2][count + Trot_LENGTH / 2] = -HB * (0.5 - 0.5 * cos(2 * PI * t / Tm));
		FOOTPOINT_X[3][count + Trot_LENGTH / 2] = S_Stair * (t / Tm - 0.5 / PI * sin(2 * PI * t / Tm));
		FOOTPOINT_Y[3][count + Trot_LENGTH / 2] = H_Back * (0.5 - 0.5 * cos(2 * PI * t / Tm));
	}
	float xF, yF_Front, yF_Back,virtual_L_Front, virtual_L_Back,rodAF_angel_Front,rodAF_angel_Back, ABF_angel_Front,ABF_angel_Back;
	float xB, yB, xC, yC, BAC_angel, rodAC_angel, L_AF_Front,L_AF_Back;
	L_AF_Front = sqrt(pow(L1, 2) + pow((L2 + L4), 2) - (pow(L1, 2) + pow(L2, 2) - pow(L3_Front, 2)) * (L2 + L4) / L2);
  L_AF_Back   =  sqrt(pow(L1, 2) + pow((L2 + L4), 2) - (pow(L1, 2) + pow(L2, 2) - pow(L3_Back, 2)) * (L2 + L4) / L2);
	for (int count = 0; count < Trot_LENGTH; count++) {
		xF = FOOTPOINT_X[0][count] - S_Stair / 2;//F点坐标
		yF_Front = FOOTPOINT_Y[0][count] - L_AF_Front;
		virtual_L_Front = pow(pow(xF,2) + pow(yF_Front,2),0.5);	
		rodAF_angel_Front = asin(xF / virtual_L_Front); // AF与竖直方向夹角（逆时针为正）		
		ABF_angel_Front = acos((pow(virtual_L_Front,2) + pow(L1,2) - pow(L2 + L4,2)) / 2 / L1 / virtual_L_Front); //AF与AB夹角
		MOTOR_ANGLE[0][count] = ABF_angel_Front - rodAF_angel_Front;
		xB = -L1 * sin(MOTOR_ANGLE[0][count]);//B点坐标
		yB = -L1 * cos(MOTOR_ANGLE[0][count]);
		xC = (L4 * xB + L2 * xF) / (L2 + L4);//C点坐标
		yC = (L4 * yB + L2 * yF_Front) / (L2 + L4);
		BAC_angel = acos((xB * xC + yB * yC) / sqrt(pow(xB,2) + pow(yB,2)) / sqrt(pow(xC,2) + pow(yC,2)));
		rodAC_angel = atan(- xC / yC); // AC与竖直方向夹角（逆时针为正）
		MOTOR_ANGLE[1][count] = BAC_angel + rodAC_angel;
	
		xF = FOOTPOINT_X[1][count] - S_Stair / 2;
		yF_Front = FOOTPOINT_Y[1][count] - L_AF_Front;
		virtual_L_Front = pow(pow(xF, 2) + pow(yF_Front, 2), 0.5);
		rodAF_angel_Front = asin(xF / virtual_L_Front); // AF与竖直方向夹角（逆时针为正）
		ABF_angel_Front = acos((pow(virtual_L_Front, 2) + pow(L1, 2) - pow(L2 + L4, 2)) / 2 / L1 / virtual_L_Front); //AF与AB夹角
		MOTOR_ANGLE[2][count] = ABF_angel_Front - rodAF_angel_Front;
		xB = -L1 * sin(MOTOR_ANGLE[2][count]);//B点坐标
		yB = -L1 * cos(MOTOR_ANGLE[2][count]);
		xC = (L4 * xB + L2 * xF) / (L2 + L4);//C点坐标
		yC = (L4 * yB + L2 * yF_Front) / (L2 + L4);
		BAC_angel = acos((xB * xC + yB * yC) / sqrt(pow(xB, 2) + pow(yB, 2)) / sqrt(pow(xC, 2) + pow(yC, 2)));
		rodAC_angel = atan(-xC / yC); // AC与竖直方向夹角（逆时针为正）
		MOTOR_ANGLE[3][count] = BAC_angel + rodAC_angel;

		xF = FOOTPOINT_X[2][count] - S_Stair / 2;
		yF_Back = FOOTPOINT_Y[2][count] - L_AF_Back;
		virtual_L_Back = pow(pow(xF, 2) + pow(yF_Back, 2), 0.5);
		rodAF_angel_Back = asin(xF / virtual_L_Back); // AF与竖直方向夹角（逆时针为正）
		ABF_angel_Back = acos((pow(virtual_L_Back, 2) + pow(L1, 2) - pow(L2 + L4, 2)) / 2 / L1 / virtual_L_Back); //AF与AB夹角
		MOTOR_ANGLE[4][count] = ABF_angel_Back - rodAF_angel_Back;
		xB = -L1 * sin(MOTOR_ANGLE[4][count]);//B点坐标
		yB = -L1 * cos(MOTOR_ANGLE[4][count]);
		xC = (L4 * xB + L2 * xF) / (L2 + L4);//C点坐标
		yC = (L4 * yB + L2 * yF_Back ) / (L2 + L4);
		BAC_angel = acos((xB * xC + yB * yC) / sqrt(pow(xB, 2) + pow(yB, 2)) / sqrt(pow(xC, 2) + pow(yC, 2)));
		rodAC_angel = atan(-xC / yC); // AC与竖直方向夹角（逆时针为正）
		MOTOR_ANGLE[5][count] = BAC_angel + rodAC_angel;

		xF = FOOTPOINT_X[3][count] - S_Stair / 2;
		yF_Back  = FOOTPOINT_Y[3][count] - L_AF_Back;
		virtual_L_Back = pow(pow(xF, 2) + pow(yF_Back, 2), 0.5);
		rodAF_angel_Back = asin(xF / virtual_L_Back); // AF与竖直方向夹角（逆时针为正）
		ABF_angel_Back = acos((pow(virtual_L_Back, 2) + pow(L1, 2) - pow(L2 + L4, 2)) / 2 / L1 / virtual_L_Back); //AF与AB夹角
		MOTOR_ANGLE[6][count] = ABF_angel_Back - rodAF_angel_Back;
		xB = -L1 * sin(MOTOR_ANGLE[6][count]);//B点坐标
		yB = -L1 * cos(MOTOR_ANGLE[6][count]);
		xC = (L4 * xB + L2 * xF) / (L2 + L4);//C点坐标
		yC = (L4 * yB + L2 * yF_Back ) / (L2 + L4);
		BAC_angel = acos((xB * xC + yB * yC) / sqrt(pow(xB, 2) + pow(yB, 2)) / sqrt(pow(xC, 2) + pow(yC, 2)));
		rodAC_angel = atan(-xC / yC); // AC与竖直方向夹角（逆时针为正）
		MOTOR_ANGLE[7][count] = BAC_angel + rodAC_angel;
		
		//弧度制 -> 角度制
		for(int i = 0; i < 8; i++){
			MOTOR_ANGLE[i][count] = MOTOR_ANGLE[i][count] * 180 / PI;
		}
	}
	//传递电机角
	for(int i = 0; i < 8; i++){
		for(int j = 0; j < Trot_LENGTH; j++) {
			MotorUpstair_Fix[i][j] = MOTOR_ANGLE[i][j];
		}
	}
}

