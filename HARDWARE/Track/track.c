
#include "track.h"

static u8 currentRobotState;
static u8 lastRobotState;

/*	flag = 0, Move Left 
		flag = 1, Rotate
		flag = 2, Track Mode
*/
static u8 flag = 0;
static u8 posFlag = 0;
static u8 rotateFlag = 0;
static u8 stopRightFlag = 0;
static u8 falseCnt = 0;
static u16 rotateCnt = 0;


void GPIOCLKInit(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA , ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB , ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC , ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD , ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE , ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF , ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG , ENABLE);
}

void RedRayInit(void){
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	GPIO_InitStructure.GPIO_Pin = SEARCH_OUT_PIN_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(SEARCH_OUT_GPIO_1, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = SEARCH_OUT_PIN_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(SEARCH_OUT_GPIO_2, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = SEARCH_OUT_PIN_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(SEARCH_OUT_GPIO_3, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = SEARCH_OUT_PIN_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(SEARCH_OUT_GPIO_4, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = SEARCH_OUT_PIN_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(SEARCH_OUT_GPIO_5, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = SEARCH_OUT_PIN_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(SEARCH_OUT_GPIO_6, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = SEARCH_OUT_PIN_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(SEARCH_OUT_GPIO_7, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = SEARCH_OUT_PIN_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(SEARCH_OUT_GPIO_11, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = SEARCH_OUT_PIN_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(SEARCH_OUT_GPIO_12, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = SEARCH_OUT_PIN_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(SEARCH_OUT_GPIO_13, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = SEARCH_OUT_PIN_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(SEARCH_OUT_GPIO_14, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = SEARCH_OUT_PIN_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(SEARCH_OUT_GPIO_15, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = SEARCH_OUT_PIN_16;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(SEARCH_OUT_GPIO_16, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = SEARCH_OUT_PIN_17;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(SEARCH_OUT_GPIO_17, &GPIO_InitStructure);
}

//??????
u8 wayAllBlack(void){
	if(SEARCH_OUT_IO_1 == BLACK_AREA &&
	SEARCH_OUT_IO_2 == BLACK_AREA &&
	SEARCH_OUT_IO_3 == BLACK_AREA &&
	SEARCH_OUT_IO_4 == BLACK_AREA &&
	SEARCH_OUT_IO_5 == BLACK_AREA &&
	SEARCH_OUT_IO_6 == BLACK_AREA &&
	SEARCH_OUT_IO_7 == BLACK_AREA)
	{
		return 1;
	}
	else{
		return 0;
	}
}

//
u8 canRotate(void){
	if(SEARCH_OUT_IO_1 == WHITE_AREA &&
	SEARCH_OUT_IO_2 == WHITE_AREA &&
	SEARCH_OUT_IO_6 == WHITE_AREA &&
	SEARCH_OUT_IO_7 == WHITE_AREA &&
	SEARCH_OUT_IO_4 == BLACK_AREA){
		return 1;
	}
	else{
		return 0;
	}
}

u8 needRotateClockwise(void){
	if(canRotate() && SEARCH_OUT_IO_17 == WHITE_AREA
		&& SEARCH_OUT_IO_16 == WHITE_AREA
		&& SEARCH_OUT_IO_15 == WHITE_AREA
		&& SEARCH_OUT_IO_14 == WHITE_AREA
		&& (SEARCH_OUT_IO_13 == WHITE_AREA
		|| SEARCH_OUT_IO_12 == BLACK_AREA
		|| SEARCH_OUT_IO_11 == BLACK_AREA)){
			return 1;
		}
		else{
			return 0;
		}
}

u8 needRotateCounterClockwise(void){
	if(canRotate() && SEARCH_OUT_IO_11 == WHITE_AREA
		&& SEARCH_OUT_IO_12 == WHITE_AREA
		&& SEARCH_OUT_IO_13 == WHITE_AREA
		&& SEARCH_OUT_IO_14 == WHITE_AREA
		&& (SEARCH_OUT_IO_15 == WHITE_AREA
		|| SEARCH_OUT_IO_16 == BLACK_AREA
		|| SEARCH_OUT_IO_17 == BLACK_AREA)){
			return 1;
		}
		else{
			return 0;
		}
}

u8 lightBlack(u8 type, u8 pos1, u8 pos2){
	u8 cnt = 0;
	switch(pos1){
		case 1:{
			if(SEARCH_OUT_IO_7 == BLACK_AREA){
				++cnt;
			}
		}
		case 2:{
			if(SEARCH_OUT_IO_6 == BLACK_AREA){
				++cnt;
			}
		}
		case 3:{
			if(SEARCH_OUT_IO_5 == BLACK_AREA){
				++cnt;
			}
		}
		case 4:{
			if(SEARCH_OUT_IO_4 == BLACK_AREA){
				++cnt;
			}
		}
		case 5:{
			if(SEARCH_OUT_IO_3 == BLACK_AREA){
				++cnt;
			}
			break;
		}
		case 6:{
			if(SEARCH_OUT_IO_2 == BLACK_AREA){
				++cnt;
			}
			break;
		}
		case 7:{
			if(SEARCH_OUT_IO_1 == BLACK_AREA){
				++cnt;
			}
			break;
		}
		default:{
			break;
		}
	}
	switch(pos2){
		case 1:{
			if(SEARCH_OUT_IO_11 == BLACK_AREA){
				++cnt;
			}
		}
		case 2:{
			if(SEARCH_OUT_IO_12 == BLACK_AREA){
				++cnt;
			}
		}
		case 3:{
			if(SEARCH_OUT_IO_13 == BLACK_AREA){
				++cnt;
			}
		}
		case 4:{
			if(SEARCH_OUT_IO_14 == BLACK_AREA){
				++cnt;
			}
		}
		case 5:{
			if(SEARCH_OUT_IO_15 == BLACK_AREA){
				++cnt;
			}
			break;
		}
		case 6:{
			if(SEARCH_OUT_IO_16 == BLACK_AREA){
				++cnt;
			}
			break;
		}
		case 7:{
			if(SEARCH_OUT_IO_17 == BLACK_AREA){
				++cnt;
			}
			break;
		}
		default:{
			break;
		}
	}
	if(type == 1 || type == 2){
		if(cnt == 1){
			return 1;
		}
		else {
			return 0;
		}
	}
	if(type == 3){
		if(cnt == 2){
			return 1;
		}
		else{
			return 0;
		}
	}
}

u8 rotateToTrack(void){
	if(
//		(SEARCH_OUT_IO_1 == BLACK_AREA && SEARCH_OUT_IO_17 == BLACK_AREA
//		&& SEARCH_OUT_IO_2 == WHITE_AREA && SEARCH_OUT_IO_16 == WHITE_AREA)
//	
//		|| (SEARCH_OUT_IO_1 == BLACK_AREA && SEARCH_OUT_IO_17 == BLACK_AREA
//		&& SEARCH_OUT_IO_2 == BLACK_AREA && SEARCH_OUT_IO_16 == BLACK_AREA
//		&& SEARCH_OUT_IO_3 == WHITE_AREA && SEARCH_OUT_IO_15 == WHITE_AREA)
	
		(SEARCH_OUT_IO_1 == WHITE_AREA && SEARCH_OUT_IO_17 == WHITE_AREA
		&& SEARCH_OUT_IO_2 == BLACK_AREA && SEARCH_OUT_IO_16 == BLACK_AREA
		&& SEARCH_OUT_IO_3 == BLACK_AREA && SEARCH_OUT_IO_15 == BLACK_AREA
		&& SEARCH_OUT_IO_4 == WHITE_AREA && SEARCH_OUT_IO_14 == WHITE_AREA)
	
		|| (SEARCH_OUT_IO_2 == WHITE_AREA && SEARCH_OUT_IO_16 == WHITE_AREA
		&& SEARCH_OUT_IO_3 == BLACK_AREA && SEARCH_OUT_IO_15 == BLACK_AREA
		&& SEARCH_OUT_IO_4 == BLACK_AREA && SEARCH_OUT_IO_14 == BLACK_AREA
		&& SEARCH_OUT_IO_5 == WHITE_AREA && SEARCH_OUT_IO_13 == WHITE_AREA)
	
		|| (SEARCH_OUT_IO_3 == WHITE_AREA && SEARCH_OUT_IO_15 == WHITE_AREA
		&& SEARCH_OUT_IO_4 == BLACK_AREA && SEARCH_OUT_IO_14 == BLACK_AREA
		&& SEARCH_OUT_IO_5 == BLACK_AREA && SEARCH_OUT_IO_13 == BLACK_AREA
		&& SEARCH_OUT_IO_6 == WHITE_AREA && SEARCH_OUT_IO_12 == WHITE_AREA)
	
		|| (SEARCH_OUT_IO_4 == WHITE_AREA && SEARCH_OUT_IO_14 == WHITE_AREA
		&& SEARCH_OUT_IO_5 == BLACK_AREA && SEARCH_OUT_IO_13 == BLACK_AREA
		&& SEARCH_OUT_IO_6 == BLACK_AREA && SEARCH_OUT_IO_12 == BLACK_AREA
		&& SEARCH_OUT_IO_7 == WHITE_AREA && SEARCH_OUT_IO_11 == WHITE_AREA)
		
//		|| (SEARCH_OUT_IO_5 == WHITE_AREA && SEARCH_OUT_IO_13 == WHITE_AREA
//		&& SEARCH_OUT_IO_6 == BLACK_AREA && SEARCH_OUT_IO_12 == BLACK_AREA
//		&& SEARCH_OUT_IO_7 == BLACK_AREA && SEARCH_OUT_IO_11 == BLACK_AREA)
		
//		|| (SEARCH_OUT_IO_6 == WHITE_AREA && SEARCH_OUT_IO_12 == WHITE_AREA
//		&& SEARCH_OUT_IO_7 == BLACK_AREA && SEARCH_OUT_IO_11 == BLACK_AREA)
		){
			return 1;
		}
		
		else if(
//		(SEARCH_OUT_IO_1 == BLACK_AREA && SEARCH_OUT_IO_17 == BLACK_AREA
//		&& SEARCH_OUT_IO_2 == WHITE_AREA && SEARCH_OUT_IO_16 == WHITE_AREA)
	
		(SEARCH_OUT_IO_1 == WHITE_AREA && SEARCH_OUT_IO_17 == WHITE_AREA
		&& SEARCH_OUT_IO_2 == BLACK_AREA && SEARCH_OUT_IO_16 == BLACK_AREA
		&& SEARCH_OUT_IO_3 == WHITE_AREA && SEARCH_OUT_IO_15 == WHITE_AREA)
	
		|| (SEARCH_OUT_IO_2 == WHITE_AREA && SEARCH_OUT_IO_16 == WHITE_AREA
		&& SEARCH_OUT_IO_3 == BLACK_AREA && SEARCH_OUT_IO_15 == BLACK_AREA
		&& SEARCH_OUT_IO_4 == WHITE_AREA && SEARCH_OUT_IO_14 == WHITE_AREA)
	
		|| (SEARCH_OUT_IO_3 == WHITE_AREA && SEARCH_OUT_IO_15 == WHITE_AREA
		&& SEARCH_OUT_IO_4 == BLACK_AREA && SEARCH_OUT_IO_14 == BLACK_AREA
		&& SEARCH_OUT_IO_5 == WHITE_AREA && SEARCH_OUT_IO_13 == WHITE_AREA)
	
		|| (SEARCH_OUT_IO_4 == WHITE_AREA && SEARCH_OUT_IO_14 == WHITE_AREA
		&& SEARCH_OUT_IO_5 == BLACK_AREA && SEARCH_OUT_IO_13 == BLACK_AREA
		&& SEARCH_OUT_IO_6 == WHITE_AREA && SEARCH_OUT_IO_12 == WHITE_AREA)
	
		|| (SEARCH_OUT_IO_5 == WHITE_AREA && SEARCH_OUT_IO_13 == WHITE_AREA
		&& SEARCH_OUT_IO_6 == BLACK_AREA && SEARCH_OUT_IO_12 == BLACK_AREA
		&& SEARCH_OUT_IO_7 == WHITE_AREA && SEARCH_OUT_IO_11 == WHITE_AREA)
		
//		|| (SEARCH_OUT_IO_6 == WHITE_AREA && SEARCH_OUT_IO_12 == WHITE_AREA
//		&& SEARCH_OUT_IO_7 == BLACK_AREA && SEARCH_OUT_IO_11 == BLACK_AREA)
		){
			return 2;
		}
		
		else if(
		(SEARCH_OUT_IO_1 == BLACK_AREA && SEARCH_OUT_IO_17 == BLACK_AREA
		&& SEARCH_OUT_IO_2 == BLACK_AREA && SEARCH_OUT_IO_16 == BLACK_AREA
		&& SEARCH_OUT_IO_3 == BLACK_AREA && SEARCH_OUT_IO_15 == BLACK_AREA)
	
		|| (SEARCH_OUT_IO_2 == BLACK_AREA && SEARCH_OUT_IO_16 == BLACK_AREA
		&& SEARCH_OUT_IO_3 == BLACK_AREA && SEARCH_OUT_IO_15 == BLACK_AREA
		&& SEARCH_OUT_IO_4 == BLACK_AREA && SEARCH_OUT_IO_14 == BLACK_AREA)
	
		|| (SEARCH_OUT_IO_3 == BLACK_AREA && SEARCH_OUT_IO_15 == BLACK_AREA
		&& SEARCH_OUT_IO_4 == BLACK_AREA && SEARCH_OUT_IO_14 == BLACK_AREA
		&& SEARCH_OUT_IO_5 == BLACK_AREA && SEARCH_OUT_IO_13 == BLACK_AREA)
	
		|| (SEARCH_OUT_IO_4 == BLACK_AREA && SEARCH_OUT_IO_14 == BLACK_AREA
		&& SEARCH_OUT_IO_5 == BLACK_AREA && SEARCH_OUT_IO_13 == BLACK_AREA
		&& SEARCH_OUT_IO_6 == BLACK_AREA && SEARCH_OUT_IO_12 == BLACK_AREA)
	
		|| (SEARCH_OUT_IO_5 == BLACK_AREA && SEARCH_OUT_IO_13 == BLACK_AREA
		&& SEARCH_OUT_IO_6 == BLACK_AREA && SEARCH_OUT_IO_12 == BLACK_AREA
		&& SEARCH_OUT_IO_7 == BLACK_AREA && SEARCH_OUT_IO_11 == BLACK_AREA)
		){
			return 3;
		}
		else if(
		(SEARCH_OUT_IO_1 == BLACK_AREA && SEARCH_OUT_IO_17 == BLACK_AREA
		&& SEARCH_OUT_IO_2 == WHITE_AREA && SEARCH_OUT_IO_16 == WHITE_AREA)
	
		|| (SEARCH_OUT_IO_1 == BLACK_AREA && SEARCH_OUT_IO_17 == BLACK_AREA
		&& SEARCH_OUT_IO_2 == BLACK_AREA && SEARCH_OUT_IO_16 == BLACK_AREA
		&& SEARCH_OUT_IO_3 == WHITE_AREA && SEARCH_OUT_IO_15 == WHITE_AREA)
		
		|| (SEARCH_OUT_IO_5 == WHITE_AREA && SEARCH_OUT_IO_13 == WHITE_AREA
		&& SEARCH_OUT_IO_6 == BLACK_AREA && SEARCH_OUT_IO_12 == BLACK_AREA
		&& SEARCH_OUT_IO_7 == BLACK_AREA && SEARCH_OUT_IO_11 == BLACK_AREA)
		
		|| (SEARCH_OUT_IO_6 == WHITE_AREA && SEARCH_OUT_IO_12 == WHITE_AREA
		&& SEARCH_OUT_IO_7 == BLACK_AREA && SEARCH_OUT_IO_11 == BLACK_AREA)
		
		){
			return 4;
		}
		else{
			return 0;
		}
}

u8 way1AllWhite(void){
	if(SEARCH_OUT_IO_1 == WHITE_AREA &&
	SEARCH_OUT_IO_2 == WHITE_AREA &&
	SEARCH_OUT_IO_3 == WHITE_AREA &&
	SEARCH_OUT_IO_4 == WHITE_AREA &&
	SEARCH_OUT_IO_5 == WHITE_AREA &&
	SEARCH_OUT_IO_6 == WHITE_AREA &&
	SEARCH_OUT_IO_7 == WHITE_AREA)
	{
		return 1;
	}
	else{
		return 0;
	}
}

u8 way2AllWhite(void){
	if(SEARCH_OUT_IO_11 == WHITE_AREA &&
	SEARCH_OUT_IO_12 == WHITE_AREA &&
	SEARCH_OUT_IO_13 == WHITE_AREA &&
	SEARCH_OUT_IO_14 == WHITE_AREA &&
	SEARCH_OUT_IO_15 == WHITE_AREA &&
	SEARCH_OUT_IO_16 == WHITE_AREA &&
	SEARCH_OUT_IO_17 == WHITE_AREA)
	{
		return 1;
	}
	else{
		return 0;
	}
}

u8 reachTarget(u8 target){
	u8 cnt = 0;
	if(SEARCH_OUT_IO_1 == BLACK_AREA){
		++cnt;
	}
	if(SEARCH_OUT_IO_2 == BLACK_AREA){
		++cnt;
	}
	if(SEARCH_OUT_IO_3 == BLACK_AREA){
		++cnt;
	}
	if(SEARCH_OUT_IO_4 == BLACK_AREA){
		++cnt;
	}
	if(SEARCH_OUT_IO_5 == BLACK_AREA){
		++cnt;
	}
	if(SEARCH_OUT_IO_6 == BLACK_AREA){
		++cnt;
	}
	if(SEARCH_OUT_IO_7 == BLACK_AREA){
		++cnt;
	}
	if(cnt >= target){
		return 1;
	}
	else{
		return 0;
	}
}

u8 needRight(void){
	if(SEARCH_OUT_IO_7 == WHITE_AREA &&
		SEARCH_OUT_IO_6 == WHITE_AREA &&
		SEARCH_OUT_IO_5 == WHITE_AREA &&
		SEARCH_OUT_IO_4 == WHITE_AREA &&
		(SEARCH_OUT_IO_1 == BLACK_AREA ||
		SEARCH_OUT_IO_2 == BLACK_AREA ||
		SEARCH_OUT_IO_3 == BLACK_AREA)){
			return 1;
		}
		else{
			return 0;
		}
}

u8 needLeft(void){
	if(SEARCH_OUT_IO_1 == WHITE_AREA &&
		SEARCH_OUT_IO_2 == WHITE_AREA &&
		SEARCH_OUT_IO_3 == WHITE_AREA &&
		SEARCH_OUT_IO_4 == WHITE_AREA &&
		(SEARCH_OUT_IO_7 == BLACK_AREA ||
		SEARCH_OUT_IO_6 == BLACK_AREA ||
		SEARCH_OUT_IO_5 == BLACK_AREA)){
			return 1;
		}
		else{
			return 0;
		}
}

u8 needForward(void){
	if(SEARCH_OUT_IO_7 == WHITE_AREA &&
		SEARCH_OUT_IO_6 == WHITE_AREA &&
		SEARCH_OUT_IO_1 == WHITE_AREA &&
		SEARCH_OUT_IO_2 == WHITE_AREA &&
		SEARCH_OUT_IO_4 == BLACK_AREA 
//		|| SEARCH_OUT_IO_3 == BLACK_AREA 
//		|| SEARCH_OUT_IO_5 == BLACK_AREA
	){
			return 1;
		}
		else{
			return 0;
		}
}


void runFront(float speed){
	MotorSpeedExpected(speed, -speed, -speed, speed, 0);
	delay_ms(1);
}

void runBack(float speed){
	MotorSpeedExpected(-speed, speed, speed, -speed, 0);
	delay_ms(1);
}

void runLeft(float speed){
	MotorSpeedExpected(-speed, -speed, speed, speed, 0);
	delay_ms(1);
}

void runRight(float speed){
	MotorSpeedExpected(speed, speed, -speed, -speed, 0);
	delay_ms(1);
}

void runStop(void){
	MotorSpeedExpected(0, 0, 0, 0, 0);
	delay_ms(1);
}

void clockwiseRotate(void){
	MotorSpeedExpected(30, 30, 30, 30, 0);
	delay_ms(1);
}

void clockwiseCounterRotate(void){
	MotorSpeedExpected(-30, -30, -30, -30, 0);
	delay_ms(1);
}

//void redRayDebug(void){
//	if(SEARCH_OUT_IO_1 == WHITE_AREA){
//		printf("1: W\n");
//	}
//	else{
//		printf("1: B\n");
//	}
//	if(SEARCH_OUT_IO_2 == WHITE_AREA){
//		printf("2: W\n");
//	}
//	else{
//		printf("2: B\n");
//	}
//	if(SEARCH_OUT_IO_3 == WHITE_AREA){
//		printf("3: W\n");
//	}
//	else{
//		printf("3: B\n");
//	}
//	if(SEARCH_OUT_IO_4 == WHITE_AREA){
//		printf("4: W\n");
//	}
//	else{
//		printf("4: B\n");
//	}
//	if(SEARCH_OUT_IO_5 == WHITE_AREA){
//		printf("5: W\n");
//	}
//	else{
//		printf("5: B\n");
//	}
//	if(SEARCH_OUT_IO_6 == WHITE_AREA){
//		printf("6: W\n");
//	}
//	else{
//		printf("6: B\n");
//	}
//	if(SEARCH_OUT_IO_7 == WHITE_AREA){
//		printf("7: W\n");
//	}
//	else{
//		printf("7: B\n");
//	}
//}

void stateInit(void){
	lastRobotState = COMM_STOP;
	currentRobotState = COMM_STOP;
	flag = 0;
	rotateFlag = 0;
	posFlag = 0;
	stopRightFlag = 0;
	falseCnt = 0;
	rotateCnt = 0;
}



u8 startTrack(float speed){
	/* Detected where the robot is */
	if(posFlag == 0){
		if(way1AllWhite() && way2AllWhite()){
			posFlag = 1;
		}
		else{
			posFlag = 4;
		}
	}
	
	/* Have not detected the rotate needed situation */
	if(posFlag == 4){
		if(rotateToTrack() == 1){
			rotateFlag = 10;
			flag = 2;
//			for(u8 i = 0; i < 5; ++i){
//				if(currentRobotState == COMM_CLOCK){
//					clockwiseRotate(speed);
//				}
//				else if(currentRobotState == COMM_CTCLOCK){
//					clockwiseCounterRotate(speed);
//				}
//				else{
//					break;
//				}
//			}
		}
		else if(rotateToTrack() == 2){
			rotateFlag = 10;
			flag = 2;
//			for(u8 i = 0; i < 10; ++i){
//				if(currentRobotState == COMM_CLOCK){
//					clockwiseRotate(speed);
//				}
//				else if(currentRobotState == COMM_CTCLOCK){
//					clockwiseCounterRotate(speed);
//				}
//				else{
//					break;
//				}
//			}
		}
		else if(rotateToTrack() == 3){
			rotateFlag = 10;
			flag = 2;
		}
		
		else if(rotateToTrack() == 4 && flag == 1){
			rotateFlag = 10;
			flag = 2;
		}
		
		
		if(flag == 0){
			lastRobotState = currentRobotState;
			currentRobotState = COMM_LEFT;
			if(rotateFlag == 0){
				if(SEARCH_OUT_IO_1 == BLACK_AREA && SEARCH_OUT_IO_17 == BLACK_AREA){
					rotateFlag = 1;
					flag = 1;
				}
				else if(way1AllWhite() && SEARCH_OUT_IO_17 == BLACK_AREA){
					rotateFlag = 2; // need clockwise rotate
					flag = 1;
				}
				else if(SEARCH_OUT_IO_1 == BLACK_AREA && way2AllWhite()){
					rotateFlag = 3; // need counterclockwise rotate
					flag = 1;
				}
			}
		}
		if(flag == 1){
			if(rotateFlag == 1){
				if(SEARCH_OUT_IO_4 == WHITE_AREA && stopRightFlag == 0){
					lastRobotState = currentRobotState;
					currentRobotState = COMM_RIGHT;
				}
				else{
					flag = 2;
					lastRobotState = currentRobotState;
					currentRobotState = COMM_FORWARD;
					stopRightFlag = 1;
				}
			}
			else if(rotateFlag == 2){
				if(SEARCH_OUT_IO_4 == WHITE_AREA && stopRightFlag == 0){
					lastRobotState = currentRobotState;
					currentRobotState = COMM_RIGHT;
					
				}
				else{
					lastRobotState = currentRobotState;
					currentRobotState = COMM_CLOCK;
					stopRightFlag = 1;
					++rotateCnt;
	
					if(falseCnt == 0){
						if(rotateCnt > 1500){
							rotateCnt = 0;
							++falseCnt;
							rotateFlag = 3;
						}
					}
//					else if(falseCnt != 3){
//						if(rotateCnt == 2500){
//							rotateCnt = 0;
//							++falseCnt;
//							rotateFlag = 3;
//						}
//					}
//					else{
//						if(rotateCnt ==1500){
//							rotateCnt = 0;
//							falseCnt = 0;
//							flag = 2;
//						}
//					}
					else{
						if(rotateCnt > 2500){
							rotateCnt = 0;
							++falseCnt;
							rotateFlag = 3;
						}
					}
				}
			}
			else if(rotateFlag == 3){
				if(SEARCH_OUT_IO_14 == WHITE_AREA && stopRightFlag == 0){
					lastRobotState = currentRobotState;
					currentRobotState = COMM_RIGHT;
				}
				else{
					lastRobotState = currentRobotState;
					currentRobotState = COMM_CTCLOCK;
					stopRightFlag = 1;
					++rotateCnt;
					
					if(falseCnt == 0){
						if(rotateCnt > 1500){
							rotateCnt = 0;
							++falseCnt;
							rotateFlag = 2;
						}
					}
//					else if(falseCnt != 3){
//						if(rotateCnt == 2500){
//							rotateCnt = 0;
//							++falseCnt;
//							rotateFlag = 2;
//						}
//					}
//					else{
//						if(rotateCnt ==1500){
//							rotateCnt = 0;
//							falseCnt = 0;
//							flag = 2;
//						}
//					}
					else{
						if(rotateCnt > 2500){
							rotateCnt = 0;
							++falseCnt;
							rotateFlag = 2;
						}
					}
				}
			}
		}
	//	if(flag == 1){
	//		if(needRotateClockwise()){
	//			lastRobotState = currentRobotState;
	//			currentRobotState = COMM_CLOCK;
	//		}
	//		else if(needRotateCounterClockwise()){
	//			lastRobotState = currentRobotState;
	//			currentRobotState = COMM_CTCLOCK;
	//		}
	//	}
		if(flag == 2){
			if(needForward()){
				lastRobotState = currentRobotState;
				currentRobotState = COMM_FORWARD;
			}
			else if(needLeft()){
				lastRobotState = currentRobotState;
				currentRobotState = COMM_LEFT;
			}
			else if(needRight()){
				lastRobotState = currentRobotState;
				currentRobotState = COMM_RIGHT;
			}
			
//			else if(reachTarget(6)){
//				lastRobotState = currentRobotState;
//				currentRobotState = COMM_STOP;
//			}
//			else if(way1AllWhite()){
//				lastRobotState = currentRobotState;
//				currentRobotState = COMM_STOP;
//			}
			if(currentRobotState == COMM_FORWARD){
				if(lastRobotState == COMM_LEFT){
					for(u8 i = 0; i < 10; ++i){
						runLeft(speed);
					}
				}
				else if(lastRobotState == COMM_RIGHT){
					for(u8 i = 0; i < 10; ++i){
						runRight(speed);
					}
				}
			}
		}
		
//		if(currentRobotState == COMM_STOP && way1AllWhite() && flag == 2){
//			if(lastRobotState == COMM_LEFT){
//				currentRobotState = COMM_LEFT;
//			}
//			if(lastRobotState == COMM_RIGHT){
//				currentRobotState = COMM_RIGHT;
//			}
//			if(lastRobotState == COMM_FORWARD){
//				if(reachTarget(6)){
//					currentRobotState = COMM_STOP;
//					lastRobotState = COMM_STOP;
//					flag = 10;
//				}
//				else{
//					currentRobotState = COMM_FORWARD;
//				}
//			}
//		}

		switch(currentRobotState){
			case COMM_FORWARD:{
				runFront(speed);
				break;
			}
			case COMM_LEFT:{
				runLeft(speed);
				break;
			}
			case COMM_RIGHT:{
				runRight(speed);
				break;
			}
			case COMM_STOP:{
				runStop();
				break;
			}
			case COMM_CLOCK:{
				clockwiseRotate();
				break;
			}
			case COMM_CTCLOCK:{
				clockwiseCounterRotate();
				break;
			}
			default:{
				delay_ms(10);
				break;
			}
		}
		
		if(reachTarget(6)){
			runStop();
			currentRobotState = COMM_STOP;
			lastRobotState = COMM_STOP;
			flag = 10;
			rotateFlag = 10;
			return 1;
		}
		else{
			return 0;
		}
	}
	
	/* Have detected the rotate needed situation */
	else if(posFlag == 1){
		/* judge when to stop rotate */
		if(rotateToTrack() == 1){
			rotateFlag = 10;
			flag = 2;
//			for(u8 i = 0; i < 2; ++i){
//				if(currentRobotState == COMM_CLOCK){
//					clockwiseRotate(speed);
//				}
//				else if(currentRobotState == COMM_CTCLOCK){
//					clockwiseCounterRotate(speed);
//				}
//				else{
//					break;
//				}
//			}
		}
		else if(rotateToTrack() == 2){
			rotateFlag = 10;
			flag = 2;
//			for(u8 i = 0; i < 3; ++i){
//				if(currentRobotState == COMM_CLOCK){
//					clockwiseRotate(speed);
//				}
//				else if(currentRobotState == COMM_CTCLOCK){
//					clockwiseCounterRotate(speed);
//				}
//				else{
//					break;
//				}
//			}
		}
		else if(rotateToTrack() == 3){
			rotateFlag = 10;
			flag = 2;
		}
		else if(rotateToTrack() == 4 && flag == 1){
			rotateFlag = 10;
			flag = 2;
		}
		
		/* Move left to get the target to rotate */
		if(flag == 0){
			lastRobotState = currentRobotState;
			currentRobotState = COMM_LEFT;
			if(rotateFlag == 0){
				if(posFlag == 1){
					if(SEARCH_OUT_IO_1 == BLACK_AREA && SEARCH_OUT_IO_17 == BLACK_AREA){
						rotateFlag = 1;
						flag = 1;
					}
					else if(way2AllWhite() && SEARCH_OUT_IO_7 == BLACK_AREA){
						rotateFlag = 2; // need clockwise rotate
						flag = 1;
					}
					else if(SEARCH_OUT_IO_11 == BLACK_AREA && way1AllWhite()){
						rotateFlag = 3; // need counterclockwise rotate
						flag = 1;
					}
				}
//				else if(posFlag == 2){
//					rotateFlag = 3; // need counterclockwise rotate
//					flag = 1;
//				}
//				else if(posFlag == 3){
//					rotateFlag = 2; // need clockwise rotate
//					flag = 1;
//				}
			}
		}
		
		
		if(flag == 1){
			if(rotateFlag == 1){
				if(SEARCH_OUT_IO_4 == WHITE_AREA && stopRightFlag == 0){
					lastRobotState = currentRobotState;
					currentRobotState = COMM_LEFT;
				}
				else{
					flag = 2;
					lastRobotState = currentRobotState;
					currentRobotState = COMM_FORWARD;
					stopRightFlag = 1;
				}
			}
			else if(rotateFlag == 2){
				if(SEARCH_OUT_IO_13 == WHITE_AREA && stopRightFlag == 0){
					lastRobotState = currentRobotState;
					currentRobotState = COMM_LEFT;
					
				}
				else{
					lastRobotState = currentRobotState;
					currentRobotState = COMM_CLOCK;
					stopRightFlag = 1;
					
					++rotateCnt;
	
					if(falseCnt == 0){
						if(rotateCnt > 1500){
							rotateCnt = 0;
							++falseCnt;
							rotateFlag = 3;
						}
					}
//					else if(falseCnt != 3){
//						if(rotateCnt == 2500){
//							rotateCnt = 0;
//							++falseCnt;
//							rotateFlag = 3;
//						}
//					}
//					else{
//						if(rotateCnt ==1500){
//							rotateCnt = 0;
//							falseCnt = 0;
//							flag = 2;
//						}
//					}
					else{
						if(rotateCnt > 2500){
							rotateCnt = 0;
							++falseCnt;
							rotateFlag = 3;
						}
					}
				}
			}
			else if(rotateFlag == 3){
				if(SEARCH_OUT_IO_5 == WHITE_AREA && stopRightFlag == 0){
					lastRobotState = currentRobotState;
					currentRobotState = COMM_LEFT;
				}
				else{
					lastRobotState = currentRobotState;
					currentRobotState = COMM_CTCLOCK;
					stopRightFlag = 1;
					++rotateCnt;
					
					if(falseCnt == 0){
						if(rotateCnt > 1500){
							rotateCnt = 0;
							++falseCnt;
							rotateFlag = 2;
						}
					}
//					else if(falseCnt != 3){
//						if(rotateCnt == 2500){
//							rotateCnt = 0;
//							++falseCnt;
//							rotateFlag = 2;
//						}
//					}
//					else{
//						if(rotateCnt ==1500){
//							rotateCnt = 0;
//							falseCnt = 0;
//							flag = 2;
//						}
					else{
						if(rotateCnt > 2500){
							rotateCnt = 0;
							++falseCnt;
							rotateFlag = 2;
						}
					}
					
				}
			}
		}
	//	if(flag == 1){
	//		if(needRotateClockwise()){
	//			lastRobotState = currentRobotState;
	//			currentRobotState = COMM_CLOCK;
	//		}
	//		else if(needRotateCounterClockwise()){
	//			lastRobotState = currentRobotState;
	//			currentRobotState = COMM_CTCLOCK;
	//		}
	//	}
		if(flag == 2){
			if(needForward()){
				lastRobotState = currentRobotState;
				currentRobotState = COMM_FORWARD;
			}
			else if(needLeft()){
				lastRobotState = currentRobotState;
				currentRobotState = COMM_LEFT;
			}
			else if(needRight()){
				lastRobotState = currentRobotState;
				currentRobotState = COMM_RIGHT;
			}
			
//			else if(reachTarget(6)){
//				lastRobotState = currentRobotState;
//				currentRobotState = COMM_STOP;
//			}
//			else if(way1AllWhite()){
//				lastRobotState = currentRobotState;
//				currentRobotState = COMM_STOP;
//			}
			if(currentRobotState == COMM_FORWARD){
				if(lastRobotState == COMM_LEFT){
					for(u8 i = 0; i < 10; ++i){
						runLeft(speed);
					}
				}
				else if(lastRobotState == COMM_RIGHT){
					for(u8 i = 0; i < 10; ++i){
						runRight(speed);
					}
				}
			}
		}
		
//		if(currentRobotState == COMM_STOP && way1AllWhite() && flag == 2){
//			if(lastRobotState == COMM_LEFT){
//				currentRobotState = COMM_LEFT;
//			}
//			if(lastRobotState == COMM_RIGHT){
//				currentRobotState = COMM_RIGHT;
//			}
//			if(lastRobotState == COMM_FORWARD){
//				if(reachTarget(6)){
//					currentRobotState = COMM_STOP;
//					lastRobotState = COMM_STOP;
//					flag = 10;
//				}
//				else{
//					currentRobotState = COMM_FORWARD;
//				}
//			}
//		}

		switch(currentRobotState){
			case COMM_FORWARD:{
				runFront(30);
				break;
			}
			case COMM_LEFT:{
				runLeft(speed);
				break;
			}
			case COMM_RIGHT:{
				runRight(speed);
				break;
			}
			case COMM_STOP:{
				runStop();
				break;
			}
			case COMM_CLOCK:{
				clockwiseRotate();
				break;
			}
			case COMM_CTCLOCK:{
				clockwiseCounterRotate();
				break;
			}
			default:{
				delay_ms(10);
				break;
			}
		}
		
		if(reachTarget(6)){
			runStop();
			currentRobotState = COMM_STOP;
			lastRobotState = COMM_STOP;
			flag = 10;
			rotateFlag = 10;
			return 1;
		}
		else{
			return 0;
		}
	}
	
	/* Get error, to ensure the robot move normally */
	else{
		posFlag = 2;
	}
}

u8 nextTrack(float speed){
	if(needForward()){
		lastRobotState = currentRobotState;
		currentRobotState = COMM_FORWARD;
	}
	else if(needLeft()){
		lastRobotState = currentRobotState;
		currentRobotState = COMM_LEFT;
	}
	else if(needRight()){
		lastRobotState = currentRobotState;
		currentRobotState = COMM_RIGHT;
	}
	
	switch(currentRobotState){
		case COMM_FORWARD:{
			runFront(speed);
			break;
		}
		case COMM_LEFT:{
			runLeft(speed);
			break;
		}
		case COMM_RIGHT:{
			runRight(speed);
			break;
		}
	}
	
	if(reachTarget(6)){
		runStop();
		return 1;
	}
	else{
		return 0;
	}
}

