#include "track.h"

u8 currentRobotState;
u8 lastRobotState;
static u8 flag = 0;

/*！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！儉治旗鷹◎！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！*/
/*！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！儉治旗鷹◎！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！*/
/*！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！儉治旗鷹◎！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！*/
/*！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！儉治旗鷹◎！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！*/
/*！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！儉治旗鷹◎！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！*/
void GPIOCLKInit(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA , ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB , ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC , ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD , ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE , ENABLE);
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
}


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

u8 wayAllWhite(void){
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

u8 needLeft(void){
	if(SEARCH_OUT_IO_1 == WHITE_AREA &&
		SEARCH_OUT_IO_2 == WHITE_AREA &&
		SEARCH_OUT_IO_3 == WHITE_AREA &&
		SEARCH_OUT_IO_4 == WHITE_AREA &&
		(SEARCH_OUT_IO_7 == BLACK_AREA ||
		SEARCH_OUT_IO_6 == BLACK_AREA)){
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
		SEARCH_OUT_IO_2 == BLACK_AREA)){
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
		(SEARCH_OUT_IO_3 == BLACK_AREA ||
		SEARCH_OUT_IO_4 == BLACK_AREA ||
		SEARCH_OUT_IO_5 == BLACK_AREA)){
			return 1;
		}
		else{
			return 0;
		}
}


void runFront(void){
	MotorSpeedExpected(50, -50, -50, 50, 0);
	delay_ms(1);
}

void runBack(void){
	MotorSpeedExpected(-50, 50, 50, -50, 0);
	delay_ms(1);
}

void runLeft(void){
	MotorSpeedExpected(50, 50, -50, -50, 0);
	delay_ms(1);
}

void runRight(void){
	MotorSpeedExpected(-50, -50, 50, 50, 0);
	delay_ms(1);
}

void runStop(void){
	MotorSpeedExpected(0, 0, 0, 0, 0);
	delay_ms(1);
}

void redRayDebug(void){
	if(SEARCH_OUT_IO_1 == WHITE_AREA){
		printf("1: W\n");
	}
	else{
		printf("1: B\n");
	}
	if(SEARCH_OUT_IO_2 == WHITE_AREA){
		printf("2: W\n");
	}
	else{
		printf("2: B\n");
	}
	if(SEARCH_OUT_IO_3 == WHITE_AREA){
		printf("3: W\n");
	}
	else{
		printf("3: B\n");
	}
	if(SEARCH_OUT_IO_4 == WHITE_AREA){
		printf("4: W\n");
	}
	else{
		printf("4: B\n");
	}
	if(SEARCH_OUT_IO_5 == WHITE_AREA){
		printf("5: W\n");
	}
	else{
		printf("5: B\n");
	}
	if(SEARCH_OUT_IO_6 == WHITE_AREA){
		printf("6: W\n");
	}
	else{
		printf("6: B\n");
	}
	if(SEARCH_OUT_IO_7 == WHITE_AREA){
		printf("7: W\n");
	}
	else{
		printf("7: B\n");
	}
}

void stateInit(void){
	lastRobotState = COMM_STOP;
	currentRobotState = COMM_STOP;
}

u8 startTrack(void){
	
	if(needForward()){
		lastRobotState = currentRobotState;
		currentRobotState = COMM_FORWARD;
		flag = 1;
	}
	else if(needLeft()){
		lastRobotState = currentRobotState;
		currentRobotState = COMM_LEFT;
		flag = 1;
	}
	else if(needRight()){
		lastRobotState = currentRobotState;
		currentRobotState = COMM_RIGHT;
		flag = 1;
	}
	else if(reachTarget(4)){
		lastRobotState = currentRobotState;
		currentRobotState = COMM_STOP;
		flag = 1;
	}
	else if(wayAllWhite() && flag == 0){
		lastRobotState = currentRobotState;
		currentRobotState = COMM_LEFT;
	}
	else if(wayAllWhite() && flag == 1){
		lastRobotState = currentRobotState;
		currentRobotState = COMM_STOP;
	}
	if(currentRobotState == COMM_STOP && wayAllWhite()){
		if(lastRobotState == COMM_LEFT){
			currentRobotState = COMM_LEFT;
		}
		if(lastRobotState == COMM_RIGHT){
			currentRobotState = COMM_RIGHT;
		}
		if(lastRobotState == COMM_FORWARD){
			if(reachTarget(4)){
				currentRobotState = COMM_STOP;
			}
			else{
				currentRobotState = COMM_FORWARD;
			}
		}
	}

	switch(currentRobotState){
		case COMM_FORWARD:{
			runFront();
			break;
		}
		case COMM_LEFT:{
			runLeft();
			break;
		}
		case COMM_RIGHT:{
			runRight();
			break;
		}
		case COMM_STOP:{
			runStop();
			break;
		}
		default:{
			delay_ms(10);
			break;
		}
	}
	
	if(reachTarget(4)){
		return 1;
	}
	else{
		return 0;
	}
}