#include <math.h>
#include "motor.h"
#include "delay.h"


#define SEARCH_OUT_PIN_1 GPIO_Pin_12
#define SEARCH_OUT_GPIO_1 GPIOE
#define SEARCH_OUT_IO_1 GPIO_ReadInputDataBit(GPIOE, SEARCH_OUT_PIN_1)

#define SEARCH_OUT_PIN_2 GPIO_Pin_10
#define SEARCH_OUT_GPIO_2 GPIOE
#define SEARCH_OUT_IO_2 GPIO_ReadInputDataBit(GPIOE, SEARCH_OUT_PIN_2)

#define SEARCH_OUT_PIN_3 GPIO_Pin_8
#define SEARCH_OUT_GPIO_3 GPIOE
#define SEARCH_OUT_IO_3 GPIO_ReadInputDataBit(GPIOE, SEARCH_OUT_PIN_3)

#define SEARCH_OUT_PIN_4 GPIO_Pin_1
#define SEARCH_OUT_GPIO_4 GPIOG
#define SEARCH_OUT_IO_4 GPIO_ReadInputDataBit(GPIOG, SEARCH_OUT_PIN_4)

#define SEARCH_OUT_PIN_5 GPIO_Pin_15
#define SEARCH_OUT_GPIO_5 GPIOF
#define SEARCH_OUT_IO_5 GPIO_ReadInputDataBit(GPIOF, SEARCH_OUT_PIN_5)

#define SEARCH_OUT_PIN_6 GPIO_Pin_13
#define SEARCH_OUT_GPIO_6 GPIOF
#define SEARCH_OUT_IO_6 GPIO_ReadInputDataBit(GPIOF, SEARCH_OUT_PIN_6)

#define SEARCH_OUT_PIN_7 GPIO_Pin_11
#define SEARCH_OUT_GPIO_7 GPIOF
#define SEARCH_OUT_IO_7 GPIO_ReadInputDataBit(GPIOF, SEARCH_OUT_PIN_7)

#define BLACK_AREA 1
#define WHITE_AREA 0

#define COMM_STOP 0
#define COMM_FORWARD 1
#define COMM_LEFT 2
#define COMM_RIGHT 3

typedef uint8_t u8;

void GPIOCLKInit(void);
void RedRayInit(void);
u8 wayAllBlack(void); 
u8 wayAllWhite(void); 
u8 needLeft(void);
u8 needRight(void);
u8 needForward(void);
u8 reachTarget(u8 target);

void runFront(void);
void runBack(void);
void runLeft(void);
void runRight(void);
void runStop(void);

void redRayDebug(void);

void stateInit(void);
u8 startTrack(void);




