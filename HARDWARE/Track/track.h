#ifndef __TRACT_H
#define __TRACT_H

#include <math.h>
#include "motor.h"
#include "delay.h"

/* Front Track Module */

#define SEARCH_OUT_PIN_1 GPIO_Pin_7
#define SEARCH_OUT_GPIO_1 GPIOB
#define SEARCH_OUT_IO_1 GPIO_ReadInputDataBit(GPIOB, SEARCH_OUT_PIN_1)

#define SEARCH_OUT_PIN_2 GPIO_Pin_5
#define SEARCH_OUT_GPIO_2 GPIOB
#define SEARCH_OUT_IO_2 GPIO_ReadInputDataBit(GPIOB, SEARCH_OUT_PIN_2)

#define SEARCH_OUT_PIN_3 GPIO_Pin_3
#define SEARCH_OUT_GPIO_3 GPIOB
#define SEARCH_OUT_IO_3 GPIO_ReadInputDataBit(GPIOB, SEARCH_OUT_PIN_3)

#define SEARCH_OUT_PIN_4 GPIO_Pin_14
#define SEARCH_OUT_GPIO_4 GPIOG
#define SEARCH_OUT_IO_4 GPIO_ReadInputDataBit(GPIOG, SEARCH_OUT_PIN_4)

#define SEARCH_OUT_PIN_5 GPIO_Pin_12
#define SEARCH_OUT_GPIO_5 GPIOG
#define SEARCH_OUT_IO_5 GPIO_ReadInputDataBit(GPIOG, SEARCH_OUT_PIN_5)

#define SEARCH_OUT_PIN_6 GPIO_Pin_10
#define SEARCH_OUT_GPIO_6 GPIOG
#define SEARCH_OUT_IO_6 GPIO_ReadInputDataBit(GPIOG, SEARCH_OUT_PIN_6)

#define SEARCH_OUT_PIN_7 GPIO_Pin_7
#define SEARCH_OUT_GPIO_7 GPIOD
#define SEARCH_OUT_IO_7 GPIO_ReadInputDataBit(GPIOD, SEARCH_OUT_PIN_7)



/* Back Track Module */

#define SEARCH_OUT_PIN_11 GPIO_Pin_6
#define SEARCH_OUT_GPIO_11 GPIOA
#define SEARCH_OUT_IO_11 GPIO_ReadInputDataBit(GPIOA, SEARCH_OUT_PIN_11)

#define SEARCH_OUT_PIN_12 GPIO_Pin_4
#define SEARCH_OUT_GPIO_12 GPIOC
#define SEARCH_OUT_IO_12 GPIO_ReadInputDataBit(GPIOC, SEARCH_OUT_PIN_12)

#define SEARCH_OUT_PIN_13 GPIO_Pin_0
#define SEARCH_OUT_GPIO_13 GPIOB
#define SEARCH_OUT_IO_13 GPIO_ReadInputDataBit(GPIOB, SEARCH_OUT_PIN_13)

#define SEARCH_OUT_PIN_14 GPIO_Pin_2
#define SEARCH_OUT_GPIO_14 GPIOB
#define SEARCH_OUT_IO_14 GPIO_ReadInputDataBit(GPIOB, SEARCH_OUT_PIN_14)

#define SEARCH_OUT_PIN_15 GPIO_Pin_12
#define SEARCH_OUT_GPIO_15 GPIOF
#define SEARCH_OUT_IO_15 GPIO_ReadInputDataBit(GPIOF, SEARCH_OUT_PIN_15)

#define SEARCH_OUT_PIN_16 GPIO_Pin_14
#define SEARCH_OUT_GPIO_16 GPIOF
#define SEARCH_OUT_IO_16 GPIO_ReadInputDataBit(GPIOF, SEARCH_OUT_PIN_16)

#define SEARCH_OUT_PIN_17 GPIO_Pin_0
#define SEARCH_OUT_GPIO_17 GPIOG
#define SEARCH_OUT_IO_17 GPIO_ReadInputDataBit(GPIOG, SEARCH_OUT_PIN_17)


#define BLACK_AREA 1
#define WHITE_AREA 0

#define COMM_STOP 0
#define COMM_FORWARD 1
#define COMM_LEFT 2
#define COMM_RIGHT 3
#define COMM_CLOCK 4
#define COMM_CTCLOCK 5

#define TRACK_SPEED 50

typedef uint8_t u8;
typedef uint16_t u16;

/* Judge how to move */
void GPIOCLKInit(void);
void RedRayInit(void);
u8 wayAllBlack(void); 
u8 way1AllWhite(void); 
u8 way2AllWhite(void); 
u8 needLeft(void);
u8 needRight(void);
u8 needForward(void);
u8 reachTarget(u8 target);
u8 canRotate(void);
u8 rotateToTrack(void);
u8 needRotateClockwise(void);
u8 needRotateCounterClockwise(void);

/* Move Command */
void runFront(float speed);
void runBack(float speed);
void runLeft(float speed);
void runRight(float speed);
void runStop(void);
void clockwiseRotate(void);
void clockwiseCounterRotate(void);

void redRayDebug(void);

void stateInit(void);
u8 startTrack(float speed);
u8 nextTrack(float speed);

#endif


