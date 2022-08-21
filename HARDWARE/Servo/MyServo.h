#ifndef _MYSERVO_H
#define _MYSERVO_H
#include "sys.h"

#define Center_Angle 18550
#define Right_Angle 17500 //  +90
#define Left_Angle 19600  //  -90
#define Servo_PSC 83
#define Servo_arr 19999

#define Clockwise Servo_Angle_Set(82)
#define CounterClockwise Servo_Angle_Set(-85)
#define MoveToDefault Servo_Angle_Set(-0.5)

void Servo_Init(void );
void Servo_Angle_Set(float angle );
#endif
