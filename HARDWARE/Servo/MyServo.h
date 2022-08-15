#ifndef _MYSERVO_H
#define _MYSERVO_H
#include "sys.h"

#define Center_Angle 18670
#define Right_Angle 18000 //  +90
#define Left_Angle 19340  //  -90
#define Servo_PSC 83
#define Servo_arr 19999

void Servo_Init(void );
void Servo_Angle_Set(int angle );
#endif
