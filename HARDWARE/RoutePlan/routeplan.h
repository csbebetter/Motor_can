#include <math.h>
#include "motor.h"
#include "delay.h"

#define MOTOR_DIAMETER	450.00f
#define WHEEL_DIAMETER  75.00f
#define LINEAR_SPEED 300   //straight speed
#define SPINNING_SPEED 100 //rotate speed
#define PI 3.141592768f
#define CSPDERR 50
#define CPOSERR 50
#define theta -PI/4		   //Coordinate transformation
#define LINEAR_ERR 1.05f   //Correcting straight-line deviation
#define SPINNING_ERR 1.0f  //Correcting rotate deviation



/*！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！轟治何蛍！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！*/
void CoordinatePositionMovement(float CurrentX, float CurrentY, float ExpectedX, float ExpectedY);
void AngularRotationMovement(float CurrentA, float ExpectedA);
void Lift_Drop_box(float s);
void routeplan(int s, int e, int startORend);
bool stopCspdJudging(float cspderr,int x);
bool stopCposJudging(float cposerr,int x);
float CTransX(float x,float y);
float CTransY(float x,float y);
int sign(float a);





















