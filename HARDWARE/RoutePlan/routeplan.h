#include <math.h>
#include "motor.h"
#include "delay.h"
#include "MyServo.h"

//System Config
#define USE_SLOWLY_SPEED_UP 
//#define USE_HIGH_SPEED
#define wzy 0.70711  // sqrt(2)/2
#define MOTOR_DIAMETER	450.00f
#define WHEEL_DIAMETER  75.00f
#define LINEAR_SPEED 200   //straight speed
#define LINEAR_HIGH_SPEED 500 /////////////
#define SPINNING_SPEED 80 //rotate speed
#define PI 3.141592768f
#define CSPDERR 5
#define CPOSERR 50
#define theta -PI/4		   //Coordinate transformation
#define LINEAR_ERR 1.05f   //Correcting straight-line deviation
#define SPINNING_ERR 1.025f  //Correcting rotate deviation
#define Length_Speed_K   0.00053  //0.00063 //kx
#define Length_Speed_Ky  0.00053  //ky


/*¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¹ì¼£²¿·Ö¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª*/
void CoordinatePositionMovement(float CurrentX, float CurrentY, float ExpectedX, float ExpectedY);  //straight
void CoordinatePositionMovement_Speed(float CurrentX, float CurrentY, float ExpectedX, float ExpectedY);
void AngularRotationMovement(float CurrentA, float ExpectedA); //rotate
void Lift_Drop_box(float s);	//Cantilever Lift or Drop (related to the winding direction)
void routeplan(int s, int e, int startORend); //routeplan ,s -> start point; e -> end point  
void liftplan(int upordown, int state);  //upordown£º milk box is up or down£» state: milk state¡£
void dropplan(int e, int state); //e end point; state: milk box state 
bool stopCspdJudging(float cspderr,int x);  //The speed control process decides whether to stop
bool stopCposJudging(float cposerr,int x);	//The position control process decides whether to stop
float CTransX(float x,float y);  //Coordinate transformation -> X
float CTransY(float x,float y);  //Coordinate transformation -> Y
int sign(float a); //sign: if >0 =1; if <0 =-1; if ==0 =0.
void Line_Turn(float Vexpx ,float Vexpy,float Angle_Speed);
void Speed_Disassemble(float Vx,float Vy , float Angle);





















