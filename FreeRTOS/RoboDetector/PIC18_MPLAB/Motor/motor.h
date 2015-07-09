#ifndef MOTOR_H
#define MOTOR_H
#include "behaviors.h"

void vMotorInitialize( void );
unsigned int uiLeftCurrent( void );
unsigned int uiRightCurrent( void );
unsigned int uiLeftEMF( void );
unsigned int uiRightEMF( void );
int iVelocityLeft( void );
int iVelocityRight( void );
void vMotorDrive( int leftSpeed, int rightSpeed );
void vMotorDriveRequest( int leftSpeed, int rightSpeed, long distance, enum BEHAVIORS behaviorID );
void vMotorDriveRadius( int radius, int speed, unsigned long distance, enum BEHAVIORS behaviorID );
void vMotorTurnHeading( int radius, int speed, unsigned int new_heading, enum BEHAVIORS behaviorID );
void vReleaseControl( enum BEHAVIORS behaviorID );
unsigned int current_heading(void);

// Motor PWM speed defines
#define	PRESCALE2	4
//#define PWM_PERIOD	74					// PWM_PERIOD = (1/20000000)*4*4*1 =  50 us or 20 khz
#define STOP		PWM_PERIOD + 2		// PWM duty cycle is inverted 

#define PWM_PERIOD	50					// PWM_PERIOD = ((1/32000000)*16*4*1)/(1/10000) =  100 us or 10 khz
#define SPEED_100	220					// * 1 = 100 percent duty cycle (Always high)
#define SPEED_85	170					// * 4 = 85 percent duty cycle (75uS)
#define SPEED_75	150					// * 3 = 75 percent duty cycle (75uS)
#define SPEED_50	150					// * 2 = 50 percent duty cycle (50uS)
#define SPEED_25	50					// * 1 = 25 percent duty cycle (25uS)
#define SPEED_15	15					// * 1 = 15 percent duty cycle (15uS)
#define SPEED_0		64036				// * 0 = 0 percent duty cycle (Always low)

enum MOTOR_SELECT
{
	LEFT_MOTOR,
	RIGHT_MOTOR
};

enum MOTOR_STATE
{
	STOPPED,
	RUNNING,
	COMPLETED
};

typedef struct MOTOR_CONTROL_PARAMETERS
{
	enum MOTOR_SELECT 	Select;
	enum MOTOR_STATE	State;
	int					Speed;
	unsigned char       MotorCurrent_channel;
	unsigned char       MotorEMFA_channel;
	unsigned char       MotorEMFB_channel;
	unsigned int        MotorCurrent;
	int                 LastEMF;
	int                 MotorEMF;
	unsigned int        MotorEMFA;
	unsigned int        MotorEMFB;
	portTickType        LastEMFTime;
	portTickType        EMFTime;
	unsigned char		Direction;
	unsigned long       Distance;
	int 				CurrentVelocity;
	unsigned char 		CurrentDirection;
	unsigned long       CurrentDistance;
}xMotorControlParameters;

typedef struct BEHAVIOR_PARAMETERS
{
	int					Speed;
	unsigned long       Distance;
}MOTOR_CONTROL_PARAMETERS;

struct MOTOR_CONTROL_REQUESTS
{
	enum BEHAVIORS BehaviorControl;
	MOTOR_CONTROL_PARAMETERS left_motor;
	MOTOR_CONTROL_PARAMETERS right_motor;
};	
#endif

extern unsigned long odometer;

