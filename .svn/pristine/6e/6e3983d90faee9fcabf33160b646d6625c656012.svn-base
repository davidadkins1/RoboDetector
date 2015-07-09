/*********************************************************************
 *
 *      Metal Detector Robot to be used with resident Bootloader
 *
 *********************************************************************
 * FileName:        Motor.c
 * Processor:       PIC18
 * Compiler:        C18 3.01+
 *
 * Author               Date        Comment
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * David Adkins       02/15/12    Original.
 ********************************************************************/

/** I N C L U D E S **********************************************************/
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include <timers.h>
#include <pwm.h>
#include <adc.h>
#include <math.h>
#define CFG_ONCE
#include "io_cfg.h"
#undef CFG_ONCE
#include "motor.h"
#include "behaviors.h"
#include "oi.h"

#include "RoboDetector_tasks.h"
/** C O N F I G U R A T I O N ********************************************************/
#define MOTOR_NOISE 10
//#define MM_PER_COUNT 7.5
#define METRIC_TO_SPEED 75
#define METRIC_SPEED (signed int)495
#define METRIC_WHEEL_BASE (float)261
#define MAX_METRIC_SPEED 1000
#define RADIAN_CONVERSION 0.0174533
#define MM_PER_RADIAN 130.5

/** Global Variables ********************************************************/
#pragma udata
extern xQueueHandle xScriptQueue;
extern unsigned char script[48];
extern SENSORS sensorPacket;

unsigned long odometer;
//xTaskHandle left_motor_task;
//xTaskHandle right_motor_task;

/** Local Variables ********************************************************/
#pragma udata
//static int velocityLeft[MAX_BEHAVIOR];
//static int velocityRight[MAX_BEHAVIOR];

static struct MOTOR_CONTROL_REQUESTS motorControl[MAX_BEHAVIOR];
//static enum BEHAVIORS behaviorControl[MAX_BEHAVIOR];
static enum BEHAVIORS behaviorPriority[MAX_BEHAVIOR];

static xMotorControlParameters *LeftMotor;
static xMotorControlParameters *RightMotor;
static float b2r;
static int speedLeft;
static int speedRight;

static unsigned char avgCount;
static unsigned int motorCurrent;
static portTickType timeOut;
	
static int leftEscapeSpeed;
static int rightEscapeSpeed;
static enum ESCAPE_STATES escapeState = NORMAL_CRUISE;

static int vError;
static int abs_speed;
static long avg_emf;
static portTickType delta_time;
static unsigned long current_velocity;
static DWORD_VAL current_distance;
static DWORD_VAL distance_delta_accumulator;
        
    

/*-----------------------------------------------------------*/
#pragma code
static void motor_drive( struct MOTOR_CONTROL_REQUESTS *motor_parameters );
static void vMotorDriveLeft( int speed, unsigned long distance );
static void vMotorDriveRight( int speed, unsigned long distance );


void vMotorInitialize( void )
{
	enum BEHAVIORS i;

	ADCON1 = 0x06;

	// Stop the motors by disabling them
	LMF = LM_FORWARD_DISABLE;		// LMF = LMR is a stop condition
	LMR = LM_REVERSE_DISABLE;
	RMF = RM_FORWARD_DISABLE;		// RMF = RMR is a stop condition
	RMR = RM_REVERSE_DISABLE;

	// Motor control pins are outputs
	TRIS_RMR = OUTPUT_PIN;			// The right motor reverse output pin
	TRIS_RMF = OUTPUT_PIN;			// The right motor foward output pin
	TRIS_RMS = OUTPUT_PIN;			// The right motor PWM output pin

	TRIS_LMR = OUTPUT_PIN;			// The left motor reverse output pin
	TRIS_LMF = OUTPUT_PIN;			// The left motor foward output pin
	TRIS_LMS = OUTPUT_PIN;			// The left motor PWM output pin

	// Stop the motors by disabling them
	LMS = LM_STOP;					// Speed is stop
	RMS = RM_STOP;
	//LMS = LM_START;				// For testing
	//RMS = RM_START;

	// Now setup the PWM speed control
	OpenPWM1( PWM_PERIOD );
	OpenPWM2( PWM_PERIOD );
	OpenTimer2( TIMER_INT_OFF + T2_PS_1_1 + T2_POST_1_1 );
	LeftMotorSpeed( SPEED_0 );
	RightMotorSpeed( SPEED_0 );


	// Next setup the motor current measurement
	//ADCON1 = 0x0B;					// PORTA Bit 5 is digital
	TRIS_RM_CURRENT = INPUT_PIN;	// AN2 measures right motor current
	TRIS_LM_CURRENT = INPUT_PIN;	// AN3 measures left motor current
	OpenADC( 	ADC_FOSC_64 & ADC_RIGHT_JUST & ADC_12_TAD,
				ADC_CH2 & ADC_INT_OFF & ADC_REF_VDD_VSS,
				0xB );

	// Next setup the Back-EMF measurement
	TRIS_RMA_EMF = INPUT_PIN;		// AN0 measures right motor EMF +lead
	TRIS_RMB_EMF = INPUT_PIN;		// AN1 measures right motor EMF -lead
	TRIS_LMA_EMF = INPUT_PIN;		// AN4 measures right motor EMF +lead
	TRIS_LMB_EMF = INPUT_PIN;		// AN8 measures right motor EMF -lead

	//TRIS_TEST_PIN = OUTPUT_PIN;
	//TEST_PIN = 0;
	OpenTimer1(TIMER_INT_OFF & T1_16BIT_RW & T1_SOURCE_INT & T1_PS_1_8 & T1_OSC1EN_OFF & T1_SYNC_EXT_OFF);
	WriteTimer1( (unsigned int)SPEED_0 );

	// Vacuum behavior initialization
	behaviorPriority[0] = REMOTE_CONTROL;
	behaviorPriority[1] = ESCAPE;
	behaviorPriority[2] = CRUISE;

	for( i = NO_CONTROL_REQUEST; i < MAX_BEHAVIOR; i++ )
	{
		motorControl[i].BehaviorControl = NO_CONTROL_REQUEST;			// No behavior asking for control
		motorControl[i].left_motor.Speed = SPEED_0;
		motorControl[i].left_motor.Distance = 0;
		motorControl[i].right_motor.Speed = SPEED_0;
		motorControl[i].right_motor.Distance = 0;
	}

	/* Create the structures used to pass parameters to the left motor task. */
	LeftMotor = ( xMotorControlParameters * ) pvPortMalloc( sizeof( xMotorControlParameters ) );
	
	if( LeftMotor != NULL )
	{
		LeftMotor->Select = LEFT_MOTOR;
		LeftMotor->State = STOPPED;
		LeftMotor->Speed = 0;
		LeftMotor->CurrentVelocity = 0;
		LeftMotor->MotorCurrent_channel = ADC_CH3;
		LeftMotor->MotorCurrent = 0;
		LeftMotor->LastEMFTime = 0;
		LeftMotor->EMFTime = 0;
		LeftMotor->MotorEMF = 0;
		LeftMotor->LastEMF = 0;
		LeftMotor->MotorEMFA_channel = ADC_CH4;
		LeftMotor->MotorEMFB_channel = ADC_CH8;
		LeftMotor->MotorEMFA = 0;
		LeftMotor->MotorEMFB = 0;
		LeftMotor->Direction = LM_FORWARD_DISABLE;
		LeftMotor->CurrentDirection = LM_FORWARD_DISABLE;
		LeftMotor->Distance = 0;
		//xTaskCreate(vMotorMonitor, (const portCHAR * const)"L", motorSTACK_SIZE, (void *)LeftMotor, MOTOR_MONITOR_PRIORITY, &left_motor_task);
	}
		
	/* Create the structures used to pass parameters to the left motor task. */
	RightMotor = (xMotorControlParameters *)pvPortMalloc(sizeof( xMotorControlParameters));
	
	if( RightMotor != NULL )
	{
		RightMotor->Select = RIGHT_MOTOR;
		RightMotor->State = STOPPED;
		RightMotor->Speed = 0;
		RightMotor->CurrentVelocity = 0;
		RightMotor->MotorCurrent_channel = ADC_CH2;
		RightMotor->MotorCurrent = 0;
		RightMotor->LastEMFTime = 0;
		RightMotor->EMFTime = 0;
		RightMotor->MotorEMF = 0;
		RightMotor->LastEMF = 0;
		RightMotor->MotorEMFA_channel = ADC_CH0;
		RightMotor->MotorEMFB_channel = ADC_CH1;
		RightMotor->MotorEMFA = 0;
		RightMotor->MotorEMFB = 0;
		RightMotor->Direction = RM_FORWARD_DISABLE;
		RightMotor->CurrentDirection = RM_FORWARD_DISABLE;
		RightMotor->Distance = 0;
		//xTaskCreate( vMotorMonitor, ( const portCHAR * const ) "R", motorSTACK_SIZE, ( void * )RightMotor, MOTOR_MONITOR_PRIORITY, &right_motor_task );
	}	

	// Force Remote control
	//vMotorDriveRequest( SPEED_0, SPEED_0, 0, REMOTE_CONTROL );	

	//xTaskCreate( vMotorArbitration, ( const portCHAR * const ) "Drive", arbiterSTACK_SIZE, NULL, MOTOR_ARBITRATION_PRIORITY, ( xTaskHandle * ) NULL );	
}

static void vArbitrate( void )
{
	enum BEHAVIORS i;
	enum BEHAVIORS winner = NO_CONTROL_REQUEST;
	
	// Step through the behaviors. Highest priority first
	for( i = 0; i < MAX_BEHAVIOR; i++ )
	{
/*
		// if this priority behavior requires control
		// then grant it.
		if( behaviorControl[behaviorPriority[i]] != NO_CONTROL_REQUEST )
		{
			// motor control code goes here
			winner = behaviorControl[behaviorPriority[i]];
			vMotorDrive( velocityLeft[winner], velocityRight[winner] );
			break;
		}
*/

		// if this priority behavior requires control
		// then grant it.
		if( motorControl[behaviorPriority[i]].BehaviorControl != NO_CONTROL_REQUEST )
		{
			// motor control code goes here
			winner = motorControl[behaviorPriority[i]].BehaviorControl;

			motor_drive( &(motorControl[winner]) );
			break;
		}
	}
}

#pragma interruptlow vMotorISR
void vMotorISR( void )
{
    PIE1bits.TMR1IE = 0;
    PIR1bits.TMR1IF = 0;

	if(1 == LMF)
	{
	    LMF = 0;
	}

	if(1 == RMF)
	{
	    RMF = 0;
	}
}

static void motor_left_speed(unsigned int time_in)
{
    PIE1bits.TMR1IE = 0;
    PIR1bits.TMR1IF = 0;
	WriteTimer1( time_in );
    PIE1bits.TMR1IE = 1;
	LMF = 1;
}

static void motor_right_speed(unsigned int time_in)
{
    PIE1bits.TMR1IE = 0;
    PIR1bits.TMR1IF = 0;
	WriteTimer1( time_in );
    PIE1bits.TMR1IE = 1;
	RMF = 1;
}

/********************************************************************
*    Function Name:	vMotorMonitor									*
*    Return Value:	none											*
*    Parameters:	data:                         					*
*    Description:	Read motor current and speed. Run motor			*
*					control Arbiter.								*				
********************************************************************/
portTASK_FUNCTION( vMotorMonitor, pvParameters )
{
	// Start the default CRUISE behavior after a two second delay
    vTaskDelay(2000);
	//xQueueSendToBack( xScriptQueue, ( void * ) script, 0 );	
	//vMotorDriveRequest( 270, 270, 440, CRUISE );
	//vMotorDriveRequest(270, 270, 0, CRUISE );
	//vMotorDriveRadius( -1, 270, 0, REMOTE_CONTROL );
	

	for( ;; )
	{
		if((STOPPED == LeftMotor->State) && (STOPPED == RightMotor->State))
		{
    		if((0 != LeftMotor->Speed) || (0 != RightMotor->Speed))
    		{
        		RightMotor->State = RUNNING;
        		LeftMotor->State = RUNNING;
    		}
    		else
    		{
        		RightMotor->CurrentDistance = 0;
        		LeftMotor->CurrentDistance = 0;
        		LeftMotor->LastEMFTime = 0;
        		LeftMotor->EMFTime = 0;
        		RightMotor->EMFTime = 0;
        		RightMotor->LastEMFTime = 0;
        		//vTaskDelay(10);
    		}
		}
		//else if((RUNNING == LeftMotor->State) || (RUNNING == RightMotor->State))
		{
/*
    		motorCurrent = 0;
    		SetChanADC( LeftMotor->MotorCurrent_channel );
    	
    		for( avgCount = 0; avgCount < 16; avgCount++ )
    		{
    			//If motor is running wait for a rising edge
    			timeOut = xTaskGetTickCount( ) + 2;
    			while( (PORTCbits.RC2 == 1) && (LMF != LMR) && (timeOut > xTaskGetTickCount( )) );
    			timeOut = xTaskGetTickCount( ) + 2;
    			while( (PORTCbits.RC2 == 0) && (LMF != LMR) && (timeOut > xTaskGetTickCount( )) );
    
    			// Read the analog value from the current sensor
    			ConvertADC( );
    			while( BusyADC( ) );
    			motorCurrent += ReadADC( );
    		}
    	
    		LeftMotor->MotorCurrent = motorCurrent >> 4;
    		
    		//If motor is running wait for a rising edge
            SetChanADC( RightMotor->MotorCurrent_channel );
    		motorCurrent = 0;
    
    		for( avgCount = 0; avgCount < 16; avgCount++ )
    		{
    			//If motor is running wait for a rising edge
    			timeOut = xTaskGetTickCount( ) + 2;
    			while( (PORTCbits.RC1 == 1) && (RMF != RMR) && (timeOut > xTaskGetTickCount( )) );
    			timeOut = xTaskGetTickCount( ) + 2;
    			while( (PORTCbits.RC1 == 0) && (RMF != RMR) && (timeOut > xTaskGetTickCount( )) );
    
    			ConvertADC( );			
    			while( BusyADC( ) );
    			motorCurrent += ReadADC( );
    		}
    	
    		RightMotor->MotorCurrent = motorCurrent >> 4;
*/    		
    		// Start a Back-EMF speed measurement
    		// Save the motor drive state
   			LeftMotor->LastEMFTime = LeftMotor->EMFTime;
   			RightMotor->LastEMFTime = RightMotor->EMFTime;
    		//TEST_PIN = 1;
    
    		// Set the motors to free wheel
            motor_left_speed(SPEED_0);            
    		vTaskDelay( (portTickType) 5 );	// Allow back EMF to settle
    
    		SetChanADC( LeftMotor->MotorEMFA_channel );
    		ConvertADC( );
    		while( BusyADC( ) );
    		LeftMotor->MotorEMFA = ReadADC( );
    
    		SetChanADC( LeftMotor->MotorEMFB_channel );
    		ConvertADC( );
    		while( BusyADC( ) );
    		LeftMotor->MotorEMFB = ReadADC( );
   			LeftMotor->EMFTime = xTaskGetTickCount( );
    
    		// Set the motors to free wheel
            motor_right_speed(SPEED_0);            
    		vTaskDelay( (portTickType) 5 );	// Allow back EMF to settle
    		// Read the back-EMF voltage
    		SetChanADC( RightMotor->MotorEMFA_channel );
    		ConvertADC( );
    		while( BusyADC( ) );
    		RightMotor->MotorEMFA = ReadADC( );
    
            SetChanADC( RightMotor->MotorEMFB_channel );
    		ConvertADC( );
    		while( BusyADC( ) );		
    		RightMotor->MotorEMFB = ReadADC( );
   			RightMotor->EMFTime = xTaskGetTickCount( );
    
    		//TEST_PIN = 0;
		}

		// Restore the motor drive state
		vArbitrate( );
		//vTaskDelay( (portTickType) 500 );			// Allow motors to run
		vTaskDelay( (portTickType) 200 );			// Allow motors to run
	}
}

unsigned int uiLeftCurrent( void )
{
	return LeftMotor->MotorCurrent;
}

unsigned int uiRightCurrent( void )
{
	return RightMotor->MotorCurrent;
}

unsigned int uiLeftEMF( void )
{
	return LeftMotor->MotorEMF;
}

unsigned int uiRightEMF( void )
{
	return RightMotor->MotorEMF;
}

int iVelocityLeft( void )
{
	return LeftMotor->CurrentVelocity;
}

int iVelocityRight( void )
{
	return RightMotor->CurrentVelocity;
}

static void vMotorDriveLeft( int speed, unsigned long distance )
{
    LeftMotor->Speed = speed;
    
    // Save the current back EMF as last (T1 EMF)
    LeftMotor->LastEMF = LeftMotor->MotorEMF;
    	
    // Calculate the voltage across the motor (T2 EMF)
    if(LeftMotor->MotorEMFA > LeftMotor->MotorEMFB)
    {
	    LeftMotor->MotorEMF = (LeftMotor->MotorEMFA / 4) - (LeftMotor->MotorEMFB / 4);
    }
    else
    {
	    LeftMotor->MotorEMF = (LeftMotor->MotorEMFB / 4) - (LeftMotor->MotorEMFA / 4);
    }
    
    if(speed < 0)
    {
        abs_speed = -speed;
    }
    else
    {
        abs_speed = speed;
    }

	// For proportional control: Error = SP - PV (Set Point - Process Variable)
	// SP = requested speed
	// PV = Motor back EMF
	vError = abs_speed - LeftMotor->MotorEMF;
	LeftMotor->CurrentVelocity += vError;
	
	// Now calculate the distance moved between T1 EMF and T2 EMF readings
	if(0 == LeftMotor->LastEMFTime)
	{
    	LeftMotor->LastEMFTime = LeftMotor->EMFTime;
	}
	
	avg_emf = (LeftMotor->LastEMF + LeftMotor->MotorEMF) / 2;
	
	if(avg_emf < 0)
	{
    	avg_emf = -avg_emf;
	}

	if(avg_emf < MOTOR_NOISE)
	{
    	avg_emf = 0;
	}

    // convert the average EMF to the current velocity
    // METRIC_TO_SPEED is scaled by 10 so the current velocity must be too	
	current_velocity = (avg_emf * METRIC_TO_SPEED) / 10;
	
	// To calculate distance traveled between T1 and T2
	// we need the delta time
	delta_time = LeftMotor->EMFTime - LeftMotor->LastEMFTime;
    
    //LeftMotor->CurrentDistance += (((avg_emf * 7) + (avg_emf / 2)) * delta_time) / 1000;
    //LeftMotor->CurrentDistance += ((current_velocity * delta_time) / 1000);
    current_distance.Val = (current_velocity * delta_time) / 1000;
    
    LeftMotor->CurrentDistance += current_distance.Val;

	if( LeftMotor->CurrentVelocity < -SPEED_100 )
	{
		LeftMotor->CurrentVelocity = -SPEED_100;
	}
	else if(LeftMotor->CurrentVelocity > SPEED_100)
	{
		LeftMotor->CurrentVelocity = SPEED_100;
	}

    distance_delta_accumulator.Val = 0;

    // Protect from interrupts
    //taskENTER_CRITICAL();
    //Convert from big endian to little
    distance_delta_accumulator.byte.HB = sensorPacket.sensor.iDistanceTraveled.byte.LB;
    distance_delta_accumulator.byte.LB = sensorPacket.sensor.iDistanceTraveled.byte.HB;

	if( speed == 0 )
	{
		//LMF = LM_FORWARD_DISABLE;
		//LMR = LM_REVERSE_DISABLE;
		LeftMotor->CurrentVelocity = 0;
		LeftMotor->CurrentDistance = 0;
		LeftMotor->State = STOPPED;
		abs_speed = 1500;
	}
	else if( speed > 0 )
	{
		//LMF = LM_FORWARD_ENABLE;
		//LMR = LM_REVERSE_DISABLE;
		abs_speed = 1500 + LeftMotor->CurrentVelocity;
		distance_delta_accumulator.sVal += current_distance.sVal;
	}
	else
	{
		//LMF = LM_FORWARD_DISABLE;
		//LMR = LM_REVERSE_ENABLE;
		abs_speed = 1500 - LeftMotor->CurrentVelocity;
		distance_delta_accumulator.sVal -= current_distance.sVal;
	}
	
	if(distance_delta_accumulator.sVal > 32767)
	{
    	distance_delta_accumulator.sVal = 32767;
	}
	else if(distance_delta_accumulator.sVal < -32768)
	{
    	distance_delta_accumulator.sVal = -32768;
	}
    
    //Convert from big endian to little
    sensorPacket.sensor.iDistanceTraveled.byte.LB = distance_delta_accumulator.byte.HB;
    sensorPacket.sensor.iDistanceTraveled.byte.HB = distance_delta_accumulator.byte.LB;
    
    //motor_left_speed(65535 - abs_speed + 1);
    motor_left_speed(65535 - abs_speed + 1);
	vTaskDelay( (portTickType) 10 );			// Allow motors to run
/*
	if( LeftMotor->CurrentVelocity < 0 )
	{
		LeftMotorSpeed(-LeftMotor->CurrentVelocity);
	}
	else
	{
		LeftMotorSpeed(LeftMotor->CurrentVelocity);
	}
*/
}

static void vMotorDriveRight( int speed, unsigned long distance )
{
	//int vError;
	//int abs_speed;
	//long avg_emf;
	//portTickType delta_time;
	//unsigned long current_velocity;

    RightMotor->Speed = speed;

    // Save the current back EMF as last (T1 EMF)
    RightMotor->LastEMF = RightMotor->MotorEMF;
    
    // Calculate the voltage across the motor
    if(RightMotor->MotorEMFA > RightMotor->MotorEMFB)
    {
    	RightMotor->MotorEMF = (RightMotor->MotorEMFA / 4) - (RightMotor->MotorEMFB / 4);
    }
    else
    {
    	RightMotor->MotorEMF = (RightMotor->MotorEMFB / 4) - (RightMotor->MotorEMFA / 4);
    }
    
    #ifdef __DEBUG
    //RightMotor->MotorEMF = LeftMotor->MotorEMF;
    
    if(0 != speed)
    {
        Nop();
    }
    #endif

    if(speed < 0)
    {
        abs_speed = -speed;
    }
    else
    {
        abs_speed = speed;
    }

	// For proportional control Error = SP - PV or Set Point - Process Variable
	// SP = requested speed
	// PV = Motor back EMF
	vError = abs_speed - RightMotor->MotorEMF;
	RightMotor->CurrentVelocity += vError;

 	// Now calculate the distance moved between T1 EMF and T2 EMF readings
/*
	if(0 == RightMotor->LastEMFTime)
	{
    	RightMotor->LastEMFTime = RightMotor->EMFTime;
	}
	
	avg_emf = (RightMotor->LastEMF + RightMotor->MotorEMF) / 2;
	
	if(avg_emf < 0)
	{
    	avg_emf = -avg_emf;
	}

    // convert the average EMF to the current velocity
    // METRIC_TO_SPEED is scaled by 10 so the current velocity must be too	
	current_velocity = (avg_emf * METRIC_TO_SPEED) / 10;
	
	// To calculate distance traveled between T1 and T2
	// we need the delta time
	delta_time = RightMotor->EMFTime - RightMotor->LastEMFTime;
    
    //LeftMotor->CurrentDistance += (((avg_emf * 7) + (avg_emf / 2)) * delta_time) / 1000;
    RightMotor->CurrentDistance += ((current_velocity * delta_time) / 1000);
*/

	if( RightMotor->CurrentVelocity < -SPEED_100 )
	{
		RightMotor->CurrentVelocity = -SPEED_100;
	}
	else if( RightMotor->CurrentVelocity > SPEED_100 ) 
	{
		RightMotor->CurrentVelocity = SPEED_100;
	}

	if( speed == 0 )
	{
		//RMF = RM_FORWARD_DISABLE;
		//RMR = RM_REVERSE_DISABLE;
		RightMotor->CurrentVelocity = 0;
		RightMotor->CurrentDistance = 0;
		RightMotor->State = STOPPED;
		abs_speed = 1500;
	}
	else if( speed > 0 )
	{
		//RMF = RM_FORWARD_ENABLE;
		//RMR = RM_REVERSE_DISABLE;
		abs_speed = 1500 + RightMotor->CurrentVelocity;
	}
	else
	{
		//RMF = RM_FORWARD_DISABLE;
		//RMR = RM_REVERSE_ENABLE;
		abs_speed = 1500 - RightMotor->CurrentVelocity;
	}
	
    motor_right_speed(65535 - abs_speed + 1);
	vTaskDelay( (portTickType) 10 );			// Allow motors to run
/*
	if( RightMotor->CurrentVelocity < 0 )
	{
		RightMotorSpeed( -RightMotor->CurrentVelocity );
	}
	else
	{
		RightMotorSpeed( RightMotor->CurrentVelocity );
	}
*/
}

static void motor_drive( struct MOTOR_CONTROL_REQUESTS *motor_parameters )
{
    odometer = motor_parameters->left_motor.Distance;

	if( motor_parameters->left_motor.Speed == 0 )
	{
		vMotorDriveLeft( motor_parameters->left_motor.Speed, motor_parameters->left_motor.Distance );
	}
	//else if( motor_parameters->left_motor.Speed >= METRIC_SPEED )
	//{
	//	vMotorDriveLeft( 1, motor_parameters->left_motor.Distance );
	//}-
	else
	{
    	// speed is in mm/s
    	// each back EMF count = METRIC_TO_SPEED mm
    	// so back EMF target = speed / METRIC_TO_SPEED
    	// METRIC_TO_SPEED is scaled by 10 so the requested speed must be too
		vMotorDriveLeft((motor_parameters->left_motor.Speed * 10) / METRIC_TO_SPEED, motor_parameters->left_motor.Distance);
		//vMotorDriveLeft(36, motor_parameters->left_motor.Distance);
	}
		
	if( motor_parameters->right_motor.Speed == 0 )
	{
		vMotorDriveRight( motor_parameters->right_motor.Speed, motor_parameters->right_motor.Distance );
	}
	//else if( motor_parameters->right_motor.Speed >= METRIC_SPEED )
	//{
	//	vMotorDriveRight( 1, motor_parameters->right_motor.Distance );
	//}
	else
	{
    	// speed is in mm/s
    	// each back EMF count = METRIC_TO_SPEED mm
    	// so back EMF target = speed / METRIC_TO_SPEED
    	// METRIC_TO_SPEED is scaled by 10 so the requested speed must be too
		vMotorDriveRight((motor_parameters->right_motor.Speed * 10) / METRIC_TO_SPEED, motor_parameters->right_motor.Distance );
		//vMotorDriveRight(36, motor_parameters->right_motor.Distance );		
	}
    
	if( 0 != odometer)
	{
    	if( LeftMotor->CurrentDistance >= odometer)
    	{
        	// Execute an immediate stop
        	vMotorDriveLeft(0,0);
        	vMotorDriveRight(0,0);
        	vMotorDriveRequest(0, 0, 0, REMOTE_CONTROL);
        	odometer = 0;
    	}
	}
}


void vMotorDriveRequest( int leftSpeed, int rightSpeed, long distance, enum BEHAVIORS behaviorID )
{
	//velocityLeft[behaviorID] = leftSpeed;
	//velocityRight[behaviorID] = rightSpeed;
	//behaviorControl[behaviorID] = behaviorID;
	if( leftSpeed > MAX_METRIC_SPEED )
	{
		leftSpeed = MAX_METRIC_SPEED;
	}
	else if( leftSpeed < -MAX_METRIC_SPEED )
	{
		leftSpeed = -MAX_METRIC_SPEED;
	}
		
	if( rightSpeed > MAX_METRIC_SPEED )
    {
		rightSpeed = MAX_METRIC_SPEED;
    }
	else if( rightSpeed < -MAX_METRIC_SPEED )
	{
		rightSpeed = -MAX_METRIC_SPEED;
	}
		
	motorControl[behaviorID].left_motor.Speed = leftSpeed;
	motorControl[behaviorID].left_motor.Distance = distance;
	motorControl[behaviorID].right_motor.Speed = rightSpeed;
	motorControl[behaviorID].right_motor.Distance = distance;
	motorControl[behaviorID].BehaviorControl = behaviorID;
    //odometer = distance;
}

// radius:
//   0      = Straight, also allowed are 32767 and -32768 (0x7fff and 0x8000)
//            negative speed is reverse, positive speed is forward
//   1      = Spin counter clockwise if positive spped
//  -1	    = Spin clockwise if positive speed
void vMotorDriveRadius( int radius, int speed, unsigned long distance, enum BEHAVIORS behaviorID )
{
	//int speedLeft, speedRight;
	//float b2r;
	
	if( (radius == 0) || (radius == (int)32767) || (radius == (int)-32768) )
	{
		vMotorDriveRequest( speed, speed, distance, behaviorID );
	}
	else if( radius == (int)-1 )
	{
		vMotorDriveRequest( speed, -speed, distance, behaviorID );
	}
	else if( radius == (int)1 )
	{
		vMotorDriveRequest( -speed, speed, distance, behaviorID );
	}
	else
	{
		b2r = METRIC_WHEEL_BASE / (2 * radius);
		speedLeft = speed * (1 - b2r);
		speedRight = speed * (1 + b2r);
		vMotorDriveRequest( speedLeft, speedRight, distance, behaviorID );
	}
}

/********************************************************************
*    Function Name:	vMotorEscape									*
*    Return Value:	none											*
*    Parameters:	data:                         					*
*    Description:	Monitor motor current and speed.				*
*					Backup if current too high.						*				
********************************************************************/
portTASK_FUNCTION( vMotorEscape, pvParameters )
{
	for( ;; )
	{
		if( RightMotor->MotorCurrent > 600 )
		{
			if( RMF == RM_FORWARD_ENABLE )				// Determine stall direction
			{
				leftEscapeSpeed = -40;
				rightEscapeSpeed = -20;
			}
			else
			{
				leftEscapeSpeed = 27;
				rightEscapeSpeed = 27;
			}

			escapeState = STALL;
		}
		//else if( (leftMotorCurrent > (currentVelocityLeft * 3)) || (leftMotorCurrent > 450) )
		else if( LeftMotor->MotorCurrent > 600 )
		{
			if( RMF == RM_FORWARD_ENABLE )				// Determine stall direction
			{
				leftEscapeSpeed = -20;
				rightEscapeSpeed = -40;
			}
			else
			{
				leftEscapeSpeed = 27;
				rightEscapeSpeed = 27;
			}

			escapeState = STALL;
		}

		if( escapeState == STALL )
		{
			vMotorDriveRequest( 0, 0, 0, ESCAPE );			// Stop
			escapeState = STALL_STOP;
		}
		else if( escapeState == STALL_STOP )
		{
			if( (RMF == RMR) && (LMF == LMR) )			// Wait for stop
			{
				escapeState = MOVE_AWAY;
				vMotorDriveRequest( leftEscapeSpeed, rightEscapeSpeed, 0, ESCAPE );	// Move and turn away
			}
		}
		else if( escapeState == MOVE_AWAY )
		{
			if( (RMF != RMR) && (LMF != LMR) )			// Wait for go
			{
				vTaskDelay( (portTickType) 2500 );		// Go for 2 seconds
				vMotorDriveRequest( -rightEscapeSpeed, -leftEscapeSpeed, 0, ESCAPE );	// Drive foward and turn more
				escapeState = TURN_FORWARD;
			}
		}
		else if( escapeState == TURN_FORWARD )
		{
    		motorControl[ESCAPE].BehaviorControl = NO_CONTROL_REQUEST;
			//behaviorControl[ESCAPE] = NO_CONTROL_REQUEST;	// Escape behavior termination
			escapeState = NORMAL_CRUISE;
		}

		vTaskDelay( (portTickType) 1000 );				// Multi-task for one second
	}
}

void vMotorTurnHeading( int radius, int speed, unsigned int new_heading, enum BEHAVIORS behaviorID )
{
	int new_radius = 0;
	unsigned int degrees_turn;
	float radians_turn;
	unsigned long distance;
	
	unsigned int heading = current_heading();
	
	if(heading < new_heading)
	{
    	if((new_heading - heading) > 180)
    	{
        	new_radius = (int)1;
        	degrees_turn = (heading + 360) - new_heading; 
    	}
    	else
    	{
        	new_radius = (int)-1;
        	degrees_turn = new_heading - heading; 
    	}
	}
	else if(heading > new_heading)
	{
    	if((heading - new_heading) > 180)
    	{
        	new_radius = (int)-1;
        	degrees_turn = (new_heading + 360) - heading; 
    	}
    	else
    	{
        	new_radius = (int)1;
        	degrees_turn = heading - new_heading; 
    	}
	}
	else if(heading == new_heading)
	{
    	return;
	}
	
   	radians_turn = (float)degrees_turn * RADIAN_CONVERSION;
   	
	// radius = 0 is just a simple spin
    if(radius == 0)
    {
        radius = new_radius;
    	distance = radians_turn * MM_PER_RADIAN;
    }	
	
	if( radius == (int)-1 )
	{
		vMotorDriveRequest( speed, -speed, distance, behaviorID );
	}
	else if( radius == (int)1 )
	{
		vMotorDriveRequest( -speed, speed, distance, behaviorID );
	}
	else
	{
		b2r = METRIC_WHEEL_BASE / (2 * radius);
		speedLeft = speed * (1 - b2r);
		speedRight = speed * (1 + b2r);
		distance = radians_turn * radius;
		vMotorDriveRequest( speedLeft, speedRight, distance, behaviorID );
	}
}

void vReleaseControl( enum BEHAVIORS behaviorID )
{
	motorControl[behaviorID].BehaviorControl = NO_CONTROL_REQUEST;
    vArbitrate( );
}
/*-----------------------------------------------------------*/

