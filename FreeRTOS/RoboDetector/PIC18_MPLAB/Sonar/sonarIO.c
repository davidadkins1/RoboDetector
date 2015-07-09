/*********************************************************************
 *
 *      Robot Research Platform
 *
 *********************************************************************
 * FileName:       	sonarIO.c
 * Dependencies:    See INCLUDES section below
 * Processor:       PIC18
 * Compiler:        C18 3.21+
 *
 * Author               Date        Comment
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * David Adkins       09/03/08    Original.
 ********************************************************************/
/** I N C L U D E S **********************************************************/
#include <delays.h>
#include <timers.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "behaviors.h"
#include "io_cfg.h"
#include "oi.h"
#include "motor.h"

/** Local Variables ********************************************************/
#pragma udata
static WORD_VAL sonarStart;
static WORD_VAL sonarEnd;
static enum ESCAPE_STATES escapeState;

// Queue to interface between the command module and the song task.
xQueueHandle xSonarQueue; 

/*-----------------------------------------------------------*/

#pragma code

void vSonarInitialize( void )
{
	const unsigned portBASE_TYPE uxSonarQueueSize = 1;
	
	SONAR_OUT = NOT_TRIGGERED;
	TRIS_SONARIO = OUTPUT_PIN;		//The pin is bi-directional. Start as an output.
	OpenTimer3( TIMER_INT_OFF & T3_16BIT_RW & T3_SOURCE_INT & T3_PS_1_4 & T3_SYNC_EXT_OFF );
	T3CONbits.T3CCP1 = 1;
	CCP2CON = 0;						// Capture mode is off
	IPR2bits.CCP2IP = 0;			// Low priority interrupt
	PIE2bits.CCP2IE = 0;
	
	// Create the queue used by the Sonar interrupt
	xSonarQueue = xQueueCreate( uxSonarQueueSize, ( unsigned portBASE_TYPE ) sizeof( portSHORT ) );

	escapeState = NORMAL_CRUISE;
}

/*-----------------------------------------------------------*/

/*
 * ISR for the sonar.
 */
#pragma interrupt vSonarISR
void vSonarISR( void )
{
	unsigned int sonarRange;
	
	if( SONAR_IN == (unsigned char)1 )
	{
		sonarStart.byte.LB = CCPR2L;
		sonarStart.byte.HB = CCPR2H;

		CCP2CON = 4;					// Capture mode every rising edge
		Nop();Nop();Nop();Nop();Nop();
	}
	else
	{
		sonarEnd.byte.LB = CCPR2L;
		sonarEnd.byte.HB = CCPR2H;
		
		sonarRange = sonarEnd.Val - sonarStart.Val;
		xQueueSendToBackFromISR( xSonarQueue, (void *)&sonarRange, 0 );
	
		PIE2bits.CCP2IE = 0;
	}

	PIR2bits.CCP2IF = 0;
}

/********************************************************************
*    Function Name:	vSonarIO										*
*    Return Value:	none											*
*    Parameters:	data:                         					*
*    Description:	Sonar I/O										*
* The PING))) returns a pulse width of 73.746 uS per inch. Since the
* Javelin pulseIn() round-trip echo time is in 8.68 uS units, this is the
* same as a one-way trip in 4.34 uS units. Dividing 73.746 by 4.34 we
* get a time-per-inch conversion factor of 16.9922 (x 0.058851).*																	*				
********************************************************************/
portTASK_FUNCTION( vSonarIO, pvParameters )
{
	unsigned int sonarData;
	
	for(;;)
	{
		TRIS_SONARIO = OUTPUT_PIN;		// The pin is bi-directional. Start as an output.
		SONAR_OUT = TRIGGERED;			// Pulse to start Sonar
		Delay10TCYx( 4 );				// Pulse 5uS (4 * 10tcy)
		SONAR_OUT = NOT_TRIGGERED;
		TMR3H = TMR3L = 0;				// Start timer at zero
		CCP2CON = 5;					// Capture start time at first rising edge
		PIR2bits.CCP2IF = 0;			// Enable capture interrupt
		PIE2bits.CCP2IE = 1;
		TRIS_SONARIO = INPUT_PIN;		//The pin is bi-directional. Switch to an input.
		
        if( xQueueReceive( xSonarQueue, (void *)&sonarData, portMAX_DELAY ) == pdTRUE )
		{
			if( escapeState == NORMAL_CRUISE )
			{
				if( sonarData < (unsigned int)0x1000 )
					escapeState = STALL_STOP;

				vTaskDelay( 20 );				// Maximum Sonar ping rate allowed
			}
			else if( escapeState == STALL_STOP )
			{
				vMotorDriveRequest( 0, 0, 0, ESCAPE );			// Stop
				escapeState = BACKUP;
				vTaskDelay( 750 );
			}
			else if( escapeState == BACKUP )
			{
				vMotorDriveRequest( -SPEED_15, -SPEED_15, 0, ESCAPE );	// Backup
				escapeState = TURN_AWAY;
				vTaskDelay( 750 );
			}
			else if( escapeState == TURN_AWAY )
			{
				vMotorDriveRequest( -SPEED_15, SPEED_15, 0, ESCAPE );	// Spin a random time
				escapeState = NORMAL_CRUISE;
				//vTaskDelay( TMR3L * 8 + 500 );
				vTaskDelay( 1875 );
				vReleaseControl( ESCAPE );		// Escape behavior termination
			}
  		}
		else
		{
			vTaskDelay( 20 );
		}
 	}
}
/**/
