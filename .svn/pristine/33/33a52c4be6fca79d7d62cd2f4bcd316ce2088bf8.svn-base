/*********************************************************************
 *
 *      Robot Research Platform
 *
 *********************************************************************
 * FileName:       	PDACmd.c
 * Dependencies:    See INCLUDES section below
 * Processor:       PIC18
 * Compiler:        C18 3.02+
 *
 * Author               Date        Comment
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * David Adkins       02/27/06    Original.
 ********************************************************************/
/** I N C L U D E S **********************************************************/
#include <p18cxxx.h>
#include <usart.h>
#include <stdio.h>
#include "io_cfg.h"

/* Scheduler header files. */
#include "FreeRTOS.h"
#include "task.h"
#include "serial.h"
#include "queue.h"
#include "RoboDetector_tasks.h"
#include "motor.h"
#include "behaviors.h"

/** Local Variables ********************************************************/
#pragma udata
enum CMD_STATE { PREFIX_1, PREFIX_2, CMD_READ, READ_SPEED_1, READ_SPEED_2 };

static enum CMD_STATE cmdState = PREFIX_2; 
static char BANNER[ ] = { "Ready!\r\n" };
static char motor_string[36];
static int string_size;


// Handle to the com port used by both tasks.
//static xComPortHandle xPort = NULL;

/** Public Prototypes ***************************************/
//extern xQueueHandle xSongQueue;
extern xQueueHandle xScriptQueue;

/** External  P R O T O T Y P E S ***************************************/

/** P R I V A T E  P R O T O T Y P E S ***************************************/
#pragma udata
#pragma code
/********************************************************************
*    Function Name:  vUARTIOControl                                     	*
*    Return Value:   none                                           *
*    Parameters:     data: data to transmit                         *
*    Description:    Host command processor done via Bluetooth		*
********************************************************************/
void vPDACmd( void )
{
	//
	// The command sent from the Host computer
	//
	unsigned portCHAR cByteRxed, cmdByte;
	//int speedL, speedR;

	for( ;; )
	{
		if( cmdState == PREFIX_1 )
		{
			cmdState = PREFIX_2;
			return;
		}

		/* Block on the queue that contains received bytes until a byte is
		available. */
		if( xUARTIOGetChar( NULL, &cByteRxed, comRX_BLOCK_TIME ) )
		{
			if( cmdState == PREFIX_2 )
			{
				if( cByteRxed == (unsigned portCHAR)0x07 )
					cmdState = CMD_READ;
				else
					cmdState = PREFIX_1;
			}
			else if( cmdState == CMD_READ )
			{
				cmdByte = cByteRxed;					// Store the command

				//Forward
				if( cmdByte == 'F' )					// Both motors forward
				{
					vMotorDriveRequest( 100, 100, 0, REMOTE_CONTROL );
					cmdState = PREFIX_1;
				}
	
				//Reverse
				else if( cmdByte == 'R' )				// Both motors reverse
				{
					vMotorDriveRequest( -100, -100, 0, REMOTE_CONTROL );
					cmdState = PREFIX_1;
				}
				// Motor status
				else if( cmdByte == 'M' )				// Both motors reverse
				{
    				string_size = sprintf(motor_string,(const far rom char *)"%03u,%05u,%05u,%+05i,%+05i\n", current_heading(), uiLeftEMF(), uiRightEMF(), iVelocityLeft(), iVelocityRight() );
    				motor_string[string_size] = 0;
    				vUARTIOputsBT(motor_string);
				}
				// Autonomous
				else if( cmdByte == 'a' )
				{
					vReleaseControl( REMOTE_CONTROL );
					cmdState = PREFIX_1;
				}
				else if( cmdByte == 'g' )
				{
					//
					// Read the analog value from the current sensor
					//
					//
					// Send the values to the PDA
					//
		
					//vUARTIOWriteBT( (char)(uiLeftCurrent( ) >> 2) );	
					//vUARTIOWriteBT( (char)(uiRightCurrent( ) >> 2) );	
					//vUARTIOWriteBT( (iVelocityLeft( ) & 0xff) );	
					//vUARTIOWriteBT( (char)(iVelocityLeft( ) >> 8 & 0xff) );	
					//vUARTIOWriteBT( (char)(iVelocityRight( ) & 0xff) );	
					//vUARTIOWriteBT( (char)(iVelocityRight( ) >> 8 & 0xff) );	
					cmdState = PREFIX_1;
				}
				else if( cmdByte == 'h' )
				{
					//
					// Am I alive? command
					//
				
					//
					// Send $ value to the PDA
					//
					//while(1)
					vUARTIOputsBT( BANNER );
					//xQueueSend( xSongQueue, ( void * ) &cmdByte, portMAX_DELAY  );
					cmdState = PREFIX_1;
				}	
				//Left
				else if( cmdByte == 'l' )							// Turn left
				{
					//vMotorDriveRadius( 1, SPEED_15, REMOTE_CONTROL );
					vMotorDriveRadius( 1, 50, 0, REMOTE_CONTROL );
					cmdState = PREFIX_1;
				}
				//Right
				else if( cmdByte == 'r' )							// Turn right
				{
					//vMotorDriveRadius( -1, SPEED_15, REMOTE_CONTROL );
					vMotorDriveRadius( -1, 50, 0, REMOTE_CONTROL );
					cmdState = PREFIX_1;
				}
				else if( cmdByte == 'S' )							// Both motors stop
				{
					vMotorDriveRequest( 0, 0, 0, REMOTE_CONTROL );
					cmdState = PREFIX_1;
				}
			}
/*
			else if( cmdState == READ_SPEED_1 )
			{
				speedL = cByteRxed;
				cmdState = READ_SPEED_2;
			}
			else if( cmdState == READ_SPEED_2 )
			{
				speedR = cByteRxed;

				//Forward
				if( cmdByte == 'F' )							// Both motors forward
				{
					//speed = fgetc( CMD_PORT );				// Read the motor speed
					vMotorDriveRequest( speedL, speedR, REMOTE_CONTROL );	
				}
		
				//Reverse
				else if( cmdByte == 'R' )							// Both motors reverse
				{
					//speed = fgetc( CMD_PORT );				// Read the motor speed
		
					vMotorDriveRequest( -speedL, -speedR, REMOTE_CONTROL );	
				}

				cmdState = PREFIX_1;
			}
*/
			else
			{
				cmdState = PREFIX_1;
			}
		}		
	}//for( ;; )
}

/*-----------------------------------------------------------*/
void ToHex(char cin, char *cout1, char *cout2)
{
    *cout1 = (cin & 0xf0) >> 4;
    
    if(*cout1 > 9)
    {
        *cout1 += 0x37;
    }
    else
    {
        *cout1 += 0x30;
    }
    
    *cout2 = (cin & 0xf);\
       
    if(*cout2 > 9)
    {
        *cout2 += 0x37;
    }
    else
    {
        *cout2 += 0x30;
    }
}

/*-----------------------------------------------------------*/
