/*********************************************************************
 *
 *      Robot Research Platform
 *
 *********************************************************************
 * FileName:       	iRobotCmd.c
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
#include "io_cfg.h"
#include "oi.h"

/* Scheduler header files. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "serial.h"
#include "RoboDetector_tasks.h"
#include "motor.h"
#include "behaviors.h"

/** Local Variables ********************************************************/
#pragma udata

/** Global Variables ********************************************************/
#pragma udata

/** Public Prototypes ***************************************/

/** External  P R O T O T Y P E S ***************************************/

/** P R I V A T E  P R O T O T Y P E S ***************************************/


/** Global Variables ********************************************************/
//extern xQueueHandle xSongQueue;
extern xQueueHandle xScriptQueue;

#pragma udata
unsigned portCHAR songs[2][17];
unsigned portCHAR script[48];

#pragma idata
SENSORS sensorPacket =
{ 0,        // Packet ID:  7 - Bumps and Wheel Drops 
  0,        // Packet ID:  8 - Wall
  0,        // Packet ID:  9 - Cliff Left
  0,        // Packet ID: 10 - Cliff Front Left
  0,        // Packet ID: 11 - Cliff Front Right
  0,        // Packet ID: 12 - Cliff Right
  0,        // Packet ID: 13 - Virtual Wall
  0,        // Packet ID: 14 - Low Side Driver and Wheel Overcurrents
  0,        // Packet ID: 15 - Unused byte
  0,        // Packet ID: 16 - Unused byte
  255,      // Packet ID: 17 - Infrared byte 
  0,        // Packet ID: 18 - Buttons
  0x0000,   // Packet ID: 19 - Distance Traveled
  0x0000,   // Packet ID: 20 - Angle Travled
  0,        // Packet ID: 21 - Charging State
  0x4038,   // Packet ID: 22 - Battery Voltage (0x3840 = 14.4v)
  0x0CFE,   // Packet ID: 23 - Battery Current (0xFE0C = -500 mA)
  26,       // Packet ID: 24 - Battery Temperature
  0x6810,   // Packet ID: 25 - Battery Charge (0x1068 = 4200 mAh)
  0x6810,   // Packet ID: 26 - Battery Capacity
  0x0000,   // Packet ID: 27 - Wall Signal
  0x0000,   // Packet ID: 28 - Cliff Left Signal
  0x0000,   // Packet ID: 29 - Cliff Front Left Signal
  0x0000,   // Packet ID: 30 - Cliff Front Right Signal
  0x0000,   // Packet ID: 31 - Cliff Right Signal
  0x1F,     // Packet ID: 32 - Cargo Bay Digital Inputs
  0x0000,   // Packet ID: 33 - Cargo Bay Analog Signal
  0,        // Packet ID: 34 - Charging Sources Available
  OI_OFF,   // Packet ID: 35 - OI Mode
  0,        // Packet ID: 36 - Song Number
  0,        // Packet ID: 37 - Song Playing
  0,        // Packet ID: 38 - Number of Stream Packets
  0x0000,   // Packet ID: 39 - Requested Velocity
  0x0000,   // Packet ID: 40 - Requested Radius
  0x0000,   // Packet ID: 41 - Requested Right Velocity
  0x0000    // Packet ID: 42 - Requested Left Velocity
};

/** Local Variables ********************************************************/
// Handle to the com port used by both tasks.
//static xComPortHandle xPort = NULL;

/** Public Prototypes ***************************************/

/** P R I V A T E  P R O T O T Y P E S ***************************************/
#pragma udata
#pragma code

/********************************************************************
*    Function Name:  iRobotCmd                                     	*
*    Return Value:   none                                           *
*    Parameters:     data: data to transmit                         *
*    Description:    iRobot command processor done via Bluetooth	*
********************************************************************/
void iRobotCmd( unsigned portCHAR cByteRxed )
{
	//
	// The command sent from the Host computer
	//
	unsigned portCHAR cmdByte, i;
	WORD_VAL leRight;
	WORD_VAL leLeft;
	#define turn_radius leRight
	DWORD_VAL leDistance;
	WORD_VAL input_heading;

	enum CMD_STATE { PREFIX_1, PREFIX_2, CMD_READ, READ_SPEED_1, READ_SPEED_2 };

	cmdByte = cByteRxed;					// Store the command
	
	leRight.Val = 0;
	leLeft.Val = 0;
	leDistance.Val = 0;
	
	//if( (sensorPacket.sensor.ucOIMode == OIOff) && (cmdByte != CmdStart) )
	//{
	//    return;
	//}
			
	switch( cmdByte )
	{
		case CmdStart:
			sensorPacket.sensor.ucOIMode = OIPassive;
		break;
		
		case CmdBaud:
		    Nop();
		    Nop();
		break;
		
		case CmdControl:
			sensorPacket.sensor.ucOIMode = OISafe;
		break;
		
		case CmdSafe:
			sensorPacket.sensor.ucOIMode = OISafe;
		break;
		
		case CmdFull:
			sensorPacket.sensor.ucOIMode = OIFull;
		break;
		
		case CmdSpot:
		break;
		
		case CmdClean:
		break;
		
		case CmdDemo:
		break;
		
		case CmdDrive:
			xUARTIOGetChar( NULL, &leLeft.byte.HB , comRX_BLOCK_TIME );
			sensorPacket.sensor.iRequestedLeftVelocity.byte.HB = leLeft.byte.HB;
			sensorPacket.sensor.iRequestedRightVelocity.byte.HB = leLeft.byte.HB;
			
			xUARTIOGetChar( NULL, &leLeft.byte.LB, comRX_BLOCK_TIME );
			sensorPacket.sensor.iRequestedLeftVelocity.byte.LB = leLeft.byte.LB;
			sensorPacket.sensor.iRequestedRightVelocity.byte.LB = leLeft.byte.LB;
			
			xUARTIOGetChar( NULL, &leRight.byte.HB, comRX_BLOCK_TIME );
			sensorPacket.sensor.iRequestedRadius.byte.HB = leRight.byte.HB;
			
			xUARTIOGetChar( NULL, &leRight.byte.LB, comRX_BLOCK_TIME );
			sensorPacket.sensor.iRequestedRadius.byte.LB = leRight.byte.LB;
			
			vMotorDriveRadius( leRight.Val, leLeft.Val, 0, REMOTE_CONTROL );
		break;
		
		case CmdMotors:
		break;
		
		case CmdLeds:
		break;
		
		case CmdSong:
			//xUARTIOGetChar( NULL, &sensorPacket.sensor.ucSongNumber, comRX_BLOCK_TIME );
		    sensorPacket.sensor.ucSongNumber = cmdByte;
			xUARTIOGetChar( NULL, &cmdByte, comRX_BLOCK_TIME );
			xUARTIOGetChar( NULL, (unsigned char *)&songs[cmdByte][0], comRX_BLOCK_TIME );
			
			for( i = 1; i < songs[cmdByte][0] * 2 + 1; i++ )
			{
				xUARTIOGetChar( NULL, (unsigned char *)&songs[cmdByte][i], comRX_BLOCK_TIME ); 
			}
		break;
		
		case CmdPlay:
			xUARTIOGetChar( NULL, &sensorPacket.sensor.ucSongPlaying, comRX_BLOCK_TIME );
			//xQueueSendToBack( xSongQueue, ( void * ) &sensorPacket.sensor.ucSongNumber, 0 );
			//xUARTIOGetChar( NULL, (void *)&i, comRX_BLOCK_TIME );
			//xQueueSend( xSongQueue, ( void * ) &i, portMAX_DELAY  );
		break;
		
		case CmdSensors:
			i = xUARTIOGetChar( NULL, &cmdByte, comRX_BLOCK_TIME );
			
			if( cmdByte == (unsigned portCHAR)0 )
			{
				vUARTIOWriteBT( (char *)sensorPacket.byte, Sen0Size );
				sensorPacket.sensor.iDistanceTraveled.Val = 0;          // Reset motion accumulators
				sensorPacket.sensor.iAngleTraveled.Val = 0;             // after read
			}
			else if( cmdByte == (unsigned portCHAR)1 )
			{
				vUARTIOWriteBT( (char *)sensorPacket.byte, Sen1Size );
			}
			else if( cmdByte == (unsigned portCHAR)2 )
			{
				vUARTIOWriteBT( (char *)&sensorPacket.byte[SenIRChar], Sen2Size );
				sensorPacket.sensor.iDistanceTraveled.Val = 0;          // Reset motion accumulators
				sensorPacket.sensor.iAngleTraveled.Val = 0;             // after read
			}
			else if( cmdByte == (unsigned portCHAR)3 )
			{
				vUARTIOWriteBT( (char *)&sensorPacket.byte[SenChargeState], Sen3Size );
			}
			else if( cmdByte == (unsigned portCHAR)4 )
			{
				vUARTIOWriteBT( (char *)&sensorPacket.byte[SenWallSig1], Sen4Size );
			}
			else if( cmdByte == (unsigned portCHAR)5 )
			{
				vUARTIOWriteBT( (char *)&sensorPacket.byte[SenOIMode], Sen5Size );
			}
			else if( cmdByte == (unsigned portCHAR)6 )
			{
				vUARTIOWriteBT( (char *)sensorPacket.byte, Sen6Size );
				sensorPacket.sensor.iDistanceTraveled.Val = 0;          // Reset motion accumulators
				sensorPacket.sensor.iAngleTraveled.Val = 0;             // after read
			}
		break;
		
		case CmdDock:
		break;
		
		case CmdPWMMotors:
		break;
		
		case CmdDriveWheels:
			xUARTIOGetChar( NULL, &leRight.byte.HB, comRX_BLOCK_TIME );
			sensorPacket.sensor.iRequestedRightVelocity.byte.HB = leRight.byte.HB;
			
			xUARTIOGetChar( NULL, & leRight.byte.LB, comRX_BLOCK_TIME );
			sensorPacket.sensor.iRequestedRightVelocity.byte.LB = leRight.byte.LB;
			
			xUARTIOGetChar( NULL, &leLeft.byte.HB, comRX_BLOCK_TIME );
			sensorPacket.sensor.iRequestedLeftVelocity.byte.HB = leLeft.byte.HB;
			
			xUARTIOGetChar( NULL, &leLeft.byte.LB, comRX_BLOCK_TIME );
			sensorPacket.sensor.iRequestedLeftVelocity.byte.LB = leLeft.byte.LB;
			
			if(0 != sensorPacket.sensor.iRequestedLeftVelocity.Val)
			{
			    Nop();
			    Nop();
			}
			else
			{
			    Nop();
			    Nop();
			}
			
			vMotorDriveRequest( leLeft.Val, leRight.Val, 0, REMOTE_CONTROL );
		break;
		
		case CmdOutputs:
		break;
		
		case CmdSensorList:
			Nop();
			Nop();
		break;
		
		case CmdIRChar:
		break;
		
		case CmdScript:
			xUARTIOGetChar( NULL, (unsigned char *)&script[0], comRX_BLOCK_TIME );
			
			for( i = 1; i < (script[0] + 1); i++ )
			{
				xUARTIOGetChar( NULL, (unsigned char *)&script[i], comRX_BLOCK_TIME ); 
			}
			
			Nop();
			Nop();
		break;
		
		case PlayScript:
		    xQueueSendToBack( xScriptQueue, ( void * ) script, 0 );
		break;
		
		case CmdDriveDistance:
			xUARTIOGetChar( NULL, &leLeft.byte.HB , comRX_BLOCK_TIME );
			sensorPacket.sensor.iRequestedLeftVelocity.byte.HB = leLeft.byte.HB;
			sensorPacket.sensor.iRequestedRightVelocity.byte.HB = leLeft.byte.HB;
			
			xUARTIOGetChar( NULL, &leLeft.byte.LB, comRX_BLOCK_TIME );
			sensorPacket.sensor.iRequestedLeftVelocity.byte.LB = leLeft.byte.LB;
			sensorPacket.sensor.iRequestedRightVelocity.byte.LB = leLeft.byte.LB;
			
			xUARTIOGetChar( NULL, &turn_radius.byte.HB, comRX_BLOCK_TIME );
			sensorPacket.sensor.iRequestedRadius.byte.HB = turn_radius.byte.HB;
			
			xUARTIOGetChar( NULL, &turn_radius.byte.LB, comRX_BLOCK_TIME );
			sensorPacket.sensor.iRequestedRadius.byte.LB = turn_radius.byte.LB;
			
			xUARTIOGetChar( NULL, &leDistance.byte.MB , comRX_BLOCK_TIME );
			xUARTIOGetChar( NULL, &leDistance.byte.UB , comRX_BLOCK_TIME );
			xUARTIOGetChar( NULL, &leDistance.byte.HB , comRX_BLOCK_TIME );
			xUARTIOGetChar( NULL, &leDistance.byte.LB , comRX_BLOCK_TIME );

			vMotorDriveRadius( turn_radius.Val, leLeft.Val, leDistance.Val, REMOTE_CONTROL );
		break;
		
		case TurnToHeading:
			xUARTIOGetChar( NULL, &leLeft.byte.HB , comRX_BLOCK_TIME );
			sensorPacket.sensor.iRequestedLeftVelocity.byte.HB = leLeft.byte.HB;
			sensorPacket.sensor.iRequestedRightVelocity.byte.HB = leLeft.byte.HB;
			
			xUARTIOGetChar( NULL, &leLeft.byte.LB, comRX_BLOCK_TIME );
			sensorPacket.sensor.iRequestedLeftVelocity.byte.LB = leLeft.byte.LB;
			sensorPacket.sensor.iRequestedRightVelocity.byte.LB = leLeft.byte.LB;
			
			xUARTIOGetChar( NULL, &turn_radius.byte.HB, comRX_BLOCK_TIME );
			sensorPacket.sensor.iRequestedRadius.byte.HB = turn_radius.byte.HB;
			
			xUARTIOGetChar( NULL, &turn_radius.byte.LB, comRX_BLOCK_TIME );
			sensorPacket.sensor.iRequestedRadius.byte.LB = turn_radius.byte.LB;
			
			xUARTIOGetChar( NULL, &input_heading.byte.HB , comRX_BLOCK_TIME );
			xUARTIOGetChar( NULL, &input_heading.byte.LB , comRX_BLOCK_TIME );

			vMotorTurnHeading( turn_radius.Val, leLeft.Val, input_heading.Val, REMOTE_CONTROL );
		break;
		
		default:
		    Nop();
		    Nop();
		break;
	}
}
