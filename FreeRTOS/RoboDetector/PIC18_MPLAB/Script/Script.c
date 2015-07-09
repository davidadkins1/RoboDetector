/*********************************************************************
 *
 *      Robot Research Platform
 *
 *********************************************************************
 * FileName:        Script.c
 * Processor:       PIC18
 * Compiler:        C18 3.20+
 *
 * Author               Date        Comment
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * David Adkins       05/20/08    Original.
 ********************************************************************/

/** I N C L U D E S **********************************************************/
#include <timers.h>
#include <pwm.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "io_cfg.h"
#include "oi.h"
#include "motor.h"
#include "behaviors.h"

/** Global Variables ********************************************************/
#pragma udata
/*-----------------------------------------------------------*/
// Queue to interface between the command module and the song task.
xQueueHandle xScriptQueue; 
extern unsigned char script[48];
extern SENSORS sensorPacket;

/** Local Variables ********************************************************/
#pragma udata
static unsigned char scriptSize;
static unsigned char script_index;
static WORD_VAL leRight;
#define radius_turn leRight
static WORD_VAL leLeft;
static DWORD_VAL leDistance;
static WORD_VAL script_heading;
    


#pragma romdata

#pragma code

void vScriptInitialise( void )
{
	const unsigned portBASE_TYPE uxQueueSize = 1;
	
	// Create the queue used by the iRobot Command processor
	xScriptQueue = xQueueCreate( uxQueueSize, ( unsigned portBASE_TYPE ) sizeof( portCHAR ) );

    // load a test script	
	script[0] = 7;
	script[1] = TurnToHeading;
	script[2] = 0x01;       // speed = 384 mm/s;
	script[3] = 0x80;
	script[4] = 0x00;       // Spin in place
	script[5] = 0x00;
	script[6] = 0x01;       // Heading South, 270
	script[7] = 0x0E;
	script[8] = CmdDriveDistance;
	script[9] = 0x01;       // speed = 384 mm/s
	script[10] = 0x80;       
	script[11] = 0x80;      // Direction = forward
	script[12] = 0x00;
	script[13] = 0x00;      // Distance = 1024 mm
	script[14] = 0x00;
	script[15] = 0x04;
	script[16] = 0x00;
	script[17] = TurnToHeading;
	script[18] = 0x01;       // speed = 384 mm/s;
	script[19] = 0x80;
	script[20] = 0x00;       // Spin in place
	script[21] = 0x00;
	script[22] = 0x00;        // Heading North, 0
	script[23] = 0x00;
	script[24] = CmdDriveDistance;
	script[25] = 0x01;       // speed = 384 mm/s
	script[26] = 0x80;       
	script[27] = 0x80;      // Direction = forward
	script[28] = 0x00;
	script[29] = 0x00;      // Distance = 2048 mm
	script[30] = 0x00;
	script[31] = 0x04;
	script[32] = 0x00;
	script[33] = CmdDrive;  // Stop
	script[34] = 0x00;
	script[35] = 0x00;
	script[36] = 0x00;
	script[37] = 0x00;
	script[38] = 0;
	script[39] = 0;
	script[40] = 0;
	script[41] = 0;
	script[42] = 0;
	script[43] = 0;
	script[44] = 0;
	script[45] = 0;
	script[46] = 0;
	script[47] = 0;
}

portTASK_FUNCTION( vScriptTask, pvParameters )
{
	for( ; ; )
	{
        // Block waiting for a song to play.
        if(pdTRUE  == xQueueReceive( xScriptQueue, (void *)&scriptSize, portMAX_DELAY ))
        {            
            //while(pdTRUE)
	        for(script_index = 1; script_index < (script[0] + 1); script_index++)
	        {
            	switch(script[script_index])
            	{
            		case CmdStart:
            			sensorPacket.sensor.ucOIMode = OIPassive;
            		break;
            		
            		case CmdBaud:
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
            		    script_index++;
            		    leLeft.byte.HB = script[script_index++];
            			sensorPacket.sensor.iRequestedLeftVelocity.byte.HB = leLeft.byte.HB;
            			sensorPacket.sensor.iRequestedRightVelocity.byte.HB = leLeft.byte.HB;
            			
            		    leLeft.byte.LB = script[script_index++];
            			sensorPacket.sensor.iRequestedLeftVelocity.byte.LB = leLeft.byte.LB;
            			sensorPacket.sensor.iRequestedRightVelocity.byte.LB = leLeft.byte.LB;
            			
            			leRight.byte.HB = script[script_index++];
            			sensorPacket.sensor.iRequestedRadius.byte.HB = leRight.byte.HB;
            			
            			leRight.byte.LB = script[script_index];
            			sensorPacket.sensor.iRequestedRadius.byte.LB = leRight.byte.LB;
            			
            			vMotorDriveRadius( leRight.Val, leRight.Val, 0, REMOTE_CONTROL );
            		break;
            		
            		case CmdMotors:
            		break;
            		
            		case CmdLeds:
            		break;
            		
            		case CmdSong:
            		break;
            		
            		case CmdPlay:
            			//xUARTIOGetChar( NULL, &sensorPacket.sensor.ucSongNumber, comRX_BLOCK_TIME );
            			//xQueueSendToBack( xSongQueue, ( void * ) &sensorPacket.sensor.ucSongNumber, 0 );
            		break;
            		
            		case CmdSensors:
            		/*
            			i = xUARTIOGetChar( NULL, &cmdByte, comRX_BLOCK_TIME );
            			
            			if( cmdByte == (unsigned portCHAR)0 )
            			{
            				vUARTIOWriteBT( (char *)sensorPacket.byte, Sen0Size );
            			}
            			else if( cmdByte == (unsigned portCHAR)1 )
            			{
            				vUARTIOWriteBT( (char *)sensorPacket.byte, Sen1Size );
            			}
            			else if( cmdByte == (unsigned portCHAR)2 )
            			{
            				vUARTIOWriteBT( (char *)&sensorPacket.byte[SenIRChar], Sen2Size );
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
            			}
            		*/
            		break;
            		
            		case CmdDock:
            		break;
            		
            		case CmdPWMMotors:
            		break;
            		
            		case CmdDriveWheels:
               		    script_index++;
            			leRight.byte.HB = script[script_index++];
            			sensorPacket.sensor.iRequestedRightVelocity.byte.HB = leRight.byte.HB;
            			
            			leRight.byte.LB = script[script_index++];
            			sensorPacket.sensor.iRequestedRightVelocity.byte.LB = leRight.byte.LB;
            			
            		    leLeft.byte.HB = script[script_index++];
            			sensorPacket.sensor.iRequestedLeftVelocity.byte.HB = leLeft.byte.HB;
            			
            		    leLeft.byte.LB = script[script_index];
            			sensorPacket.sensor.iRequestedLeftVelocity.byte.LB = leLeft.byte.LB;
            			
            			vMotorDriveRequest( leLeft.Val, leRight.Val, 0, REMOTE_CONTROL );
            		break;
            		
            		case CmdOutputs:
            		break;
            		
            		case CmdSensorList:
            		break;
            		
            		case CmdIRChar:
            		break;

            		case CmdDriveDistance:
            		    script_index++;
            		    leLeft.byte.HB = script[script_index++];
                		sensorPacket.sensor.iRequestedLeftVelocity.byte.HB = leLeft.byte.HB;
                		sensorPacket.sensor.iRequestedRightVelocity.byte.HB = leLeft.byte.HB;
                		
            		    leLeft.byte.LB = script[script_index++];
                		sensorPacket.sensor.iRequestedLeftVelocity.byte.LB = leLeft.byte.LB;
                		sensorPacket.sensor.iRequestedRightVelocity.byte.LB = leLeft.byte.LB;
                		
            			radius_turn.byte.HB = script[script_index++];
                		sensorPacket.sensor.iRequestedRadius.byte.HB = radius_turn.byte.HB;
                		
            			radius_turn.byte.LB = script[script_index++];
                		sensorPacket.sensor.iRequestedRadius.byte.LB = radius_turn.byte.LB;

            			leDistance.byte.MB = script[script_index++];
            			leDistance.byte.UB = script[script_index++];
            			leDistance.byte.HB = script[script_index++];
            			leDistance.byte.LB = script[script_index];
                
                		vMotorDriveRadius( radius_turn.Val, leLeft.Val, leDistance.Val, REMOTE_CONTROL );
                		
                		while(odometer == 0ul)
                		{
                    		vTaskDelay(10);
                		}

                		while(odometer != 0ul)
                		{
                    		vTaskDelay(100);
                		}
            	    break;
            	    
            		case TurnToHeading:
            		    script_index++;
            		    leLeft.byte.HB = script[script_index++];
                		sensorPacket.sensor.iRequestedLeftVelocity.byte.HB = leLeft.byte.HB;
                		sensorPacket.sensor.iRequestedRightVelocity.byte.HB = leLeft.byte.HB;
                		
            		    leLeft.byte.LB = script[script_index++];
                		sensorPacket.sensor.iRequestedLeftVelocity.byte.LB = leLeft.byte.LB;
                		sensorPacket.sensor.iRequestedRightVelocity.byte.LB = leLeft.byte.LB;
            			
            			radius_turn.byte.HB = script[script_index++];
                		sensorPacket.sensor.iRequestedRadius.byte.HB = radius_turn.byte.HB;
                		
            			radius_turn.byte.LB = script[script_index++];
                		sensorPacket.sensor.iRequestedRadius.byte.LB = radius_turn.byte.LB;

            			script_heading.byte.HB = script[script_index++];
            			script_heading.byte.LB = script[script_index];
            			
            			while(0 != (script_heading.Val - current_heading()))
            			{
                			//vMotorTurnHeading( leLeft.Val, script_heading.Val, REMOTE_CONTROL );
    			            vMotorTurnHeading( radius_turn.Val, leLeft.Val, script_heading.Val, REMOTE_CONTROL );
    			            
    			            // Wait for motor start
                       		while(odometer == 0ul)
                    		{
                        		vTaskDelay(10);
                    		}
                    		
                            // Wait for motor stop
                    		while(odometer != 0ul)
                    		{
                        		vTaskDelay(100);
                    		}
                    		
                    		// Wait for compass update
                    		vTaskDelay( 300 );
                    		
                    		// Heading can be +/-1
                    		if(script_heading.Val > current_heading())
                    		{
                        		if(1 == (script_heading.Val - current_heading()))
                        		{
                            		// Force loop exit if difference is 1
                            		script_heading.Val = current_heading();
                        		}
                    		}
                    		else
                    		{
                        		if(1 == (current_heading() - script_heading.Val))
                        		{
                            		// Force loop exit if difference is 1
                            		script_heading.Val = current_heading();
                        		}
                    		}
            			}            
            		break;

            	    default:
            	    break;
               }   		
	        }
        }
	}
}
