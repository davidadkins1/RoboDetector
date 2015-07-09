/*********************************************************************
 *
 *      Vacuum Robot to be used with resident Bootloader
 *
 *********************************************************************
 * FileName:        Motor.c
 * Processor:       PIC18
 * Compiler:        C18 3.01+
 *
 * Author               Date        Comment
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * David Adkins       03/15/06    Original.
 ********************************************************************/

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "serial.h"
#include "comtest.h"
#include "RoboDetector_tasks.h"

/* Vacuum include files. */
#include "motor.h"

/* Constants required for the communications. */
#define mainCOMMS_QUEUE_LENGTH			( ( unsigned portBASE_TYPE ) 5 )
/*-----------------------------------------------------------*/
#pragma udata
xTaskHandle led_task;
xTaskHandle escape_task;
xTaskHandle script_task;
xTaskHandle motor_task;

/*-----------------------------------------------------------*/

#pragma code
/* Creates the tasks, then starts the scheduler. */
void main( void )
{
	// Initialise the block memory allocator.
	vPortInitialiseBlocks();

	/* Initialise the required hardware. */
	//#ifndef __DEBUG
	vTestLEDInitialise();
	init_compass();
	vScriptInitialise();
    //#endif
    
	// Initialise the motor hardware.
	vMotorInitialize( );

	// Start the standard comtest tasks as defined in demo/common/minimal.
	vStartUARTIOTask( );

	/* Start the check task defined in this file. */
	//#ifndef __DEBUG
	xTaskCreate( vTestLED, ( const portCHAR * const ) "C", ledSTACK_SIZE, NULL, LED_TASK_PRIORITY, &led_task );
    //#endif
    
	// Start motor task
	xTaskCreate( vMotorMonitor, ( const portCHAR * const ) "M", motorSTACK_SIZE, NULL, MOTOR_MONITOR_PRIORITY, &motor_task );

	// Start the Escape task
	//xTaskCreate( vMotorEscape, ( const portCHAR * const ) "E", ESCAPE_STACK_SIZE, NULL, ESCAPE_TASK_PRIORITY, &escape_task );
	//xTaskCreate( vScriptTask, ( const portCHAR * const ) "S", scriptSTACKSIZE, NULL, SCRIPT_PRIORITY, &script_task );

	// Start the scheduler.  This will never return.
	vTaskStartScheduler();
}

/*-----------------------------------------------------------*/

