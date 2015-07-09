/*
	FreeRTOS.org V4.1.2 - Copyright (C) 2003-2006 Richard Barry.

	This file is part of the FreeRTOS.org distribution.

	FreeRTOS.org is free software; you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation; either version 2 of the License, or
	(at your option) any later version.

	FreeRTOS.org is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with FreeRTOS.org; if not, write to the Free Software
	Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

	A special exception to the GPL can be applied should you wish to distribute
	a combined work that includes FreeRTOS.org, without being obliged to provide
	the source code for any proprietary components.  See the licensing section 
	of http://www.FreeRTOS.org for full details of how and when the exception
	can be applied.

	***************************************************************************
	See http://www.FreeRTOS.org for documentation, latest information, license 
	and contact details.  Please ensure to read the configuration and relevant 
	port sections of the online documentation.
	***************************************************************************
*/

/* 
Changes from V2.0.0

	+ Use scheduler suspends in place of critical sections.
*/

#include "FreeRTOS.h"
#include "task.h"
#include "io_cfg.h"

/* The period between executions of the check task before and after an error
has been discovered.  If an error has been discovered the check task runs
more frequently - increasing the LED flash rate. */
#define NO_ERROR_PERIOD		configTICK_RATE_HZ

/* The period for which mainRESET_LED remain on every reset. */
#define RESET_LED_PERIOD	configTICK_RATE_HZ / 10

/* The LED that is flashed by the check task at a rate that indicates the 
error status. */
#define mainCHECK_TASK_LED				( ( unsigned portBASE_TYPE ) 1 )


/*-----------------------------------------------------------*/
#pragma udata
#pragma code

portTASK_FUNCTION( vTestLED, pvParameters )
{
	portCHAR i;
	portTickType xDelayTime = NO_ERROR_PERIOD;

	/* Toggle the LED so we can see when a reset occurs. */
	for( i = 0; i < 25; i++ )
	{
	TEST_LED = ~TEST_LED;
		vTaskDelay( RESET_LED_PERIOD );
	}

	/* Cycle for ever, delaying then checking all the other tasks are still
	operating without error. */
	for( ;; )
	{
		/* Wait until it is time to check the other tasks. */
		vTaskDelay( xDelayTime );

		/* Flash the LED for visual feedback. */
		TEST_LED = ~TEST_LED;
	}
}

void vTestLEDInitialise( void )
{
	/* Set the port B3 to output. */
	TRIS_TEST_LED = OUTPUT_PIN;

	/* Start with LED off. */
	TEST_LED = LED_OFF;
}
/*-----------------------------------------------------------*/


