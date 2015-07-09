#ifndef VACUUM_TASKS_H
#define VACUUM_TASKS_H

// Task declarations
portTASK_FUNCTION( vTestLED, pvParameters );
portTASK_FUNCTION( vUARTIOControl, pvParameters );
portTASK_FUNCTION( vMotorMonitor, pvParameters );
portTASK_FUNCTION( vMotorEscape, pvParameters );

void vTestLEDInitialise( void );

// Task priorities
//#define MOTOR_MONITOR_PRIORITY		( tskIDLE_PRIORITY + ( unsigned portBASE_TYPE ) 1 )
//#define UART_PRIORITY				( tskIDLE_PRIORITY + ( unsigned portBASE_TYPE ) 2 )
//#define LED_TASK_PRIORITY			( tskIDLE_PRIORITY + ( unsigned portBASE_TYPE ) 3 )
//#define ESCAPE_TASK_PRIORITY		( tskIDLE_PRIORITY + ( unsigned portBASE_TYPE ) 3 )

// Task priorities
enum TASK_PRIORITIES
{
    MOTOR_MONITOR_PRIORITY		= tskIDLE_PRIORITY + 1,
	LED_TASK_PRIORITY			= tskIDLE_PRIORITY + 1,
	MOTOR_ARBITRATION_PRIORITY	= tskIDLE_PRIORITY + 1,
	UART_PRIORITY				= tskIDLE_PRIORITY + 2,
	//SONAR_PRIORITY				= tskIDLE_PRIORITY + 1,
	//SCRIPT_PRIORITY		        = tskIDLE_PRIORITY + 2,
	//MOTOR_DRIVE_PRIORITY		= tskIDLE_PRIORITY + 2,
	//SPEAKER_TASK_PRIORITY		= tskIDLE_PRIORITY + 3,
    ESCAPE_TASK_PRIORITY		= tskIDLE_PRIORITY + 3
};
	
// Task stack size
#define comSTACK_SIZE				configMINIMAL_STACK_SIZE
#define ledSTACK_SIZE				128
//#define comSTACK_SIZE				255
#define motorSTACK_SIZE				192
#define arbiterSTACK_SIZE			192
#define sonarSTACK_SIZE				192
#define speakerSTACKSIZE			192
#define scriptSTACKSIZE			    192

#endif
