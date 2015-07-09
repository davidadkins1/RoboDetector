#ifndef VACUUM_TASKS_H
#define VACUUM_TASKS_H

enum TASK_LIST
{
    LED_TASK,
    MOTOR_TASK,
    ESCAPE_TASK,
    UART_RX_TASK,
    UART_TX_TASK
};

// Task declarations
portTASK_FUNCTION( vTestLED, pvParameters );
portTASK_FUNCTION( vMotorMonitor, pvParameters );
portTASK_FUNCTION( vMotorEscape, pvParameters );
portTASK_FUNCTION( vUARTRxControl, pvParameters );
portTASK_FUNCTION( vUARTTxControl, pvParameters );
portTASK_FUNCTION( vScriptTask, pvParameters );
portTASK_FUNCTION( compass_task, pvParameters );
void vScriptInitialise( void );
void vTestLEDInitialise( void );
void init_compass(void);

// Task priorities
// Higher numbers are higher priority
enum TASK_PRIORITIES
{
    COMPASS_TASK_PRIORITY       = tskIDLE_PRIORITY + 1,
	SCRIPT_PRIORITY		        = tskIDLE_PRIORITY + 1,
	LED_TASK_PRIORITY			= tskIDLE_PRIORITY + 1,
    ESCAPE_TASK_PRIORITY		= tskIDLE_PRIORITY + 1,
    MOTOR_MONITOR_PRIORITY		= tskIDLE_PRIORITY + 1,
    UART_TX_PRIORITY		    = tskIDLE_PRIORITY + 2,
    UART_RX_PRIORITY		    = tskIDLE_PRIORITY + 3,
	//SONAR_PRIORITY				= tskIDLE_PRIORITY + 1,
	//SPEAKER_TASK_PRIORITY		= tskIDLE_PRIORITY + 3,
};
	
// Task stack sizes
#define comSTACK_SIZE				0x120
#define ledSTACK_SIZE				configMINIMAL_STACK_SIZE
#define motorSTACK_SIZE				0x120
#define arbiterSTACK_SIZE			configMINIMAL_STACK_SIZE
#define scriptSTACKSIZE			    configMINIMAL_STACK_SIZE
#define ESCAPE_STACK_SIZE           configMINIMAL_STACK_SIZE
#define COMPASS_STACK_SIZE          0x120
//#define sonarSTACK_SIZE				192
//#define speakerSTACKSIZE			192

//extern xTaskHandle task_handles[6];

#endif
