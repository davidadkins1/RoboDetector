/*********************************************************************
 *
 *      Robot Research Platform
 *
 *********************************************************************
 * FileName:       	UARTIO.c
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
#include <pwm.h>
#include <adc.h>
#include "io_cfg.h"

/* Scheduler header files. */
#include "FreeRTOS.h"
#include "task.h"
#include "serial.h"
#include "queue.h"
#include "RoboDetector_tasks.h"
#include "motor.h"
#include "behaviors.h"

/** Global Variables ********************************************************/
#pragma udata
xTaskHandle uart_rx_task;
//xTaskHandle uart_tx_task;

extern char *BANNER[ ];

/** Local Variables ********************************************************/
#pragma udata

//#pragma idata
//unsigned int high_ram_mark = 0;
//unsigned int low_ram_mark = 0;

#pragma udata
// Handle to the com port used by both tasks.
//static xComPortHandle xPort = NULL;

/** Public Prototypes ***************************************/
/*
 * Prototypes for ISR's.  The PIC architecture means that these functions
 * have to be called from port.c.  The prototypes are not however included
 * in the header as the header is common to all ports.
 */
void vSerialTxISR( void );
void vSerialRxISR( void );
void vUARTIOputsBT( char *data);
void vUARTIOWriteBT( char *data, unsigned int byteCount );
void vUARTIOInitUSART( void );


/* Bit/register definitions. */
#define serINPUT				( 1 )
#define serOUTPUT				( 0 )
#define serTX_ENABLE			( ( unsigned portSHORT ) 1 )
#define serRX_ENABLE			( ( unsigned portSHORT ) 1 )
#define serHIGH_SPEED			( ( unsigned portSHORT ) 1 )
#define serCONTINUOUS_RX		( ( unsigned portSHORT ) 1 )
#define serCLEAR_OVERRUN		( ( unsigned portSHORT ) 0 )
#define serINTERRUPT_ENABLED 	( ( unsigned portSHORT ) 1 )
#define serINTERRUPT_DISABLED 	( ( unsigned portSHORT ) 0 )

/* All ISR's use the PIC18 low priority interrupt. */
#define							serLOW_PRIORITY ( 0 )

/*-----------------------------------------------------------*/
// Queues to interface between comms API and interrupt routines.
#pragma udata
static xQueueHandle xRxedChars; 
static xQueueHandle xCharsForTx;
//static xQueueHandle bt_CharsForTx;

/** External  P R O T O T Y P E S ***************************************/
extern void vPDACmd( void );
extern void iRobotCmd( unsigned portCHAR );

/** P R I V A T E  P R O T O T Y P E S ***************************************/

#pragma code

/*-----------------------------------------------------------*/

void vStartUARTIOTask( void )
{
	// Initialise the com port then spawn the Rx and Tx tasks.
	vUARTIOInitUSART( );

	// The Tx task is spawned with a lower priority than the Rx task.
	xTaskCreate( vUARTRxControl, ( const signed portCHAR * const ) "R", comSTACK_SIZE, NULL, UART_RX_PRIORITY, &uart_rx_task );
	//xTaskCreate( vUARTTxControl, ( const signed portCHAR * const ) "T", comSTACK_SIZE, NULL, UART_TX_PRIORITY, &uart_tx_task );
}

/*-----------------------------------------------------------*/

void vUARTIOInitUSART( void )
{
	/* Create the queues used by the ISR's to interface to tasks. */
	xRxedChars = xQueueCreate( comBUFFER_LEN, ( unsigned portBASE_TYPE ) sizeof( portCHAR ) );
	xCharsForTx = xQueueCreate( comBUFFER_LEN, ( unsigned portBASE_TYPE ) sizeof( portCHAR ) );
	//bt_CharsForTx = xQueueCreate( comBUFFER_LEN, ( unsigned portBASE_TYPE ) sizeof( portCHAR ) );

	portENTER_CRITICAL();
	{
		// Setup the IO pins to enable the USART IO.
		TRIS_CTS = INPUT_PIN;
		TRIS_USART_TX = INPUT_PIN;
		TRIS_USART_RX = INPUT_PIN;

		// Initialize the USART
		OpenUSART(	USART_TX_INT_ON &
					USART_RX_INT_ON &
					USART_ASYNCH_MODE &
					USART_EIGHT_BIT &
					USART_CONT_RX &
					USART_BRGH_HIGH,
					((configCPU_CLOCK_HZ / mainBAUD_RATE) / 16 ) - 1 );

		// Reset the Bluetooth module
		TRIS_BT_RESET = OUTPUT_PIN;
		BT_RESET = RESET_BT;			// Reset starts here 
	}
	portEXIT_CRITICAL();
}

portBASE_TYPE xUARTIOGetChar( xComPortHandle pxPort, unsigned portCHAR *pcRxedChar, portTickType xBlockTime )
{
	/* Get the next character from the buffer.  Return false if no characters
	are available, or arrive before xBlockTime expires. */
	if( xQueueReceive( xRxedChars, (void *)pcRxedChar, xBlockTime ) )
	{
		return pdTRUE;
	}
	else
	{
		return pdFALSE;
	}
}
/*-----------------------------------------------------------*/

portBASE_TYPE xUARTIOPutChar( unsigned portCHAR cOutChar )
{
	/* Return false if after the block time there is no room on the Tx queue. */
	if( xQueueSend( xCharsForTx, ( const void * ) &cOutChar, portMAX_DELAY ) != pdPASS )
	{
		return pdFAIL;
	}

	/* Turn interrupt on - ensure the compiler only generates a single 
	instruction for this. */
	PIE1bits.TXIE = serINTERRUPT_ENABLED;

	return pdPASS;
}

/*-----------------------------------------------------------*/

/**********************************************************************
*    Function Name:  vUARTIOputsBT                                    *
*    Return Value:   void                                             *
*    Parameters:     data: pointer to string of data                  *
*    Description:    This routine transmits a string of characters    *
*                    to the USART including the null.                 *
**********************************************************************/
void vUARTIOputsBT( char *data)
{	
	while( *data )
		vUARTIOWriteBT( data++, 1 );		// Transmit a byte
}

/********************************************************************
*    Function Name:  vUARTIOWriteBT                                	*
*    Return Value:   none                                           *
*    Parameters:     data: data to transmit                         *
*    Description:    This routine transmits a byte out the USART.   *
********************************************************************/
void vUARTIOWriteBT( char *data, unsigned int byteCount )
{
	unsigned int i;
	
	for( i = 0; i < byteCount; i++ )
	{
    	
		//while( CTS )
		//{
		//	vTaskDelay( (portTickType)1 );			// Wait for CTS
		//}
        
    	//xQueueSend( bt_CharsForTx, (void *)data++, portMAX_DELAY );
    	xUARTIOPutChar(data[i]);
	}
}

/********************************************************************
*    Function Name:  vUARTIOControl                                     	*
*    Return Value:   none                                           *
*    Parameters:     data: data to transmit                         *
*    Description:    Host command processor done via Bluetooth		*
********************************************************************/
portTASK_FUNCTION( vUARTRxControl, pvParameters )
{
	//
	// The command sent from the Host computer
	//
	unsigned portCHAR cByteRxed;
	/* Just to stop compiler warnings. */
	//( void ) pvParameters;
    /*
	do
    {
        cByteRxed = 0;
		BT_RESET = RESET_BT;			    // Reset starts here 
    	vTaskDelay( (portTickType)20 );		// Reset Bluetooth module for 6 ms
    	BT_RESET = START_BT;
    	
	    if( xUARTIOGetChar(NULL, &cByteRxed, comRX_BLOCK_TIME))
	    {
            //if( cByteRxed == (unsigned portCHAR)0xAA )
            //{
            //    break;
            //}
        }   
    } while ( cByteRxed != (unsigned portCHAR)0xAA );
	*/

	for( ;; )
	{
		// Block on the queue that contains received bytes until a byte is
		// available.
		if( xUARTIOGetChar( NULL, &cByteRxed, comRX_BLOCK_TIME ) )
		{		
			//if( cmdState == PREFIX_1 )				// Wait for characters received
			{
				if( cByteRxed == (unsigned portCHAR)0x01 )
					vPDACmd( );
				else if( cByteRxed > (unsigned portCHAR)127 )
					iRobotCmd( cByteRxed );
			}
		}
		/*
		vTaskDelay( (portTickType)200 );
		vUARTIOputsBT( BANNER );
		*/
	}//for( ;; )
}

/*
portTASK_FUNCTION( vUARTTxControl, pvParameters )
{
	//
	// The command sent from the Host computer
	//
	unsigned portCHAR byte_received;
	// Just to stop compiler warnings.
	//( void ) pvParameters;

	for( ;; )
	{
		// Block on the queue that contains received bytes until a byte is
		// available.
		if( xQueueReceive( bt_CharsForTx, (void *)&byte_received, portMAX_DELAY ) )
		{		
		    // Transmit a byte
		    xUARTIOPutChar(byte_received);
		}
	}//for( ;; )
}
*/

#pragma interruptlow vSerialRxISR
void vSerialRxISR( void )
{
portCHAR cChar;
portCHAR rx_status;
portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

	/* Get the character and post it on the queue of Rxed characters.
	If the post causes a task to wake force a context switch as the woken task
	may have a higher priority than the task we have interrupted. */
	rx_status = RCSTA;
	cChar = RCREG;

	// Clear any overrun errors.
	if(0 != (rx_status & 6))
	{
		RCSTAbits.CREN = serCLEAR_OVERRUN;
		Nop();
		Nop();
		RCSTAbits.CREN = serCONTINUOUS_RX;	
		Nop();
		Nop();
	}

	xQueueSendFromISR( xRxedChars, ( const void * ) &cChar, &xHigherPriorityTaskWoken );

	if( xHigherPriorityTaskWoken )
	{
		taskYIELD();
	}
}
/*-----------------------------------------------------------*/

//#pragma interruptlow vSerialTxISR save=PRODH, PRODL, TABLAT, section(".tmpdata")
#pragma interruptlow vSerialTxISR
void vSerialTxISR( void )
{
portCHAR cChar, cTaskWoken;

	if( xQueueReceiveFromISR( xCharsForTx, (void *)&cChar, &cTaskWoken ) == pdTRUE )
	{
		/* Send the next character queued for Tx. */
		TXREG = cChar;
	}
	else
	{
		/* Queue empty, nothing to send. */
		PIE1bits.TXIE = serINTERRUPT_DISABLED;
	}
}

