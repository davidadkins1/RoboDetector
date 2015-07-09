/*********************************************************************
 *
 *                Microchip USB C18 Firmware Version 1.0
 *
 *********************************************************************
 * FileName:        io_cfg.h
 * Processor:       PIC18
 * Compiler:        C18 2.30.01+
 *
 * Author               Date        Comment
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * David Adkins       01/08/07     Original.
 ********************************************************************/

/******************************************************************************
 * -io_cfg.h-
 * I/O Configuration File
 * The purpose of this file is to provide a mapping mechanism between
 * pin functions and pin assignments. This provides a layer of abstraction
 * for the firmware code and eases the migration process from one target
 * board design to another.
 *
 *****************************************************************************/

#ifndef IO_CFG_H
#define IO_CFG_H

#include <p18cxxx.h>
#include <pwm.h>

//Port A I/O configuration
	//Bit:  7 - CTS in from Zeevo Bluetooth module.
	//      6
	//	    5 - Left motor EMF AtoD input A.
	//		4 - 
	//		3 - Right motor current AtoD input
	//		2 - Left motor current AtoD input
	//		1 - Right motor EMF AtoD input B.
	//		0 - Right motor EMF AtoD input A.

//Port B  I/O configuration
	//Bit:	7 - PGD - ICSP Data and TEST pin
	//		6 - PGC - ICSP Clock and LED
	//		5 - Right motor reverse enable to L298.
	//		4 - 
	// 		3 - RESET out to Zeevo Bluetooth module.
	//		2 - Left motor EMF AtoD input B.
	//		1 - Right motor foward enable to L298.
	//		0 - Left motor foward enable to L298.

//Port C  I/O configuration
	//Bit:	7 - RX in from Zeevo Bluetooth module
	//		6 - TX out to Zeevo Bluetooth module
	//		5 
	//		4 - Compass SDA
	// 		3 - Compass SCL
	//		2 - Left motor speed (PWM) out to L298 enable.
	//		1 - Right motor speed (PWM) out to L298 enable.
	//		0 - Left motor reverse enable to L298.

/** I N C L U D E S *************************************************/

/** C O N F I G U R A T I O N ********************************************************/
#if defined( CFG_ONCE )
#pragma config OSC = INTIO67
#pragma config FCMEN = OFF

#if defined (__DEBUG)
#pragma config PWRT = OFF
#else
#pragma config PWRT = ON
#endif

#pragma config BOREN = OFF
#pragma config BORV = 0
#pragma config WDT = OFF
#pragma config WDTPS = 128
#pragma config MCLRE = ON
#pragma config LPT1OSC = OFF
#pragma config PBADEN = OFF
#pragma config CCP2MX = PORTC
#pragma config STVREN = OFF
#pragma config LVP = OFF
#pragma config XINST = OFF
#pragma config IESO = OFF


#pragma config CP0 = OFF
#pragma config CP1 = OFF
#pragma config CP2 = OFF
#pragma config CP3 = OFF
#pragma config CPB = OFF
#pragma config CPD = OFF
#pragma config WRT0 = OFF
#pragma config WRT1 = OFF
#pragma config WRT2 = OFF
#pragma config WRT3 = OFF
#pragma config WRTB = OFF
#pragma config WRTC = OFF
#pragma config WRTD = OFF
#pragma config EBTR0 = OFF
#pragma config EBTR1 = OFF
#pragma config EBTR2 = OFF
#pragma config EBTR3 = OFF
#pragma config EBTRB = OFF
#endif

/** T R I S *********************************************************/
#define INPUT_PIN           1
#define OUTPUT_PIN          0

/********************************************************************/
#ifdef DETECTOR_BOARD
/********************************************************************/
#define TRIS_RMA_EMF		TRISAbits.TRISA0    // Analog Input 0
#define TRIS_RMB_EMF		TRISAbits.TRISA1    // Analog Input 1
#define TRIS_RM_CURRENT		TRISAbits.TRISA2    // Analog Input 2
#define TRIS_LM_CURRENT		TRISAbits.TRISA3    // Analog Input 3
#define TRIS_CTS	        TRISAbits.TRISA7    // Input
#define TRIS_LMA_EMF		TRISAbits.TRISA5    // Analog Input 4


#define TRIS_LMF		TRISBbits.TRISB0    // Output
#define TRIS_RMF		TRISBbits.TRISB1    // Output
#define TRIS_LMB_EMF	TRISBbits.TRISB2    // Analog Input 8
#define TRIS_BT_RESET	TRISBbits.TRISB3	// Output
#define TRIS_TEST_LED	TRISBbits.TRISB4    // Output
#define TRIS_RMR		TRISBbits.TRISB5    // Output
//#define TRIS_TEST_LED	TRISBbits.TRISB6    // Output
//#define TRIS_TEST_PIN	TRISBbits.TRISB7    // Output

#define TRIS_LMR		TRISCbits.TRISC0    // Output
#define TRIS_RMS		TRISCbits.TRISC1    // Output
#define TRIS_LMS		TRISCbits.TRISC2    // Output
#define TRIS_SCL		TRISCbits.TRISC3    // Output
#define TRIS_SDA		TRISCbits.TRISC4    // Output
#define TRIS_USART_TX	TRISCbits.TRISC6    // Output
#define TRIS_USART_RX	TRISCbits.TRISC7    // Input
#else 
/********************************************************************/
#ifdef RRP_BOARD
/********************************************************************/
#define TRIS_RMA_EMF		TRISAbits.TRISA0    // Analog Input 0
#define TRIS_RMB_EMF		TRISAbits.TRISA1    // Analog Input 1
#define TRIS_RM_CURRENT		TRISAbits.TRISA2    // Analog Input 2
#define TRIS_LM_CURRENT		TRISAbits.TRISA3    // Analog Input 3
#define TRIS_LMA_EMF		TRISAbits.TRISA5    // Analog Input 4
#define TRIS_LMR		    TRISAbits.TRISA6    // Output Need to move

#define TRIS_TEST_LED	TRISBbits.TRISB0    // Output
#define TRIS_RMF		TRISBbits.TRISB1    // Output
#define TRIS_LMB_EMF	TRISBbits.TRISB2    // Analog Input 8
#define TRIS_LMF		TRISBbits.TRISB4    // Output
#define TRIS_RMR		TRISBbits.TRISB5    // Output

//#define TRIS_LMR		TRISCbits.TRISC0    // Output
#define TRIS_BT_RESET	TRISCbits.TRISC0	// Output
#define TRIS_RMS		TRISCbits.TRISC1    // Output
#define TRIS_LMS		TRISCbits.TRISC2    // Output
#define TRIS_SCL		TRISCbits.TRISC3    // Output
#define TRIS_SDA		TRISCbits.TRISC4    // Output
//#define TRIS_BT_RESET	TRISCbits.TRISC4	// Output
#define TRIS_CTS		TRISCbits.TRISC5    // Input
#define TRIS_USART_TX	TRISCbits.TRISC6    // Output
#define TRIS_USART_RX	TRISCbits.TRISC7    // Input
#endif
#endif

//** L E D ***********************************************************/
/********************************************************************/
#ifdef DETECTOR_BOARD
/********************************************************************/
#define TEST_LED		LATBbits.LATB4
#else 
/********************************************************************/
#ifdef RRP_BOARD
/********************************************************************/
#define TEST_LED		LATBbits.LATB0
#endif
#endif

#define LED_OFF	0
#define LED_ON	1

//#define TEST_PIN	LATBbits.LATB7

//** M O T O R S ***********************************************************/
#define RM_STOP 0
#define RM_START 1
#define RM_FORWARD_ENABLE	1
#define RM_FORWARD_DISABLE	0
#define RM_REVERSE_ENABLE	0
#define RM_REVERSE_DISABLE	1

#define LM_STOP 0
#define LM_START 1
#define LM_FORWARD_ENABLE	1
#define LM_FORWARD_DISABLE	0
#define LM_REVERSE_ENABLE	0
#define LM_REVERSE_DISABLE	1

/********************************************************************/
#ifdef DETECTOR_BOARD
/********************************************************************/
#define	LMF		LATBbits.LATB0			// Left motor foward enable to L298.
										// 0 = Left motor forward enabled, 1 = Left motor forward disabled.
#define	RMF		LATBbits.LATB1			// Right motor foward enable to L298.
										// 0 = Right motor forward enabled, 1 = Right motor forward disabled.
#define	RMR		LATBbits.LATB5			// Right motor reverse enable to L298.
										// 0 = Right motor reverse enabled, 1 = Right motor reverse disabled.
#define	LMR		LATCbits.LATC0			// Left motor reverse enable to L298.
										// 0 = Left motor reverse enabled, 1 = Left motor reverse disabled.
#define	RMS		LATCbits.LATC1			// Right motor speed to L298 enable.
										// 0 = Stop, 1 = Full speed, or PWM
#define	LMS		LATCbits.LATC2			// Left motor speed to L298 enable.
										// 0 = stop, 1 = Full speed, or 20 khz PWM
#else 
/********************************************************************/
#ifdef RRP_BOARD
/********************************************************************/
#define	LMF		LATBbits.LATB4			// Left motor foward enable to L298.
										// 0 = Left motor forward enabled, 1 = Left motor forward disabled.
#define	RMF		LATBbits.LATB1			// Right motor foward enable to L298.
										// 0 = Right motor forward enabled, 1 = Right motor forward disabled.
#define	RMR		LATBbits.LATB5			// Right motor reverse enable to L298.
										// 0 = Right motor reverse enabled, 1 = Right motor reverse disabled.
#define	LMR		LATAbits.LATA6			// Left motor reverse enable to L298.
										// 0 = Left motor reverse enabled, 1 = Left motor reverse disabled.
#define	RMS		LATCbits.LATC1			// Right motor speed to L298 enable.
										// 0 = Stop, 1 = Full speed, or PWM
#define	LMS		LATCbits.LATC2			// Left motor speed to L298 enable.
										// 0 = stop, 1 = Full speed, or 20 khz PWM
#endif
#endif

#define LeftMotorSpeed SetDCPWM1
#define RightMotorSpeed SetDCPWM2
//** U S A R T ***********************************************************/
#define RESET_BT 0						// Bluetooth module reset is active low.
#define START_BT 1						// Bluetooth module reset is active low.

// Communication port defines
/********************************************************************/
#ifdef DETECTOR_BOARD
/********************************************************************/
#define	CTS			PORTAbits.RA7		// CTS input
#define BT_RESET	LATBbits.LATB3		// Bluetooth module reset output. Active low.
#else 
/********************************************************************/
#ifdef RRP_BOARD
/********************************************************************/
#define	CTS			PORTCbits.RC5		// CTS input
#define BT_RESET	LATCbits.LATC0		// Bluetooth module reset output. Active low.
#endif
#endif

#define	USART_TX	PORTBbits.RC6		// TX out to Zeevo module 
#define	USART_RX	PORTBbits.RC7		// RX in from Zeevo module

//** Sonar ***********************************************************/
#define NOT_TRIGGERED	0;				// Ping sonar module triggered on rising edge.
#define TRIGGERED		1;

#define SONAR_OUT		LATBbits.LATB3	// CCP2 used to time the return pulse.
#define SONAR_IN		PORTBbits.RB3	// B3 used to trigger a pulse.
 
#endif //IO_CFG_H
