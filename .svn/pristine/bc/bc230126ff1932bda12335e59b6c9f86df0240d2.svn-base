/*********************************************************************
 *
 *      Metal Detector Robot
 *
 *********************************************************************
 * FileName:        Compass.c
 * Processor:       PIC18
 * Compiler:        C18 3.01+
 *
 * Author               Date        Comment
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * David Adkins       03/02/12    Original.
 ********************************************************************/

/** I N C L U D E S **********************************************************/
#include "FreeRTOS.h"
#include "task.h"
//#include "queue.h"

//#include "behaviors.h"
#include "RoboDetector_tasks.h"

#include "io_cfg.h"
#include <i2c.h>
#include "oi.h"
#include <math.h>

/** Defines ********************************************************/
#define WRITE_DATA  0x3C
#define READ_DATA   0x3D
#define MODE        0x02
#define X_MSB       0x03

#define PI 3.141593
#define NINETY_DEGREES      1.570796
#define ONE_EIGHTY_DEGREES  3.141593
#define TWO_SEVENTY_DEGREES 4.712389
#define THREE_SIXTY_DEGREES 6.283185
#define DECLINATION_CORRECTION .1902401
#define DEGREE_CONVERSION 57.29578


/** Global Variables ********************************************************/
#pragma udata
xTaskHandle compass_task_handle;
float azimuth;
extern SENSORS sensorPacket;
/** Local Variables ********************************************************/
#pragma udata
static WORD_VAL x;
static WORD_VAL y;
static WORD_VAL z;
static unsigned int heading_degrees;
static unsigned int last_heading_degrees;
static WORD_VAL turn_delta_accumulator;
static WORD_VAL turn_delta;
static DWORD_VAL turn_delta_test;

/*-----------------------------------------------------------*/
#pragma code
void init_compass(void)
{
    CloseI2C();    //close i2c if was operating earlier
    
	// configure SSP for hardware master mode I2C
    OpenI2C(MASTER, SLEW_OFF);
    SSPADD = 79;             //100kHz Baud clock
					
	PIR1bits.SSPIF = 0;			// remove any interrupt condition
	
	heading_degrees = 0;
	last_heading_degrees = 0xffffu;  // Flag for first update

    xTaskCreate( compass_task, ( const portCHAR * const ) "C", COMPASS_STACK_SIZE, NULL, COMPASS_TASK_PRIORITY, &compass_task_handle );
}

void write_i2c_register(unsigned char register_addr, unsigned char register_value)
{
    unsigned char flush_data;
    unsigned char status;
    
    //---START I2C---
    StartI2C();
    IdleI2C();
    WriteI2C( WRITE_DATA );   //write the address of slave
    IdleI2C();
    WriteI2C( register_addr );   //write the address of slave
    IdleI2C();
    WriteI2C( register_value );
    StopI2C();
}

void get_raw_reading(void)
{
    //---START I2C---
    StartI2C();
    IdleI2C();
    WriteI2C( WRITE_DATA );    //write the address of slave
    IdleI2C();
    WriteI2C( X_MSB );         //write the address of slave
    IdleI2C();

    RestartI2C(); // Initiate a RESTART command    
    IdleI2C();
    WriteI2C( READ_DATA );    //write the address of slave
    IdleI2C();
    
    // Read X
    x.byte.HB = ReadI2C();
    IdleI2C();
    AckI2C();
    IdleI2C();
    x.byte.LB = ReadI2C();
    IdleI2C();
    AckI2C();
    IdleI2C();

    // Read Z
    z.byte.HB = ReadI2C();
    IdleI2C();
    AckI2C();
    IdleI2C();
    z.byte.LB = ReadI2C();
    IdleI2C();
    AckI2C();
    IdleI2C();

    // Read Y
    y.byte.HB = ReadI2C();
    IdleI2C();
    AckI2C();
    IdleI2C();
    y.byte.LB = ReadI2C();
    IdleI2C();
    NotAckI2C();
    IdleI2C();
    StopI2C();
}

void convert_azimuth(void)
{
    float azimuth_temp;
					
    if(0 == x.sVal)
    {
        if( y.sVal < 0)
        {
            azimuth_temp = NINETY_DEGREES;
        }
        else
        {
            azimuth_temp = TWO_SEVENTY_DEGREES;
        }
    }
    else
    {
        azimuth_temp = atan((float)y.sVal / (float)x.sVal);

        if( x.sVal < 0)
        {
            azimuth_temp = ONE_EIGHTY_DEGREES - azimuth_temp;
        }
        else
        {
            if( y.sVal < 0)
            {
                azimuth_temp = -azimuth_temp;
            }
            else
            {
                azimuth_temp = THREE_SIXTY_DEGREES - azimuth_temp;
            }
        }
    }
    
    // Don't change 0 to 360
    if(0 != azimuth_temp)
    {
        // For unknown reasons the compass reads CCW
        // which reverses E and W
        // This fixes the problem
        azimuth_temp = THREE_SIXTY_DEGREES - azimuth_temp;
    }

    // Add the local declination compensation for true North
    azimuth_temp += DECLINATION_CORRECTION;
    
    if(azimuth_temp < 0)
    {
        azimuth_temp += THREE_SIXTY_DEGREES;
    }
    else if(azimuth_temp >= THREE_SIXTY_DEGREES)
    {
        azimuth_temp -= THREE_SIXTY_DEGREES;
    }
    
    azimuth = azimuth_temp;
    heading_degrees = azimuth * DEGREE_CONVERSION;
}

void update_turn_angle(void)
{
    // Update and return if first time through
    if(0xffffu == last_heading_degrees)
    {
        last_heading_degrees = heading_degrees;
        return;
    }
    
    // Calculate the heading delta
    if(last_heading_degrees == heading_degrees)
    {
        // Exit if no change in heading
        return;
    }
    
    // Counter-clockwise angles are positive and clockwise angles are negative.
    turn_delta.sVal = last_heading_degrees - heading_degrees;
    
    // check for zero cross
    if(turn_delta.sVal > 180)
    {
        turn_delta.sVal -= 360;
    }
    else if(turn_delta.sVal < -180)
    {
        turn_delta.sVal += 360;
    }
    
    // Protect from interrupts
    taskENTER_CRITICAL();
    
    //Convert from big endian to little
    turn_delta_accumulator.byte.HB = sensorPacket.sensor.iAngleTraveled.byte.LB;
    turn_delta_accumulator.byte.LB = sensorPacket.sensor.iAngleTraveled.byte.HB;
    
    // Convert turn accumulator to long for overflow testing
    turn_delta_test.sVal = (long)turn_delta_accumulator.sVal + (long)turn_delta.sVal;
     
    // Limit result to +32767 -32768
    if(turn_delta_test.sVal > 32767)
    {
        turn_delta_accumulator.sVal = 32767;
    }
    else if(turn_delta_test.sVal < -32768)
    {
        turn_delta_accumulator.sVal = -32768;
    }
    else
    {
        turn_delta_accumulator.byte.HB = turn_delta_test.byte.HB;
        turn_delta_accumulator.byte.LB = turn_delta_test.byte.LB;
    }
    
    //Convert from little endian to big
    sensorPacket.sensor.iAngleTraveled.byte.LB = turn_delta_accumulator.byte.HB;
    sensorPacket.sensor.iAngleTraveled.byte.HB = turn_delta_accumulator.byte.LB;
    taskEXIT_CRITICAL();
    
    // Save current heading as last
    last_heading_degrees = heading_degrees;
}

portTASK_FUNCTION( compass_task, pvParameters )
{
    vTaskDelay( 100 );      // Power-up delay for compass

    //check for bus idle condition in multi master communication
    IdleI2C();

    // Start compass continous conversion mode
    write_i2c_register(MODE, 0);
    
    for(;;)
    {
        vTaskDelay( 50 );
        get_raw_reading();
        convert_azimuth();
        update_turn_angle();
    }
}

unsigned int current_heading(void)
{
    return heading_degrees;
}
/*end of module*/
