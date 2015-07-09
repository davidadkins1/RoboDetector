/* oi.h
 *
 * Definitions for the Open Interface
 */


// Command values
#define CmdStart        (unsigned portCHAR)128
#define CmdBaud         (unsigned portCHAR)129
#define CmdControl      (unsigned portCHAR)130
#define CmdSafe         (unsigned portCHAR)131
#define CmdFull         (unsigned portCHAR)132
#define CmdSpot         (unsigned portCHAR)134
#define CmdClean        (unsigned portCHAR)135
#define CmdDemo         (unsigned portCHAR)136
#define CmdDrive        (unsigned portCHAR)137
#define CmdMotors       (unsigned portCHAR)138
#define CmdLeds         (unsigned portCHAR)139
#define CmdSong         (unsigned portCHAR)140
#define CmdPlay         (unsigned portCHAR)141
#define CmdSensors      (unsigned portCHAR)142
#define CmdDock         (unsigned portCHAR)143
#define CmdPWMMotors    (unsigned portCHAR)144
#define CmdDriveWheels  (unsigned portCHAR)145
#define CmdOutputs      (unsigned portCHAR)147
#define CmdSensorList   (unsigned portCHAR)149
#define CmdIRChar       (unsigned portCHAR)151
#define CmdScript       (unsigned portCHAR)152
#define PlayScript      (unsigned portCHAR)153

// My enhancements
#define CmdDriveDistance    (unsigned portCHAR)201
#define TurnToHeading       (unsigned portCHAR)202

enum OI_STATE
{
    OI_OFF,
    OI_PASSIVE,
    OI_SAFE,
    OI_FULL
};


/*
typedef unsigned char   byte;           // 8-bit
typedef unsigned int    word;           // 16-bit
typedef unsigned long   dword;          // 32-bit

typedef union _BYTE
{
    byte _byte;
    struct
    {
        unsigned b0:1;
        unsigned b1:1;
        unsigned b2:1;
        unsigned b3:1;
        unsigned b4:1;
        unsigned b5:1;
        unsigned b6:1;
        unsigned b7:1;
    };
} BYTE;

typedef union _WORD
{
    int _word;
    struct
    {
        byte byte0;
        byte byte1;
    };
    struct
    {
        BYTE Byte0;
        BYTE Byte1;
    };
    struct
    {
        byte lsb;
        byte msb;
    };
    struct
    {
        byte MSB;
        byte LSB;
    };
    struct
    {
        byte v[2];
    };
} WORD;
*/
typedef unsigned char		BYTE;				// 8-bit unsigned
typedef unsigned short int	WORD;				// 16-bit unsigned
typedef unsigned long		DWORD;				// 32-bit unsigned
typedef signed char		CHAR;				// 8-bit signed
typedef signed short int	SHORT;				// 16-bit signed
typedef signed long		LONG;				// 32-bit signed

typedef union _BYTE_VAL
{
    BYTE Val;
    struct
    {
        unsigned char b0:1;
        unsigned char b1:1;
        unsigned char b2:1;
        unsigned char b3:1;
        unsigned char b4:1;
        unsigned char b5:1;
        unsigned char b6:1;
        unsigned char b7:1;
    } bits;
} BYTE_VAL;

typedef union _WORD_VAL
{
    WORD Val;
    SHORT sVal;
    BYTE v[2];
    struct
    {
        BYTE LB;
        BYTE HB;
    } byte;
    struct
    {
        unsigned char b0:1;
        unsigned char b1:1;
        unsigned char b2:1;
        unsigned char b3:1;
        unsigned char b4:1;
        unsigned char b5:1;
        unsigned char b6:1;
        unsigned char b7:1;
        unsigned char b8:1;
        unsigned char b9:1;
        unsigned char b10:1;
        unsigned char b11:1;
        unsigned char b12:1;
        unsigned char b13:1;
        unsigned char b14:1;
        unsigned char b15:1;
    } bits;
} WORD_VAL;

typedef union _DWORD_VAL
{
    DWORD Val;
    LONG sVal;
	WORD w[2];
    BYTE v[4];
    struct
    {
        WORD LW;
        WORD HW;
    } word;
    struct
    {
        BYTE LB;
        BYTE HB;
        BYTE UB;
        BYTE MB;
    } byte;
    struct
    {
        unsigned char b0:1;
        unsigned char b1:1;
        unsigned char b2:1;
        unsigned char b3:1;
        unsigned char b4:1;
        unsigned char b5:1;
        unsigned char b6:1;
        unsigned char b7:1;
        unsigned char b8:1;
        unsigned char b9:1;
        unsigned char b10:1;
        unsigned char b11:1;
        unsigned char b12:1;
        unsigned char b13:1;
        unsigned char b14:1;
        unsigned char b15:1;
        unsigned char b16:1;
        unsigned char b17:1;
        unsigned char b18:1;
        unsigned char b19:1;
        unsigned char b20:1;
        unsigned char b21:1;
        unsigned char b22:1;
        unsigned char b23:1;
        unsigned char b24:1;
        unsigned char b25:1;
        unsigned char b26:1;
        unsigned char b27:1;
        unsigned char b28:1;
        unsigned char b29:1;
        unsigned char b30:1;
        unsigned char b31:1;
    } bits;
} DWORD_VAL;

typedef struct _SENSOR_DATA
{
	unsigned char 	ucBumpDrop;                 // Packet ID:  7 - Bumps and Wheel Drops 
	unsigned char	bWall;                      // Packet ID:  8 - Wall
	unsigned char	bCliffLeft;                 // Packet ID:  9 - Cliff Left
	unsigned char	bCliffFrontLeft;            // Packet ID: 10 - Cliff Front Left
	unsigned char	bCliffFrontRight;           // Packet ID: 11 - Cliff Front Right
	unsigned char	bCliffRight;                // Packet ID: 12 - Cliff Right
	unsigned char	bVirtualWall;               // Packet ID: 13 - Virtual Wall
	unsigned char	ucOverCurrent;              // Packet ID: 14 - Low Side Driver and Wheel Overcurrents
	unsigned char	ucUnused1;                  // Packet ID: 15 - Unused byte
	unsigned char	ucUnused2;                  // Packet ID: 16 - Unused byte
	unsigned char	ucInfraredRX;               // Packet ID: 17 - Infrared byte
	unsigned char	ucButtons;                  // Packet ID: 18 - Buttons
	WORD_VAL		iDistanceTraveled;          // Packet ID: 19 - Distance Traveled
	WORD_VAL		iAngleTraveled;             // Packet ID: 20 - Angle Travled 
	unsigned char	ucChargingState;
	unsigned int	uiBatteryVoltage;
	int				iCurrent;
	char			cBatteryTemp;
	unsigned int	uiBatteryCharge;
	unsigned int	uiBatteryCapacity;
	unsigned int	uiWallSignal;
	unsigned int	uiCliffLeftSignal;
	unsigned int	uiCliffFrontLeftSignal;
	unsigned int	uiCliffFrontRightSignal;
	unsigned int	uiCliffRightSignal;
	unsigned char	ucDigitalInputs;
	unsigned int	uiAnalogSignal;
	unsigned char	ucChargingSource;
	enum OI_STATE 	ucOIMode;
	unsigned char	ucSongNumber;
	unsigned char	ucSongPlaying;
	unsigned char	ucNumberofStreamPackets;
	int				iRequestedVelocity;
	WORD_VAL		iRequestedRadius;
	WORD_VAL		iRequestedRightVelocity;
	WORD_VAL		iRequestedLeftVelocity;
}SENSOR_DATA;

typedef union _SENSORS
{
	SENSOR_DATA sensor;
	
	struct
	{
		unsigned char byte[sizeof(SENSOR_DATA)];
	};
} SENSORS;

// Sensor byte indices - offsets in packets 0, 5 and 6
#define SenBumpDrop     0            
#define SenWall         1
#define SenCliffL       2
#define SenCliffFL      3
#define SenCliffFR      4
#define SenCliffR       5
#define SenVWall        6
#define SenOverC        7
#define SenIRChar       10
#define SenButton       11
#define SenDist1        12
#define SenDist0        13
#define SenAng1         14
#define SenAng0         15
#define SenChargeState  16
#define SenVolt1        17
#define SenVolt0        18
#define SenCurr1        19
#define SenCurr0        20
#define SenTemp         21
#define SenCharge1      22
#define SenCharge0      23
#define SenCap1         24
#define SenCap0         25
#define SenWallSig1     26
#define SenWallSig0     27
#define SenCliffLSig1   28
#define SenCliffLSig0   29
#define SenCliffFLSig1  30
#define SenCliffFLSig0  31
#define SenCliffFRSig1  32
#define SenCliffFRSig0  33
#define SenCliffRSig1   34
#define SenCliffRSig0   35
#define SenInputs       36
#define SenAInput1      37
#define SenAInput0      38
#define SenChAvailable  39
#define SenOIMode       40
#define SenOISong       41
#define SenOISongPlay   42
#define SenStreamPckts  43
#define SenVel1         44
#define SenVel0         45
#define SenRad1         46
#define SenRad0         47
#define SenVelR1        48
#define SenVelR0        49
#define SenVelL1        50
#define SenVelL0        51


// Sensor packet sizes
#define Sen0Size        26
#define Sen1Size        10
#define Sen2Size        6
#define Sen3Size        10
#define Sen4Size        14
#define Sen5Size        12
#define Sen6Size        52

// Sensor bit masks
#define WheelDropFront  0x10
#define WheelDropLeft   0x08
#define WheelDropRight  0x04
#define BumpLeft        0x02
#define BumpRight       0x01
#define BumpBoth        0x03
#define BumpEither      0x03
#define WheelDropAll    0x1C
#define ButtonAdvance   0x04
#define ButtonPlay      0x01


// LED Bit Masks
#define LEDAdvance       0x08
#define LEDPlay         0x02
#define LEDsBoth        0x0A

// OI Modes
#define OIOff		    (unsigned portCHAR)0
#define OIPassive       (unsigned portCHAR)1
#define OISafe          (unsigned portCHAR)2
#define OIFull          (unsigned portCHAR)3


// Baud codes
#define Baud300         0
#define Baud600         1
#define Baud1200        2
#define Baud2400        3
#define Baud4800        4
#define Baud9600        5
#define Baud14400       6
#define Baud19200       7
#define Baud28800       8
#define Baud38400       9
#define Baud57600       10
#define Baud115200      11


// Drive radius special cases
#define RadStraight     32768
#define RadCCW          1
#define RadCW           -1



// Baud UBRRx values
#define Ubrr300         3839
#define Ubrr600         1919
#define Ubrr1200        959
#define Ubrr2400        479
#define Ubrr4800        239
#define Ubrr9600        119
#define Ubrr14400       79
#define Ubrr19200       59
#define Ubrr28800       39
#define Ubrr38400       29
#define Ubrr57600       19
#define Ubrr115200      9


