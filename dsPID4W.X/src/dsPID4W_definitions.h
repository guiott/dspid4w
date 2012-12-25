/* ////////////////////////////////////////////////////////////////////////////
Included in "dsPID33.c", it contains global definitions and variables 
initialization
/////////////////////////////////////////////////////////////////////////////*/

/*---------------------------------------------------------------------------*/
/* standard include                                                          */
/*---------------------------------------------------------------------------*/
#include "dsPID4W_common.h"

/*---------------------------------------------------------------------------*/
/* Status bits                                                               */
/*---------------------------------------------------------------------------*/

/* Boot Segment Program Memory:No Boot Segment Program Memory
** Write Protect:Disabled **/
//_FBS(BSS_NO_FLASH & BWRP_WRPROTECT_OFF);

// External Oscillator
_FOSCSEL(FNOSC_PRI);			// Primary (XT, HS, EC) Oscillator 

// Clock Switching is enabled and Fail Safe Clock Monitor is disabled
// OSC2 Pin Function: OSC2 is Clock Output
// Primary Oscillator Mode: XT Crystal
_FOSC(FCKSM_CSECMD & OSCIOFNC_OFF  & POSCMD_XT);  				

/* Background Debug Enable Bit: Device will Reset in user mode
** Debugger/Emulator Enable Bit: Reset in operational mode
** JTAG Enable Bit: JTAG is disabled
** ICD communication channel select bits: 
** communicate on PGC1/EMUC1 and PGD1/EMUD1 **/
//_FICD(BKBUG_ON & COE_ON & JTAGEN_OFF & ICS_PGD1); // valid until C30 3.11
_FICD(JTAGEN_OFF & ICS_PGD1);
/*Enable MCLR reset pin and turn on the power-up timers with 64ms delay.
PIN managed by PWM at reset*/           			 
_FPOR(FPWRT_PWR64 & PWMPIN_ON & HPOL_ON & LPOL_ON);				

/* Code Protect: Code Protect off
** Code Protect: Disabled
** Write Protect: Disabled **/ 
_FGS(GSS_OFF & GCP_OFF & GWRP_OFF); 

/* Watchdog Timer Enable:
 * Watchdog timer enabled/disabled by user software
 * Prescler = 128
 * Postscaler = 256
 * WDT timer total about 1sec
*/

_FWDT(FWDTEN_OFF & WDTPOST_PS256 & WDTPRE_PR128);
  			 

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/* External defininitions                                                    */
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

//UART
//{
extern volatile unsigned char UartRxBuff[][2];
extern unsigned char 
		UartTxBuff[] __attribute__((space(dma),aligned(128)));
extern volatile unsigned char UartRxPtrIn;
extern unsigned char UartRxPtrOut;	
extern volatile int UartRxStatus;

extern volatile unsigned char Uart2RxPtrIn;
extern unsigned char Uart2RxPtrOut;
extern volatile int Uart2RxStatus;	
extern unsigned char TmpPtr;
extern unsigned char UartRxPtrData;
extern unsigned char UartRxCmd[];
extern unsigned char UartTmpBuff[][2];
extern const unsigned char Test[];
extern unsigned char TmpPtr2;
extern unsigned char Uart2RxPtrData;
int Port;           // Port = 0 for UART1, Port = 1 for UART2
int SendMapPort;    // to temporay store port number for delayed TX
int ResetPort;      // to temporay store port number for reset
int TmpBufIndx;     // to fill the UartTmpBuff before sending
//}

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/* Global defininitions                                                      */
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

struct Bits{
	unsigned bit0:1;
	unsigned bit1:1;
	unsigned bit2:1;
	unsigned bit3:1;
	unsigned bit4:1;
	unsigned bit5:1;
	unsigned bit6:1;
	unsigned bit7:1;
	unsigned bit8:1;
	unsigned bit9:1;
	unsigned bit10:1;
	unsigned bit11:1;
	unsigned bit12:1;
	unsigned bit13:1;
	unsigned bit14:1;
	unsigned bit15:1;
};

struct Bits VARbits1;
struct Bits VARbits2;
volatile struct Bits VOLbits1;

//----- Flags

#define RC6Tick(x) (Delay1KTCYx(4*x),Delay100TCYx(4*x),Delay10TCYx(3*x))
#define	DELAY100MS Delay10KTCYx(100)	// 100ms

#if defined(__dsPIC33FJ64MC802__) || defined(__dsPIC33FJ128MC802__)
// -- compiling for a 28 pin DSC ***************************************

	#define LED1 _LATA4
	#define LED2 _LATB4
	#define TEST _LATB7
	#define MOTOR_ENABLE_R _LATB13
	#define MOTOR_ENABLE_L _LATB15

#elif defined(__dsPIC33FJ64MC804__) || defined(__dsPIC33FJ128MC804__)
// -- compiling for a 44 pin DSC ***************************************

#ifdef DSNAVCON33

	#define LED1 _LATA8
	#define MOTOR_ENABLE1 _LATA7
	#define MOTOR_ENABLE2 _LATA10

#endif

#ifdef ROBOCONTROLLER
	
	#define LED1 _LATA8
	#define LED2 _LATA9
	#define AUX1 _LATA7
	#define AUX2 _LATA10
	#define MOTOR_ENABLE1 _LATA0
	#define MOTOR_ENABLE2 _LATA1
	#define DIR_485 _LATC3

#endif

#else

#error -- dsPIC33FJ not recognized. Accepted only 64/128MC802 or 64/128MC804

#endif

const long Tcy = 1000000/(float)(FCY)* 100000000;

//----- Variabili
int i = 0; 		// generic index
int j = 0; 		// generic index
long Blink = 0; // heartbeat blink index

// ADC
#define ADC_CALC_FLAG VOLbits1.bit0	// enable ADC value average calculus
#define ADC_OVLD_LIMIT 900 // in mA
#define ADC_OVLD_TIME	100             // n x 10ms
char ADCOvldCount[2] = {0,0};		// how long overload status last 	

// Input Capture (speed measurement)
volatile int Ic1Indx = 0;               // samples buffer index for IC1
volatile int Ic2Indx = 0;		// samples buffer index for IC2
volatile unsigned int Ic1PrevPeriod=0;	// previous sample for IC1
volatile unsigned int Ic2PrevPeriod=0;	// previous sample for IC2
volatile long Ic1Period = 0;		// n samples average value for IC1
volatile long Ic2Period = 0;		// n samples average value for IC2
volatile unsigned int Ic1CurrPeriod;	// current value for IC1
volatile unsigned int Ic2CurrPeriod;	// current value for IC2
volatile int Tmr2OvflwCount1;   	// timer 2 overflow for IC1
volatile int Tmr2OvflwCount2;		// timer 2 overflow for IC2
#define PID_CALC_FLAG VOLbits1.bit1   	// PID and speed elaboration flag

/*
-> [4] [7] [19]
THIS PARAMETERS ARE VALID FOR LINO ROBOTIC PLATFORM.
HERE JUST AS AN EXAMPLE ON HOW TO CALCULATE THEM
Encoder = 512 cpr
Gear reduction ratio = 25:1
Wheel speed = 220 rpm
Encoder pulses for each wheel turn = 12,800
Wheel diameter = 120mm -> circumference = 377mm
Space for each encoder pulse   2x mode Delta S = 0.014726215564mm
Space for each encoder pulse   1x mode Delta S = 0.029452431127mm
Space for each encoder pulse 1/4x mode Delta S = 0.117809724510mm
Speed calculation K in micron/second = 298536736 -> [19]
*/
#define CPR 512.0
#define GEAR_RATIO 25.0
#define CPR_WHEEL CPR*GEAR_RATIO
#define DIAMETER_R  0.12        // right wheel
#define DIAMETER_L  0.12        // left wheel

#define R 0	// right index
#define L 1	// left index
#define RS 2	// right slave index
#define LS 3	// left salve index


// Speed calculation K in m/s
long KvelMode[4][2]; // constants K_VEL << 15 [19c]
// see also http://www.guiott.com/Lino/Speed/DynamicIC.htm
long Kvel[2];        // K_VEL << 15
//ICM<2:0>:Input Capture Mode Select bits
/*
    IC_MODE0 0b001 // 2X mode
    IC_MODE1 0b011 // 1X mode
    IC_MODE2 0b100 // 1/4X mode
    IC_MODE3 0b101 // 1/16X mode
*/
const int IcMode[4] = {0b001, 0b011, 0b100, 0b101};

//ICM<2:0>:Input Capture Mode Select bits
#define IC_MODE0 0b001 // 2X mode
#define IC_MODE1 0b011 // 1X mode
#define IC_MODE2 0b100 // 1/4X mode
#define IC_MODE3 0b101 // 1/16X mode

#define MIN1 3088  // VelLong @ 15 RPM
#define MAX1 5147  // VelLong @ 25 RPM
#define MIN2 10294 // VelLong @ 50 RPM
#define MAX2 12353 // VelLong @ 60 RPM
#define MIN3 39119 // VelLong @ 190 RPM
#define MAX3 43236 // VelLong @ 210 RPM

int VelDes[2] = {0,0};	// desired speed mm/s
int VelMes[2] = {0,0};	// measured speed mm/s	
// threshold to change PID calculation frequency. = Speed in mm/s *2^15/1000
#define VEL_MIN_PID 1600 // Approx 50mm/s
unsigned char PidCycle[2] = {0,0}; // counter for longer cycle PID

// constants for traveled distance calculation: SPACE_ENC_4X in mm
// see [2] in descrEng.exe in dsPID folder
long double Ksp[2];
float Axle;// base width, distance between center of the wheels
float SemiAxle;// Axle/2 useful for odometry calculation
float SemiAxle; // Axle / 2
int SpTick[2] = {0,0};	// distance traveled by a wheel as encoder pulses

float Spmm[2] = {0,0};	// distance traveled by a wheel as mm(SpTick[x]*Ksp[x])
float Space = 0;		// total distance traveled by the robot
#define SPMIN 0.01		// Minimum value to perform odometry
float CosPrev =	1;		// previous value for Cos(ThetaMes)
float SinPrev =	0;		// previous value for Sin(ThetaMes)

long Vel[2];	// speed in m/s  << 15 (long) for PIDR and PIDL


// PID [19d]
tPID PIDstructR;
fractional abcCoefficientR[3] __attribute__ ((section (".xbss, bss, xmemory")));
fractional controlHistoryR[3] __attribute__ ((section (".ybss, bss, ymemory")));
fractional kCoeffs[2][3];

#define KPR kCoeffs[R][0]
#define KIR kCoeffs[R][1]
#define KDR kCoeffs[R][2]
#define PID_OUTR PIDstructR.controlOutput
#define PID_MESR PIDstructR.measuredOutput
#define PID_REFR PIDstructR.controlReference

tPID PIDstructL;
fractional abcCoefficientL[3] __attribute__ ((section (".xbss, bss, xmemory")));
fractional controlHistoryL[3] __attribute__ ((section (".ybss, bss, ymemory")));
#define KPL kCoeffs[L][0]
#define KIL kCoeffs[L][1]
#define KDL kCoeffs[L][2]
#define PID_OUTL PIDstructL.controlOutput
#define PID_MESL PIDstructL.measuredOutput
#define PID_REFL PIDstructL.controlReference

fractional VelFin[2] = {0,0};		// temp for speed during acceleration
fractional PidRef[2]= {0,0};            // speed before normalization for PID

//#define ACC 0.00025			// acceleration
//#define DEC 0.0005			// deceleration
fractional Acc=Q15(0.0005);		// acceleration
fractional Dec=Q15(0.005);              // deceleration

#define PID_CYCLE_MULT 10               // for slow speed calculation
int Curr[2] = {0,0};			// motor current

// Angle PID [23]
tPID AnglePIDstruct;
fractional AngleabcCoefficient[3] 
	__attribute__ ((section (".xbss, bss, xmemory")));
fractional AnglecontrolHistory[3] 
	__attribute__ ((section (".ybss, bss, ymemory")));
fractional AngleKCoeffs[] = {0,0,0};
#define ANGLE_KP AngleKCoeffs[0]	// K parameters
#define ANGLE_KI AngleKCoeffs[1]
#define ANGLE_KD AngleKCoeffs[2]
#define ANGLE_PID_DES AnglePIDstruct.controlReference	// desired angle
#define ANGLE_PID_MES AnglePIDstruct.measuredOutput	// measured angle
#define ANGLE_PID_OUT AnglePIDstruct.controlOutput	// PID output
#define MAX_ROT_SPEED 1200	// MAX speed (+ or -) of wheels during rotation
#define RAD2DEG 57.295779513	// = 180/PI
#define DEG2RAD 0.0174532925	// = PI/180

// #define PI 	  3.1415926536 180°
#define TWOPI 6.2831853072	// 360°
#define HALFPI 1.570796327	// 90°
#define QUARTPI 0.7853981634// 45°

#define ANGLE_OK_FLAG VARbits1.bit0	// target angle reached
#define MIN_THETA_ERR DEG2RAD	// acceptable angle error in radians = 1°

float VelDecr;	// multiplied by VelDesM correct the speed [24d]

// slower cycles setting [13]	
volatile unsigned char Cycle1;
volatile unsigned char Cycle2;
#define CYCLE1_FLAG VOLbits1.bit2
#define CYCLE2_FLAG VOLbits1.bit3
#define	CICLE1_TMO 10		// starts first cycle every Xms
#define	CICLE2_TMO 5		// starts second cycle every N first cycle times

// Idle time [25]
unsigned long IdleCount = 0;	// how many times it executes main loop
volatile unsigned long IdleSample=0;//sampling time to compute idle percentage
#define IDLE_CYCLE 20	
// LOOP_TIME = CICLE1_TMO * CICLE2_TMO	= 50ms	
// so -> IDLE_SAMPLE_TIME = LOOP_TIME * IDLE_CYCLE = 1000ms 
// in order to have percentage as int IDLE_SAMPLE_TIME * 10,000
#define IDLE_SAMPLE_TIME 10000000
// time in ns to execute main without any call
// measured with stopwatch in SIM [25a]
#define IDLE_TIME_PERIOD 1550 	// without optimizations
// #define IDLE_TIME_PERIOD XXXX 	// with optimizations 3
// #define IDLE_TIME_PERIOD XXXX 	// with optimizations s

int IdlePerc;				  // idle percentage

volatile unsigned int BlinkPeriod ;	// LED1 blinking period (ms)
unsigned int BlinkOn;		// LED1 on time (ms)
#define NORM_BLINK_PER 1000	// Period for OK condition
#define NORM_BLINK_ON   200	// ON TIME for OK condition
#define K_ERR_BLINK_PER 4000// Period for ERR condition
#define K_ERR_BLINK_ON 2000	// ON TIME for ERR condition


int ErrCode;                       // Error Code

#define CONSOLE_DEBUG VARbits1.bit1// [30]

#define MAX_SPEED 1200        // rangecheck
unsigned char ResetCount = 0; // [28]

//I2C definitions
unsigned char I2cRegPtr;//Pointer to first byte to read or write in the register

//TX registers array
#define I2C_BUFF_SIZE_TX 16

// I2C TX registers array
struct _TxBuff
{
    int VelInt[4];   // speed in mm/s as an integer for all the wheels
    int ADCValue[4]; // 64 sample average ADC also for slave
};

union __TxBuff
{
    struct _TxBuff I;// to use as integers, little endian LSB first
    char C[I2C_BUFF_SIZE_TX]; // to use as bytes to send on I2C buffer
}I2CTxBuff;


#define I2C_BUFF_SIZE_RX 8

// RX Buffer
struct _RxBuff
{
    int VelDesM;    // mean measured speed mm/s [23]
    int ThetaDes;   // desired orientation angle(set point)[23](Degx10 0-3599)
    int ThetaMes;   // Current orientation angle (Deg x 10 0-3599)
    char NewFlag;   // new values arrived
    char MasterFlag;// to set the board as a master
};

union __RxBuff
{
    struct _RxBuff I;// to use as integers or chars, little endian LSB first
    char C[I2C_BUFF_SIZE_RX];  // to use as bytes to send on I2C buffer
}I2CRxBuff;


// VOLbits1.bit3 and up available

// VARbits1.bit4 and up available

// VARbits2.bit0 and up available

/* Definitions end ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
