/* //////////////////////////////////////////////////////////////////////////*/
/*Commons includes and definitions                                           */
/* //////////////////////////////////////////////////////////////////////////*/

// standard include 
#include <p33Fxxxx.h>
#include <dsp.h>
#include <pwm12.h>
#include <uart.h>
#include <qei.h>
#include <adc.h>
#include <timer.h>
#include <InCap.h>
#include <ports.h>
#include <dma.h>
#include <math.h>
#include <stdlib.h>
#include <libq.h>

/*---------------------------------------------------------------------------*/
/* include       														     */
/*---------------------------------------------------------------------------*/
#include "dsPID4W_prototypes.h"


// Configuration switches for compilation. Comment the undesired lines [34]****
#define CLOCK_FREQ_10 10	// [34b]

// [34c] valid only for 28 pin dsPIC. Choose one only
//#define PROTOTYPE 1
//#define DROIDS 1
#define FOUR_WHEELS 1

// [34d] valid only for 44 pin dsPIC. Choose one only
// #define DSNAVCON33 1
//#define ROBOCONTROLLER 1 

// for debugging purposes------------------------------------------------------
// #define NO_OBSTACLE 1	//  [34e]
// #define debug_SA	// [15]
// #define SIM 1	//	[21]
// #define TIMER_OFF 1
//*****************************************************************************


#ifdef CLOCK_FREQ_10
/*
CLOCK_FREQ = 10 MHz
Fosc = CLOCK_FREQ*M/(N1*N2) = 10M*32/(2*2)=80Mhz
Fcy = Fosc / 2 = 40
*/
#define FCY 40000000
// TCY = 1 / FCY = 25 ns

#else
/*
CLOCK_FREQ = 7,3728 MHz
Fosc = CLOCK_FREQ*M/(N1*N2) = 7.3728M*43/(2*2)=79,2576Mhz
Fcy = Fosc / 2 = 39.6288
*/
#define FCY 39628800
// TCY = 1 / FCY = 25.2341731266 ns

#endif
#define TCY 1/((float)(FCY)

#define _ISR_PSV __attribute__((interrupt, auto_psv))

#define MAX_TX_BUFF 90 // big enough to store a row of cells grid (X_SIZE)
#define MAX_RX_BUFF 144
#define MAX_TX2_BUFF 32

// macro to round a float in a int
#define FLOAT2INT(x) ((x)>=0?(int)((x)+0.5):(int)((x)-0.5))
// #define max(a,b) ((a) > (b) ? (a) : (b))

#define geographic	// [22aa]

#define RX_HEADER_LEN 3					// command string header length (byte)
