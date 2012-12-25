/* ////////////////////////////////////////////////////////////////////////////
** It contains dsPIC settings and initializations
**
** Detailed description are on file "descrEng.txt" 
** numbers between brackets, eg.: [1] , are the references to the specific 
** decription into the file     
/////////////////////////////////////////////////////////////////////////////*/

// standard includes
#include "dsPid4W_settings.h"
#include "dsPID4W_common.h"

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/* ports and peripherals registers setting an initialization                 */
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

// DMA buffers
// ADC buffer, 2 channels (AN0, AN1), 32 bytes each, 2 x 32 = 64 bytes
int DmaAdc[2][64] __attribute__((space(dma),aligned(256)));

void Settings(void)
{/**
*\brief It contains everything needed for dsPIC settings and initializations
*/

#ifdef CLOCK_FREQ_10 //{
// Configure Oscillator to operate the device at 40 Mhz
// Fosc= Fin*M/(N1*N2), Fcy=Fosc/2
// Fosc= 10M*32/(2*2)=80Mhz for 10 MHz input clock
PLLFBD=30;					// M=32
#warning **********************************************************************
#warning -- compiling for 10MHz oscillator, set MPLAB SIM to 80.0000 MHz ******
#else

// Configure Oscillator to operate the device at 39,6288 Mhz
// Fosc= Fin*M/(N1*N2), Fcy=Fosc/2
// Fosc= 7.3728M*43/(2*2)=79,2576Mhz for 7,3728 MHz input clock
PLLFBD=41;					// M=43
#warning **********************************************************************
#warning -- compiling for 7.3728MHz oscillator, set MPLAB SIM to 79.2576 MHz **
#endif //}

CLKDIVbits.PLLPOST=0;		// N1=2
CLKDIVbits.PLLPRE=0;		// N2=2

// Enable Watch Dog Timer via SW
_SWDTEN=1;

// Clock switching to incorporate PLL
__builtin_write_OSCCONH(0x03);		// Initiate Clock Switch to Primary
					// Oscillator with PLL (NOSC=0b011)
__builtin_write_OSCCONL(0x01);		// Start clock switching

#ifndef SIM	// [21]
	while (OSCCONbits.COSC != 0b011);// Wait for Clock switch to occur	

	while(OSCCONbits.LOCK!=1) {};	// Wait for PLL to lock
#endif 

#if defined(__dsPIC33FJ64MC802__) || defined(__dsPIC33FJ128MC802__) //{
#warning -- compiling for a 28 pin DSC ****************************************

#ifdef PROTOTYPE //{
#warning -- compiling for prototype board *************************************

	// Peripheral PIN selection ***********************************
	// Unlock Registers
	//*************************************************************
	asm volatile ( "mov #OSCCONL, w1 \n"
	"mov #0x45, w2 \n"
	"mov #0x57, w3 \n"
	"mov.b w2, [w1] \n"
	"mov.b w3, [w1] \n"
	"bclr OSCCON, #6 ");
	//************************************************************
	// Configure Input Functions
	//************************************************************
	//***************************
	// Assign IC1 To Pin RP6
	//***************************
	RPINR7bits.IC1R = 6;
	
	//***************************
	// Assign IC2 To Pin RP10
	//***************************
	RPINR7bits.IC2R = 10;
	
	//***************************
	// Assign QEA1 To Pin RP6
	//***************************
	RPINR14bits.QEA1R = 6;
	
	//***************************
	// Assign QEB1 To Pin RP5
	//***************************
	RPINR14bits.QEB1R = 5;
	
	//***************************
	// Assign QEA2 To Pin RP10
	//***************************
	RPINR16bits.QEA2R = 10;
	
	//***************************
	// Assign QEB2 To Pin RP11
	//***************************
	RPINR16bits.QEB2R = 11;
	
	//***************************
	// Assign U1RX To Pin RP8
	//***************************
	RPINR18bits.U1RXR = 8;
	
	//***************************
	// Assign U2RX To Pin RP7
	//***************************
	RPINR19bits.U2RXR = 7;
	
	//************************************************************
	// Configure Output Functions
	//************************************************************
	
	//***************************
	// Assign U1Tx To Pin RP9
	//***************************
	RPOR4bits.RP9R = 3;
	
	//***************************
	// Assign U2Tx To Pin RP4
	//***************************
	RPOR2bits.RP4R = 5;
	
	//************************************************************
	// Lock Registers
	//************************************************************
	asm volatile ( "mov #OSCCONL, w1 \n"
	"mov #0x45, w2 \n"
	"mov #0x57, w3 \n"
	"mov.b w2, [w1] \n"
	"mov.b w3, [w1] \n"
	"bset OSCCON, #6");
	// *********************************** Peripheral PIN selection
	
	/*-----------------------------------------------------------*/
	/* Port	A   						     */
	/*-----------------------------------------------------------*/
	_TRISA4 = 0;
	/*
	RA4 12 LED1
	*/
	
	/*-----------------------------------------------------------*/
	/* Port	B   			    			     */
	/*-----------------------------------------------------------*/
	_TRISB2  = 0;
	_TRISB3	 = 0;
	
	/*
	RB2  6  H-bridge1 enable
	RB3	 7	H-bridge2 enable
	*/
#endif //}


#ifdef FOUR_WHEELS //{
#warning -- compiling for 990.011 board in 4 wheel drive (master-slave) config*

	// Peripheral PIN selection ***********************************
	// Unlock Registers
	//*************************************************************
	asm volatile ( "mov #OSCCONL, w1 \n"
	"mov #0x45, w2 \n"
	"mov #0x57, w3 \n"
	"mov.b w2, [w1] \n"
	"mov.b w3, [w1] \n"
	"bclr OSCCON, #6 ");
	//************************************************************
	// Configure Input Functions
	//************************************************************
	//***************************
	// Assign IC1 To Pin RP6
	//***************************
	RPINR7bits.IC1R = 6;

	//***************************
	// Assign IC2 To Pin RP10
	//***************************
	RPINR7bits.IC2R = 10;

	//***************************
	// Assign QEA1 To Pin RP6
	//***************************
	RPINR14bits.QEA1R = 6;

	//***************************
	// Assign QEB1 To Pin RP5
	//***************************
	RPINR14bits.QEB1R = 5;

	//***************************
	// Assign QEA2 To Pin RP10
	//***************************
	RPINR16bits.QEA2R = 10;

	//***************************
	// Assign QEB2 To Pin RP11
	//***************************
	RPINR16bits.QEB2R = 11;

	//***************************
	// Assign U1RX To Pin RP3
  // ATTENTION
  // on the dsNav schematic the UART1 and 2 are inverted
	//***************************
	RPINR18bits.U1RXR = 3;

	//***************************
	// Assign U2RX To Pin RP7
  // not used. I2C is used instead
	//***************************
	// RPINR19bits.U2RXR = 7;

	//************************************************************
	// Configure Output Functions
	//************************************************************

	//***************************
	// Assign U1Tx To Pin RP2
  // ATTENTION
  // on the dsNav schematic the UART1 and 2 are inverted
	//***************************
	RPOR1bits.RP2R = 3;
	//***************************
	// Assign U2Tx To Pin RP8
  // not used. I2C is used instead
	//***************************
	// RPOR4bits.RP8R = 5;

	//************************************************************
	// Lock Registers
	//************************************************************
	asm volatile ( "mov #OSCCONL, w1 \n"
	"mov #0x45, w2 \n"
	"mov #0x57, w3 \n"
	"mov.b w2, [w1] \n"
	"mov.b w3, [w1] \n"
	"bset OSCCON, #6");
	// *********************************** Peripheral PIN selection

	/*-----------------------------------------------------------*/
	/* Port	A   			    			     */
	/*-----------------------------------------------------------*/
	_TRISA4 = 0;
	/*
	RA4 12 LED1
	*/

	/*-----------------------------------------------------------*/
	/* Port	B   			 			     */
	/*-----------------------------------------------------------*/
	_TRISB4	 = 0;
  // _TRISB7	 = 0; // used just for test
	_TRISB13 = 0;
	_TRISB15 = 0;
  /*
	RB4  11 LED2
  RB7  16 Used for tests
	RB13 24 Motor 1 Enable
	RB15 26 Motor 2 Enable
 	*/
#endif //}


#ifdef DROIDS //{
#warning -- compiling for 990.011 board **************************************

	// Peripheral PIN selection ***********************************
	// Unlock Registers
	//*************************************************************
	asm volatile ( "mov #OSCCONL, w1 \n"
	"mov #0x45, w2 \n"
	"mov #0x57, w3 \n"
	"mov.b w2, [w1] \n"
	"mov.b w3, [w1] \n"
	"bclr OSCCON, #6 ");
	//************************************************************
	// Configure Input Functions
	//************************************************************
	//***************************
	// Assign IC1 To Pin RP10
	//***************************
	RPINR7bits.IC1R = 10;
	
	//***************************
	// Assign IC2 To Pin RP6
	//***************************
	RPINR7bits.IC2R = 6;
	
	//***************************
	// Assign QEA1 To Pin RP10
	//***************************
	RPINR14bits.QEA1R = 10;
	
	//***************************
	// Assign QEB1 To Pin RP11
	//***************************
	RPINR14bits.QEB1R = 11;
	
	//***************************
	// Assign QEA2 To Pin RP6
	//***************************
	RPINR16bits.QEA2R = 6;
	
	//***************************
	// Assign QEB2 To Pin RP5
	//***************************
	RPINR16bits.QEB2R = 5;
	
	//***************************
	// Assign U1RX To Pin RP3
	//***************************
	RPINR18bits.U1RXR = 3;
	
	//***************************
	// Assign U2RX To Pin RP7
	//***************************
	RPINR19bits.U2RXR = 7;
	
	//************************************************************
	// Configure Output Functions
	//************************************************************
	
	//***************************
	// Assign U1Tx To Pin RP2
	//***************************
	RPOR1bits.RP2R = 3;
	//***************************
	// Assign U2Tx To Pin RP8
	//***************************
	RPOR4bits.RP8R = 5;

	//************************************************************
	// Lock Registers
	//************************************************************
	asm volatile ( "mov #OSCCONL, w1 \n"
	"mov #0x45, w2 \n"
	"mov #0x57, w3 \n"
	"mov.b w2, [w1] \n"
	"mov.b w3, [w1] \n"
	"bset OSCCON, #6");
	// *********************************** Peripheral PIN selection
	
	/*-----------------------------------------------------------*/
	/* Port	A                               		     */
	/*-----------------------------------------------------------*/
	_TRISA4 = 0;
	/*
	RA4 12 LED1
	*/
	
	/*-----------------------------------------------------------*/
	/* Port	B                                                    */
	/*-----------------------------------------------------------*/
	_TRISB0  = 0;
	_TRISB1	 = 0;
	_TRISB4	 = 0;
	_TRISB9	 = 0;
	
	/*
	RB0  4  H-bridge2 enable
	RB1	 5	H-bridge1 enable
	RB4  11 LED2
	RB9  18 DIR 485
	*/
#endif //}
//}
	
#elif defined(__dsPIC33FJ64MC804__) || defined(__dsPIC33FJ128MC804__) //{
#warning -- compiling for a 44 pin DSC ***************************************


#ifdef DSNAVCON33 //{
#warning -- compiling for DSNAVCON33 *****************************************
	// Peripheral PIN selection ***********************************
	// Unlock Registers
	//*************************************************************
	asm volatile ( "mov #OSCCONL, w1 \n"
	"mov #0x45, w2 \n"
	"mov #0x57, w3 \n"
	"mov.b w2, [w1] \n"
	"mov.b w3, [w1] \n"
	"bclr OSCCON, #6 ");
	//************************************************************
	// Configure Input Functions
	//************************************************************
	//***************************
	// Assign IC1 To Pin RP23
	//***************************
	RPINR7bits.IC1R = 23;
	
	//***************************
	// Assign IC2 To Pin RP25
	//***************************
	RPINR7bits.IC2R = 25;
	
	//***************************
	// Assign QEA1 To Pin RP23
	//***************************
	RPINR14bits.QEA1R = 23;
	
	//***************************
	// Assign QEB1 To Pin RP22
	//***************************
	RPINR14bits.QEB1R = 22;
	
	//***************************
	// Assign QEA2 To Pin RP25
	//***************************
	RPINR16bits.QEA2R = 25;
	
	//***************************
	// Assign QEB2 To Pin RP24
	//***************************
	RPINR16bits.QEB2R = 24;
	
	//***************************
	// Assign U1RX To Pin RP5
	//***************************
	RPINR18bits.U1RXR = 5;
	
	//***************************
	// Assign U2RX To Pin RP20
	//***************************
	RPINR19bits.U2RXR = 20;
	
	//************************************************************
	// Configure Output Functions
	//************************************************************
	
	//***************************
	// Assign U1Tx To Pin RP6
	//***************************
	RPOR3bits.RP6R = 3;
	
	//***************************
	// Assign U2Tx To Pin RP21
	//***************************
	RPOR10bits.RP21R = 5;
	
	//************************************************************
	// Lock Registers
	//************************************************************
	asm volatile ( "mov #OSCCONL, w1 \n"
	"mov #0x45, w2 \n"
	"mov #0x57, w3 \n"
	"mov.b w2, [w1] \n"
	"mov.b w3, [w1] \n"
	"bset OSCCON, #6");
	// *********************************** Peripheral PIN selection
	
	/*-----------------------------------------------------------*/
	/* Port	A   			    			     */
	/*-----------------------------------------------------------*/
	_TRISA8 = 0;
	/*
	RA8 32 LED1
	*/
	
	/*-----------------------------------------------------------*/
	/* Port	B   			    			     */
	/*-----------------------------------------------------------*/
	_TRISA7  = 0;
	_TRISA10 = 0;
	
	/*
	RA7  13  H-bridge1 enable
	RA10 12	H-bridge2 enable
	*/
#endif //}

#ifdef ROBOCONTROLLER //{
#warning -- compiling for ROBOCONTROLLER *************************************
	// Peripheral PIN selection ***********************************
	// Unlock Registers
	//*************************************************************
	asm volatile ( "mov #OSCCONL, w1 \n"
	"mov #0x45, w2 \n"
	"mov #0x57, w3 \n"
	"mov.b w2, [w1] \n"
	"mov.b w3, [w1] \n"
	"bclr OSCCON, #6 ");
	//************************************************************
	// Configure Input Functions
	//************************************************************
	//***************************
	// Assign IC1 To Pin RP22
	//***************************
	RPINR7bits.IC1R = 22;	// (Input Capture 1) QEA_1 su RoboController
	
	//***************************
	// Assign IC2 To Pin RP24					
	//***************************
	RPINR7bits.IC2R = 24;	// (Input Capture 2) QEA_2 su RoboController
	
	//***************************
	// Assign QEA1 To Pin RP22
	//***************************
	RPINR14bits.QEA1R = 22;		// QEA_1 su RoboController
	
	//***************************
	// Assign QEB1 To Pin RP23
	//***************************
	RPINR14bits.QEB1R = 23;		// QEB_1 su RoboController
	
	//***************************
	// Assign QEA2 To Pin RP24					
	//***************************
	RPINR16bits.QEA2R = 24;		// QEA_2 su RoboController
	
	//***************************
	// Assign QEB2 To Pin RP25
	//***************************
	RPINR16bits.QEB2R = 25;		// QEB_2 su RoboController
	
	//***************************
	// Assign U1RX To Pin RP8
	//***************************
	RPINR18bits.U1RXR = 20;		// (UART1 Receive) su RoboController
	
	//***************************
	// Assign U2RX To Pin RP20
	//***************************
	RPINR19bits.U2RXR = 8;		// (UART2 Receive) su RoboController		
	
	//************************************************************
	// Configure Output Functions
	//************************************************************
	
	//***************************
	// Assign U1Tx
	//***************************
	RPOR10bits.RP21R = 3;
	
	//***************************
	// Assign U2Tx
	//***************************
	RPOR4bits.RP8R = 5;
	
	//************************************************************
	// Lock Registers
	//************************************************************
	asm volatile ( "mov #OSCCONL, w1 \n"
	"mov #0x45, w2 \n"
	"mov #0x57, w3 \n"
	"mov.b w2, [w1] \n"
	"mov.b w3, [w1] \n"
	"bset OSCCON, #6");
	// *********************************** Peripheral PIN selection
		
	/*-----------------------------------------------------------*/
	/* Port	A                                                    */
	/*-----------------------------------------------------------*/
	_TRISA0 = 0;		// MOTOR_EN1
	_TRISA1 = 0;		// MOTOR_EN2
	_TRISA8 = 0;		// LED1
	_TRISA9 = 0;		// LED2
	_TRISA7 = 0;		// AUX1
	_TRISA10 = 0;		// AUX2
	
	/*-----------------------------------------------------------*/
	/* Port	B   			       			     */
	/*-----------------------------------------------------------*/
	//_TRISB2  = 0;
	//_TRISB3  = 0;
	
	
	/*-----------------------------------------------------------*/
	/* Port	C   			    			     */
	/*-----------------------------------------------------------*/
	_TRISC3  = 0;		// DIR RS485
	_TRISC5  = 0;		// TX UART1

#endif //}

#else

#error -- dsPIC33FJ not recognized. Accepted only 64/128MC802 or 64/128MC804

#endif //}
#warning *********************************************************************

/*-------------------------------------------------------------------*/
/* A/D converter [2]                                                 */
/*-------------------------------------------------------------------*/
AD1CON1bits.ADSIDL = 1;	// stop in idle
AD1CON1bits.ADDMABM = 0;// scatter/gather mode
AD1CON1bits.AD12B = 0;	// 10 bits ADC 
AD1CON1bits.FORM = 0;	// integer format
AD1CON1bits.SSRC = 0b111;// auto-convert
AD1CON1bits.SIMSAM = 1;	// CH0 CH1 sampled simultaneously
AD1CON1bits.ASAM = 1;	// auto sampling

AD1CON2bits.CSCNA = 0;	// do not scan inputs
AD1CON2bits.CHPS = 0b01;// convert CH0 and CH1
AD1CON2bits.SMPI = 0b0001;// number of DMA buffers -1
AD1CON2bits.BUFM = 0;	// filling buffer from start address
AD1CON2bits.ALTS = 0;	// sample A

AD1CON3bits.ADRC = 0;	// system clock
AD1CON3bits.SAMC=0b11111;// 31 Tad auto sample time
AD1CON3bits.ADCS = 63;	// Tad = 64 * Tcy

AD1CON4bits.DMABL=0b110;// 64 word DMA buffer for each analog input

AD1CHS123bits.CH123NB=0;// don't care -> sample B
AD1CHS123bits.CH123SB=0;// don't care -> sample B
AD1CHS123bits.CH123NA=0;// CH1,2,3 negative input = Vrefl
AD1CHS123bits.CH123SA=0;// CH1 = AN0, CH2=AN1, CH3=AN2

AD1CHS0bits.CH0NB = 0;	// don't care -> sample B
AD1CHS0bits.CH0SB = 0;	// don't care -> sample B
AD1CHS0bits.CH0NA = 0;	// CH0 neg -> Vrefl
AD1CHS0bits.CH0SA = 1;	// CH0 pos -> AN1

AD1PCFGL = 0xFFFF;		// set all Analog ports as digital
AD1PCFGLbits.PCFG0 = 0;	// AN0
AD1PCFGLbits.PCFG1 = 0;	// AN1

IFS0bits.AD1IF = 0; 	// Clear the A/D interrupt flag bit
IEC0bits.AD1IE   = 0;	// Do Not Enable A/D interrupt 
AD1CON1bits.ADON = 1;	// module on

/*.........................................................DMA A/D converter */

// ADC connected to DMA 7
DMA7CONbits.AMODE = 2; 	// Peripheral Indirect Addressing mode
DMA7CONbits.MODE = 0; 	// Continuous,
DMA7PAD = (volatile unsigned int)&ADC1BUF0;// Point DMA to ADC1BUF0
DMA7CNT = 127; // 64 DMA request
DMA7REQ = 13; // Select ADC1 as DMA Request source
DMA7STA = __builtin_dmaoffset(DmaAdc);
_DMA7IF = 0; //Clear the DMA interrupt flag bit
_DMA7IE = 1; //Set the DMA interrupt enable bit
DMA7CONbits.CHEN=1; // Enable DMA

/*.............................................................A/D converter */

/*---------------------------------------------------------------------------*/
/* PWM	[11]                                                                 */
/*---------------------------------------------------------------------------*/
// Holds the value to be loaded into dutycycle register
unsigned int period;
// Holds the value to be loaded into special event compare register
unsigned int sptime;
// Holds PWM configuration value
unsigned int config1;
// Holds the value be loaded into PWMCON1 register
unsigned int config2;
// Holds the value to config the special event trigger postscale and dutycycle
unsigned int config3;

// Config PWM
period = 2048;
// PWM F=19,340Hz counting UP 12bit resolution @ Fcy=39.628 MHz (osc 7.3728MHz)
// PWM F=19,522Hz counting UP 12bit resolution @ Fcy=40.000 MHz (osc 10MHz)
sptime = 0x0;
// 1:1 postscaler, 1:1 prescale, free running mode
// PWM time base ON, count up
config1 = 	PWM1_EN & PWM1_IDLE_CON & PWM1_OP_SCALE1 & PWM1_IPCLK_SCALE1 & 
			PWM1_MOD_FREE;
			
// PWMxH enabled in indipendent mode
// PWMxL used as a generic I/O to enable the H-bridge
// dsPICs with 3 pairs of PWM pins have one timer only (A)		
config2 = 	PWM1_MOD1_IND & PWM1_PDIS1L & PWM1_PEN1H &
			PWM1_MOD2_IND & PWM1_PDIS2L & PWM1_PEN2H &
			PWM1_PDIS3H & PWM1_PDIS3L;
			
config3 = 	PWM1_SEVOPS1 & PWM1_OSYNC_PWM & PWM1_UEN;
OpenMCPWM1(period, sptime, config1, config2, config3);

// dutycyclereg=1, dutycycle=50% (motore fermo in LAP mode , updatedisable=0
SetDCMCPWM1(1, 2048, 0);
SetDCMCPWM1(2, 2048, 0);

// configure PWM2 pins as a generic I/O
PWM2CON1bits.PEN1L = 0;
PWM2CON1bits.PEN1H = 0;
/*.......................................................................PWM */


/*---------------------------------------------------------------------------*/
/* QEI1	[4]                                                                  */
/*---------------------------------------------------------------------------*/
/* 
OpenQEI(QEI_MODE_x4_MATCH & QEI_INPUTS_NOSWAP & QEI_IDLE_STOP
	& QEI_NORMAL_IO & QEI_INDEX_RESET_DISABLE,
	QEI_QE_CLK_DIVIDE_1_128 & QEI_QE_OUT_ENABLE & POS_CNT_ERR_INT_DISABLE);
*/

QEI1CONbits.QEIM 	= 7;      //	QEI_MODE_x4_MATCH
QEI1CONbits.SWPAB 	= 0;    //	QEI_INPUTS_SWAP
QEI1CONbits.QEISIDL	= 1;    //	QEI_IDLE_STOP
QEI1CONbits.POSRES	= 0;    //	QEI_INDEX_RESET_DISABLE
QEI1CONbits.PCDOUT	= 0;    //	QEI_NORMAL_IO
QEI1CONbits.POSRES	= 0;    //	POS_CNT_ERR_INT_DISABLE

DFLT1CONbits.QECK	= 5;      //	QEI_QE_CLK_DIVIDE_1_64
DFLT1CONbits.QEOUT	= 1;    //	QEI_QE_OUT_ENABLE
				
MAX1CNT = 0xFFFF;
POS1CNT = 0;
/*.......................................................................QEI */

/*---------------------------------------------------------------------------*/
/* QEI2	[4]                                                                  */
/*---------------------------------------------------------------------------*/
/* 
OpenQEI(QEI_MODE_x4_MATCH & QEI_INPUTS_NOSWAP & QEI_IDLE_STOP
		& QEI_NORMAL_IO & QEI_INDEX_RESET_DISABLE,
		QEI_QE_CLK_DIVIDE_1_128 & QEI_QE_OUT_ENABLE & POS_CNT_ERR_INT_DISABLE);
*/

QEI2CONbits.QEIM 	= 7;      //	QEI_MODE_x4_MATCH
QEI2CONbits.SWPAB 	= 0;    //	QEI_INPUTS_SWAP
QEI2CONbits.QEISIDL	= 1;    //	QEI_IDLE_STOP
QEI2CONbits.POSRES	= 0;    //	QEI_INDEX_RESET_DISABLE
QEI2CONbits.PCDOUT	= 0;    //	QEI_NORMAL_IO
QEI2CONbits.POSRES	= 0;    //	POS_CNT_ERR_INT_DISABLE

DFLT2CONbits.QECK	= 5;      //	QEI_QE_CLK_DIVIDE_1_64
DFLT2CONbits.QEOUT	= 1;    //	QEI_QE_OUT_ENABLE
				
MAX2CNT = 0xFFFF;
POS2CNT = 0;
/*.......................................................................QEI */

/*---------------------------------------------------------------------------*/
/* Input Capture 1 [7]                                                       */
/*---------------------------------------------------------------------------*/
IC1CONbits.ICSIDL = 1;//	Stop in idle
IC1CONbits.ICTMR = 1;	//	Timer 2
IC1CONbits.ICI = 0;		//	Interrupt on every capture event
IC1CONbits.ICM = 1;		//	Capture mode every edge
/*.............................................................Input Capture */

/*---------------------------------------------------------------------------*/
/* Input Capture 2 [7]                                                       */
/*---------------------------------------------------------------------------*/
// IC_EVERY_EDGE & IC_INT_1CAPTURE & IC_IDLE_STOP & IC_TIMER2_SRC)
IC2CONbits.ICSIDL = 1;//	Stop in idle
IC2CONbits.ICTMR = 1;	//	Timer 2
IC2CONbits.ICI = 0;		//	Interrupt on every capture event
IC2CONbits.ICM = 1;		//	Capture mode every  edge
/*.............................................................Input Capture */

/*---------------------------------------------------------------------------*/
/* I2C                                                                       */
/*---------------------------------------------------------------------------*/

I2C1ADD = I2C_ADDR;   
_I2CEN = 1;           //Enable I2C on I/O pins
_A10M  = 0;           //7-bit addressing
_STREN = 1;           //Clock stretching enabled
I2C1TRN = 0;          // empty TX buffer
I2C1RCV = 0;          // empty RX buffer
_SCLREL = 1;          //release clock

I2C_POINTER_FLAG = 0;   // reset State 1B

/*.......................................................................I2C */


#ifndef TIMER_OFF
/*---------------------------------------------------------------------------*/
/* Timer 2	[12]                                                             */
/*---------------------------------------------------------------------------*/
#define TMR2_VALUE 0xFFFF
/*OpenTimer2(	T2_ON & 
			T2_GATE_OFF & 
			T2_PS_1_1 &
			T1_SYNC_EXT_OFF &
			T2_SOURCE_INT, 
			TMR2_VALUE);
*/
T2CONbits.TON = 0; 		// Disable Timer
T2CONbits.TCS = 0; 		// Select internal instruction cycle clock
T2CONbits.TGATE = 0; 	// Disable Gated Timer mode
T2CONbits.TCKPS = 0b00;	// Select 1:1 Prescaler
TMR2 = 0x00; 			// Clear timer register
PR2 = TMR2_VALUE; 		// Load the period value
/*...................................................................Timer 2 */


/*---------------------------------------------------------------------------*/
/* Timer 1	1ms [13]                                                         */
/*---------------------------------------------------------------------------*/
#ifdef CLOCK_FREQ_10
	#define TMR1_VALUE 40000
#else
	#define TMR1_VALUE 39628
#endif

OpenTimer1(	T1_ON & 
			T1_GATE_OFF & 
			T1_PS_1_1 & 
			T1_SYNC_EXT_OFF &
			T1_SOURCE_INT, 
			TMR1_VALUE);
#endif
/*................................................................ ..Timer 1 */
}


/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/* Interrupts setting                                                        */
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
void ISR_Settings(void)
{/**
*\brief It contains everything needed to initialize Interrupt Service Routines
*/

//-------PWM	[11]
ConfigIntMCPWM1(PWM1_INT_DIS);

//-------Input Capture 1 [7]
// ConfigIntCapture1(IC_INT_ON & IC_INT_PRIOR_5);
IEC0bits.IC1IE=1;
IPC0bits.IC1IP=5;
_IC1IF = 0; // interrupt flag reset

//-------Input Capture 2 [7]
// ConfigIntCapture2(IC_INT_ON & IC_INT_PRIOR_5);
IEC0bits.IC2IE=1;
IPC1bits.IC2IP=5;
_IC2IF = 0; // interrupt flag reset

//-------I2C
// Slave I2C. Priority set lower than IC to avoid loss of encoder pulses
IEC1bits.SI2C1IE=1;
IPC4bits.SI2C1IP=4;
_SI2C1IF = 0;     //Clear I2C slave Interrupt flag
				
#ifndef TIMER_OFF
	//-------Timer 2	[12]
	//	ConfigIntTimer2(T2_INT_PRIOR_4 & T2_INT_ON);
	IPC1bits.T2IP = 0x04; // Set Timer 2 Interrupt Priority Level
	_T2IF = 0;            // interrupt flag reset
	IEC0bits.T2IE = 1;    // Enable Timer2 interrupt
	T2CONbits.TON = 1;    // Start Timer

	//-------Timer 1	[13]
	ConfigIntTimer1(T1_INT_PRIOR_4 & T1_INT_ON);
	PR1 = TMR1_VALUE;
#endif
}

