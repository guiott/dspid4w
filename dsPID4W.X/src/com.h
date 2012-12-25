/* ////////////////////////////////////////////////////////////////////////////
** Included in "com.c", it contains definitions and variables initialization
/////////////////////////////////////////////////////////////////////////////*/

#define DISABLE_TX U1STAbits.UTXEN = 0
#define ENABLE_TX U1STAbits.UTXEN = 1
#define DISABLE_T2 U2STAbits.UTXEN = 0
#define ENABLE_TX2 U2STAbits.UTXEN = 1

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/* External defininitions                                                    */
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

extern volatile unsigned int BlinkPeriod ;
extern unsigned int BlinkOn;
extern long Blink;
extern int ErrCode;

#define ERR_BLINK_PER   200	// Period for ERR condition
#define ERR_BLINK_ON    100	// ON TIME for ERR condition

// **debug**
unsigned char ErrNo[16];	// store the occurrence of any kind of error 
// **debug**

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

struct Bits COMbits1;

//UART1
//{
unsigned char Id = 0;			// Motor Controller Id [14]
volatile unsigned char UartRxBuff[MAX_RX_BUFF][2];//serial communication buffer
unsigned char UartTxBuff[MAX_TX_BUFF] __attribute__((space(dma),aligned(128)));
unsigned char UartTmpBuff[MAX_TX_BUFF-4][2];// temp buffer to compose TX buffer 
unsigned char TxFlag = 0;		// transmission status
unsigned char ChkSum=0;			// checksum
unsigned char UartRxCmd[2];		// command code
// Rx circular queue pointers
volatile unsigned char UartRxPtrIn=0;	// ingoing bytes captured by ISR
unsigned char UartRxPtrOut=0;	// outgoing bytes read by UartRx function
unsigned char UartRxPtrStart=0;	// message packet starting pointer in queue
unsigned char UartRxPtrEnd=0;	// message packet ending pointer in queue
unsigned char UartRxPtrData=0;	// pointer to first data in queue
unsigned char TmpPtr;			// temporary pointer
unsigned int UartTxCntr=0;
volatile int UartRxStatus =0;			// index for command decoding status
#define OVERRUN_ERROR U1STAbits.OERR	// RX overrun error flag
#define TX_REG_EMPTY  U1STAbits.TRMT	// TX shift register empty
#define FRAME_ERROR U1STAbits.FERR		// RX frame error flag
#define HEADER '@'						// command string header
#define RX_ID_FLAG COMbits1.bit1		// command decode flag

const unsigned char Test[]="-mary had a little lamb-";
//}

//UART2
//{ 
unsigned char Id2 = 0;			// Motor Controller Id [14]
extern unsigned char 
		Uart2TxBuff[MAX_TX2_BUFF] __attribute__((space(dma),aligned(128)));
unsigned char Tx2Flag = 0;		// transmission status
unsigned char ChkSum2=0;		// checksum
// Rx circular queue pointers
volatile unsigned char Uart2RxPtrIn=0;	// ingoing bytes captured by ISR
unsigned char Uart2RxPtrOut=0;	// outgoing bytes read by UartRx function
unsigned char Uart2RxPtrStart=0;// message packet starting pointer in queue
unsigned char Uart2RxPtrEnd=0;	// message packet ending pointer in queue
unsigned char Uart2RxPtrData=0;	// pointer to first data in queue
unsigned char TmpPtr2;			// temporary pointer
unsigned int Uart2TxCntr=0;
#define UART2_CONT_TIMEOUT 100		// continuos parameters TX in ms(debug) 
int Uart2ContTxTimer=UART2_CONT_TIMEOUT;// timer for continuos parameters TX 
volatile int Uart2RxStatus =0;		// index for command decoding status
#define OVERRUN_ERROR2 U2STAbits.OERR	// RX overrun error flag
#define TX2_REG_EMPTY  U2STAbits.TRMT	// TX shift register empty
#define FRAME_ERROR2 U2STAbits.FERR	// RX frame error flag
#define RX2_ID_FLAG COMbits1.bit2	// command decode flag
unsigned int Tx2ContFlag=0;		// continuos parameters TX flag (debug)
//}

