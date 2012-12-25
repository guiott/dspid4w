/* ////////////////////////////////////////////////////////////////////////////
** It contains all functions related to communication
**
** Detailed description are on file "descrEng.txt" 
** numbers between brackets, eg.: [1] , are the references to the specific 
** decription into the file     
/////////////////////////////////////////////////////////////////////////////*/

// standard includes
#include "dsPID4W_common.h"
#include "com.h"

// DMA buffers
unsigned char UartTxBuff[MAX_TX_BUFF] __attribute__((space(dma),aligned(128)));
unsigned char Uart2TxBuff[MAX_TX2_BUFF] __attribute__((space(dma),aligned(128)));

void UsartSetting(void)
{/**
*\brief USART1 configuration

*\ref _6 "details [6]"
*/

float BaudRate;
float BRG;
/* 	Baud Rate = Fcy / ( 4 * (UxBRG + 1) ) with BRGH = 1
	value for the U1BRG register rounded to closest integer (+0.5)
*/
BaudRate = 640000; // desired baud rate
BRG = (FCY/(4*(BaudRate)))-0.5; 

/*...............................................................DMA UART TX */
//  Associate DMA Channel 6 with UART Tx
DMA6REQ = 0x000c;		// Select UART1 Transmitter
DMA6PAD = (volatile unsigned int) &U1TXREG;

DMA6CONbits.SIZE  = 1;	// Transfer bytes
DMA6CONbits.DIR   = 1;	// Transfer data from RAM to UART
DMA6CONbits.HALF  = 0;	// Interrupt when all of the data has been moved
DMA6CONbits.AMODE = 0;	// Register Indirect with Post-Increment
DMA6CONbits.MODE  = 1;	// One-Shot mode

//DMA6CNT = 31;			// # of DMA requests

// Associate one buffer with Channel 6 for one-shot operation
DMA6STA = __builtin_dmaoffset(UartTxBuff);

//	Enable DMA Interrupts
IFS4bits.DMA6IF  = 0;	// Clear DMA Interrupt Flag
IEC4bits.DMA6IE  = 1;	// Enable DMA interrupt
/*...............................................................DMA UART TX */

/*.....................................................................USART */
U1MODEbits.STSEL = 0;		// 1-stop bit
U1MODEbits.PDSEL = 0;		// No Parity, 8-data bits
U1MODEbits.ABAUD = 0;		// Autobaud Disabled
U1MODEbits.RTSMD = 1;		// No flow control
U1MODEbits.BRGH  = 1;		// Hi speed
U1BRG = BRG;				// BAUD Rate Setting

//  Configure UART for DMA transfers
U1STAbits.UTXISEL0 = 0;		// Interrupt after one Tx character is transmitted
U1STAbits.UTXISEL1 = 0;			                            
U1STAbits.URXISEL  = 0;		// Interrupt after one RX character is received

//  Enable UART1 Rx and Tx
U1MODEbits.UARTEN = 1;// Enable UART
U1STAbits.UTXEN 	= 1;// Enable UART Tx
IEC4bits.U1EIE 		= 0;
_U1RXIF				= 0;	// Reset RX interrupt flag
_U1RXIE				= 1;	// Enable RX interrupt

UartRxStatus = 0;
ChkSum=0;

/*.....................................................................USART */
}

void Usart2Setting(void)
{/**
*\brief USART2 configuration

*\ref _6z "details [6z]"
*/

float BaudRate2;
float BRG2;

/* 	Baud Rate = Fcy / ( 4 * (UxBRG + 1) ) with BRGH = 1
	value for the U2BRG register rounded to closest integer (+0.5)
*/
BaudRate2 = 57600; // desired baud rate
BRG2 = (FCY/(4*(BaudRate2)))-0.5; 

/*..............................................................DMA UART2 TX */
//  Associate DMA Channel 5 with UART2 Tx
DMA5REQ = 0x003f;		// Select UART2 Transmitter
DMA5PAD = (volatile unsigned int) &U2TXREG;

DMA5CONbits.SIZE  = 1;	// Transfer bytes
DMA5CONbits.DIR   = 1;	// Transfer data from RAM to UART
DMA5CONbits.HALF  = 0;	// Interrupt when all of the data has been moved
DMA5CONbits.AMODE = 0;	// Register Indirect with Post-Increment
DMA5CONbits.MODE  = 1;	// One-Shot mode

//DMA5CNT = 31;			// # of DMA requests

// Associate one buffer with Channel 5 for one-shot operation
DMA5STA = __builtin_dmaoffset(Uart2TxBuff);

//	Enable DMA Interrupts
IFS3bits.DMA5IF  = 0;	// Clear DMA Interrupt Flag
IEC3bits.DMA5IE  = 0;	// Disable DMA interrupt because the port is used for I2C
/*..............................................................DMA UART2 TX */

/*....................................................................USART2 */
U2MODEbits.STSEL = 0;		// 1-stop bit
U2MODEbits.PDSEL = 0;		// No Parity, 8-data bits
U2MODEbits.ABAUD = 0;		// Autobaud Disabled
U2MODEbits.RTSMD = 1;		// No flow control
U2MODEbits.BRGH  = 1;		// Hi speed
U2BRG = BRG2;				// BAUD Rate Setting

//  Configure UART2 for DMA transfers
U2STAbits.UTXISEL0 = 0;		// Interrupt after one Tx character is transmitted
U2STAbits.UTXISEL1 = 0;			                            
U2STAbits.URXISEL  = 0;		// Interrupt after one RX character is received

//  Disable UART Rx and Tx if the same porte are used for I2C
U2MODEbits.UARTEN = 0;	// Disable UART
U2STAbits.UTXEN 	= 0;	// Disable UART Tx
IEC4bits.U2EIE 		= 0;
_U2RXIF				= 0;	// Reset RX interrupt flag
_U2RXIE				= 0;	// Disable RX interrupt

Uart2RxStatus = 0;
ChkSum2=0;

/*.....................................................................USART */
}

void TxParameters(char TxCmd, int TxCmdLen, int Port)
{/**
*\brief Send the requested parameters, with the correct handshake

*\param TxCmd char Command to send
*\param TxCmdLen int Number of bytes to send
  * (Data are stored in UartTmpBuff buffer)
*\param Port int Port number to use for sending

*\ref _18 "details [18]"
*/

	int TxCount;
	unsigned int UartTxBuffSize=0;
	
	if (Port==0)
	{
		UartTxBuff[0] = '@';		// Header
		UartTxBuff[1] = Id;			// Id
		UartTxBuff[2] = TxCmd;		// Cmd
		UartTxBuff[3] = TxCmdLen +1;	// CmdLen
		for (TxCount = 0; TxCount < TxCmdLen; TxCount ++)
		{
			UartTxBuff[TxCount + 4] = UartTmpBuff[TxCount][Port];
		}
		UartTxBuff[TxCount + 4] = UartChkSum(UartTxBuff,TxCount + 4);
		UartTxBuffSize = TxCount + 5;
		
		DMA6CNT = UartTxBuffSize-1;	// # of DMA requests
		DMA6CONbits.CHEN  = 1;		// Re-enable DMA Channel
		DMA6REQbits.FORCE = 1;	// Manual mode: Kick-start the first transfer
	}
	else
	{
		Uart2TxBuff[0] = '@';		// Header
		Uart2TxBuff[1] = Id;		// Id
		Uart2TxBuff[2] = TxCmd;		// Cmd
		Uart2TxBuff[3] = TxCmdLen +1;	// CmdLen
		for (TxCount = 0; TxCount < TxCmdLen; TxCount ++)
		{
			Uart2TxBuff[TxCount + 4] = UartTmpBuff[TxCount][Port];
		}
		Uart2TxBuff[TxCount + 4] = UartChkSum(Uart2TxBuff,TxCount + 4);
		UartTxBuffSize = TxCount + 5;
		
		DMA5CNT = UartTxBuffSize-1;	// # of DMA requests
		DMA5CONbits.CHEN  = 1;		// Re-enable DMA Channel
		DMA5REQbits.FORCE = 1;	// Manual mode: Kick-start the first transfer
	}
}

void UartRx(void)
{/**
*\brief U1ARX RX serial communication

*\ref _6b "details [6b]"
*/

	if (UartRxStatus < 0)	// if error, RX is terminated
	{
		UartRxError(UartRxStatus,0);		   	
	}
	else					// 	otherwise it analyzes status 
	{					
		switch (UartRxStatus)
		{     
            case 0:	// idle
   	    		if (UartRxBuff[UartRxPtrOut][0] == HEADER)
   	    		{
	   	    		UartRxStatus = 1;	// next status
	   	    		UartRxPtrStart = UartRxPtrOut;
	   	    		ChkSum = ChkSum + UartRxBuff[UartRxPtrOut][0];
					UartRxPtrOut ++;	// next byte
	   	    	}
	   	    	else					// out of command sequence -> error
	   	    	{
					UartRxError(-5,0);		   	
				}
			break;
		
			case 1:	// header received
   	    		if(UartRxBuff[UartRxPtrOut][0]==Id||UartRxBuff[UartRxPtrOut][0]==0)
   	    		{// command addressed to this board or broadcast
					// Id received and recognized
   	     	 		RX_ID_FLAG = 1;			// it has to decode the command
   	     	 		UartRxStatus = 2;		// next status 
	   	   		}
	   	   		else if (UartRxBuff[UartRxPtrOut][0] == 'z') // 
	   	      	{
   	      			UartRxStatus=99;	// enables command parser 
   	     	 		UartRxCmd[0]=UartRxBuff[UartRxPtrOut][0];
   	      		}
				else
	   	   		{
					// Id received but NOT recognized 
   	     	 		RX_ID_FLAG = 0;	// receives but does not decode the command
   	     	 	   	UartRxStatus = 2;		// next status 
		   		}
   	     	 	ChkSum = ChkSum + UartRxBuff[UartRxPtrOut][0];
				UartRxPtrOut ++;		// next byte
			break;
		
			case 2:	// command received
   	     	 	UartRxStatus = 3;		// next status
   	     	 	UartRxCmd[0]=UartRxBuff[UartRxPtrOut][0];
   	     	 	ChkSum = ChkSum + UartRxBuff[UartRxPtrOut][0];
   	     	 	UartRxPtrOut ++;		// next byte
			break;
		
			case 3:	// command length received
				// calculating end pointer of command string
				UartRxPtrEnd=UartRxBuff[UartRxPtrOut][0]+RX_HEADER_LEN+
					UartRxPtrStart;
				if (UartRxPtrEnd >= MAX_RX_BUFF)UartRxPtrEnd -= MAX_RX_BUFF;
				if (UartRxPtrEnd >= MAX_RX_BUFF)	// still bigger?
				{
					UartRxError(-11,0);	// queue overflow	
				}
				else
				{
   	     	 		UartRxStatus = 4;		// next status
   	     	 		ChkSum = ChkSum + UartRxBuff[UartRxPtrOut][0];
   	     	 		UartRxPtrData=UartRxPtrOut;// pointer to last header byte
   	     	 		UartRxPtrOut ++;		// next byte
   	     	 	}
			break;
			
			case 4:	// waiting for command end
   	      		if (UartRxPtrOut == UartRxPtrEnd)
   	      		{	
	   	      		if (ChkSum == UartRxBuff[UartRxPtrOut][0])// checksum OK?		   	    
	   	      		{
	   	      			if (RX_ID_FLAG)	// if right Id
	   	      			{
		   	      			UartRxStatus=99;// enables command parser 
							UartRxPtrOut ++;	// next byte
		   	      		}
		   	      		else
		   	      		{
			   	      		UartRxStatus=0;	// end of command receive 
							UartRxPtrOut ++;	// next byte
			   	      	}
		   	      	}
		   	      	else
		   	      	{
			   	    	UartRxError(-1,0); // checksum error	
			   	    }
			   	  	ChkSum=0;
   	      		}
   	      		else
   	      		{
   	      			ChkSum = ChkSum + UartRxBuff[UartRxPtrOut][0];
					UartRxPtrOut ++;	// next byte
			   	}
			break;
		
			default:// error: not a known status
				UartRxError(-6,0);				   	
			break;
		} // switch end
	}	// if end
	
	if (UartRxPtrOut >= MAX_RX_BUFF) UartRxPtrOut=0;//reset circular queue [6d]
}

void Uart2Rx(void)
{/**
*\brief U2ARX RX serial communication

*\ref _6b "details [6b]"
*\ref _6z "details [6z]"
*/

	if (Uart2RxStatus < 0)	// if error, RX is terminated
	{
		UartRxError(Uart2RxStatus,1);		   	
	}
	else					// 	otherwise it analyzes status 
	{					
		switch (Uart2RxStatus)
		{     
   			case 0:	// idle
   	    		if (UartRxBuff[Uart2RxPtrOut][1] == HEADER)
   	    		{
	   	    		Uart2RxStatus = 1;	// next status
	   	    		Uart2RxPtrStart = Uart2RxPtrOut;
	   	    		ChkSum2 = ChkSum2 + UartRxBuff[Uart2RxPtrOut][1];
					Uart2RxPtrOut ++;	// next byte
	   	    	}
	   	    	else					// out of command sequence -> error
	   	    	{
					UartRxError(-105,1);		   	
				}
			break;
		
			case 1:	// header received
   	    		if(UartRxBuff[Uart2RxPtrOut][1]==Id ||
   	    			UartRxBuff[Uart2RxPtrOut][1]==0)
   	    		{// command addressed to this board or broadcast
					// Id received and recognized
   	     	 		RX2_ID_FLAG = 1;			// it has to decode the command
   	     	 		Uart2RxStatus = 2;		// next status 
	   	   		}
	   	   		else if (UartRxBuff[Uart2RxPtrOut][1] == 'z') // 
	   	      	{
   	      			Uart2RxStatus=99;	// enables command parser 
   	     	 		UartRxCmd[1]=UartRxBuff[Uart2RxPtrOut][1];
   	      		}
				else
	   	   		{
					// Id received but NOT recognized 
   	     	 		RX2_ID_FLAG = 0;	// receives but does not decode the command
   	     	 	   	Uart2RxStatus = 2;		// next status 
		   		}
   	     	 	ChkSum2 = ChkSum2 + UartRxBuff[Uart2RxPtrOut][1];
				Uart2RxPtrOut ++;		// next byte
			break;
		
			case 2:	// command received
   	     	 	Uart2RxStatus = 3;		// next status
   	     	 	UartRxCmd[1]=UartRxBuff[Uart2RxPtrOut][1];
   	     	 	ChkSum2 = ChkSum2 + UartRxBuff[Uart2RxPtrOut][1];
   	     	 	Uart2RxPtrOut ++;		// next byte
			break;
		
			case 3:	// command length received
				// calculating end pointer of command string
				Uart2RxPtrEnd=UartRxBuff[Uart2RxPtrOut][1]+RX_HEADER_LEN+
					Uart2RxPtrStart;
				if (Uart2RxPtrEnd >= MAX_RX_BUFF)Uart2RxPtrEnd -= MAX_RX_BUFF;
				if (Uart2RxPtrEnd >= MAX_RX_BUFF)	// still bigger?
				{
					UartRxError(-111,1);	// queue overflow	
				}
				else
				{
   	     	 		Uart2RxStatus = 4;		// next status
   	     	 		ChkSum2 = ChkSum2 + UartRxBuff[Uart2RxPtrOut][1];
   	     	 		Uart2RxPtrData=Uart2RxPtrOut;// pointer to last header byte
   	     	 		Uart2RxPtrOut ++;		// next byte
   	    }
			break;
			
			case 4:	// waiting for command end
   	      		if (Uart2RxPtrOut == Uart2RxPtrEnd)
   	      		{	
	   	      		if (ChkSum2 == UartRxBuff[Uart2RxPtrOut][1])// checksum OK?		   	    
	   	      		{
	   	      			if (RX2_ID_FLAG)	// if right Id
	   	      			{
		   	      			Uart2RxStatus=99;// enables command parser 
							Uart2RxPtrOut ++;	// next byte
		   	      		}
		   	      		else
		   	      		{
			   	      		Uart2RxStatus=0;	// end of command receive 
							Uart2RxPtrOut ++;	// next byte
			   	      	}
		   	      	}
		   	      	else
		   	      	{
			   	    	UartRxError(-101,1); // checksum error	
			   	    }
			   	  	ChkSum2=0;
   	      		}
   	      		else
   	      		{
   	      			ChkSum2 = ChkSum2 + UartRxBuff[Uart2RxPtrOut][1];
                Uart2RxPtrOut ++;	// next byte
              }
			break;
		
			default:// error: not a known status
				UartRxError(-106,1);				   	
			break;
		} // switch end
	}	// if end
	
	//reset circular queue [6zd]
	if (Uart2RxPtrOut >= MAX_RX_BUFF) Uart2RxPtrOut=0;
}

unsigned char UartChkSum (unsigned char *Buff, unsigned int BuffSize)
{/**
*\brief CheckSum calculation

*\param Buff unsigned char* buffer cointaining data to send
*\param BuffSize unsigned int buffer size
*\return unsigned char UartChkSum checksum
	
*\ref _17 "details [17]"
*/

	unsigned char ChkSum=0;	// checksum
	int ChkIndx;
	for (ChkIndx = 0; ChkIndx < BuffSize; ChkIndx ++)
	{
		ChkSum = ChkSum + Buff[ChkIndx];
	}
   	
	return (ChkSum);      	
}	
	 	   	      	
void UartRxError(int Err, int Port)
{/**
*\brief RX error occured: changes LED1 blinking frequency

*\param Err int error number
*\param Port int port number
	
*\ref _6c "details [6c]"
*/

	BlinkPeriod = ERR_BLINK_PER;// LED1 blinking period (ms)
	BlinkOn     = ERR_BLINK_ON;	// LED1 on time (ms)
	Blink = 0;
	
	if (Port==0)
	{
		UartRxPtrOut=0;				// flush circular queue
		UartRxPtrIn=0;
		UartRxPtrEnd=0;
		UartRxStatus=0; 
		ChkSum=0;
		while (U1STAbits.URXDA)		// flush USART RX buffer
		{
			ReadUART1();
		}
	}
	else
	{
		Uart2RxPtrOut=0;				// flush circular queue
		Uart2RxPtrIn=0;
		Uart2RxPtrEnd=0;
		Uart2RxStatus=0; 
		ChkSum2=0;
		while (U2STAbits.URXDA)		// flush USART RX buffer
		{
			ReadUART2();
		}
	}
	
	// **debug**
	ErrNo[abs(Err)]++;			// error log
	// **debug**
	ErrCode=Err;				// store the last Error Code
}

unsigned char IncrCircPtr(int Port)
{/**
*\brief The bytes are exchanged between ISR and UartRx function through a 
	circular buffer

*\param Port int port number
*\return unsigned char queue pointer
	
*\ref _6d "details [6d]"
*/

	if (Port==0)
	{
		TmpPtr ++;								// next byte
		if (TmpPtr >= MAX_RX_BUFF) TmpPtr=0;	//reset circular queue
		return TmpPtr;
	}
	else
	{
		TmpPtr2 ++;								// next byte
		if (TmpPtr2 >= MAX_RX_BUFF) TmpPtr2=0;	//reset circular queue
		return TmpPtr2;
	}
}

/*---------------------------------------------------------------------------*/
/* Interrupt Service Routines                                                */
/*---------------------------------------------------------------------------*/

void _ISR_PSV _U1RXInterrupt(void)
{/**
*\brief U1ARX RX serial communication

*\ref _6b "details [6b]"
*/

	_U1RXIF = 0; 	// interrupt flag reset
	ClrWdt();		// [1]
	// UART errors or still parsing command ?
	if (!OVERRUN_ERROR && !FRAME_ERROR && UartRxStatus!=99)	
	{
		UartRxBuff[UartRxPtrIn][0] = ReadUART1();	// fills RX queue
		UartRxPtrIn ++;							// next byte
		if (UartRxPtrIn>=MAX_RX_BUFF) UartRxPtrIn=0;//reset circ queue[6d]
	}
	else
	{
		// indicates the error kind [6b]
		if (OVERRUN_ERROR)
		{
			UartRxStatus = -4;
			OVERRUN_ERROR = 0;	// error reset & flush RX buffer
		}
		if (FRAME_ERROR) UartRxStatus = -3;
		if (UartRxStatus==99) UartRxStatus = -9;
		UartRxPtrIn ++;	// start RX routine to analyze error
		if (UartRxPtrIn>=MAX_RX_BUFF) UartRxPtrIn=0;//reset circ queue[6d]
	}
}

void _ISR_PSV _U2RXInterrupt(void)
{/**
*\brief U2ARX RX serial communication

*\ref _6b "details [6b]"
*\ref _6z "details [6z]"
*/

	_U2RXIF = 0; 	// interrupt flag reset
	ClrWdt();		// [1]
	// UART2 errors or still parsing command ?
	if (!OVERRUN_ERROR2 && !FRAME_ERROR2 && Uart2RxStatus!=99)	
	{
		UartRxBuff[Uart2RxPtrIn][1] = ReadUART2();	// fills RX queue
		Uart2RxPtrIn ++;							// next byte
		if (Uart2RxPtrIn>=MAX_RX_BUFF) Uart2RxPtrIn=0;//reset circ queue[6zd]
	}
	else
	{
		// indicates the error kind [6zb]
		if (OVERRUN_ERROR2)
		{
			Uart2RxStatus = -4;
			OVERRUN_ERROR2 = 0;	// error reset & flush RX buffer
		}
		if (FRAME_ERROR2) Uart2RxStatus = -3;
		if (Uart2RxStatus==99) Uart2RxStatus = -9;
		Uart2RxPtrIn ++;	// start RX routine to analyze error
		if (Uart2RxPtrIn>=MAX_RX_BUFF) Uart2RxPtrIn=0;//reset circ queue[6zd]
	}
}
