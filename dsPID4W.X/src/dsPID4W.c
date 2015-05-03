/* ////////////////////////////////////////////////////////////////////////////
 ** File:      dsPid4W.c
 */
unsigned char Ver[] = "dsPid4W 1.1.0 Guiott 11-12"; // 26+1 char
/**
 * \mainpage dsPid4W.c
 * \author    Guido Ottaviani-->guido@guiott.com<--
 * \version 1.1.0
 * \date 11/12
 * \details This is a porting on a single dsPIC33FJ64MC802 of previous
 **       double PID Motor Control (dsPID) formerly performed
 **				with two dsPIC30F4012 plus odometry and field mapping formerly
 **				performed with a single dsPIC30F3013 (dsODO program).
 **
 **        This further version is for a 4W drive Robot
 **          (http://www.guiott.com/Lino/)
 **        where the four wheels are driven with two dsNav boards a master
 **        one, that receives commands a data from an IMU, and a slave that
 **        only maintains the rear wheels at the same speed of the front one
 **
 ** Detailed descriptions are on file "descrEng.txt"
 ** numbers between brackets, eg.: [1], are the references
 ** to the specific description into the file
 **
-------------------------------------------------------------------------------
\copyright 2012 Guido Ottaviani
guido@guiott.com

  dsPid4W is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    dsPid4W is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with dsPid4W.  If not, see <http://www.gnu.org/licenses/>.
    
-------------------------------------------------------------------------------      
 */

#include "dsPID4W_definitions.h"

int main(void)
{
/**
\details
The program is full interrupt driven. At the startup the program enters in a
 * very simple main-loop, acting as a state machine. In the main-loop, the
 * program checks flags enabled by external events, and it enters in the
 * relative state.

Since it's a kind of a very simple cooperative "Real Time Operating System,"
 * each routine has to be executed in the shortest possible time, freeing the
 * system up to take care of the very frequent interrupts.

There are no "wait until" and no delays in the code. Whenever possible
 * interrupts are used, in particular for slow operations like transmission or
 * reception of strings of characters.
 */

 // <editor-fold defaultstate="collapsed" desc="Settings">

 I2CRxBuff.I.VelDesM = 0x7FFF; //it means both wheels stop whatever orientation is
 Settings();

//[1]
LED1 = 1;
LED2 = 0;
MOTOR_ENABLE_R = 0;
MOTOR_ENABLE_L = 0;

Tmr2OvflwCount1 = 0;
Tmr2OvflwCount2 = 0;

PID_REFR = 0;
PID_REFL = 0;

VelDecr = 1;

// to start first and second slower cycles over 1ms cycle
Cycle1 = 0;
Cycle2 = 0;
CYCLE1_FLAG = 0;
CYCLE2_FLAG = 0;

/*Delay needed to ensure the program memory is not modified before
  verification in programming procedure
  http://forum.microchip.com/tm.aspx?m=326442 */
DelayN1ms(30);

BlinkPeriod = NORM_BLINK_PER; // LED1 blinking period (ms)
BlinkOn = NORM_BLINK_ON; // LED1 on time (ms)

ConstantsDefault(); // set constant values

InitPidR();
InitPidL();

ANGLE_PID_DES = 0;
InitAnglePid();

CONSOLE_DEBUG = 0;

ADC_CALC_FLAG = 0;
IdleSample = 0;
IdleCount = 0;

UsartSetting(); // initialize Usart1
Usart2Setting(); // initialize Usart2

ResetCount = 0;

ISR_Settings(); // Configures and enables ISRs

/* Send string 'Ver'
for (i = 0; i < 26; i++)
{
    UartTxBuff[i] = Ver[i];
}
DMA6CNT = 25; // # of DMA requests
DMA6CONbits.CHEN = 1; // Re-enable DMA Channel
DMA6REQbits.FORCE = 1; // Manual mode: Kick-start the first transfer
*/

// by default the board is configures as a slave, waiting for commands
I2CRxBuff.I.MasterFlag = 0;

// end settings
// </editor-fold>

/*===========================================================================*/
/* Main loop                                                                 */
/*===========================================================================*/

while (1) // start of idle cycle [25a]
{
// TEST ^= 1; // used for debug, toggle pin 16 alternatively with I2C


  //      ClrWdt();		// [1] ============debug


/* ----------------------------------------- one character coming from UART1 */
if (UartRxPtrIn != UartRxPtrOut) UartRx(); // [6d]

/* ----------------------------------------- one character coming from UART2 */
if (Uart2RxPtrIn != Uart2RxPtrOut) Uart2Rx(); // [6zd]

/* ------------------------ a command is coming from serial interface 1 or 2 */
if ((UartRxStatus == 99) || (Uart2RxStatus == 99)) Parser();

/* ------------------------------------------------------- DeadReckonig [22] */
if (CYCLE1_FLAG && I2CRxBuff.I.MasterFlag) DeadReckoning();

/* ----------------------------------------------------------------- Cycle 2 */
if (CYCLE2_FLAG) SlowCycleOp();

/*----------------------------------------- ADC value average calculus  [2a] */
if (ADC_CALC_FLAG) AdcCalc();

/*--------------------------------------------------- Heartbeat led blinking */
if (Blink == BlinkOn)
{
    LED1 = 0;
    LED2 = 1;
    Blink++;
}

if (Blink >= BlinkPeriod)
{
    LED1 = 1;
    LED2 = 0;
    Blink = 0;
}

/*------------------------------------------- Idle percentage calculus  [25] */
IdleCount++; // [25]
if (IdleSample >= IDLE_CYCLE)
{
    IdlePerc = (long) (IdleCount * IDLE_TIME_PERIOD)/(long) (IDLE_SAMPLE_TIME);
    IdleCount = 0;
    IdleSample = 0;
}

Nop(); // end of idle cycle [25a]

}/*....Main loop*/

}/*.....................................................................Main */


/*===========================================================================*/
/* Functions                                                                 */
/*===========================================================================*/

void DeadReckoning(void)
{/**
*\brief Odometry & dead reckoning

The coordinates of the current position of the robot is achieved with an
algorithm elaborated starting from G.W. Lucas' paper "A Tutorial and Elementary
Trajectory Model for the Differential Steering System of Robot Wheel Actuators"
available on Internet:
 http://rossum.sourceforge.net/papers/DiffSteer/DiffSteer.html

*\ref _22 "details [22]"
*/

    float WheelDTheta;      // delta rotation angle measured by odometry
    float ImuDTheta;        // delta rotation angle measured by IMU
    float SaMinusSb;
    float SrPlusSl;
    float ImuThetaMes = 0;  // measured by IMU rad
                        // global float ThetaMes, current correct measure (rad)
    
    CYCLE1_FLAG = 0;

    Spmm[R] = SpTick[R] * Ksp[R]; // distance of right wheel in mm
    SpTick[R] = 0; // reset counter for the next misure
    Spmm[L] = SpTick[L] * Ksp[L]; // distance of left wheel in mm
    SpTick[L] = 0; // reset counter for the next misure

    #ifdef geographic			// [22aa]
        SaMinusSb = Spmm[L] - Spmm[R];
    #else
        SaMinusSb = Spmm[R] - Spmm[L];
    #endif

    SrPlusSl = Spmm[R] + Spmm[L];

    if(!I2CRxBuff.I.NewFlag)
    {/*if no data from IMU it computes orientation by odometry this also allows
      * a different update rate in IMU and dsNav, currently 100Hz for dsNav,
      * 40Hz for IMU
     */
        if (fabs(SaMinusSb) <= SPMIN)
        {// traveling in a nearly straight line
            WheelDTheta = DeadReckCalc(0, SaMinusSb, SrPlusSl, ImuThetaMes);
        }
        else if (fabs(SrPlusSl) <= SPMIN)
        {// pivoting around vertical axis without translation [22a]
            WheelDTheta = DeadReckCalc(1, SaMinusSb, SrPlusSl, ImuThetaMes);
        }
        else
        {// rounding a curve
            WheelDTheta = DeadReckCalc(2, SaMinusSb, SrPlusSl, ImuThetaMes);
        }
    }
    else
    {//IMU data available, use yaw
        I2CRxBuff.I.NewFlag = 0;            // data read, wait for next packet
        ImuThetaMes = (float)I2CRxBuff.I.ImuThetaAbs * DEG2RAD10; // excluded YawOffset for possible error

        // calculate rotation rate-of-change for stasis detector
        ImuDTheta = ImuThetaMes - ThetaMes; // rotation rate from IMU
        // compute also ThetaMes and position
        WheelDTheta = DeadReckCalc(3, SaMinusSb, SrPlusSl, ImuThetaMes);
 
        Stasis(fabs(WheelDTheta),fabs(ImuDTheta));
    }

    if(I2CRxBuff.I.OrientFlag)
    {
        Orientation(); // position coordinates computed, Angle PID can start [23]
    }
    else
    {
        DirectDrive(); // to directly control the direction via remote joystick
    }
}

void Stasis(float wheel_drv, float imu_drv)
{
/*from David Paul Anderson jBot site:
The variable stasis_err is incremented whenever stasis conditions are met and
decremented to 0 when they are not met. This integration acts as a simple low
pass filter that prevents the  escape() behavior from triggering too often, as
when the robot is stuck only momentarily and then wiggles free, but also insures
a trigger when the robot is bouncing around such that sometimes the wheel theta
and IMU theta agree, but the robot actually is trapped. When the  stasis_err
exceeds a global limit set by the user, the escape alarm is triggered. That
limit is currently set such that the robot normally has about 2 seconds to free
itself before the alarm is activated.
*/

        if ((imu_drv < NOT_TURNING) && (wheel_drv > ARE_TURNING))
        {
            I2CTxBuff.I.stasis_err++;
        } else
        {
            if (I2CTxBuff.I.stasis_err) I2CTxBuff.I.stasis_err--;
            else I2CTxBuff.I.stasis_alarm = 0;
        }

        if (I2CTxBuff.I.stasis_err > STASIS_ERR)
        {
            I2CTxBuff.I.stasis_alarm = 1;
            I2CTxBuff.I.stasis_err = 0;
        }

}

float DeadReckCalc (char Mode, float SaMinusSb, float SrPlusSl, float ImuThetaMes)
{
	static float CosPrev =	1;	// previous value for Cos(ThetaMes)
	static float SinPrev =	0;	// previous value for Sin(ThetaMes)
	float DSpace; 				// delta traveled distance by the robot
	float CosNow; 				// current value for Cos
  float SinNow; 				// current value for Sin
  float DPosX; 				// delta space on X axis
  float DPosY; 				// delta space on Y axis
  float Radius;
  float WheelDTheta;      // delta rotation angle measured by odometry

  switch (Mode)
	{
		case 0: // traveling in a nearly straight line
            WheelDTheta = 0;
            DSpace = SrPlusSl / 2;

            #ifdef geographic
                DPosX = DSpace*SinPrev;
                DPosY = DSpace*CosPrev;
            #else
                DPosX = DSpace*CosPrev;
                DPosY = DSpace*SinPrev;
            #endif
            break;

		case 1: // pivoting around vertical axis without translation
            WheelDTheta = SaMinusSb / Axle;
            DSpace = 0;
            ThetaMes = fmodf((ThetaMes + WheelDTheta), TWOPI);//current orient. 2PI range
            CosPrev = cosf(ThetaMes); // for the next cycle
            SinPrev = sinf(ThetaMes);
            DPosX = 0;
            DPosY = 0;
            break;

        default:// rounding a curve
            WheelDTheta = SaMinusSb / Axle;
            DSpace = SrPlusSl / 2;
            if(Mode == 3)
            {// use IMU
  	          ThetaMes = ImuThetaMes;
  			}
  			else
  			{// use odometry
  	          ThetaMes = fmodf((ThetaMes + WheelDTheta), TWOPI);//current orient .2PI range
  			}
            CosNow = cosf(ThetaMes);
            SinNow = sinf(ThetaMes);
            Radius = (SemiAxle)*(SrPlusSl / SaMinusSb);

            #ifdef geographic
                DPosX = Radius * (CosPrev - CosNow);
                DPosY = Radius * (SinNow - SinPrev);
            #else
                DPosX = Radius * (SinNow - SinPrev);
                DPosY = Radius * (CosPrev - CosNow);
            #endif

            CosPrev = CosNow; // to avoid re-calculation on the next cycle
            SinPrev = SinNow;
        	break;
    }

    Space += DSpace; // total traveled distance
    I2CTxBuff.I.PosXmes += DPosX;       // current position
    I2CTxBuff.I.PosYmes += DPosY;

    return WheelDTheta;
}

void Orientation(void)
{/**
*\brief Angle PID.
PID procedure to maintain the desired orientation.

*\ref _23 "details [23]"
*/

    int DeltaVel; // difference in speed between the wheels to rotate
    int RealVel;  // VelDesM after reduction controlled by Dist PID
    float Error;

    ThetaDes = I2CRxBuff.I.ThetaDes * DEG2RAD10;

    if (ThetaDes < 0) ThetaDes += TWOPI;	// keep angle value positive
    if (ThetaMes < 0) ThetaMes += TWOPI;
    Error = ThetaMes - ThetaDes;
    if (Error > PI)	// search for the best direction to correct error [23a]
    {
      Error -= TWOPI;
    }
    else if (Error < -PI)
    {
      Error += TWOPI;
    }

    ANGLE_PID_MES = 0; // des value translated to 0 [23c]
    ANGLE_PID_DES = Q15(Error / PI); // current error [23b]
    PID(&AnglePIDstruct);

    if(I2CRxBuff.I.VelDesM != 0x7FFF)
    {// OXFFFF means both wheels stop whatever orentation is set
        DeltaVel = (ANGLE_PID_OUT >> 7); // MAX delta in int = 256 [23d]
        RealVel = I2CRxBuff.I.VelDesM * VelDecr; // [24d]

        VelDes[R] = RealVel - DeltaVel; // [23e]
        VelDes[L] = RealVel + DeltaVel;
    }
    else
    {
        VelDes[R] = 0;
        VelDes[L] = 0;
    }

    Speed();      // set the rotation speed for each wheel
    SpeedSlave(); // Set the rotation speed for slave board
}

void DirectDrive(void)
{/**
*\brief Direct driving by joystick.
*/

    int DeltaVel; // difference in speed between the wheels to rotate
    int RealVel;  // VelDesM after reduction controlled by Dist PID

    if(I2CRxBuff.I.VelDesM != 0x7FFF)
    {// OXFFFF means both wheels stop whatever orentation is set
        DeltaVel = I2CRxBuff.I.ThetaDes;
        RealVel = I2CRxBuff.I.VelDesM * VelDecr; // [24d]

        VelDes[R] = RealVel - DeltaVel; // [23e]
        VelDes[L] = RealVel + DeltaVel;
    }
    else
    {
        VelDes[R] = 0;
        VelDes[L] = 0;
    }

    Speed();      // set the rotation speed for each wheel
    SpeedSlave(); // Set the rotation speed for slave board
}

void SpeedSlave(void)
{/**
*\brief Send the desired speed for each wheel to maintain the rear wheels at
  * the same speed of the front ones
*/
    static int SpeedSlaveCount;

    TmpBufIndx=0; // reset buffer index
    WriteIntBuff(PidRef[R]);
    WriteIntBuff(PidRef[L]);
    if(SpeedSlaveCount<10) // every 10 cycles send a parameters request too
    {
        TxParameters('V', TmpBufIndx, 0);
        SpeedSlaveCount++;
    }
    else
    {
        TxParameters('W', TmpBufIndx, 0);
        SpeedSlaveCount=0;
    }
}

void Speed(void)
{/**
*\brief Manage the acc/deceleration ramp and speed control to start Speed PID.
*/

    /*
      if (VelDes[R] > MAX_ROT_SPEED)
      {
        VelDes[R] = MAX_ROT_SPEED;
        VelDes[L] = MAX_ROT_SPEED + (DeltaVel << 1);
      }
      else if (VelDes[R] < -MAX_ROT_SPEED)
      {
        VelDes[R] = -MAX_ROT_SPEED;
        VelDes[L] = -MAX_ROT_SPEED + (DeltaVel << 1);
      }
      else if (VelDes[L] > MAX_ROT_SPEED)
      {
        VelDes[L] = MAX_ROT_SPEED;
        VelDes[R] = MAX_ROT_SPEED - (DeltaVel << 1);
      }
      else if (VelDes[L] < -MAX_ROT_SPEED)
      {
        VelDes[L] = -MAX_ROT_SPEED;
        VelDes[R] = -MAX_ROT_SPEED - (DeltaVel << 1);
      }
    */

    if(I2CRxBuff.I.MasterFlag) //if slave the ramp is controlled by master board
    {
        // <editor-fold defaultstate="collapsed" desc="Manage ramp">
        VelFin[R] = Q15((float) (VelDes[R]) / 2000); //normalized to 2m/s[23f]
        VelFin[L] = Q15((float) (VelDes[L]) / 2000); //normalized to 2m/s[23f]

        if (VelFin[R] != PidRef[R])
        {
            if (VelFin[R] > PidRef[R])
            {
              if(PidRef[R] >= 0)
              {
                PidRef[R] += Acc;
              }
              else
              {
                PidRef[R] += Dec;
              }

              if (PidRef[R] >= VelFin[R])
              {
                PidRef[R] = VelFin[R];// acceleration is over
              }
            }
            else
            {
              if (PidRef[R] >= 0)
              {
                PidRef[R] -= Dec;
              }
              else
              {
                PidRef[R] -= Acc;
              }

              if (PidRef[R] <= VelFin[R])
              {
                PidRef[R] = VelFin[R];// acceleration is over
              }
            }
        }

        if (VelFin[L] != PidRef[L])
        {
            if (VelFin[L] > PidRef[L])
            {
              if(PidRef[L] >= 0)
              {
                PidRef[L] += Acc;
              }
              else
              {
                PidRef[L] += Dec;
              }

              if (PidRef[L] >= VelFin[L])
              {
                PidRef[L] = VelFin[L];// acceleration is over
              }
            }
            else
            {
              if (PidRef[L] >= 0)
              {
                PidRef[L] -= Dec;
              }
              else
              {
                PidRef[L] -= Acc;
              }

              if (PidRef[L] <= VelFin[L])
              {
                PidRef[L] = VelFin[L];// acceleration is over
              }
            }
        }

    // end Manage ramp
    // </editor-fold>
    }
    
    SelectIcPrescaler();
}

void SelectIcPrescaler(void)
{/**
*\brief Select the correct Input Capture prescaler

*/
    switch (IC1CONbits.ICM)
    {
        case IC_MODE0:
            if (Vel[R] >= MAX1)
            {
                SwitchIcPrescaler(1,R);
            }
            break;

        case IC_MODE1:
            if (Vel[R] < MIN1)
            {
                SwitchIcPrescaler(0,R);
            }
            else if (Vel[R] >= MAX2)
            {
                SwitchIcPrescaler(2,R);
            }
            break;

        case IC_MODE2:
            if (Vel[R] < MIN2)
            {
                SwitchIcPrescaler(1,R);
            }
            else if (Vel[R] >= MAX3)
            {
                SwitchIcPrescaler(3,R);
            }
            break;

        case IC_MODE3:
            if (Vel[R] < MIN3)
            {
                SwitchIcPrescaler(2,R);
            }
            break;

        default:
                SwitchIcPrescaler(0,R);
            break;
    }

    switch (IC2CONbits.ICM)
    {
        case IC_MODE0:
            if (Vel[L] >= MAX1)
            {
                SwitchIcPrescaler(1,L);
            }
            break;

        case IC_MODE1:
            if (Vel[L] < MIN1)
            {
                SwitchIcPrescaler(0,L);
            }
            else if (Vel[L] >= MAX2)
            {
                SwitchIcPrescaler(2,L);
            }
            break;

        case IC_MODE2:
            if (Vel[L] < MIN2)
            {
                SwitchIcPrescaler(1,L);
            }
            else if (Vel[L] >= MAX3)
            {
                SwitchIcPrescaler(3,L);
            }
            break;

        case IC_MODE3:
            if (Vel[L] < MIN3)
            {
                SwitchIcPrescaler(2,L);
            }
            break;

        default:
                SwitchIcPrescaler(0,L);
            break;
    }
}

void SwitchIcPrescaler(int Mode, int RL)
{/**
*\brief Safely switch to the new Input Capture prescaler

*/
    __builtin_disi(0x3FFF); //disable interrupts up to priority 6 for n cycles

// here is the assignment of the ICx module to the correct wheel
    if (RL == R)
    {
        IC1CONbits.ICM = 0; // turn off prescaler
        Ic1Indx = 0; // reset IC1 interrupt count
        Ic1CurrPeriod = 0;
        IC1CONbits.ICM = IcMode[Mode];
        _IC1IF = 0;  // interrupt flag reset
    }
    else
    {
        IC2CONbits.ICM = 0; // turn off prescaler
        Ic2Indx = 0; // reset IC2 interrupt count
        Ic2CurrPeriod = 0;
        IC2CONbits.ICM = IcMode[Mode];
        _IC2IF = 0;  // interrupt flag reset
    }

    Kvel[RL] = KvelMode[Mode][RL];

    DISICNT = 0; //re-enable interrupts
}

void Parser(void)
{/** 
*\brief Command Parser from commands coming from both UARTs

*\ref _16 "details [16]"
*/
    int Ptmp;    // temp for position values
    int CurrTmp; // temp for motors current value
    int Alpha;   // rotation angle in degrees

    if (UartRxStatus == 99) // the command come from UART1
    {
        TmpPtr = UartRxPtrData; // last data of header
        UartRxStatus = 0;
        Port = 0;
    }
    else // the command come from UART2
    {
        TmpPtr2 = Uart2RxPtrData; // last data of header
        Uart2RxStatus = 0;
        Port = 1;
    }


    if (UartRxCmd[ResetPort] != '*') // [28]
    {
        ResetCount = 0;
    }

    switch (UartRxCmd[Port])
    {
    // --- Navigation values read
        case 'A': // all parameters request (mean)
            // VelMes = Int -> 2 byte (mm/s)
            //			Ptmp = (VelMes[R] + VelMes[L]) >> 1;	// average speed
            Ptmp=(I2CTxBuff.I.VelInt[R]+I2CTxBuff.I.VelInt[L])>>1;//avrg. speed
            if (CONSOLE_DEBUG) //[30]
            {
                Ptmp = I2CRxBuff.I.VelDesM;
                I2CTxBuff.I.ADCValue[R] = abs(I2CRxBuff.I.VelDesM);
                I2CTxBuff.I.ADCValue[L] = abs(I2CRxBuff.I.VelDesM);
            }
            TmpBufIndx=0; // reset buffer index
            WriteIntBuff(Ptmp);
            // Curr = int -> 2byte (mA)
            CurrTmp=I2CTxBuff.I.ADCValue[R]+I2CTxBuff.I.ADCValue[L];//totalCurr.
            WriteIntBuff(CurrTmp);
            
            TxParameters('a', TmpBufIndx, Port);
            break;

        case 'B': // all parameters request (details)
            TmpBufIndx=0; // reset buffer index
            WriteIntBuff(I2CTxBuff.I.VelInt[R]);
            WriteIntBuff(I2CTxBuff.I.VelInt[L]);

            // ADCValue = int -> 2byte
            WriteIntBuff(I2CTxBuff.I.ADCValue[R]); // CurrR
            WriteIntBuff(I2CTxBuff.I.ADCValue[L]); // CurrL

            TxParameters('b', TmpBufIndx, Port);
            break;

        case 'b':
            // High Byte * 256 + Low Byte
            // rear wheels detailed parameters received from slave board:
            // VelRightSlave, VelLeftSlave, CurrentRightSlave, CurrentLeftSlave
            I2CTxBuff.I.VelInt[RS] = ReadIntBuff();
            I2CTxBuff.I.VelInt[LS] = ReadIntBuff();
            I2CTxBuff.I.ADCValue[RS] = ReadIntBuff();
            I2CTxBuff.I.ADCValue[LS] = ReadIntBuff();
            break;

        case 'C':
            // High Byte * 256 + Low Byte
            // four wheels detailed param.request for master board:
            // VrM, CrM, VlM, ClM, VrS, CrS, VlS, ClS

            TmpBufIndx=0; // reset buffer index
            WriteIntBuff(I2CTxBuff.I.VelInt[R]);
            WriteIntBuff(I2CTxBuff.I.VelInt[L]);
            WriteIntBuff(I2CTxBuff.I.VelInt[RS]);
            WriteIntBuff(I2CTxBuff.I.VelInt[LS]);
            // ADCValue = int -> 2byte
            WriteIntBuff(I2CTxBuff.I.ADCValue[R]); // CurrR
            WriteIntBuff(I2CTxBuff.I.ADCValue[L]); // CurrL
            WriteIntBuff(I2CTxBuff.I.ADCValue[RS]); // CurrR slave
            WriteIntBuff(I2CTxBuff.I.ADCValue[LS]); // CurrL slave

            TxParameters('c', TmpBufIndx, Port);
            break;
            

    //--- Navigation parameters settings

        case 'F': // setting ref. orientation angle in degrees (absolute)
            // [24] Mode A
            // High Byte * 256 + Low Byte
            I2CRxBuff.I.ThetaDes = ReadIntBuff(); // 0 - 3599
            VelDecr = 1; // [24d]
            if (CONSOLE_DEBUG) //[30]
            {
                ThetaMes = I2CRxBuff.I.ThetaDes;
            }
            I2CRxBuff.I.MasterFlag = 1; // master mode
            break;

        case 'G': // setting reference orientation angle in degrees
            // as a delta of the current orientation (relative)
            // [24] Mode A
            // High Byte * 256 + Low Byte
            I2CRxBuff.I.ThetaDes += ReadIntBuff();
            VelDecr = 1; // [24d]
            I2CRxBuff.I.MasterFlag = 1; // master mode
            break;

        case 'S': // setting reference speed for the vehicle (as mm/s)
            // High Byte * 256 + Low Byte
            I2CRxBuff.I.VelDesM = ReadIntBuff();
            // range check
            if (I2CRxBuff.I.VelDesM>MAX_SPEED) I2CRxBuff.I.VelDesM = MAX_SPEED;
            if (I2CRxBuff.I.VelDesM<-MAX_SPEED) I2CRxBuff.I.VelDesM= -MAX_SPEED;
            I2CRxBuff.I.MasterFlag = 1; // master mode
            break;

        case 'V': //Reference speed setting in mm/s for each wheel in slave mode
            // High Byte * 256 + Low Byte
            PidRef[R] = ReadIntBuff();
            PidRef[L] = ReadIntBuff();

            I2CRxBuff.I.MasterFlag = 0; // slave mode
            Speed();
            break;

        case 'W': //Ref. wheel speed set in slave mode with parameter request
            // High Byte * 256 + Low Byte
            PidRef[R] = ReadIntBuff();
            PidRef[L] = ReadIntBuff();

            I2CRxBuff.I.MasterFlag = 0; // slave mode
            Speed();

            TmpBufIndx=0; // reset buffer index
            WriteIntBuff(I2CTxBuff.I.VelInt[R]);
            WriteIntBuff(I2CTxBuff.I.VelInt[L]);

            // ADCValue = int -> 2byte
            WriteIntBuff(I2CTxBuff.I.ADCValue[R]); // CurrR
            WriteIntBuff(I2CTxBuff.I.ADCValue[L]); // CurrL

            TxParameters('b', TmpBufIndx, Port);
            break;

         case 'H': // immediate Halt without decelerating ramp.In this way it
            // uses the brake effect of H bridge in LAP mode
            I2CRxBuff.I.VelDesM = 0X7FFF;
            if (CONSOLE_DEBUG) //[30]
            {
                VelMes[R] = 0;
                VelMes[L] = 0;
                I2CTxBuff.I.ADCValue[R] = 0;
                I2CTxBuff.I.ADCValue[L] = 0;
            }
            break;

    //--- Service
        case '*': // Board reset [28]
            if (ResetCount < 3)
            {
                ResetCount++;
                ResetPort = Port;
            }
            else
            {
                SET_CPU_IPL(7); // disable all user interrupts
                DelayN1ms(200);
                asm("RESET");
            }
            break;

        case 'E': // Firmware version request
            // Send string 'Ver'
            for (i = 0; i < 27; i++)
            {
                UartTmpBuff[i][Port] = Ver[i];
            }
            TxParameters('R', 26, Port);
            break;

        case 'Z': // send back a text string, just for debug
            // Send string 'Test'
            for (i = 0; i < 27; i++)
            {
                UartTmpBuff[i][Port] = Test[i];
            }
            TxParameters('z', 24, Port);
            break;

        case 'D': // Read error code and reset error condition
            // Send error value
            BlinkPeriod = NORM_BLINK_PER; // LED1 blinking period (ms)
            BlinkOn = NORM_BLINK_ON; // LED1 on time (ms)
            Blink = 0;
            UartTmpBuff[0][Port] = ErrCode >> 8;
            UartTmpBuff[1][Port] = ErrCode;
            TxParameters('d', 2, Port);
            break;

        case 'I': // set "Console Debug" mode [30]
            CONSOLE_DEBUG = UartRxBuff[IncrCircPtr(Port)][Port];
            break;


        default:
            UartRxError(-7, Port); //	error: not a known command
            break;
    }
}

void AdcCalc(void)
{/**
*\brief Motor current reading through Rsense on H bridge

*\ref _2 "details [2]"
*/
    extern int DmaAdc[2][64];
    int AdcCount = 0;
    long ADCValueTmp[2] = {0, 0}; // to store intermediate ADC calculations

    ADC_CALC_FLAG = 0; // will be set by ISR when all data available
    // averages the 64 ADC values for each ANx
    for (AdcCount = 0; AdcCount < 64; AdcCount++)
    {
        ADCValueTmp[R] += DmaAdc[R][AdcCount];
        ADCValueTmp[L] += DmaAdc[L][AdcCount];
    }
    I2CTxBuff.I.ADCValue[R] = ADCValueTmp[R] >> 6; // [2a]
    I2CTxBuff.I.ADCValue[L] = ADCValueTmp[L] >> 6; // [2a]

    if (I2CTxBuff.I.ADCValue[R] > ADC_OVLD_LIMIT) // [2b]
    {
        ADCOvldCount[R]++;
    }
    else
    {
        ADCOvldCount[R] = 0;
    }

    if (I2CTxBuff.I.ADCValue[L] > ADC_OVLD_LIMIT) // [2b]
    {
        ADCOvldCount[L]++;
    }
    else
    {
        ADCOvldCount[L] = 0;
    }

    if ((ADCOvldCount[R] > ADC_OVLD_TIME) || (ADCOvldCount[L] > ADC_OVLD_TIME))
    {
        ADCOvldCount[R] = 0;
        ADCOvldCount[R] = 0;
        I2CRxBuff.I.VelDesM = 0x7FFF; // immediate halt
        BlinkOn = 50; // very fast blink for alarm
        BlinkPeriod = 100;
        ErrCode = -30;
    }
}

/*---------------------------------------------------------------------------*/
/* Initialization Routines                                                   */
/*---------------------------------------------------------------------------*/

void ConstantsDefault(void)
{/**
*\brief Set default constant parameters in case no other value provided

*\ref _29 "details [29]"
*/

ANGLE_KP = Q15(0.9999);
ANGLE_KI = Q15(0.7);
ANGLE_KD = Q15(0.0001);

/*
KP1=Q15(0.9);
KI1=Q15(0.7);
KD1=Q15(0.1);
KP2=Q15(0.9);
KI2=Q15(0.7);
KD2=Q15(0.1);
 */

KPR = Q15(0.9);
KIR = Q15(0.9);
KDR = Q15(0.001);
KPL = Q15(0.9);
KIL = Q15(0.9);
KDL = Q15(0.001);

// constants K_VEL << 15 [19c]
KvelMode[0][R] = (((DIAMETER_R * PI) / (CPR * GEAR_RATIO)) * FCY)*32768 / 2; //X2   right
KvelMode[0][L] = (((DIAMETER_L * PI) / (CPR * GEAR_RATIO)) * FCY)*32768 / 2; //X2   left
KvelMode[1][R] = (((DIAMETER_R * PI) / (CPR * GEAR_RATIO)) * FCY)*32768; //X1   right
KvelMode[1][L] = (((DIAMETER_L * PI) / (CPR * GEAR_RATIO)) * FCY)*32768; //X1   left
KvelMode[2][R] = (((DIAMETER_R * PI) / (CPR * GEAR_RATIO)) * FCY)*32768 * 4; //X1/4 right
KvelMode[2][L] = (((DIAMETER_L * PI) / (CPR * GEAR_RATIO)) * FCY)*32768 * 4; //X1/4 left
KvelMode[3][R] = (((DIAMETER_R * PI) / (CPR * GEAR_RATIO)) * FCY)*32768 * 16; //X1/4 right
KvelMode[3][L] = (((DIAMETER_L * PI) / (CPR * GEAR_RATIO)) * FCY)*32768 * 16; //X1/4 left

Kvel[R] = KvelMode[0][R];
Kvel[L] = KvelMode[0][L];

Ksp[R] = 0.005065008;
Ksp[L] = 0.0050520;
// Axle = 184.8728; // mm Rino Robot
Axle = 510.0;   // mm Lino Robot

SemiAxle = Axle / 2;
}

void InitAnglePid(void)
{/**
*\brief Initialize the PID data structure: PIDstruct.
Use the Microchip C30 PID library according to the Microchip Code Example CE019
		
*\ref _19d "details [19d]"
*/

    //Initialize the PID data structure: PIDstruct
    //Set up pointer to derived coefficients
    AnglePIDstruct.abcCoefficients = &AngleabcCoefficient[0];
    //Set up pointer to controller history samples
    AnglePIDstruct.controlHistory = &AnglecontrolHistory[0];
    // Clear the controler history and the controller output
    PIDInit(&AnglePIDstruct);
    //Derive the a,b, & c coefficients from the Kp, Ki & Kd
    PIDCoeffCalc(&AngleKCoeffs[0], &AnglePIDstruct);
}

void InitPidR(void)
{ /**
*\brief Initialize the PID data structure: PIDstruct.
Use the Microchip C30 PID library according to the Microchip Code Example CE019
		
*\ref _19d "details [19d]"
*/

    //Set up pointer to derived coefficients
    PIDstructR.abcCoefficients = &abcCoefficientR[0];
    //Set up pointer to controller history samples
    PIDstructR.controlHistory = &controlHistoryR[0];
    // Clear the controler history and the controller output
    PIDInit(&PIDstructR);
    //Derive the a,b, & c coefficients from the Kp, Ki & Kd
    PIDCoeffCalc(&kCoeffs[R][0], &PIDstructR);
}

void InitPidL(void)
{/**
*\brief Initialize the PID data structure: PIDstruct.
Use the Microchip C30 PID library according to the Microchip Code Example CE019
		
*\ref _19d "details [19d]"
*/

    //Set up pointer to derived coefficients
    PIDstructL.abcCoefficients = &abcCoefficientL[0];
    //Set up pointer to controller history samples
    PIDstructL.controlHistory = &controlHistoryL[0];
    // Clear the controler history and the controller output
    PIDInit(&PIDstructL);
    //Derive the a,b, & c coefficients from the Kp, Ki & Kd
    PIDCoeffCalc(&kCoeffs[L][0], &PIDstructL);
}

/*---------------------------------------------------------------------------*/
/* Service Routines                                                          */
/*---------------------------------------------------------------------------*/
void SlowCycleOp(void)
{/**
*\brief Execute the operations at a Cycle2 timing
*/
    CYCLE2_FLAG = 0;
      
    //convert the Q15 Vel values to the equivalent int values
    I2CTxBuff.I.VelInt[R] = (long) (Vel[R] * 1000) >> 15;
    I2CTxBuff.I.VelInt[L] = (long) (Vel[L] * 1000) >> 15;
}

void WriteIntBuff(int TxBuffValue)
{/**
*\brief Fill an integer (2 bytes) to Uart TX Buffer
  * keep the buffer index updated to the next free
*/

    UartTmpBuff[TmpBufIndx][Port] = TxBuffValue >> 8;
    TmpBufIndx ++;
    UartTmpBuff[TmpBufIndx][Port] = TxBuffValue;
    TmpBufIndx ++;
}

int ReadIntBuff(void)
{/**
*\brief Return an integer (2 bytes) from Uart RX Buffer
*/
    return (UartRxBuff[IncrCircPtr(Port)][Port] << 8) +
              (UartRxBuff[IncrCircPtr(Port)][Port]);
}

void DelayN1ms(int n)
{
    int ms;
    for (ms = 0; ms < n; ms++)
    {
        DelayN10us(100);
    }
}

void DelayN10us(int n) // [22]
{
    int DelayCount;
    for (DelayCount = 0; DelayCount < (57 * n); DelayCount++);
}

/*---------------------------------------------------------------------------*/
/* Interrupt Service Routines                                                */
/*---------------------------------------------------------------------------*/
#ifndef TIMER_OFF

void _ISR_PSV _T1Interrupt(void)
{/**
*\brief 1 ms timer

*\ref _13 "details [13]"
*/
    
    long IcTmpR, IcTmpL;
    int IcIndxTmpR, IcIndxTmpL;
    int PWM_R, PWM_L;

    _T1IF = 0; // interrupt flag reset
    Blink++; // heartbeat LED blink
    
// cycle 0 actions
    // <editor-fold defaultstate="collapsed" desc="Speed and PID procedures">
    /**
     *\brief Use C30 PID libraries according to the Microchip Code Example 
        CE019 in order to keep one wheel at a constant desired speed
        If a less resolution encoder is used the PID parameters must change
        below a certain speed in order to mantain stability
     *\ref _19 "details [19]"
     */

    SpTick[R] += (int) POS1CNT; // cast to signed to store direction [19]
    POS1CNT = 0;
    SpTick[L] += (int) POS2CNT; // cast to signed to store direction [19]
    POS2CNT = 0;

    //if slow speed the PID is computed every PID_CYCLE_MULT ms
    if (labs(Vel[R])<VEL_MIN_PID)
    {
        PidCycle[R]++;
        if (PidCycle[R] >= PID_CYCLE_MULT)
        {
            PidCycle[R] = 0;
        }
        else
        {
            goto PidLCalc;
        }
    }

    __builtin_disi(0x3FFF); //disable interrupts up to priority 6 for n cycles
    IcTmpR = Kvel[R] / Ic1Period; // [19a]
    IcIndxTmpR = Ic1Indx;
    Ic1Indx = 0;
    Ic1Period = 0;
    DISICNT = 0; //re-enable interrupts

    if (IcIndxTmpR) // motor is running [19c]
    {
        Vel[R] = IcTmpR*IcIndxTmpR;
    }
    else
    {
        Vel[R] = 0;
    }

    // PID computation
    PID_MESR = Vel[R] >> 1; // speed in m/s normalized to 2m/s
    PID_REFR = PidRef[R];
    PID(&PIDstructR);
    PWM_R = (PID_OUTR >> 4) + 2047; // [19e]

    SetDCMCPWM1(2, PWM_R, 0);

    MOTOR_ENABLE_R = 1; // [1]

PidLCalc:
    //if slow speed the PID is computed every PID_CYCLE_MULT ms
    if (labs(Vel[L])<VEL_MIN_PID)
    {
        PidCycle[L]++;
        if (PidCycle[L] >= PID_CYCLE_MULT)
        {
            PidCycle[L] = 0;
        }
        else
        {
            goto EndPidCalc;
        }
    }

    __builtin_disi(0x3FFF); //disable interrupts up to priority 6 for n cycles
    IcTmpL = Kvel[L] / Ic2Period; // [19a]
    IcIndxTmpL = Ic2Indx;
    Ic2Indx = 0;
    Ic2Period = 0;
    DISICNT = 0; //re-enable interrupts

    if (IcIndxTmpL) // motor is running [19c]
    {
        Vel[L] = IcTmpL*IcIndxTmpL;
    }
    else
    {
        Vel[L] = 0;
    }

    PID_MESL = Vel[L] >> 1; // speed in m/s normalized to 2m/s
    PID_REFL = PidRef[L];
    PID(&PIDstructL);
    PWM_L = (PID_OUTL >> 4) + 2047; // [19e]

    SetDCMCPWM1(1, PWM_L, 0);
       
    MOTOR_ENABLE_L = 1; // [1]

EndPidCalc:
    Nop();
//========================================= Speed & PID calc END
// </editor-fold>


    Cycle1++;
    // cycle 1 actions
    if (Cycle1 >= CICLE1_TMO)
    {
        Cycle1 = 0;
        CYCLE1_FLAG = 1; // it's time to start first cycle actions [23]

        Cycle2++;
        // cycle 2 actions
        if (Cycle2 >= CICLE2_TMO)
        {
            Cycle2 = 0;
            CYCLE2_FLAG = 1; // it's time to start second cycle actions [24]
            IdleSample++; // [25]
        }
    }
}

void _ISR_PSV _T2Interrupt(void)
{/**
*\brief Timer2, used by Input Capture to measure speed

*\ref _12 "details [12]"
*/
    _T2IF = 0; // interrupt flag reset
    Tmr2OvflwCount1++; // TMR2 overflow as occurred
    Tmr2OvflwCount2++; // TMR2 overflow as occurred
    /* Just for debug
    if((Tmr2OvflwCount1>1) || (Tmr2OvflwCount2>1))
    {
        Nop();
    }
     */
}

#endif

void _ISR_PSV _DMA7Interrupt(void)
{/**
*\brief DMA used for ADC

*\ref _2 "details [2]"
*/
    _DMA7IF = 0; // interrupt flag reset
    ADC_CALC_FLAG = 1; // enable ADC average calculus
}

void _ISR_PSV _DMA6Interrupt(void)
{/**
*\brief DMA for UART1 TX 

*\ref _6d "details [6d]"
*/

    _DMA6IF = 0; // interrupt flag reset
}

void _ISR_PSV _DMA5Interrupt(void)
{/**
*\brief DMA for UART2 TX 

*\ref _6d "details [6d]"
*\ref _6z "details [6z]"
*/

    _DMA5IF = 0; // interrupt flag reset
}

void _ISR_PSV _IC1Interrupt(void)
{/**
*\brief ISR for Input Capture 1

*\ref _7 "details [7]"
*/

    _IC1IF = 0; // interrupt flag reset
    Ic1CurrPeriod = IC1BUF;
    if (Tmr2OvflwCount1 == 0) // TMR2 overflowed?
    {// see Microchip AN545
        Ic1Period += (Ic1CurrPeriod - Ic1PrevPeriod);
    }
    else
    {// [7a]
        Ic1Period += (Ic1CurrPeriod + (0xFFFF - Ic1PrevPeriod)
          +(0xFFFF * (Tmr2OvflwCount1 - 1)));

        Tmr2OvflwCount1 = 0;
    }
    Ic1PrevPeriod = Ic1CurrPeriod;

    if (QEI1CONbits.UPDN) // [7b]
    {
        Ic1Indx++;
    }
    else
    {
        Ic1Indx--;
    }

}

void _ISR_PSV _IC2Interrupt(void)
{/**
*\brief ISR for Input Capture 2

*\ref _7 "details [7]"
*/

    _IC2IF = 0; // interrupt flag reset
    Ic2CurrPeriod = IC2BUF;
    if (Tmr2OvflwCount2 == 0)
    {
        Ic2Period += (Ic2CurrPeriod - Ic2PrevPeriod);
    }
    else
    {// [7a]
        Ic2Period += (Ic2CurrPeriod + (0xFFFF - Ic2PrevPeriod)
          +(0xFFFF * (Tmr2OvflwCount2 - 1)));

        Tmr2OvflwCount2 = 0;
    }
    Ic2PrevPeriod = Ic2CurrPeriod;

    if (QEI2CONbits.UPDN) // [7b]
    {
        Ic2Indx++;
    }
    else
    {
        Ic2Indx--;
    }
}

void __attribute__((interrupt, no_auto_psv)) _U1ErrInterrupt(void)
{/**
*\brief Clear the UART1 Error Interrupt Flag
*/

    IFS4bits.U1EIF = 0;
}

void __attribute__((interrupt, no_auto_psv)) _U2ErrInterrupt(void)
{/**
*\brief Clear the UART2 Error Interrupt Flag
*/

    IFS4bits.U2EIF = 0;
}

void __attribute__((interrupt,no_auto_psv)) _SI2C1Interrupt(void)
{/**
*\brief Slave I2C
 *----This is a porting of assembly code for Microchip AN734-------------------
 * the original AN734 ASM code was written for a PIC18F series MCU. It was
 * first ported in C language: http://www.guiott.com/Lino/Display/DisplayI2C.htm
 * this a porting of the porting on a dsPIC33F DSC
 * over the I2C protocol it was added a 24Cxx EEPROM like data exchange mode.
 * The procedure is executed within an ISR in order to speed up
 * the response on I2C bus and avoid interruption, so an high care must be
 * taken on execution time and on external calls. If there are less strict
 * requirements the entire procedure can be executed in the main part after
 * the setting of a flag from the ISR.

/*         D/A -P- -S- R/W RBF TBF
 *
 *  0   0   0   0   1   0   1   0  State 1: I2C write operation,
 *                                          last byte was an address byte*/
#define STATE1 0X000A

/*  0   0   1   0   1   0   1   0  State 2: I2C write operation,
 *                                          last byte was a data byte*/
#define STATE2 0X002A

/*  0   0   0   0   1   1   X   0  State 3: I2C read operation,
 *                                          last byte was an address byte*/
#define STATE3 0X000C

/*  0   0   1   0   1   1   0   0  State 4: I2C read operation,
 *                                          last byte was a data byte*/
#define STATE4 0X002C

/*  0   0   1   0   1   X   0   0  State 5: Slave I2C logic reset
 *                       _SCLREL = 1        by NACK from master*/
#define STATE5 0X0028

#define STATE_MASK 0X002E // 00101110 to mask out unimportant bits in I2C1STAT
#define BF_MASK 0X002C    // 00101100 to mask out RBF bit in I2C1STAT
#define RW_MASK 0X0028    // 00101000 to mask out R/W bit in I2C1STAT
#define MAX_TRY 100       // maximum number of retries for transmission
//----------------------------------------------------------------------------*/
unsigned char SspstatMsk;
unsigned char I2cAddr; //to perform dummy reading of the SSPBUFF

 if (_SI2C1IF)   // I2C interrupt?
 {
    ClrWdt();		// [1]
    SspstatMsk=I2C1STAT & STATE_MASK; //mask out unimportant bits

//State 1------------------
    if(SspstatMsk == STATE1)
    {/* After State 1 the State Machine goes in State 1B, the next received
      * byte will be the register pointer*/
        I2C_POINTER_FLAG = 1;   // go into the State 1B
        I2cAddr = I2C1RCV;  //dummy reading of the buffer to empty it
        _SCLREL = 1; //release clock immediately to free up the bus
    }

//State 2------------------
    else if(SspstatMsk == STATE2)
    {
        if(I2C_POINTER_FLAG)
        {//come from State 1B, the received data is the register pointer
            I2cRegPtr = I2C1RCV;
            _SCLREL = 1; //release clock immediately to free up the bus
        }
        else
        {//the received data is a valid byte to store
            I2CRxTmpBuff.C[I2cRegPtr] = I2C1RCV;

            _SCLREL = 1; //release clock immediately to free up the bus
            I2cRegPtr ++;
        }

        if(I2cRegPtr >= I2C_BUFF_SIZE_RX)
        {
            for(i = 0; i < I2C_BUFF_SIZE_RX; i++)
            {
                I2CRxBuff.C[i]=I2CRxTmpBuff.C[i];//double buffering to avoid inconsistency
            }
            I2cRegPtr = I2C_BUFF_SIZE_RX - 1;
        }
        I2C_POINTER_FLAG = 0;
    }

//State 3------------------
    else if((SspstatMsk & BF_MASK) == STATE3)
    {//first reading from master after it sends address
        for(i = 0; i < I2C_BUFF_SIZE_TX; i++)
        {
            I2CTxTmpBuff.C[i]=I2CTxBuff.C[i];//double buffering to avoid inconsistency
        }

        I2cAddr = I2C1RCV;  //dummy reading of the buffer to empty it
        for(i=0; i<MAX_TRY; i++)
        {//check MAX_TRY times if buffer is empty
            if(!_TBF)
            {//if buffer is empty send a byte
                I2C1TRN = I2CTxTmpBuff.C[I2cRegPtr]; //send requested byte
                _SCLREL = 1; //free up the bus
                I2cRegPtr ++;
                if(I2cRegPtr >= I2C_BUFF_SIZE_TX)
                {
                    I2cRegPtr = I2C_BUFF_SIZE_TX - 1;
                }
                // insert here an OK flag if needed
                goto State3End; //everything's fine. Procedure over
           }
        }
        // insert here a fault flag if needed
        // if here the buffer was never empty
        Nop();

        State3End:
        I2C_POINTER_FLAG = 0;
    }

//State 4------------------
    else if((!_SCLREL) && (SspstatMsk == STATE4))
    {//subsequent readings. Usually it does the same as State 3

        for(i=0; i<MAX_TRY; i++)
        {//check MAX_TRY times if buffer is empty
            if(!_TBF)
            {//if buffer is empty send a byte
                I2C1TRN = I2CTxTmpBuff.C[I2cRegPtr]; //send requested byte
                _SCLREL = 1; //free up the bus
                I2cRegPtr ++;
                if(I2cRegPtr >= I2C_BUFF_SIZE_TX)
                {
                    I2cRegPtr = I2C_BUFF_SIZE_TX - 1;
                }
                // insert here an OK flag if needed
                goto State4End; //everything's fine. Procedure over
            }
        }
        // insert here a fault flag if needed
        // if here the buffer was never empty
        Nop();

        State4End:
        I2C_POINTER_FLAG = 0;
    }

//State 5------------------
    else if((SspstatMsk & RW_MASK) == STATE5)
    {//end cycle
        _SCLREL = 1; //release clock
        I2C_POINTER_FLAG = 0;
    }

//Default------------------
    else
    {//not in a known state. A different safety action could be performed here
        _SCLREL = 1; //release clock
        I2C_POINTER_FLAG = 0;
    }

    _SCLREL = 1; //release clock
    _SI2C1IF = 0;  //Clear MSSP Interrupt flag
}
}
/*****************************************************************************/
