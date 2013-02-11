/* ///////////////////////////////////////////////////////////////////////// */
/* Included	in "dsPid_definitions.h", it contains functions prototypes / */
/* ///////////////////////////////////////////////////////////////////////// */
float DeadReckCalc (char Mode, float SaMinusSb, float SrPlusSl, float ImuThetaMes);
void Stasis(float wheel_drv, float imu_drv);
void SpeedSlave(void);
void SlowCycleOp(void);
void WriteIntBuff(int TxBuffValue);
int ReadIntBuff(void);
void SelectIcPrescaler(void);
void SwitchIcPrescaler(int Mode, int RL);
void ConstantsDefault (void);
void PidCalc (void);
void PidLCalc (void);
void AdcCalc(void);
void DelayN1ms(int n);
void DelayN10us(int n);
void InitAnglePid(void);
void Orientation(void);
void DeadReckoning(void);
unsigned char IncrCircPtr(int Port);
void InitPidR(void);
void InitPidL(void);
void PidL(void);
void TxParameters(char TxCmd,int TxCmdLen, int Port);
void UartTx(void);
void UartRxError(int Err, int Port);
unsigned char UartChkSum (unsigned char *Buff, unsigned int BuffSize);
void Parser (void);
void UartRx(void);
void Uart2Rx(void);
void Settings(void);
void ISR_Settings(void);
void UsartSetting(void);
void Usart2Setting(void);
void Speed(void);
// void _ISR _INT1Interrupt(void);
// void _ISR_PSV _U1RXInterrupt(void);
// void _ISR _U1TXInterrupt(void);
// void _ISR_PSV _IC1Interrupt(void);
// void _ISR_PSV _T1Interrupt(void);
// void _ISR _CNInterrupt(void);

// void _ISR _ADCInterrupt(void);
// void _ISR _QEIInterrupt(void);

