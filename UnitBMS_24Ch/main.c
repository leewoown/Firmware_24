#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#pragma CODE_SECTION(cpu_timer0_isr,"ramfuncs");

//************************************************************************************//
//                                System variables                                    //
//************************************************************************************//
extern Uint16 RamfuncsLoadStart;
extern Uint16 RamfuncsLoadEnd;
extern Uint16 RamfuncsRunStart;

SystemReg           SysRegs;
CANRXReg            CANRXRegs;
CANTXReg            CANTXRegs;
ADCReg              ADCRegs;

SlaveReg            Slave1Regs;
SlaveReg            Slave2Regs;
SlaveReg            Slave3Regs;


struct  ECAN_REGS   ECanaShadow;
//************************************************************************************//
//                                Test variables                                      //
//************************************************************************************//


char *msg;

float32 CurrentTest = 0.0;
float32 CellTempFAN = 0.0;
float32 gainTest    = 200.0;
float32 TEMPTest    = 0.0 ;
Uint16 Cantestrx =0;
Uint16 CANARX_300=0;
Uint16 TempNum = 0;

unsigned long index;
unsigned long testindex=0;
unsigned long index2;
unsigned long testindex2=0;
unsigned long CANindex=240;

unsigned int cancnt=200;
unsigned int slave1voltagoffset =0;
unsigned int slave2voltagoffset =0;
void main(void)
{
// Step 1. Initialize System Control
// PLL, WatchDog, enable Peripheral Clocks
// This example function is found in the DSP2803x_SysCtrl.c file.
   	InitSysCtrl();
   
// Step 2. Initalize GPIO: 
// This example function is found in the DSP2803x_Gpio.c file and
// illustrates how to set the GPIO to it's default state.
// For this example use the following configuration:
// Step 3. Clear all interrupts and initialize PIE vector table:
   	DINT;
// Initialize PIE control registers to their default state.
// The default state is all PIE interrupts disabled and flags
// are cleared.  
// This function is found in the DSP2803x_PieCtrl.c file.
   	InitPieCtrl();
// Disable CPU interrupts and clear all CPU interrupt flags:
   	IER = 0x0000;
   	IFR = 0x0000;
// Initialize the PIE vector table with pointers to the shell Interrupt 
// Service Routines (ISR).  
// This will populate the entire table, even if the interrupt
// is not used in this example.  This is useful for debug purposes.
// The shell ISR routines are found in DSP2803x_DefaultIsr.c.
// This function is found in DSP2803x_PieVect.c.
// Step 4. USEGE Initialize all
// Step 5. enable interrupts:
	InitPieVectTable();

   	EALLOW;  // This is needed to write to EALLOW protected registers
   	PieVectTable.TINT0      = &cpu_timer0_isr;
	PieVectTable.ECAN0INTA 	= &ISR_CANRXINTA;
//	PieVectTable.SCIRXINTA 	= &ISR_SCIRXINTA;
  	EDIS;    // This is needed to disable write to EALLOW protected registers

	WG_InitGpio();
    WG_InitECana();
    WG_InitSPI();
   // WG_InitSci();
   	InitCpuTimers();
	MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);
	InitFlash();
	ConfigCpuTimer(&CpuTimer0, 60,1000);
	CpuTimer0Regs.TCR.all = 0x4001;

	InitAdc();
	IER |= M_INT1;
	IER |= M_INT9;	// CAN RX,SCIA_RX 
   	PieCtrlRegs.PIEIER1.bit.INTx7 = 1;		// Enable TINT0 in the PIE: Group 1 interrupt 7
	PieCtrlRegs.PIEIER9.bit.INTx5 = 1;		// Enable ECAN-A interrupt of PIE group 9
//	PieCtrlRegs.PIEIER9.bit.INTx1 = 1;		// SCIA RX interrupt of PIE group
	EINT;   // Enable Global interrupt INTM
   	ERTM;   // Enable Global realtime interrupt DBGM
   	SysRegs.SysVauleInit=0;
   	SysRegs.SysMachine = STATE_INIT_REG;
   	while(1)
	{
   	    SysRegs.MainIsr++;
   	    ModuleCurrent(&SysRegs);
        switch(SysRegs.SysMachine)
        {
            case STATE_INIT_REG:
                //----------Temp 초기화----------//
                TEMP1DO00L;
                TEMP1DO01L;
                TEMP1DO02L;
                TEMP2DO00L;
                TEMP2DO01L;
                TEMP2DO02L;
                TEMP3DO00L;
                TEMP3DO01L;
                TEMP3DO02L;
                SysRegs.BMSIDRegs.all=0;
                SystemRegsInit(&SysRegs);
                CANRxRegsInit(&CANRXRegs);
                CANTxRegsInit(&CANTXRegs);

                Slave1Regs.ID=C_Slave1_ID;
                SlaveRegsInit(&Slave1Regs);
                Slave2Regs.ID=C_Slave2_ID;
                SlaveRegsInit(&Slave2Regs);
                SlaveBMSIint(&Slave1Regs);
                SlaveBMSIint(&Slave2Regs);

                memset(&Slave1Regs.CellVoltage[0], 3500, 12);
                memset(&Slave2Regs.CellVoltage[0], 3500, 12);
                memset(&Slave1Regs.CellTemp[0], 250, 8);
                memset(&Slave2Regs.CellTemp[0], 250, 8);
                memset(&Slave3Regs.CellTemp[0], 250, 8);
                memset(&SysRegs.CellVoltage[0], 3500, 24);
                memset(&SysRegs.CellTemperture[0], 250,24);
                SysRegs.SysMachine = STATE_STANDBY;
                Slave1Regs.SysMachine =STATE_RUNNING;
                Slave2Regs.SysMachine =STATE_RUNNING;

            break;
            case STATE_STANDBY:
                 SysRegs.SysMachine = STATE_RUNNING;
                 SysRegs.SystemStateARegs.bit.INITOK =1;
                 SysRegs.DigitalOutPutReg.bit.FAN =1;
                 delay_ms(500);
            break;
            case STATE_RUNNING:

                memcpy(&SysRegs.CellVoltage[0], &Slave1Regs.CellVoltage[0],12);
                memcpy(&SysRegs.CellVoltage[12],&Slave2Regs.CellVoltage[0],12);
                //SysRegs.CellVoltage[22]=0; //22
                //SysRegs.CellVoltage[23]=0; //23
                SysRegs.CellVoltDivCntVaule=12;
                for(SysRegs.CellVoltDivCnt=0;SysRegs.CellVoltDivCnt < SysRegs.CellVoltDivCntVaule;SysRegs.CellVoltDivCnt++)
                {
                    Slave1Regs.CellVoltage[SysRegs.CellVoltDivCnt] = Slave1Regs.CellVoltageBuf[SysRegs.CellVoltDivCnt]/10;
                    Slave2Regs.CellVoltage[SysRegs.CellVoltDivCnt] = Slave2Regs.CellVoltageBuf[SysRegs.CellVoltDivCnt]/10;
                }
                memcpy(&SysRegs.CellTemperture[0],&Slave1Regs.CellTemp[0],8);
                memcpy(&SysRegs.CellTemperture[8],&Slave2Regs.CellTemp[0],8);
                memcpy(&SysRegs.CellTemperture[16],&Slave3Regs.CellTemp[0],8);
                //SysRegs.CellTemperture[22] = 0; //22
                //SysRegs.CellTemperture[23] = 0; //23

                ModuleCurrent(&SysRegs);
                SysRegs.CellVoltageCount=24;
                CalVoltageHandler(&SysRegs);
                SysRegs.CellTempertureCount=24;
                CalTempHandler(&SysRegs);
                memcpy(&CANTXRegs.BatteryVoltageCell[0],&SysRegs.CellVoltage[0],24);
                //CANTXRegs.BatteryVoltageCell[22]=0; //22
                //CANTXRegs.BatteryVoltageCell[23]=0; //23
                memcpy(&CANTXRegs.BatteryTempCell[0],&SysRegs.CellTemperture[0],24);
                //CANTXRegs.BatteryTempCell[22] = 0; //22
                //CANTXRegs.BatteryTempCell[23] = 0; //23
                SysRegs.SysMachine = STATE_RUNNING;
                if(CANRXRegs.PackStatus.bit.PackReset==1)
                {
                    SysRegs.SysMachine = STATE_INIT_REG;
                }
            break;
            case STATE_SALVEING:
                 //SysRegs.SysMachine = STATE_FAULT;
            break;
            case STATE_FAULT:
            break;
            case STATE_CLEAR:
            break;
            default :
            break;
        }
        if(SysRegs.CellVoltsampling>=100)
        {
           if(SysRegs.SystemStateARegs.bit.BalanceStartStop==0)
           {
             Slave1Regs.ID=C_Slave1_ID;
             SlaveVoltagHandler(&Slave1Regs);
             Slave2Regs.ID=C_Slave2_ID;
             SlaveVoltagHandler(&Slave2Regs);
           }
           SysRegs.CellVoltsampling=0;
        }
        if(SysRegs.CellTempSampling>1000)
        {
            CellTempsAdcHandler(1,&Slave1Regs);
            CellTempsAdcHandler(2,&Slave2Regs);
            CellTempsAdcHandler(3,&Slave3Regs);
            SysRegs.TempNumber++;
            if(SysRegs.TempNumber>=8)
            {
                SysRegs.TempNumber=0;
            }
            TEMP1_CAL(&Slave1Regs,SysRegs.TempNumber);
            TEMP2_CAL(&Slave2Regs,SysRegs.TempNumber);
            TEMP3_CAL(&Slave3Regs,SysRegs.TempNumber);
            SysRegs.CellTempSampling=0;
        }
        if(SysRegs.SystemStateARegs.bit.BalanceEnable == 1)
        {

           if (SysRegs.SystemStateARegs.bit.BalanceStartStop == 1)
           {
               Slave1Regs.BalaCellVolt=SysRegs.BalaCellMinVolt;
               BalanceCtrlHandler(&Slave1Regs);
               SlaveBmsBalance(&Slave1Regs);
               Slave2Regs.BalaCellVolt=SysRegs.BalaCellMinVolt;
               BalanceCtrlHandler(&Slave2Regs);
               SlaveBmsBalance(&Slave2Regs);
           }
           else
           {
               Slave1Regs.Balance.all = 0x0000;
               Slave2Regs.Balance.all = 0x0000;
               SlaveBmsBalance(&Slave1Regs);
               SlaveBmsBalance(&Slave2Regs);
           }
        }
        else
        {
            SysRegs.SystemStateARegs.bit.BalanceStartStop=0;
            Slave1Regs.Balance.all = 0x0000;
            Slave2Regs.Balance.all = 0x0000;
            SlaveBmsBalance(&Slave1Regs);
            SlaveBmsBalance(&Slave2Regs);
        }
    }
} //EOF void main()

interrupt void cpu_timer0_isr(void)   // 매 5ms마다 인터럽트 발생
{
    DigitalInput(&SysRegs);

//----------Timer값 증가----------//
    SysRegs.CellVoltsampling++;
    SysRegs.CellTempSampling++;
    if(CANRXRegs.LEDRXCount>=10)
      {
            LEDSTATE_Toggle;
            CANRXRegs.LEDRXCount=0;
    }
    if(SysRegs.SystemStateARegs.bit.INITOK ==1)
    {
        SysRegs.Timer50msec++;
        SysRegs.Timer100msec++;
        SysRegs.Timer200msec++;
        SysRegs.Timer500msec++;
        SysRegs.Timer1000msec++;
        SysRegs.Timer2000msec++;
    }
    else
    {
        SysRegs.Timer50msec=0;
        SysRegs.Timer100msec=0;
        SysRegs.Timer200msec=0;
        SysRegs.Timer500msec=0;
        SysRegs.Timer1000msec=0;
        SysRegs.Timer2000msec=0;
    }
    if(SysRegs.Timer50msec>=50)          { SysRegs.Timer50msec   = 0;          }
    if(SysRegs.Timer100msec>=100)        { SysRegs.Timer100msec  = 0;          }
    if(SysRegs.Timer200msec>=200)        { SysRegs.Timer200msec  = 0;          }
    if(SysRegs.Timer500msec>=500)        { SysRegs.Timer500msec  = 0;          }
    if(SysRegs.Timer1000msec>=1000)      { SysRegs.Timer1000msec = 0;          }
    if(SysRegs.Timer2000msec>=2000)      { SysRegs.Timer2000msec = 0;          }
    SysRegs.BMSIDRegs.all=0;
    BMSIDHandler(&SysRegs);
    /*
     *
     */
    SysRegs.BATICErrCount[0]= Slave1Regs.ErrorCount;
    SysRegs.BATICErrCount[1]= Slave1Regs.ErrorCount;

    ModuleErrHandler(&SysRegs);
    /*
     *
     */
    BalanceEnableHandle(&SysRegs);
    SysRegs.HMICellMinVolt  = CANRXRegs.HMICellMinVoltage;
    SysRegs.RackCellMinVolt = CANRXRegs.RackCellMinVoltage;
    SysRegs.SystemStateARegs.bit.PackEn =  CANRXRegs.PackStatus.bit.PackComEn;
    SysRegs.SystemStateARegs.bit.HmiEn  =  CANRXRegs.HMIREQ.bit.HMIComEn;
    BalanceMinVoltHandler(&SysRegs);
    Slave1Regs.BalaCellVolt   = SysRegs.BalaCellMinVolt;
    Slave2Regs.BalaCellVolt   = SysRegs.BalaCellMinVolt;
    SysRegs.RackCellMinVoltage =SysRegs.BalaCellMinVolt;
    /*
     *
     */
    switch(SysRegs.Timer100msec)//100
    {
        case 1:
                CANTXRegs.Timer1000msec++;
                if(CANTXRegs.Timer1000msec>=10)
                {
                  CANTXRegs.BMAVlotTempsQty  = C_CellVoltageNum;
                  CANTXRegs.BMASWVer          = C_SWVer;
                  CANTXRegs.UnitBMSIfro      = ComBine(CANTXRegs.BMASWVer,CANTXRegs.BMAVlotTempsQty);
                  CANTXRegs.UnitBMSID        = (0x401|SysRegs.BMSIDRegs.all);
                  CANATX(CANTXRegs.UnitBMSID,8,CANTXRegs.UnitBMSIfro,C_NorVoltage,C_Capacity,SysRegs.RackCellMinVoltage);
                  CANTXRegs.Timer1000msec=0;
                }
        break;

        case 9:
                CANTXRegs.UnitBMSID = (0x402|SysRegs.BMSIDRegs.all);
                CANATX(CANTXRegs.UnitBMSID,8,SysRegs.CellMaxVoltage,SysRegs.CellMinVoltage,SysRegs.CellAvgVoltage,SysRegs.CellDivVoltage);
        break;
        case 18:
                CANTXRegs.UnitBMSID = (0x403|SysRegs.BMSIDRegs.all);
                CANATX(CANTXRegs.UnitBMSID,8,SysRegs.CellMaxTemperature,SysRegs.CellMinTemperature,SysRegs.CellAvgTemperature,SysRegs.CellDivTemperature);
        break;
        case 23:
                CANTXRegs.BalanceSate.all         = Slave1Regs.Balance.all;
                CANTXRegs.BalanceSate.all         = CANTXRegs.BalanceSate.all<<C_CellBalacneShift;
                CANTXRegs.BalanceSate.all         = CANTXRegs.BalanceSate.all | Slave2Regs.Balance.all;
                CANTXRegs.CellVotlageMaxMinNum    = ComBine(SysRegs.CellMinVoltageNum,SysRegs.CellMaxVoltageNum);
                CANTXRegs.CellTempsMaxMinNum      = ComBine(SysRegs.CellMinTemperatureNum,SysRegs.CellMaxTemperatureNum);
                CANTXRegs.UnitBMSID = (0x404|SysRegs.BMSIDRegs.all);
                CANATX(CANTXRegs.UnitBMSID,8,SysRegs.ModuleVoltage,CANTXRegs.CellVotlageMaxMinNum, CANTXRegs.CellTempsMaxMinNum,SysRegs.SystemStateARegs.all);
        break;
        case 30:
                SysRegs.LEDCanCount++;
                if(SysRegs.LEDCanCount>10)
                {
                    LEDCAN_Toggle;
                    SysRegs.LEDCanCount=0;
                }
        break;

        default:
        break;
    }
    switch(SysRegs.Timer500msec)//500
    {
        case 10:
                 SysRegs.SystemStateARegs.bit.CellVoltCAN =CANRXRegs.HMIREQ.bit.VoltReq;
                 SysRegs.SystemStateARegs.bit.CellTempCAN =CANRXRegs.HMIREQ.bit.TempsReq;
        break;
        case 30:

                if(SysRegs.SystemStateARegs.bit.CellVoltCAN ==1)
                {
                    CANTXRegs.UnitBMSID = (0x500|SysRegs.BMSIDRegs.all);
                    CANTXRegs.CellNumStart=0;
                    CANATX(CANTXRegs.UnitBMSID,8,CANTXRegs.BatteryVoltageCell[CANTXRegs.CellNumStart],CANTXRegs.BatteryVoltageCell[CANTXRegs.CellNumStart+1],
                                                 CANTXRegs.BatteryVoltageCell[CANTXRegs.CellNumStart+2],CANTXRegs.BatteryVoltageCell[CANTXRegs.CellNumStart+3]);
                }
        break;
        case 40:
                if(SysRegs.SystemStateARegs.bit.CellVoltCAN==1)
                {
                    CANTXRegs.UnitBMSID = (0x501|SysRegs.BMSIDRegs.all);
                    CANTXRegs.CellNumStart=4;
                    CANATX(CANTXRegs.UnitBMSID,8,CANTXRegs.BatteryVoltageCell[CANTXRegs.CellNumStart],CANTXRegs.BatteryVoltageCell[CANTXRegs.CellNumStart+1],
                                                 CANTXRegs.BatteryVoltageCell[CANTXRegs.CellNumStart+2],CANTXRegs.BatteryVoltageCell[CANTXRegs.CellNumStart+3]);
                }
        break;
        case 50:
                if(SysRegs.SystemStateARegs.bit.CellVoltCAN==1)
                {
                    CANTXRegs.UnitBMSID = (0x502|SysRegs.BMSIDRegs.all);
                    CANTXRegs.CellNumStart=8;
                    CANATX(CANTXRegs.UnitBMSID,8,CANTXRegs.BatteryVoltageCell[CANTXRegs.CellNumStart],CANTXRegs.BatteryVoltageCell[CANTXRegs.CellNumStart+1],
                                                 CANTXRegs.BatteryVoltageCell[CANTXRegs.CellNumStart+2],CANTXRegs.BatteryVoltageCell[CANTXRegs.CellNumStart+3]);
                }
        break;
        case 60:
                if(SysRegs.SystemStateARegs.bit.CellVoltCAN==1)
                {
                    CANTXRegs.CellNumStart=12;
                    CANTXRegs.UnitBMSID = (0x503|SysRegs.BMSIDRegs.all);
                    CANATX(CANTXRegs.UnitBMSID,8,CANTXRegs.BatteryVoltageCell[CANTXRegs.CellNumStart],CANTXRegs.BatteryVoltageCell[CANTXRegs.CellNumStart+1],
                                                 CANTXRegs.BatteryVoltageCell[CANTXRegs.CellNumStart+2],CANTXRegs.BatteryVoltageCell[CANTXRegs.CellNumStart+3]);
                }
        break;
        case 80:
                if(SysRegs.SystemStateARegs.bit.CellVoltCAN==1)
                {
                    CANTXRegs.CellNumStart=16;
                   // CANTXRegs.UnitBMSID = (0x504|SysRegs.BMSIDRegs.all);
                    CANATX(CANTXRegs.UnitBMSID,8,CANTXRegs.BatteryVoltageCell[CANTXRegs.CellNumStart],CANTXRegs.BatteryVoltageCell[CANTXRegs.CellNumStart+1],
                                                 CANTXRegs.BatteryVoltageCell[CANTXRegs.CellNumStart+2],CANTXRegs.BatteryVoltageCell[CANTXRegs.CellNumStart+3]);
                }
        break;
        case 110:
                if(SysRegs.SystemStateARegs.bit.CellVoltCAN==1)
                {
                    CANTXRegs.CellNumStart=20;
                    CANTXRegs.UnitBMSID = (0x505|SysRegs.BMSIDRegs.all);
                    CANATX(CANTXRegs.UnitBMSID,8,CANTXRegs.BatteryVoltageCell[CANTXRegs.CellNumStart],CANTXRegs.BatteryVoltageCell[CANTXRegs.CellNumStart+1],
                                                 CANTXRegs.BatteryVoltageCell[CANTXRegs.CellNumStart+2],CANTXRegs.BatteryVoltageCell[CANTXRegs.CellNumStart+3]);
                }
        break;
        case 120:
                if(SysRegs.SystemStateARegs.bit.CellTempCAN ==1)
                {
                    CANTXRegs.UnitBMSID = (0x506|SysRegs.BMSIDRegs.all);
                    CANTXRegs.CellNumStart=0;
                    CANATX(CANTXRegs.UnitBMSID,8,CANTXRegs.BatteryTempCell[CANTXRegs.CellNumStart],CANTXRegs.BatteryTempCell[CANTXRegs.CellNumStart+1],
                                                 CANTXRegs.BatteryTempCell[CANTXRegs.CellNumStart+2],CANTXRegs.BatteryTempCell[CANTXRegs.CellNumStart+3]);
                }
        break;
        case 130:
                if(SysRegs.SystemStateARegs.bit.CellTempCAN ==1)
                {
                    CANTXRegs.UnitBMSID = (0x507|SysRegs.BMSIDRegs.all);
                    CANTXRegs.CellNumStart=4;
                    CANATX(CANTXRegs.UnitBMSID,8,CANTXRegs.BatteryTempCell[CANTXRegs.CellNumStart],CANTXRegs.BatteryTempCell[CANTXRegs.CellNumStart+1],
                                             CANTXRegs.BatteryTempCell[CANTXRegs.CellNumStart+2],CANTXRegs.BatteryTempCell[CANTXRegs.CellNumStart+3]);
                }
        break;
        case 140:
                 if(SysRegs.SystemStateARegs.bit.CellTempCAN ==1)
                 {
                     CANTXRegs.UnitBMSID = (0x508|SysRegs.BMSIDRegs.all);
                     CANTXRegs.CellNumStart=8;
                     CANATX(CANTXRegs.UnitBMSID,8,CANTXRegs.BatteryTempCell[CANTXRegs.CellNumStart],CANTXRegs.BatteryTempCell[CANTXRegs.CellNumStart+1],
                                                  CANTXRegs.BatteryTempCell[CANTXRegs.CellNumStart+2],CANTXRegs.BatteryTempCell[CANTXRegs.CellNumStart+3]);
                 }
        break;
        case 200:
                  if(SysRegs.SystemStateARegs.bit.CellTempCAN ==1)
                  {
                    CANTXRegs.CellNumStart=12;
                    CANTXRegs.UnitBMSID = (0x509|SysRegs.BMSIDRegs.all);
                    CANATX(CANTXRegs.UnitBMSID,8,CANTXRegs.BatteryTempCell[CANTXRegs.CellNumStart],CANTXRegs.BatteryTempCell[CANTXRegs.CellNumStart+1],
                                                 CANTXRegs.BatteryTempCell[CANTXRegs.CellNumStart+2],CANTXRegs.BatteryTempCell[CANTXRegs.CellNumStart+3]);
                  }
        break;
        case 210:
                 if(SysRegs.SystemStateARegs.bit.CellTempCAN ==1)
                 {

                    CANTXRegs.CellNumStart=16;
                    CANTXRegs.UnitBMSID = (0x50A|SysRegs.BMSIDRegs.all);
                    CANATX(CANTXRegs.UnitBMSID,8,CANTXRegs.BatteryTempCell[CANTXRegs.CellNumStart],CANTXRegs.BatteryTempCell[CANTXRegs.CellNumStart+1],
                                                 CANTXRegs.BatteryTempCell[CANTXRegs.CellNumStart+2],CANTXRegs.BatteryTempCell[CANTXRegs.CellNumStart+3]);
                 }
        break;
        case 220:
                 if(SysRegs.SystemStateARegs.bit.CellTempCAN ==1)
                 {
                     CANTXRegs.CellNumStart=20;
                     CANTXRegs.UnitBMSID = (0x50B|SysRegs.BMSIDRegs.all);
                     CANATX(CANTXRegs.UnitBMSID,8,CANTXRegs.BatteryTempCell[CANTXRegs.CellNumStart],CANTXRegs.BatteryTempCell[CANTXRegs.CellNumStart+1],
                                                  CANTXRegs.BatteryTempCell[CANTXRegs.CellNumStart+2],CANTXRegs.BatteryTempCell[CANTXRegs.CellNumStart+3]);
                 }
        break;
        case 250 :
                SysRegs.SystemStateARegs.bit.CellVoltCAN =0;
                SysRegs.SystemStateARegs.bit.CellTempCAN =0;
        break;
        default:
        break;
    }

    switch(SysRegs.Timer1000msec) //1000
    {
         case 2:

         break;
         case 260:
                   /*
                   if(SysRegs.CellMaxTemperature>=C_FANOnTemperature)
                   {
                       SysRegs.DigitalOutPutReg.bit.FAN = 1;
                   }
                   if(SysRegs.CellMaxTemperature < C_FANOffTemperature)
                   {
                     SysRegs.DigitalOutPutReg.bit.FAN = 0;
                   }*/

           break;

         default:
         break;
    }
    DigitalOutput(&SysRegs);

	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

interrupt void ISR_CANRXINTA(void)
{
    CANRXRegs.RxCount++;
    if(ECanaRegs.CANRMP.bit.RMP0==1)
    {
        CANRXRegs.MasterCount++;
        if(ECanaMboxes.MBOX0.MSGID.bit.STDMSGID==0x300)
        {

            CANRXRegs.LEDRXCount++;
            SysRegs.MasterCount = 0;
            if(CANRXRegs.MasterCount>3000)       {CANRXRegs.MasterCount=0;}
            CANRXRegs.PackStatus.all           =  (ECanaMboxes.MBOX0.MDL.byte.BYTE1<<8)|(ECanaMboxes.MBOX0.MDL.byte.BYTE0);
            CANRXRegs.RackCellAgvTemps         = (ECanaMboxes.MBOX0.MDH.byte.BYTE5<<8)|(ECanaMboxes.MBOX0.MDH.byte.BYTE4);
            CANRXRegs.RackCellMinVoltage       = (ECanaMboxes.MBOX0.MDH.byte.BYTE7<<8)|(ECanaMboxes.MBOX0.MDH.byte.BYTE6);
        }
          ECanaShadow.CANRMP.all = 0;
          ECanaShadow.CANRMP.bit.RMP0 = 1;
          ECanaRegs.CANRMP.all = ECanaShadow.CANRMP.all;
    }
    if(ECanaRegs.CANRMP.bit.RMP1==1)
    {
        SysRegs.CurrentCount = 0;
        if(ECanaMboxes.MBOX1.MSGID.bit.STDMSGID==0x3C2)
        {
           CANRXRegs.CurrentCount++;
           if(CANRXRegs.CurrentCount>3000){CANRXRegs.CurrentCount=0;}
           SysRegs.CurrentData.byte.CurrentH   = (ECanaMboxes.MBOX1.MDL.byte.BYTE0<<8)|(ECanaMboxes.MBOX1.MDL.byte.BYTE1);
           SysRegs.CurrentData.byte.CurrentL   = (ECanaMboxes.MBOX1.MDL.byte.BYTE2<<8)|(ECanaMboxes.MBOX1.MDL.byte.BYTE3);
        }
          ECanaShadow.CANRMP.all = 0;
          ECanaShadow.CANRMP.bit.RMP1 = 1;  //interrupt pending clear by writing 1
          ECanaRegs.CANRMP.all = ECanaShadow.CANRMP.all;

     }
     if(ECanaRegs.CANRMP.bit.RMP2==1)
     {
         CANRXRegs.HmiCount++;
         if(ECanaMboxes.MBOX2.MSGID.bit.STDMSGID==0x301)
         {
             SysRegs.HMICount=0;
             if(CANRXRegs.HmiCount>3000){CANRXRegs.HmiCount=0;}
             CANRXRegs.HMIREQ.all              = (ECanaMboxes.MBOX2.MDL.byte.BYTE1<<8)|(ECanaMboxes.MBOX2.MDL.byte.BYTE0);
             CANRXRegs.HMICellAgvTemps       = (ECanaMboxes.MBOX2.MDH.byte.BYTE5<<8)|(ECanaMboxes.MBOX2.MDH.byte.BYTE4);
             CANRXRegs.HMICellMinVoltage       = (ECanaMboxes.MBOX2.MDH.byte.BYTE7<<8)|(ECanaMboxes.MBOX2.MDH.byte.BYTE6);
         }
         ECanaShadow.CANRMP.all = 0;
         ECanaShadow.CANRMP.bit.RMP2 = 1;
         ECanaRegs.CANRMP.all = ECanaShadow.CANRMP.all;
     }

     ECanaShadow.CANME.all=ECanaRegs.CANME.all;
     ECanaShadow.CANME.bit.ME0=1;    //0x5NA MCU Rx Enable
     ECanaShadow.CANME.bit.ME1=1;    //0x5NA MCU Rx Enable
     ECanaShadow.CANME.bit.ME2=1;    //0x5NB MCU Rx Enable
     ECanaShadow.CANME.bit.ME31=1;   //CAN-A Tx Enable
     ECanaRegs.CANME.all=ECanaShadow.CANME.all;
     PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;
     //  IER |= 0x0100;                  // Enable INT9
     //  EINT;
 }//EOF
//interrupt void ISR_SCIRXINTA(void)
//{
//    PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;     // Acknowledge interrupt to PIE
//}
//



