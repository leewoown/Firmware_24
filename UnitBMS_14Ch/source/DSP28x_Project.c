
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include <stdio.h>
#include <string.h>

extern SystemReg           SysRegs;
extern CANRXReg            CANRXRegs;
extern CANTXReg            CANTXRegs;
extern ADCReg              ADCRegs;

extern SlaveReg            Slave1Regs;
extern SlaveReg            Slave2Regs;
extern SlaveReg            Slave3Regs;
extern void temperatureRelayHandler(SystemReg *sys);
//************************************************************************************//
//                                   User Code                                        //
//************************************************************************************//

void TEMP1_CAL(SlaveReg *s,Uint16 num)
{
     s->CellTempV[num] = (Slave1Regs.ADCTEMPBuf[num] * AdcNormalizerUnipolar)*3.3;
     s->X01 = s->CellTempV[num];
     s->X02 = (s->CellTempV[num]*s->X01);
     s->X03 = (s->CellTempV[num]*s->X02);

     s->CellTempF[num] = -6.88 * s->X03 +37.14 * s->X02 -98.17 * s->X01 +116.74;
     s->CellTempF[num] = s->CellTempF[num]+3;

     s->CellTemp[num] = (int)(s->CellTempF[num]*10);
}

void TEMP2_CAL(SlaveReg *s,Uint16 num)
{
//
     s->CellTempV[num] = (Slave2Regs.ADCTEMPBuf[num] * AdcNormalizerUnipolar)*3.3;
     s->X01 = s->CellTempV[num];
     s->X02 = (s->CellTempV[num]*s->X01);
     s->X03 = (s->CellTempV[num]*s->X02);

     s->CellTempF[num] = -6.88 * s->X03 +37.14 * s->X02 -98.17 * s->X01 +116.74;
     s->CellTempF[num] = s->CellTempF[num]+3;

     s->CellTemp[num] = (int)(s->CellTempF[num]*10);
}

void TEMP3_CAL(SlaveReg *s,Uint16 num)
{
//     s->CellTempV[num] = (float)(ADCRegs.ADCA07buf * AdcNormalizerUnipolar)*3.3;
     s->CellTempV[num] = (Slave3Regs.ADCTEMPBuf[num] * AdcNormalizerUnipolar)*3.3;
     s->X01 = s->CellTempV[num];
     s->X02 = (s->CellTempV[num]*s->X01);
     s->X03 = (s->CellTempV[num]*s->X02);

     s->CellTempF[num] = -6.88 * s->X03 +37.14 * s->X02 -98.17 * s->X01 +116.74;
     s->CellTempF[num] = s->CellTempF[num]+3;

     s->CellTemp[num] = (int)s->CellTempF[num]*10;
}

void SystemRegsInit(SystemReg *s)
{
       //----------TIMER 초기화----------//
       s->MainIsr=0;
       s->Timer100msec  = 0;
       s->Timer200msec  = 0;
       s->Timer500msec  = 0;
       s->Timer1000msec = 0;
       s->Timer2000msec = 0;
       s->initCount=0;
       s->CellVoltsampling=0;
       s->CellTempSampling=0;
       s->LEDCanCount=0;

       //----------DIO 초기화----------//
       s->DigitalOutPutReg.all           = 0;
       s->DigitalInputReg.all            = 0;
       s->BMSIDRegs.all                  = 0;
       s->SystemStateARegs.all           = 0;
       s->CurrentData.all                = 0;
       s->HMIREQ.all                     = 0;
       s->PackStatus.all                 = 0;
       s->SystemStateARegs.all           = 0;
       //----------UnitBMS Balancing 초기화----------//
       s->BalanceMode                    = 0;
       s->BalanceState                   = 0;
       s->RackCellMinVoltage             = 0;
       s->RackCellDivVoltage             = 0;

       s->SystemStateARegs.bit.CellVoltCAN=0;
       //----------UnitBMS Module state 초기화----------//
       s->HeartBeat             = 0;
       s->ModuleVoltage         = 0;
       s->ModuleCurrentF        = 0.0;
       s->ModuleCurrentabsF     = 0.0;

       //----------UnitBMS CELL Voltage 초기화----------//
       s->Number                  = 0;
       s->CellVoltageCount        = 0;
       s->CellMaxVoltage          = 0;
       s->CellMinVoltage          = 0;
       s->RackCellMinVoltage      = 0;
       s->CellMaxVoltageNum       = 0;
       s->CellMinVoltageNum       = 0;
       s->CellVoltageCount=0;
       s->CellVoltagePositionbuf1 = 0;
       s->CellVoltagePositionbuf2 = 0;
       s->CellOffsetVotageDiv     = 0;
       s->CellVoltDivCnt          = 0;
       //----------UnitBMS CELL Temperature 초기화----------//
       s->TempNumber                     = 0;
       s->CellTempertureCount=0;
       s->CellMaxTemperature      = 0;
       s->CellMaxTemperatureNum   = 0;
       s->CellMinTemperature      = 0;
       s->CellMinTemperatureNum   = 0;
       s->CellSumVoltage          = 0;

       //----------UnitBMS CAN 초기화----------//
       s->BATICErrCount[0]        = 0;
       s->BATICErrCount[1]        = 0;
       s->CurrentCount            = 0;
       s->MasterCount             = 0;
       s->BalanceStartCount       = 0;
       s->HMICellMinVolt          = 0;
       s->RackCellMinVolt         = 0;
}
void CANRxRegsInit(CANRXReg *P)
{
    P->RackCellMinVoltage=0; // Pack or Rack
    P->RackCellAgvTemps=0; // Pack or Rack
    P->HMICellMinVoltage=0; // Pack or RacK
    P->HMICellAgvTemps=0; // Pack or RacK
    P->CurrentCount=0;
    P->MasterCount=0;
    P->HmiCount=0;
    P->LEDRXCount=0;
    P->RxCount                  = 0;
    P->CurrentCount             = 0;
    P->MasterCount              = 0;
    P->HmiCount                 = 0;
    P->LEDRXCount               = 0;
   // P->Cell_Data_Request  = 0;
    P->HMIREQ.all               =0;
    P->PackStatus.all           =0;
}
void CANTxRegsInit(CANTXReg *P)
{
    memset(&P->BatteryVoltageCell[0],0, 12);
    memset(&P->BatteryTempCell[0],0, 12);
    P->UnitBMSID         = 0;
    P->ErrorCount        = 0;
    P->BalanceSate.all   = 0;
    P->CellVoltageNumCnt = 0;
    P->CellVotlNum       = 0;
    P->CellTemNum        = 0;
    P->CellTemNumCnt     = 0;
    P->Timer1000msec     = 0;
    P->CellNumStart      = 0;
 // P->NumberShift       = 0;
    P->UnitBMSIfro       = 0;

    P->CellVotlNum          = 0;
    P->CellVotlageMaxMinNum = 0;
}
void SlaveRegsInit(SlaveReg *P)
{
    P->Balance.all = 0x0000;
    P->ErrorCount  = 0;
    P->ID=C_Slave1_ID;
}
void SystemInit(SystemReg *s)  //msec
{
       if(s->initCount<5)
       {
           s->SystemStateARegs.bit.INITOK=0;
       }
       if(s->initCount>=5)
       {
           s->SystemStateARegs.bit.INITOK=1;
           s->initCount=10;
       }
}
void ModuleErrHandler(SystemReg *P)
{
    /*
     * INPUT Variable
     *  -. SysRegs.CellMaxVoltage
     *  -. SysRegs.CellMinVoltage
     *  -. SysRegs.CellMaxTemperature
     *  -. SysRegs.CellMinTemperature
     *  -. SysRegs.BATICErrCount[0]= Slave1Regs.ErrorCount;
     *  -. SysRegs.BATICErrCount[1]= Slave1Regs.ErrorCount;
     * OUTPUT Variable
     *  -. SysRegs.SystemStateARegs.bit.Fault=1;
     *  -. SysRegs.SystemStateARegs.bit.BalanceEnable=0
     */
    P->MasterCount++;
    P->CurrentCount++;
    P->HMICount++;
    if(P->MasterCount>=1000)  // 1sec
    {
        P->SystemStateARegs.bit.MBCOMErrFault=1;
    }
    else
    {
        P->SystemStateARegs.bit.MBCOMErrFault=0;
    }
    if(P->HMICount>=1000)    // 1sec
    {
        P->SystemStateARegs.bit.HMICOMErrFault=1;
    }
    else
    {
        P->SystemStateARegs.bit.HMICOMErrFault=0;
    }
    if(P->CurrentCount>=1000) //100msec
    {
        P->SystemStateARegs.bit.CTCOMErrFault=1;
    }
    else
    {
        P->SystemStateARegs.bit.CTCOMErrFault=0;
    }

    if (P->DigitalInputReg.bit.Wakeleakstates==1)
    {
        P->SystemStateARegs.bit.WaterleakFault=1;
    }
    else
    {
        P->SystemStateARegs.bit.WaterleakFault=0;
    }


    if(SysRegs.CellMinVoltage < 2501 || SysRegs.CellMaxVoltage> 4200)
    {
        P->SystemStateARegs.bit.CellVoltageFault=1;
    }
    else
    {
        P->SystemStateARegs.bit.CellVoltageFault=0;
    }
    if(SysRegs.CellMinTemperature < -250 || SysRegs.CellMaxTemperature>1000)
    {
        P->SystemStateARegs.bit.CellTemperatureFault=1;
    }
    else
    {
        P->SystemStateARegs.bit.CellTemperatureFault=0;
    }
    if(P->BATICErrCount[0]>=1000 || P->BATICErrCount[1]>=1000)
    {
        P->SystemStateARegs.bit.BATIC1ErrFault=1;
    }

    if((P->SystemStateARegs.bit.CTCOMErrFault ==1)  ||(P->SystemStateARegs.bit.MBCOMErrFault==1)   ||(P->SystemStateARegs.bit.BATIC1ErrFault ==1)||
       (P->SystemStateARegs.bit.WaterleakFault ==1) ||(P->SystemStateARegs.bit.CellVoltageFault==1)||(P->SystemStateARegs.bit.CellTemperatureFault==1))
    {
        P->SystemStateARegs.bit.Fault=1;
       // P->SystemStateARegs.bit.BalanceEnable=0;
    }
}
void CalVoltageHandler(SystemReg *s)
{
    s->Number = 0;
    unsigned int CellMaxVolt     = 0;
    unsigned int CellMaxVoltNum  = 0;

    unsigned int CellMinVolt     = 0;
    unsigned int CellMinVoltNum  = 0;

    CellMaxVolt = s->CellVoltage[0];
    CellMinVolt = s->CellVoltage[0];
    s->CellSumVoltageBuf =0;
    for(s->Number=0;s->Number<s->CellVoltageCount;s->Number++)
    {
        if(CellMaxVolt <= s->CellVoltage[s->Number])
        {
            CellMaxVolt    = s->CellVoltage[s->Number];
            CellMaxVoltNum = s->Number;
        }
        if(CellMinVolt >=  s->CellVoltage[s->Number])
        {
            CellMinVolt    = s->CellVoltage[s->Number];
            CellMinVoltNum = s->Number;
        }
        s->CellSumVoltageBuf = s->CellSumVoltageBuf + s->CellVoltage[s->Number];
    }


    s->CellMaxVoltageNum = CellMaxVoltNum + 1;
    s->CellMaxVoltage    = CellMaxVolt;
    s->CellMinVoltageNum = CellMinVoltNum + 1;
    s->CellMinVoltage    = CellMinVolt;
    s->CellDivVoltage   = CellMaxVolt - CellMinVolt;
   // s->RackCellDivVoltage = CellMaxVolt - s->RackCellMinVoltage;
    s->CellSumVoltage   = s->CellSumVoltageBuf;
    s->ModuleVoltage    = (unsigned int) (s->CellSumVoltage *0.1);
    s->CellAvgVoltage    =(unsigned int) (s->CellSumVoltage /C_CellVoltageNum);

}
void ModuleCurrent(SystemReg *s)
{
    long  CurrentCT  = 0;
    float32 Currentbuf = 0;
    CurrentCT  = s->CurrentData.all;
    CurrentCT  =  CurrentCT - 0x80000000;

    Currentbuf  = ((float)CurrentCT)/1000;              // (mA to A) CAB500 resolution 1mA
    s->ModuleCurrentF  = C_CTDirection * Currentbuf;    // Decide Current sensor's direction
    if(s->ModuleCurrentF>=500.0)
    {
        s->ModuleCurrentF=500.0;
    }
    if(s->ModuleCurrentF<=-500.0)
    {
        s->ModuleCurrentF=-500.0;
    }
    if(s->ModuleCurrentF < 0)
    {
        s->ModuleCurrentabsF =-1.0 * s->ModuleCurrentF;
    }
    else
    {
        s->ModuleCurrentabsF =s->ModuleCurrentF;
    }

}
void CellTempsAdcHandler(int Ch, SlaveReg *s)
{
    if(Ch==1)
    {
        TEMP1DO00H;
        TEMP1DO01L;
        TEMP1DO02L;
        delay_ms(1);
        AdcRegs.ADCSOCFRC1.all = 0xFF;
        ADC_Read(&ADCRegs);
        s->ADCTEMPBuf[0] = (ADCRegs.ADCA01buf);

        TEMP1DO00L;
        TEMP1DO01H;
        TEMP1DO02L;
        delay_ms(1);
        AdcRegs.ADCSOCFRC1.all = 0xFF;
        ADC_Read(&ADCRegs);
        s->ADCTEMPBuf[1] = (ADCRegs.ADCA01buf);

        TEMP1DO00H;
        TEMP1DO01H;
        TEMP1DO02L;
        delay_ms(1);
        AdcRegs.ADCSOCFRC1.all = 0xFF;
        ADC_Read(&ADCRegs);
        s->ADCTEMPBuf[2] = (ADCRegs.ADCA01buf);

        TEMP1DO00L;
        TEMP1DO01L;
        TEMP1DO02H;
        delay_ms(1);
        AdcRegs.ADCSOCFRC1.all = 0xFF;
        ADC_Read(&ADCRegs);
        s->ADCTEMPBuf[3] = (ADCRegs.ADCA01buf);

        TEMP1DO00H;
        TEMP1DO01L;
        TEMP1DO02H;
        delay_ms(1);
        AdcRegs.ADCSOCFRC1.all = 0xFF;
        ADC_Read(&ADCRegs);
        s->ADCTEMPBuf[4] = (ADCRegs.ADCA01buf);

        TEMP1DO00L;
        TEMP1DO01H;
        TEMP1DO02H;
        delay_ms(1);
        AdcRegs.ADCSOCFRC1.all = 0xFF;
        ADC_Read(&ADCRegs);
        s->ADCTEMPBuf[5] = (ADCRegs.ADCA01buf);

        TEMP1DO00H;
        TEMP1DO01H;
        TEMP1DO02H;
        delay_ms(1);
        AdcRegs.ADCSOCFRC1.all = 0xFF;
        ADC_Read(&ADCRegs);
        s->ADCTEMPBuf[6] = (ADCRegs.ADCA01buf);

        TEMP1DO00L;
        TEMP1DO01L;
        TEMP1DO02L;
        delay_ms(1);
        AdcRegs.ADCSOCFRC1.all = 0xFF;
        ADC_Read(&ADCRegs);
        s->ADCTEMPBuf[7] = (ADCRegs.ADCA01buf);
    }
    if(Ch==2)
    {
        TEMP2DO00H;
        TEMP2DO01L;
        TEMP2DO02L;
        delay_ms(1);
        AdcRegs.ADCSOCFRC1.all = 0xFF;
        ADC_Read(&ADCRegs);
        s->ADCTEMPBuf[0] = (ADCRegs.ADCA03buf);

        TEMP2DO00L;
        TEMP2DO01H;
        TEMP2DO02L;
        delay_ms(1);
        AdcRegs.ADCSOCFRC1.all = 0xFF;
        ADC_Read(&ADCRegs);
        s->ADCTEMPBuf[1] = (ADCRegs.ADCA03buf);

        TEMP2DO00H;
        TEMP2DO01H;
        TEMP2DO02L;
        delay_ms(1);
        AdcRegs.ADCSOCFRC1.all = 0xFF;
        ADC_Read(&ADCRegs);
        s->ADCTEMPBuf[2] = (ADCRegs.ADCA03buf);

        TEMP2DO00L;
        TEMP2DO01L;
        TEMP2DO02H;
        delay_ms(1);
        AdcRegs.ADCSOCFRC1.all = 0xFF;
        ADC_Read(&ADCRegs);
        s->ADCTEMPBuf[3] = (ADCRegs.ADCA03buf);

        TEMP2DO00H;
        TEMP2DO01L;
        TEMP2DO02H;
        delay_ms(1);
        AdcRegs.ADCSOCFRC1.all = 0xFF;
        ADC_Read(&ADCRegs);
        s->ADCTEMPBuf[4] = (ADCRegs.ADCA03buf);

        TEMP2DO00L;
        TEMP2DO01H;
        TEMP2DO02H;
        delay_ms(1);
        AdcRegs.ADCSOCFRC1.all = 0xFF;
        ADC_Read(&ADCRegs);
        s->ADCTEMPBuf[5] = (ADCRegs.ADCA03buf);

        TEMP2DO00H;
        TEMP2DO01H;
        TEMP2DO02H;
        delay_ms(1);
        AdcRegs.ADCSOCFRC1.all = 0xFF;
        ADC_Read(&ADCRegs);
        s->ADCTEMPBuf[6] = (ADCRegs.ADCA03buf);

        TEMP2DO00L;
        TEMP2DO01L;
        TEMP2DO02L;
        delay_ms(1);
        AdcRegs.ADCSOCFRC1.all = 0xFF;
        ADC_Read(&ADCRegs);
        s->ADCTEMPBuf[7] = (ADCRegs.ADCA03buf);
    }
    if(Ch==3)
    {
        TEMP3DO00H;
        TEMP3DO01L;
        TEMP3DO02L;
        delay_ms(1);
        AdcRegs.ADCSOCFRC1.all = 0xFF;
        ADC_Read(&ADCRegs);
        s->ADCTEMPBuf[0] = (ADCRegs.ADCA07buf);

        TEMP3DO00L;
        TEMP3DO01H;
        TEMP3DO02L;
        delay_ms(1);
        AdcRegs.ADCSOCFRC1.all = 0xFF;
        ADC_Read(&ADCRegs);
        s->ADCTEMPBuf[1] = (ADCRegs.ADCA07buf);

        TEMP3DO00H;
        TEMP3DO01H;
        TEMP3DO02L;
        delay_ms(1);
        AdcRegs.ADCSOCFRC1.all = 0xFF;
        ADC_Read(&ADCRegs);
        s->ADCTEMPBuf[2] = (ADCRegs.ADCA07buf);

        TEMP3DO00L;
        TEMP3DO01L;
        TEMP3DO02H;
        delay_ms(1);
        AdcRegs.ADCSOCFRC1.all = 0xFF;
        ADC_Read(&ADCRegs);
        s->ADCTEMPBuf[3] = (ADCRegs.ADCA07buf);

        TEMP3DO00H;
        TEMP3DO01L;
        TEMP3DO02H;
        delay_ms(1);
        AdcRegs.ADCSOCFRC1.all = 0xFF;
        ADC_Read(&ADCRegs);
        s->ADCTEMPBuf[4] = (ADCRegs.ADCA07buf);

        TEMP3DO00L;
        TEMP3DO01H;
        TEMP3DO02H;
        delay_ms(1);
        AdcRegs.ADCSOCFRC1.all = 0xFF;
        ADC_Read(&ADCRegs);
        s->ADCTEMPBuf[5] = (ADCRegs.ADCA07buf);

        TEMP3DO00H;
        TEMP3DO01H;
        TEMP3DO02H;
        delay_ms(1);
        AdcRegs.ADCSOCFRC1.all = 0xFF;
        ADC_Read(&ADCRegs);
        s->ADCTEMPBuf[6] = (ADCRegs.ADCA07buf);

        TEMP3DO00L;
        TEMP3DO01L;
        TEMP3DO02L;
        delay_ms(1);
        AdcRegs.ADCSOCFRC1.all = 0xFF;
        ADC_Read(&ADCRegs);
        s->ADCTEMPBuf[7] = (ADCRegs.ADCA07buf);

    }
}
void CalTempHandler(SystemReg *s)
{
    int i = 0;

    int CellMaxTemp    = 0;
    int CellMaxTempNum = 0;

    int CellMinTemp    = 0;
    int CellMinTempNum = 0;

    int CellAvgTemp    = 0;

    CellMaxTemp = s->CellTemperture[0];
    CellMinTemp = s->CellTemperture[0];

    for(i=0;i<s->CellTempertureCount;i++)
    {
        if(CellMaxTemp <= s->CellTemperture[i])
        {
            CellMaxTemp    = s->CellTemperture[i];
            CellMaxTempNum = i;
        }
        if(CellMinTemp >= s->CellTemperture[i])
        {
            CellMinTemp    = s->CellTemperture[i];
            CellMinTempNum = i;
        }

        CellAvgTemp = CellAvgTemp + s->CellTemperture[i];
    }

    s->CellMaxTemperature    = CellMaxTemp;
    s->CellMaxTemperatureNum = CellMaxTempNum +1;

    s->CellMinTemperature    = CellMinTemp;
    s->CellMinTemperatureNum = CellMinTempNum +1;

    s->CellDivTemperature = CellMaxTemp - CellMinTemp;

    s->CellAvgTemperature = CellAvgTemp / C_CellTemperatureNum;

    if(s->CellAvgTemperature>=100)
    {
      s->ModuleTemperature = s->CellMaxTemperature;
    }
    if(s->CellAvgTemperature<100)
    {
      s->ModuleTemperature  = s->CellMinTemperature;
    }
}
void BalanceEnableHandle(SystemReg *P)
{
    if(P->ModuleCurrentabsF <=3.0)
    {
       P->BalanceStartCount++;
       if(P->BalanceStartCount>=5000)
       {
           P->BalanceStartCount=5100;
           P->SystemStateARegs.bit.BalanceEnable=1;
       }
       if(P->CellMinVoltage <= 2950)
       {
           P->SystemStateARegs.bit.BalanceEnable=0;
           P->SystemStateARegs.bit.BalanceStartStop=0;
       }
       if(P->SystemStateARegs.bit.CTCOMErrFault==1)
       {
           P->SystemStateARegs.bit.BalanceEnable=0;
           P->SystemStateARegs.bit.BalanceStartStop=0;
       }
    }
    else
    {
        P->SystemStateARegs.bit.BalanceEnable=0;
        P->SystemStateARegs.bit.BalanceStartStop=0;
    }
    if(P->SystemStateARegs.bit.BalanceEnable==1)
    {
        P->TimerBalance++;
        if(P->TimerBalance>500)
        {
            P->SystemStateARegs.bit.BalanceStartStop= ~P->SystemStateARegs.bit.BalanceStartStop;
            P->TimerBalance=0;
        }
    }


}
//INPUT1 : SystemReg.HMICellMinVolt  = CANRXRegs.HMICellMinVoltage
//INPUT2 : SystemReg.RackCellMinVolt = CANRXRegs.RackCellMinVoltage
//INPUT3 : SystemReg.SystemStateARegs.bit.PackEn = CANRXRegs.PackStatus.bit.PackComEn;
//INPUT4 : SystemReg.SystemStateARegs.bit.HmiEn =  CANRXRegs.HMIREQ.bit.HMIComEn;
//OUTPUT : Slave1Regs.BalaCellVolt=SystemReg.BalaCellMinVolt
//OUTPUT : Slave2Regs.BalaCellVolt=SystemReg.BalaCellMinVolt
void BalanceMinVoltHandler(SystemReg *P)
{
    if(P->SystemStateARegs.bit.CTCOMErrFault==0)
    {
        if(P->SystemStateARegs.bit.MBCOMErrFault==0)//
        {
            P->BalaCellMinVolt=P->RackCellMinVolt;
            if(P->SystemStateARegs.bit.HMICOMErrFault==0)
            {
               if(P->SystemStateARegs.bit.HmiEn==1)
               {
                   P->BalaCellMinVolt=4500;
                   if(P->HMICellMinVolt>3000)
                   {
                       P->BalaCellMinVolt=P->HMICellMinVolt;
                   }
               }
            }
            if(P->SystemStateARegs.bit.HMICOMErrFault==1)
            {
                P->SystemStateARegs.bit.CellTempCAN=0;
                P->SystemStateARegs.bit.CellVoltCAN =0;
                P->SystemStateARegs.bit.HmiEn=0;
                P->HMICellMinVolt=4500;
            }
        }
        if(P->SystemStateARegs.bit.MBCOMErrFault==1)
        {
           // P->BalaCellMinVolt=4500;
            if(P->SystemStateARegs.bit.HMICOMErrFault==0)
            {
               if(P->SystemStateARegs.bit.HmiEn==1)
               {
                  // P->BalaCellMinVolt=4500;
                   P->BalaCellMinVolt=P->HMICellMinVolt;
                   if(P->HMICellMinVolt<2800)
                   {
                       P->BalaCellMinVolt=4500;
                   }

               }
            }
        }
    }
    if(P->SystemStateARegs.bit.CTCOMErrFault==1)
     {
         P->BalaCellMinVolt=4500;
     }
    if((P->SystemStateARegs.bit.MBCOMErrFault==1)&&(P->SystemStateARegs.bit.MBCOMErrFault==1))
    {
       // P->BalaCellMinVolt=4500;
    }
}
//Slave1Regs.BalaCellVolt=SysRegs.SysBalaCellMinVolt;
//Slave2Regs.BalaCellVolt=SysRegs.SysBalaCellMinVolt;
void BalanceCtrlHandler(SlaveReg *P)
{
   // int DivVoltage=0;


    P->DivVoltage[0]= P->CellVoltage[0]-P->BalaCellVolt;
    if(P->DivVoltage[0] >= C_BalanceDivVoltage)
    {
        P->Balance.bit.B_Cell00=1;
    }
    else
    {
        P->Balance.bit.B_Cell00=0;
    }

    P->DivVoltage[1]=P->CellVoltage[1]-P->BalaCellVolt;
    if(P->DivVoltage[1] >= C_BalanceDivVoltage)
    {
        P->Balance.bit.B_Cell01=1;
    }
    else
    {
        P->Balance.bit.B_Cell01=0;
    }

    P->DivVoltage[2]=P->CellVoltage[2]-P->BalaCellVolt;
    if(P->DivVoltage[2] >= C_BalanceDivVoltage)
    {
        P->Balance.bit.B_Cell02=1;
    }
    else
    {
        P->Balance.bit.B_Cell02=0;
    }
    P->DivVoltage[3]=P->CellVoltage[3]-P->BalaCellVolt;
    if(P->DivVoltage[3] >= C_BalanceDivVoltage)
    {
        P->Balance.bit.B_Cell03=1;
    }
    else
    {
        P->Balance.bit.B_Cell03=0;
    }
    P->DivVoltage[4]=P->CellVoltage[4]-P->BalaCellVolt;
    if(P->DivVoltage[4] >= C_BalanceDivVoltage)
    {
        P->Balance.bit.B_Cell04=1;
    }
    else
    {
        P->Balance.bit.B_Cell04=0;
    }
    P->DivVoltage[5]=P->CellVoltage[5]-P->BalaCellVolt;
    if(P->DivVoltage[5] >= C_BalanceDivVoltage)
    {
        P->Balance.bit.B_Cell05=1;
    }
    else
    {
        P->Balance.bit.B_Cell05=0;
    }

    P->DivVoltage[6]=P->CellVoltage[6]-P->BalaCellVolt;
    if(P->DivVoltage[6] >= C_BalanceDivVoltage)
    {
        P->Balance.bit.B_Cell06=1;
    }
    else
    {
        P->Balance.bit.B_Cell06=0;
    }
    P->DivVoltage[7]=P->CellVoltage[7]-P->BalaCellVolt;
    if(P->DivVoltage[7] >= C_BalanceDivVoltage)
    {
        P->Balance.bit.B_Cell07=1;
    }
    else
    {
        P->Balance.bit.B_Cell07=0;
    }

    P->DivVoltage[8]=P->CellVoltage[8]-P->BalaCellVolt;
    if(P->DivVoltage[8] >= C_BalanceDivVoltage)
    {
        P->Balance.bit.B_Cell08=1;
    }
    else
    {
        P->Balance.bit.B_Cell08=0;
    }
    P->DivVoltage[9]=P->CellVoltage[9]-P->BalaCellVolt;
    if(P->DivVoltage[9] >= C_BalanceDivVoltage)
    {
        P->Balance.bit.B_Cell09=1;
    }
    else
    {
        P->Balance.bit.B_Cell09=0;
    }
    P->DivVoltage[10]=P->CellVoltage[10]-P->BalaCellVolt;
    if(P->DivVoltage[10] >= C_BalanceDivVoltage)
    {
        P->Balance.bit.B_Cell10=1;
    }
    else
    {
        P->Balance.bit.B_Cell10=0;
    }
    P->DivVoltage[11]=P->CellVoltage[11]-P->BalaCellVolt;
    if(P->DivVoltage[11] >= C_BalanceDivVoltage)
    {
        P->Balance.bit.B_Cell11=1;
    }
    else
    {
        P->Balance.bit.B_Cell11=0;
    }

}
void BMSIDHandler(SystemReg *sys)
{
    if(ID_SW00==0)
    {
        sys->BMSIDRegs.bit.BMS_ID_00=1;
    }
    else
    {
        sys->BMSIDRegs.bit.BMS_ID_00=0;
    }
    if(ID_SW01==0)
    {
        sys->BMSIDRegs.bit.BMS_ID_01=1;
    }
    else
    {
        sys->BMSIDRegs.bit.BMS_ID_01=0;
    }
    if(ID_SW02==0)
    {
        sys->BMSIDRegs.bit.BMS_ID_02=1;
    }
    else
    {
        sys->BMSIDRegs.bit.BMS_ID_02=0;
    }
    if(ID_SW03==0)
    {
        sys->BMSIDRegs.bit.BMS_ID_03=1;
    }
    else
    {
        sys->BMSIDRegs.bit.BMS_ID_03=0;
    }
    if(ID_SW04==0)
    {
        sys->BMSIDRegs.bit.BMS_ID_04=0;
    }
    else
    {
        sys->BMSIDRegs.bit.BMS_ID_04=0;
    }

}

void DigitalInput(SystemReg *sys)
{
  /*

    if(sys->DigitalOutPutReg.bit.FAN==1)
    {
        sys->DigitalInputReg.bit.Fanstates=1;
    }
    else
    {
        sys->DigitalInputReg.bit.Fanstates=0;
    }
    */
    if(WakeleakState ==1)
    {
        sys->DigitalInputReg.bit.Wakeleakstates=1;
    }
    else
    {
        sys->DigitalInputReg.bit.Wakeleakstates=0;
    }
//    sys->PACKBMSID  = sys->DigitalInputReg.all & 0x001F; // To operate independently of the ChargeFault state Condition
}

void DigitalOutput(SystemReg *sys)
{
    if(sys->DigitalOutPutReg.bit.FAN==1)
    {
        FAN_ON;
    }
    else
    {
        FAN_OFF;
    }

    if(sys->DigitalOutPutReg.bit.LEDCAN)
    {
       // LEDCAN_OFF;
    }
    else
    {
     //   LEDCAN_ON;
    }

    if(sys->DigitalOutPutReg.bit.LEDSTATE)
    {
      //  LEDSTATE_OFF;
    }
    else
    {
      //  LEDSTATE_ON;
    }

}

void ADC_Read(ADCReg *A)
{

    //while(AdcRegs.ADCINTFLG.bit.ADCINT1 == 0){}
   // AdcRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;    //Clear ADCINT1
    A->ADCA01buf = AdcResult.ADCRESULT1;    //Temperature00
    A->ADCA03buf = AdcResult.ADCRESULT3;    //Temperature01
    A->ADCA07buf = AdcResult.ADCRESULT4;    //Temperature02

}

void SlaveBMSSPIEnable_low(void)
{
	GpioDataRegs.GPACLEAR.bit.GPIO10 = 1;
}

void SlaveBMSSPIEnable_high(void)
{
	GpioDataRegs.GPASET.bit.GPIO10 = 1;
}
void SlaveBms_WakeUp(void)
{
	// need to check
	SlaveBMSSPIEnable_low();
	SPI_Write(0);
	SPI_Write(0);
	SPI_Write(0);
	SPI_Write(0);
	SPI_Write(0);
	SlaveBMSSPIEnable_high();
}
int SlaveBMSCellReadCommand(SlaveReg *s)
{
	int i;
	SlaveBms_WakeUp();
	SlaveBMSSPIEnable_low();
//	s->WError= LTC6804_write(s->ID, s->Command, 0, 0);

	if((s->WError!=0)&&(s->len !=0))
	{
		for (i = 0; i < s->len; i++) 
		{
			s->ADCX[i]= SPI_READ();
		}
		s->pecr = (SPI_READ() << 8) & 0xff00;
		s->pecr |= (SPI_READ() & 0x00ff);
  //		s->pecg = pec15(s->ADCX[i], s->len);
		if (s->pecr ==s->pecg) 
		{
			s->RError=0;
			s->ErrorCount= 0;
		} 
		else 
		{
			s->RError=1;
			s->ErrorCount++;
		}
	}
	if(s->RError==1)
	{
		if(s->Command==LTC6804_CMD_RDCVA)
		{
			s->ErrorCode=LTC6804_ERROR_RDCVA;
		}
		if(s->Command==LTC6804_CMD_RDCVB)
		{
			s->ErrorCode=LTC6804_ERROR_RDCVB;
		}
		if(s->Command==LTC6804_CMD_RDCVC)
		{
			s->ErrorCode=LTC6804_ERROR_RDCVC;
		}
		if(s->Command==LTC6804_CMD_RDCVD)
		{
			s->ErrorCode=LTC6804_ERROR_RDCVD;
		}
	}
	SlaveBMSSPIEnable_high();
	return s->ErrorCode;	
}
//-----------------------------------------------------------------------
/*signed int read_ringbuffer(ModbusRegs *m)
{
    m->Action_pointer++;
    if(m->Action_pointer > ring_buffer_max)  //ring_buffer max = 40
    {
        m->Action_pointer = 0;
    }
    return(m->Ring_buffer[m->Action_pointer]);
}


void Get_Word(ModbusRegs *m)
{
    unsigned char wh,wl;
    wh = read_ringbuffer();
    wl = read_ringbuffer();
    return(wh<<8) + wl;
}
*/


unsigned char auchCRCHi[256] =
{
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40
};

unsigned char auchCRCLo[256] =
{
0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4,
0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD,
0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7,
0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE,
0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2,
0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB,
0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91,
0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88,
0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80,
0x40
};
void SlaveVoltagHandler(SlaveReg *s)
{
    if(s->SysMachine==STATE_RUNNING)
    {
       s->Error = LTC6804_write_cmd(s->ID,LTC6804_CMD_ADCV | (1 << 8)|(0 << 4)|(0 << 0),0, 0);
       if(s->Error==1)
       {
           s->ErrorCount=0;
       }
       else
       {
           s->ErrorCount++;
       }
       s->Error  = LTC6804_read_cmd(s->ID,LTC6804_CMD_RDCVA, s->ADCX, 6);
       if(s->Error==1)
       {
           s->ErrorCount=0;
           s->CellVoltageBuf[0]  = ((s->ADCX[1] << 8) & 0xff00) | (s->ADCX[0]  & 0x00ff);
           s->CellVoltageBuf[1]  = ((s->ADCX[3] << 8) & 0xff00) | (s->ADCX[2]  & 0x00ff);
           s->CellVoltageBuf[2]  = ((s->ADCX[5] << 8) & 0xff00) | (s->ADCX[4]  & 0x00ff);

       }
       else
       {
           s->ErrorCount++;
       }
       s->Error = LTC6804_read_cmd(s->ID,LTC6804_CMD_RDCVB, s->ADCX, 6);
       if(s->Error==1)
       {
           s->ErrorCount=0;
           s->CellVoltageBuf[3] = ((s->ADCX[1] << 8 & 0xff00)  | (s->ADCX[0]) & 0x00ff);
           s->CellVoltageBuf[4] = ((s->ADCX[3] << 8 & 0xff00)  | (s->ADCX[2]) & 0x00ff);
           s->CellVoltageBuf[5] = ((s->ADCX[5] << 8 & 0xff00)  | (s->ADCX[4]) & 0x00ff);

       }
       else
       {
           s->ErrorCount++;
       }

       s->Error = LTC6804_read_cmd(s->ID,LTC6804_CMD_RDCVC, s->ADCX, 6);
       if(s->Error==1)
       {
           s->ErrorCount=0;
           s->CellVoltageBuf[6] = ((s->ADCX[1] << 8 & 0xff00)  | (s->ADCX[0]) & 0x00ff);
           s->CellVoltageBuf[7] = ((s->ADCX[3] << 8 & 0xff00)  | (s->ADCX[2]) & 0x00ff);
           s->CellVoltageBuf[8] = ((s->ADCX[5] << 8 & 0xff00)  | (s->ADCX[4]) & 0x00ff);
           // 소수정 3자리 변환

       }
       else
       {
           s->ErrorCount++;
       }
       s->Error = LTC6804_read_cmd(s->ID,LTC6804_CMD_RDCVD, s->ADCX, 6);
       if(s->Error==1)
       {
           s->ErrorCount=0;
           s->CellVoltageBuf[9]  = ((s->ADCX[1] << 8 & 0xff00)  | (s->ADCX[0]) & 0x00ff);
           s->CellVoltageBuf[10] = ((s->ADCX[3] << 8 & 0xff00)  | (s->ADCX[2]) & 0x00ff);
           s->CellVoltageBuf[11] = ((s->ADCX[5] << 8 & 0xff00)  | (s->ADCX[4]) & 0x00ff);
           // 소수정 3자리 변환

       }
       else
       {
           s->ErrorCount++;
       }
       s->CellVoltage[0]    = round(s->CellVoltageBuf[0]/10);
       s->CellVoltage[1]    = round(s->CellVoltageBuf[1]/10);
       s->CellVoltage[2]    = round(s->CellVoltageBuf[2]/10);
       s->CellVoltage[3]    = round(s->CellVoltageBuf[3]/10);
       s->CellVoltage[4]    = round(s->CellVoltageBuf[4]/10);
       s->CellVoltage[5]    = round(s->CellVoltageBuf[5]/10);
       s->CellVoltage[6]    = round(s->CellVoltageBuf[6]/10);
       s->CellVoltage[7]    = round(s->CellVoltageBuf[7]/10);
       s->CellVoltage[8]    = round(s->CellVoltageBuf[8]/10);
       s->CellVoltage[9]    = round(s->CellVoltageBuf[9]/10);
       s->CellVoltage[10]   = round(s->CellVoltageBuf[10]/10);
       s->CellVoltage[11]   = round(s->CellVoltageBuf[11]/10);

    }
}

unsigned int MakeCRC(unsigned char *pStr,int Len)
{
    unsigned char uchCRCHi = 0xFF;
    unsigned char uchCRCLo = 0xFF;
    unsigned short uIndex;

    while(Len--)
    {
        uIndex = uchCRCHi ^ (*pStr++ & 0xff);
        uchCRCHi = uchCRCLo ^ auchCRCHi[uIndex];
        uchCRCLo = auchCRCLo[uIndex];
    }
    return (uchCRCHi << 8 | uchCRCLo);
}

void SciaTxchar(char Txchar)
{
    while(!(SciaTxReadyFlag));
    SciaRegs.SCITXBUF=Txchar;
}


void scia_msg(char * msg)
{
    int i;
    i = 0;
    while(msg[i] != '\0')
    {
        SciaTxchar(msg[i]);
        i++;
    }
}


void CANATX(unsigned int ID, unsigned char Length, unsigned int Data0, unsigned int Data1,unsigned int Data2,unsigned int Data3)
{
    struct ECAN_REGS ECanaShadow;
    unsigned int CANWatchDog;
    unsigned int Data0Low, Data0High, Data1Low, Data1High;
    unsigned int Data2Low, Data2High, Data3Low, Data3High;
    CANWatchDog=0;
 //   unsigned int CANBUSOFF=0;

    Data0Low  = 0x00ff&Data0;
    Data0High = 0x00ff&(Data0>>8);
    Data1Low  = 0x00ff&Data1;
    Data1High = 0x00ff&(Data1>>8);
    Data2Low  = 0x00ff&Data2;
    Data2High = 0x00ff&(Data2>>8);
    Data3Low  = 0x00ff&Data3;
    Data3High = 0x00ff&(Data3>>8);

    EALLOW;
    ECanaShadow.CANME.bit.ME31=0;
    ECanaRegs.CANME.bit.ME31= ECanaShadow.CANME.bit.ME31;

    ECanaMboxes.MBOX31.MSGID.bit.STDMSGID=ID;

    ECanaShadow.CANME.bit.ME31=1;
    ECanaRegs.CANME.bit.ME31= ECanaShadow.CANME.bit.ME31;
    EDIS;

    ECanaMboxes.MBOX31.MSGCTRL.bit.DLC=Length;
    ECanaMboxes.MBOX31.MDL.byte.BYTE0=Data0Low;
    ECanaMboxes.MBOX31.MDL.byte.BYTE1=Data0High;
    ECanaMboxes.MBOX31.MDL.byte.BYTE2=Data1Low;
    ECanaMboxes.MBOX31.MDL.byte.BYTE3=Data1High;
    ECanaMboxes.MBOX31.MDH.byte.BYTE4=Data2Low;
    ECanaMboxes.MBOX31.MDH.byte.BYTE5=Data2High;
    ECanaMboxes.MBOX31.MDH.byte.BYTE6=Data3Low;
    ECanaMboxes.MBOX31.MDH.byte.BYTE7=Data3High;


    ECanaShadow.CANTRS.all=0;
    ECanaShadow.CANTRS.bit.TRS31= 1;
    ECanaRegs.CANTRS.all = ECanaShadow.CANTRS.all;
    do
    {
      ECanaShadow.CANTA.all = ECanaRegs.CANTA.all;
      CANWatchDog++;
      if(CANWatchDog>20)
      {
          ECanaShadow.CANTA.bit.TA31=0;
       //   ServiceDog();
          CANWatchDog=0;
      }
  //    ECanaShadow.CANES.all = ECanaRegs.CANES.all;
    }while(!ECanaShadow.CANTA.bit.TA31);
    //CAN Tx Request
     WG_InitECana();
     ECanaShadow.CANTA.all = 0;
     ECanaShadow.CANTA.bit.TA31=1;                   // Clear TA5
     ECanaRegs.CANTA.all = ECanaShadow.CANTA.all;


}//EOF
void temperatureRelayHandler(SystemReg *sys)
{
    if(sys->CellMaxTemperature>350)
    {
       sys->TemperatureRelayCount++;
       if(sys->TemperatureRelayCount>=100)
       {
           sys->DigitalOutPutReg.bit.FAN =1;
           sys->TemperatureRelayCount>=1005;
       }
    }
    else
    {
        sys->DigitalOutPutReg.bit.FAN =1;
        sys->TemperatureRelayCount=0;
    }
}


//----------------------------------------------------------------------------------�뜝�럥�맶�뜝�럥�쑅�뜝�럥�뵒�뜝�럥�맶�뜝�럥吏쀥뜝�럩援꿨뜝�럥�맶�뜝�럥竊놬PUT/OUTPUT 占쎈쐻占쎈윞占쎈뭼�뜝�럥�맶�뜝�럥�쑋�댖怨ㅼ삕 �뜝�럥�맶�뜝�럥�몘力놂옙�⑥궡�굲�뜝�럥臾�





