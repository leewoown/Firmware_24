
// TI File $Revision: /main/1 $
// Checkin $Date: December 5, 2008   18:00:32 $
//###########################################################################
//
// FILE:   DSP28x_Project.h
//
// TITLE:  DSP28x Project Headerfile and Examples Include File
//
//###########################################################################
// $TI Release: 2803x C/C++ Header Files V1.21 $
// $Release Date: December 1, 2009 $
//###########################################################################

#ifndef DSP28x_PROJECT_H
#define DSP28x_PROJECT_H

//컴파일러 매크로 선언 --------------------------------------------------------------------//

#include "DSP2803x_Device.h"     // DSP2803x Headerfile Include File
#include "DSP2803x_Examples.h"   // DSP2803x Examples Include File
#include "stdio.h"
#include "string.h"
#include "math.h"

//EDLAY 매크로 선언 --------------------------------------------------------------------//

// TI SDK 1.10의 소스 DSP2803x_usDelay.asm에서 제공하는 DELAY_US 함수를 사용
#define     delay_us(us)		DELAY_US(us)

// TI SDK 1.10의 소스 DSP2803x_usDelay.asm에서 제공하는 DELAY_US 함수를 사용
#define     delay_ms(ms)		DELAY_US(ms*1000)

//GPIO 매크로 선언 -----------------------------------------------------------------//
#define 	GPIO					0
#define 	QEPA					1
#define 	QEPB					1
#define 	SCIRX					1
#define 	SCITX					1
#define 	CANRX					1
#define 	CANTX					1
#define		SPI						1
#define 	SDDA					1
#define 	SCLK					1
#define 	INPUT					0
#define 	OUTPUT					1
#define 	Default					0
#define   	Disable 				0
#define  	LTC6804_ENABLE			1
#define  	ADC_ENABLE  			2
#define  	EPROM_ENABLE			3

#define     Shift_RIGHT(val, bit)   ((val) >> (al))
#define     Shift_LEFT(val,  bit)   ((val) << (val))
#define     ComBine(Val_H, Val_L)   (((Val_H) << 8) |(Val_L))
#define     BIT_MASK(bit)           (1 << (bit))
#define     GetBit(val, bit)        (((val) & BIT_MASK(bit)) >> (bit))
#define     SetBit(val, bit)        (val |= BIT_MASK(bit))
#define     ClearBit(val, bit)      (val &= ~BIT_MASK(bit))
#define     ToggleBit(val, bit)     (val ^= BIT_MASK(bit))
#define     bit_is_set(val, bit)    (val & BIT_MASK(bit))
#define     bit_is_clear(val, bit)  (~val & BIT_MASK(bit))

#define     TEMP1DO00H               GpioDataRegs.GPASET.bit.GPIO0=1;
#define     TEMP1DO01H               GpioDataRegs.GPASET.bit.GPIO1=1;
#define     TEMP1DO02H               GpioDataRegs.GPASET.bit.GPIO2=1;

#define     TEMP1DO00L               GpioDataRegs.GPACLEAR.bit.GPIO0=1;
#define     TEMP1DO01L               GpioDataRegs.GPACLEAR.bit.GPIO1=1;
#define     TEMP1DO02L               GpioDataRegs.GPACLEAR.bit.GPIO2=1;

#define     TEMP2DO00H               GpioDataRegs.GPASET.bit.GPIO3=1;
#define     TEMP2DO01H               GpioDataRegs.GPASET.bit.GPIO4=1;
#define     TEMP2DO02H               GpioDataRegs.GPASET.bit.GPIO5=1;

#define     TEMP2DO00L               GpioDataRegs.GPACLEAR.bit.GPIO3=1;
#define     TEMP2DO01L               GpioDataRegs.GPACLEAR.bit.GPIO4=1;
#define     TEMP2DO02L               GpioDataRegs.GPACLEAR.bit.GPIO5=1;

#define     TEMP3DO00H               GpioDataRegs.GPASET.bit.GPIO6=1;
#define     TEMP3DO01H               GpioDataRegs.GPASET.bit.GPIO7=1;
#define     TEMP3DO02H               GpioDataRegs.GPASET.bit.GPIO8=1;

#define     TEMP3DO00L               GpioDataRegs.GPACLEAR.bit.GPIO6=1;
#define     TEMP3DO01L               GpioDataRegs.GPACLEAR.bit.GPIO7=1;
#define     TEMP3DO02L               GpioDataRegs.GPACLEAR.bit.GPIO8=1;

#define     RS485_CS                 GpioDataRegs.GPACLEAR.bit.GPIO9=1;
#define     RS485_DS                 GpioDataRegs.GPASET.bit.GPIO9=1;

#define     LTC680x_CS               GpioDataRegs.GPACLEAR.bit.GPIO10=1;
#define     LTC680X_DS               GpioDataRegs.GPASET.bit.GPIO10=1;

#define     SPITEST1_CS              GpioDataRegs.GPACLEAR.bit.GPIO11=1;
#define     SPITEST1_DS              GpioDataRegs.GPASET.bit.GPIO11=1;

#define     SPITEST2_CS              GpioDataRegs.GPASET.bit.GPIO12=1;
#define     SPITEST2_DS              GpioDataRegs.GPACLEAR.bit.GPIO12=1;

#define     FAN_ON                   GpioDataRegs.GPASET.bit.GPIO21=1;
#define     FAN_OFF                  GpioDataRegs.GPACLEAR.bit.GPIO21=1;
#define     FAN_Toggle               GpioDataRegs.GPATOGGLE.bit.GPIO21=1;


//#define     FANSTATE               GpioDataRegs.GPADAT.bit.GPIO22
#define     WakeleakState            GpioDataRegs.GPADAT.bit.GPIO22

#define     LEDSTATE_ON              GpioDataRegs.GPASET.bit.GPIO23=1;    //MAIN_Relay
#define     LEDSTATE_OFF             GpioDataRegs.GPACLEAR.bit.GPIO23=1;
#define     LEDSTATE_Toggle          GpioDataRegs.GPATOGGLE.bit.GPIO23=1;

#define     LEDCAN_ON                GpioDataRegs.GPASET.bit.GPIO24=1;
#define     LEDCAN_OFF               GpioDataRegs.GPACLEAR.bit.GPIO24=1;
#define     LEDCAN_Toggle            GpioDataRegs.GPATOGGLE.bit.GPIO24=1;

//#define   WAKEUP                   GpioDataRegs.AIODAT.bit.AIO10          // WAKEUP GPIO3  -> AIO10 ID_SW00
#define     ID_SW00                  GpioDataRegs.AIODAT.bit.AIO2
#define     ID_SW01                  GpioDataRegs.AIODAT.bit.AIO4
#define     ID_SW02                  GpioDataRegs.AIODAT.bit.AIO6
#define     ID_SW03                  GpioDataRegs.AIODAT.bit.AIO10
#define 	ID_SW04       	    	 GpioDataRegs.AIODAT.bit.AIO12


#define     C_CellVoltageNum            24
#define     C_CellTemperatureNum        24
#define     C_SWVer                     1
#define     C_NorVoltage                883
#define     C_Capacity                  1000

#define     C_CellBalacneShift          11
#define     C_CellBalanceNum            2

//Slave ID
#define     C_Slave1_ID               0x0
#define     C_Slave2_ID               0xf

#define     C_Slave1CellTemperatureNum    8
#define     C_Slave2CellTemperatureNum    8
#define     C_Slave3CellTemperatureNum    8
#define     C_Slave1CellVoltageNum        12
#define     C_Slave2CellVoltageNum        12


#define     C_CTSensorCOMCount           800 // "통신 TimeOut //1sec
#define     C_MASTERCOMCount             800 // "통신 TimeOut //1sec
#define     C_shutdownVoltage            2850  //1 --> 2.7V
#define     C_shutdownTemperature       -300  //1 --> 2.7V


// Cell Balancing/
#define     C_BalanceStartDiffVoltage       0.01 //24
#define     C_BalanceENDDiffVoltage         0.005 //25
#define     C_BalanceCountStart             330
#define     C_BalanceCountStop              660
#define     C_BalanceDivVoltage             5
#define     C_FANOnTemperature              350
#define     C_FANOffTemperature             300
#define     C_BATICComErrCount              50
// CAN Current Sensor
#define     C_CTDirection                   1.0
//
#define     AdcNormalizerBipolar            0.00048828125           // 1/2048
#define     AdcNormalizerUnipolar           0.000244140625          // 1/4096

/*
 * CPU SET UP
 */
//CPU 매크로선언 ------------------------------------------------------------------//
#define	    CPUCLK            	     60000000L						    // CPU Main Clock
#define     RackNum                  2
//SCIA 매크로 선언 ----------------------------------------------------------------//
#define		SCIA_LSPCLK			     (CPUCLK/2)							// Peripheral Low Speed Clock for SCI-B  //LOSPCP에서 설정
#define		SCIA_BAUDRATE		     115200L							// SCI-A Baudrate
#define		SCIA_BRR_VAL		     (SCIA_LSPCLK/(8*SCIA_BAUDRATE)-1)	// SCI-A BaudRate 설정 Register 값
#define 	SciaTxReadyFlag          SciaRegs.SCICTL2.bit.TXRDY         //


//CAN Baudrate 설정
#define 	CAN_1MBPS			     0
#define 	CAN_500KBPS 		     1
#define     CAN_250KBPS              0
//CAN LAM MASK BIT SETTING
//#define 	_LAM(CANADDR)   	    ((~(CANADDR) & 0x07FF)<<2) & 0x1FFD
//CAN GROUP ID
//#define     CAN_BIT_MASK            0x0700
//#define     CAN_ID_0x0000           0x0500
//#define     CAN_ID_0x0100           0x0100
//#define     CAN_ID_0x0200           0x0200
//#define     CAN_ID_0x0300           0x0300

// 유효 숫자 4자리 

/*
 * LTC 6804-2 COM COMMAND
 */

#define     LTC6804_CMD_WRCFG				    (0x0001)
#define     LTC6804_CMD_RDCFG				    (0x0002)
#define     LTC6804_CMD_RDCVA				    (0x0004)
#define     LTC6804_CMD_RDCVB				    (0x0006)
#define     LTC6804_CMD_RDCVC				    (0x0008)
#define     LTC6804_CMD_RDCVD				    (0x000a)
#define     LTC6804_CMD_ADCV				    (0x0260)
#define     LTC6804_CMD_RDAUXA				    (0x000c)
#define     LTC6804_CMD_RDAUXB				    (0x000e)
#define     LTC6804_CMD_ADAX				    (0x0460)
#define     LTC6804_CMD_CLRAUX				    (0x0712)
#define     LTC6804_CMD_ADCVAX				    (0x046F)

#define     LTC6804_ERROR_WRCFG				    (0x0001)
#define     LTC6804_ERROR_RDCFG				    (0x0002)
#define     LTC6804_ERROR_RDCVA				    (0x0004)
#define     LTC6804_ERROR_RDCVB				    (0x0006)
#define     LTC6804_ERROR_RDCVC				    (0x0008)
#define     LTC6804_ERROR_RDCVD				    (0x000a)
#define     LTC6804_ERROR_ADCV				    (0x0260)

/*
 *  RS485 Modbus defien
 */
#define ring_buffer_max                     40
#define HMI_WORD_TXDATA                     030306000100020003

typedef enum
{
  STATE_INIT_REG,
  STATE_STANDBY,
  STATE_RUNNING,
  STATE_SALVEING,
  STATE_FAULT,
  STATE_CLEAR
}SysState;

struct DigitalInPut_BIT
{     	// bits   description
   unsigned int 	Fanstates		          :1;   // 0
   unsigned int     Wakeleakstates            :1;   // 0
};
union DigitalInput_REG
{
   unsigned int     all;
   struct DigitalInPut_BIT bit;
};

struct DigitalOutPut_BIT
{     	// bits   description
   unsigned int     FAN                 :1; // 10
   unsigned int     LEDCAN              :1; // 10
   unsigned int     LEDSTATE            :1; // 10
};
union DigitalOutPut_REG
{
   unsigned int     all;
   struct DigitalOutPut_BIT bit;
};

struct SystemStateA_BIT
{     	// bits   description
    unsigned int     INITOK                     :1;    // 0
    unsigned int     Fault                      :1;    // 1
    unsigned int     BalanceStartStop           :1;    // 2
    unsigned int     WaterleakFault             :1;    // 3, FAN Delection
    unsigned int     CellVoltageFault           :1;    // 4
    unsigned int     CellTemperatureFault       :1;    // 5
    unsigned int     BATIC1ErrFault             :1;    // 6
    unsigned int     CTCOMErrFault              :1;    // 7
    unsigned int     MBCOMErrFault              :1;    // 8
    unsigned int     HMICOMErrFault             :1;    // 9
    unsigned int     BalanceEnable              :1;    // 8CellVoltCAN
    unsigned int     CellVoltCAN                :1;    // 8HMICOMErrFault
    unsigned int     CellTempCAN                :1;    // 8
    unsigned int     HmiEn                      :1;    // 8
    unsigned int     PackEn                     :1;    // 8
    unsigned int     NotUsed15                  :1;    // 8
};
union SystemStateA_REG
{
   unsigned int     all;
   struct SystemStateA_BIT bit;
};

struct BMSID_BIT
{       // bits   description
    unsigned int     BMS_ID00      :1; // 0
    unsigned int     BMS_ID01      :1; // 1
    unsigned int     BMS_ID02      :1; // 2
    unsigned int     BMS_ID03      :1; // 3
    unsigned int     BMS_ID_00     :1; // 4
    unsigned int     BMS_ID_01     :1; // 5
    unsigned int     BMS_ID_02     :1; // 6
    unsigned int     BMS_ID_03     :1; // 7
    unsigned int     BMS_ID_04     :1; // 8
    unsigned int     BMS_ID09      :1; // 9
    unsigned int     BMS_ID10      :1; // 10
    unsigned int     BMS_ID11      :1; // 11
    unsigned int     BMS_ID12      :1; // 12
    unsigned int     BMS_ID13      :1; // 13
    unsigned int     BMS_ID14      :1; // 14
    unsigned int     BMS_ID15      :1; // 15
};
union BMSID_BIT_REG
{
   unsigned int     all;
   struct BMSID_BIT bit;
};


struct Current_byte
{
    unsigned int CurrentL;
    unsigned int CurrentH;
};
union Currnet_Reg
{
    long                all;
    struct Current_byte byte;
};
struct BalanceSate_BIT
{       // bits   description
   unsigned int     B_Cell00        :1; // 00
   unsigned int     B_Cell01        :1; // 01
   unsigned int     B_Cell02        :1; // 02
   unsigned int     B_Cell03        :1; // 03
   unsigned int     B_Cell04        :1; // 04
   unsigned int     B_Cell05        :1; // 05
   unsigned int     B_Cell06        :1; // 06
   unsigned int     B_Cell07        :1; // 07
   unsigned int     B_Cell08        :1; // 08
   unsigned int     B_Cell09        :1; // 09
   unsigned int     B_Cell10        :1; // 10
   unsigned int     B_Cell11        :1; // 11
   unsigned int     B_Cell12        :1; // 12
   unsigned int     B_Cell13        :1; // 13
   unsigned int     B_Cell14        :1; // 14
   unsigned int     B_Cell15        :1; // 15
   unsigned int     B_Cell16        :1; // 16
   unsigned int     B_Cell17        :1; // 17
   unsigned int     B_Cell18        :1; // 18
   unsigned int     B_Cell19        :1; // 19
   unsigned int     B_Cell20        :1; // 20
   unsigned int     B_Cell21        :1; // 21
   unsigned int     B_Cell22        :1; // 22
   unsigned int     B_Cell23        :1; // 23
   unsigned int     B_Cell24        :1; // 24
   unsigned int     B_Cell25        :1; // 25
   unsigned int     B_Cell26        :1; // 26
   unsigned int     B_Cell27        :1; // 27
   unsigned int     B_Cell28        :1; // 28
   unsigned int     B_Cell29        :1; // 29
   unsigned int     B_Cell30        :1; // 30
   unsigned int     B_Cell31        :1; // 31
};
struct BalanceSate_Word
{
    Uint16      LOW_WORD:16; // 0:15
    Uint16      HI_WORD:16;  // 31:16
};
union BalanceSate_REG
{
   Uint32                       all;
   struct BalanceSate_BIT       bit;
   struct BalanceSate_Word      Word;
};
struct HMIRequet_BIT
{       // bits   description
    unsigned int     HMIComEn             :1;    // 00
    unsigned int     VoltReq              :1;    // 01
    unsigned int     TempsReq             :1;    // 02
    unsigned int     BATMD03              :1;    // 03
    unsigned int     BATMD04              :1;    // 04
    unsigned int     BATMD05              :1;    // 05
    unsigned int     BATMD06              :1;    // 06
    unsigned int     BATMD07              :1;    // 07
    unsigned int     BATMD08              :1;    // 08
    unsigned int     BATMD09              :1;    // 09
    unsigned int     BATMD10              :1;    // 10
    unsigned int     BATMD11              :1;    // 11
    unsigned int     BATMD12              :1;    // 12
    unsigned int     BATMD13              :1;    // 12
    unsigned int     BATMD14              :1;    // 14
    unsigned int     BATMD15              :1;    // 15
};
union HMIRequet_REG
{
   unsigned int     all;
   struct HMIRequet_BIT bit;
};
struct PackStatus_BIT
{       // bits   description
    unsigned int     PackStatus           :3;    //
    unsigned int     BATMD03              :1;    // 03
    unsigned int     BATMD04              :1;    // 04
    unsigned int     BATMD05              :1;    // 05
    unsigned int     BATMD06              :1;    // 06
    unsigned int     BATMD07              :1;    // 07
    unsigned int     PackComEn            :1;    // 08
    unsigned int     PackReset            :1;    // 09
    unsigned int     BATMD10              :1;    // 10
    unsigned int     BATMD11              :1;    // 11
    unsigned int     BATMD12              :1;    // 12
    unsigned int     BATMD13              :1;    // 12
    unsigned int     BATMD14              :1;    // 14
    unsigned int     BATMD15              :1;    // 15
};
union PackStatus_REG
{
   unsigned int          all;
   struct PackStatus_BIT bit;
};
struct BatteryBalance_BIT
{       // bits   description
   unsigned int     B_Cell00        :1; // 00
   unsigned int     B_Cell01        :1; // 01
   unsigned int     B_Cell02        :1; // 02
   unsigned int     B_Cell03        :1; // 03
   unsigned int     B_Cell04        :1; // 04
   unsigned int     B_Cell05        :1; // 05
   unsigned int     B_Cell06        :1; // 06
   unsigned int     B_Cell07        :1; // 07
   unsigned int     B_Cell08        :1; // 08
   unsigned int     B_Cell09        :1; // 09
   unsigned int     B_Cell10        :1; // 10
   unsigned int     B_Cell11        :1; // 11
   unsigned int     B_Cell12        :1; // 12
   unsigned int     B_Cell13        :1; // 13
   unsigned int     B_Cell14        :1; // 14
   unsigned int     B_Cell15        :1; // 15
};
union BatteryBalance_REG
{
   unsigned int     all;
   struct BatteryBalance_BIT    bit;
};
typedef struct System_Date
{
    //Status
    SysState     SysMachine;
    Uint16       SysVauleInit;
    Uint16       initCount;
    Uint16       MainIsr;
    Uint16       CpuTimer0Loop;
    Uint16       CanRxLoop;
    Uint16       Timer50msec;
    Uint16       Timer100msec;
    Uint16       Timer200msec;
    Uint16       Timer500msec;
    Uint16       Timer1000msec;
    Uint16       CellVoltsampling;
    Uint16       CellTempSampling;
    Uint16       Timer2000msec;
    Uint16       TimerBalance;
    Uint16       Number;
    Uint16       TempNumber;
    Uint16       BalanceStartCount;

    Uint32       CellSumVoltageBuf;
    Uint32       CellSumVoltage;
    Uint16       ModuleVoltage;

    float32      ModuleCurrentF;
    float32      ModuleCurrentabsF;


    Uint16       CellVoltage[24];
    Uint16       CellVoltageCount;
    Uint16       CellMaxVoltage;
    Uint16       CellMaxVoltageNum;
    Uint16       CellMinVoltage;
    Uint16       RackCellMinVoltage;

    Uint16       BalaCellMinVolt;
    Uint16       HMICellMinVolt;
    Uint16       RackCellMinVolt;


    Uint16       RackCellDivVoltage;
    Uint16       CellMinVoltageNum;
    Uint16       CellAvgVoltage;
    Uint16       CellDivVoltage;
    Uint16       CellVoltDivCnt;
    Uint16       CellVoltDivCntVaule;
    Uint16       CellVoltagePosition;
    Uint16       CellVoltagePositionbuf1;
    Uint16       CellVoltagePositionbuf2;

    Uint16       CellTempertureCount;
    int16        CellTemperture[24];
    int16        ModuleTemperature;
    int16        CellMaxTemperature;
    int16        CellMaxTemperatureNum;
    int16        CellMinTemperature;
    int16        CellMinTemperatureNum;
    int16        CellAvgTemperature;
    int16        CellDivTemperature;
    Uint16       CellTempPosition;
    Uint16       CellOffsetVotageDiv;


    int16        CellBalanceState[C_CellBalanceNum];

    Uint16       CurrentCount;
    Uint16       MasterCount;
    Uint16       HMICount;
    Uint16       BATICErrCount[2];

    Uint16       SciRxflag;
    Uint16       HeartBeat;
    Uint16       BalanceState;
    Uint16       BalanceMode;
    Uint16       LEDCanCount;

    union        DigitalInput_REG           DigitalInputReg;
    union        DigitalOutPut_REG          DigitalOutPutReg;
    union        SystemStateA_REG           SystemStateARegs;
    union        BMSID_BIT_REG              BMSIDRegs;
    union        Currnet_Reg                CurrentData;
    union        HMIRequet_REG              HMIREQ;
    union        PackStatus_REG             PackStatus;
}SystemReg;

typedef struct Slave_DATA																							
{
	
    SysState     SysMachine;

	short  Pec1;
	short  Pec2;
	short  CommandArrey[5]; 
	short  Command;
	unsigned short pecg;
	unsigned short pecr;
	char   ID;
	char   InitTable[6];
	char   BalanceTable[6];
	char   ADCX[6];
	char   ADCV[6];
	char   CommandBufbuffer[32];
	unsigned int	 len;
	unsigned int 	 ErrorCode;
	unsigned int	 ErrorCount;
	unsigned int     RError;
	unsigned int 	 WError;
	unsigned int 	 Error;
	unsigned int 	 CellVoltage[12];
	unsigned int     CellVoltageBuf[12];
	int              DivVoltage[12];
//
    float32          CellTempF[8];
    float32          CellTempV[8];
    float32          ADCTEMPBuf[8];
	int 	         CellTemp[8];
	unsigned int 	 CellTempBuffer[8];
    Uint16           BalaCellVolt;
//	float32 		ForceBalanceVoltage;
	float32         tmpF0;
	float32         X03;
	float32         X02;
	float32         X01;
    union BatteryBalance_REG    Balance;

}SlaveReg;

typedef struct CANTX_DATA
{

	unsigned int BatteryVoltageCell[24];
	int BatteryTempCell[24];

	Uint16 CellNumStart;
	Uint16 NumberShift;
	Uint16 CellVotlNum;
	Uint16 CellVoltageNumCnt;
	Uint16 CellVotlageMaxMinNum;

	Uint16 CellNumTempStart;
	Uint16 NumberTempShift;
	Uint16 CellTemNum;
	Uint16 CellTemNumCnt;
	Uint16 CellTempsMaxMinNum;

    Uint16 UnitBMSID;
    Uint16 UnitBMSIfro;
 //   Uint16 CAN_STATUS;
    Uint16 ErrorCount;
    Uint16 Timer1000msec;
    Uint16 Heartbeat;
    Uint16 BMAVlotTempsQty;
    Uint16 BMASWVer;
//    Uint16 CANTXTime1000msec;
    union BalanceSate_REG  BalanceSate;

}CANTXReg;

typedef struct CANRX_DATA																							
{
	Uint16	RackCellMinVoltage; // Pack or Rack
    Uint16  RackCellAgvTemps; // Pack or Rack
    Uint16  HMICellMinVoltage; // Pack or RacK
	Uint16	HMICellAgvTemps; // Pack or RacK
    Uint16  CurrentCount;
    Uint16  MasterCount;
    Uint16  LEDRXCount;
    Uint16  HmiCount;
	Uint16  RxCount;
	union HMIRequet_REG HMIREQ;
	union PackStatus_REG PackStatus;

}CANRXReg;

typedef struct ADCREG_DATA
{
	Uint16 ADCA00buf;
	Uint16 ADCA01buf;
	Uint16 ADCA02buf;
	Uint16 ADCA03buf;
	Uint16 ADCA04buf;
	Uint16 ADCA05buf;
	Uint16 ADCA06buf;
	Uint16 ADCA07buf;

	Uint16 ADCB00buf;
	Uint16 ADCB01buf;
	Uint16 ADCB02buf;
	Uint16 ADCB03buf;
	Uint16 ADCB04buf;
	Uint16 ADCB05buf;
	Uint16 ADCB06buf;
	Uint16 ADCB07buf;

	float PackTemp;
	float PackTempReal;

}ADCReg;

///////////////////////////////////////////////////////////////////////////////
//************************************************************************************//
//                                  System Interrupt                                  //
//************************************************************************************//
interrupt void cpu_timer0_isr(void);
interrupt void ISR_CANRXINTA(void);
interrupt void ISR_SCIRXINTA(void);

//************************************************************************************//
//                                extern function                                     //
//************************************************************************************//
extern int LTC6804_Init(void);
extern int SlaveBMSIint(SlaveReg *s);
extern int SlaveBmsBalance(SlaveReg *s);
extern int LTC6804_write_cmd(char address, short command, char data[], int len);
extern int LTC6804_read_cmd(char address, short command, char data[], int len);

//************************************************************************************//
//                                  System Init                                       //
//************************************************************************************//
extern void WG_InitGpio(void);
extern void WG_InitSci(void);
extern void WG_InitECana(void);
extern void WG_InitSPI(void);
extern void InitAdc(void);
extern void SystemRegsInit(SystemReg *s);
extern void CANRxRegsInit(CANRXReg *P);
extern void CANTxRegsInit(CANTXReg *P);
extern void SystemInit(SystemReg *s);

//************************************************************************************//
//                                   Slave Ctrl                                       //
//************************************************************************************//
extern void SlaveBMSSPIEnable_low(void);
extern void SlaveBMSSPIEnable_high(void);
extern void SlaveRegsInit(SlaveReg *P);
extern void SlaveBms_WakeUp(void);
extern int  SlaveBMSCellReadCommand(SlaveReg *s);
extern void BalanceCtrl(SlaveReg *P);

//************************************************************************************//
//                                  Communication                                     //
//************************************************************************************//
//CAN_Tx
void CANATX(unsigned int ID, unsigned char Length, unsigned int Data0, unsigned int Data1,unsigned int Data2,unsigned int Data3 );
//SCI_HMI
void SciaTxchar(char Txchar);
void scia_msg(char * msg);

unsigned int MakeCRC(unsigned char *pStr,int Len);


void SPI_Write(unsigned int WRData);
unsigned int SPI_READ(void);
//************************************************************************************//
//                                    ADC Cal                                         //
//************************************************************************************//
void ADC_Read(ADCReg *A);
void PackCurrentAsb(SystemReg *s);

//************************************************************************************//
//                                  System State                                      //
//************************************************************************************//
void DigitalInput(SystemReg *sys);
void DigitalOutput(SystemReg *sys);

//void Timer(SystemReg *s);

//************************************************************************************//
//                           Voltage & Temp State                                     //
//************************************************************************************//

void ModuleErrHandler(SystemReg *P);
void ModuleCurrent(SystemReg *s);
void CalVoltageHandler(SystemReg *s);
void CalTempHandler(SystemReg *s);
void CellTempsAdcHandler(int Ch, SlaveReg *s);
void TEMP1_CAL(SlaveReg *s,Uint16 num);
void TEMP2_CAL(SlaveReg *s,Uint16 num);
void TEMP3_CAL(SlaveReg *s,Uint16 num);
void SlaveVoltagHandler(SlaveReg *s);
void BalanceEnableHandle(SystemReg *P);
void BalanceMinVoltHandler(SystemReg *P);
void BalanceCtrlHandler(SlaveReg *P);
//************************************************************************************//
//                                  System Ctrl                                       //
//************************************************************************************//

void BMSIDHandler(SystemReg *sys);




#endif  // end of DSP28x_PROJECT_H definition
