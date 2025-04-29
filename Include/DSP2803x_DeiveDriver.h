
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

#include "DSP2803x_Device.h"     // DSP2803x Headerfile Include File
#include "DSP2803x_Examples.h"   // DSP2803x Examples Include File

//컴파일러 매크로 선언 --------------------------------------------------------------------//
#define 	PCS  		 	1      		 
#define 	ENERTSTREAM  	2           
#define   	BUILDLEVEL 		PCS

//EDLAY 매크로 선언 --------------------------------------------------------------------//

// TI SDK 1.10의 소스 DSP2803x_usDelay.asm에서 제공하는 DELAY_US 함수를 사용
#define delay_us(us)		DELAY_US(us)	 

// TI SDK 1.10의 소스 DSP2803x_usDelay.asm에서 제공하는 DELAY_US 함수를 사용
#define delay_ms(ms)		DELAY_US(ms*1000)




//MMISYSTEM 파라미터값 설정 --------------------------------------------------------------//


//GPIO 매크로 선언 -----------------------------------------------------------------//

#define GPIO			0
#define QEPA			1
#define QEPB			1
#define SCIRX			1
#define SCITX			1
#define CANRX			1
#define CANTX			1
#define SDDA			1
#define SCLK			1
#define INPUT			0
#define OUTPUT			1
#define Default			0

#define LedLatch00OFF	GpioDataRegs.GPACLEAR.bit.GPIO17=1
#define LedLatch00ON	GpioDataRegs.GPASET.bit.GPIO17=1

#define LedLatch01OFF	GpioDataRegs.GPASET.bit.GPIO18=1
#define LedLatch01ON	GpioDataRegs.GPACLEAR.bit.GPIO18=1

#define LCDRSLOW		GpioDataRegs.GPBCLEAR.bit.GPIO32=1
#define LCDRSHIGH		GpioDataRegs.GPBSET.bit.GPIO32=1

#define LCDEWHIGH		GpioDataRegs.GPBSET.bit.GPIO32=1	
#define LCDEWLOW		GpioDataRegs.GPBCLEAR.bit.GPIO32=1		
//CPU 매크로선언 ------------------------------------------------------------------//
#define	CPUCLK				60000000L							// CPU Main Clock
//LCD 매크로 선언 -----------------------------------------------------------------//
#define LCDClEAR		0X01
//SCIA 매크로 선언 ----------------------------------------------------------------//

#define	SCIA_LSPCLK			(CPUCLK/4)							// Peripheral Low Speed Clock for SCI-B  //LOSPCP에서 설정
#define	SCIA_BAUDRATE		115200L								// SCI-A Baudrate
#define	SCIA_BRR_VAL		(SCIA_LSPCLK/(8*SCIA_BAUDRATE)-1)	// SCI-ABaudRate 설정 Register 값

#define SciaTxReadyFlag     SciaRegs.SCICTL2.bit.TXRDY




//CAN Baudrate 설정
#define CAN_1MBPS	0
#define CAN_500KBPS 1

//MODBUS관련 Define
#define MOD_MSTR_ADDR			0x01
#define MOD_SLAV_ADDR			0x02
#define MOD_TX_READ_VI			0x00
#define MOD_FNCTN_READ_HR		0x03
#define MOD_SADDR_VI_REGS		2000
#define MOD_TOT_NUM_VI_REGS		  14
#define MOD_RESP_NO				0x55
#define MOD_RESP_YES			0xAA

//CAN LAM MASK BIT SETTING
#define _LAM(CANADDR)   ((~(CANADDR) & 0x07FF)<<2) & 0x1FFD

//CAN GROUP ID
#define CAN_BIT_MASK 0x07F0

#define CAN_EMS_GRID_0 0x0000  
#define CAN_EMS_GRID_1 0x0010 
#define CAN_PCS_GRID   0x0650
#define CAN_BMS_GRID   0x0030


//CAN ID/////////////////////////////////////////

//EMS
#define CAN_EMS_MODE_CMND 0x000
#define CAN_EMS_BAT_CMD   0x001


//MMI
#define CAN_EMS_INST_VOLT 0x061
#define CAN_EMS_INST_CURR 0x062
#define CAN_MMI_BMS_SEQ   0x065
#define CAN_MMI_PCS_STS   0x066
#define CAN_MMI_PCS_STS   0x066
#define CAN_MMI_PCS_CMD   0x06E  //PCS에 보내는 MODE 명령
#define CAN_MMI_BAT_CMD   0x06F  //PCS 또는 BMS에 보내는 MODE 명령

//PCS
#define CAN_PCS_INST_VOLT 0x650
#define CAN_PCS_INST_CURR 0x651
#define CAN_PCS_INST_TEMP 0x652
#define CAN_PCS_BMS_SEQ   0x653
#define CAN_PCS_BITS_STAT 0x654

//BMS
#define CAN_BMS_CHRG_DATA 0x030
#define CAN_BMS_STAT_DATA 0x032


///////////////////////////////////////////////////////////////////////////////
// from here sequence control 관련 define
//

#define PCS_MODE_1_CMD 0x0001
#define PCS_MODE_2_CMD 0x0002
#define PCS_MODE_3_CMD 0x0004	
#define PCS_MODE_4_CMD 0x0008
#define PCS_MODE_5_CMD 0x0010
#define PCS_MODE_6_CMD 0x0020
#define PCS_MODE_7_CMD 0x0040
#define PCS_MODE_9_CMD 0x0100

#define	PCS_CMD_INV_EBL	   0x0001
#define	PCS_CMD_BUC_EBL	   0x0002	
#define	PCS_CMD_BST_EBL	   0x0004	
#define	PCS_CMD_PVO_EBL	   0x0008	
#define	PCS_CMD_BMS_WAK	   0x0010

#define	PCS_CMD_INV_DSBL   0x0100
#define	PCS_CMD_BUC_DSBL   0x0200	
#define	PCS_CMD_BST_DSBL   0x0400	
#define	PCS_CMD_PVO_DSBL   0x0800	

#define EMS_MODE_1_ACK 0x0001
#define EMS_MODE_2_ACK 0x0002
#define EMS_MODE_3_ACK 0x0004	
#define EMS_MODE_4_ACK 0x0008
#define EMS_MODE_5_ACK 0x0010
#define EMS_MODE_6_ACK 0x0020
#define EMS_MODE_7_ACK 0x0040
#define EMS_MODE_9_ACK 0x0100

  
#define BAT_CURR_ZERODELTA 1  //current 0 범위

#define PCS_MODE2_GRDI_PWR 3000

//
// to here sequence control 관련 define
///////////////////////////////////////////////////////////////////////////////







extern unsigned char lcdtest;

struct DigitalInPut_BIT
{     	// bits   description
   unsigned int 	SW00	:1;		// 0  
   unsigned int 	SW01	:1;     // 1
   unsigned int 	SW02	:1;     // 2
   unsigned int  	SW03	:1;     // 3  
   unsigned int 	SW04	:1;     // 4 
   unsigned int 	SW05	:1;		// 5
   unsigned int 	SW06	:1;		// 6
   unsigned int 	SW07	:1;     // 7
   unsigned int 	SW08	:1;     // 8  
   unsigned int 	SW09	:1;     // 9
   unsigned int 	SW10	:1;     // 10
   unsigned int  	SW11	:1;     // 11
   unsigned int 	SW12	:1;     // 12  
   unsigned int 	SW13	:1;		// 13
   unsigned int 	SW14	:1;		// 14
   unsigned int 	SW15	:1;     // 15
};
union DigitalInput_REG
{
   unsigned int     all;
   struct DigitalInPut_BIT bit;
};

struct DigitalOutPut_BIT
{     	// bits   description
   unsigned int 	LED00	:1;	// 0  
   unsigned int 	LED01	:1;	// 1
   unsigned int 	LED02	:1;	// 2
   unsigned int  	LED03	:1;	// 3  
   unsigned int 	LED04	:1;	// 4 
   unsigned int 	LED05	:1;	// 5
   unsigned int 	LED06	:1;	// 6
   unsigned int 	LED07	:1;	// 7
   unsigned int 	LED08	:1;	// 8  
   unsigned int 	LED09	:1;	// 9
   unsigned int 	LED10	:1;	// 10
   unsigned int  	LED11	:1;	// 11
   unsigned int 	LED12	:1;	// 12  
   unsigned int 	LED13	:1;	// 13
   unsigned int 	LACH00	:1;	// 14
   unsigned int 	LACH01	:1; // 15
};
union DigitalOutPut_REG
{
   unsigned int     all;
   struct DigitalOutPut_BIT bit;
};
struct SystemStateA_BIT
{     	// bits   description
   unsigned int 	InitOk	:1;	// 0  
   unsigned int 	LED01	:1;	// 1
   unsigned int 	LED02	:1;	// 2
   unsigned int  	LED03	:1;	// 3  
   unsigned int 	LED04	:1;	// 4 
   unsigned int 	LED05	:1;	// 5
   unsigned int 	LED06	:1;	// 6
   unsigned int 	LED07	:1;	// 7
   unsigned int 	LED08	:1;	// 8  
   unsigned int 	LED09	:1;	// 9
   unsigned int 	LED10	:1;	// 10
   unsigned int  	LED11	:1;	// 11
   unsigned int 	LED12	:1;	// 12  
   unsigned int 	LED13	:1;	// 13
   unsigned int 	LACH00	:1;	// 14
   unsigned int 	LACH01	:1; // 15
};
union SystemStateA_REG
{
   unsigned int     all;
   struct SystemStateA_BIT bit;
};
typedef struct System_Date																							
{
	Uint16 	MainIsr;
	Uint16	MainLoop;
	Uint16 	CpuTimer0Loop;
	Uint16 	CanRxLoop;
	Uint16 	Timer10msec;
	Uint16	Timer20msec;
	Uint16 	Timer50msec;
	Uint16 	Timer100msec;
	Uint16 	Timer500msec;
	Uint16 	Timer200msec;
	Uint16 	Timer1000msec;
	Uint16  SystemCheckCout;
	union DigitalInput_REG	DigitalInputReg;
	union DigitalOutPut_REG	DigitalOutPutReg;
	union SystemStateA_REG  SystemStateARegs;
	unsigned char SW00Count;
	unsigned char SW01Count;
	unsigned char SW02Count;
	unsigned char SW03Count;

	Uint16 CANTxLoop;
	unsigned char ForcingMode;
	
}SystemReg;


struct LCDMenuA_BIT
{     	// bits   description
   unsigned int     CURMODE     :1;
   unsigned int 	MANMODE  	:1;	// 0  
   unsigned int 	AUTOMODE	:1;	// 1
   unsigned int 	SETTING 	:1;	// 2
   unsigned int  	MODE01	:1;	// 3  
   unsigned int 	MODE02	:1;	// 4 
   unsigned int 	MODE03	:1;	// 5
   unsigned int 	MODE04	:1;	// 6
   unsigned int 	MODE05	:1;	// 7
   unsigned int 	MODE06	:1;	// 8  
   unsigned int 	MODE07	:1;	// 9
   unsigned int  	MODE09	:1;	// 11
   unsigned int 	ARROW	:1;	// 12  
   unsigned int 	ESC		:1;	// 13
   unsigned int     MENU	:1;	// 14
   unsigned int 	ENTER	:1; // 15
};

union LCDMenuA_REG
{
   unsigned int     all;
   struct LCDMenuA_BIT bit;
};

struct LCDMenuB_BIT
{     	// bits   description
   unsigned int 	SETTGPWR  	:1;	// 0  
   unsigned int 	SETSOC  	:1;	// 1
   unsigned int 	REV02 	:1;	// 2
   unsigned int  	REV03	:1;	// 3  
   unsigned int 	REV04	:1;	// 4 
   unsigned int 	REV05	:1;	// 5
   unsigned int 	REV06	:1;	// 6
   unsigned int 	REV07	:1;	// 7
   unsigned int 	REV08	:1;	// 8  
   unsigned int 	REV09	:1;	// 9
   unsigned int 	REV10	:1;	// 10
   unsigned int  	REV11	:1;	// 11
   unsigned int 	REV12	:1;	// 12  
   unsigned int 	REV13		:1;	// 13
   unsigned int     REV14	:1;	// 14
   unsigned int 	REV15	:1; // 15
};

union LCDMenuB_REG
{
   unsigned int     all;
   struct LCDMenuB_BIT bit;
};


typedef struct LCD_Date																							
{
	char 	NewLCDPage;
	char 	OLDLCDPage;

	char 	d10000;
	char	d1000;
	char    d100;
	char    d10;
	char    d1;
	int 	Dataline;
	int		NumData;
	int 	NumDataBuf;
	int		NData;
	int 	PData;
	unsigned char	pos;
	unsigned char	PosX;
	unsigned char	PosY;

	union LCDMenuA_REG LCDMenuAReg;
	union LCDMenuB_REG LCDMenuBReg;

	unsigned int LCDDisplayCnt;

}LCDReg;

typedef struct NOB_Date																							
{
	int 	NEWNOBCount;
	int 	OldNOBCount;
	
}NOBReg;


typedef struct PCSRX_DATA																							
{
	Uint16 GridVoltage;
	Uint16 BatteryVoltage;
	Uint16 PVVoltage;
	Uint16 LoadVoltage;
	unsigned char CountVoltage;

	Uint16 GridCurrent;
	int BatteryCurrent;
	Uint16 PVCurrent;
	Uint16 LoadCurrent;
	unsigned char CountCurrent;
	
}PCSRXReg;




struct PCSMODE_BIT
{     	// bits   description
   unsigned int 	MODE01			:1;        // 0  
   unsigned int 	MODE02			:1;        // 1
   unsigned int 	MODE03			:1;        // 2
   unsigned int  	MODE04			:1;        // 3  
   unsigned int 	MODE05			:1;       // 4 
   unsigned int 	MODE06			:1;		  // 5
   unsigned int 	MODE07			:1;		  // 6
   unsigned int 	REV07			:1;       // 7
   unsigned int 	MODE09			:1;       // 8  
   unsigned int 	AUTO			:1;      // 9
   unsigned int 	MAN 			:1;      // 10
   unsigned int  	REV11			:1;      // 11
   unsigned int 	REV12			:1;      // 12  
   unsigned int 	REV13			:1;		// 13
   unsigned int 	REV14			:1;		// 14
   unsigned int 	REV15			:1;     // 15
};

union PCSMODE_REG 
{
   unsigned int     all;
   struct PCSMODE_BIT bit;
};

struct PCSSTATE_BIT
{     	// bits   description
   unsigned int		PCSRUN			:1;        // 0  
   unsigned int 	PCSSTOP			:1;        // 1
   unsigned int 	TARGETPWR	    :1;        // 2
   unsigned int		REV03			:1;        // 3  
   unsigned int 	REV04			:1;        // 4 
   unsigned int  	REV05			:1;		   // 5
   unsigned int 	REV06			:1;		   // 6
   unsigned int 	REV07			:1;        // 7
   unsigned int		REV08			:1;        // 0  
   unsigned	int 	REV09			:1;        // 1
   unsigned int 	REV10			:1;        // 2
   unsigned int		REV11			:1;        // 3  
   unsigned int 	REV12			:1;        // 4 
   unsigned int  	REV13			:1;		   // 5
   unsigned int 	REV14			:1;		   // 6
   unsigned int 	REV15			:1;        // 7
};

union PCSSTATE_REG 
{
   unsigned int     all;
   struct PCSSTATE_BIT bit;
};



///////////////////////////////////////////////////////////////////////////////
// 여기서부터 CAN 통신 관련 구조체
//

struct PCSINIT_BIT
{     	// bits   description
   unsigned char 	PCS_RUN			:1;        // 0  
   unsigned char 	PCS_STOP		:1;        // 1
   unsigned char 	PCS_RESET		:1;        // 2
   unsigned char  	REV03			:1;        // 3  
   unsigned char 	REV04			:1;       // 4 
   unsigned char 	REV05			:1;		  // 5
   unsigned char 	REV06			:1;		  // 6
   unsigned char 	REV07			:1;       // 7
};

union PCSINIT_REG 
{
   unsigned char     all;
   struct PCSINIT_BIT bit;
};

struct PCSERRA_BIT
{     	// bits   description
   unsigned int		AC_OVRVLT_ALM			:1;        // 0  
   unsigned int 	AC_UNDVLT_ALM			:1;        // 1
   unsigned int 	PV_OVRVLT_ALM			:1;        // 2
   unsigned int		BT_OVRVLT_ALM			:1;        // 3  
   unsigned int 	BT_UNDVLT_ALM			:1;        // 4 
   unsigned int  	AC_OVRCUR_ALM			:1;		   // 5
   unsigned int 	PV_OVRCUR_ALM			:1;		   // 6
   unsigned int 	BT_OVRCUR_ALM			:1;        // 7
   unsigned int		AC_OVRVLT_EMR			:1;        // 8  
   unsigned int 	AC_UNDVLT_EMR			:1;        // 9
   unsigned int 	PV_OVRVLT_EMR			:1;        // 10
   unsigned int		BT_OVRVLT_EMR			:1;        // 11  
   unsigned int 	BT_UNDVLT_EMR			:1;        // 12 
   unsigned int  	AC_OVRCUR_EMR			:1;		   // 13
   unsigned int 	PV_OVRCUR_EMR			:1;		   // 14
   unsigned int 	BT_OvRCUR_EMR			:1;        // 15
};

union PCSERRA_REG 
{
   unsigned int     all;
   struct PCSERRA_BIT bit;
};

struct PCSERRB_BIT
{     	// bits   description
   unsigned int		PCSGTW_COM_ALM			:1;        // 0  
   unsigned int 	PCSHMI_COM_ALM			:1;        // 1
   unsigned int 	PCSBMS_COM_ALM			:1;        // 2
   unsigned int		AC_OVRFREQ_EMR			:1;        // 3  
   unsigned int 	AC_UNDFREQ_EMR			:1;        // 4 
   unsigned int  	PLL_LOCK_ALM			:1;		   // 5
   unsigned int 	TMP_HIGH_EMR			:1;		   // 6
   unsigned int 	PV_UNDVLT_ALM			:1;        // 7
   unsigned int		PV_UNDVLT_EMR			:1;        // 8  
   unsigned int 	TMP_HIGH_ALM			:1;        // 9
   unsigned int 	PCS_OPR_ERR				:1;        // 10
   unsigned int		LD_OVR_VLT_EMR			:1;        // 11  
   unsigned int 	LD_UND_VLT_EMR			:1;        // 12 
   unsigned int  	LD_OVR_CUR_EMR			:1;		   // 13
   unsigned int 	REV14					:1;		   // 14
   unsigned int 	REV15					:1;        // 15
};

union PCSERRB_REG 
{
   unsigned int     all;
   struct PCSERRB_BIT bit;
};




struct PCSCMD_BIT
{					// bits   description
   unsigned int		CMD_MODE_01			:1;        // 0  
   unsigned int 	CMD_MODE_02			:1;        // 1
   unsigned int 	CMD_MODE_03		    :1;        // 2
   unsigned int		CMD_MODE_04			:1;        // 3  
   unsigned int 	CMD_MODE_05			:1;        // 4 
   unsigned int  	CMD_MODE_06		    :1;		   // 5
   unsigned int 	CMD_MODE_07			:1;		   // 6
   unsigned int 	REV07				:1;        // 7
   unsigned int		CMD_MODE_09 		:1;        // 8  
   unsigned int 	REV09 				:1;        // 9
   unsigned int 	REV10				:1;        // 10
   unsigned int		REV11				:1;        // 11  
   unsigned int 	REV12			    :1;        // 12 
   unsigned int  	REV13				:1;		   // 13
   unsigned int 	REV14				:1;		   // 14
   unsigned int 	REV15				:1;        // 15
};

union PCSCMD_REG 
{
   unsigned int     all;
   struct PCSCMD_BIT bit;
};

struct BATCMD_BIT
{     	// bits   description
   unsigned int 	BAT_WAKEUP_CMD	:1;        // 0  
   unsigned int 	BAT_SHUTDN_CMD	:1;        // 1
   unsigned int 	REV02			:1;        // 2
   unsigned int  	REV03			:1;        // 3  
   unsigned int 	REV04			:1;       // 4 
   unsigned int 	REV05			:1;		  // 5
   unsigned int 	REV06			:1;		  // 6
   unsigned int 	REV07			:1;       // 7
   unsigned int 	REV08   		:1;       // 8  
   unsigned int 	REV09			:1;      // 9
   unsigned int 	REV10 			:1;      // 10
   unsigned int  	REV11			:1;      // 11
   unsigned int 	REV12			:1;      // 12  
   unsigned int 	REV13			:1;		// 13
   unsigned int 	REV14			:1;		// 14
   unsigned int 	REV15			:1;     // 15
};

union BATCMD_REG 
{
   unsigned int     all;
   struct BATCMD_BIT bit;
};


struct BMSSEQA_BIT
{     	// bits   description
   unsigned  int	BMS_RDY_ST			:1;        // 0  
   unsigned	 int 	BMS_PWR_ST			:1;        // 1
   unsigned  int 	PCS_RLY_ST			:1;        // 2
   unsigned  int	REV03				:1;        // 3  
   unsigned  int 	PCS_EMRG			:1;        // 4 
   unsigned  int  	BMS_CHG_ST			:1;		   // 5
   unsigned  int 	BMS_DCHG_ST			:1;		   // 6
   unsigned  int 	REV07				:1;        // 7
   unsigned  int	SYS_RDY				:1;        // 8  
   unsigned	 int 	PCS_RUNCMD_ACK		:1;        // 9
   unsigned  int 	PCS_STOPCMD_ACK		:1;        // 10
   unsigned  int	BAT_WAKEUP_ACK		:1;        // 11  
   unsigned  int 	BAT_SHUTDNCMD_ACK	:1;        // 12 
   unsigned  int  	PCS_READY_ACK		:1;		   // 13
   unsigned  int 	REV14				:1;		   // 14
   unsigned  int 	REV15				:1;        // 15
};

union BMSSEQA_REG 
{
   unsigned int     all;
   struct BMSSEQA_BIT bit;
};

struct BMSSEQB_BIT
{     	// bits   description
   unsigned  int	MODE01_ACK			:1;        // 0  
   unsigned	 int 	MODE02_ACK			:1;        // 1
   unsigned  int 	MODE03_ACK			:1;        // 2
   unsigned  int	MODE04_ACK			:1;        // 3  
   unsigned  int 	MODE05_ACK			:1;        // 4 
   unsigned  int  	MODE06_ACK			:1;		   // 5
   unsigned  int 	MODE07_ACK			:1;		   // 6
   unsigned  int 	MODE08_ACK			:1;        // 7
   unsigned  int	MODE09_ACK			:1;        // 0  
   unsigned	 int 	REV09			:1;        // 1
   unsigned  int 	REV10			:1;        // 2
   unsigned  int	REV11		:1;        // 3  
   unsigned  int 	REV12			:1;        // 4 
   unsigned  int  	REV13	:1;		   // 5
   unsigned  int 	REV14			:1;		   // 6
   unsigned  int 	REV15			:1;        // 7
};

union BMSSEQB_REG 
{
   unsigned int     all;
   struct BMSSEQB_BIT bit;
};

struct BMSSTS_BIT
{     	// bits   description
   unsigned  int	BMS_RDY				:1;        // 0  
   unsigned	 int 	BMS_STNDBY			:1;        // 1
   unsigned  int 	BMS_WARNING			:1;        // 2
   unsigned  int	BMS_FLT				:1;        // 3  
   unsigned  int 	M_RLY_STS			:1;        // 4 
   unsigned  int  	P_RLY_STS			:1;		   // 5
   unsigned  int 	CUR_STS_REQ			:1;		   // 6
   unsigned  int 	BAT_ENABLE			:1;        // 7
   unsigned  int	REV08				:1;        // 0  
   unsigned	 int 	REV09				:1;        // 1
   unsigned  int 	REV10				:1;        // 2
   unsigned  int	REV11				:1;        // 3  
   unsigned  int 	REV12				:1;        // 4 
   unsigned  int  	REV13				:1;		   // 5
   unsigned  int 	REV14				:1;		   // 6
   unsigned  int 	REV15				:1;        // 7
};

union BMSSTS_REG 
{
   unsigned int     all;
   struct BMSSTS_BIT bit;
};




struct PCSSTS_A_BIT
{     	// bits   description
   unsigned int		PCS_IDLE0_STS			:1;        // 0  
   unsigned int 	PCS_PWRUP_STS			:1;        // 1
   unsigned int 	PCS_ALARM_STS			:1;        // 2
   unsigned int		PCS_EMERG_STS			:1;        // 3  
   unsigned int 	PCS_ANTIL_STS			:1;        // 4 
   unsigned int  	PCS_RCNTL_MOD			:1;		   // 5
   unsigned int 	PCS_MCNTL_MOD			:1;		   // 6
   unsigned int 	PCS_ECNTL_MOD			:1;        // 7
   unsigned int		PCS_ALTAS_STS			:1;        // 8  
   unsigned int 	PCS_ALTAS_GRD			:1;        // 9
   unsigned int 	PV_MOD	        		:1;        // 10
   unsigned int		PV_DSCHG_MOD			:1;        // 11  
   unsigned int 	PV_CHRG1_MOD			:1;        // 12 
   unsigned int  	PV_CHRG2_MOD			:1;		   // 13
   unsigned int 	PCS_NOP00_MOD			:1;		   // 14
   unsigned int 	PCS_UPS_MOD			:1;        // 15
};

union PCSSTS_A_REG 
{
   unsigned int     all;
   struct PCSSTS_A_BIT bit;
};

struct PCSSTS_B_BIT
{     	// bits   description
   unsigned int		GRD_CONCT_STS			:1;        // 0  
   unsigned int 	PCS_MODE_09				:1;        // 1
   unsigned int 	GRD_DSCHG_MOD			:1;        // 2
   unsigned int		GRD_CHARG_MOD			:1;        // 3  
   unsigned int 	PCS_PVCP_MOD			:1;        // 4 
   unsigned int  	PV_UND_VOLT				:1;		   // 5
   unsigned int 	REV06					:1;		   // 6
   unsigned int 	REV07					:1;        // 7
   unsigned int		RLY_1_ST				:1;        // 8  
   unsigned int 	RLY_2_ST				:1;        // 9
   unsigned int 	RLY_3_ST				:1;        // 10
   unsigned int		RLY_4_ST				:1;        // 11  
   unsigned int 	RLY_5_ST				:1;        // 12 
   unsigned int  	RLY_6_ST				:1;		   // 13
   unsigned int 	RLY_7_ST				:1;		   // 14
   unsigned int 	RLY_8_ST				:1;        // 15
};

union PCSSTS_B_REG 
{
   unsigned int     all;
   struct PCSSTS_B_BIT bit;
};


struct PCSALM_BIT
{     	// bits   description
   unsigned int		AC_OVRVLT_ALM			:1;        // 0  
   unsigned int 	AC_UNDVLT_ALM			:1;        // 1
   unsigned int 	AC_OVRFRQ_ALM			:1;        // 2
   unsigned int		AC_UNDFRQ_ALM			:1;        // 3  
   unsigned int 	AC_OVRCUR_ALM			:1;        // 4 
   unsigned int  	BT_OVRVLT_ALM			:1;		   // 5
   unsigned int 	BT_UNDVLT_ALM			:1;		   // 6
   unsigned int 	BT_OVRCUR_ALM			:1;        // 7
   unsigned int		PV_OVRVLT_ALM			:1;        // 8  
   unsigned int 	PV_OVRCUR_ALM			:1;        // 9
   unsigned int 	PC_STOVTM_ALM			:1;        // 10 
   unsigned int  	LD_OVRVLT_ALM			:1;		   // 11
   unsigned int 	LD_UNDVLT_ALM			:1;		   // 12
   unsigned int 	LD_OVRCUR_ALM			:1;        // 13
   unsigned int		AC_OVRVLT_EMR			:1;        // 14  
   unsigned int 	AC_UNDVLT_EMR			:1;        // 15
};

union PCSALM_REG 
{
   unsigned int     all;
   struct PCSALM_BIT bit;
};

struct PCSEMR_BIT
{     	// bits   description
   unsigned int 	AC_OVRFRQ_EMR			:1;        // 0
   unsigned int		AC_UNDFRQ_EMR			:1;        // 1  
   unsigned int 	AC_OVRCUR_EMR			:1;        // 2 
   unsigned int  	BT_OVRVLT_EMR			:1;		   // 3
   unsigned int 	BT_UNDVLT_EMR			:1;		   // 4
   unsigned int 	BT_OVRCUR_EMR			:1;        // 5
   unsigned int		PV_OVRVLT_EMR			:1;        // 6  
   unsigned int 	PV_OVRCUR_EMR			:1;        // 7
   unsigned int 	PC_STOVTM_EMR			:1;        // 8 
   unsigned int  	LD_OVRVLT_EMR			:1;		   // 9
   unsigned int 	LD_UNDVLT_EMR			:1;		   // 10
   unsigned int 	LD_OVRCUR_EMR			:1;        // 11
   unsigned int 	PCS_GW_COMALM			:1;        // 12
   unsigned int 	PCS_HMI_COMALM			:1;        // 13
   unsigned int 	PCS_BMS_COMALM			:1;        // 14
   unsigned int 	REV15					:1;		   // 15

};

union PCSEMR_REG 
{
   unsigned int     all;
   struct PCSEMR_BIT bit;
};

typedef struct CANTX_DATA																							
{
	//EMS로 송신
	Uint16 GridVoltage;
	Uint16 BatVoltage;
	Uint16 PVVoltage;
	Uint16 LoadVoltage;

	Uint16 GridCurrent;
	int16 BatCurrent;
	Uint16 PVCurrent;
	Uint16 LoadCurrent;
	
	//PCS로 송신
	union PCSCMD_REG PCSCMDReg;	
	int16 TargetPwr;

	union PCSINIT_REG PCSINITReg;


	union BATCMD_REG  EMSBATCMDRegs;
	
	union BMSSEQA_REG MMIBmsSeqARegs;
	union BMSSEQB_REG MMIBmsSeqBRegs;

	union PCSSTS_A_REG PCSSTS_A_Regs;
	union PCSSTS_B_REG PCSSTS_B_Regs;

	//union PCSALM_REG PCSALMReg;
	//union PCSEMR_REG PCSEMRReg;

	union PCSERRA_REG	PCSALMReg;
	union PCSERRB_REG	PCSEMRReg;

}CANTXReg;


///////////////////////////////////////////////////////////////////////////////
// 여기서부터 CAN RX 관련
///////////////////////////////////////////////////////////////////////////////


struct PCSMOD_BIT
{     	// bits   description
   unsigned int 	MODE01			:1;        // 0  
   unsigned int 	MODE02			:1;        // 1
   unsigned int 	MODE03			:1;        // 2
   unsigned int  	MODE04			:1;        // 3  
   unsigned int 	MODE05			:1;       // 4 
   unsigned int 	MODE06			:1;		  // 5
   unsigned int 	MODE07			:1;		  // 6
   unsigned int 	REV07			:1;       // 7
   unsigned int 	MODE09			:1;       // 8  
   unsigned int 	REV09			:1;      // 9
   unsigned int 	REV10 			:1;      // 10
   unsigned int  	REV11			:1;      // 11
   unsigned int 	REV12			:1;      // 12  
   unsigned int 	REV13			:1;		// 13
   unsigned int 	REV14			:1;		// 14
   unsigned int 	REV15			:1;     // 15
};

union PCSMOD_REG 
{
   unsigned int     all;
   struct PCSMOD_BIT bit;
};





struct PCSACK_BIT
{     	// bits   description
   unsigned int 	ACK_MODE_01			:1;        // 0  
   unsigned int 	ACK_MODE_02			:1;        // 1
   unsigned int 	ACK_MODE_03			:1;        // 2
   unsigned int 	ACK_MODE_04			:1;        // 3  
   unsigned int 	ACK_MODE_05			:1;        // 4 
   unsigned int 	ACK_MODE_06			:1;		   // 5
   unsigned int     ACK_MODE_07			:1;		   // 6
   unsigned int 	REV07	     		:1;        // 7
   unsigned int 	ACK_MODE_09			:1;        // 8  
   unsigned int 	REV09				:1;        // 9
   unsigned int 	REV10				:1;        // 10
   unsigned int 	REV11				:1;        // 11  
   unsigned int 	REV12				:1;        // 12 
   unsigned int 	REV13				:1;		   // 13
   unsigned int     REV14				:1;		   // 14
   unsigned int 	REV15	  	    	:1;        // 15
};

union PCSACK_REG 
{
   unsigned char     all;
   struct PCSACK_BIT bit;
};

struct PCSRLY_BIT
{     	// bits   description
   unsigned char	RLY_001_STS			:1;        // 0  
   unsigned char 	RLY_002_STS	    	:1;        // 1
   unsigned char 	RLY_003_STS			:1;        // 2
   unsigned char	RLY_004_STS		    :1;        // 3  
   unsigned char 	RLY_005_STS			:1;        // 4 
   unsigned char  	RLY_006_STS			:1;		   // 5
   unsigned char 	RLY_007_STS		    :1;		   // 6
   unsigned char 	RLY_008_STS			:1;        // 7
};

union PCSRLY_REG 
{
   unsigned char     all;
   struct PCSRLY_BIT bit;
};



typedef struct CANRX_DATA																							
{
	//EMS로 부터 수신되는 데이터 그룹
	union PCSMOD_REG PCSMODReg;
	int16 TargetPwr;
	union PCSINIT_REG PCSINITReg;

	
	//PCS로부터 수신되는 데이터 그룹
	Uint16 GridVoltage;
	Uint16 BatVoltage;
	Uint16 PVVoltage;
	Uint16 LoadVoltage;

	Uint16 GridCurrent;
	int16  BatCurrent;
	Uint16 PVCurrent;
	Uint16 LoadCurrent;

	int16  PCSInsideTemp;
	int16  PCSStackTemp;
	
	#if 0
	union PCSALM_REG PCSALMReg;
	union PCSEMR_REG PCSEMRReg;
	union PCSACK_REG PCSACKReg;
	union PCSRLY_REG PCSRLYReg;
	#endif
	
	//from BMS
	unsigned char BMS_SOC;
	unsigned char BMS_SOH;
	unsigned char BMS_Power;
	char		  BMS_Current;
	union BMSSTS_REG  BMSTSTRegs;
	//To here BMS


	union BATCMD_REG  EMSBATCMDRegs;

	union BMSSEQA_REG MMIBmsSeqARegs;
	union BMSSEQB_REG MMIBmsSeqBRegs;

	union PCSSTS_A_REG PCSSTS_A_Regs;
	union PCSSTS_B_REG PCSSTS_B_Regs;

	//union PCSALM_REG PCSALMReg;
	//union PCSEMR_REG PCSEMRReg;

	union PCSERRA_REG	PCSALMReg;
	union PCSERRB_REG	PCSEMRReg;

	//BMS로 부터 수신되는 데이터 그룹
	
}CANRXReg;



//
//여기까지 CAN 통신 관련 구조체
///////////////////////////////////////////////////////////////////////////////



struct PCSSTSA_BIT
{     	// bits   description
   unsigned int		PCS_IDLE			:1;        // 0  
   unsigned int 	PCS_PWRUP			:1;        // 1
   unsigned int 	PCS_ALRM			:1;        // 2
   unsigned int		PCS_EMRG			:1;        // 3  
   unsigned int 	ANTI_ISLND			:1;        // 4 
   unsigned int  	PCS_RMT_CNTL_MODE	:1;		   // 5
   unsigned int 	PCS_MAN_CNTL_MODE	:1;		   // 6
   unsigned int 	PCS_EMUL_CNTL_MODE	:1;        // 7
   unsigned int		ALTAS_CONT_PNT		:1;        // 8  
   unsigned int 	ALTS_GRID_ST		:1;        // 9
   unsigned int 	PCS_PV_MODE			:1;        // 10
   unsigned int		PCS_PV_DCHRG_MODE	:1;        // 11  
   unsigned int 	PCS_PV_CHRG1_MODE	:1;        // 12 
   unsigned int  	PCS_PV_CHRG2_MODE	:1;		   // 13
   unsigned int 	PCS_NO_MODE			:1;		   // 14
   unsigned int 	PCS_STND_ALONE		:1;        // 15
};

union PCSSTSA_REG 
{
   unsigned int     all;
   struct PCSSTSA_BIT bit;
};

struct PCSSTSB_BIT
{     	// bits   description
   unsigned int		GRID_CONNECT				:1;        // 0  
   unsigned int 	PCS_E_NO_OPRMODE			:1;        // 1
   unsigned int 	PCS_GRID_DCHRG_MODE			:1;        // 2
   unsigned int		PCS_GRID_CHRG_MODE			:1;        // 3  
   unsigned int 	REV04						:1;        // 4 
   unsigned int  	REV05			:1;		   // 5
   unsigned int 	REV06			:1;		   // 6
   unsigned int 	REV07			:1;        // 7
   unsigned int		REV08			:1;        // 8  
   unsigned int 	REV09			:1;        // 9
   unsigned int 	REV10			:1;        // 10
   unsigned int		REV11			:1;        // 11  
   unsigned int 	REV12			:1;        // 12 
   unsigned int  	REV13			:1;		   // 13
   unsigned int 	REV14			:1;		   // 14
   unsigned int 	REV15			:1;        // 15
};

union PCSSTSB_REG 
{
   unsigned int     all;
   struct PCSSTSB_BIT bit;
};



typedef struct MODBUS_DATA																							
{
#define MAX_TX_BUF	32
#define MAX_RX_BUF	32

unsigned char CommandNum;

Uint16 GridVoltage;
Uint16 BatVoltage;
Uint16 PVVoltage;
Uint16 LoadVoltage;

Uint16 GridCurrent;
int16 BatCurrent;
Uint16 PVCurrent;
Uint16 LoadCurrent;

Uint16 FWVersion;
Uint16 BATSoc;

union BMSSEQA_REG PCSBmsSeqARegs;
union BMSSEQB_REG PCSBmsSeqBRegs;

union PCSERRA_REG PCSErrorARegs;
union PCSERRB_REG PCSErrorBRegs;

//WR: MMI --> PCS
int16  WR_GridMaxVoltage;  
int16  WR_GrdiMinVoltage;
Uint16 WR_ChargeBatShutVoltage;
Uint16 WR_DischargeBatShutVoltage;
Uint16 WR_PVMaxVoltage;
Uint16 WR_PVMPPVoltage;
Uint16 WR_PVMaxCurrent;
Uint16 WR_PCSMaxOutputPower;
Uint16 WR_PVMaxOutputPower;
Uint16 WR_PVMPPCurrent;

//RD: PCS --> MMI
int16  RD_GridMaxVoltage;
int16  RD_GrdiMinVoltage;
Uint16 RD_ChargeBatShutVoltage;
Uint16 RD_DischargeBatShutVoltage;
Uint16 RD_PVMaxVoltage;
Uint16 RD_PVMPPVoltage;
Uint16 RD_PVMaxCurrent;
Uint16 RD_PCSMaxOutputPower;
Uint16 RD_PVMaxOutputPower;
Uint16 RD_PVMPPCurrent;

	

unsigned char MMITxBuf[MAX_TX_BUF];
unsigned char MMIRxBuf[MAX_RX_BUF];
unsigned char RxLength;
unsigned char LastCMD;
unsigned char LastSA;
unsigned char LastQT;
unsigned char RespOK;
	
}MODReg;


///////////////////////////////////////////////////////////////////////////////
//여기서 부터 시퀀스 Control을 위한 구조체
//

#define _YES_		1
#define _NO_		0x00

#define _ACK_OK_  0xAA
#define _ACK_NO_  0x00



typedef struct SEQ_CNTL																							
{
	unsigned char EMSModCmdNEW;
	unsigned char EMSModCmdCUR;
	unsigned char EMSModCmdAck;

	unsigned char PCSModCmdNEW;
	unsigned char PCSModCmdCUR;
	unsigned char PCSModCmdAck;

	union PCSMOD_REG PCSMODReg;

	unsigned char CURRENTMode;



}SEQReg;


//
//여기까지 시퀀스 Control을 위한 구조체
///////////////////////////////////////////////////////////////////////////////

#endif  // end of DSP28x_PROJECT_H definition

