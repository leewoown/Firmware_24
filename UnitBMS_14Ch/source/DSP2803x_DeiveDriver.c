// TI File $Revision: /main/5 $
// Checkin $Date: November 30, 2009   17:43:48 $
//###########################################################################
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File

void WG_InitGpio(void);
void WG_InitSci(void);
void WG_InitECana(void);
void WG_InitSPI(void);

void SPI_Write(unsigned int WRData);
unsigned int SPI_READ(void);

int CellVoltageUpdata(SlaveReg *s);
int SlaveBMSIint(SlaveReg *s);
int SlaveBmsBalance(SlaveReg *s);

void WG_InitGpio(void)
{
	
   EALLOW;
  // GpioCtrlRegs.GPAMUX1.all = 0x0000;     
   GpioCtrlRegs.GPAMUX1.bit.GPIO0=GPIO;     // TEMP01 OUT
   GpioCtrlRegs.GPAMUX1.bit.GPIO1=GPIO;     // TEMP03 OUT
   GpioCtrlRegs.GPAMUX1.bit.GPIO2=GPIO;     // TEMP02 OUT
   GpioCtrlRegs.GPAMUX1.bit.GPIO3=GPIO;     // Wake up OUT
   GpioCtrlRegs.GPAMUX1.bit.GPIO4=GPIO;     // Charge Wake up
   GpioCtrlRegs.GPAMUX1.bit.GPIO5=GPIO;     // Charge Fault
   GpioCtrlRegs.GPAMUX1.bit.GPIO6=GPIO;
   GpioCtrlRegs.GPAMUX1.bit.GPIO7=GPIO;     // SPI CAN RESET OUT
   GpioCtrlRegs.GPAMUX1.bit.GPIO8=GPIO;     // SPI CAN ENABLE OUT
   GpioCtrlRegs.GPAMUX1.bit.GPIO9=GPIO;     // SPI TCP ENABLE OUT
   GpioCtrlRegs.GPAMUX1.bit.GPIO10=GPIO;    // SPI BATTERY IC BATTERY EN
   GpioCtrlRegs.GPAMUX1.bit.GPIO11=GPIO;    // SPI NVRAM ENABLE
   GpioCtrlRegs.GPAMUX1.bit.GPIO12=GPIO;    // FAN DIGITAL OUT
 //GpioCtrlRegs.GPAMUX1.bit.GPIO13=GPIO;
 //GpioCtrlRegs.GPAMUX1.bit.GPIO14=GPIO;
 //GpioCtrlRegs.GPAMUX1.bit.GPIO15=GPIO;

 //GpioCtrlRegs.GPAMUX2.all = 0x0000;       // GPIO functionality GPIO16-GPIO31
   GpioCtrlRegs.GPAMUX2.bit.GPIO16=SPI;     // SPI FUNCTION MOSI
   GpioCtrlRegs.GPAMUX2.bit.GPIO17=SPI;     // SPI FUNCTION MISO
   GpioCtrlRegs.GPAMUX2.bit.GPIO18=SPI;     // SPI FUNCTION CLK
   //GpioCtrlRegs.GPAMUX2.bit.GPIO18=3;     // 내부 클럭 확인, 반드시 SPI CLK  변경 할 것
   GpioCtrlRegs.GPAMUX2.bit.GPIO19=SPI;		// H/W DISCONNTER
   GpioCtrlRegs.GPAMUX2.bit.GPIO20=GPIO;	// POWER RELAY
   GpioCtrlRegs.GPAMUX2.bit.GPIO21=GPIO;    // Protect BMS PRECHARGE OUT
   GpioCtrlRegs.GPAMUX2.bit.GPIO22=GPIO;    // Protect BMS DISCHARGE OUT
   GpioCtrlRegs.GPAMUX2.bit.GPIO23=GPIO;
   GpioCtrlRegs.GPAMUX2.bit.GPIO24=GPIO;    // Protect BMS CHARGE OUT
   GpioCtrlRegs.GPAMUX2.bit.GPIO25=GPIO;
   GpioCtrlRegs.GPAMUX2.bit.GPIO26=GPIO;
   GpioCtrlRegs.GPAMUX2.bit.GPIO27=GPIO;
   GpioCtrlRegs.GPAMUX2.bit.GPIO28=SCIRX;
   GpioCtrlRegs.GPAMUX2.bit.GPIO29=SCITX;
   GpioCtrlRegs.GPAMUX2.bit.GPIO30=CANRX;
   GpioCtrlRegs.GPAMUX2.bit.GPIO31=CANTX;
  // GpioCtrlRegs.GPBMUX1.all = 0x0000;     // GPIO functionality GPIO32-GPIO44

   GpioCtrlRegs.GPBMUX1.bit.GPIO32=GPIO;    // RTCMFP (기능확인 필요함)
   GpioCtrlRegs.GPBMUX1.bit.GPIO33=GPIO;
   GpioCtrlRegs.GPBMUX1.bit.GPIO34=GPIO;    //
   //GpioCtrlRegs.GPBMUX1.bit.GPIO39=GPIO;
   //GpioCtrlRegs.GPBMUX1.bit.GPIO40=GPIO;
   //GpioCtrlRegs.GPBMUX1.bit.GPIO41=GPIO;
   //GpioCtrlRegs.GPBMUX1.bit.GPIO42=GPIO;
   //GpioCtrlRegs.GPBMUX1.bit.GPIO43=GPIO;
   //GpioCtrlRegs.GPBMUX1.bit.GPIO44=GPIO;
  
   //GpioCtrlRegs.AIOMUX1.all = 0x0000;     // Dig.IO  funct. applies to AIO2,4,6,10,12,14
   GpioCtrlRegs.AIOMUX1.bit.AIO2=GPIO;      // Digital input 1 ID_01
   GpioCtrlRegs.AIOMUX1.bit.AIO4=GPIO;      // Digital input 2 ID_02
   GpioCtrlRegs.AIOMUX1.bit.AIO6=GPIO;      // Digital input 3 ID_03
   GpioCtrlRegs.AIOMUX1.bit.AIO10=GPIO;     // Digital input 5 ID_04
   GpioCtrlRegs.AIOMUX1.bit.AIO12=GPIO;     // Digital input 4 ID_05
   //GpioCtrlRegs.AIOMUX1.bit.AIO14=GPIO;
	
   //GpioCtrlRegs.GPADIR.all = 0x0000;      // GPIO0-GPIO31 are GP inputs
   GpioCtrlRegs.GPADIR.bit.GPIO0=OUTPUT;
   GpioCtrlRegs.GPADIR.bit.GPIO1=OUTPUT;
   GpioCtrlRegs.GPADIR.bit.GPIO2=OUTPUT;
   GpioCtrlRegs.GPADIR.bit.GPIO3=OUTPUT;
   GpioCtrlRegs.GPADIR.bit.GPIO4=OUTPUT;
   GpioCtrlRegs.GPADIR.bit.GPIO5=OUTPUT;
   GpioCtrlRegs.GPADIR.bit.GPIO6=OUTPUT;
   GpioCtrlRegs.GPADIR.bit.GPIO7=OUTPUT;
   GpioCtrlRegs.GPADIR.bit.GPIO8=OUTPUT;
   GpioCtrlRegs.GPADIR.bit.GPIO9=OUTPUT;
   GpioCtrlRegs.GPADIR.bit.GPIO10=OUTPUT;
   GpioCtrlRegs.GPADIR.bit.GPIO11=OUTPUT;
   GpioCtrlRegs.GPADIR.bit.GPIO12=OUTPUT;
   //GpioCtrlRegs.GPADIR.bit.GPIO13=OUTPUT;
   //GpioCtrlRegs.GPADIR.bit.GPIO14=OUTPUT;
   //GpioCtrlRegs.GPADIR.bit.GPIO15=OUTPUT;

   GpioCtrlRegs.GPADIR.bit.GPIO16=Default;
   GpioCtrlRegs.GPADIR.bit.GPIO17=Default;
   GpioCtrlRegs.GPADIR.bit.GPIO18=Default;
   GpioCtrlRegs.GPADIR.bit.GPIO19=Default;
   GpioCtrlRegs.GPADIR.bit.GPIO20=OUTPUT;
   GpioCtrlRegs.GPADIR.bit.GPIO21=OUTPUT;	//QEPB
   GpioCtrlRegs.GPADIR.bit.GPIO22=INPUT;
   GpioCtrlRegs.GPADIR.bit.GPIO23=OUTPUT;
   GpioCtrlRegs.GPADIR.bit.GPIO24=OUTPUT;
  // GpioCtrlRegs.GPADIR.bit.GPIO25=OUTPUT;
  //GpioCtrlRegs.GPADIR.bit.GPIO26=OUTPUT;
  //GpioCtrlRegs.GPADIR.bit.GPIO27=OUTPUT;
   GpioCtrlRegs.GPADIR.bit.GPIO28=Default;	//SCIARX
   GpioCtrlRegs.GPADIR.bit.GPIO29=Default;	//SCIATX
   GpioCtrlRegs.GPADIR.bit.GPIO30=Default;	//CANRX
   GpioCtrlRegs.GPADIR.bit.GPIO31=Default;	//CANTX
 

   GpioCtrlRegs.GPBDIR.all = 0x0000;       // GPIO32-GPIO44 are inputs
   GpioCtrlRegs.GPBDIR.bit.GPIO32=OUTPUT;  //SDDA
   GpioCtrlRegs.GPBDIR.bit.GPIO33=OUTPUT;  //SLCK
   GpioCtrlRegs.GPBDIR.bit.GPIO34=OUTPUT;
   //GpioCtrlRegs.GPBDIR.bit.GPIO39=OUTPUT;
   //GpioCtrlRegs.GPBDIR.bit.GPIO40=OUTPUT;
   //GpioCtrlRegs.GPBDIR.bit.GPIO41=OUTPUT;
   //GpioCtrlRegs.GPBDIR.bit.GPIO42=OUTPUT;
   //GpioCtrlRegs.GPBDIR.bit.GPIO43=OUTPUT;
   //GpioCtrlRegs.GPBDIR.bit.GPIO44=OUTPUT;
   
  // GpioCtrlRegs.AIODIR.all = 0x0000;     // AIO2,4,6,19,12,14 are digital inputs
   
   GpioCtrlRegs.AIODIR.bit.AIO2=INPUT;
   GpioCtrlRegs.AIODIR.bit.AIO4=INPUT;
   GpioCtrlRegs.AIODIR.bit.AIO6=INPUT;
   GpioCtrlRegs.AIODIR.bit.AIO10=INPUT;
   GpioCtrlRegs.AIODIR.bit.AIO12=INPUT;
  // GpioCtrlRegs.AIODIR.bit.AIO14=INPUT;
   
   // Each input can have different qualification
   // a) input synchronized to SYSCLKOUT
   // b) input qualified by a sampling window
   // c) input sent asynchronously (valid for peripheral inputs only)
   GpioCtrlRegs.GPAQSEL1.all = 0x0000;    // GPIO0-GPIO15 Synch to SYSCLKOUT
   GpioCtrlRegs.GPAQSEL2.all = 0x0000;    // GPIO16-GPIO31 Synch to SYSCLKOUT
   GpioCtrlRegs.GPBQSEL1.all = 0x0000;    // GPIO32-GPIO44 Synch to SYSCLKOUT

   // Pull-ups can be enabled or disabled.
   GpioCtrlRegs.GPAPUD.all = 0x0000;      // Pullup's enabled GPIO0-GPIO31
   GpioCtrlRegs.GPBPUD.all = 0x0000;      // Pullup's enabled GPIO32-GPIO44
   //GpioCtrlRegs.GPAPUD.all = 0xFFFF;    // Pullup's disabled GPIO0-GPIO31
   //GpioCtrlRegs.GPBPUD.all = 0xFFFF;    // Pullup's disabled GPIO32-GPIO44
   EDIS;

}

void WG_InitSci(void)
{
    // Initialize SCI-A:
    SciaRegs.SCICTL1.bit.SWRESET=0;

    SciaRegs.SCICCR.all=7;   // 1 stop bit, No loopback, No parity, 8 char bits, async mode, idle-line protocol
    SciaRegs.SCICCR.bit.LOOPBKENA=0;
//    SciaRegs.SCICTL1.bit.RXENA=1;  // Enable TX, RX, internal SCICLK, SWRESET 0
    SciaRegs.SCICTL1.bit.TXENA=1;  // Disable RX ERR, SLEEP, TXWAKE
    SciaRegs.SCIHBAUD=SCIA_BRR_VAL>>8;
    SciaRegs.SCILBAUD=SCIA_BRR_VAL & 0XFF;
    SciaRegs.SCIPRI.bit.FREE=1;   //에뮬레이션 프리 모드

    //SCI 송신사용
//    SciaRegs.SCIFFTX.bit.SCIRST=1;
//    SciaRegs.SCIFFTX.bit.TXFIFOXRESET=1;
    SciaRegs.SCIFFCT.all=0x0000;    //Clear ABD 자동 BAUDRATE 설정
/*
    //SCI 수신 인터럽트
    SciaRegs.SCICTL2.bit.RXBKINTENA = 1;  // Rx 수신 인터럽트 허용
    SciaRegs.SCIFFRX.bit.RXFFINTCLR  = 1;    // SCI 수신 FIFO 인터럽트 플래그 클리어
    SciaRegs.SCIFFRX.bit.RXFIFORESET = 1;    // SCI 수신 FIFO RE-Enable
    SciaRegs.SCIFFRX.bit.RXFFIENA    = 1;    // SCI 수신 FIFO 인터럽트 Enable
    SciaRegs.SCIFFRX.bit.RXFFIL      = 16;   // SCI 수신 FIFO 인터럽트 레벨 설정
*/
    SciaRegs.SCICTL1.all =0x0023;     // Relinquish SCI from Reset
//  SciaRegs.SCIFFTX.bit.TXFIFOXRESET=1;
//  SciaRegs.SCIFFRX.bit.RXFIFORESET=1;
    SciaRegs.SCICTL1.bit.SWRESET=1; // SWRESET 1

}

void WG_InitECana(void)        // Initialize eCAN-A module
{

/* Create a shadow register structure for the CAN control registers. This is
 needed, since only 32-bit access is allowed to these registers. 16-bit access
 to these registers could potentially corrupt the register contents or return
 false data. */

   struct ECAN_REGS ECanaShadow;

    EALLOW;     // EALLOW enables access to protected bits

/* Configure eCAN RX and TX pins for CAN operation using eCAN regs*/

    ECanaShadow.CANTIOC.all = ECanaRegs.CANTIOC.all;
    ECanaShadow.CANTIOC.bit.TXFUNC = 1;
    ECanaRegs.CANTIOC.all = ECanaShadow.CANTIOC.all;  //ByCHOO:32bit로만 접근 가능하기에 이렇게 통짜로 복사하는 방법을 쓴다 

    ECanaShadow.CANRIOC.all = ECanaRegs.CANRIOC.all;
    ECanaShadow.CANRIOC.bit.RXFUNC = 1;
    ECanaRegs.CANRIOC.all = ECanaShadow.CANRIOC.all;

/* Configure eCAN for HECC mode - (reqd to access mailboxes 16 thru 31) */
                                    // HECC mode also enables time-stamping feature

    ECanaShadow.CANMC.all = ECanaRegs.CANMC.all;
    ECanaShadow.CANMC.bit.SCB = 1;  // eCAN Mode. 모든 Mail Box를 사용할 수 있다.
	ECanaShadow.CANMC.bit.ABO = 1;  //
	ECanaShadow.CANMC.bit.DBO = 0;
    ECanaRegs.CANMC.all = ECanaShadow.CANMC.all;


/* Initialize all bits of 'Message Control Register' to zero */
// Some bits of MSGCTRL register come up in an unknown state. For proper operation,
// all bits (including reserved bits) of MSGCTRL must be initialized to zero

    ECanaMboxes.MBOX0.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX1.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX2.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX3.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX4.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX5.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX6.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX7.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX8.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX9.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX10.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX11.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX12.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX13.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX14.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX15.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX16.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX17.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX18.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX19.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX20.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX21.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX22.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX23.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX24.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX25.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX26.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX27.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX28.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX29.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX30.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX31.MSGCTRL.all = 0x00000000;

// TAn, RMPn, GIFn bits are all zero upon reset and are cleared again
//  as a matter of precaution.

    ECanaRegs.CANTA.all   = 0xFFFFFFFF;  /* Clear all TAn bits */
    ECanaRegs.CANRMP.all  = 0xFFFFFFFF;  /* Clear all RMPn bits */
    ECanaRegs.CANGIF0.all = 0xFFFFFFFF;  /* Clear all interrupt flag bits */
    ECanaRegs.CANGIF1.all = 0xFFFFFFFF;


//ID 설정
	

    ECanaMboxes.MBOX0.MSGID.all     = 0;
    ECanaMboxes.MBOX1.MSGID.all     = 0;
    ECanaMboxes.MBOX2.MSGID.all     = 0;
    ECanaMboxes.MBOX3.MSGID.all     = 0;
    ECanaMboxes.MBOX4.MSGID.all     = 0;
    ECanaMboxes.MBOX5.MSGID.all     = 0;
    ECanaMboxes.MBOX6.MSGID.all     = 0;
    ECanaMboxes.MBOX7.MSGID.all     = 0;
    ECanaMboxes.MBOX8.MSGID.all     = 0;
    ECanaMboxes.MBOX9.MSGID.all     = 0;
    ECanaMboxes.MBOX10.MSGID.all    = 0;
    ECanaMboxes.MBOX11.MSGID.all    = 0;
    ECanaMboxes.MBOX12.MSGID.all    = 0;
    ECanaMboxes.MBOX23.MSGID.all    = 0;
    ECanaMboxes.MBOX14.MSGID.all    = 0;
    ECanaMboxes.MBOX15.MSGID.all    = 0;
    ECanaMboxes.MBOX16.MSGID.all    = 0;
    ECanaMboxes.MBOX17.MSGID.all    = 0;
    ECanaMboxes.MBOX18.MSGID.all    = 0;
    ECanaMboxes.MBOX19.MSGID.all    = 0;
    ECanaMboxes.MBOX20.MSGID.all    = 0;
    ECanaMboxes.MBOX21.MSGID.all    = 0;
    ECanaMboxes.MBOX22.MSGID.all    = 0;
    ECanaMboxes.MBOX23.MSGID.all    = 0;
    ECanaMboxes.MBOX24.MSGID.all    = 0;
    ECanaMboxes.MBOX25.MSGID.all    = 0;
    ECanaMboxes.MBOX26.MSGID.all    = 0;
    ECanaMboxes.MBOX27.MSGID.all    = 0;
    ECanaMboxes.MBOX28.MSGID.all    = 0;
    ECanaMboxes.MBOX29.MSGID.all    = 0;
    ECanaMboxes.MBOX30.MSGID.all    = 0;
    ECanaMboxes.MBOX31.MSGID.all    = 0;

    #if(RackNum==1)
    ECanaMboxes.MBOX0.MSGID.bit.STDMSGID  = 0x510;
    ECanaMboxes.MBOX2.MSGID.bit.STDMSGID  = 0x512;
    ECanaMboxes.MBOX1.MSGID.bit.STDMSGID  = 0x511;
	ECanaMboxes.MBOX4.MSGID.bit.STDMSGID  = 0x004;
	ECanaMboxes.MBOX5.MSGID.bit.STDMSGID  = 0x005;
	ECanaMboxes.MBOX6.MSGID.bit.STDMSGID  = 0x006;
	ECanaMboxes.MBOX7.MSGID.bit.STDMSGID  = 0x007;
	ECanaMboxes.MBOX8.MSGID.bit.STDMSGID  = 0x008;
	ECanaMboxes.MBOX9.MSGID.bit.STDMSGID  = 0x009;
	ECanaMboxes.MBOX10.MSGID.bit.STDMSGID = 0x00A;
	ECanaMboxes.MBOX11.MSGID.bit.STDMSGID = 0x00B;
	ECanaMboxes.MBOX12.MSGID.bit.STDMSGID = 0x00C;
	ECanaMboxes.MBOX13.MSGID.bit.STDMSGID = 0x00D;
	ECanaMboxes.MBOX14.MSGID.bit.STDMSGID = 0x00E;
	ECanaMboxes.MBOX15.MSGID.bit.STDMSGID = 0x00F;
	ECanaMboxes.MBOX16.MSGID.bit.STDMSGID = 0x010;
	ECanaMboxes.MBOX17.MSGID.bit.STDMSGID = 0x011;
	ECanaMboxes.MBOX18.MSGID.bit.STDMSGID = 0x012;
	ECanaMboxes.MBOX19.MSGID.bit.STDMSGID = 0x013;
	ECanaMboxes.MBOX20.MSGID.bit.STDMSGID = 0x014;
	ECanaMboxes.MBOX21.MSGID.bit.STDMSGID = 0x015;
	ECanaMboxes.MBOX22.MSGID.bit.STDMSGID = 0x016;
	ECanaMboxes.MBOX23.MSGID.bit.STDMSGID = 0x017;
	ECanaMboxes.MBOX24.MSGID.bit.STDMSGID = 0x018;
	ECanaMboxes.MBOX25.MSGID.bit.STDMSGID = 0x019;
	ECanaMboxes.MBOX26.MSGID.bit.STDMSGID = 0x01A;
	ECanaMboxes.MBOX27.MSGID.bit.STDMSGID = 0x01B;
	ECanaMboxes.MBOX28.MSGID.bit.STDMSGID = 0x01C;
	ECanaMboxes.MBOX29.MSGID.bit.STDMSGID = 0x01D;
	ECanaMboxes.MBOX30.MSGID.bit.STDMSGID = 0x01E;
	ECanaMboxes.MBOX31.MSGID.bit.STDMSGID = 0x01F;
    #endif
    #if(RackNum==2)
    ECanaMboxes.MBOX0.MSGID.bit.STDMSGID  = 0x520;
    ECanaMboxes.MBOX2.MSGID.bit.STDMSGID  = 0x522;
    ECanaMboxes.MBOX1.MSGID.bit.STDMSGID  = 0x521;
    ECanaMboxes.MBOX4.MSGID.bit.STDMSGID  = 0x004;
    ECanaMboxes.MBOX5.MSGID.bit.STDMSGID  = 0x005;
    ECanaMboxes.MBOX6.MSGID.bit.STDMSGID  = 0x006;
    ECanaMboxes.MBOX7.MSGID.bit.STDMSGID  = 0x007;
    ECanaMboxes.MBOX8.MSGID.bit.STDMSGID  = 0x008;
    ECanaMboxes.MBOX9.MSGID.bit.STDMSGID  = 0x009;
    ECanaMboxes.MBOX10.MSGID.bit.STDMSGID = 0x00A;
    ECanaMboxes.MBOX11.MSGID.bit.STDMSGID = 0x00B;
    ECanaMboxes.MBOX12.MSGID.bit.STDMSGID = 0x00C;
    ECanaMboxes.MBOX13.MSGID.bit.STDMSGID = 0x00D;
    ECanaMboxes.MBOX14.MSGID.bit.STDMSGID = 0x00E;
    ECanaMboxes.MBOX15.MSGID.bit.STDMSGID = 0x00F;
    ECanaMboxes.MBOX16.MSGID.bit.STDMSGID = 0x010;
    ECanaMboxes.MBOX17.MSGID.bit.STDMSGID = 0x011;
    ECanaMboxes.MBOX18.MSGID.bit.STDMSGID = 0x012;
    ECanaMboxes.MBOX19.MSGID.bit.STDMSGID = 0x013;
    ECanaMboxes.MBOX20.MSGID.bit.STDMSGID = 0x014;
    ECanaMboxes.MBOX21.MSGID.bit.STDMSGID = 0x015;
    ECanaMboxes.MBOX22.MSGID.bit.STDMSGID = 0x016;
    ECanaMboxes.MBOX23.MSGID.bit.STDMSGID = 0x017;
    ECanaMboxes.MBOX24.MSGID.bit.STDMSGID = 0x018;
    ECanaMboxes.MBOX25.MSGID.bit.STDMSGID = 0x019;
    ECanaMboxes.MBOX26.MSGID.bit.STDMSGID = 0x01A;
    ECanaMboxes.MBOX27.MSGID.bit.STDMSGID = 0x01B;
    ECanaMboxes.MBOX28.MSGID.bit.STDMSGID = 0x01C;
    ECanaMboxes.MBOX29.MSGID.bit.STDMSGID = 0x01D;
    ECanaMboxes.MBOX30.MSGID.bit.STDMSGID = 0x01E;
    ECanaMboxes.MBOX31.MSGID.bit.STDMSGID = 0x01F;
    #endif
    #if(RackNum==3)
    ECanaMboxes.MBOX0.MSGID.bit.STDMSGID  = 0x530;
    ECanaMboxes.MBOX2.MSGID.bit.STDMSGID  = 0x532;
    ECanaMboxes.MBOX1.MSGID.bit.STDMSGID  = 0x531;
    ECanaMboxes.MBOX4.MSGID.bit.STDMSGID  = 0x004;
    ECanaMboxes.MBOX5.MSGID.bit.STDMSGID  = 0x005;
    ECanaMboxes.MBOX6.MSGID.bit.STDMSGID  = 0x006;
    ECanaMboxes.MBOX7.MSGID.bit.STDMSGID  = 0x007;
    ECanaMboxes.MBOX8.MSGID.bit.STDMSGID  = 0x008;
    ECanaMboxes.MBOX9.MSGID.bit.STDMSGID  = 0x009;
    ECanaMboxes.MBOX10.MSGID.bit.STDMSGID = 0x00A;
    ECanaMboxes.MBOX11.MSGID.bit.STDMSGID = 0x00B;
    ECanaMboxes.MBOX12.MSGID.bit.STDMSGID = 0x00C;
    ECanaMboxes.MBOX13.MSGID.bit.STDMSGID = 0x00D;
    ECanaMboxes.MBOX14.MSGID.bit.STDMSGID = 0x00E;
    ECanaMboxes.MBOX15.MSGID.bit.STDMSGID = 0x00F;
    ECanaMboxes.MBOX16.MSGID.bit.STDMSGID = 0x010;
    ECanaMboxes.MBOX17.MSGID.bit.STDMSGID = 0x011;
    ECanaMboxes.MBOX18.MSGID.bit.STDMSGID = 0x012;
    ECanaMboxes.MBOX19.MSGID.bit.STDMSGID = 0x013;
    ECanaMboxes.MBOX20.MSGID.bit.STDMSGID = 0x014;
    ECanaMboxes.MBOX21.MSGID.bit.STDMSGID = 0x015;
    ECanaMboxes.MBOX22.MSGID.bit.STDMSGID = 0x016;
    ECanaMboxes.MBOX23.MSGID.bit.STDMSGID = 0x017;
    ECanaMboxes.MBOX24.MSGID.bit.STDMSGID = 0x018;
    ECanaMboxes.MBOX25.MSGID.bit.STDMSGID = 0x019;
    ECanaMboxes.MBOX26.MSGID.bit.STDMSGID = 0x01A;
    ECanaMboxes.MBOX27.MSGID.bit.STDMSGID = 0x01B;
    ECanaMboxes.MBOX28.MSGID.bit.STDMSGID = 0x01C;
    ECanaMboxes.MBOX29.MSGID.bit.STDMSGID = 0x01D;
    ECanaMboxes.MBOX30.MSGID.bit.STDMSGID = 0x01E;
    ECanaMboxes.MBOX31.MSGID.bit.STDMSGID = 0x01F;
    #endif
    #if(RackNum==4)
    ECanaMboxes.MBOX0.MSGID.bit.STDMSGID  = 0x540;
    ECanaMboxes.MBOX2.MSGID.bit.STDMSGID  = 0x542;
    ECanaMboxes.MBOX1.MSGID.bit.STDMSGID  = 0x541;
    ECanaMboxes.MBOX4.MSGID.bit.STDMSGID  = 0x004;
    ECanaMboxes.MBOX5.MSGID.bit.STDMSGID  = 0x005;
    ECanaMboxes.MBOX6.MSGID.bit.STDMSGID  = 0x006;
    ECanaMboxes.MBOX7.MSGID.bit.STDMSGID  = 0x007;
    ECanaMboxes.MBOX8.MSGID.bit.STDMSGID  = 0x008;
    ECanaMboxes.MBOX9.MSGID.bit.STDMSGID  = 0x009;
    ECanaMboxes.MBOX10.MSGID.bit.STDMSGID = 0x00A;
    ECanaMboxes.MBOX11.MSGID.bit.STDMSGID = 0x00B;
    ECanaMboxes.MBOX12.MSGID.bit.STDMSGID = 0x00C;
    ECanaMboxes.MBOX13.MSGID.bit.STDMSGID = 0x00D;
    ECanaMboxes.MBOX14.MSGID.bit.STDMSGID = 0x00E;
    ECanaMboxes.MBOX15.MSGID.bit.STDMSGID = 0x00F;
    ECanaMboxes.MBOX16.MSGID.bit.STDMSGID = 0x010;
    ECanaMboxes.MBOX17.MSGID.bit.STDMSGID = 0x011;
    ECanaMboxes.MBOX18.MSGID.bit.STDMSGID = 0x012;
    ECanaMboxes.MBOX19.MSGID.bit.STDMSGID = 0x013;
    ECanaMboxes.MBOX20.MSGID.bit.STDMSGID = 0x014;
    ECanaMboxes.MBOX21.MSGID.bit.STDMSGID = 0x015;
    ECanaMboxes.MBOX22.MSGID.bit.STDMSGID = 0x016;
    ECanaMboxes.MBOX23.MSGID.bit.STDMSGID = 0x017;
    ECanaMboxes.MBOX24.MSGID.bit.STDMSGID = 0x018;
    ECanaMboxes.MBOX25.MSGID.bit.STDMSGID = 0x019;
    ECanaMboxes.MBOX26.MSGID.bit.STDMSGID = 0x01A;
    ECanaMboxes.MBOX27.MSGID.bit.STDMSGID = 0x01B;
    ECanaMboxes.MBOX28.MSGID.bit.STDMSGID = 0x01C;
    ECanaMboxes.MBOX29.MSGID.bit.STDMSGID = 0x01D;
    ECanaMboxes.MBOX30.MSGID.bit.STDMSGID = 0x01E;
    ECanaMboxes.MBOX31.MSGID.bit.STDMSGID = 0x01F;
    #endif


/* Configure bit timing parameters for eCANA*/

    ECanaShadow.CANMC.all = ECanaRegs.CANMC.all;
    ECanaShadow.CANMC.bit.CCR = 1 ;            // Set CCR = 1
    ECanaRegs.CANMC.all = ECanaShadow.CANMC.all;

    // Wait until the CPU has been granted permission to change the configuration registers
    do
    {
      ECanaShadow.CANES.all = ECanaRegs.CANES.all;
    } while(ECanaShadow.CANES.bit.CCE != 1 );       // Wait for CCE bit to be set..

    ECanaShadow.CANBTC.all = 0;



    //Identifier mask는 어디에 넣어야 하는가? 아래의 mask setting은 intialization(CCR=1, CCE=0)인 동안만 가능하다.

    /* Write to the mailbox RAM field, 이건 아직 잘 모르겠네... */
    //AMI가 1, IDE =0 이면 The RECEIVED message had a standard identifier
    //AMI = 1 일때 Filtering Criterion must be satisfied on order to receive message
    ECanaShadow.CANGAM.all = ECanaRegs.CANGAM.all;
    ECanaShadow.CANGAM.bit.AMI = 0;
    //ECanaShadow.CANGAM.all = 0xFFFFFFFF; //byCHOO
    ECanaRegs.CANGAM.all   = ECanaShadow.CANGAM.all;


    ECanaMboxes.MBOX0.MSGID.bit.AME = 0;
    ECanaMboxes.MBOX1.MSGID.bit.AME = 0;
    ECanaMboxes.MBOX2.MSGID.bit.AME = 0;
    ECanaMboxes.MBOX3.MSGID.bit.AME = 0;

	#if(CAN_1MBPS)
    ECanaShadow.CANBTC.bit.BRPREG =  2;
    ECanaShadow.CANBTC.bit.TSEG2REG = 1;
	ECanaShadow.CANBTC.bit.TSEG1REG = 6;
    ECanaShadow.CANBTC.bit.SAM = 1;  //ByCHOO : SAM : Thripple Sampling. BRP > 4 일 경우만 가능하다고 하는데.. 그러면 BRPREG가 3보다 커야 하는 것 아닌가?
	#endif
	
	#if(CAN_500KBPS)
    ECanaShadow.CANBTC.bit.BRPREG =  5;
    ECanaShadow.CANBTC.bit.TSEG2REG = 1;
	ECanaShadow.CANBTC.bit.TSEG1REG = 6;
    ECanaShadow.CANBTC.bit.SAM = 1;  //ByCHOO : SAM : Thripple Sampling. BRP > 4 일 경우만 가능하다고 하는데.. 그러면 BRPREG가 3보다 커야 하는 것 아닌가?
	#endif

    #if(CAN_250KBPS)
    ECanaShadow.CANBTC.bit.BRPREG = 11;
    ECanaShadow.CANBTC.bit.TSEG2REG = 1;
    ECanaShadow.CANBTC.bit.TSEG1REG = 6;
    ECanaShadow.CANBTC.bit.SAM = 1;
    #endif

	ECanaRegs.CANBTC.all = ECanaShadow.CANBTC.all;
    ECanaShadow.CANMC.all = ECanaRegs.CANMC.all;
    ECanaShadow.CANMC.bit.CCR = 0 ;            // Set CCR = 0
    ECanaRegs.CANMC.all = ECanaShadow.CANMC.all;
    // Wait until the CPU no longer has permission to change the configuration registers
	//ECanaShadow.CANES.all = ECanaRegs.CANES.all;
	do
    {
      ECanaShadow.CANES.all = ECanaRegs.CANES.all;
    } while(ECanaShadow.CANES.bit.CCE != 0 );       // Wait for CCE bit to be  cleared..

/* Disable all Mailboxes  */
    ECanaRegs.CANME.all = 0;        // Required before writing the MSGIDs
    EDIS;

	//ByCHOO : 여기서부터 User Setting 부분 TX, RX Interrupt 설정	

    // ByCHOO 두개의 인터럽트 서비스 중 하나를 선택, 32개 모두를 0번 인터럽트에 배당
	EALLOW;
	ECanaShadow.CANMIL.all = ECanaRegs.CANMIL.all;		// 1 이면 해당 MailBox 인트럽트 Generateon 1 선언함 ; 0 이면 해당 MailBox 인트럽트 Generateon 0 선언함
 	ECanaShadow.CANMIL.all = 0x00000000;                //ByCHOO 0이면 interrupt 0에 연결. 나중에 Main에서  PieVectTable.ECAN0INTA 	= &ISR_CANRXINTA; 이런 식으로 연계
	ECanaRegs.CANMIL.all  = ECanaShadow.CANMIL.all;

	//해당 MailBox를 RX, TX 선정

	ECanaShadow.CANMD.all = ECanaRegs.CANMD.all;		// 해당 MailBox을 1:RX, 0:TX 선정함. 우선은 모두 RX 로 설정. 32개 메일박스 RX인터럽트 확인한다.
    ECanaShadow.CANMD.bit.MD0=1;   //RX:Pack
    ECanaShadow.CANMD.bit.MD1=1;   //RX:CT
    ECanaShadow.CANMD.bit.MD2=1;   //RX:HMI
    ECanaShadow.CANMD.bit.MD3=0;
    ECanaShadow.CANMD.bit.MD4=0;
    ECanaShadow.CANMD.bit.MD5=0;
    ECanaShadow.CANMD.bit.MD6=0;
    ECanaShadow.CANMD.bit.MD7=0;
    ECanaShadow.CANMD.bit.MD8=0;
    ECanaShadow.CANMD.bit.MD9=0;
    ECanaShadow.CANMD.bit.MD10=0;
    ECanaShadow.CANMD.bit.MD11=0;
    ECanaShadow.CANMD.bit.MD12=0;
    ECanaShadow.CANMD.bit.MD13=0;
    ECanaShadow.CANMD.bit.MD14=0;
    ECanaShadow.CANMD.bit.MD15=0;
    ECanaShadow.CANMD.bit.MD16=0;
    ECanaShadow.CANMD.bit.MD17=0;
    ECanaShadow.CANMD.bit.MD18=0;
    ECanaShadow.CANMD.bit.MD19=0;
    ECanaShadow.CANMD.bit.MD20=0;
    ECanaShadow.CANMD.bit.MD21=0;
    ECanaShadow.CANMD.bit.MD22=0;
    ECanaShadow.CANMD.bit.MD23=0;
    ECanaShadow.CANMD.bit.MD24=0;
    ECanaShadow.CANMD.bit.MD25=0;
    ECanaShadow.CANMD.bit.MD26=0;
    ECanaShadow.CANMD.bit.MD27=0;
    ECanaShadow.CANMD.bit.MD28=0;
    ECanaShadow.CANMD.bit.MD29=0;
    ECanaShadow.CANMD.bit.MD30=0;
    ECanaShadow.CANMD.bit.MD31=0;

	ECanaRegs.CANMD.all = ECanaShadow.CANMD.all;
	
	// 해당 MailBox를 CAN Enable 시킴 
	ECanaShadow.CANME.all = ECanaRegs.CANME.all;
    ECanaShadow.CANME.bit.ME0= 1;
    ECanaShadow.CANME.bit.ME1= 1;
    ECanaShadow.CANME.bit.ME2= 1;
    ECanaShadow.CANME.bit.ME3= 0;
    ECanaShadow.CANME.bit.ME4= 0;
    ECanaShadow.CANME.bit.ME5= 0;
    ECanaShadow.CANME.bit.ME6= 0;
    ECanaShadow.CANME.bit.ME7= 0;
    ECanaShadow.CANME.bit.ME8= 0;
    ECanaShadow.CANME.bit.ME9= 0;
    ECanaShadow.CANME.bit.ME10= 0;
    ECanaShadow.CANME.bit.ME11= 0;
    ECanaShadow.CANME.bit.ME12= 0;
    ECanaShadow.CANME.bit.ME13= 0;
    ECanaShadow.CANME.bit.ME14= 0;
    ECanaShadow.CANME.bit.ME15= 0;
    ECanaShadow.CANME.bit.ME16= 0;
    ECanaShadow.CANME.bit.ME17= 0;
    ECanaShadow.CANME.bit.ME18= 0;
    ECanaShadow.CANME.bit.ME19= 0;
    ECanaShadow.CANME.bit.ME20= 0;
    ECanaShadow.CANME.bit.ME21= 0;
    ECanaShadow.CANME.bit.ME22= 0;
    ECanaShadow.CANME.bit.ME23= 0;
    ECanaShadow.CANME.bit.ME24= 0;
    ECanaShadow.CANME.bit.ME25= 0;
    ECanaShadow.CANME.bit.ME26= 0;
    ECanaShadow.CANME.bit.ME27= 0;
    ECanaShadow.CANME.bit.ME28= 0;
    ECanaShadow.CANME.bit.ME29= 0;
    ECanaShadow.CANME.bit.ME30= 0;
    ECanaShadow.CANME.bit.ME31= 1;


	ECanaRegs.CANME.all = ECanaShadow.CANME.all;

	
	// CAN 0 인터럽트 활성화  시킴 
	ECanaShadow.CANGIM.all = ECanaRegs.CANGIM.all;
	ECanaShadow.CANGIM.bit.I0EN = 1;								
	ECanaRegs.CANGIM.all   = ECanaShadow.CANGIM.all; 

	//Mailbox 별로 Interrupt를 Mask 함 Mask 된 것만 인터럽트 발생 가능
	ECanaShadow.CANMIM.all = ECanaRegs.CANMIM.all;	
    ECanaShadow.CANMIM.bit.MIM0 = 1;
    ECanaShadow.CANMIM.bit.MIM1 = 1;
    ECanaShadow.CANMIM.bit.MIM2 = 1;
    ECanaShadow.CANMIM.bit.MIM3 = 0;
    ECanaShadow.CANMIM.bit.MIM4=  0;
    ECanaShadow.CANMIM.bit.MIM5 = 0;
    ECanaShadow.CANMIM.bit.MIM6 = 0;
    ECanaShadow.CANMIM.bit.MIM7 = 0;
    ECanaShadow.CANMIM.bit.MIM8 = 0;
    ECanaShadow.CANMIM.bit.MIM9 = 0;
    ECanaShadow.CANMIM.bit.MIM10 = 0;

    ECanaShadow.CANMIM.bit.MIM11 = 0;
    ECanaShadow.CANMIM.bit.MIM12 = 0;
    ECanaShadow.CANMIM.bit.MIM13 = 0;
    ECanaShadow.CANMIM.bit.MIM14=  0;
    ECanaShadow.CANMIM.bit.MIM15 = 0;
    ECanaShadow.CANMIM.bit.MIM16 = 0;
    ECanaShadow.CANMIM.bit.MIM17 = 0;
    ECanaShadow.CANMIM.bit.MIM18 = 0;
    ECanaShadow.CANMIM.bit.MIM19 = 0;
    ECanaShadow.CANMIM.bit.MIM20 = 0;

    ECanaShadow.CANMIM.bit.MIM21 = 0;
    ECanaShadow.CANMIM.bit.MIM22 = 0;
    ECanaShadow.CANMIM.bit.MIM23 = 0;
    ECanaShadow.CANMIM.bit.MIM24=  0;
    ECanaShadow.CANMIM.bit.MIM25 = 0;
    ECanaShadow.CANMIM.bit.MIM26 = 0;
    ECanaShadow.CANMIM.bit.MIM27 = 0;
    ECanaShadow.CANMIM.bit.MIM28 = 0;
    ECanaShadow.CANMIM.bit.MIM29 = 0;
    ECanaShadow.CANMIM.bit.MIM30 = 0;
    ECanaShadow.CANMIM.bit.MIM31 = 0;

	ECanaRegs.CANMIM.all   = ECanaShadow.CANMIM.all;

	/* Write to DLC field in Master Control reg  Data Size를 8byte로 선언*/
	ECanaMboxes.MBOX31.MSGCTRL.bit.DLC = 8;

	/*
	ECanaMboxes.MBOX0.MSGCTRL.bit.DLC = 8;
    ECanaMboxes.MBOX1.MSGCTRL.bit.DLC = 8;
    ECanaMboxes.MBOX2.MSGCTRL.bit.DLC = 8;

    ECanaMboxes.MBOX3.MSGCTRL.bit.DLC = 8;
    ECanaMboxes.MBOX4.MSGCTRL.bit.DLC = 8;
    ECanaMboxes.MBOX5.MSGCTRL.bit.DLC = 8;
    ECanaMboxes.MBOX6.MSGCTRL.bit.DLC = 8;
    ECanaMboxes.MBOX7.MSGCTRL.bit.DLC = 8;
    ECanaMboxes.MBOX8.MSGCTRL.bit.DLC = 8;
    ECanaMboxes.MBOX9.MSGCTRL.bit.DLC = 8;
    ECanaMboxes.MBOX10.MSGCTRL.bit.DLC = 8;

    ECanaMboxes.MBOX11.MSGCTRL.bit.DLC = 8;
    ECanaMboxes.MBOX12.MSGCTRL.bit.DLC = 8;
    ECanaMboxes.MBOX13.MSGCTRL.bit.DLC = 8;
    ECanaMboxes.MBOX14.MSGCTRL.bit.DLC = 8;
    ECanaMboxes.MBOX15.MSGCTRL.bit.DLC = 8;
    ECanaMboxes.MBOX16.MSGCTRL.bit.DLC = 8;
    ECanaMboxes.MBOX17.MSGCTRL.bit.DLC = 8;
    ECanaMboxes.MBOX18.MSGCTRL.bit.DLC = 8;
    ECanaMboxes.MBOX19.MSGCTRL.bit.DLC = 8;
    ECanaMboxes.MBOX20.MSGCTRL.bit.DLC = 8;

    ECanaMboxes.MBOX21.MSGCTRL.bit.DLC = 8;
    ECanaMboxes.MBOX22.MSGCTRL.bit.DLC = 8;
    ECanaMboxes.MBOX23.MSGCTRL.bit.DLC = 8;
    ECanaMboxes.MBOX24.MSGCTRL.bit.DLC = 8;
    ECanaMboxes.MBOX25.MSGCTRL.bit.DLC = 8;
    ECanaMboxes.MBOX26.MSGCTRL.bit.DLC = 8;
    ECanaMboxes.MBOX27.MSGCTRL.bit.DLC = 8;
    ECanaMboxes.MBOX28.MSGCTRL.bit.DLC = 8;
    ECanaMboxes.MBOX29.MSGCTRL.bit.DLC = 8;
    ECanaMboxes.MBOX30.MSGCTRL.bit.DLC = 8;
    */

	ECanaMboxes.MBOX31.MSGID.bit.IDE = 0;
    ECanaMboxes.MBOX0.MSGID.bit.IDE  = 0;
    ECanaMboxes.MBOX1.MSGID.bit.IDE  = 0;
    ECanaMboxes.MBOX2.MSGID.bit.IDE  = 0;
    /*
    ECanaMboxes.MBOX3.MSGID.bit.IDE = 0;
    ECanaMboxes.MBOX4.MSGID.bit.IDE = 0;
    ECanaMboxes.MBOX5.MSGID.bit.IDE = 0;
    ECanaMboxes.MBOX6.MSGID.bit.IDE = 0;
    ECanaMboxes.MBOX7.MSGID.bit.IDE = 0;
    ECanaMboxes.MBOX8.MSGID.bit.IDE = 0;
    ECanaMboxes.MBOX9.MSGID.bit.IDE = 0;
    ECanaMboxes.MBOX10.MSGID.bit.IDE = 0;

    ECanaMboxes.MBOX11.MSGID.bit.IDE = 0;
    ECanaMboxes.MBOX12.MSGID.bit.IDE = 0;
    ECanaMboxes.MBOX13.MSGID.bit.IDE = 0;
    ECanaMboxes.MBOX14.MSGID.bit.IDE = 0;
    ECanaMboxes.MBOX15.MSGID.bit.IDE = 0;
    ECanaMboxes.MBOX16.MSGID.bit.IDE = 0;
    ECanaMboxes.MBOX17.MSGID.bit.IDE = 0;
    ECanaMboxes.MBOX18.MSGID.bit.IDE = 0;
    ECanaMboxes.MBOX19.MSGID.bit.IDE = 0;
    ECanaMboxes.MBOX20.MSGID.bit.IDE = 0;

    ECanaMboxes.MBOX21.MSGID.bit.IDE = 0;
    ECanaMboxes.MBOX22.MSGID.bit.IDE = 0;
    ECanaMboxes.MBOX23.MSGID.bit.IDE = 0;
    ECanaMboxes.MBOX24.MSGID.bit.IDE = 0;
    ECanaMboxes.MBOX25.MSGID.bit.IDE = 0;
    ECanaMboxes.MBOX26.MSGID.bit.IDE = 0;
    ECanaMboxes.MBOX27.MSGID.bit.IDE = 0;
    ECanaMboxes.MBOX28.MSGID.bit.IDE = 0;
    ECanaMboxes.MBOX29.MSGID.bit.IDE = 0;
    ECanaMboxes.MBOX30.MSGID.bit.IDE = 0;
    */

	//Message Box 값을 모두 초기화


    ECanaMboxes.MBOX0.MDL.all = 0x00000000;
    ECanaMboxes.MBOX0.MDH.all = 0x00000000;

    ECanaMboxes.MBOX1.MDL.all = 0x00000000;
    ECanaMboxes.MBOX1.MDH.all = 0x00000000;

    ECanaMboxes.MBOX2.MDL.all = 0x00000000;
    ECanaMboxes.MBOX2.MDH.all = 0x00000000;

    ECanaMboxes.MBOX3.MDL.all = 0x00000000;
    ECanaMboxes.MBOX3.MDH.all = 0x00000000;

    ECanaMboxes.MBOX4.MDL.all = 0x00000000;
    ECanaMboxes.MBOX4.MDH.all = 0x00000000;

    ECanaMboxes.MBOX5.MDL.all = 0x00000000;
    ECanaMboxes.MBOX5.MDH.all = 0x00000000;

    ECanaMboxes.MBOX6.MDL.all = 0x00000000;
    ECanaMboxes.MBOX6.MDH.all = 0x00000000;

    ECanaMboxes.MBOX7.MDL.all = 0x00000000;
    ECanaMboxes.MBOX7.MDH.all = 0x00000000;

    ECanaMboxes.MBOX8.MDL.all = 0x00000000;
    ECanaMboxes.MBOX8.MDH.all = 0x00000000;

    ECanaMboxes.MBOX9.MDL.all = 0x00000000;
    ECanaMboxes.MBOX9.MDH.all = 0x00000000;

    ECanaMboxes.MBOX10.MDL.all = 0x00000000;
    ECanaMboxes.MBOX10.MDH.all = 0x00000000;


    ECanaMboxes.MBOX11.MDL.all = 0x00000000;
    ECanaMboxes.MBOX11.MDH.all = 0x00000000;

    ECanaMboxes.MBOX12.MDL.all = 0x00000000;
    ECanaMboxes.MBOX12.MDH.all = 0x00000000;

    ECanaMboxes.MBOX13.MDL.all = 0x00000000;
    ECanaMboxes.MBOX13.MDH.all = 0x00000000;

    ECanaMboxes.MBOX14.MDL.all = 0x00000000;
    ECanaMboxes.MBOX14.MDH.all = 0x00000000;

    ECanaMboxes.MBOX15.MDL.all = 0x00000000;
    ECanaMboxes.MBOX15.MDH.all = 0x00000000;

    ECanaMboxes.MBOX16.MDL.all = 0x00000000;
    ECanaMboxes.MBOX16.MDH.all = 0x00000000;

    ECanaMboxes.MBOX17.MDL.all = 0x00000000;
    ECanaMboxes.MBOX17.MDH.all = 0x00000000;

    ECanaMboxes.MBOX18.MDL.all = 0x00000000;
    ECanaMboxes.MBOX18.MDH.all = 0x00000000;

    ECanaMboxes.MBOX19.MDL.all = 0x00000000;
    ECanaMboxes.MBOX19.MDH.all = 0x00000000;

    ECanaMboxes.MBOX20.MDL.all = 0x00000000;
    ECanaMboxes.MBOX20.MDH.all = 0x00000000;

    ECanaMboxes.MBOX21.MDL.all = 0x00000000;
    ECanaMboxes.MBOX21.MDH.all = 0x00000000;

    ECanaMboxes.MBOX22.MDL.all = 0x00000000;
    ECanaMboxes.MBOX22.MDH.all = 0x00000000;

    ECanaMboxes.MBOX23.MDL.all = 0x00000000;
    ECanaMboxes.MBOX23.MDH.all = 0x00000000;

    ECanaMboxes.MBOX24.MDL.all = 0x00000000;
    ECanaMboxes.MBOX24.MDH.all = 0x00000000;

    ECanaMboxes.MBOX25.MDL.all = 0x00000000;
    ECanaMboxes.MBOX25.MDH.all = 0x00000000;

    ECanaMboxes.MBOX26.MDL.all = 0x00000000;
    ECanaMboxes.MBOX26.MDH.all = 0x00000000;

    ECanaMboxes.MBOX27.MDL.all = 0x00000000;
    ECanaMboxes.MBOX27.MDH.all = 0x00000000;

    ECanaMboxes.MBOX28.MDL.all = 0x00000000;
    ECanaMboxes.MBOX28.MDH.all = 0x00000000;

    ECanaMboxes.MBOX29.MDL.all = 0x00000000;
    ECanaMboxes.MBOX29.MDH.all = 0x00000000;

    ECanaMboxes.MBOX30.MDL.all = 0x00000000;
    ECanaMboxes.MBOX30.MDH.all = 0x00000000;

    ECanaMboxes.MBOX31.MDL.all = 0x00000000;
    ECanaMboxes.MBOX31.MDH.all = 0x00000000;

    EDIS;
}
void WG_InitSPI(void)                                                      
{             
//	Uint16 i;

	SpiaRegs.SPICCR.bit.SPISWRESET 		= 0; 	// SPI SW Reset hold 
	SpiaRegs.SPICCR.bit.CLKPOLARITY 	= 1;    // Falling edge output
	SpiaRegs.SPICCR.bit.SPILBK 			= 0;	// Disable Loopback  
  	SpiaRegs.SPICCR.bit.SPICHAR 		= 0x7;	// 8bit character
                                                                                                                                                                                           
	SpiaRegs.SPICTL.bit.SPIINTENA 		= 0;	// Disable SPI int
	SpiaRegs.SPICTL.bit.TALK 			= 1;	// Tx enable
	SpiaRegs.SPICTL.bit.MASTER_SLAVE 	= 1;	// Master mode
	SpiaRegs.SPICTL.bit.CLK_PHASE 		= 0;	// without delay mode
	SpiaRegs.SPICTL.bit.OVERRUNINTENA 	= 0;	// Disable OverRun int     
	                                                              
	SpiaRegs.SPIBRR 					= 119;	// Baud rate = LSPCLK/(SPIBRR+1) 약  30Mhz/(119+1) = 250Khz
	                                           	// when SPIBRR 3 to 127
 	SpiaRegs.SPICCR.bit.SPISWRESET 		= 1; 	// SPI SW Reset release    
	
}

void SPI_Write(unsigned int WRData)
{
	unsigned int Dummy;
	unsigned int Tmp;
	SpiaRegs.SPICCR.bit.SPICHAR = 0x07;
	Dummy = (WRData<<8)&0xFF00;
	while(SpiaRegs.SPISTS.bit.BUFFULL_FLAG);
	SpiaRegs.SPITXBUF = Dummy;				// Send
	while(SpiaRegs.SPISTS.bit.INT_FLAG!=1);	// Wait for Tx   전송이 끝났거나 수신이 시작되면 1이됨.
	Tmp=SpiaRegs.SPIRXBUF;
	Tmp=Tmp;
}
unsigned int SPI_READ(void)
{
	Uint16 ReadData;
	Uint16 Dummy=0x0000;
	SpiaRegs.SPICCR.bit.SPICHAR = 0x07;
	Dummy = (Dummy<<8)&0xFF00;
	while(SpiaRegs.SPISTS.bit.BUFFULL_FLAG);
	SpiaRegs.SPITXBUF = Dummy;				// Send
	while(SpiaRegs.SPISTS.bit.INT_FLAG!=1);	// Wait for Tx전송이 끝났거나 수신이 시작되면 1이됨.
//	delay_us(30);
	ReadData = SpiaRegs.SPIRXBUF& 0xff;
	return (ReadData);
}

#define SPI_Read()	SPI_READ()

////////////////////////////////////////////////////////////////////////////////////////
// LTC6804 device drivers
// /////////////////////////////////////////////////////////////////////////////////////

/************************************
 * Copyright 2012 Linear Technology Corp. (LTC)
 * Permission to freely use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies:
 * THIS SOFTWARE IS PROVIDED “AS IS” AND LTC DISCLAIMS ALL WARRANTIES
 * INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO
 * EVENT SHALL LTC BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL
 * DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM ANY USE OF SAME, INCLUDING
 * ANY LOSS OF USE OR DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE
 * OR OTHER TORTUOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR
 * PERFORMANCE OF THIS SOFTWARE.
 * ***********************************************************/
unsigned short pec15Table[256];
unsigned short CRC15POLY = 0x4599;
void init_PEC15_Table(void)
{
	int i;
	int bit;

	for (i = 0; i < 256; i++)
	{
		unsigned short remainder = i << 7;
		for (bit = 8; bit > 0; --bit)
		{
			if (remainder & 0x4000)
			{
				remainder = (remainder << 1);
				remainder = (remainder ^ CRC15POLY);
			}
			else
			{
				remainder = (remainder << 1);
			}
		}
		pec15Table[i] = remainder;
	}
}

unsigned short pec15(char *data, int len)
{
	unsigned short remainder,address;
	int i;
	remainder = 16;//PEC seed
	for (i = 0; i < len; i++)
	{
		address = ((remainder >> 7) ^ data[i]) & 0xff;//calculate PEC table address
		remainder = (remainder << 8 ) ^ pec15Table[address];
	}
	return (remainder*2);//The CRC15 has a 0 in the LSB so the final value must be multiplied by 2
}

#define MAX_LTC6804 				16
#define LTC6804_CMD_WRCFG			(0x0001)
#define LTC6804_CMD_RDCFG			(0x0002)
#define LTC6804_CMD_RDCVA			(0x0004)
#define LTC6804_CMD_RDCVB			(0x0006)
#define LTC6804_CMD_RDCVC			(0x0008)
#define LTC6804_CMD_RDCVD			(0x000a)
#define LTC6804_CMD_RDAUXA		    (0x000c)
#define LTC6804_CMD_RDAUXB		    (0x000e)
#define LTC6804_CMD_RDSTATA		    (0x0010)
#define LTC6804_CMD_RDSTATB		    (0x0012)
#define LTC6804_CMD_ADCV		    (0x0260)
#define LTC6804_CMD_ADOW		    (0x0228)
#define LTC6804_CMD_CVST		    (0x0207)
#define LTC6804_CMD_ADAX		    (0x0460)
#define LTC6804_CMD_AXST		    (0x0407)
#define LTC6804_CMD_ADSTAT		    (0x0468)
#define LTC6804_CMD_STATST		    (0x040f)
//#define LTC6804_CMD_ADCVAX		(0x043f)
#define LTC6804_CMD_CLRCELL		    (0x0711)
#define LTC6804_CMD_CLRAUX		    (0x0712)
#define LTC6804_CMD_CLRSTAT	    	(0x0713)
#define LTC6804_CMD_PLADC		    (0x0714)
#define LTC6804_CMD_DIAGN	    	(0x0715)
#define LTC6804_CMD_WRCOMM	        (0x0721)
#define LTC6804_CMD_RDCOMM	    	(0x0722)
#define LTC6804_CMD_STCOMM	    	(0x0723)

typedef struct _LTC6804
{
	int count;
	char address[MAX_LTC6804];
} LTC6804_t;

LTC6804_t LTC6804;
int ltc_state;
int ltc_error_count;

int LTC6804_GetErrorCount(void)
{
	return ltc_error_count;
}


char LTC6804_init_table[6] = 
{
	0xfc,		// GPIOx : pull down off, Ref ON, SWTEN, ADCOPT=0
	0x00,		// VUV[7:0]
	0x00,		// VOV[3:0]VUV[11:8]
	0x00,		// VOV[11:4]
	0x00,		// DCC8~DCC1
	0x00		// DCTO disable, DCC12~DCC9
};

void BATSPIEnable_low(void)
{
//	delay_us(50);
	GpioDataRegs.GPACLEAR.bit.GPIO10 = 1;
//	delay_us(50);
}

void BATSPIEnable_high(void)
{
//	delay_us(50);
	GpioDataRegs.GPASET.bit.GPIO10 = 1;
}

void LTC6804_WakeUp(void)
{
	// need to check
	BATSPIEnable_low();
	SPI_Write(0);
	SPI_Write(0);
	SPI_Write(0);
	SPI_Write(0);
	SPI_Write(0);
	BATSPIEnable_high();
}

// ltc6804 write

int LTC6804_write(char address, short command, char data[], int len)
{
	char buffer[32];
	short pec;
	int i;

	if (len != 0 && len + 6 > 32) 
	{
		return 0;
		//return 1;
	}

	// make command & it's PEC
	buffer[0] = 0x80|((address<<3)&0x78)|((command>>8) & 0x07);
	buffer[1] = command & 0xff;
	pec = pec15(buffer, 2);
	buffer[2] = (pec>>8);
	buffer[3] = pec;
	// make data & it's PEC
	if (len != 0) 
	{
		for (i = 0; i < len; i++) 
		{
			buffer[4 + i] = data[i];
		}
		pec = pec15(data, len);
		buffer[4 + i] = (pec >> 8);
		buffer[4 + i + 1] = pec;

		len += 2;		// + 2 bytes pec	중요!!! don't remove!!!
	}

	// send all
	for(i=0;i<len+4;i++) 
	{
		SPI_Write(buffer[i]);
	}
	return (len+4); // LEEWOOWON Changes on '19.04.13
	//return len;
}

int LTC6804_write_cmd(char address, short command, char data[], int len)
{
	int ret;
	LTC6804_WakeUp();
	BATSPIEnable_low();
	ret = LTC6804_write(address, command, data, len);
	BATSPIEnable_high();
	return ret;
}

int LTC6804_write_cmd_with_dummy(char address, short command, int len)
{
	int ret;
	int i;

	LTC6804_WakeUp();
	BATSPIEnable_low();
	ret = LTC6804_write(address, command, 0, 0);

	for (i = 0; i < len; i++)
	{
		SPI_Write(0xff);
	}
	BATSPIEnable_high();
	return ret;

}
int LTC6804_read_cmd(char address, short command, char data[], int len)
{
	int i;
	int valid = 0;
	int ret;
	
	LTC6804_WakeUp();
	BATSPIEnable_low();
	ret = LTC6804_write(address, command, 0, 0);
	if (ret != 0 && len != 0)
	{
		unsigned short pecr;
		unsigned short pecg;
		for (i = 0; i < len; i++) 
		{
			data[i] = SPI_Read();
		}
		pecr = (SPI_Read() << 8) & 0xff00;
		pecr |= (SPI_Read() & 0x00ff);
		pecg = pec15(data, len);
		if (pecr == pecg) 
		{
			valid = 1;
			ltc_state = 1;
			ltc_error_count = 0;
		} else 
		{
			ltc_state = 0;
			ltc_error_count += 1;
		}
	}

	BATSPIEnable_high();

	return valid;
}


#define LTC6804_ICOM_WR_START					(6)
#define LTC6804_ICOM_WR_STOP					(1)
#define LTC6804_ICOM_WR_BLANK					(0)
#define LTC6804_ICOM_WR_NO_TRANSMIT				(7)

#define LTC6804_ICOM_RD_START					(6)
#define LTC6804_ICOM_RD_STOP					(1)
#define LTC6804_ICOM_RD_SDA_LOW					(0)
#define LTC6804_ICOM_RD_SDA_HIGH				(7)

#define LTC6804_FCOM_WR_MASTER_ACK				(0)
#define LTC6804_FCOM_WR_MASTER_NACK				(8)
#define LTC6804_FCOM_WR_MASTER_NACK_STOP		(9)

#define LTC6804_FCOM_RD_MASTER_ACK				(0)
#define LTC6804_FCOM_RD_SLAVE_ACK				(7)
#define LTC6804_FCOM_RD_SLAVE_NACK				(15)
#define LTC6804_FCOM_RD_SLAVE_ACK_MASTER_STOP	(1)
#define LTC6804_FCOM_RD_SLAVE_NACK_MASTER_STOP	(9)

#if 0
typedef struct 
{
	unsigned char D0H4:4;		// 3-0	COMM0
	unsigned char ICOM0:4;		// 7-4	COMM0
	unsigned char FCOM0:4;		// 3-0	COMM1
	unsigned char D0L4:4;		// 7-4	COMM1

	unsigned char D1H4:4;		// 3-0	COMM2
	unsigned char ICOM1:4;		// 7-4	COMM2
	unsigned char FCOM1:4;		// 3-0	COMM3
	unsigned char D1L4:4;		// 7-4	COMM3

	unsigned char D2H4:4;		// 3-0	COMM4
	unsigned char ICOM2:4;		// 7-4	COMM4
	unsigned char FCOM2:4;		// 3-0	COMM5
	unsigned char D2L4:4;		// 7-4	COMM5
} LTC6804_COMM_reg;
#else
typedef struct 
{
	unsigned int D0H4:4;		// 3-0	COMM0
	unsigned int ICOM0:4;		// 7-4	COMM0
	unsigned int FCOM0:4;		// 3-0	COMM1
	unsigned int D0L4:4;		// 7-4	COMM1

	unsigned int D1H4:4;		// 3-0	COMM2
	unsigned int ICOM1:4;		// 7-4	COMM2
	unsigned int FCOM1:4;		// 3-0	COMM3
	unsigned int D1L4:4;		// 7-4	COMM3

	unsigned int D2H4:4;		// 3-0	COMM4
	unsigned int ICOM2:4;		// 7-4	COMM4
	unsigned int FCOM2:4;		// 3-0	COMM5
	unsigned int D2L4:4;		// 7-4	COMM5
} LTC6804_COMM_reg;
#endif

#define MAX1238_SLAVE_ADDRESS	(0x6a)
struct MAX1238_SETUP_BIT
{
	unsigned char X:1;			// 0 	don't care
	unsigned char RST:1;		// 1	1=no action, 0=resets the config register to default
	unsigned char BIP_UNI:1;	// 2	1=bipolar, 0=unipolar
	unsigned char CLK:1;		// 3	1=external clock, 0=internal clock
	unsigned char SEL:3;		// 6-4	select the reference volatage and the state of AIN_/REF
	unsigned char REG:1;		// 7	1=setup byte, 0=config byte
} ;

typedef union 
{
	unsigned char all;
	struct MAX1238_SETUP_BIT bit;
} MAX1238_SETUP_REG;


struct MAX1238_CONFIG_BIT
{
	unsigned char SGL_DIF:1;	// 0 	1=single-ended, 0=differential
	unsigned char CS:4;			// 4-1	channel select
	unsigned char SCAN:2;		// 6-5	scan select
	unsigned char REG:1;		// 7	1=setup byte, 0=config byte
} ;

typedef union
{
	unsigned char all;
	struct MAX1238_CONFIG_BIT bit;
} MAX1238_CONFIG_REG;

// LTC4804 초기화 
//
// S->ID : SLAVE BMS 
// S->Command : Command 
// S->Command Table 
// S->Command Table LEN 
int SlaveBMSIint(SlaveReg *s)
{
	init_PEC15_Table();

//	LTC6804_write_cmd(s->ID, LTC6804_CMD_WRCFG, (char *)s->InitTable, sizeof(s->InitTable));
	LTC6804_write_cmd(s->ID, LTC6804_CMD_WRCFG, (char *)LTC6804_init_table, sizeof(LTC6804_init_table));
	return 0;
}
// 입력 
// S->ID : SLAVE BMS 
// S->Command : Command 
// S->Command Table, 명령어가 하나이면 인자 없음  배열 주소 기입  
// S->Command Table LEN 몀령어가 하나이면 "len =0", 배열이면 ,sizeof(배열명)
// S->Command 
int SlaveBmsWriteCmd(SlaveReg *s)
{
	int i;
	if ((s->len != 0) && (s->len + 6 > 32)) 
	{
		return 0;
	}

	// make command & it's PEC
	s->CommandBufbuffer[0] = 0x80|((s->ID<<3)&0x78)|((s->Command>>8) & 0x07);
	s->CommandBufbuffer[1]= s->Command & 0xff;
	s->Pec1= pec15(s->CommandBufbuffer, 2);
	s->CommandBufbuffer[2] = (s->Pec1>>8);
	s->CommandBufbuffer[3] = s->Pec1;
	// make data & it's PEC
	if (s->len != 0) 
	{
		for (i = 0; i <s->len; i++) 
		{
 //			s->CommandBufbuffer[4 + i] = *(s->PtCmd)*+i;
		} 
	  //	s->Pec1 = pec15(data, s->len);
		s->CommandBufbuffer[4 + i] = (s->Pec1 >> 8);
		s->CommandBufbuffer[4 + i + 1] = s->Pec1;

		s->len += 2;		// + 2 bytes pec	중요!!! don't remove!!!
	}

	// send all
	for(i=0;i<s->len+4;i++) 
	{
		SPI_Write(s->CommandBufbuffer[i]);
	}
	return (s->len+4);

}
int SlaveBmsBalance(SlaveReg *s)
{
	int ret;
	s->BalanceTable[0]=0xfc;
	s->BalanceTable[1]=0x00;
	s->BalanceTable[2]=0x00;
	s->BalanceTable[3]=0x00;
	s->BalanceTable[4]= (s->Balance.all & 0xff);
	s->BalanceTable[5]= (s->BalanceTable[4] & 0xf0) | ((s->Balance.all >> 8) & 0x0f);
	ret = LTC6804_write_cmd(s->ID,LTC6804_CMD_WRCFG,s->BalanceTable,sizeof(s->BalanceTable));
	return ret;
}

int LTC6804_Init()
{
	int i;
	// write PEC table
	init_PEC15_Table();

	// init variables
	LTC6804.count = 2;			// 전체 팩 갯수
	LTC6804.address[0] = 0xf;		// 0번 팩 주소 설정
	LTC6804.address[1] = 0;		// 1번 팩 주소 설정
	ltc_state = 0;
	ltc_error_count = 0;
	// write initial values
	for (i = 0; i < LTC6804.count; i++)
	{
		LTC6804_write_cmd(LTC6804.address[i], LTC6804_CMD_WRCFG, (char *)LTC6804_init_table, sizeof(LTC6804_init_table));
	}
	return 0;
}
// 셀 전압 읽기
//   pack_id: 팩 id (0~)
//   voltage: 전압 출력 (2바이트 x 12개)
int CellVoltageUpdata(SlaveReg *s)
{
	int ret;
	int i;
	char buf[6];
	int vcount = 0;
	s->CommandArrey[0]=LTC6804_CMD_RDCVA;
	s->CommandArrey[1]=LTC6804_CMD_RDCVB;
	s->CommandArrey[3]=LTC6804_CMD_RDCVC;
	s->CommandArrey[4]=LTC6804_CMD_RDCVD;

//	s->Command[4] = {LTC6804_CMD_RDCVA,LTC6804_CMD_RDCVB,LTC6804_CMD_RDCVC,LTC6804_CMD_RDCVD};
	// init values
	for (i = 0; i < 12; i++) 
	{
	//	s->CellVoltage[i]=0;
		//voltage[i] = 0;
	}
	// send ADC start to master
	ret = LTC6804_write_cmd(s->ID,LTC6804_CMD_ADCV | (3 << 7)|(0 << 4)|(0 << 0),0, 0);
			//LTC6804_CMD_ADCV | (2 << 7)|(0 << 4)|(0 << 0),	// 8~7:MD[1:0]=10, 4:DCP, 2~0:CH[2:0]=000 (all cells)
			//LTC6804_CMD_ADCV | (3 << 7)|(0 << 4)|(0 << 0),	// 8~7:MD[1:0]=10, 4:DCP, 2~0:CH[2:0]=000 (all cells)	
	if (ret == 0) 
	{
		return 0;
	}

	delay_us(202000);
	
	for (i = 0; i < 4; i++) 
	{
	   	ret = LTC6804_read_cmd(s->ID,s->CommandArrey[i], buf, 6);
		if (ret == 0) 
		{
			// skip it if invalid comm.
			continue;
		}
		vcount += 6;
		s->CellVoltage[i * 3 + 0] = ((buf[1] << 8) & 0xff00) | (buf[0] & 0x00ff);
		s->CellVoltage[i * 3 + 1] = ((buf[3] << 8) & 0xff00) | (buf[2] & 0x00ff);
		s->CellVoltage[i * 3 + 2] = ((buf[5] << 8) & 0xff00) | (buf[4] & 0x00ff);
	}

	//return (i * 6);
	return vcount;
}

int CellVoltageRead(int pack_id, unsigned short voltage[])
{
	int ret;
	char addr = LTC6804.address[pack_id];
	short cmd[4] = 
	{
		LTC6804_CMD_RDCVA,
		LTC6804_CMD_RDCVB,
		LTC6804_CMD_RDCVC,
		LTC6804_CMD_RDCVD
	};
	int i;
	char buf[6];
	int vcount = 0;

	// init values
	for (i = 0; i < 12; i++) 
	{
		voltage[i] = 0;
	}

	// send ADC start to master
	ret = LTC6804_write_cmd(addr, 
			//LTC6804_CMD_ADCV | (2 << 7)|(0 << 4)|(0 << 0),	// 8~7:MD[1:0]=10, 4:DCP, 2~0:CH[2:0]=000 (all cells)
			LTC6804_CMD_ADCV | (3 << 7)|(0 << 4)|(0 << 0),	// 8~7:MD[1:0]=10, 4:DCP, 2~0:CH[2:0]=000 (all cells)
			0, 0);
	if (ret == 0) {
		return 0;
	}

	// wait ADC, 2335us for all cells
	//delay_us(2335);
	//delay_us(5000);
	delay_us(202000);
	
	for (i = 0; i < 4; i++) 
	{
		ret = LTC6804_read_cmd(addr, cmd[i], buf, 6);
		if (ret == 0) {
			// skip it if invalid comm.
			continue;
		}
		vcount += 6;
		voltage[i * 3 + 0] = ((buf[1] << 8) & 0xff00) | (buf[0] & 0x00ff);
		voltage[i * 3 + 1] = ((buf[3] << 8) & 0xff00) | (buf[2] & 0x00ff);
		voltage[i * 3 + 2] = ((buf[5] << 8) & 0xff00) | (buf[4] & 0x00ff);
	}

	//return (i * 6);
	return vcount;
}

int MAX1238_start_conv(int pack_id, unsigned char channel)
{
	int ret;
	char addr = LTC6804.address[pack_id];
	MAX1238_SETUP_REG setup;
	MAX1238_CONFIG_REG config;
	unsigned char comm[6];
	unsigned char dat;

	dat = MAX1238_SLAVE_ADDRESS;
	comm[0] = ((LTC6804_ICOM_WR_START << 4) & 0xf0) | ((dat >> 4) & 0x0f);
	comm[1] = ((dat << 4) & 0xf0) | (LTC6804_FCOM_WR_MASTER_NACK & 0x0f);

	setup.bit.RST = 1;
	setup.bit.BIP_UNI = 0;		// unipolar
	//setup.bit.CLK = 0;			// internal
	setup.bit.CLK = 1;			// external
	setup.bit.SEL = 0;			// ref=VDD, analog input
	setup.bit.REG = 1;			// setup
	dat = setup.all;
	comm[2] = ((LTC6804_ICOM_WR_BLANK << 4) & 0xf0) | ((dat >> 4) & 0x0f);
	comm[3] = ((dat << 4) & 0xf0) | (LTC6804_FCOM_WR_MASTER_NACK & 0x0f);

	config.bit.SGL_DIF = 1;		// single-ended
	config.bit.CS = channel;	// 0-11
	config.bit.SCAN = 3;		// converts channel selected
	config.bit.REG = 0;			// config
	dat = config.all;
	comm[4] = ((LTC6804_ICOM_WR_BLANK << 4) & 0xf0) | ((dat >> 4) & 0x0f);
	comm[5] = ((dat << 4) & 0xf0) | (LTC6804_FCOM_WR_MASTER_NACK_STOP & 0x0f);

	ret = LTC6804_write_cmd(addr, LTC6804_CMD_WRCOMM, (char *)&comm[0], 6);
	ret = LTC6804_write_cmd_with_dummy(addr, LTC6804_CMD_STCOMM, 9);

	return ret;
}

int MAX1238_read_result(int pack_id, unsigned short *result)
{
	int ret;
	char addr = LTC6804.address[pack_id];
	unsigned char comm[6];
	unsigned char dat = 0xff;

	dat = (MAX1238_SLAVE_ADDRESS | 1);			// read
	comm[0] = ((LTC6804_ICOM_WR_START << 4) & 0xf0) | ((dat >> 4) & 0x0f);
	comm[1] = ((dat << 4) & 0xf0) | (LTC6804_FCOM_WR_MASTER_NACK & 0x0f);

	dat = 0xff;
	comm[2] = ((LTC6804_ICOM_WR_BLANK << 4) & 0xf0) | ((dat >> 4) & 0x0f);
	comm[3] = ((dat << 4) & 0xf0) | (LTC6804_FCOM_WR_MASTER_ACK & 0x0f);		// ack

	dat = 0xff;
	comm[4] = ((LTC6804_ICOM_WR_BLANK << 4) & 0xf0) | ((dat >> 4) & 0x0f);
	comm[5] = ((dat << 4) & 0xf0) | (LTC6804_FCOM_WR_MASTER_NACK_STOP & 0x0f);

	ret = LTC6804_write_cmd(addr, LTC6804_CMD_WRCOMM, (char *)&comm[0], 6);
	ret = LTC6804_write_cmd_with_dummy(addr, LTC6804_CMD_STCOMM, 9);
	ret = LTC6804_read_cmd(addr, LTC6804_CMD_RDCOMM, (char *)&comm[0], 6);
	if (ret == 0) {
		*result = 0;
		return ret;
	}

	*result = ((((unsigned short)comm[3] <<  4) & 0x0f00) |
			   (((unsigned short)comm[4] <<  4) & 0x00f0) |
			   (((unsigned short)comm[5] >>  4) & 0x000f));

	return ret;
}

/*
int CellBalanceSet(int pack_id, CellBalance_t cell)
{
	int ret;
	char addr = LTC6804.address[pack_id];
	char *tab = (char *)LTC6804_init_table;
	tab[4] = (cell.all & 0xff);
	tab[5] = (tab[5] & 0xf0) | ((cell.all >> 8) & 0x0f);
	ret = LTC6804_write_cmd(addr, LTC6804_CMD_WRCFG, tab, sizeof(LTC6804_init_table));
	return ret;
}
*/
//int LTC6804_DieTemperatureRead(int pack_id, short *temperature)
int LTC6804_DieTemperatureRead(int pack_id, float *temperature)
{
	int ret;
	char addr = LTC6804.address[pack_id];
	char buf[6];
	unsigned short soc;
	unsigned short itmp;
	unsigned short va;

	ret = LTC6804_write_cmd(addr, 
			LTC6804_CMD_ADSTAT | (2 << 7) | (0 << 0), 
			0, 0);
	if (ret == 0) {
		return 0;
	}

	delay_us(1563);	

	ret = LTC6804_read_cmd(addr, LTC6804_CMD_RDSTATA, buf, 6);
	if (ret == 0) {
		*temperature = 0;
		return ret;
	}

	soc = ((buf[1] << 8) & 0xff00) | (buf[0] & 0x00ff);
	itmp = ((buf[3] << 8) & 0xff00) | (buf[2] & 0x00ff);
	va = ((buf[5] << 8) & 0xff00) | (buf[4] & 0x00ff);
	soc = soc;
	va = va;
	//*temperature = itmp;

	{
		// [die temp(C)] = (ITMP) * 100uV / (7.5mV) - 273
		//float tmp = (float)itmp * 100 / 7500 - 273.0;
		*temperature = (float)itmp / 75.0 - 273.0;
	}

	return ret;
}

int LTC6804_SocItmpVaRead(int pack_id, unsigned short *osoc, float *oitmp, unsigned short *ova)
{
	int ret;
	char addr = LTC6804.address[pack_id];
	char buf[6];
	unsigned short soc;
	unsigned short itmp;
	unsigned short va;

	ret = LTC6804_write_cmd(addr, 
			LTC6804_CMD_ADSTAT | (2 << 7) | (0 << 0), 
			0, 0);
	if (ret == 0) {
		return 0;
	}

	delay_us(1563);	

	ret = LTC6804_read_cmd(addr, LTC6804_CMD_RDSTATA, buf, 6);
	if (ret == 0) {
		*osoc = 0;
		*oitmp = 0.0;
		*ova = 0;
		return ret;
	}

	soc = ((buf[1] << 8) & 0xff00) | (buf[0] & 0x00ff);
	itmp = ((buf[3] << 8) & 0xff00) | (buf[2] & 0x00ff);
	va = ((buf[5] << 8) & 0xff00) | (buf[4] & 0x00ff);
	*osoc = soc;
	{
		// [die temp(C)] = (ITMP) * 100uV / (7.5mV) - 273
		//float tmp = (float)itmp * 100 / 7500 - 273.0;
		*oitmp = (float)itmp / 75.0 - 273.0;
	}
	*ova = va;

	return ret;
}

#define EEPROM_23LCV1024

#define _EEPROM_READ		(0x03)
#define _EEPROM_WRITE		(0x02)
#define _EEPROM_WREN		(0x06)
#define _EEPROM_WRDI		(0x04)
#define _EEPROM_RDSR		(0x05)


