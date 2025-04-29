// TI File $Revision: /main/3 $
// Checkin $Date: November 6, 2009   16:15:43 $
//###########################################################################
//
// FILE:    DSP2803x_ECan.c
//
// TITLE:   DSP2803x Enhanced CAN Initialization & Support Functions.
//
//###########################################################################
// $TI Release: 2803x C/C++ Header Files V1.21 $
// $Release Date: December 1, 2009 $
//###########################################################################

#include "DSP2803x_Device.h"     // DSP28 Headerfile Include File
#include "DSP2803x_Examples.h"   // DSP28 Examples Include File
#include "DSP28x_Project.h" 

//---------------------------------------------------------------------------
// InitECan:
//---------------------------------------------------------------------------
// This function initializes the eCAN module to a known state.
//
void InitECan(void)
{
   InitECana();
}

void InitECana(void)        // Initialize eCAN-A module
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
    ECanaShadow.CANMC.bit.SCB = 1;  // ByCHOO eCAN Mode. 모든 Mail Box를 사용할 수 있다.
	ECanaShadow.CANMC.bit.ABO = 1;  ///
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

    ECanaRegs.CANTA.all = 0xFFFFFFFF;   /* Clear all TAn bits */
    ECanaRegs.CANRMP.all = 0xFFFFFFFF;  /* Clear all RMPn bits */
    ECanaRegs.CANGIF0.all = 0xFFFFFFFF; /* Clear all interrupt flag bits */
    ECanaRegs.CANGIF1.all = 0xFFFFFFFF;

//ID 설정


	ECanaMboxes.MBOX0.MSGID.all		= 0;
	ECanaMboxes.MBOX1.MSGID.all		= 0;
	ECanaMboxes.MBOX2.MSGID.all		= 0;
	ECanaMboxes.MBOX3.MSGID.all		= 0;
	ECanaMboxes.MBOX4.MSGID.all		= 0;
	ECanaMboxes.MBOX5.MSGID.all		= 0;	
	ECanaMboxes.MBOX6.MSGID.all		= 0;
	ECanaMboxes.MBOX7.MSGID.all		= 0;
	ECanaMboxes.MBOX8.MSGID.all		= 0;
	ECanaMboxes.MBOX9.MSGID.all		= 0;
	ECanaMboxes.MBOX10.MSGID.all	= 0;	
	ECanaMboxes.MBOX11.MSGID.all	= 0;
	ECanaMboxes.MBOX12.MSGID.all	= 0;
	ECanaMboxes.MBOX23.MSGID.all	= 0;
	ECanaMboxes.MBOX14.MSGID.all	= 0;
	ECanaMboxes.MBOX15.MSGID.all	= 0;	
	ECanaMboxes.MBOX16.MSGID.all	= 0;
	ECanaMboxes.MBOX17.MSGID.all	= 0;
	ECanaMboxes.MBOX18.MSGID.all	= 0;
	ECanaMboxes.MBOX19.MSGID.all	= 0;
	ECanaMboxes.MBOX20.MSGID.all	= 0;
	ECanaMboxes.MBOX21.MSGID.all	= 0;
	ECanaMboxes.MBOX22.MSGID.all	= 0;
	ECanaMboxes.MBOX23.MSGID.all	= 0;
	ECanaMboxes.MBOX24.MSGID.all	= 0;
	ECanaMboxes.MBOX25.MSGID.all	= 0;	
	ECanaMboxes.MBOX26.MSGID.all	= 0;
	ECanaMboxes.MBOX27.MSGID.all	= 0;
	ECanaMboxes.MBOX28.MSGID.all	= 0;
	ECanaMboxes.MBOX29.MSGID.all	= 0;
	ECanaMboxes.MBOX30.MSGID.all	= 0;	
	ECanaMboxes.MBOX31.MSGID.all	= 0;

	
	
	ECanaMboxes.MBOX0.MSGID.bit.STDMSGID  = CAN_EMS_GRID_0;  
	ECanaMboxes.MBOX1.MSGID.bit.STDMSGID  = CAN_EMS_GRID_1;
	ECanaMboxes.MBOX2.MSGID.bit.STDMSGID  = CAN_PC_GRID;
	ECanaMboxes.MBOX3.MSGID.bit.STDMSGID  = CAN_BMS_GRID;
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
	ECanaShadow.CANGAM.bit.AMI = 1;
	//ECanaShadow.CANGAM.all = 0xFFFFFFFF; //byCHOO
	ECanaRegs.CANGAM.all   = ECanaShadow.CANGAM.all;
	
	
	//EMS용 RX MBOX00 Setting 0x000~0x00F
	ECanaMboxes.MBOX0.MSGID.bit.AME = 1;				//Acceptance Mask 사용. 그리고 나서 LAM0 에서 모든 bit Don't care
	ECanaLAMRegs.LAM0.bit.LAMI = 1;						//Standard Identifier 가능
	ECanaLAMRegs.LAM0.bit.LAM_H = _LAM(CAN_BIT_MASK);	//1 : Don't Care
	


	//EMS용 RX MBOX01 Setting 0x010 ~0x01F
	ECanaMboxes.MBOX1.MSGID.bit.AME = 1;				//Acceptance Mask 사용. 그리고 나서 LAM0 에서 모든 bit Don't care
	ECanaLAMRegs.LAM1.bit.LAMI = 1;						//Standard Identifier 가능
	ECanaLAMRegs.LAM1.bit.LAM_H = _LAM(CAN_BIT_MASK);	//1 : Don't Care

	
	//PCS용 RX MBOX01 Setting 0x600~0x60F
	ECanaMboxes.MBOX2.MSGID.bit.AME = 1;				//Acceptance Mask 사용. 그리고 나서 LAM0 에서 모든 bit Don't care
	ECanaLAMRegs.LAM2.bit.LAMI = 1;						//Standard Identifier 가능
	ECanaLAMRegs.LAM2.bit.LAM_H = _LAM(CAN_BIT_MASK);	//1 : Don't Care

	
	//BMS용 RX MBOX03 Setting 0x0030~0x003F
	ECanaMboxes.MBOX3.MSGID.bit.AME = 1;				//Acceptance Mask 사용. 그리고 나서 LAM0 에서 모든 bit Don't care
	ECanaLAMRegs.LAM3.bit.LAMI = 1;						//Standard Identifier 가능
	ECanaLAMRegs.LAM3.bit.LAM_H = _LAM(CAN_BIT_MASK);	//1 : Don't Care

	
   	/* The following block is only for 60 MHz SYSCLKOUT. (30 MHz CAN module clock Bit rate = 1 Mbps
       See Note at end of file. */
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


	ECanaRegs.CANBTC.all = ECanaShadow.CANBTC.all;
    ECanaShadow.CANMC.all = ECanaRegs.CANMC.all;
    ECanaShadow.CANMC.bit.CCR = 0 ;            // Set CCR = 0
    ECanaRegs.CANMC.all = ECanaShadow.CANMC.all;
	ECanaShadow.CANES.all = ECanaRegs.CANES.all;///
    // Wait until the CPU no longer has permission to change the configuration registers
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
	ECanaShadow.CANMIL.all = ECanaRegs.CANMIL.all;					// 1 이면 해당 MailBox 인트럽트 Generateon 1 선언함 ; 0 이면 해당 MailBox 인트럽트 Generateon 0 선언함
 	ECanaShadow.CANMIL.all = 0x00000000;  //ByCHOO 0이면 interrupt 0에 연결. 나중에 Main에서  PieVectTable.ECAN0INTA 	= &ISR_CANRXINTA; 이런 식으로 연계
	ECanaRegs.CANMIL.all  = ECanaShadow.CANMIL.all;

	//해당 MailBox를 RX, TX 선정

	ECanaShadow.CANMD.all = ECanaRegs.CANMD.all;					// 해당 MailBox을 1:RX, 0:TX 선정함. 우선은 모두 RX 로 설정. 32개 메일박스 RX인터럽트 확인한다.
	
	ECanaShadow.CANMD.bit.MD0=1;   //RX:EMS_1			 					
	ECanaShadow.CANMD.bit.MD1=1;   //RX:EMS_2			 				
	ECanaShadow.CANMD.bit.MD2=1;   //RX:PCS			 					
	ECanaShadow.CANMD.bit.MD3=1;   //RX:BMS			 					
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
	ECanaShadow.CANME.bit.ME3= 1;									
	ECanaShadow.CANME.bit.ME4= 1;									
	ECanaShadow.CANME.bit.ME5= 1;									
	ECanaShadow.CANME.bit.ME6= 1;									
	ECanaShadow.CANME.bit.ME7= 1;									
	ECanaShadow.CANME.bit.ME8= 1;									
	ECanaShadow.CANME.bit.ME9= 1;									
	ECanaShadow.CANME.bit.ME10= 1;
	ECanaShadow.CANME.bit.ME11= 1;									 
	ECanaShadow.CANME.bit.ME12= 1;									 
	ECanaShadow.CANME.bit.ME13= 1;									
	ECanaShadow.CANME.bit.ME14= 1;									
	ECanaShadow.CANME.bit.ME15= 1;									
	ECanaShadow.CANME.bit.ME16= 1;									
	ECanaShadow.CANME.bit.ME17= 1;									
	ECanaShadow.CANME.bit.ME18= 1;									
	ECanaShadow.CANME.bit.ME19= 1;									
	ECanaShadow.CANME.bit.ME20= 1;	
	ECanaShadow.CANME.bit.ME21= 1;	
	ECanaShadow.CANME.bit.ME22= 1;									 
	ECanaShadow.CANME.bit.ME23= 1;									
	ECanaShadow.CANME.bit.ME24= 1;									
	ECanaShadow.CANME.bit.ME25= 1;									
	ECanaShadow.CANME.bit.ME26= 1;									
	ECanaShadow.CANME.bit.ME27= 1;									
	ECanaShadow.CANME.bit.ME28= 1;									
	ECanaShadow.CANME.bit.ME29= 1;									
	ECanaShadow.CANME.bit.ME30= 1;									
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
	ECanaShadow.CANMIM.bit.MIM3 = 1;								
	ECanaShadow.CANMIM.bit.MIM4=  1;								
	ECanaShadow.CANMIM.bit.MIM5 = 1;								
	ECanaShadow.CANMIM.bit.MIM6 = 1;								   
	ECanaShadow.CANMIM.bit.MIM7 = 1;								   
	ECanaShadow.CANMIM.bit.MIM8 = 1;								
	ECanaShadow.CANMIM.bit.MIM9 = 1;								
	ECanaShadow.CANMIM.bit.MIM10 = 1;						
	
	ECanaShadow.CANMIM.bit.MIM11 = 1;								
	ECanaShadow.CANMIM.bit.MIM12 = 1;								
	ECanaShadow.CANMIM.bit.MIM13 = 1;								
	ECanaShadow.CANMIM.bit.MIM14=  1;								
	ECanaShadow.CANMIM.bit.MIM15 = 1;								
	ECanaShadow.CANMIM.bit.MIM16 = 1;								   
	ECanaShadow.CANMIM.bit.MIM17 = 1;								   
	ECanaShadow.CANMIM.bit.MIM18 = 1;								
	ECanaShadow.CANMIM.bit.MIM19 = 1;								
	ECanaShadow.CANMIM.bit.MIM20 = 1;								
								
	ECanaShadow.CANMIM.bit.MIM21 = 1;	
	ECanaShadow.CANMIM.bit.MIM22 = 1;								
	ECanaShadow.CANMIM.bit.MIM23 = 1;								
	ECanaShadow.CANMIM.bit.MIM24=  1;								
	ECanaShadow.CANMIM.bit.MIM25 = 1;								
	ECanaShadow.CANMIM.bit.MIM26 = 1;								   
	ECanaShadow.CANMIM.bit.MIM27 = 1;								   
	ECanaShadow.CANMIM.bit.MIM28 = 1;								
	ECanaShadow.CANMIM.bit.MIM29 = 1;								
	ECanaShadow.CANMIM.bit.MIM30 = 1;								
	
	//ECanaShadow.CANMIM.bit.MIM31 = 1;								
	
	ECanaRegs.CANMIM.all   = ECanaShadow.CANMIM.all;

	/* Write to DLC field in Master Control reg  Data Size를 8byte로 선언*/
	
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
	ECanaMboxes.MBOX31.MSGCTRL.bit.DLC = 8;	

	

	ECanaMboxes.MBOX0.MSGID.bit.IDE = 0;
	ECanaMboxes.MBOX1.MSGID.bit.IDE = 0;
	ECanaMboxes.MBOX2.MSGID.bit.IDE = 0;
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
	
	ECanaMboxes.MBOX31.MSGID.bit.IDE = 0;

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

	ECanaRegs.CANMC.bit.DBO = 0;			

	
	//ECanaLAMRegs.LAM0.bit.LAMI = 1; //locan acceptance mask are used 
	//ECanaLAMRegs.LAM0.bit.LAM_H = 0x7FF; //1 : Don't Care

	
	

	
	
    EDIS;
}

//---------------------------------------------------------------------------
// Example: InitECanGpio:
//---------------------------------------------------------------------------
// This function initializes GPIO pins to function as eCAN pins
//
// Each GPIO pin can be configured as a GPIO pin or up to 3 different
// peripheral functional pins. By default all pins come up as GPIO
// inputs after reset.
//
// Caution:
// Only one GPIO pin should be enabled for CANTXA operation.
// Only one GPIO pin shoudl be enabled for CANRXA operation.
// Comment out other unwanted lines.

void InitECanGpio(void)
{
   InitECanaGpio();
}

void InitECanaGpio(void)
{
   EALLOW;

/* Enable internal pull-up for the selected CAN pins */
// Pull-ups can be enabled or disabled by the user.
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAPUD.bit.GPIO30 = 0;     // Enable pull-up for GPIO30 (CANRXA)
    GpioCtrlRegs.GPAPUD.bit.GPIO31 = 0;     // Enable pull-up for GPIO31 (CANTXA)

/* Set qualification for selected CAN pins to asynch only */
// Inputs are synchronized to SYSCLKOUT by default.
// This will select asynch (no qualification) for the selected pins.

    GpioCtrlRegs.GPAQSEL2.bit.GPIO30 = 3;   // Asynch qual for GPIO30 (CANRXA)

/* Configure eCAN-A pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be eCAN functional pins.

    GpioCtrlRegs.GPAMUX2.bit.GPIO30 = 1;    // Configure GPIO30 for CANRXA operation
    GpioCtrlRegs.GPAMUX2.bit.GPIO31 = 1;    // Configure GPIO31 for CANTXA operation

    EDIS;
}

/* Note: Bit timing parameters must be chosen based on the network parameters such as
   the sampling point desired and the propagation delay of the network. The propagation
   delay is a function of length of the cable, delay introduced by the
   transceivers and opto/galvanic-isolators (if any).

   The parameters used in this file must be changed taking into account the above mentioned
   factors in order to arrive at the bit-timing parameters suitable for a network.
*/

//===========================================================================
// End of file.
//===========================================================================



