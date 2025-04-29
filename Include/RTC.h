/*!
Copyright 2021 HIS Corp. (RTC)
***********************************************************/
#include "DSP28x_Project.h"

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//                     GLOBAL CONSTANTS SPI COMMANDS
//...................................................................................
#define  SPI_RTCC_READ      0x13       // SPI READ  RTCC COMMAND
#define  SPI_RTCC_WRITE     0x12       // SPI WRITE RTCC COMMAND
#define  SPI_RTCC_CLRWDT    0x44       // SPI CLEAR WDT  COMMAND
#define  SPI_RTCC_CLRRAM    0x54       // SPI CLEAR RAM  COMMAND

#define  SPI_RTCC_EEREAD    0x03       // SPI EEPROM Read          COMMAND
#define  SPI_RTCC_EEWRITE   0x02       // SPI EEPROM Write         COMMAND
#define  SPI_RTCC_EEWRDI    0x04       // SPI EEPROM Write Disable COMMAND
#define  SPI_RTCC_EEWREN    0x06       // SPI EEPROM Write Enable  COMMAND

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//                     SPI RTCC REGISTERS ADDRESSES
// ..................................................................................
 #define  RTCHSEC           0x00       //  address of hundredth of
                                       //             SECONDS      register
 #define  RTCSEC            0x01       //  address of SECONDS      register
 #define  RTCMIN            0x02       //  address of MINUTES      register
 #define  RTCHOUR           0x03       //  address of HOURS        register
 #define  RTCWKDAY          0x04       //  address of DAY OF WK    register
 #define  RTCDATE           0x05       //  address of DATE         register
 #define  RTCMTH            0x06       //  address of MONTH        register
 #define  RTCYEAR           0x07       //  address of YEAR         register
 #define  CONTROL           0x08       //  address of CONTROL      register
 #define  OSCTRIM           0x09       //  address of OSC TRIMMING register
 #define  WDTCON            0x0A       //  address of WATCH DOG    register
 #define  EVDTCON           0x0B       //  address of EVENT DETECT register
//...................................................................................
 #define  ALM0SEC           0x0C       //  address of ALARMO SEC   register
 #define  ALM0MIN           0x0D       //  address of ALARMO MIN   register
 #define  ALM0HOUR          0x0E       //  address of ALARMO HOUR  register
 #define  ALM0WKDAY         0x0F       //  address of ALARM0 CONTR register
 #define  ALM0DATE          0x10       //  address of ALARMO DATE  register
 #define  ALM0MTH           0x11       //  address of ALARMO MONTH register
//...................................................................................
 #define  ALM1HSEC          0x12       //  address of hundredth of
                                       //             ALARM1 SEC   register
 #define  ALM1SEC           0x13       //  address of ALARM1 SEC   register
 #define  ADDR_ALM1MIN      0x14       //  address of ALARM1 MIN   register
 #define  ALM1HOUR          0x15       //  address of ALARM1 HOUR  register
 #define  ALM1WKDAY         0x16       //  address of ALARM1 CONTR register
 #define  ALM1DATE          0x17       //  address of ALARM1 DATE  register
//...................................................................................
 #define  PWRDNMIN          0x18       //  address of T_SAVER MIN(VDD->BAT)
 #define  PWRDNHOUR         0x19       //  address of T_SAVER HR (VDD->BAT)
 #define  PWRDNDATE         0x1A       //  address of T_SAVER DAT(VDD->BAT)
 #define  PWRDNMTH          0x1B       //  address of T_SAVER MTH(VDD->BAT)
//...................................................................................
 #define  PWRUPMIN          0x1C       //  address of T_SAVER MIN(BAT->VDD)
 #define  PWRUPHOUR         0x1D       //  address of T_SAVER HR (BAT->VDD)
 #define  PWRUPDATE         0x1E       //  address of T_SAVER DAT(BAT->VDD)
 #define  PWRUPMTH          0x1F       //  address of T_SAVER MTH(BAT->VDD)

//***********************************************************************************
//                     GLOBAL CONSTANTS RTCC : MASKS / BITS
//...................................................................................
 #define  ST           0x80            //  START OSC               (RTCSEC REG)
 #define  LPYR         0x20            //  MASK FOR LEAP YEAR      (RTCMTH REG)
 #define  PM           0x20            //  post-meridian bit (RTCHOUR REG)
 #define  H12          0x40            //  12 HOURS FORMAT   (RTCHOUR REG)
 #define  TRIMSIGN     0x80            //  minus sign for the calibration reg
 #define  OUT          0x80            //  = SET CLKO PIN          (CONTROL REG)
 #define  SQWEN        0x40            //  enable SQWAVE ON CLKOUT (CONTROL REG)
 #define  ALM_NO       0x00            //  no alarm activated      (CONTROL REG)
 #define  ALM0EN       0x10            //  ALARM0 enabled          (CONTROL REG)
 #define  ALM1EN       0x20            //  ALARM1 enabled          (CONTROL REG)
 #define  ALM_01       0x30            //  both alarms are enabled (CONTROL REG)
 #define  CLKO_01H     0x00            //  CLKO = SQWAV(01 HERTZ)  (CONTROL REG)
 #define  CLKO_04K     0x01            //  CLKO = SQWAV(04 KHZ)    (CONTROL REG)
 #define  CLKO_08K     0x02            //  CLKO = SQWAV(08 KHZ)    (CONTROL REG)
 #define  CLKO_32K     0x03            //  CLKO = SQWAV(32 KHZ)    (CONTROL REG)
 #define  CLKO_64H     0x04            //  CLKO = SQVAW(64 HERTZ)  (CONTROL REG)
 #define  ALMxPIN      0x80            //  CLKO polarity on alarm  (ALMxWKDAY)
 #define  ALMx_SEC     0x00            //  ALARM compare on SEC    (ALMxWKDAY)
 #define  ALMx_MIN     0x10            //  ALARM compare on MIN    (ALMxWKDAY)
 #define  ALMx_HR      0x20            //  ALARM compare on HOUR   (ALMxWKDAY)
 #define  ALMx_DAY     0x30            //  ALARM compare on DAY    (ALMxWKDAY)
 #define  ALMx_DAT     0x40            //  ALARM compare on DATE   (ALMxWKDAY)
 #define  ALMx_ALL     0x70            //  ALARM compare on all    (ALMxWKDAY)
 #define  ALMx_IF      0x08            //  MASK of the ALARM_IF    (ALMxWKDAY)
 #define  OSCRUN       0x20            //  state of the oscillator (running/not)
                                       //                          (RTCWKDAY REG)
 #define  VBATEN       0x08            //  enable battery BACKUP   (RTCWKDAY REG)

void Spi_Rtcc_Clear(void);
void Spi_Rtcc_Write(unsigned char rtcc_reg, unsigned char time_var);
unsigned char Spi_Rtcc_Read(unsigned char rtcc_reg); // SPI read from the SPI RTCC
void Init_Rtcc(void);
void Init_time(SystemReg *s);
void RTC_Read_Time(SystemReg *s);

void Spi_Rtcc_EEWRITE(unsigned int addr,unsigned int dat1,unsigned int dat2,unsigned int dat3,unsigned int dat4);

void SPI_Write(unsigned int WRData);
unsigned int SPI_Read(void);
//===========================================================================
// End of file.
//===========================================================================
