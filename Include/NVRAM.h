/*!
LTC6804-2 Multicell Battery Monitor

http://www.linear.com/product/LTC6804-1

http://www.linear.com/product/LTC6804-1#demoboards

REVISION HISTORY
$Revision: 4432 $
$Date: 2015-11-30 14:03:02 -0800 (Mon, 30 Nov 2015) $

Copyright (c) 2013, Linear Technology Corp.(LTC)
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies,
either expressed or implied, of Linear Technology Corp.

The Linear Technology Linduino is not affiliated with the official Arduino team.
However, the Linduino is only possible because of the Arduino team's commitment
to the open-source community.  Please, visit http://www.arduino.cc and
http://store.arduino.cc , and consider a purchase that will help fund their
ongoing work.

Copyright 2013 Linear Technology Corp. (LTC)
***********************************************************/
#include "DSP28x_Project.h"

int NVRAM_Read_Status_Register();
void NVRAM_WriteEnable(int enable);
unsigned int NVRAM_Read_Words(unsigned long addr);
int NVRAM_Read_Bytes(unsigned long addr);
void NVRAM_Write_Bytes(unsigned long addr,unsigned int dat);
void NVRAM_Write_Words(unsigned long addr, unsigned int dat);
unsigned long NVRAM_Tx(unsigned long addr,unsigned int dat1,unsigned int dat2,unsigned int dat3,unsigned int dat4);

void NVRAM_Write_Enable();
void NVRAM_Write_Disable();
void NVRAM_Sleep();
void NVRAM_Wakeup();

void SPI_Write(unsigned int WRData);
unsigned int SPI_Read(void);
//***********************************************************************************
//                     GLOBAL CONSTANTS NVRAM COMMANDS
//...................................................................................
#define NVRAM_MR25H40CDC

#define _NVRAM_WREN         0x06
#define _NVRAM_WRDI         0x04
#define _NVRAM_RDSR         0x05
#define _NVRAM_WRSR         0x01
#define _NVRAM_READ         0x03
#define _NVRAM_WRITE        0x02
#define _NVRAM_SLEEP        0xB9
#define _NVRAM_WAKE         0xAB

//===========================================================================
// End of file.
//===========================================================================
