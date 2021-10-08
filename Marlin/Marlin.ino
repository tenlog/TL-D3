/* -*- c++ -*- */

/*
 Reprap firmware based on Sprinter and grbl.
 Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 Copyright (C) 2016-2019 zyf@tenlog3d.com
 
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 This firmware is a mashup between Sprinter and grbl.
  (https://github.com/kliment/Sprinter)
  (https://github.com/simen/grbl/tree)
 
 It has preliminary support for Matthew Roberts advance algorithm 
    http://reprap.org/pipermail/reprap-dev/2011-May/003323.html
 */

/* All the implementation is done in *.cpp files to get better compatibility with avr-gcc without the Arduino IDE */
/* Use this file to help the Arduino IDE find which Arduino libraries are needed and to keep documentation on GCode */

//

#include "Configuration.h"
#include "pins.h"
#include "marlin.h"
#include "tl_touch_screen.h"

#define TLSERIAL Serial2



FORCE_INLINE void TLPRINTPGM(const char * str)
{
  char ch = pgm_read_byte(str);
  while (ch)
  {
    TLSERIAL.write(ch);
    ch = pgm_read_byte(++str);
  }
}

const char cEnd PROGMEM = 0xFF;

void TenlogScreen_printconstln(const String s)
{
    TLSERIAL.print(s);
    TenlogScreen_printend();
}
void TenlogScreen_printconst(const String s)
{
    TLSERIAL.print(s);
}
void TenlogScreen_println(const char s[])
{
    TLSERIAL.print(s);
    TenlogScreen_printend();
}

void TenlogScreen_print(const char s[])
{
    TLSERIAL.print(s);
}

void TenlogScreen_printend()
{
    /*
    TLSERIAL.write(chrEnd);
    TLSERIAL.write(chrEnd);
    TLSERIAL.write(chrEnd);
    */

    char ch = pgm_read_byte(&cEnd);
    TLSERIAL.write(ch);
    TLSERIAL.write(ch);
    TLSERIAL.write(ch);
}

void TenlogScreen_printEmptyend()
{
    TLSERIAL.write(0x00);
    TenlogScreen_printend();
}


bool MTLSERIAL_available()
{
    return TLSERIAL.available();
}

int MTLSERIAL_read()
{
    return TLSERIAL.read();
}

#define DWN_HEAD0 0x5A
#define DWN_HEAD1 0xA5
#define DWN_WRITE 0x82
#define DWN_READ 0x83
#define DWN_ZERO 0x00

#define CP_ADD     \
    {              \
        0x00, 0x84 \
    }
#define CP_CMD     \
    {              \
        0x5A, 0x01 \
    }

void TenlogScreen_begin(const long boud)
{
    TLSERIAL.begin(boud);
}

void PGMWRITE(const long str)
{
    char ch = pgm_read_byte(&str);
    TLSERIAL.write(ch);
}

void DWN_Page(const int ID)
{
    byte add[] = CP_ADD;
    byte cmd[] = CP_CMD;

    TLSERIAL.write(DWN_HEAD0);   
    TLSERIAL.write(DWN_HEAD1);
    
    TLSERIAL.write(3 + sizeof(add) + sizeof(cmd));
    
    TLSERIAL.write(DWN_WRITE);
    
    TLSERIAL.write(add, sizeof(add));
    TLSERIAL.write(cmd, sizeof(cmd));
    TLSERIAL.write(DWN_ZERO);
    
    TLSERIAL.write(ID);
    iDWNPageID = ID;
}

void DWN_Text(const long ID, int Len, String s, bool Center = false)
{
    TLSERIAL.write(DWN_HEAD0);
    TLSERIAL.write(DWN_HEAD1);
    TLSERIAL.write(3 + Len);
    TLSERIAL.write(0x82);
    int ID0 = ID / 0x100;
    int ID1 = ID % 0x100;

    TLSERIAL.write(ID0);
    TLSERIAL.write(ID1);

    if (s.length() > Len - 2)
    {
        s = s.substring(0, Len - 2);
        TLSERIAL.print(s);
        TLSERIAL.write(0xFF);
        TLSERIAL.write(0xFF);
    }
    else
    {
        if (!Center)
        {
            TLSERIAL.print(s);
            for (int i = 0; i < Len - s.length() - 2; i++)
            {
                TLSERIAL.print(F(" "));
            }
            TLSERIAL.write(0xFF);
            TLSERIAL.write(0xFF);
        }
        else
        {
            //String s1 = "";
            int Count = 0;
            for (int i = 0; i < (Len - s.length() - 2) / 2; i++)
            {
                TLSERIAL.print(F(" "));
                //s1 += " ";
                Count++;
            }
            //TLSERIAL.print(s1);
            TLSERIAL.print(s);
            Count += s.length();
            //s1 = "";
            for (int i = 0; i < Len - Count - 2; i++)
            {
                TLSERIAL.print(F(" "));
                //s1 += " ";
            }
            //TLSERIAL.print(s1);
            TLSERIAL.write(0xFF);
            TLSERIAL.write(0xFF);
        }
    }
}

void DWN_Language(const int ID)
{
    DWN_Change_Icon(0x90, 0x40, ID);
    DWN_Change_Icon(0x80, 0x10, !ID);
    DWN_Change_Icon(0x80, 0x11, ID);
}

void DWN_Change_Icon(const int IID0, const int IID1, const int ID)
{
    //5A A5 05 82 50 31 00 01
    TLSERIAL.write(DWN_HEAD0);
    TLSERIAL.write(DWN_HEAD1);
    TLSERIAL.write(0x05);
    TLSERIAL.write(0x82);
    TLSERIAL.write(IID0);
    TLSERIAL.write(IID1);
    TLSERIAL.write(0x00);
    TLSERIAL.write(ID);
}

void DWN_NORFData(const long NorID, const long ID, int Length, const bool WR)
{
    int iWR = 0x5A;
    if (WR)
        iWR = 0xA5;
    TLSERIAL.write(DWN_HEAD0);
    TLSERIAL.write(DWN_HEAD1);
    TLSERIAL.write(0x0B);
    TLSERIAL.write(0x82);
    TLSERIAL.write(0x00);
    TLSERIAL.write(0x08);
    TLSERIAL.write(iWR);
    int Nor0 = NorID / 0x10000;
    long lNorID = NorID % 0x10000;
    int Nor1 = lNorID / 0x100;
    int Nor2 = lNorID % 0x100;
    TLSERIAL.write(Nor0);
    TLSERIAL.write(Nor1);
    TLSERIAL.write(Nor2);
    int ID0 = ID / 0x100;
    int ID1 = ID % 0x100;
    TLSERIAL.write(ID0);
    TLSERIAL.write(ID1);
    TLSERIAL.write(0x00);
    TLSERIAL.write(Length);
}

void DWN_VClick(const int X, const int Y)
{
    TLSERIAL.write(DWN_HEAD0);
    TLSERIAL.write(DWN_HEAD1);
    TLSERIAL.write(0x0B);
    TLSERIAL.write(0x82);
    TLSERIAL.write(0x00);
    TLSERIAL.write(0xD4);
    TLSERIAL.write(DWN_HEAD0);
    TLSERIAL.write(DWN_HEAD1);
    TLSERIAL.write(0x00);
    TLSERIAL.write(0x04);
    TLSERIAL.write(X / 0x100);
    TLSERIAL.write(X % 0x100);
    TLSERIAL.write(Y / 0x100);
    TLSERIAL.write(Y % 0x100);
}

void DWN_RData(const long ID, const int DataLen)
{

    int iLen = 4;
    TLSERIAL.write(DWN_HEAD0);
    TLSERIAL.write(DWN_HEAD1);
    TLSERIAL.write(iLen);
    TLSERIAL.write(DWN_READ);
    int ID0 = ID / 0x100;
    int ID1 = ID % 0x100;
    TLSERIAL.write(ID0);
    TLSERIAL.write(ID1);
    TLSERIAL.write(DataLen);
}

void DWN_Data(const long ID, long Data, const int DataLen)
{

    int iLen = 3 + DataLen;
    TLSERIAL.write(DWN_HEAD0);
    TLSERIAL.write(DWN_HEAD1);
    TLSERIAL.write(iLen);
    TLSERIAL.write(DWN_WRITE);
    int ID0 = ID / 0x100;
    int ID1 = ID % 0x100;
    TLSERIAL.write(ID0);
    TLSERIAL.write(ID1);

    if (DataLen == 4)
    {

        if (Data > 0x1000000)
        {
            TLSERIAL.write(Data / 0x1000000);
            Data = Data % 0x1000000;
        }
        else
        {
            TLSERIAL.write(0x00);
        }

        if (Data > 0x10000)
        {
            TLSERIAL.write(Data / 0x10000);
            Data = Data % 0x10000;
        }
        else
        {
            TLSERIAL.write(0x00);
        }
    }

    if (Data > 0x100)
    {
        TLSERIAL.write(Data / 0x100);
        TLSERIAL.write(Data % 0x100);
    }
    else
    {
        TLSERIAL.write(0x00);
        TLSERIAL.write(Data);
    }
}

void DWN_LED(const int LED)
{

    TLSERIAL.write(DWN_HEAD0);
    TLSERIAL.write(DWN_HEAD1);
    TLSERIAL.write(0x05);
    TLSERIAL.write(DWN_WRITE);
    TLSERIAL.write(0x00);
    TLSERIAL.write(0x82);
    TLSERIAL.write(LED);
    TLSERIAL.write(LED);
}

#if defined(DIGIPOTSS_PIN) && DIGIPOTSS_PIN > -1
#include <SPI.h>
#endif
