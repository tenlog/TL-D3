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

#ifdef TL_TJC_CONTROLLER
void TenlogScreen_begin(int boud){
  Serial2.begin(boud);
}

char chrEnd = 0xFF;
void TenlogScreen_println(const char s[]) {
	Serial2.print(s);
	Serial2.write(chrEnd);
	Serial2.write(chrEnd);
	Serial2.write(chrEnd);
}

void TenlogScreen_print(const char s[]) {
	Serial2.print(s);
}

void TenlogScreen_printend() {
	Serial2.write(chrEnd);
	Serial2.write(chrEnd);
	Serial2.write(chrEnd);
}

void TenlogScreen_printEmptyend() {
	Serial2.write(0x00);
	Serial2.write(chrEnd);
	Serial2.write(chrEnd);
	Serial2.write(chrEnd);    
}

#endif

#if defined(TL_TJC_CONTROLLER) || defined(TL_DWN_CONTROLLER)
bool MSerial2_available(){
    return Serial2.available();
}

int MSerial2_read(){
    return Serial2.read();
}

#endif

#ifdef TL_DWN_CONTROLLER

#define DWN_HEAD0 0x5A
#define DWN_HEAD1 0xA5
#define DWN_WRITE 0x82
#define DWN_READ  0x83

#define CP_ADD {0x00,0x84}
#define CP_CMD {0x5A,0x01}

void DWN_begin(){
    Serial2.begin(115200);
}

void DWN_Page(int ID){    
    byte add[] = CP_ADD;
    byte cmd[] = CP_CMD;
    Serial2.write(DWN_HEAD0);
    Serial2.write(DWN_HEAD1);
    Serial2.write(3 + sizeof(add) + sizeof(cmd));
    Serial2.write(DWN_WRITE);
    Serial2.write(add, sizeof(add));
    Serial2.write(cmd, sizeof(cmd));
    Serial2.write(0x00);
    Serial2.write(ID);
    iDWNPageID = ID;
}

void DWN_Text(long ID, int Len, String s, bool Center=false){
    Serial2.write(DWN_HEAD0);
    Serial2.write(DWN_HEAD1);
    Serial2.write(3 + Len);
    Serial2.write(0x82);
    int ID0 = ID / 0x100;
    int ID1 = ID % 0x100;

    Serial2.write(ID0);
    Serial2.write(ID1);
        
    if(s.length() > Len - 2){
        s = s.substring(0,Len - 2);
        Serial2.print(s);
        Serial2.write(0xFF);
        Serial2.write(0xFF);
    }else{
        if(!Center){
            Serial2.print(s);
            for(int i=0; i<Len-s.length() - 2; i++){
                Serial2.print(" ");
            }
            Serial2.write(0xFF);
            Serial2.write(0xFF);
        }else{
            String s1 = "";
            int Count = 0;
            for(int i=0; i<(Len-s.length() - 2)/2; i++){
                s1 += " ";
                Count++;
            }
            Serial2.print(s1);
            Serial2.print(s);
            Count += s.length();
            s1 = "";
            for(int i=0; i<Len-Count-2; i++){
                s1 += " ";
            }
            Serial2.print(s1);
            Serial2.write(0xFF);
            Serial2.write(0xFF);
        }
    }
    
}

void DWN_Language(int ID){
    DWN_Change_Icon(0x90, 0x40, ID);
    DWN_Change_Icon(0x80, 0x10, !ID);
    DWN_Change_Icon(0x80, 0x11, ID);
}

void DWN_Change_Icon(int IID0, int IID1, int ID){
    //5A A5 05 82 50 31 00 01
    Serial2.write(DWN_HEAD0);
    Serial2.write(DWN_HEAD1);
    Serial2.write(0x05);
    Serial2.write(0x82);
    Serial2.write(IID0);
    Serial2.write(IID1);
    Serial2.write(0x00);
    Serial2.write(ID);    
}


void DWN_NORFData(long NorID, long ID, int Length, bool WR){
    int iWR = 0x5A;
    if(WR) iWR = 0xA5;
    Serial2.write(DWN_HEAD0);
    Serial2.write(DWN_HEAD1);
    Serial2.write(0x0B);
    Serial2.write(0x82);
    Serial2.write(0x00);
    Serial2.write(0x08);
    Serial2.write(iWR);
    int Nor0 = NorID / 0x10000;
    NorID = NorID % 0x10000;
    int Nor1 = NorID / 0x100;
    int Nor2 = NorID % 0x100;
    Serial2.write(Nor0);
    Serial2.write(Nor1);
    Serial2.write(Nor2);
    int ID0 = ID / 0x100;
    int ID1 = ID % 0x100;
    Serial2.write(ID0);
    Serial2.write(ID1);
    Serial2.write(0x00);
    Serial2.write(Length);
}

void DWN_VClick(int X, int Y){
    Serial2.write(DWN_HEAD0);
    Serial2.write(DWN_HEAD1);
    Serial2.write(0x0B);
    Serial2.write(0x82);
    Serial2.write(0x00);
    Serial2.write(0xD4);
    Serial2.write(DWN_HEAD0);
    Serial2.write(DWN_HEAD1);
    Serial2.write(0x00);
    Serial2.write(0x04);
    Serial2.write(X/0x100);
    Serial2.write(X%0x100);
    Serial2.write(Y/0x100);
    Serial2.write(Y%0x100);
}

void DWN_RData(long ID, int DataLen){

    int iLen = 4;
    Serial2.write(DWN_HEAD0);
    Serial2.write(DWN_HEAD1);
    Serial2.write(iLen);
    Serial2.write(DWN_READ);
    int ID0 = ID / 0x100;
    int ID1 = ID % 0x100;
    Serial2.write(ID0);
    Serial2.write(ID1);
    Serial2.write(DataLen);
}

void DWN_Data(long ID, long Data, int DataLen){

    int iLen = 3 + DataLen;
    Serial2.write(DWN_HEAD0);
    Serial2.write(DWN_HEAD1);
    Serial2.write(iLen);
    Serial2.write(DWN_WRITE);
    int ID0 = ID / 0x100;
    int ID1 = ID % 0x100;
    Serial2.write(ID0);
    Serial2.write(ID1);
    
    if(DataLen == 4){
        
        if(Data > 0x1000000){
            Serial2.write(Data / 0x1000000);
            Data = Data % 0x1000000;
        }else{
            Serial2.write(0x00);
        }

        if(Data > 0x10000){
            Serial2.write(Data / 0x10000);
            Data = Data % 0x10000;
        }else{
            Serial2.write(0x00);
        }
    }

    if(Data > 0x100){
        Serial2.write(Data / 0x100);
        Serial2.write(Data % 0x100);        
    }else{
        Serial2.write(0x00);
        Serial2.write(Data);
    }
}

void DWN_LED(int LED){

    Serial2.write(DWN_HEAD0);
    Serial2.write(DWN_HEAD1);
    Serial2.write(0x05);
    Serial2.write(DWN_WRITE);
    Serial2.write(0x00);
    Serial2.write(0x82);
    Serial2.write(LED);
    Serial2.write(LED);
}

#endif

#if defined(DIGIPOTSS_PIN) && DIGIPOTSS_PIN > -1
#include <SPI.h>
#endif
