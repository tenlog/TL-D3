#include "Marlin.h"
#include "tl_touch_screen.h"
#include "planner.h"
#include "temperature.h"
#include "cardreader.h"
#include "ConfigurationStore.h" 


float fECOZ = 0;
bool bECOSeted = false;
String gsM117 = "";
String gsPrinting = "";
long dwn_command[255] = {0};
bool bLogoGot = false;
int i_print_page_id = 0;
int iDWNPageID = 0;

int tenlog_status_update_delay;
int tenlogScreenUpdate;
int iBeepCount = 0;
bool b_PLR_MODULE_Detected = false;
int iMoveRate = 100;
bool bInited = false;
bool bHeatingStop = false;

bool bAtvGot0 = false;
bool bAtvGot1 = false;
bool bAtv = false;
int iOldLogoID = 0;
long lAtvCode = 0;

String file_name_list[6]={""};
String file_name_long_list[6]={""};
bool b_is_last_page = false;

void tenlog_status_screen()
{
    if (tenlog_status_update_delay)
        tenlog_status_update_delay--;
    else
        tenlogScreenUpdate = 1;

    if (tenlogScreenUpdate)
    {
        tenlogScreenUpdate = 0;
        if(tl_TouchScreenType == 0)
            tenlog_screen_update_dwn();
        else
            tenlog_screen_update_tjc();
        
        tenlog_status_update_delay = 7500; /* redraw the main screen every second. This is easier then trying keep track of all things that change on the screen */
    }
}


void tenlog_screen_update_dwn()
{
    if (!bAtv)
        return;
    //static bool bISHH0;
    //if(bISHH0 != isHeatingHotend(0)){
    DWN_Data(0x8000, isHeatingHotend(0), 2);
    _delay_ms(5);
    //}
    //bISHH0 = isHeatingHotend(0);

    //if(!isHeatingHotend(0) && millis() < 30000){
    //    DWN_Data(0x8000, 0, 2);
    //}

    //static bool bISHH1;
    //if(bISHH1 != isHeatingHotend(1)){
    DWN_Data(0x8002, isHeatingHotend(1), 2);
    _delay_ms(5);
    //}
    //bISHH1 = isHeatingHotend(1);
    //if(!isHeatingHotend(1) && millis() < 30000){
    //    DWN_Data(0x8002, 0, 2);
    //}

    //static bool bISHB;
    //if(bISHB != isHeatingBed()){
    DWN_Data(0x8004, isHeatingBed(), 2);
    _delay_ms(5);
    //}
    //bISHB = isHeatingBed();
    //if(!isHeatingBed() && millis() < 30000){
    //    DWN_Data(0x8004, 0, 2);
    //}

    //static int sDTH0;
    //if(sDTH0 != degTargetHotend(0) + 0.5){
    DWN_Data(0x6000, int(degTargetHotend(0) + 0.5), 2);
    _delay_ms(5);
    //}
    //sDTH0 = degTargetHotend(0) + 0.5;

    //static int sDH0;
    //if(sDH0 != degHotend(0) + 0.5){
    DWN_Data(0x6001, int(degHotend(0) + 0.5), 2);
    _delay_ms(5);
    //}
    //sDH0 = degHotend(0) + 0.5;

    //static int sDTH1;
    //if(sDTH1 != degTargetHotend(1) + 0.5){
    DWN_Data(0x6002, int(degTargetHotend(1) + 0.5), 2);
    _delay_ms(5);
    //}
    //sDTH1 = degTargetHotend(1) + 0.5;

    //static int sDH1;
    //if(sDH1 != degHotend(1) + 0.5){
    DWN_Data(0x6003, int(degHotend(1) + 0.5), 2);
    _delay_ms(5);
    //}
    //sDH1 = degHotend(1) + 0.5;

    //static int sDTB;
    //if(sDTB != degTargetBed() + 0.5){
    DWN_Data(0x6004, int(degTargetBed() + 0.5), 2);
    _delay_ms(5);
    //}
    //sDTB = degTargetBed() + 0.5;

    //static int sDB;
    //if(sDB != degBed() + 0.5){
    DWN_Data(0x6005, int(degBed() + 0.5), 2);
    _delay_ms(5);
    //}
    //sDB = degBed() + 0.5;

    //static float sCPX;
    //if(sCPX != current_position[X_AXIS]){
    if (current_position[X_AXIS] < 0)
        DWN_Data(0x6006, (current_position[X_AXIS] * 10.0 + 0x10000), 2);
    else
        DWN_Data(0x6006, current_position[X_AXIS] * 10.0, 2);
    _delay_ms(5);
    //}
    //sCPX = current_position[X_AXIS];

    //static float sCPY;
    //if(sCPY != current_position[Y_AXIS]){
    if (current_position[Y_AXIS] < 0)
        DWN_Data(0x6007, (current_position[Y_AXIS] * 10.0 + 0x10000), 2);
    else
        DWN_Data(0x6007, current_position[Y_AXIS] * 10.0, 2);
    _delay_ms(5);
    //}
    //sCPY = current_position[Y_AXIS];

    //static float sCPZ;
    //if(sCPZ != current_position[Z_AXIS]){
    if (current_position[Z_AXIS] < 0)
        DWN_Data(0x6008, (current_position[Z_AXIS] * 10.0 + 0x10000), 2);
    else
        DWN_Data(0x6008, current_position[Z_AXIS] * 10.0, 2);
    _delay_ms(5);
    //}
    //sCPZ = current_position[Z_AXIS];

    //static int siMoveRate;
    //if(siMoveRate != iMoveRate){
    DWN_Data(0x602A, iMoveRate, 2);
    _delay_ms(5);
    //}
    //siMoveRate = iMoveRate;

    static int siFanStatic;
    if (siFanStatic > 3)
        siFanStatic = 0;
    if (fanSpeed > 0)
    {
        DWN_Data(0x8010, siFanStatic, 2);
        siFanStatic++;
    }

    //BOF For old version UI
    //static int sfanSpeed;
    int iFan = (int)((float)fanSpeed / 256.0 * 100.0 + 0.5);
    //if(sfanSpeed != fanSpeed){
    if (fanSpeed == 0)
        DWN_Data(0x8006, 0, 2);
    else
        DWN_Data(0x8006, 1, 2);

    _delay_ms(5);
    DWN_Data(0x600A, iFan, 2);
    _delay_ms(5);
    //}
    //if(millis() < 30000){
    //    if (fanSpeed == 0)
    //        DWN_Data(0x8006, 0, 2);
    //    else
    //        DWN_Data(0x8006, 1, 2);
    //   DWN_Data(0x600A, iFan, 2);
    //}
    //EOF

    ///sfanSpeed = fanSpeed;

    //static int sfeedmultiply;
    //if(sfeedmultiply != feedmultiply){
    DWN_Data(0x6052, feedmultiply, 2);
    _delay_ms(5);
    //}
    //sfeedmultiply = feedmultiply;

    String sTime = "-- :--";
    int iTimeS = 0;
    int iPercent = 0;
    if (card.sdprinting == 1)
    {
        uint16_t time = millis() / 60000 - starttime / 60000;
        sTime = String(itostr2(time / 60)) + " :" + String(itostr2(time % 60));
        iPercent = card.percentDone();
        DWN_Data(0x6051, iPercent, 2);
        _delay_ms(5);
        DWN_Data(0x8820, iPercent, 2);
        _delay_ms(5);
    }
    else
    {
        DWN_Data(0x6051, 0, 2);
        _delay_ms(5);
        DWN_Data(0x8820, 0, 2);
        _delay_ms(5);
        iPercent = 0;
        iTimeS = 1;
    }

    //static int siPercent;
    //if(siPercent != iPercent){
    //    DWN_Data(0x6051, iPercent, 2);
    //    _delay_ms(5);
    //    DWN_Data(0x8820, iPercent, 2);
    //    _delay_ms(5);
    //}
    //siPercent = iPercent;
    /*
    if(millis() < 10000 && iPercent == 0){
        DWN_Data(0x6051, 0, 2);
        _delay_ms(5);
        DWN_Data(0x8820, 0, 2);
        _delay_ms(5);    
    }
    */

    //static int iCPI;
    //if(iCPI != card.sdprinting){
    DWN_Data(0x8840, card.sdprinting + languageID * 3, 2);
    _delay_ms(5);
    DWN_Data(0x8842, card.sdprinting, 2);
    _delay_ms(5);
    //}
    //iCPI = card.sdprinting;

    //static String ssTime;
    //if(ssTime != sTime){
    DWN_Text(0x7540, 8, sTime);
    _delay_ms(5);
    //}
    //ssTime = sTime;
    /*
    if(millis() < 10000){
        DWN_Text(0x7540, 8, "-- :--");
        _delay_ms(5);    
        DWN_Data(0x8841, 0, 2);
        _delay_ms(5);    
    }
    */
    //static int siTimeS;
    //if(siTimeS != iTimeS){
    DWN_Data(0x8841, iTimeS, 2);
    _delay_ms(5);
    //}
    //siTimeS = iTimeS;

    static int iECOBedT;
    if (current_position[Z_AXIS] >= ECO_HEIGHT && !bECOSeted && iPercent > 1 && tl_ECO_MODE == 1)
    {
        iECOBedT = degTargetBed();
        setTargetBed(0);
        bECOSeted = true;
    }
    else if (current_position[Z_AXIS] >= ECO_HEIGHT && card.sdprinting == 1 && tl_ECO_MODE == 0 && bECOSeted && iECOBedT > 0)
    {
        setTargetBed(iECOBedT);
    }

    if (current_position[Z_AXIS] <= ECO_HEIGHT && bECOSeted)
    {
        bECOSeted = false;
    }

    static int siCM;
    int iCM;

    if (dual_x_carriage_mode == 2)
    {
        iCM = 3;
    }
    else if (dual_x_carriage_mode == 3)
    {
        iCM = 4;
    }
    else if (dual_x_carriage_mode == 1)
    {
        static bool bAPMNozzle;
        bAPMNozzle = !bAPMNozzle;
        if (active_extruder == 0 && bAPMNozzle)
        {
            iCM = 1;
        }
        else if (active_extruder == 1 && bAPMNozzle)
        {
            iCM = 2;
        }
        else if (!bAPMNozzle)
        {
            iCM = 0;
        }
    }
    if (siCM != iCM)
    {
        DWN_Data(0x8800, iCM, 2);
        _delay_ms(5);
    }
    siCM = iCM;

    int iMode = (dual_x_carriage_mode - 1) + languageID * 3;
    //static int siMode;
    //if(siMode != iMode || millis() < 30000){
    DWN_Data(0x8801, iMode, 2);
    DWN_Data(0x8804, (dual_x_carriage_mode - 1), 2);
    _delay_ms(5);
    //}

    //siMode = iMode;

    //static int siAN;
    int iAN = active_extruder + languageID * 2;
    //if(siAN != iAN){
    DWN_Data(0x8802, iAN, 2); // is for UI V1.3.6
    DWN_Data(0x8805, active_extruder, 2);
    _delay_ms(5);
    //}
    //siAN = iAN;

    if (gsM117 != "" && gsM117 != "Printing...")
    { //Do not display "Printing..."
        String sPrinting = "";
        static int icM117;

        if (icM117 > 0)
        {
            icM117--;
        }

        if (icM117 == 0)
        {
            sPrinting = gsM117;
            icM117 = 60;
        }
        else if (icM117 == 30)
        {
            sPrinting = gsPrinting;
        }

        if (icM117 == 30 || icM117 == 0 || icM117 == 60)
        { //Switch message every 30 secounds
            DWN_Text(0x7500, 32, sPrinting, true);
        }
    }

    //static long sprint_from_z_target;
    //if(sprint_from_z_target != print_from_z_target){
    DWN_Data(0x6041, (long)(print_from_z_target * 10.0), 2);
    _delay_ms(5);
    //}
    //if(print_from_z_target == 0.0 && millis() < 10000){
    //    DWN_Data(0x6041, 0.0, 2);
    //    _delay_ms(5);
    //}
    //sprint_from_z_target = print_from_z_target;

    if (iDWNPageID == DWN_P_PRINTING && !card.isFileOpen())
    {
        //DWN_Page(DWN_P_MAIN);
    }
    else if (iDWNPageID == DWN_P_MAIN && card.sdprinting == 1)
    {
        DWN_Page(DWN_P_PRINTING);
    }

    if (lLEDTimeTimecount <= DWN_LED_TIMEOUT)
    {
        lLEDTimeTimecount++;
    }

    if (lLEDTimeTimecount == DWN_LED_TIMEOUT)
    {
        DWN_LED(DWN_LED_OFF);
        lLEDTimeTimecount++;
    }

    if (iBeepCount >= 0)
    {
#if (BEEPER > 0)
        if (iBeepCount % 2 == 1)
        {
            WRITE(BEEPER, BEEPER_ON);
        }
        else
        {
            WRITE(BEEPER, BEEPER_OFF);
        }
#endif
        iBeepCount--;
    }
    if (!bInited)
    {
        Init_TLScreen_dwn();
        bInited = true;
    }
}

/*
Start Print	 	            0
Print finished	 	        1
Power Off	 	            2
Power Loss Detected	 	    3
Reset Default	 	        4
Stop Print 	 	            5
Filament runout!	 	    6
input Z height	 	        7
Nozzle heating error	 	8
Nozzle High temp error	 	9
Nozzle Low Temp error	 	10
Bed High temp error	 	    11
Bed Low temp error	 	    12
*/
void DWN_Message(const int MsgID, const String sMsg, const bool PowerOff)
{
    dwnMessageID = MsgID;
    int iSend = dwnMessageID + languageID * 13;
    if (dwnMessageID == 13)
        iSend = 3 + languageID * 13;

    DWN_Data(0x9052, dwnMessageID, 2);
    DWN_Data(0x9050, iSend, 2);
    _delay_ms(5);
    DWN_Text(0x7000, 32, sMsg);
    _delay_ms(5);

    if (PowerOff == 0)
        iSend = 0;
    else
        iSend = PowerOff + languageID;

    DWN_Data(0x8830, iSend, 2);
    _delay_ms(5);
    DWN_Page(DWN_P_MSGBOX);
}


int iPrintID = -1;
void DWN_MessageBoxHandler(const bool ISOK)
{
    switch (dwnMessageID)
    {
    case MSG_RESET_DEFALT:
        if (card.isFileOpen())
        {
            DWN_Page(DWN_P_SETTING_PRINTING);
        }
        else
        {
            DWN_Page(DWN_P_SETTING_MAIN);
        }

        if (ISOK)
            command_M502();
        break;
    case MSG_POWER_OFF:
        if (ISOK)
            command_M81(false);
        else
            DWN_Page(DWN_P_TOOLS);
        break;
    case MSG_START_PRINT:
        if (ISOK)
        {
            if (file_name_list[iPrintID] != "")
            {

                if (print_from_z_target > 0)
                    PrintFromZHeightFound = false;
                else
                    PrintFromZHeightFound = true;

                if (card.sdprinting == 1)
                {
                    st_synchronize();
                    card.closefile();
                }
                const char *str0 = file_name_list[iPrintID].c_str();
                const char *str1 = file_name_long_list[iPrintID].c_str();

                feedrate = 4000;
                card.openFile(str1, str0, true);
                card.startFileprint();
                starttime = millis();
                DWN_Page(DWN_P_PRINTING);
                gsPrinting = "Printing " + file_name_long_list[iPrintID];
                DWN_Text(0x7500, 32, gsPrinting, true);
            }
        }
        else
        {
            if (print_from_z_target > 0)
                DWN_Page(DWN_P_SEL_Z_FILE);
            else
                DWN_Page(DWN_P_SEL_FILE);
        }
        break;
    case MSG_PRINT_FINISHED:
        DWN_Page(DWN_P_MAIN);
        break;
    case MSG_STOP_PRINT:
        if (ISOK)
        {
            DWN_Text(0x7000, 32, F(" Stopping, Pls wait..."));
            quickStop();
            bHeatingStop = true;
            enquecommand_P(PSTR("M1033"));
        }
        else
            DWN_Page(DWN_P_PRINTING);
        break;
    case MSG_INPUT_Z_HEIGHT:
        DWN_Page(DWN_P_PRINTZ);
        break;
    case MSG_NOZZLE_HEATING_ERROR:
    case MSG_NOZZLE_HIGH_TEMP_ERROR:
    case MSG_NOZZLE_LOW_TEMP_ERROR:
    case MSG_BED_HIGH_TEMP_ERROR:
    case MSG_BED_LOW_TEMP_ERROR:
        sdcard_stop();
        break;
    case MSG_FILAMENT_RUNOUT:
        enquecommand_P(PSTR("M605 S1"));
        _delay_ms(300);
        enquecommand_P(PSTR("G28 X"));
        _delay_ms(100);
        if (card.isFileOpen() && card.sdprinting == 0)
            DWN_Page(DWN_P_RELOAD);
        break;
    case MSG_POWER_LOSS_DETECTED:
        bAtv = true;
        if (ISOK)
        {
            command_M1003();
        }
        else
        {
            DWN_Page(DWN_P_MAIN);
#if defined(POWER_LOSS_SAVE_TO_EEPROM)
            EEPROM_Write_PLR();
            EEPROM_PRE_Write_PLR();
#elif defined(POWER_LOSS_SAVE_TO_SDCARD)
            card.Write_PLR();
            card.PRE_Write_PLR();
#endif
        }
        break;
    }
}


void CheckTempError_tjc()
{
    if (iTempErrID > 0)
    {
        String strMessage = sTempErrMsg;

        TLSTJC_printconstln(F("sleep=0"));
        TLSTJC_printconstln(F("msgbox.vaFromPageID.val=1"));
        TLSTJC_printconstln(F("msgbox.vaToPageID.val=1"));
        TLSTJC_printconst(F("msgbox.vaMID.val="));
        strMessage = "" + String(iTempErrID) + "";
        const char *str1 = strMessage.c_str();
        TLSTJC_println(str1);

        TLSTJC_printconst(F("msgbox.vtMS.txt=\""));
        strMessage = " " + sShortErrMsg + "";
        str1 = strMessage.c_str();
        TLSTJC_print(str1);
        TLSTJC_printconstln(F("\""));
        sShortErrMsg = "";

        TLSTJC_printconst(F("msgbox.tMessage.txt=\""));
        strMessage = "" + strMessage + "";
        str1 = strMessage.c_str();
        TLSTJC_print(str1);
        TLSTJC_printconstln(F("\""));
        TLSTJC_printconstln(F("page msgbox"));
        
#ifdef HAS_PLR_MODULE
        if (iTempErrID == MSG_NOZZLE_HEATING_ERROR || iTempErrID == MSG_NOZZLE_HIGH_TEMP_ERROR || iTempErrID == MSG_BED_HIGH_TEMP_ERROR)
        {
            _delay_ms(5000);
            command_M81(false, false);
        }
#endif
        iTempErrID = 0;

    }
}

void Init_TLScreen_tjc()
{
    _delay_ms(20);
    TLSTJC_printconst(F("main.vLanguageID.val="));
    TLSTJC_print(String(languageID).c_str());
    TLSTJC_printend();
    _delay_ms(20);

    long iSend = tl_X2_MAX_POS * 100.0;
    TLSTJC_printconst(F("setting.xX2.val="));
    TLSTJC_print(String(iSend).c_str());
    TLSTJC_printend();
    _delay_ms(20);

    TLSTJC_printconst(F("offset.xX2.val="));
    TLSTJC_print(String(iSend).c_str());
    TLSTJC_printend();
    _delay_ms(20);

    iSend = tl_Y2_OFFSET * 100.0;
    TLSTJC_printconst(F("setting.xY2.val="));
    TLSTJC_print(String(iSend).c_str());
    TLSTJC_printend();
    _delay_ms(20);

    TLSTJC_printconst(F("offset.xY2.val="));
    TLSTJC_print(String(iSend).c_str());
    TLSTJC_printend();
    _delay_ms(20);

    iSend = tl_Z2_OFFSET * 100.0;
    TLSTJC_printconst(F("setting.xZ2.val="));
    TLSTJC_print(String(iSend).c_str());
    TLSTJC_printend();
    _delay_ms(20);

    iSend = axis_steps_per_unit[0] * 100.0;
    TLSTJC_printconst(F("setting.xXs.val="));
    TLSTJC_print(String(iSend).c_str());
    TLSTJC_printend();
    _delay_ms(20);

    iSend = axis_steps_per_unit[1] * 100.0;
    TLSTJC_printconst(F("setting.xYs.val="));
    TLSTJC_print(String(iSend).c_str());
    TLSTJC_printend();
    _delay_ms(20);

    iSend = axis_steps_per_unit[2] * 100.0;
    TLSTJC_printconst(F("setting.xZs.val="));
    TLSTJC_print(String(iSend).c_str());
    TLSTJC_printend();
    _delay_ms(20);

    iSend = axis_steps_per_unit[3] * 100.0;
    TLSTJC_printconst(F("setting.xEs.val="));
    TLSTJC_print(String(iSend).c_str());
    TLSTJC_printend();
    _delay_ms(20);

#ifdef FAN2_CONTROL
    iSend = tl_FAN2_VALUE;
    TLSTJC_printconst(F("setting.nF2s.val="));
    TLSTJC_print(String(iSend).c_str());
    TLSTJC_printend();
#else
    TLSTJC_printconstln(F("setting.nF2s.val=0"));
#endif
    _delay_ms(20);

#ifdef FAN2_CONTROL
    iSend = tl_FAN2_START_TEMP;
    TLSTJC_printconst(F("setting.nF2t.val="));
    TLSTJC_print(String(iSend).c_str());
    TLSTJC_printend();
#else
    TLSTJC_printconstln(F("setting.nF2t.val=0"));
#endif
    _delay_ms(20);

    iSend = tl_X2_MAX_POS * 10.0;
    TLSTJC_printconst(F("main.vXMax.val="));
    TLSTJC_print(String(iSend).c_str());
    TLSTJC_printend();
    _delay_ms(20);

    iSend = Y_MAX_POS * 10.0;
    TLSTJC_printconst(F("main.vYMax.val="));
    TLSTJC_print(String(iSend).c_str());
    TLSTJC_printend();
    _delay_ms(20);

    iSend = Z_MAX_POS * 10.0;
    TLSTJC_printconst(F("main.vZMax.val="));
    TLSTJC_print(String(iSend).c_str());
    TLSTJC_printend();
    _delay_ms(20);

    iSend = tl_HEATER_0_MAXTEMP;
    TLSTJC_printconst(F("main.vTempMax.val="));
    TLSTJC_print(String(iSend).c_str());
    TLSTJC_printend();
    _delay_ms(20);

    iSend = tl_BED_MAXTEMP;
    TLSTJC_printconst(F("main.vTempMaxBed.val="));
    TLSTJC_print(String(iSend).c_str());
    TLSTJC_printend();
    _delay_ms(20);

#ifdef HAS_PLR_MODULE
    if (b_PLR_MODULE_Detected)
        TLSTJC_printconstln(F("main.vPFR.val=1"));
    else
        TLSTJC_printconstln(F("main.vPFR.val=0"));

    _delay_ms(20);

    iSend = tl_AUTO_OFF;
    TLSTJC_printconst(F("setting.cAutoOff.val="));
    TLSTJC_print(String(iSend).c_str());
    TLSTJC_printend();
    _delay_ms(20);

#endif //HAS_PLR_MODULE

    iSend = tl_Filament_Detect;
    TLSTJC_printconst(F("setting.cFilaSensor.val="));
    TLSTJC_print(String(iSend).c_str());
    TLSTJC_printend();
    _delay_ms(20);

    iSend = tl_ECO_MODE;
    TLSTJC_printconst(F("setting.cECOMode.val="));
    TLSTJC_print(String(iSend).c_str());
    TLSTJC_printend();
    _delay_ms(20);

    iSend = tl_SLEEP_TIME;
    TLSTJC_printconst(F("setting.nSleep.val="));
    TLSTJC_print(String(iSend).c_str());
    TLSTJC_printend();
    _delay_ms(20); //
    TLSTJC_printconstln(F("sleep=0"));
    iBeepCount = 2;
    String strDate = __DATE__;
    TLSTJC_printconst(F("about.tVer.txt=\""));
    TLSTJC_printconst(F(FW_STR));
#ifdef HAS_PLR_MODULE
    if (b_PLR_MODULE_Detected)
        TLSTJC_printconst(F(" PLR "));
    else
        TLSTJC_printconst(F(" "));
#endif
    _delay_ms(20);
    TLSTJC_printconst(F("V "));
    TLSTJC_printconst(F(VERSION_STRING));
    TLSTJC_printconstln(F("\""));
    _delay_ms(20);
    TLSTJC_printconstln(F("bkcmd=0"));
    _delay_ms(20); //
#if (BEEPER > 0)
    SET_OUTPUT(BEEPER);
    WRITE(BEEPER, BEEPER_OFF);
#endif
}

void tenlog_screen_update_tjc()
{
    String strAll = "main.sStatus.txt=\"";
    long lN = current_position[X_AXIS] * 10.0; //1
    String sSend = String(lN);
    strAll = strAll + sSend + "|";              //6

    lN = current_position[Y_AXIS] * 10.0; //2
    sSend = String(lN);
    strAll = strAll + sSend + "|";              //12

    lN = current_position[Z_AXIS] * 10.0; //3
    sSend = String(lN);
    strAll = strAll + sSend + "|";              //18

    lN = current_position[E_AXIS] * 10.0; //4
    sSend = String(lN);
    strAll = strAll + "|"; //do not sent E Position    //19
    //strAll = strAll + sSend + "|";

    lN = int(degTargetHotend(0) + 0.5); //5
    sSend = String(lN);
    strAll = strAll + sSend + "|";              //23

    lN = int(degHotend(0) + 0.5); //6
    sSend = String(lN);
    strAll = strAll + sSend + "|";              //27

    lN = int(degTargetHotend(1) + 0.5); //7
    sSend = String(lN);
    strAll = strAll + sSend + "|";              //31

    lN = int(degHotend(1) + 0.5); //8
    sSend = String(lN);
    strAll = strAll + sSend + "|";              //35

    lN = int(degTargetBed() + 0.5); //9
    sSend = String(lN);
    strAll = strAll + sSend + "|";              //3

    lN = int(degBed() + 0.5); //10
    sSend = String(lN);
    strAll = strAll + sSend + "|";

    lN = fanSpeed * 100.0 / 255.0 + 0.5; //11
    sSend = String(lN);
    strAll = strAll + sSend + "|";

    lN = feedmultiply; //12
    sSend = String(lN);
    strAll = strAll + sSend + "|";

    int iPercent = 0;
    if (card.sdprinting == 1) //13
    {
        strAll = strAll + "1|";
        lN = card.percentDone();
        iPercent = card.percentDone();
        sSend = String(lN); //14
        strAll = strAll + sSend + "|";
    }
    else if (card.sdprinting == 0)
    {
        strAll = strAll + "0|0|";
    }
    else if (card.sdprinting == 2)
    {
        strAll = strAll + "2|0|";
    }

    lN = active_extruder; //15
    sSend = String(lN);
    strAll = strAll + sSend + "|";

    lN = dual_x_carriage_mode; //16
    sSend = String(lN);
    strAll = strAll + sSend + "|";

    //lN=dual_x_carriage_mode;                //17 time
    if (IS_SD_PRINTING)
    {
        uint16_t time = millis() / 60000 - starttime / 60000;
        sSend = String(itostr2(time / 60)) + ":" + String(itostr2(time % 60));
        strAll = strAll + sSend + "|";
    }
    else
    {
        strAll = strAll + "00:00|";
    }

    if (card.isFileOpen())
    { //18 is file open
        strAll = strAll + "1|";
    }
    else
    {
        strAll = strAll + "0|";
    }

    if (isHeatingHotend(0))
    { //19 is heating nozzle 0
        strAll = strAll + "1|";
    }
    else
    {
        strAll = strAll + "0|";
    }

    if (isHeatingHotend(1))
    { //20 is heating nozzle 1
        strAll = strAll + "1|";
    }
    else
    {
        strAll = strAll + "0|";
    }

    if (isHeatingBed())
    { //21 is heating Bed
        strAll = strAll + "1|";
    }
    else
    {
        strAll = strAll + "0|";
    }

    strAll = strAll + "\"";
    const char *strAll0 = strAll.c_str();
    TLSTJC_println(strAll0);

    static int iECOBedT;
    if (current_position[Z_AXIS] >= ECO_HEIGHT && !bECOSeted && iPercent > 1 && tl_ECO_MODE == 1)
    {
        iECOBedT = degTargetBed();
        setTargetBed(0);
        bECOSeted = true;
    }
    else if (current_position[Z_AXIS] >= ECO_HEIGHT && tl_ECO_MODE == 0 && bECOSeted && iECOBedT > 0)
    {
        setTargetBed(iECOBedT);
    }

    if (current_position[Z_AXIS] <= ECO_HEIGHT && bECOSeted)
    {
        bECOSeted = false;
    }

    if (gsM117 != "" && gsM117 != "Printing...")
    { //Do not display "Printing..."
        static int icM117;

        if (icM117 > 0)
        {
            icM117--;
        }

        if (icM117 == 0)
        {
            _delay_ms(50);
            TLSTJC_printconst(F("printing.tM117.txt=\""));            
            String strM117 = "" + gsM117 + "";
            const char *strM1170 = strM117.c_str();
            TLSTJC_println(strM1170);
            TLSTJC_printconstln(F("\""));
            icM117 = 60;
        }
        else if (icM117 == 30)
        {
            _delay_ms(50);
            TLSTJC_printconstln(F("printing.tM117.txt=\"\""));
            _delay_ms(50);
        }
    }

    _delay_ms(50);
    TLSTJC_printconstln(F("click btReflush,0"));

    if (iBeepCount >= 0)
    {

#if (BEEPER > 0)
        if (iBeepCount % 2 == 1)
        {
            WRITE(BEEPER, BEEPER_ON);
        }
        else
        {
            WRITE(BEEPER, BEEPER_OFF);
        }
#endif
        iBeepCount--;
    }
    if (!bInited)
    {
        Init_TLScreen_tjc();
        bInited = true;
    }
}

void sdcard_tlcontroller_dwn()
{
    card.initsd();
    _delay_ms(50);
    uint16_t fileCnt = card.getnrfilenames();
    card.getWorkDirName();
    if (card.filename[0] == '/')
    {
    }
    else
    {
    }

    if (i_print_page_id == 0)
    {
        DWN_Data(0x8810, 1, 2);
    }
    else
    {
        DWN_Data(0x8810, 0, 2);
    }

    int iFileID = 0;
    //Clear the boxlist
    for (int i = 0; i < 6; i++)
    {
        DWN_Text(0x7300 + i * 0x30, 32, "");
        file_name_list[i] = "";
        file_name_long_list[i] = "";
    }

    for (uint16_t i = 0; i < fileCnt; i++)
    {
        card.getfilename(fileCnt - 1 - i);
        String strFN = String(card.filename);

        if (!card.filenameIsDir && strFN.length() > 0)
        {
            if (strISAscii(strFN))
            {
                strFN = String(card.longFilename);
                strFN.toLowerCase();
                String strLFN = strFN;
                iFileID++;
                if (iFileID >= (i_print_page_id)*6 + 1 && iFileID <= (i_print_page_id + 1) * 6)
                {
                    int iFTemp = iFileID - (i_print_page_id)*6;

                    strFN = String(card.filename);
                    strFN.toLowerCase();
                    if (strLFN == "")
                        strLFN = strFN;
                    DWN_Text(0x7300 + (iFTemp - 1) * 0x30, 32, strLFN.c_str());
                    file_name_list[iFTemp - 1] = strFN.c_str();
                    file_name_long_list[iFTemp - 1] = strLFN.c_str();
                }
            }
        }
    }

    if ((i_print_page_id + 1) * 6 >= iFileID)
    {
        DWN_Data(0x8811, 1, 2);
        b_is_last_page = true;
    }
    else
    {
        DWN_Data(0x8811, 0, 2);
        b_is_last_page = false;
    }
}


void sdcard_tlcontroller_tjc()
{
    uint16_t fileCnt = card.getnrfilenames();
    card.getWorkDirName();
    if (card.filename[0] == '/')
    {
    }
    else
    {
    }

    if (i_print_page_id == 0)
    {
        TLSTJC_printconstln(F("vis btUp,0"));
    }
    else
    {
        TLSTJC_printconstln(F("vis btUp,1"));
    }

    int iFileID = 0;
    //Clear the boxlist
    for (int i = 1; i < 7; i++)
    {
        TLSTJC_print("select_file.tL");
        TLSTJC_print(String(i).c_str());
        TLSTJC_print(".txt=\"\"");
        TLSTJC_printend();

        TLSTJC_print("select_file.sL");
        TLSTJC_print(String(i).c_str());
        TLSTJC_print(".txt=\"\"");
        TLSTJC_printend();
    }

    for (uint16_t i = 0; i < fileCnt; i++)
    {
        card.getfilename(fileCnt - 1 - i);    //card.getfilename(i);   // card.getfilename(fileCnt-1-i); //By Zyf sort by time desc
        String strFN = String(card.filename); // + " | " + String(card.filename);

        if (!card.filenameIsDir && strFN.length() > 0)
        {
            if (strISAscii(strFN))
            {
                strFN = String(card.longFilename);
                strFN.toLowerCase();
                String strLFN = strFN;
                iFileID++;
                if (iFileID >= (i_print_page_id)*6 + 1 && iFileID <= (i_print_page_id + 1) * 6)
                {
                    strFN = String(card.filename);
                    strFN.toLowerCase();

                    if (strLFN == "")
                        strLFN = strFN;

                    int iFTemp = iFileID - (i_print_page_id)*6;
                    TLSTJC_print("select_file.tL");
                    TLSTJC_print(String(iFTemp).c_str());
                    TLSTJC_print(".txt=\"");
                    strLFN.toLowerCase();
                    TLSTJC_print(strLFN.c_str());
                    TLSTJC_print("\"");
                    TLSTJC_printend();

                    TLSTJC_print("select_file.sL");
                    TLSTJC_print(String(iFTemp).c_str());
                    TLSTJC_print(".txt=\"");
                    TLSTJC_print(strFN.c_str());
                    TLSTJC_print("\"");
                    TLSTJC_printend();
                }
                //MENU_ITEM(sdfile, MSG_CARD_MENU, card.filename, card.longFilename);
            }
        }
    }

    TLSTJC_printconst(F("select_file.vPageID.val="));
    TLSTJC_print(String(i_print_page_id).c_str());
    TLSTJC_printend();

    if ((i_print_page_id + 1) * 6 >= iFileID)
    {
        TLSTJC_printconstln(F("vis btDown,0"));
    }
    else
    {
        TLSTJC_printconstln(F("vis btDown,1"));
    }
}

