
#ifndef TL_TOUCH_SCREEN_H
#define TL_TOUCH_SCREEN_H

#include "Arduino.h"

//Setting for DWIN touch screen
#define DWN_P_LOADING 21
#define DWN_P_MAIN 41
#define DWN_P_TOOLS 43
#define DWN_P_ABOUT 31
#define DWN_P_SETTING_MAIN 45
#define DWN_P_SETTING_PRINTING 69
#define DWN_P_MOVE 47
#define DWN_P_SEL_Z_FILE 49
#define DWN_P_SEL_FILE 70
#define DWN_P_PRINTING 0x33
#define DWN_P_TEMP 53
#define DWN_P_MODE 57
#define DWN_P_RELOAD 59
#define DWN_P_SHUTDOWN 61
#define DWN_P_PRINTZ 63
#define DWN_P_MSGBOX 14

#define DWN_TXT_VERSION 0x10
#define DWN_TXT_LOADING 0x00
#define DWN_TXT_FILE0 0x51
#define DWN_TXT_PRINTFILE 0x20
#define DWN_TXT_PERCENT 0x21

#define DWN_LED_ON 74
#define DWN_LED_OFF 03
#define DWN_LED_TIMEOUT 300

#define MSG_START_PRINT 0
#define MSG_PRINT_FINISHED 1
#define MSG_POWER_OFF 2
#define MSG_POWER_LOSS_DETECTED 3
#define MSG_RESET_DEFALT 4
#define MSG_STOP_PRINT 5
#define MSG_FILAMENT_RUNOUT 6
#define MSG_INPUT_Z_HEIGHT 7

#define MSG_NOZZLE_HEATING_ERROR 8
#define MSG_NOZZLE_HIGH_TEMP_ERROR 9
#define MSG_NOZZLE_LOW_TEMP_ERROR 10
#define MSG_BED_HIGH_TEMP_ERROR 11
#define MSG_BED_LOW_TEMP_ERROR 12

void tenlog_status_screen();
void TenlogScreen_begin(const long boud);

void DWN_MessageBoxHandler(bool ISOK);
void DWN_LED(int LED) ;
void DWN_Page(int ID);
void DWN_Text(long ID, int Len, String s, bool Center = false);
void DWN_Language(int ID);
void DWN_Data(long ID, long Data, int DataLen);
void process_command_dwn();
void DWN_Message(int MsgID, String sMsg, bool PowerOff);
void DWN_NORFData(long NorID, long ID, int Lenth, bool WR);
void DWN_RData(long ID, int DataLen);
void DWN_VClick(int X, int Y);

void tenlog_screen_update_dwn();
void tenlog_screen_update_tjc();
void Init_TLScreen_tjc();
void Init_TLScreen_dwn();

void CheckTempError_tjc();

void TLSTJC_println(const char s[]);
void TLSTJC_printconstln(const String s);
void TLSTJC_printconst(const String s);
void TLSTJC_print(const char s[]);
void TLSTJC_printend();
void TLSTJC_printEmptyend();

void get_command_dwn();
void get_command_tjc();
void get_command();
void process_commands();
void manage_inactivity();

void sdcard_tlcontroller_tjc();
void sdcard_tlcontroller_dwn();

extern int i_print_page_id;
extern bool b_is_last_page;
extern String file_name_list[6];
extern String file_name_long_list[6];
extern int iDWNPageID;
static float feedrate = 1500.0, next_feedrate, saved_feedrate;

extern int iPrintID;
extern String gsM117;
extern String gsPrinting;

extern long dwn_command[255];
extern bool bLogoGot;

extern int tenlog_status_update_delay;
extern int tenlogScreenUpdate;
extern int iBeepCount ;
extern bool b_PLR_MODULE_Detected ;
extern int iMoveRate ;
extern bool bInited ;
extern bool bHeatingStop ;

extern bool bAtvGot0 ;
extern bool bAtvGot1;
extern bool bAtv;
extern int iOldLogoID;
extern long lAtvCode;



#endif //TL_TOUCH_SCREEN_H