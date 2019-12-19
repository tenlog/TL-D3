#include "Marlin.h"
#include "planner.h"
#include "temperature.h"
#include "ultralcd.h"
#include "ConfigurationStore.h"

void _EEPROM_writeData(int &pos, uint8_t* value, uint8_t size)
{
    do
    {
        eeprom_write_byte((unsigned char*)pos, *value);
        pos++;
        value++;
    }while(--size);
}
#define EEPROM_WRITE_VAR(pos, value) _EEPROM_writeData(pos, (uint8_t*)&value, sizeof(value))
void _EEPROM_readData(int &pos, uint8_t* value, uint8_t size)
{
    do
    {
        *value = eeprom_read_byte((unsigned char*)pos);
        pos++;
        value++;
    }while(--size);
}
#define EEPROM_READ_VAR(pos, value) _EEPROM_readData(pos, (uint8_t*)&value, sizeof(value))
//======================================================================================

#define EEPROM_OFFSET 100


// IMPORTANT:  Whenever there are changes made to the variables stored in EEPROM
// in the functions below, also increment the version number. This makes sure that
// the default values are used whenever there is a change to the data, to prevent
// wrong data being written to the variables.
// ALSO:  always make sure the variables in the Store and retrieve sections are in the same order.
#define EEPROM_VERSION "V08"

#ifdef EEPROM_SETTINGS

#ifdef POWER_LOSS_SAVE_TO_EEPROM

bool b_PRE_Write_PLR_Done = false;
void EEPROM_PRE_Write_PLR(uint32_t lFPos, int iBPos, int i_dual_x_carriage_mode, float f_duplicate_extruder_x_offset, float f_feedrate)
{
	if(lFPos > 2048 && !b_PRE_Write_PLR_Done){
		int i=350;
		EEPROM_WRITE_VAR(i,iBPos);  
		EEPROM_WRITE_VAR(i,i_dual_x_carriage_mode);  
		EEPROM_WRITE_VAR(i,f_duplicate_extruder_x_offset);  
		EEPROM_WRITE_VAR(i,f_feedrate);  
		b_PRE_Write_PLR_Done = true;
	}else if(lFPos == 0){
		b_PRE_Write_PLR_Done = false;
	}
}

void EEPROM_Write_PLR(uint32_t lFPos, int iTPos, int iTPos1, int iT01, float fZPos, float fEPos)
{
    int i=300;
    EEPROM_WRITE_VAR(i,lFPos);
	if(lFPos > 2048){
		EEPROM_WRITE_VAR(i,iTPos); 
		EEPROM_WRITE_VAR(i,iTPos1);
		EEPROM_WRITE_VAR(i,iT01);
		EEPROM_WRITE_VAR(i,fZPos);
		EEPROM_WRITE_VAR(i,fEPos);
	}
}

uint32_t EEPROM_Read_PLR_0()
{	
	uint32_t lFPos;

	int i=300;
	EEPROM_READ_VAR(i,lFPos);
	return lFPos;
}

String EEPROM_Read_PLR()
{
	String strRet = "";
	uint32_t lFPos;
	int iTPos;
	int iTPos1;
	int iFanPos = 255;
	int iT01;
	int iBPos;
	float fZPos;
	float fEPos;
	float fXPos = 0.0;
	float fYPos = 0.0;
	int i_dual_x_carriage_mode;
	float f_duplicate_extruder_x_offset;
	float f_feedrate;
	
	int i=300;
	EEPROM_READ_VAR(i,lFPos);
	EEPROM_READ_VAR(i,iTPos);
	EEPROM_READ_VAR(i,iTPos1);
	EEPROM_READ_VAR(i,iT01);
	EEPROM_READ_VAR(i,fZPos);
	EEPROM_READ_VAR(i,fEPos);
	
	i = 350;
	EEPROM_READ_VAR(i,iBPos);
	EEPROM_READ_VAR(i,i_dual_x_carriage_mode);
	EEPROM_READ_VAR(i,f_duplicate_extruder_x_offset);
	EEPROM_READ_VAR(i,f_feedrate);
	//               1FPos                2TPos                3TPos1				4T01                   5ZPos                 6EPos                  7FanPos                   8XPos                 9YPos                 10BPos                     11DXCM                                     12DEXO                                      13Feedrate
	strRet = String(lFPos) + "|" + String(iTPos) + "|" + String(iTPos1)  + "|" + String(iT01)  + "|" + String(fZPos)  + "|" + String(fEPos)  + "|" + String(iFanPos)  + "|" + String(fXPos)  + "|" + String(fYPos)  + "|" + String(iBPos)  + "|" + String(i_dual_x_carriage_mode)  + "|" + String(f_duplicate_extruder_x_offset)  + "|" + String(f_feedrate);
	return strRet;
}

#endif //POWER_LOSS_SAVE_TO_EEPROM

void Config_StoreSettings() 
{
    char ver[4]= "000";
    int i=EEPROM_OFFSET;
    EEPROM_WRITE_VAR(i,ver); // invalidate data first 
    EEPROM_WRITE_VAR(i,axis_steps_per_unit);  
    EEPROM_WRITE_VAR(i,max_feedrate);  
    EEPROM_WRITE_VAR(i,max_acceleration_units_per_sq_second);
    EEPROM_WRITE_VAR(i,acceleration);
    EEPROM_WRITE_VAR(i,retract_acceleration);
    EEPROM_WRITE_VAR(i,minimumfeedrate);
    EEPROM_WRITE_VAR(i,mintravelfeedrate);
    EEPROM_WRITE_VAR(i,minsegmenttime);
    EEPROM_WRITE_VAR(i,max_xy_jerk);
    EEPROM_WRITE_VAR(i,max_z_jerk);
    EEPROM_WRITE_VAR(i,max_e_jerk);
    EEPROM_WRITE_VAR(i,add_homeing);

	/*
//#ifndef ULTIPANEL
    int plaPreheatHotendTemp = PLA_PREHEAT_HOTEND_TEMP;
	int	plaPreheatHPBTemp = PLA_PREHEAT_HPB_TEMP; 
	int	plaPreheatFanSpeed = PLA_PREHEAT_FAN_SPEED;
    int absPreheatHotendTemp = ABS_PREHEAT_HOTEND_TEMP;
	int absPreheatHPBTemp = ABS_PREHEAT_HPB_TEMP;
	int absPreheatFanSpeed = ABS_PREHEAT_FAN_SPEED;
//#endif
	*/
    EEPROM_WRITE_VAR(i,plaPreheatHotendTemp);
    EEPROM_WRITE_VAR(i,plaPreheatHPBTemp);
    EEPROM_WRITE_VAR(i,plaPreheatFanSpeed);
    EEPROM_WRITE_VAR(i,absPreheatHotendTemp);
    EEPROM_WRITE_VAR(i,absPreheatHPBTemp);
    EEPROM_WRITE_VAR(i,absPreheatFanSpeed);

#ifdef PIDTEMP
    EEPROM_WRITE_VAR(i,Kp);
    EEPROM_WRITE_VAR(i,Ki);
    EEPROM_WRITE_VAR(i,Kd);
#else
    float dummy = 3000.0f;
    EEPROM_WRITE_VAR(i,dummy);
    dummy = 0.0f;
    EEPROM_WRITE_VAR(i,dummy);
    EEPROM_WRITE_VAR(i,dummy);
#endif

#ifdef CONFIG_XYZ
    EEPROM_WRITE_VAR(i, zyf_X2_MAX_POS);					//By Zyf
#endif

#ifdef CONFIG_E2_OFFSET
    EEPROM_WRITE_VAR(i, zyf_Y2_OFFSET);					//By Zyf
    EEPROM_WRITE_VAR(i, zyf_Z2_OFFSET);					//By Zyf
#endif

#ifdef FAN2_CONTROL
    EEPROM_WRITE_VAR(i, zyf_FAN2_VALUE);					//By Zyf    
    EEPROM_WRITE_VAR(i, zyf_FAN2_START_TEMP);					//By Zyf    
#endif

#ifdef TENLOG_CONTROLLER
    EEPROM_WRITE_VAR(i, languageID);					//By Zyf    
#endif

#ifdef POWER_LOSS_RECOVERY
    EEPROM_WRITE_VAR(i, zyf_AUTO_OFF);					//By Zyf    
#endif

#ifdef TENLOG_CONTROLLER
    EEPROM_WRITE_VAR(i, zyf_SLEEP_TIME);			//By Zyf    
#endif


#ifndef DOGLCD
    int lcd_contrast = 32;
#endif
    EEPROM_WRITE_VAR(i,lcd_contrast);

    char ver2[4]=EEPROM_VERSION;
    i=EEPROM_OFFSET;
    EEPROM_WRITE_VAR(i,ver2); // validate data
    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Settings Stored");
}
#endif //EEPROM_SETTINGS

#ifdef EEPROM_CHITCHAT

void Config_PrintSettings()
{  
    // Always have this function, even with EEPROM_SETTINGS disabled, the current values will be shown
    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Steps per unit:");
    SERIAL_ECHO_START;
    SERIAL_ECHOPAIR("  M92 X",axis_steps_per_unit[0]);
    SERIAL_ECHOPAIR(" Y",axis_steps_per_unit[1]);
    SERIAL_ECHOPAIR(" Z",axis_steps_per_unit[2]);
    SERIAL_ECHOPAIR(" E",axis_steps_per_unit[3]);
    SERIAL_ECHOLN("");

    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Maximum feedrates (mm/s):");
    SERIAL_ECHO_START;
    SERIAL_ECHOPAIR("  M203 X",max_feedrate[0]);
    SERIAL_ECHOPAIR(" Y",max_feedrate[1] ); 
    SERIAL_ECHOPAIR(" Z", max_feedrate[2] ); 
    SERIAL_ECHOPAIR(" E", max_feedrate[3]);
    SERIAL_ECHOLN("");

    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Maximum Acceleration (mm/s2):");
    SERIAL_ECHO_START;
    SERIAL_ECHOPAIR("  M201 X" ,max_acceleration_units_per_sq_second[0] ); 
    SERIAL_ECHOPAIR(" Y" , max_acceleration_units_per_sq_second[1] ); 
    SERIAL_ECHOPAIR(" Z" ,max_acceleration_units_per_sq_second[2] );
    SERIAL_ECHOPAIR(" E" ,max_acceleration_units_per_sq_second[3]);
    SERIAL_ECHOLN("");
    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Acceleration: S=acceleration, T=retract acceleration");
    SERIAL_ECHO_START;
    SERIAL_ECHOPAIR("  M204 S",acceleration ); 
    SERIAL_ECHOPAIR(" T" ,retract_acceleration);
    SERIAL_ECHOLN("");

    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Advanced variables: S=Min feedrate (mm/s), T=Min travel feedrate (mm/s), B=minimum segment time (ms), X=maximum XY jerk (mm/s),  Z=maximum Z jerk (mm/s),  E=maximum E jerk (mm/s)");
    SERIAL_ECHO_START;
    SERIAL_ECHOPAIR("  M205 S",minimumfeedrate ); 
    SERIAL_ECHOPAIR(" T" ,mintravelfeedrate ); 
    SERIAL_ECHOPAIR(" B" ,minsegmenttime ); 
    SERIAL_ECHOPAIR(" X" ,max_xy_jerk ); 
    SERIAL_ECHOPAIR(" Z" ,max_z_jerk);
    SERIAL_ECHOPAIR(" E" ,max_e_jerk);
    SERIAL_ECHOLN(""); 

    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Home offset (mm):");
    SERIAL_ECHO_START;
    SERIAL_ECHOPAIR("  M206 X",add_homeing[0] );
    SERIAL_ECHOPAIR(" Y" ,add_homeing[1] );
    SERIAL_ECHOPAIR(" Z" ,add_homeing[2] );
    SERIAL_ECHOLN("");
#ifdef PIDTEMP
    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("PID settings:");
    SERIAL_ECHO_START;
    SERIAL_ECHOPAIR("   M301 P",Kp); 
    SERIAL_ECHOPAIR(" I" ,unscalePID_i(Ki)); 
    SERIAL_ECHOPAIR(" D" ,unscalePID_d(Kd));
    SERIAL_ECHOLN(""); 
#endif
#ifdef CONFIG_XYZ
    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Secondery Max Pos:");
    SERIAL_ECHOPAIR("X2:", zyf_X2_MAX_POS ); 
    SERIAL_ECHOLN(""); 
#endif
#ifdef CONFIG_E2_OFFSET
    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Extender Offset:");
    SERIAL_ECHOPAIR("Y2:", zyf_Y2_OFFSET ); 
    SERIAL_ECHOLN(""); 
    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Extender Offset:");
    SERIAL_ECHOPAIR("Z2:", zyf_Z2_OFFSET ); 
    SERIAL_ECHOLN(""); 
#endif

#ifdef POWER_LOSS_RECOVERY
    ZYF_DEBUG_PRINT("Auto Power Off:");
    ZYF_DEBUG_PRINT_LN(zyf_AUTO_OFF);					//By Zyf    
#endif
} 
#endif


#ifdef EEPROM_SETTINGS
void Config_RetrieveSettings()
{
    int i=EEPROM_OFFSET;
    char stored_ver[4];
    char ver[4]=EEPROM_VERSION;
    EEPROM_READ_VAR(i,stored_ver); //read stored version
    //  SERIAL_ECHOLN("Version: [" << ver << "] Stored version: [" << stored_ver << "]");
    if (strncmp(ver,stored_ver,3) == 0)
    {
        // version number match
        EEPROM_READ_VAR(i,axis_steps_per_unit);  
        EEPROM_READ_VAR(i,max_feedrate);  
        EEPROM_READ_VAR(i,max_acceleration_units_per_sq_second);

        // steps per sq second need to be updated to agree with the units per sq second (as they are what is used in the planner)
        reset_acceleration_rates();

        EEPROM_READ_VAR(i,acceleration);
        EEPROM_READ_VAR(i,retract_acceleration);
        EEPROM_READ_VAR(i,minimumfeedrate);
        EEPROM_READ_VAR(i,mintravelfeedrate);
        EEPROM_READ_VAR(i,minsegmenttime);
        EEPROM_READ_VAR(i,max_xy_jerk);
        EEPROM_READ_VAR(i,max_z_jerk);
        EEPROM_READ_VAR(i,max_e_jerk);
        EEPROM_READ_VAR(i,add_homeing);
		/*
//#ifndef ULTIPANEL
		
        int plaPreheatHotendTemp, plaPreheatHPBTemp, plaPreheatFanSpeed;
        int absPreheatHotendTemp, absPreheatHPBTemp, absPreheatFanSpeed;
//#endif
		*/
        EEPROM_READ_VAR(i,plaPreheatHotendTemp);
        EEPROM_READ_VAR(i,plaPreheatHPBTemp);
        EEPROM_READ_VAR(i,plaPreheatFanSpeed);
        EEPROM_READ_VAR(i,absPreheatHotendTemp);
        EEPROM_READ_VAR(i,absPreheatHPBTemp);
        EEPROM_READ_VAR(i,absPreheatFanSpeed);
#ifndef PIDTEMP
        float Kp,Ki,Kd;
#endif
        // do not need to scale PID values as the values in EEPROM are already scaled		
        EEPROM_READ_VAR(i,Kp);
        EEPROM_READ_VAR(i,Ki);
        EEPROM_READ_VAR(i,Kd);

#ifdef CONFIG_XYZ
        EEPROM_READ_VAR(i, zyf_X2_MAX_POS);		// by zyf  
#endif

#ifdef CONFIG_E2_OFFSET
        EEPROM_READ_VAR(i, zyf_Y2_OFFSET);		// by zyf  
        EEPROM_READ_VAR(i, zyf_Z2_OFFSET);		// by zyf  
#endif

#ifdef FAN2_CONTROL
        EEPROM_READ_VAR(i, zyf_FAN2_VALUE);		// by zyf          
        EEPROM_READ_VAR(i, zyf_FAN2_START_TEMP);		// by zyf          
#endif

#ifdef TENLOG_CONTROLLER
       EEPROM_READ_VAR(i, languageID);		// by zyf                  
#endif

#ifdef POWER_LOSS_RECOVERY
    EEPROM_READ_VAR(i, zyf_AUTO_OFF);		// by zyf      
#endif

#ifdef TENLOG_CONTROLLER
       EEPROM_READ_VAR(i, zyf_SLEEP_TIME);		// by zyf                  
#endif

#ifndef DOGLCD
        int lcd_contrast;
#endif
        EEPROM_READ_VAR(i,lcd_contrast);

        // Call updatePID (similar to when we have processed M301)
        updatePID();

        SERIAL_ECHO_START;
        SERIAL_ECHOLNPGM("Stored settings retrieved");
    }
    else
    {
        Config_ResetDefault();
    }
    Config_PrintSettings();
}
#endif

void Config_ResetDefault()
{
    float tmp1[]=DEFAULT_AXIS_STEPS_PER_UNIT;
    float tmp2[]=DEFAULT_MAX_FEEDRATE;
    long tmp3[]=DEFAULT_MAX_ACCELERATION;
    for (short i=0;i<4;i++) 
    {
        axis_steps_per_unit[i]=tmp1[i];  
        max_feedrate[i]=tmp2[i];  
        max_acceleration_units_per_sq_second[i]=tmp3[i];
    }


    // steps per sq second need to be updated to agree with the units per sq second
    reset_acceleration_rates();

    acceleration=DEFAULT_ACCELERATION;
    retract_acceleration=DEFAULT_RETRACT_ACCELERATION;
    minimumfeedrate=DEFAULT_MINIMUMFEEDRATE;
    minsegmenttime=DEFAULT_MINSEGMENTTIME;       
    mintravelfeedrate=DEFAULT_MINTRAVELFEEDRATE;
    max_xy_jerk=DEFAULT_XYJERK;
    max_z_jerk=DEFAULT_ZJERK;
    max_e_jerk=DEFAULT_EJERK;
    add_homeing[0] = add_homeing[1] = add_homeing[2] = 0;
//#ifdef ULTIPANEL
    plaPreheatHotendTemp = PLA_PREHEAT_HOTEND_TEMP;
    plaPreheatHPBTemp = PLA_PREHEAT_HPB_TEMP;
    plaPreheatFanSpeed = PLA_PREHEAT_FAN_SPEED;
    absPreheatHotendTemp = ABS_PREHEAT_HOTEND_TEMP;
    absPreheatHPBTemp = ABS_PREHEAT_HPB_TEMP;
    absPreheatFanSpeed = ABS_PREHEAT_FAN_SPEED;
//#endif
#ifdef DOGLCD
    lcd_contrast = DEFAULT_LCD_CONTRAST;
#endif
#ifdef PIDTEMP
    Kp = DEFAULT_Kp;
    Ki = scalePID_i(DEFAULT_Ki);
    Kd = scalePID_d(DEFAULT_Kd);

    // call updatePID (similar to when we have processed M301)
    updatePID();

#ifdef PID_ADD_EXTRUSION_RATE
    Kc = DEFAULT_Kc;
#endif//PID_ADD_EXTRUSION_RATE
#endif//PIDTEMP

#ifdef CONFIG_XYZ
    zyf_X2_MAX_POS = X2_MAX_POS;
#endif

#ifdef CONFIG_E2_OFFSET
    //#ifdef TENLOG_CONTROLLER
    //zyf_Y2_OFFSET = 0.0;
    //zyf_Z2_OFFSET = 0.0;
    //#else
    zyf_Y2_OFFSET = 4.5;
    zyf_Z2_OFFSET = 2.0;
    //#endif
#endif

#ifdef FAN2_CONTROL
    zyf_FAN2_VALUE = 255;
    zyf_FAN2_START_TEMP = 80;
#endif

#ifdef TENLOG_CONTROLLER
    languageID=0;
    zyf_SLEEP_TIME = 0;
#endif

#ifdef POWER_LOSS_RECOVERY
    zyf_AUTO_OFF=0;
#endif

SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Hardcoded Default Settings Loaded");
    Config_StoreSettings();     //By Zyf save after reset  
}
