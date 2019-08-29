#ifndef CONFIGURATION_ZYF_H
#define CONFIGURATION_ZYF_H
 
//#define ZYF_DEBUG

#define P2P1 //Yes=P No=S

//#define CUSTOM_NONE
#define CUSTOM_TENLOG
//#define CUSTOM_HICTOP
//#define CUSTOM_CAPY

#define TENLOG_CONTROLLER

////////////////////////
//Raise up z when Pause;		//By ZYF
#define PAUSE_RAISE_Z			

//#define ZYF_SIZE_220 
#define ZYF_SIZE_300
//#define ZYF_SIZE_400
//#define ZYF_SIZE_500 
//#define ZYF_SIZE_600

#define X_NOZZLE_WIDTH 52		//By ZYF
#define DUAL_X_CARRIAGE			//By Zyf

// The pullups are needed if you directly connect a mechanical endswitch between the signal and ground pins.
const bool X_ENDSTOPS_INVERTING = true; // set to true to invert the logic of the endstops.
const bool Y_ENDSTOPS_INVERTING = false; // set to true to invert the logic of the endstops. HIGH
//const bool Y_ENDSTOPS_INVERTING = true; // set to true to invert the logic of the endstops.
//const bool Z_ENDSTOPS_INVERTING = true; // set to true to invert the logic of the endstops.

#define INVERT_X_DIR true    // for Mendel set to false, for Orca set to true
#define INVERT_Z_DIR true     // for Mendel set to false, for Orca set to true
#define INVERT_E0_DIR false   // for direct drive extruder v9 set to true, for geared extruder set to false
#define INVERT_E1_DIR true  // for direct drive extruder v9 set to true, for geared extruder set to false
//#define INVERT_E2_DIR false   // for direct drive extruder v9 set to true, for geared extruder set to false

//#define DEFAULT_AXIS_STEPS_PER_UNIT   {80,80,800,94.4}  //755 94.4 default steps per unit for Ultimaker
#define DEFAULT_AXIS_STEPS_PER_UNIT   {80,80,792,92.6}  //755 94.4 default steps per unit for Ultimaker

#define FAN2_CONTROL
#ifdef FAN2_CONTROL
	#define FAN2_PIN    5//5//63
#endif

#define SHOW_BOOTSCREEN_2004

#define ZYF_DUAL_Z
#ifdef ZYF_DUAL_Z
	#define INVERT_Y_DIR false    // for Mendel set to true, for Orca set to false
	const bool Z_ENDSTOPS_INVERTING = true; // set to true to invert the logic of the endstops.	LOW
#else
	#define INVERT_Y_DIR true    // for Mendel set to true, for Orca set to false
	const bool Z_ENDSTOPS_INVERTING = false; // set to true to invert the logic of the endstops. HIGH
#endif

#ifdef ZYF_SIZE_220
    #define DEFAULT_DUPLICATION_X_OFFSET 115
    #define X_MAX_POS 225.0
    #define Y_MAX_POS 225.0
    #ifdef P2P1
        #define Z_MAX_POS 250.0
    #else
        #define Z_MAX_POS 250.0    
    #endif
    #define X2_MAX_POS 280.0    // set maximum to the distance between toolheads when both heads are homed 
#endif

#ifdef ZYF_SIZE_300
    #define DEFAULT_DUPLICATION_X_OFFSET 155
    #define X_MAX_POS 305.0
    #define Y_MAX_POS 320.0
    #ifdef P2P1
        #define Z_MAX_POS 360.0
    #else
        #define Z_MAX_POS 410.0    
    #endif
    #define X2_MAX_POS 360.0    // set maximum to the distance between toolheads when both heads are homed 
#endif

#ifdef ZYF_SIZE_400
    #define DEFAULT_DUPLICATION_X_OFFSET 205
    #define X_MAX_POS 405.0
    #define Y_MAX_POS 420.0
    #ifdef P2P1
        #define Z_MAX_POS 460.0
    #else
        #define Z_MAX_POS 510.0
    #endif
    #define X2_MAX_POS 460.0    // set maximum to the distance between toolheads when both heads are homed 
#endif

#ifdef ZYF_SIZE_500
    #define DEFAULT_DUPLICATION_X_OFFSET 250
    #define X_MAX_POS 505.0
    #define Y_MAX_POS 520.0
    #ifdef P2P1
        #define Z_MAX_POS 560.0
    #else
        #define Z_MAX_POS 610.0
    #endif
    #define X2_MAX_POS 650.0    // set maximum to the distance between toolheads when both heads are homed 
#endif
 
#ifdef ZYF_SIZE_600
    #define DEFAULT_DUPLICATION_X_OFFSET 300
    #define X_MAX_POS 750.0
    #define Y_MAX_POS 620.0
    #ifdef P2P1
        #define Z_MAX_POS 650.0
    #else
        #define Z_MAX_POS 710.0
    #endif
    #define X2_MAX_POS 750.0    // set maximum to the distance between toolheads when both heads are homed 
#endif

#define X2_MIN_POS 0       // set minimum to ensure second x-carriage doesn't hit the parked first X-carriage
#define X2_HOME_DIR 1     // the second X-carriage always homes to the maximum endstop position
#define X2_HOME_POS X2_MAX_POS // default home position is the maximum carriage position 
#define CONFIG_XYZ						//By zyf
#define CONFIG_E2_OFFSET		//By Zyf
 
#endif //CONFIGURATION_ZYF_H
