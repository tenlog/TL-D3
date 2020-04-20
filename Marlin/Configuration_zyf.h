#ifndef CONFIGURATION_ZYF_H
#define CONFIGURATION_ZYF_H
 
//#define ZYF_DEBUG

#define TENLOG_CONTROLLER
#define FILAMENT_FAIL_DETECT //Done 20191102
#define POWER_LOSS_RECOVERY //It's OK need hardware support. Done 20191120
#define PRINT_FROM_Z_HEIGHT  // Done 20191219

////////////////////////
//Raise up z when Pause;		//By ZYF
#define PAUSE_RAISE_Z

//#define MODEL_D2P		//TL-Hands 2 
//#define MODEL_D3P		//TL-D3 Pro
//#define MODEL_D3S 
//#define MODEL_D4P 
//#define MODEL_D5P  
#define MODEL_D6P 

#define X_NOZZLE_WIDTH 50		//By ZYF 
#define DUAL_X_CARRIAGE			//By Zyf

// The pullups are needed if you directly connect a mechanical endswitch between the signal and ground pins.
const bool X_ENDSTOPS_INVERTING = true; // set to true to invert the logic of the endstops.
const bool Y_ENDSTOPS_INVERTING = false; // set to true to invert the logic of the endstops. HIGH

#define INVERT_X_DIR true    // for Mendel set to false, for Orca set to true
#define INVERT_Z_DIR true     // for Mendel set to false, for Orca set to true
#define INVERT_E0_DIR false   // for direct drive extruder v9 set to true, for geared extruder set to false
#define INVERT_E1_DIR true  // for direct drive extruder v9 set to true, for geared extruder set to false

//#define DEFAULT_AXIS_STEPS_PER_UNIT   {80,80,800,94.4}  //755 94.4 default steps per unit for Ultimaker
#define DEFAULT_AXIS_STEPS_PER_UNIT   {80,80,792,92.6}  //755 94.4 default steps per unit for Ultimaker
#define DEFAULT_MAX_ACCELERATION {500, 500, 100, 1000}  // 800 800 160 1600 500 500 100 1000 X, Y, Z, E maximum start speed for accelerated moves. E default values are good for skeinforge 40+, for older versions raise them a lot.
#define DEFAULT_MAX_FEEDRATE          {120, 80, 4, 37}    // (mm/pul) 80 80 3 25
#define DEFAULT_ACCELERATION 500    // X, Y, Z and E max acceleration in mm/s^2 for printing moves
#define DEFAULT_RETRACT_ACCELERATION 500   // X, Y, Z and E max acceleration in mm/s^2 for retracts


#define FAN2_CONTROL
#ifdef FAN2_CONTROL
	#define FAN2_PIN    5
#endif

#define ZYF_DUAL_Z
#ifdef ZYF_DUAL_Z
	#define INVERT_Y_DIR false   
	const bool Z_ENDSTOPS_INVERTING = true;
#else
	#define INVERT_Y_DIR true  
	const bool Z_ENDSTOPS_INVERTING = false;
#endif

#ifdef POWER_LOSS_RECOVERY
	#define POWER_LOSS_SAVE_TO_EEPROM
	#define POWER_LOSS_TRIGGER_BY_PIN
	
	#if !defined(POWER_LOSS_TRIGGER_BY_PIN) 
		#define POWER_LOSS_TRIGGER_BY_Z_LEVEL
		#if !defined(POWER_LOSS_TRIGGER_BY_Z_LEVEL)
			#define POWER_LOSS_TRIGGER_BY_E_COUNT
			#ifdef POWER_LOSS_TRIGGER_BY_E_COUNT
				#define POWER_LOSS_E_COUNT 100
			#endif
		#endif
	#endif

	#ifndef POWER_LOSS_SAVE_TO_EEPROM
		#define POWER_LOSS_SAVE_TO_SDCARD
	#endif

	#if !defined(POWER_LOSS_TRIGGER_BY_PIN) //prevent eeprom damage 
		#undef POWER_LOSS_SAVE_TO_EEPROM
		#define POWER_LOSS_SAVE_TO_SDCARD
	#endif
#endif

#ifdef FILAMENT_FAIL_DETECT
	#define FILAMENT_FAIL_DETECT_PIN		15	
	#define FILAMENT_FAIL_DETECT_TRIGGER	LOW
#endif

#if defined (MODEL_D2P)
	#define ZYF_SIZE_220
	#define P2P1
#elif defined(MODEL_D3P)
	#define ZYF_SIZE_300
	#define P2P1
#elif defined(MODEL_D3S)
	#define ZYF_SIZE_300
#elif defined(MODEL_D4P)
	#define ZYF_SIZE_400
	#define P2P1
#elif defined(MODEL_D5P)
	#define ZYF_SIZE_500
	#define P2P1
#elif defined(MODEL_D6P)
	#define ZYF_SIZE_600
	#define P2P1
#endif

#ifdef ZYF_SIZE_220
    #define DEFAULT_DUPLICATION_X_OFFSET 115
    #define X_MAX_POS 220.0
    #define Y_MAX_POS 225.0
    #ifdef P2P1
        #define Z_MAX_POS 260.0
    #else
        #define Z_MAX_POS 260.0    
    #endif
    #define X2_MAX_POS 264.0    // set maximum to the distance between toolheads when both heads are homed 
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
    #define X2_MAX_POS 354.0    // set maximum to the distance between toolheads when both heads are homed 
#endif

#ifdef ZYF_SIZE_400
    #define DEFAULT_DUPLICATION_X_OFFSET 205
    #define X_MAX_POS 405.0
    #define Y_MAX_POS 420.0
    #ifdef P2P1
        #define Z_MAX_POS 410.0
    #else
        #define Z_MAX_POS 410.0
    #endif
    #define X2_MAX_POS 454.0    // set maximum to the distance between toolheads when both heads are homed 
#endif

#ifdef ZYF_SIZE_500
    #define DEFAULT_DUPLICATION_X_OFFSET 255
    #define X_MAX_POS 505.0
    #define Y_MAX_POS 520.0
    #ifdef P2P1
        #define Z_MAX_POS 510.0
    #else
        #define Z_MAX_POS 510.0
    #endif
    #define X2_MAX_POS 554.0    // set maximum to the distance between toolheads when both heads are homed 
#endif
 
#ifdef ZYF_SIZE_600
    #define DEFAULT_DUPLICATION_X_OFFSET 305
    #define X_MAX_POS 605.0
    #define Y_MAX_POS 620.0
    #ifdef P2P1
        #define Z_MAX_POS 610.0
    #else
        #define Z_MAX_POS 610.0
    #endif
    #define X2_MAX_POS 654.0    // set maximum to the distance between toolheads when both heads are homed 
#endif

#define X2_MIN_POS 0    // set minimum to ensure second x-carriage doesn't hit the parked first X-carriage
#define X2_HOME_DIR 1   // the second X-carriage always homes to the maximum endstop position
#define X2_HOME_POS X2_MAX_POS // default home position is the maximum carriage position 
#define CONFIG_XYZ		//By zyf
#define CONFIG_E2_OFFSET		//By Zyf
 
//#define ENGRAVE
#ifdef ENGRAVE
    #define ENGRAVE_ON 0
    #define ENGRAVE_OFF 1
    #define ENGRAVE_PIN 37
#endif


#endif //CONFIGURATION_ZYF_H