#ifndef CONFIGURATION_TL_H
#define CONFIGURATION_TL_H

//Powered by tenlog3dprinter.com
/*
BOF UPDATE LOG 

20191006:   TL_TJC_CONTROLLER done
20191102:   FILAMENT_FAIL_DETECT done
20191120:   POWER_LOSS_RECOVERY done
20191219:   PRINT_FROM_Z_HEIGHT done

20200807:   BEEPER Done
20200814:   Support 2225 driver
20200825:   fix bug: show qr code when start up 
20200915:   Version: 1.0.8
            auto detect plr hardware module.
            add VERSION string on about screen. 
20200909:   Add pt100 temp sensor. //  canceled.
20201009:   D5 ZMax is 610
20201015    Fix bug: DUPLICATION MODE & Mirror Mode E2 not finished the last several gcode lines when sd printing.
            version 1.0.9
20201022    Turn off beeper when SD Printing. 
            delete 2225 driver in hex files.
            add y mechanical switch.
            version 1.0.10
20201110    Bug: when active extruder is T1, extruder crash in duplication mode.
20201202    Bug: pause when printing E2, resume heat E1; fixed.
            DWIN screen controller finished.(QR Code can be switched off)
20201203    ECO mode done
20201204    filament senser can be switch off (if you want).
            Version 1.0.13
20201212    Fix some bugs
20201217    Bug fixed: PLR, begin if E2, wrong Y Offset.
            Version 1.0.14
20201220    Unvisable auto poweroff in setting page if no PLR Module detected(need UI v:1.3.3).
            Use LM393 to detect power loss.
            Version 1.0.15
20201231    Reduce data transmission of DWIN touch screen.
            Version 1.0.16
            DWIN UI Version 1.3.4
20210112    DWIN UI Spanish language enabled.
            Version 1.0.17
            Need DWIN UI V1.3.6 
            Do not support DWIN UI Below 1.3.6
20210128    Multi language enabled (only for DWIN UI), 
            now we support English, Chinese, Spanish, French, German, Italian and Japanses,
            more language is comming soon.
            DWN UI V 1.3.7
            Fix some UI bugs.
            Add DWN Screen saver function
            Need DWIN UI V 1.3.7
            Version 1.0.18
            Disabled beeper for TJC UI
20210313    TJC New UI(like DWIN) V1.2.8 (some function need firmware v1.0.19)
            Fix Some bugs.
            DWIN UI V1.3.8
            Version 1.0.19
20210319    Delete some unsued code.
20210329    Support M117 command (Only in printing page).(TJC UI Need v1.2.9)
            TJC UI V 1.2.9
            Fix some bugs.
            Version 1.0.20
            Show SN at start up.
20210401    TJC UI V1.2.9 R Means Rotate 180 degrees.
20210421    Fix bugs: bugs report by Tenlog Ma at 20200420
            Version 1.0.21
20210422    Upgrade M105 command, add temp of T1.
20210510    Add ELECTROMAGNETIC_VALVE control, Synchronize from E Steppers. 
            Fix bug: axis movement disorder after print finished
20210513    Fix relative_mode bug.
            Accelerate Z axis homing.
            Version 1.0.22
20210527    Solve the abnormal noise of the front fan.
            Improve ELECTROMAGNETIC_VALVE function. 
            Slow down the filament feed in speed.
            V 1.0.23            
20210604    Stop printing immediately when click OK.
            Fix some bugs.
            V 1.0.24
20210616    Change jerk value.
            Release inactive E & X driver
            V 1.0.25
20210706    Remove chinese chareters and conver files to utf-8
            V 1.0.26
20210712    cancel gohome after extruder switch when printing.
            e steps per mm 395 (for bmg extruder).
            v 1.0.27
20210728    nozzle offset test print function ok. need new ui.(TJC 1.2.13 or later, DWN not yet)
            V 1.0.28
20210730    use function F() for touch screen serial commands, saves about 1.5k of RAM.
20210830    Fix some bugs
            V 1.0.29
20210915    Auto detect TJC OR DWIN Touch Screen 
            V 1.0.30
20211012	fix some bugs
			V1.0.31
EOF UPDATE LOG
*/

#define VERSION_STRING "1.0.31"
//#define TL_DEBUG

//#define MODEL_H2P     //TL-Hands2 Pro
#define MODEL_D3P //TL-D3 Pro
//#define MODEL_D4P
//#define MODEL_D5P
//#define MODEL_D6P
//#define MODEL_M3
//#define MODEL_D2P		//TL-Hands2
//#define MODEL_D3S

#define FILAMENT_FAIL_DETECT
#define POWER_LOSS_RECOVERY
#define PRINT_FROM_Z_HEIGHT

//#define DRIVER_2225
#define DRIVER_2208 //Same as 2209
//#define DRIVER_4988

////////////////////////
//Raise up z when Pause;		//By ZYF
#define PAUSE_RAISE_Z

#define TL_DUAL_Z
#define X_NOZZLE_WIDTH 50
#define DUAL_X_CARRIAGE

// The pullups are needed if you directly connect a mechanical endswitch between the signal and ground pins.
const bool X_ENDSTOPS_INVERTING = true;
const bool Y_ENDSTOPS_INVERTING = true; //Y Optical switch
//const bool Y_ENDSTOPS_INVERTING = false;            //Y Mechanical switch

//#define MIX_COLOR_TEST
//#define ELECTROMAGNETIC_VALVE    //evaluation version for Profesor Shen from Jilin University

#if defined(DRIVER_2208) || defined(DRIVER_2225)
#define INVERT_X_DIR false
#define INVERT_Z_DIR false
#define INVERT_E0_DIR true
#define INVERT_E1_DIR false
#elif defined(DRIVER_4988)
#define INVERT_X_DIR true
#define INVERT_Z_DIR true
#define INVERT_E0_DIR false
#define INVERT_E1_DIR true
#endif

#if defined(DRIVER_2225)
#define DEFAULT_AXIS_STEPS_PER_UNIT \
    {                               \
        160, 160, 1600, 184         \
    }
#define DEFAULT_MAX_FEEDRATE \
    {                        \
        30, 30, 2, 12        \
    } // (mm/sec)
#else
#define DEFAULT_AXIS_STEPS_PER_UNIT \
    {                             \
        80, 80, 800, 395          \
    }
#define DEFAULT_MAX_FEEDRATE \
    {                        \
        70, 70, 6, 25        \
    } // (mm/sec)
#endif


#if defined(MODEL_D2P)
#define FW_STR "HANDS2"
#define TL_SIZE_220
#define P2P1
#elif defined(MODEL_H2P)
#define FW_STR "HANDS2 Pro"
#define TL_SIZE_235
#define P2P1
#elif defined(MODEL_D3P)
#define FW_STR "D3P"
#define TL_SIZE_300
#define P2P1
#elif defined(MODEL_D3S)
#define FW_STR "D3S"
#define TL_SIZE_300
#elif defined(MODEL_D4P)
#define FW_STR "D4P"
#define TL_SIZE_400
#define P2P1
#elif defined(MODEL_D5P)
#define FW_STR "D5P"
#define TL_SIZE_500
#define P2P1
#elif defined(MODEL_D6P)
#define FW_STR "D6P"
#define TL_SIZE_600
#define P2P1
#elif defined(MODEL_M3)
#define FW_STR "M3"
#define TL_SIZE_250
#define P2P1
#endif

#define X_MIN_POS -50
#define X2_MIN_POS 0           // set minimum to ensure second x-carriage doesn't hit the parked first X-carriage

#ifdef TL_SIZE_220
#define DEFAULT_DUPLICATION_X_OFFSET 115
#define X_MAX_POS 220.0
#define Y_MAX_POS 225.0
#ifdef P2P1
#define Z_MAX_POS 260.0
#else
#define Z_MAX_POS 260.0
#endif
#define X2_MAX_POS 264.0 // set maximum to the distance between toolheads when both heads are homed
#endif

#ifdef TL_SIZE_250
	#undef TL_DUAL_Z 
	#undef X_MIN_POS 
	#undef X2_MIN_POS           
	
	#define X_MIN_POS 0
	#define X2_MIN_POS 50        

	#define DEFAULT_DUPLICATION_X_OFFSET 175
	#define X_MAX_POS 300.0
	#define Y_MAX_POS 200.0
	#ifdef P2P1
		#define Z_MAX_POS 250.0
	#else
		#define Z_MAX_POS 250.0
	#endif
	#define X2_MAX_POS 354.0 // set maximum to the distance between toolheads when both heads are homed
#endif

#ifdef TL_SIZE_235
#define DEFAULT_DUPLICATION_X_OFFSET 167
#define X_MAX_POS 235.0
#define Y_MAX_POS 240.0
#ifdef P2P1
#define Z_MAX_POS 260.0
#else
#define Z_MAX_POS 260.0
#endif
#define X2_MAX_POS 279.0 // set maximum to the distance between toolheads when both heads are homed
#endif

#ifdef TL_SIZE_300
#define DEFAULT_DUPLICATION_X_OFFSET 155
#define X_MAX_POS 305.0
#define Y_MAX_POS 320.0
#ifdef P2P1
#define Z_MAX_POS 360.0
#else
#define Z_MAX_POS 410.0
#endif
#define X2_MAX_POS 359.0 // set maximum to the distance between toolheads when both heads are homed
#endif

#ifdef TL_SIZE_400
#define DEFAULT_DUPLICATION_X_OFFSET 205
#define X_MAX_POS 405.0
#define Y_MAX_POS 420.0
#ifdef P2P1
#define Z_MAX_POS 410.0
#else
#define Z_MAX_POS 410.0
#endif
#define X2_MAX_POS 454.0 // set maximum to the distance between toolheads when both heads are homed
#endif

#ifdef TL_SIZE_500
#define DEFAULT_DUPLICATION_X_OFFSET 255
#define X_MAX_POS 505.0
#define Y_MAX_POS 520.0
#ifdef P2P1
#define Z_MAX_POS 610.0
#else
#define Z_MAX_POS 610.0
#endif
#define X2_MAX_POS 554.0 // set maximum to the distance between toolheads when both heads are homed
#endif

#ifdef TL_SIZE_600
#define DEFAULT_DUPLICATION_X_OFFSET 305
#define X_MAX_POS 605.0
#define Y_MAX_POS 620.0
#ifdef P2P1
#define Z_MAX_POS 610.0
#else
#define Z_MAX_POS 610.0
#endif
#define X2_MAX_POS 654.0 // set maximum to the distance between toolheads when both heads are homed
#endif

#define FAN2_CONTROL
#ifdef FAN2_CONTROL
#define FAN2_PIN 5
#endif

#ifdef TL_DUAL_Z
	#if defined(DRIVER_2208) || defined(DRIVER_2225)
		#define INVERT_Y_DIR true
	#else
		#define INVERT_Y_DIR false
	#endif
	const bool Z_ENDSTOPS_INVERTING = true;
#else
	#define INVERT_Y_DIR true
	const bool Z_ENDSTOPS_INVERTING = true;
#endif

#ifdef POWER_LOSS_RECOVERY
#define HAS_PLR_MODULE
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
#define FILAMENT_FAIL_DETECT_PIN 15
#define FILAMENT_FAIL_DETECT_TRIGGER LOW
#endif

#define DEFAULT_MAX_ACCELERATION \
    {                            \
        500, 500, 100, 1000      \
    }                            // 800 800 160 1600 500 500 100 1000 X, Y, Z, E maximum start speed for accelerated moves. E default values are good for skeinforge 40+, for older versions raise them a lot.

#define DEFAULT_ACCELERATION 500 // X, Y, Z and E max acceleration in mm/s^2 for printing moves
#define DEFAULT_RETRACT_ACCELERATION 500 // X, Y, Z and E max acceleration in mm/s^2 for retracts

#define X2_HOME_DIR 1          // the second X-carriage always homes to the maximum endstop position
#define X2_HOME_POS X2_MAX_POS // default home position is the maximum carriage position
#define CONFIG_TL              //By zyf
#define CONFIG_E2_OFFSET       //By Zyf

//#define ENGRAVE
#ifdef ENGRAVE
#define ENGRAVE_ON 0
#define ENGRAVE_OFF 1
#define ENGRAVE_PIN 37
#endif

#endif //CONFIGURATION_TL_H