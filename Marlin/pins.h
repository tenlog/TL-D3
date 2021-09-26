#ifndef PINS_H
#define PINS_H

#define X_MS1_PIN -1
#define X_MS2_PIN -1
#define Y_MS1_PIN -1
#define Y_MS2_PIN -1
#define Z_MS1_PIN -1
#define Z_MS2_PIN -1
#define E0_MS1_PIN -1
#define E0_MS2_PIN -1
#define E1_MS1_PIN -1
#define E1_MS2_PIN -1
#define DIGIPOTSS_PIN -1

/****************************************************************************************
* Arduino Mega pin assignment
*
****************************************************************************************/
#define KNOWN_BOARD 1

//////////////////FIX THIS//////////////
#ifndef __AVR_ATmega1280__
	#ifndef __AVR_ATmega2560__
		#error Oops!  Make sure you have 'Arduino Mega' selected from the 'Tools -> Boards' menu.
	#endif
#endif

// uncomment one of the following lines for RAMPS v1.3 or v1.0, comment both for v1.2 or 1.1
// #define RAMPS_V_1_3
// #define RAMPS_V_1_0

#define LARGE_FLASH true

#define X_STEP_PIN 54
#define X_DIR_PIN 55
#define X_ENABLE_PIN 38
#define X_MIN_PIN 3
#define X_MAX_PIN 2 //Ori 2	By zyf

#define Y_STEP_PIN 60
#define Y_DIR_PIN 61
#define Y_ENABLE_PIN 56
#define Y_MIN_PIN 14 //Ori 14	by zyf
#define Y_MAX_PIN -1 //15

#define Z_STEP_PIN 46
#define Z_DIR_PIN 48
#define Z_ENABLE_PIN 62
#define Z_MIN_PIN 18

//#define Z_MAX_PIN          19

#ifdef TL_DUAL_Z //By Zyf
#define Z2_STEP_PIN 65
#define Z2_DIR_PIN 66
#define Z2_ENABLE_PIN 64
#define Z2_MIN_PIN 19
#endif

#ifdef P2P1
#define E1_STEP_PIN 26
#define E1_DIR_PIN 28
#define E1_ENABLE_PIN 24

#define E0_STEP_PIN 57
#define E0_DIR_PIN 58
#define E0_ENABLE_PIN 59
#else
#define E0_STEP_PIN 26
#define E0_DIR_PIN 28
#define E0_ENABLE_PIN 24

#define E1_STEP_PIN 57
#define E1_DIR_PIN 58
#define E1_ENABLE_PIN 59
#endif

#define SDPOWER -1
#define SDSS 53
#define LED_PIN 13

#define FAN_PIN 9    // (Sprinter config)
#define PS_ON_PIN 40 //zyf 40		//PF1

#if defined(POWER_LOSS_RECOVERY)
	#ifdef HAS_PLR_MODULE
		#define POWER_LOSS_DETECT_PIN 32 //zyf 32		//PF2
	#else
		#define POWER_LOSS_DETECT_PIN 32
	#endif
#else
	#define POWER_LOSS_DETECT_PIN -1
#endif //POWER_LOSS_RECOVERY

#define HEATER_2_PIN -1
#define TEMP_2_PIN -1 // ANALOG NUMBERING

#ifdef P2P1

    #ifdef ELECTROMAGNETIC_VALVE
        #define HEATER_1_PIN -1
        #define HEATER_0_PIN -1
        #define ELECTROMAGNETIC_VALVE_1_PIN 10
        #define ELECTROMAGNETIC_VALVE_0_PIN 11
    #else
        #define HEATER_1_PIN 10
        #define HEATER_0_PIN 11
    #endif

    #ifdef MIX_COLOR_TEST
        #define TEMP_1_PIN 15 // ANALOG NUMBERING
    #else
        #define TEMP_1_PIN 13 //13 ANALOG NUMBERING
    #endif
    #define TEMP_0_PIN 15 //15 by zyf   // ANALOG NUMBERING
#else
    #define HEATER_0_PIN 10
    #define HEATER_1_PIN 11
    #define TEMP_0_PIN 13 // ANALOG NUMBERING
    #define TEMP_1_PIN 15 //15 by zyf   // ANALOG NUMBERING
#endif

#define HEATER_BED_PIN 8 //  by zyf  // BED
#define TEMP_BED_PIN 14  // by zyf   // ANALOG NUMBERING //14

#ifdef NUM_SERVOS
#define SERVO0_PIN 11

#if NUM_SERVOS > 1
#define SERVO1_PIN 6
#endif

#if NUM_SERVOS > 2
#define SERVO2_PIN 5
#endif

#if NUM_SERVOS > 3
#define SERVO3_PIN 4
#endif
#endif

#define BEEPER 23
#define BEEPER_OFF LOW
#define BEEPER_ON HIGH

#define LCD_PINS_RS -1
#define LCD_PINS_ENABLE -1
#define LCD_PINS_D4 -1
#define LCD_PINS_D5 -1
#define LCD_PINS_D6 -1
#define LCD_PINS_D7 -1

#define SDCARDDETECT 49

#ifndef SDSUPPORT
// these pins are defined in the SD library if building with SD support
	#define MAX_SCK_PIN 52
	#define MAX_MISO_PIN 50
	#define MAX_MOSI_PIN 51
	#define MAX6675_SS 53
#else
	#define MAX6675_SS 49
#endif

#ifndef KNOWN_BOARD
	#error Unknown MOTHERBOARD value in configuration.h
#endif

//List of pins which to ignore when asked to change by gcode, 0 and 1 are RX and TX, do not mess with those!
#define _E0_PINS E0_STEP_PIN, E0_DIR_PIN, E0_ENABLE_PIN, HEATER_0_PIN,
#if EXTRUDERS > 1
	#define _E1_PINS E1_STEP_PIN, E1_DIR_PIN, E1_ENABLE_PIN, HEATER_1_PIN,
#else
	#define _E1_PINS
#endif

#if EXTRUDERS > 2
	#define _E2_PINS E2_STEP_PIN, E2_DIR_PIN, E2_ENABLE_PIN, HEATER_2_PIN,
#else
	#define _E2_PINS
#endif

#ifdef X_STOP_PIN
	#if X_HOME_DIR < 0
		#define X_MIN_PIN X_STOP_PIN
		#define X_MAX_PIN -1
	#else
		#define X_MIN_PIN -1
		#define X_MAX_PIN X_STOP_PIN
	#endif
#endif

#ifdef Y_STOP_PIN
	#if Y_HOME_DIR < 0
		#define Y_MIN_PIN Y_STOP_PIN
		#define Y_MAX_PIN -1
	#else
		#define Y_MIN_PIN -1
		#define Y_MAX_PIN Y_STOP_PIN
	#endif
#endif

#ifdef Z_STOP_PIN
	#if Z_HOME_DIR < 0
		#define Z_MIN_PIN Z_STOP_PIN
		#define Z_MAX_PIN -1
	#else
		#define Z_MIN_PIN -1
		#define Z_MAX_PIN Z_STOP_PIN
	#endif
#endif

#ifdef DISABLE_MAX_ENDSTOPS
	#define X_MAX_PIN -1
	#define Y_MAX_PIN -1
	#define Z_MAX_PIN -1
#endif

#ifdef DISABLE_X_MAX_ENDSTOPS
	#define X_MAX_PIN -1
#endif

#ifdef DISABLE_Y_MAX_ENDSTOPS
	#define Y_MAX_PIN -1
#endif

#ifdef DISABLE_Z_MAX_ENDSTOPS
	#define Z_MAX_PIN -1
#endif

#ifdef DISABLE_MIN_ENDSTOPS
	#define X_MIN_PIN -1
	#define Y_MIN_PIN -1
	#define Z_MIN_PIN -1
#endif

#define SENSITIVE_PINS                                                                                                                                                                                    \
    {                                                                                                                                                                                                     \
        0, 1, X_STEP_PIN, X_DIR_PIN, X_ENABLE_PIN, X_MIN_PIN, X_MAX_PIN, Y_STEP_PIN, Y_DIR_PIN, Y_ENABLE_PIN, Y_MIN_PIN, Y_MAX_PIN, Z_STEP_PIN, Z_DIR_PIN, Z_ENABLE_PIN, Z_MIN_PIN, Z_MAX_PIN, PS_ON_PIN, \
            HEATER_BED_PIN, FAN_PIN,                                                                                                                                                                      \
            _E0_PINS _E1_PINS _E2_PINS                                                                                                                                                                    \
            analogInputToDigitalPin(TEMP_0_PIN),																																						  \
            analogInputToDigitalPin(TEMP_1_PIN), analogInputToDigitalPin(TEMP_2_PIN), analogInputToDigitalPin(TEMP_BED_PIN)                                                                               \
    }
#endif