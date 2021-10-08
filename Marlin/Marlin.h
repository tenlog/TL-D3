// Tonokip RepRap firmware rewrite based off of Hydra-mmm firmware.
// Licence: GPL

#ifndef MARLIN_H
#define MARLIN_H

#define FORCE_INLINE __attribute__((always_inline)) inline

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#include <util/delay.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>

#include "fastio.h"
#include "Configuration.h"
#include "pins.h"

#ifndef AT90USB
#define HardwareSerial_h // trick to disable the standard HWserial
#endif

#include "Arduino.h"
#if (ARDUINO >= 100)
#else
//#include "WProgram.h"
//Arduino < 1.0.0 does not define this, so we need to do it ourselfs
#define analogInputToDigitalPin(p) ((p) + A0)
#endif

#include "MarlinSerial.h"

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#include "WString.h"

#ifdef AT90USB
#define MYSERIAL Serial
#else
#define MYSERIAL MSerial
#endif

#define SERIAL_PROTOCOL(x) MYSERIAL.print(x);
#define SERIAL_PROTOCOL_F(x, y) MYSERIAL.print(x, y);
#define SERIAL_PROTOCOLPGM(x) serialprintPGM(PSTR(x));
#define SERIAL_PROTOCOLLN(x) \
  {                          \
    MYSERIAL.print(x);       \
    MYSERIAL.write('\n');    \
  }

#define SERIAL_PROTOCOLLNPGM(x) \
  {                             \
    serialprintPGM(PSTR(x));    \        
    MYSERIAL.write('\n');       \
  }

//Zyf Print
#define TL_DEBUG_PRINT(x) (MYSERIAL.print(x))
#define TL_DEBUG_PRINT_LN(x) (MYSERIAL.print(x), MYSERIAL.write('\n'))

const char errormagic[] PROGMEM = "Error:";
const char echomagic[] PROGMEM = "echo:";
#define SERIAL_ERROR_START serialprintPGM(errormagic);
#define SERIAL_ERROR(x) SERIAL_PROTOCOL(x)
#define SERIAL_ERRORPGM(x) SERIAL_PROTOCOLPGM(x)
#define SERIAL_ERRORLN(x) SERIAL_PROTOCOLLN(x)
#define SERIAL_ERRORLNPGM(x) SERIAL_PROTOCOLLNPGM(x)

#define SERIAL_ECHO_START serialprintPGM(echomagic);
#define SERIAL_ECHO(x) SERIAL_PROTOCOL(x)
#define SERIAL_ECHOPGM(x) SERIAL_PROTOCOLPGM(x)
#define SERIAL_ECHOLN(x) SERIAL_PROTOCOLLN(x)
#define SERIAL_ECHOLNPGM(x) SERIAL_PROTOCOLLNPGM(x)

#define SERIAL_ECHOPAIR(name, value) (serial_echopair_P(PSTR(name), (value)))

void serial_echopair_P(const char *s_P, float v);
void serial_echopair_P(const char *s_P, double v);
void serial_echopair_P(const char *s_P, unsigned long v);

//things to write to serial from Programmemory. saves 400 to 2k of RAM.
FORCE_INLINE void serialprintPGM(const char *str)
{
  char ch = pgm_read_byte(str);
  while (ch)
  {
    MYSERIAL.write(ch);
    ch = pgm_read_byte(++str);
  }
}

bool MTLSERIAL_available();
int MTLSERIAL_read();

#if defined(DUAL_X_CARRIAGE) && defined(X_ENABLE_PIN) && X_ENABLE_PIN > -1 && defined(X2_ENABLE_PIN) && X2_ENABLE_PIN > -1
#define enable_x()                     \
  do                                   \
  {                                    \
    WRITE(X_ENABLE_PIN, X_ENABLE_ON);  \
    WRITE(X2_ENABLE_PIN, X_ENABLE_ON); \
  } while (0)
#define enable_x0()                     \
  do                                   \
  {                                    \
    WRITE(X_ENABLE_PIN, X_ENABLE_ON);  \
  } while (0)
#define enable_x1()                     \
  do                                   \
  {                                    \
    WRITE(X2_ENABLE_PIN, X_ENABLE_ON); \
  } while (0)
#define disable_x()                     \
  do                                    \
  {                                     \
    WRITE(X_ENABLE_PIN, !X_ENABLE_ON);  \
    WRITE(X2_ENABLE_PIN, !X_ENABLE_ON); \
  } while (0)
#define disable_x0()                     \
  do                                    \
  {                                     \
    WRITE(X_ENABLE_PIN, !X_ENABLE_ON);  \
  } while (0)
#define disable_x1()                     \
  do                                    \
  {                                     \
    WRITE(X2_ENABLE_PIN, !X_ENABLE_ON); \
  } while (0)
#elif defined(X_ENABLE_PIN) && X_ENABLE_PIN > -1
#define enable_x() WRITE(X_ENABLE_PIN, X_ENABLE_ON)
#define disable_x() WRITE(X_ENABLE_PIN, !X_ENABLE_ON)
#else
#define enable_x() ;
#define disable_x() ;
#endif

#if defined(Y_ENABLE_PIN) && Y_ENABLE_PIN > -1
#define enable_y() WRITE(Y_ENABLE_PIN, Y_ENABLE_ON)
#define disable_y() WRITE(Y_ENABLE_PIN, !Y_ENABLE_ON)
#else
#define enable_y() ;
#define disable_y() ;
#endif

#if defined(Z_ENABLE_PIN) && Z_ENABLE_PIN > -1
#ifdef Z_DUAL_STEPPER_DRIVERS
#define enable_z()                     \
  {                                    \
    WRITE(Z_ENABLE_PIN, Z_ENABLE_ON);  \
    WRITE(Z2_ENABLE_PIN, Z_ENABLE_ON); \
  }
#define disable_z()                     \
  {                                     \
    WRITE(Z_ENABLE_PIN, !Z_ENABLE_ON);  \
    WRITE(Z2_ENABLE_PIN, !Z_ENABLE_ON); \
  }
#elif defined(TL_DUAL_Z) //By zyf
#define enable_z()                     \
  {                                    \
    WRITE(Z_ENABLE_PIN, Z_ENABLE_ON);  \
    WRITE(Z2_ENABLE_PIN, Z_ENABLE_ON); \
  }
#define disable_z()                     \
  {                                     \
    WRITE(Z_ENABLE_PIN, !Z_ENABLE_ON);  \
    WRITE(Z2_ENABLE_PIN, !Z_ENABLE_ON); \
  }
#else
#define enable_z() WRITE(Z_ENABLE_PIN, Z_ENABLE_ON)
#define disable_z() WRITE(Z_ENABLE_PIN, !Z_ENABLE_ON)
#endif
#else
#define enable_z() ;
#define disable_z() ;
#endif

#if defined(E0_ENABLE_PIN) && (E0_ENABLE_PIN > -1)
#define enable_e0() WRITE(E0_ENABLE_PIN, E_ENABLE_ON)
#define disable_e0() WRITE(E0_ENABLE_PIN, !E_ENABLE_ON)
#else
#define enable_e0()  /* nothing */
#define disable_e0() /* nothing */
#endif

#if (EXTRUDERS > 1) && defined(E1_ENABLE_PIN) && (E1_ENABLE_PIN > -1)
#define enable_e1() WRITE(E1_ENABLE_PIN, E_ENABLE_ON)
#define disable_e1() WRITE(E1_ENABLE_PIN, !E_ENABLE_ON)
#else
#define enable_e1()  /* nothing */
#define disable_e1() /* nothing */
#endif

#if (EXTRUDERS > 2) && defined(E2_ENABLE_PIN) && (E2_ENABLE_PIN > -1)
#define enable_e2() WRITE(E2_ENABLE_PIN, E_ENABLE_ON)
#define disable_e2() WRITE(E2_ENABLE_PIN, !E_ENABLE_ON)
#else
#define enable_e2()  /* nothing */
#define disable_e2() /* nothing */
#endif

enum AxisEnum
{
  X_AXIS = 0,
  Y_AXIS = 1,
  Z_AXIS = 2,
  E_AXIS = 3
};

void FlushSerialRequestResend();
void ClearToSend();

void get_coordinates(float XValue, float YValue, float ZValue, float EValue, int iMode);

void prepare_move();
void kill();
void Stop();

bool IsStopped();

void enquecommand(const char *cmd);   //put an ascii command at the end of the current buffer.
void enquecommand_P(const char *cmd); //put an ascii command at the end of the current buffer, read from flash
void prepare_arc_move(char isclockwise);
void clamp_to_software_endstops(float target[3]);

void command_G1(float XValue = -99999.0, float YValue = -99999.0, float ZValue = -99999.0, float EValue = -99999.0, int iMode = 0);

void PrintStopOrFinished();
void command_G4(float dwell = 0);
void command_M81(bool Loop = true, bool ShowPage = true);

#ifdef POWER_LOSS_RECOVERY
#ifdef POWER_LOSS_TRIGGER_BY_PIN
bool Check_Power_Loss();
#endif
void Power_Off_Handler(bool MoveX = true, bool M81 = true);
void Save_Power_Loss_Status();
#endif

#ifdef FAST_PWM_FAN
void setPwmFrequency(uint8_t pin, int val);
#endif

#ifndef CRITICAL_SECTION_START
#define CRITICAL_SECTION_START \
  unsigned char _sreg = SREG;  \
  cli();
#define CRITICAL_SECTION_END SREG = _sreg;
#endif //CRITICAL_SECTION_START

//float raised_parked_position[4]; // used in mode 1
extern float homing_feedrate[];
extern bool axis_relative_modes[];
extern int feedmultiply;
extern int extrudemultiply; // Sets extrude multiply factor (in percent)
extern float current_position[NUM_AXIS];
extern float add_homeing[3];
extern float min_pos[3];
extern float max_pos[3];
extern int fanSpeed;

#ifdef DUAL_X_CARRIAGE
extern int dual_x_carriage_mode;
#endif

#ifdef BARICUDA
extern int ValvePressure;
extern int EtoPPressure;
#endif

#ifdef FAN_SOFT_PWM
extern unsigned char fanSpeedSoftPwm;
#endif

#ifdef FWRETRACT
extern bool autoretract_enabled;
extern bool retracted;
extern float retract_length, retract_feedrate, retract_zlift;
extern float retract_recover_length, retract_recover_feedrate;
#endif

extern unsigned long starttime;
extern unsigned long stoptime;

// Handling multiple extruders pins
extern uint8_t active_extruder;

extern int plaPreheatHotendTemp;
extern int plaPreheatHPBTemp;
extern int plaPreheatFanSpeed;

extern int absPreheatHotendTemp;
extern int absPreheatHPBTemp;
extern int absPreheatFanSpeed;

void preheat_abs();
void preheat_pla();
void cooldown();
bool strISAscii(String str);
void sd_init();

void sdcard_pause(int OValue = 0);
void sdcard_resume();
void sdcard_stop();
void raise_Z_E(int Z, int E);
void Nozzle_offset_test_print();

void command_M104(int iT = -1, int iS = -1);

void load_filament(int LoadUnLoad, int TValue);
void command_G92(float XValue = -99999.0, float YValue = -99999.0, float ZValue = -99999.0, float EValue = -99999.0);
void command_G28(int XHome = 0, int YHome = 0, int ZHome = 0);
void command_T(int T01 = -1);
void command_M502();
void command_M1003();
void WriteLastZYM(long lTime);

char *itostr2(const uint8_t &x);

#endif //MARLIN_H
