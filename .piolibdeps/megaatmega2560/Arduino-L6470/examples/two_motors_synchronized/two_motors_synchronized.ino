/**
 * This example synchronizes the actions of two motors by using
 * SPI daisy chaining.
 *
 * The hardware setup is:
 *   MOSI from controller tied to SDI on the first device
 *   SDO of the first device is tied to SDI of the next device
 *   SDO of the last device is tied to MISO of the controller
 *   all devices share the same SCK, SS_PIN and RESET_PIN
 *
 * Each L6470 passes the data it saw on its SDI to its neighbor
 * on the NEXT SPI cycle (8 bit delay).
 *
 * Each L6470 acts on the last SPI data it saw when the SS_PIN goes high.
 */

/**
 * Two different SPI routines are used.  One routine is used to send
 * commands to individual devices.  A different one is used for motion
 * commands so that all devices act at the same time.
 */

/**
 * The array "chain[]" is used to tell the software how the hardware is hooked up
 *   [0] - number of drivers in chain
 *   [1] - axis index for first device in the chain (closest to MOSI)
 *   [2] - axis index for next device in the chain
 *
 *  Axis index is an arbitrary identifier assigned by the user
 */

#include <L6470.h>

#define SS_PIN     10
#define SCK_PIN    11
#define MOSI_PIN   12
#define MISO_PIN   13
#define RESET_PIN  14

class TwoMotors : public L64XXHelper {
public:
  TwoMotors() { L64XX::set_helper(*this); }

  /**
   * Initialize pins for non-library SPI software
   */
  static void spi_init() {
    pinMode(SS_PIN, OUTPUT);
    pinMode(SCK_PIN, OUTPUT);
    pinMode(MOSI_PIN, OUTPUT);
    digitalWrite(SS_PIN, HIGH);
    digitalWrite(SCK_PIN, HIGH);
    digitalWrite(MOSI_PIN, HIGH);
    pinMode(MISO_PIN, INPUT);
  }

  static inline uint8_t transfer(uint8_t data, const int16_t ss_pin) { }

  /**
   * Used in all non-motion commands/transfers
   * Send/receive one uint8_t to the target device. All other devices get a NOOP command.
   * Data for the last device in the chain is sent out first!
   */
  static inline uint8_t transfer(uint8_t data, const int16_t ss_pin, const uint8_t chain_position) {
    #define CMD_NOP 0
    uint8_t data_out = 0;
    data--;
    // first device in chain has data sent last
    digitalWrite(ss_pin, LOW);

    for (uint8_t i = L64XX::chain[0]; i >= 1; i--) {
      uint8_t temp = L6470_SpiTransfer_Mode_3(uint8_t(i == chain_position ? data : CMD_NOP));
      if (L64XX::chain[i] == chain_position) data_out = temp;
    }

    digitalWrite(ss_pin, HIGH);
    return data_out;
  }

  static inline uint8_t L6470_SpiTransfer_Mode_3(uint8_t b) { // using Mode 3
    uint8_t bits = 8;
    do {
      digitalWrite(SCK_PIN, LOW);
      digitalWrite(MOSI_PIN, b & 0x80);
      //DELAY_NS(125);
      digitalWrite(SCK_PIN, HIGH);
      b <<= 1;        // little setup time
      b |= (digitalRead(MISO_PIN) != 0);
    } while (--bits);
    //DELAY_NS(125);
    return b;
  }

  /**
   * This is the routine that sends the motion commands.
   *
   * This routine sends a buffer of data to be filled by the application. The
   * library is not involved with it.
   */
  static inline void Buffer_Transfer(uint8_t buffer[], const uint8_t length) {
    //uint8_t buffer[number of steppers + 1];
      // [0] - not used
      // [1] - command for first device
      // [2] - command for second device

    // first device in chain has data sent last
    digitalWrite(SS_PIN, LOW);
    for (uint8_t i = length; i >= 1; i--)
      buffer[i] = L6470_SpiTransfer_Mode_3(uint8_t (buffer[i]));
    digitalWrite(SS_PIN, HIGH);
  }

  static inline void goTo(long location_1, long location_2) {
    // the command string to move a stepper to an absolute position is
    // four uint8_t long so four arraya are used for convenience

    uint8_t buffer_command[3] = { dSPIN_GOTO, dSPIN_GOTO };  // create and fill buffer with commands

    if (location_1 > 0x3FFFFF) location_1 = 0x3FFFFF;  // limit to 22 bits
    if (location_1 > 0x3FFFFF) location_1 = 0x3FFFFF;

    uint8_t addr21_16[3] = { 0, uint8_t(location_1 >> 16), uint8_t(location_2 >> 16) };
    uint8_t addr15_8[3]  = { 0, uint8_t(location_1 >>  8), uint8_t(location_2 >>  8) };
    uint8_t addr7_0[3]   = { 0, uint8_t(location_1)      , uint8_t(location_2)       };

    Buffer_Transfer(buffer_command, L64XX::chain[0]);  // send the commands
    Buffer_Transfer(addr21_16     , L64XX::chain[0]);  // send the MSB of the position
    Buffer_Transfer(addr15_8      , L64XX::chain[0]);
    Buffer_Transfer(addr7_0       , L64XX::chain[0]);  // this one results in the motors moving
  }

  static inline void _setup() {
    pinMode(RESET_PIN,OUTPUT);        // Reset all drivers
    digitalWrite(RESET_PIN, LOW);     // Do this before any setup commands are sent to the drivers
    delay(10);
    digitalWrite(RESET_PIN, HIGH);

    stepperA.set_chain_info(56, 1);   // Completely setup chain[] before
    stepperB.set_chain_info(56, 2);   // Any SPI traffic is sent

    stepperA.init();
    stepperA.setAcc(100);             // Set acceleration
    stepperA.setMaxSpeed(800);
    stepperA.setMinSpeed(1);
    stepperA.setMicroSteps(2);        // 1,2,4,8,16,32,64 or 128
    stepperA.setThresholdSpeed(1000);
    stepperA.setOverCurrent(6000);    // Set overcurrent protection
    stepperA.setStallCurrent(3000);

    stepperB.init();
    stepperB.setAcc(100);             // Set acceleration
    stepperB.setMaxSpeed(800);
    stepperB.setMinSpeed(1);
    stepperB.setMicroSteps(2);        // 1,2,4,8,16,32,64 or 128
    stepperB.setThresholdSpeed(1000);
    stepperB.setOverCurrent(6000);    // Set overcurrent protection
    stepperB.setStallCurrent(3000);

    goTo(200,200);                    // Spin the motors at the same time
  }

  static inline void _loop() {
    while (stepperB.isBusy()) delay(10);
    goTo(-200,-200);
    while (stepperB.isBusy()) delay(10);
    goTo(2000,2000);
  }

private:
  static L6470 stepperA;
  static L6480 stepperB;
  static powerSTEP01 stepperC;
};

L6470 TwoMotors::stepperA(SS_PIN);
L6480 TwoMotors::stepperB(SS_PIN);
powerSTEP01 TwoMotors::stepperC(SS_PIN);

TwoMotors two_motors;

void setup() { two_motors._setup(); }
void loop() { two_motors._loop(); }
