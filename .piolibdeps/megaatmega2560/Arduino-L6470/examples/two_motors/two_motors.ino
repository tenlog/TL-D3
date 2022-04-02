// two motors using independent SPIs


#include <L6470.h>

#define A_SS_PIN     10
#define A_SCK_PIN    11
#define A_MOSI_PIN   12
#define A_MISO_PIN   13
#define A_RESET_PIN  14
#define A_BUSYN_PIN  15

#define B_SS_PIN     20
#define B_SCK_PIN    21
#define B_MOSI_PIN   22
#define B_MISO_PIN   23
#define B_RESET_PIN  24
#define B_BUSYN_PIN  25

L6470 stepperA(A_SS_PIN);
L6470 stepperB(B_SS_PIN);

void setup() {

  stepperA.set_pins(A_SCK_PIN, A_MOSI_PIN, A_MISO_PIN, A_RESET_PIN, A_BUSYN_PIN);  //use library's soft SPI
  stepperA.init();
  stepperA.setAcc(100);              // Set acceleration
  stepperA.setMaxSpeed(800);
  stepperA.setMinSpeed(1);
  stepperA.setMicroSteps(2);         // 1,2,4,8,16,32,64 or 128
  stepperA.setThresholdSpeed(1000);
  stepperA.setOverCurrent(6000);     // Set overcurrent protection
  stepperA.setStallCurrent(3000);

  stepperB.set_pins(B_SCK_PIN, B_MOSI_PIN, B_MISO_PIN, B_RESET_PIN, B_BUSYN_PIN);  //use library's soft SPI
  stepperB.init();
  stepperB.setAcc(100);              // Set acceleration
  stepperB.setMaxSpeed(800);
  stepperB.setMinSpeed(1);
  stepperB.setMicroSteps(2);         // 1,2,4,8,16,32,64 or 128
  stepperB.setThresholdSpeed(1000);
  stepperB.setOverCurrent(6000);     // Set overcurrent protection
  stepperB.setStallCurrent(3000);

  //stepper.run(1, 200);

  //stepper.softStop();

  stepperA.goTo(200);        //  stepperA starts as soon as it's SS_PIN goes high
  stepperB.goTo(200);        //  stepperB's start is delayed by SPI transmission time
}

void loop() {
  while (stepperB.isBusy()) delay(10);
  stepperA.goTo(-200);
  stepperB.goTo(-200);
  while (stepperB.isBusy()) delay(10);
  stepperA.goTo(2000);
  stepperB.goTo(2000);
}
