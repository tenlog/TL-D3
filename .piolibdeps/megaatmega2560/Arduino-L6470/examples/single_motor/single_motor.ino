#include <L6470.h>

#define SS_PIN     10
#define SCK_PIN    11
#define MOSI_PIN   12
#define MISO_PIN   13
#define RESET_PIN  14
#define BUSYN_PIN  15

L6470 stepper(SS_PIN);              // create stepper object

void setup() {
  stepper.set_pins(SCK_PIN, MOSI_PIN, MISO_PIN, RESET_PIN, BUSYN_PIN);  //use library's soft SPI
  stepper.init();
  stepper.setAcc(100);              // Set acceleration
  stepper.setMaxSpeed(800);
  stepper.setMinSpeed(1);
  stepper.setMicroSteps(2);         // 1,2,4,8,16,32,64 or 128
  stepper.setThresholdSpeed(1000);
  stepper.setOverCurrent(6000);     // Set overcurrent protection
  stepper.setStallCurrent(3000);

  //stepper.run(1, 200);

  //stepper.softStop();

  stepper.goTo(200);
}

void loop() {
  while (stepper.isBusy()) delay(10);
  stepper.goTo(-200);
  while (stepper.isBusy()) delay(10);
  stepper.goTo(2000);
}
