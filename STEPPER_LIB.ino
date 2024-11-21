#include "Stepper_lib.h"


StepperMotor motors[1];






void setup() {
  Serial.begin(9600);
  motors[0].SetPins(17, 15, 16, 22, 14, 19000);
  motors[0].OverrideOn();
  motors[0].ResetPosition();
  motors[0].SetDrive(3200 * 100, 1, 70, 0);
}
int i = 0;
unsigned long prev = 0;
// Main loop function
void loop() {

  motors[0].EncUpdate();
  motors[0].DriveMotor();
  if (millis() - prev > 200) {
    prev = millis();
    Serial.print("Encoder Pos = ");
    Serial.print(motors[0].GetEncAbs() * -1);
    Serial.print("     Step Pos = ");
    Serial.println(motors[0].GetStepAbs());
    i++;
  }
  if (i > 200 && i < 3000) {
    motors[0].SetDrive(3200 * 100, 0, 70, 0);
    i=5641;
  }
}
