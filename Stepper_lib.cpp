#include <Arduino.h>
#include "Stepper_lib.h"

// Constructor to initialize the stepper motor with specified pins and maximum angle
StepperMotor::StepperMotor(const int enPin, const int dirPin, const int stepPin, int limitPin, const int encPin, const int maxAngle) {
  _enPin = enPin;
  _dirPin = dirPin;
  _stepPin = stepPin;
  _driveVarPointer->_limitPin = limitPin;

  // Set pin modes
  pinMode(limitPin, INPUT_PULLUP);
  pinMode(enPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(stepPin, INPUT_PULLUP);

  // Disable the motor by default
  digitalWrite(_enPin, LOW);
}

// Default constructor
StepperMotor::StepperMotor() {}

// Method to set pin configurations
DriveReturnValues* StepperMotor::SetPins(const int enPin, const int dirPin, const int stepPin, const int encPin, int limitPin, const int maxAngle) {
  _enPin = enPin;
  _dirPin = dirPin;
  _stepPin = stepPin;
  _encPin = encPin;
  _maxAngle = maxAngle;
  _driveVarPointer->_limitPin = limitPin;

  // Set pin modes
  pinMode(limitPin, INPUT_PULLUP);
  pinMode(enPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(stepPin, INPUT_PULLUP);

  // Disable the motor by default
  digitalWrite(_enPin, LOW);
  return _driveReturnPointer;
}

// Method to set driving parameters
void StepperMotor::SetDrive(const int steps, const uint32_t& dir, const uint32_t& speed, const uint32_t& ramp) {
  if ((    (_stepPosition + steps > _maxAngle - 50 && dir == 1) || (_stepPosition - steps < 50 && dir == 0)) && _driveVarPointer->_overrideFlag == 0) {
    _driveVarPointer->_driveCode = DRIVE_OVERLIMIT;
    this->AllStop();
    return;
  }

  _driveVarPointer->_steps = steps;
  _driveVarPointer->_direction = dir;
  _driveVarPointer->_speed = speed;
  _driveVarPointer->_ramp = ramp;
}

// Method to set up a slave motor
void StepperMotor::SetSlave(StepperMotor& stepperMaster, bool motorInverse) {
  stepperMaster.slavedMotorDrive = this;
  _driveVarPointer = &(stepperMaster.driveVariables);
  _isSlaveMode = true;
  _motorInverse = motorInverse;
}

// Main motor driving function
void StepperMotor::DriveMotor() {
  // Calculate speed including ramp effect
  if (_driveVarPointer->_steps <= 0 && _driveVarPointer->_driveCode == DRIVE_DRIVING) {
    _driveVarPointer->_driveCode = DRIVE_READY;
  }

  // Drive the motor if conditions are met
  if ((_timer + _driveVarPointer->_speed) < micros() && _driveVarPointer->_steps > 0 && (_driveVarPointer->_driveCode == DRIVE_READY || _driveVarPointer->_driveCode == DRIVE_HOMING || _driveVarPointer->_driveCode == DRIVE_DRIVING || _driveVarPointer->_overrideFlag == 1)) {
    if (_driveVarPointer->_driveCode != DRIVE_HOMING) {
      _driveVarPointer->_driveCode = DRIVE_DRIVING;
    }
    digitalWrite(_dirPin, _motorInverse ? abs(_driveVarPointer->_direction - 1) : _driveVarPointer->_direction);
    if (_driveVarPointer->_ramp > 0 && _driveVarPointer->_steps > 2000 && !_isSlaveMode) {
      _driveVarPointer->_ramp--;
    }
    if (_driveVarPointer->_steps < 2000 && !_isSlaveMode) {
      _driveVarPointer->_ramp++;
    }
    _driveReturnPointer->_returnStepsOffset = _stepPosition - _driverStepOffset;
    _driveReturnPointer->_returnStepsABS = _stepPosition;
    // Step the motor
    digitalWrite(_stepPin, _flipFlop);
    _flipFlop = abs(_flipFlop - 1);  // Toggle step pin
    _timer = micros();               // Reset timer

    // Update step position based on direction
    _stepPosition += (_driveVarPointer->_direction == 0) ? -1 : 1;

    // Decrement steps if not in slave mode
    if (!_isSlaveMode) {
      _driveVarPointer->_steps--;
    }
  }
}
void StepperMotor::ResetPosition() {
  _revActual = 0;
  _revDown = true;
  _revUp = true;
  _encCurr = analogRead(_encPin);
  _encPrevActual = _encCurr;
  _stepActual = 0;
  _stepChange = 0;
  _stepPosition = 0;
  _driveVarPointer->_steps = 0;
  _driveVarPointer->_direction = 0;
  _driveVarPointer->_speed = 0;
  _driveVarPointer->_ramp = 0;
}
// Stop all motor activity
void StepperMotor::AllStop() {
  _driveVarPointer->_steps = 0;
  _driveVarPointer->_direction = 0;
  _driveVarPointer->_speed = 0;
  _driveVarPointer->_ramp = 0;
}

// Enable override mode
void StepperMotor::OverrideOn() {
  _driveVarPointer->_overrideFlag = 1;
}

// Disable override mode
void StepperMotor::OverrideOff() {
  _driveVarPointer->_overrideFlag = 0;
}

// Get absolute step position
long StepperMotor::GetStepAbs() {
  return _stepPosition;
}

// Get working offset for steps
long StepperMotor::GetStepWorkOffset() {
  return _stepPosition - _driverStepOffset;
}

// Update encoder readings
void StepperMotor::EncUpdate() {
  _encCurr = analogRead(_encPin);
  // Rev status flags
  if (_encCurr > 400 && _encCurr < 600) {
    _revUp = true;
    _revDown = true;
  }
  // Calculate step change
  _stepChange = _encCurr - _encPrevActual;
  _encPrevActual = _encCurr;
  _stepActual += _stepChange;
  // Update rev actual based on step change
  if (_stepChange > 100 && _revUp) {
    _revActual--;
    _revUp = false;
    _revDown = true;
  } else if (_stepChange < -100 && _revDown) {
    _revActual++;
    _revDown = false;
    _revUp = true;
  }
}

// Get working offset for encoder
long StepperMotor::GetEncWorkOffset() {
  return ((_revActual * 1024 + _stepActual - _encStepOffset) * 3.125);
}

// Get absolute encoder position
long StepperMotor::GetEncAbs() {
  return ((_revActual * 1024 + _stepActual) * 3.125);
}

void StepperMotor::ErrorDetection() {
  long encVal = this->GetEncAbs();    // Use actual values
  long stepVal = this->GetStepAbs();  // Use actual values
  if (abs(abs(stepVal) - abs(encVal)) > 200 && 0){// THIS ZERO NEEDS TO REMOVED ONCE ENCS ARE INSTALLED!!!! 
    _driveVarPointer->_driveCode = ENCODER_DEVIATION;
  }
  _driveReturnPointer->_returnStatusCode = _driveVarPointer->_driveCode;
  _driveReturnPointer->_returnStepsOffset = _stepPosition - _driverStepOffset;
  _driveReturnPointer->_returnStepsABS = _stepPosition;
  return 0;  // No error detected
}
// Update working offset for driver
void StepperMotor::UpdateWorkOffset() {
  _driverStepOffset = _stepPosition;
  _encStepOffset = (_revActual >= 0) ? (_revActual * 1024 + _stepActual) : (_revActual * 1024 - _stepActual);
  _driveReturnPointer->_returnStepsOffset = _stepPosition - _driverStepOffset;
}

// Homing procedure for the motor
void StepperMotor::Homing() {
  bool currentlyHoming = true;
  _driveVarPointer->_overrideFlag = 1;
  long timeOut = millis();
  long timeOutDelay = 50000;
  _driveVarPointer->_driveCode = DRIVE_HOMING;
  _driveReturnPointer->_returnStatusCode = _driveVarPointer->_driveCode;


  // Initial Homing Stage
  while (currentlyHoming) {


    if (homingStage == 0) {
      if (digitalRead(_driveVarPointer->_limitPin) == 1) {
        homingStage = 1;
        ResetPosition();
      } else {
        SetDrive(200, 0, 200, 0);  // Set drive parameters
        DriveMotor();              // Drive the motor
        if (slavedMotorDrive != nullptr) {
          slavedMotorDrive->DriveMotor();
        }
      }
    } else if (homingStage == 1) {
      if (_stepPosition > 5000) {
        homingStage = 2;
        //ResetPosition();
      } else {
        SetDrive(200, 1, 200, 0);  // Set drive parameters
        DriveMotor();              // Drive the motor
        if (slavedMotorDrive != nullptr) {
          slavedMotorDrive->DriveMotor();
        }
      }
    } else if (homingStage == 2) {
      if (digitalRead(_driveVarPointer->_limitPin) == 1) {
        homingStage = 0;
        currentlyHoming = false;
        //ResetPosition();
      } else {
        SetDrive(200, 0, 500, 0);  // Set drive parameters
        DriveMotor();              // Drive the motor
        if (slavedMotorDrive != nullptr) {
          slavedMotorDrive->DriveMotor();
        }
      }
    }
  }
  // Final reset after homing is complete
  ResetPosition();
  _driveVarPointer->_overrideFlag = 0;
  homingStage = 0;
  _driveVarPointer->_driveCode = DRIVE_READY;
}