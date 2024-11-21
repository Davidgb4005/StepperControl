#include <Arduino.h>

#ifndef STEPPER_LIB_H
#define STEPPER_LIB_H

struct DriveInputs {
    int _direction;
    int _speed;
    long _steps;
    int _ramp;
    int _overrideFlag;
    int _limitPin;
    int _driveCode;

};
struct DriveReturnValues{
  int _returnStepsABS;
  int _returnStepsOffset;
  int _returnStatusCode;
  int _returnDriveReady;
  
};
enum DriveCodes {
    HOMING_TIMEOUT = 12,
    ENCODER_DEVIATION = 11,
    DRIVE_OVERLIMIT = 10,
    DRIVE_NOHOME = 9,
    DRIVE_READY = 0,
    DRIVE_DRIVING = 1,
    DRIVE_HOMING = 2
};

class StepperMotor {
public:
    // Constructors
    StepperMotor(const int enPin, const int dirPin, const int stepPin, const int encPin, int limitPin, const int maxAngle);
    StepperMotor();

    // Methods
    DriveReturnValues* SetPins(const int enPin, const int dirPin, const int stepPin, const int encPin, int limitPin, const int maxAngle);
    void DriveMotor();
    void ResetPosition();
    void Homing();
    void OverrideOn();
    void OverrideOff();
    void SetSlave(StepperMotor& stepperMaster, bool motorInverse);
    void SetDrive(const int steps, const uint32_t& dir, const uint32_t& speed, const uint32_t& ramp);
    long GetStepAbs();
    long GetStepWorkOffset();
    void EncUpdate();
    void UpdateWorkOffset(); 
    long GetEncWorkOffset();
    long GetEncAbs();
    void AllStop();
    void ErrorDetection();

private:
    // Parameter pointers
    DriveInputs driveVariables = {._driveCode = DRIVE_NOHOME};
    DriveReturnValues driveReturnVal;
    DriveInputs* _driveVarPointer = &driveVariables;
    DriveReturnValues* _driveReturnPointer = &driveReturnVal;
    // I/O Pins
    int _fullTurnSteps;
    int _enPin;
    int _dirPin;
    int _stepPin;
    int _encPin;

    // Drive parameters
    StepperMotor * slavedMotorDrive = nullptr;
    bool _isSlaveMode = false;
    bool _motorInverse = false;
    int _maxAngle = 40000;
    uint32_t _timer;
    long _stepPosition = 0;
    int _flipFlop = 0;
    long _driverStepOffset = 0;

    // Encoder parameters
    bool _revUp = true;
    bool _revDown = true;
    int _stepChange = 0;
    long _encPrevActual = 0;
    int _encCurr = 0;
    long _revActual = 0;
    int _stepActual = 0;
    int _encStepOffset = 0;
    int _homeOffset = 0;


    //Homing Parameters
  int homingStage = 0;

};

#endif
