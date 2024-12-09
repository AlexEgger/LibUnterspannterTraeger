#ifndef BASEMOTORCONTROLLER_H
#define BASEMOTORCONTROLLER_H

#include "BaseActuator.h"

// Abgeleitete Klasse BasePosServo
class BaseMotorController : public BaseActuator
{
protected:
    enum motorDirection
    {
        Off = 0,
        Positive,
        Negative
    };

    int _posPin; // Positive terminal pin
    int _negPin; // Negative terminal pin

    int _currentPosition; // Current position
    int _targetPosition;  // Target position

    bool _inverseMapping; // Invert the mapping of the ADC value
    const uint16_t _tol;  // Tolerance for position control

    _integral = 0;      // Integral term for PI controller
    _previousError = 0; // Previous error for PI controller

    motorDirection _currentDirection; // Direction the motor is moving in

    // Methode zum Ansteuern der Ausgänge
    virtual void goDirection(motorDirection direction);

    // Method for setting PWM signal
    void setPWM(int pwm);

    // Calibrate minimum and maximum positions based on mechanical constraints
    void calibrate();

public:
    // Standard-Konstruktor
    BaseMotorController(int posPin, int negPin, uint16_t tol, bool inverseMapping = false, bool calibrate = false);

    // Ausgeben der aktuellen Position
    int getPosition();

    // Set target position
    void setTargetPosition(int targetPosition);

    // Set target position
    void setCurrentPosition(int currentPosition);

    // Set target position with PI controller
    void runPIController();

    // Move actuator to a specific position
    virtual ModuleState positionTargetWithTol(int targetPosition);

    // Move actuator to a specific position
    virtual ModuleState positionTarget(int targetPosition);

    // Move actuator to at least targetPosition
    virtual ModuleState positionMinimum(int targetPosition);

    // Move actuator to at maximum targetPosition
    virtual ModuleState positionMaximum(int targetPosition);
};

#endif // BASEMOTORCONTROLLER_H
