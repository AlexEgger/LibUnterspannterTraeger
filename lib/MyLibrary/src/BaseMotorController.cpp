#include "BaseActuatorExternalEncoder.h"
#include <Arduino.h>
#include "myEnums.h"

// Konstruktor der abgeleiteten Klasse BaseMotorController
BaseMotorController::BaseMotorController(int posPin, int negPin, uint16_t tol, bool inverseMapping, bool calibrate)
    : _posPin(posPin), _negPin(negPin), _currentPosition(0), _tol(tol), _inverseMapping(inverseMapping), _currentDirection(Off)
{
    // Initialisierung der Pins als Ein- und Ausgänge
    pinMode(_posPin, OUTPUT);
    pinMode(_negPin, OUTPUT);

    // Motorpins ausschalten
    digitalWrite(_posPin, LOW);
    digitalWrite(_negPin, LOW);

    if (calibrate)
    {
        // TODO: REWORK!
        //  calibrate();
    }
}

// Methode zum Ansteuern der Ausgänge
void BaseMotorController::goDirection(motorDirection direction)
{
    // Überprüfen, ob sich die Richtung geändert hat
    if (_currentDirection != direction)
    {
        _currentDirection = direction;

        // Initialisieren der Pin-Zustände
        int posState = LOW;
        int negState = LOW;

        // Bestimmen der Pin-Zustände basierend auf der Richtung und inverseMapping
        switch (direction)
        {
        case Positive:
            posState = _inverseMapping ? LOW : HIGH;
            negState = _inverseMapping ? HIGH : LOW;
            break;
        case Negative:
            posState = _inverseMapping ? HIGH : LOW;
            negState = _inverseMapping ? LOW : HIGH;
            break;
        default: // Off-Zustand
            posState = LOW;
            negState = LOW;
            break;
        }

        // Setzen der Pin-Zustände
        digitalWrite(_posPin, posState);
        digitalWrite(_negPin, negState);
    }
}

// Method for setting PWM signal
void BaseMotorController::setPWM(int pwm)
{
    // TODO: Implement PWM signal generation
}

// Run PI controller to reach the target position
void BaseMotorController::runPIController()
{
    // Calculate the error
    int error = _targetPosition - _currentPosition;

    // Calculate the integral term
    _integral += error;

    // Calculate the control signal
    int controlSignal = _kp * error + _ki * _integral;

    // Limit the control signal (no windup)
    if (controlSignal > 255)
    {
        controlSignal = 255;
        _integral -= error;
    }
    else if (controlSignal < -255)
    {
        controlSignal = -255;
        _integral -= error;
    }

    // Update the previous error
    _previousError = error;

    // Set the direction based on the control signal
    setPWM(controlSignal);
}

// Move actuator to a specific position
ModuleState BaseMotorController::positionTarget(int targetPosition)
{
    if (_closedLoopTol > 0) // Closed loop control
    {
        if (_currentPosition >= targetPosition - _closedLoopTol && _currentPosition <= targetPosition + _closedLoopTol)
        {
            while (positionTargetWithTol(targetPosition) == RunningState)
            {
                // Wait until position is reached
                // Evlt delay(10) einfügen
            }
            return CompletedState; // Operation completed
        }
    }
    return positionTargetWithTol(targetPosition);
}

// Move actuator to a specific position within +-_tol
ModuleState BaseMotorController::positionTargetWithTol(int targetPosition)
{
    if (_currentPosition < targetPosition - _tol)
    {
        goDirection(Positive);

        return RunningState; // Still running
    }
    else if (_currentPosition > targetPosition + _tol)
    {
        goDirection(Negative);

        return RunningState; // Still running
    }
    else
    {
        goDirection(Off);

        // Serial.println("Position reached");

        return CompletedState; // Operation completed
    }
}

// Increase actuator position to at least targetPosition
ModuleState BaseMotorController::positionMinimum(int targetPosition)
{
    if (_currentPosition < targetPosition) // Compare with the defined open position
    {
        goDirection(Positive);
        return RunningState; // Still running
    }
    else
    {
        goDirection(Off);
        return CompletedState; // Operation completed
    }
}

// Reduce actuator position to at maximum targetPosition
ModuleState BaseMotorController::positionMaximum(int targetPosition)
{
    if (_currentPosition > targetPosition) // Compare with the defined closed position
    {
        goDirection(Negative);
        return RunningState; // Still running
    }
    else
    {
        goDirection(Off);
        return CompletedState; // Operation completed
    }
}

// Set target position
void BaseMotorController::setTargetPosition(int targetPosition)
{
    _targetPosition = targetPosition;
    if (_targetPosition > _currentPosition)
    {
        goDirection(Positive);
    }
    else if (_targetPosition < _currentPosition)
    {
        goDirection(Negative);
    }
    else
    {
        goDirection(Off);
    }
}

// Set current position
void BaseMotorController::setCurrentPosition(int currentPosition)
{
    _currentPosition = currentPosition;
}

// Ausgeben des Index der aktuellen Position
int BaseMotorController::getPositionIndex()
{
    return _currentPositionIndex;
}

// Calibrate minimum and maximum positions based on mechanical constraints
void BaseMotorController::calibrateMinMaxPositions()
{
    unsigned long startTime;            // Start time for timeout control
    const unsigned long timeout = 5000; // Maximum duration allowed for each calibration step
    const int startDelay = 500;         // Time delay before closed loop starts (in each direction)
    const int loopTime = 500;           // Waiting time in the loop before next measurement
    const int stableThreshold = 1;      // Minimum position change threshold to detect mechanical stop

    // Move to the mechanical minimum position
    goDirection(Negative);
    delay(startDelay);                // Allow the motor to start moving
    startTime = millis();             // Initialize timer for the movement
    int previousPosition = _position; // Track position changes to detect stalls

    while (true)
    {
        updatePosition(); // Continuously monitor the motor position

        // Stop if position change is below threshold or timeout is exceeded
        if (abs(previousPosition - _position) < stableThreshold || millis() - startTime > timeout)
        {
            goDirection(Off);
            _storedPositions[1] = _position; // Store the calibrated minimum position
            break;
        }
        previousPosition = _position; // Update position tracking
        delay(loopTime);              // Allow time for motor movement and position feedback
    }

    // Move to the mechanical maximum position
    goDirection(Positive);
    delay(startDelay);            // Allow the motor to start moving
    startTime = millis();         // Reset timer for max position calibration
    previousPosition = _position; // Reset position tracking for this phase

    while (true)
    {
        updatePosition(); // Continuously monitor the motor position

        // Stop if position change is below threshold or timeout is exceeded
        if (abs(previousPosition - _position) < stableThreshold || millis() - startTime > timeout)
        {
            goDirection(Off);
            _storedPositions[0] = _position; // Store the calibrated maximum position
            break;
        }
        previousPosition = _position; // Update position tracking
        delay(loopTime);              // Allow time for motor movement and position feedback
    }
}
