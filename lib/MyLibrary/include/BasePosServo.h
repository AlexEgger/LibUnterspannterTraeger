#ifndef BASEPOSSERVO_H
#define BASEPOSSERVO_H

#include "BaseActuator.h"

// Abgeleitete Klasse BasePosServo
class BasePosServo : public BaseActuator
{
protected:
    enum motorDirection
    {
        Off = 0,
        Positive,
        Negative
    };

    int _posPin;                      // Positive terminal pin
    int _negPin;                      // Negative terminal pin
    int _adcPin;                      // ADC input pin for position feedback
    int _position;                    // Current position in 1/100 cm
    int _adcValue;                    // Current ADC value
    int _storedPositions[2];          // Fixed positions the motor should move to (optional)
    int _currentPositionIndex;        // Index of the current position in _storedPositions
    bool _inverseMapping;             // Invert the mapping of the ADC value
    const uint16_t _tol;              // Tolerance in 1/100 cm
    const uint16_t _closedLoopTol;    // Tolerance for closed loop control
    motorDirection _currentDirection; // Direction the motor is moving in

    // Convert from ADC value to a decimal value
    virtual uint16_t mapADC(int x, int in_min, int in_max, uint16_t out_min, uint16_t out_max);

    // Methode zur Aktualisierung der aktuellen Position des Servos
    virtual void updatePosition();

    // Methode zum Ansteuern der Ausgänge
    virtual void goDirection(motorDirection direction);

    // Move actuator to a specific position
    virtual ModuleState positionTargetWithTol(int targetPosition);

    // Move actuator to a specific position
    virtual ModuleState positionTarget(int targetPosition);

    // Move actuator to at least targetPosition
    virtual ModuleState positionMinimum(int targetPosition);

    // Move actuator to at maximum targetPosition
    virtual ModuleState positionMaximum(int targetPosition);

    void calibrateMinMaxPositions(); // Calibrate minimum and maximum positions based on mechanical constraints

public:
    // Standard-Konstruktor
    BasePosServo(int posPin, int negPin, int adcPin, uint16_t tol, uint16_t closedLoopTol = 0, bool inverseMapping = false, bool calibrateMinMax = false);

    // Überladener Konstruktor mit zusätzlichem Positionsarray
    BasePosServo(int posPin, int negPin, int adcPin, uint16_t tol, const int positions[2], uint16_t closedLoopTol = 0, bool inverseMapping = false);

    // Ausgeben der aktuellen Position
    int getPosition();

    // Ausgeben des Index der aktuellen Position
    int getPositionIndex();
};

#endif // BASEPOSSERVO_H
