#include "BasePosServo.h"
#include <Arduino.h>
#include "myEnums.h"

// Konstruktor der abgeleiteten Klasse BasePosServo
BasePosServo::BasePosServo(int posPin, int negPin, int adcPin, uint16_t tol, uint16_t closedLoopTol, bool inverseMapping, bool calibrateMinMax)
    : _posPin(posPin), _negPin(negPin), _adcPin(adcPin), _position(0), _tol(tol), _closedLoopTol(closedLoopTol), _inverseMapping(inverseMapping), _currentDirection(Off), _currentPositionIndex(-1)
{
    // Initialisierung der Pins als Ein- und Ausgänge
    pinMode(_posPin, OUTPUT);
    pinMode(_negPin, OUTPUT);
    pinMode(_adcPin, INPUT);

    // Motorpins ausschalten
    digitalWrite(_posPin, LOW);
    digitalWrite(_negPin, LOW);

    updatePosition();

    if (calibrateMinMax)
    {
        calibrateMinMaxPositions();
    }
}

// Überladener Konstruktor der abgeleiteten Klasse BasePosServo
BasePosServo::BasePosServo(int posPin, int negPin, int adcPin, uint16_t tol, const int positions[2], uint16_t closedLoopTol, bool inverseMapping)
    : BasePosServo(posPin, negPin, adcPin, tol, closedLoopTol, inverseMapping) // Aufruf des Standardkonstruktors
{
    // Speichern der übergebenen Positionen im Array
    _storedPositions[0] = positions[0];
    _storedPositions[1] = positions[1];
}

// Methode zur Ausgabe der aktuellen Position
int BasePosServo::getPosition()
{
    updatePosition(); // Position aktualisieren
    return _position;
}

// Methode zur Umwandlung des ADC-Werts in einen Dezimalwert
uint16_t BasePosServo::mapADC(int x, int in_min, int in_max, uint16_t out_min, uint16_t out_max)
{
    // Sicherstellen, dass der Wert von x innerhalb der erwarteten Grenzen liegt
    if (x < in_min)
        x = in_min;
    if (x > in_max)
        x = in_max;

    if (_inverseMapping) // Invertiertes Mapping: Maximaler Eingang führt zum minimalen Ausgang und umgekehrt
    {
        return static_cast<uint16_t>((static_cast<float>(in_max - x) * (out_max - out_min) / (in_max - in_min)) + out_min);
    }
    else // Normales Mapping: Maximaler Eingang führt zum maximalen Ausgang
    {
        return static_cast<uint16_t>((static_cast<float>(x - in_min) * (out_max - out_min) / (in_max - in_min)) + out_min);
    }
}

// Methode zur Aktualisierung der aktuellen Position des Servos
void BasePosServo::updatePosition()
{
    _adcValue = analogRead(_adcPin);                 // Read ADC value (range: 0 to 1023)
    _position = mapADC(_adcValue, 0, 1023, 10, 520); // Map from 0-1023 to 10-520 (Position in 1/100 cm)

    // Serial.print(F("ADC-Wert: "));
    // Serial.println(_adcValue);
    // Serial.print(F("Position: "));
    // Serial.println(_position);
}

// Methode zum Ansteuern der Ausgänge
void BasePosServo::goDirection(motorDirection direction)
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

// Move actuator to a specific position
ModuleState BasePosServo::positionTarget(int targetPosition)
{
    if (_closedLoopTol > 0) // Closed loop control
    {
        if (_position >= targetPosition - _closedLoopTol && _position <= targetPosition + _closedLoopTol)
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
ModuleState BasePosServo::positionTargetWithTol(int targetPosition)
{
    updatePosition(); // Position aktualisieren

    if (_position < targetPosition - _tol)
    {
        goDirection(Positive);

        return RunningState; // Still running
    }
    else if (_position > targetPosition + _tol)
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
ModuleState BasePosServo::positionMinimum(int targetPosition)
{
    updatePosition();

    if (_position < targetPosition) // Compare with the defined open position
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
ModuleState BasePosServo::positionMaximum(int targetPosition)
{
    updatePosition();

    if (_position > targetPosition) // Compare with the defined closed position
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

// Ausgeben des Index der aktuellen Position
int BasePosServo::getPositionIndex()
{
    return _currentPositionIndex;
}

// Calibrate minimum and maximum positions based on mechanical constraints
void BasePosServo::calibrateMinMaxPositions()
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
