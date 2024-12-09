#include "TimeManager.h"

// Constructor to initialize the timing manager with a default interval
TimeManager::TimeManager(unsigned long intervalTime)
    : _passedTime(0), _intervalTime(intervalTime)
{
    _currentTime = millis();

    Serial.println(F("TimeManager initialized."));
}

/**
 * @brief Updates the current system time.
 *
 * This method should be called frequently (in the main loop or as needed)
 * to ensure the timing system remains accurate.
 */
void TimeManager::updateCurrentTime()
{
    _currentTime = millis();
}

/**
 * @brief Checks if the specified time interval has passed.
 *
 * If no time is specified, it uses the default interval set in the constructor.
 *
 * @param time The time interval to check (optional). If 0, uses _intervalTime.
 * @return true if the specified or default time has passed.
 */
bool TimeManager::timePassed(unsigned long time)
{
    updateCurrentTime(); // Ensure the current time is always up-to-date

    // If no time is provided, use the default _intervalTime
    if (time == 0)
    {
        time = _intervalTime;
    }

    if (_currentTime - _passedTime >= time)
    {
        _passedTime = _currentTime; // Update the last passed time
        return true;
    }
    return false;
}

/**
 * @brief Resets the passed time to the current time.
 *
 * This can be used to manually reset the timer.
 */
void TimeManager::resetPassedTime()
{
    _passedTime = _currentTime; // Set the passed time to the current time
}

/**
 * @brief Sets a new default interval time.
 *
 * This allows the user to change the default interval for checking timePassed().
 *
 * @param intervalTime The new time interval to set (in milliseconds).
 */
void TimeManager::setIntervalTime(unsigned long intervalTime)
{
    _intervalTime = intervalTime;
}

/**
 * @brief Gets the current system time in milliseconds.
 *
 * @return The current system time in milliseconds.
 */
unsigned long TimeManager::getCurrentTime() const
{
    return _currentTime;
}
