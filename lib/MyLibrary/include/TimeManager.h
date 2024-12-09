#ifndef TIMEMANAGER_H
#define TIMEMANAGER_H

#include <Arduino.h>

/**
 * @brief TimeManager class to handle timing for individual classes or tasks.
 *
 * Each instance of TimeManager tracks its own time, so it can be used
 * independently in different parts of the program.
 */
class TimeManager
{
public:
    /**
     * @brief Constructor to initialize the TimeManager with an optional interval time.
     *
     * @param intervalTime Default interval time in milliseconds (default is 1000 ms).
     */
    TimeManager(unsigned long intervalTime = 1000);

    /**
     * @brief Updates the current time.
     *
     * This should be called frequently (e.g., in the main loop) to keep
     * the internal time reference updated.
     */
    void updateCurrentTime();

    /**
     * @brief Checks if the specified time has passed.
     *
     * If no time is specified, it uses the default interval time set in the constructor.
     *
     * @param time The time interval to check (in milliseconds). If not specified, uses the default _intervalTime.
     * @return true if the specified time interval has passed.
     */
    bool timePassed(unsigned long time = 0);

    /**
     * @brief Resets the passed time to the current time.
     *
     * This can be used to manually reset the timer.
     */
    void resetPassedTime();

    /**
     * @brief Sets a new interval time.
     *
     * This allows you to change the default time interval for this TimeManager.
     *
     * @param intervalTime The new time interval to use (in milliseconds).
     */
    void setIntervalTime(unsigned long intervalTime);

    /**
     * @brief Gets the current system time.
     *
     * @return The current system time in milliseconds.
     */
    unsigned long getCurrentTime() const;

private:
    unsigned long _currentTime;  ///< The current system time (in milliseconds).
    unsigned long _passedTime;   ///< The last recorded time the action was executed.
    unsigned long _intervalTime; ///< Default interval time for this TimeManager (in milliseconds).
};

#endif // TIMEMANAGER_H
