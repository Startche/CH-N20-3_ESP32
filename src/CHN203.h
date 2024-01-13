#pragma once

#include <ctype.h>
#include <mutex>

#include <ESP32Encoder.h>
#include <PID.h>

class GenericDriver
{
public:
    // Bounds for speed.
    static constexpr float MIN_SPEED = -1;
    static constexpr float MAX_SPEED = 1;

    // Input: speed between -1 and 1,
    // with 1 being maximum forward speed,
    // -1 being maximum backward speed.
    virtual void drive(float speed) = 0;

    // Stop motor.
    virtual void brake() = 0;
};

class CHN203
{
public:
    CHN203(GenericDriver *motor, uint8_t c1, uint8_t c2, float ratio);

    // Passed directly to GenericDriver functions.
    void drive(float speed);
    void brake();

    // Tune PID controllers.
    void tunePositionPID(float kp, float ki, float kd);
    void tuneSpeedPID(float kp, float ki, float kd);

    // Rotate:
    // By a fraction of a turn:
    void rotate(float turn);
    // By some radians:
    void rotateRad(float angle);
    // By some degrees:
    void rotateDeg(float angle);

    // Return total rotation from initial orientation:
    // In fractions of a turn:
    float getPosition();
    // In radians:
    float getPositionRad();
    // In degrees:
    float getPositionDeg();

    // Set speed:
    // In turns per second [Hz]:
    void setSpeed(float speed);
    // In radians per second:
    void setSpeedRad(float speed);
    // In degrees per second:
    void setSpeedDeg(float speed);
    // In rotations per minute [RPM]:
    void setSpeedRPM(float speed);

    // Return speed:
    // In turns per second [Hz]:
    float getSpeed();
    // In radians per second:
    float getSpeedRad();
    // In degrees per second:
    float getSpeedDeg();
    // In rotations per minute [RPM]:
    float getSpeedRPM();

private:
    // Helper functions for turning encoder counts into rotation
    // and vice-versa.
    float countsToRotation(float counts);
    float rotationToCounts(float rotation);

    // Thread lock and helper functions.
    std::mutex m_lock;
    void lock();
    void unlock();

    // Motor driver and encoder objects.
    GenericDriver *m_motor;
    ESP32Encoder m_encoder;

    // Gear ratio for motor.
    float m_ratio;

    // Types of control task and current task.
    enum ControlTaskType
    {
        NONE,
        POSITION,
        SPEED,
    };
    CHN203::ControlTaskType m_currentTask = NONE;

    // Task handle.
    TaskHandle_t m_xHandle = NULL;

    // Controller execution interval.
    static constexpr long m_intervalMs = 20;                   // Milliseconds.
    static constexpr float m_interval = m_intervalMs / 1000.0; // Seconds.

    // PID controllers.
    PID m_pidPosition;
    PID m_pidSpeed;

    // Setpoints for position and speed.
    long m_setCounts = 0;
    float m_setCountsPerSecond = 0;

    // Current speed.
    float m_speed = 0;

    // Control function, runs on timer.
    static void motorControl(void *params);
};
