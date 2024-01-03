#pragma once

#include <ctype.h>
#include <mutex>

#include <ESP32Encoder.h>

class GenericDriver
{
public:
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
    // TODO: get unique ownership of motor.
    CHN203(GenericDriver *motor, uint8_t c1, uint8_t c2, float ratio);

    // Passed directly to GenericDriver functions.
    void drive(float speed);
    void brake();

    // Change speed or position setpoint.
    void setSpeed(float speed);
    void rotate(float angle);

    // Return total rotation from initial orientation:
    // In fractions of a turn:
    float getPosition();
    // In radians:
    float getPositionRad();
    // In degrees:
    float getPositionDeg();

    // Return speed:
    // In fractions of a turn per second [Hz]:
    float getSpeed();

protected:
    // Thread lock.
    std::mutex m_lock;

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

    // Setpoints for position and speed.
    float m_setPosition;
    float m_setSpeed;

    // Current speed.
    float m_speed = 0;

    // Control function, runs on timer.
    static void motorControl(void *params);
};
