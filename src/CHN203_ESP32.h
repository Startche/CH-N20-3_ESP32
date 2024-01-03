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

    // Rotate:
    // By a fraction of a turn:
    void rotate(float turn);
    // By some radians:
    void rotateRad(float angle);
    // By some degrees:
    void rotateDeg(float angle);

    // Set speed:
    // In turns per second [Hz]:
    void setSpeed(float speed);

    // Return total rotation from initial orientation:
    // In fractions of a turn:
    float getPosition();
    // In radians:
    float getPositionRad();
    // In degrees:
    float getPositionDeg();

    // Return speed:
    // In turns per second [Hz]:
    float getSpeed();

private:
    // Thread lock.
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

    // Setpoints for position and speed.
    long m_setCounts;
    float m_setSpeed;

    // Current speed.
    float m_speed = 0;

    // Control function, runs on timer.
    static void motorControl(void *params);
};
