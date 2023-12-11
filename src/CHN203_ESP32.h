#pragma once

#include <ctype.h>
#include <mutex>
#include <ESP32Encoder.h>

class GenericMotor
{
public:
    virtual void drive(float speed);
    virtual void brake();
};

class CHN203_ESP32
{
public:
    CHN203_ESP32(GenericMotor motor, uint8_t c1, uint8_t c2);
    ~CHN203_ESP32();

    void drive(float speed);
    void brake();

    void setSpeed(float speed);
    void rotate(float angle);

protected:
    std::mutex m_lock;

    GenericMotor m_motor;
    ESP32Encoder m_encoder;

    enum ControlTaskType
    {
        NONE,
        POSITION,
        SPEED,
    };
    CHN203_ESP32::ControlTaskType m_controlTaskType;

    float m_setPos;
    float m_setSpeed;

    static void motorControl(void *params);
};
