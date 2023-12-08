#pragma once

#include <ctype.h>

class GenericMotor {
    public:
        virtual void drive(float speed);
        virtual void brake();
};

class CHN203_ESP32 {
    public:
        CHN203_ESP32(GenericMotor motor, uint8_t c1, uint8_t c2);
        ~CHN203_ESP32();

        float getPosition();
        float getSetPosition();
        void setPosition(float newPosition);
        void rotate(float angle);

        float getSpeed();
        float getSetSpeed();
        void setSpeed(float newSpeed);
    
    protected:
        GenericMotor m_motor;
        uint8_t m_c1;
        uint8_t m_c2;

        float m_position{ 0 };
        float m_setPosition{ 0 };
        float m_speed{ 0 };
        float m_setSpeed{ 0 };

        void normalizePosition();
};
