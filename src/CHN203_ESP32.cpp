#include <Arduino.h>
#include <CHN203_ESP32.h>

CHN203_ESP32::CHN203_ESP32(GenericMotor motor, uint8_t c1, uint8_t c2) {
    m_motor = motor;
    m_c1 = c1;
    m_c2 = c2;

    m_motor.brake();
    pinMode(m_c1, INPUT);
    pinMode(m_c2, INPUT);

    //TODO: setup interrupts.
}

float CHN203_ESP32::getPosition() {
    return m_position;
}

float CHN203_ESP32::getSetPosition() {
    return m_setPosition;
}

void CHN203_ESP32::setPosition(float newPosition) {
    m_setPosition = newPosition;
    normalizePosition();
}

void CHN203_ESP32::rotate(float angle) {
    m_setPosition += angle;
    normalizePosition();
}

float CHN203_ESP32::getSpeed() {
    return m_speed;
}

float CHN203_ESP32::getSetSpeed() {
    return m_setSpeed;
}

void CHN203_ESP32::normalizePosition() {
    //TODO: normalize positions (-pi <= p < pi)
}
