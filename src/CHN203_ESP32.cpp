#include <Arduino.h>
#include <CHN203_ESP32.h>

CHN203_ESP32::CHN203_ESP32(GenericMotor motor, uint8_t c1, uint8_t c2)
{
    m_motor = motor;
    m_motor.brake();

    m_encoder.attachFullQuad(c1, c2);
    m_encoder.clearCount();
}

void CHN203_ESP32::drive(float speed)
{
    m_lock.lock();

    m_controlTaskType = NONE;
    m_motor.drive(speed);

    m_lock.unlock();
}
void CHN203_ESP32::brake()
{
    m_lock.lock();

    m_controlTaskType = NONE;
    m_motor.brake();

    m_lock.unlock();
}

void CHN203_ESP32::setSpeed(float speed)
{
    m_lock.lock();

    m_controlTaskType = SPEED;
    m_setSpeed = speed;

    m_lock.unlock();
}

void CHN203_ESP32::rotate(float angle)
{
    m_lock.lock();

    m_controlTaskType = POSITION;
    m_setPos = angle;
    m_encoder.clearCount();

    m_lock.unlock();
}

void CHN203_ESP32::motorControl(void *board)
{
    while (1)
    {
        CHN203_ESP32 *p_lock = static_cast<struct CHN203_ESP32 *>(board);
        volatile CHN203_ESP32 *p = static_cast<struct CHN203_ESP32 *>(board);

        (p_lock->m_lock).lock();

        if (p->m_controlTaskType == POSITION)
        {
        }
        else if (p->m_controlTaskType == SPEED)
        {
        }

        (p_lock->m_lock).unlock();
    }
}
