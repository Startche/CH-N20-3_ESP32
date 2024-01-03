#include <Arduino.h>
#include <CHN203_ESP32.h>

// Number of encoder counts per turn
#define ENC_COUNT 28

float clamp(float v, float lo, float hi)
{
    // Clamp value between minimum and maximum.
    if (v < lo)
        return lo;
    if (v > hi)
        return hi;
    return v;
}
float clamp_speed(float s) { return clamp(s, -1, 1); }

CHN203::CHN203(GenericDriver *motor, uint8_t c1, uint8_t c2, float ratio) : m_motor(motor),
                                                                            m_ratio(ratio)
{
    m_encoder.attachFullQuad(c1, c2);
    m_encoder.clearCount();
}

void CHN203::drive(float speed)
{
    m_lock.lock();

    speed = clamp_speed(speed);
    m_currentTask = NONE;
    m_motor->drive(speed);

    m_lock.unlock();
}
void CHN203::brake()
{
    m_lock.lock();

    m_currentTask = NONE;
    m_motor->brake();

    m_lock.unlock();
}

void CHN203::setSpeed(float speed)
{
    m_lock.lock();

    m_currentTask = SPEED;
    m_setSpeed = speed;

    m_lock.unlock();
}

void CHN203::rotate(float angle)
{
    m_lock.lock();

    m_currentTask = POSITION;
    m_setPosition = angle;

    m_lock.unlock();
}

float CHN203::getPosition()
{
    m_lock.lock();

    float angle = (float)m_encoder.getCount() / (ENC_COUNT * m_ratio);

    m_lock.unlock();

    return angle;
}

float CHN203::getPositionDeg() { return 360 * getPosition(); }

float CHN203::getPositionRad() { return 2 * M_PI * getPosition(); }

float CHN203::getSpeed()
{
    m_lock.lock();

    float speed = m_speed;

    m_lock.unlock();

    return speed;
}

void CHN203::motorControl(void *board)
{
    CHN203::ControlTaskType lastTask = NONE;

    // Controller parameters.

    // Position.

    // Speed.

    while (1)
    {
        CHN203 *p = static_cast<struct CHN203 *>(board);

        (p->m_lock).lock();

        CHN203::ControlTaskType task = p->m_currentTask;

        if (lastTask != task)
        {
            // TODO: cleanup.
            lastTask = task;
        }

        if (p->m_currentTask == POSITION)
        {
            // TODO: control position.
        }
        else if (p->m_currentTask == SPEED)
        {
            // TODO: control speed.
        }

        (p->m_lock).unlock();
    }
}
