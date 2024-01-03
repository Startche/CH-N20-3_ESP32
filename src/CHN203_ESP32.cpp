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

CHN203::CHN203(GenericDriver *motor, uint8_t c1, uint8_t c2, float ratio)
{
    m_motor = motor;

    // Initialize encoder.
    m_encoder.attachFullQuad(c1, c2);
    m_encoder.clearCount();

    m_ratio = ratio;

    // Create control task.
    xTaskCreate(CHN203::motorControl, // Control function.
                "CHN203Control",      // Name.
                2048,                 // Stack size.
                this,                 // Parameter.
                tskIDLE_PRIORITY + 1, // Priority.
                &m_xHandle            // Task handle.
    );

    // Assert control task was created.
    configASSERT(m_xHandle);
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

void CHN203::rotate(float turn)

{
    // Convert turn into encoder counts.
    long counts = turn * ENC_COUNT * m_ratio;

    m_lock.lock();

    // Calculate new set encoder count and store it.
    if (m_currentTask == POSITION)
    {
        // If task is already POSITION, update set point based on
        // old set point i.e. multiple rotate calls should act as a single
        // call with the total angle (rotate(1) and then rotate(2)
        // should be equivalente to rotate(3)).
        m_setCounts += counts;
    }
    else
    {
        // For other tasks, make setpoint based on current position.
        m_setCounts = counts + m_encoder.getCount();
    }

    m_currentTask = POSITION;

    m_lock.unlock();
}

void CHN203::rotateRad(float angle) { rotate(angle / M_2_PI); }

void CHN203::rotateDeg(float angle) { rotate(angle / 360.0); }

float CHN203::getPosition()
{
    m_lock.lock();

    float turn = (float)m_encoder.getCount() / (ENC_COUNT * m_ratio);

    m_lock.unlock();

    return turn;
}

float CHN203::getPositionDeg() { return 360.0 * getPosition(); }

float CHN203::getPositionRad() { return M_2_PI * getPosition(); }

void CHN203::setSpeed(float speed)
{
    m_lock.lock();

    m_currentTask = SPEED;
    m_setSpeed = speed;

    m_lock.unlock();
}

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
    const long xIntervalMs = 20;

    // Position.

    // Speed.

    // Execution frequency parameters. See:
    // <https://www.freertos.org/xtaskdelayuntiltask-control.html>
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(xIntervalMs);
    BaseType_t xWasDelayed;

    xLastWakeTime = xTaskGetTickCount();
    while (1)
    {
        // Wait for the next cycle.
        xWasDelayed = xTaskDelayUntil(&xLastWakeTime, xFrequency);
        if (xWasDelayed)
        {
            xLastWakeTime = xTaskGetTickCount();
        }

        // Get board pointer.
        CHN203 *volatile p = static_cast<struct CHN203 *>(board);

        // Lock board.
        (p->m_lock).lock();

        CHN203::ControlTaskType task = p->m_currentTask;

        if (lastTask != task)
        {
            // Reset control parameters.
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

        // Unlock board.
        (p->m_lock).unlock();
    }
}
