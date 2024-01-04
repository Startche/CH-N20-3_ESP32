#include <Arduino.h>
#include <CHN203_ESP32.h>

// Number of encoder counts per turn
const int ENCODER_COUNTS = 28;

CHN203::CHN203(GenericDriver *motor, uint8_t c1, uint8_t c2, float ratio)
{
    m_motor = motor;
    m_ratio = ratio;

    // Initialize encoder.
    m_encoder.attachFullQuad(c1, c2);
    m_encoder.clearCount();

    // Initialize PID controllers.
    m_pidPosition.setSampleTime(m_interval / 1000.0);
    m_pidSpeed.setSampleTime(m_interval / 1000.0);

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

/*
 * Raw motor functions.
 */

void CHN203::drive(float speed)
{
    lock();

    m_currentTask = NONE;

    unlock();

    // Clamp speed and apply it.
    if (speed < GenericDriver::MIN_SPEED)
    {
        speed = GenericDriver::MIN_SPEED;
    }
    else if (speed > GenericDriver::MAX_SPEED)
    {
        speed = GenericDriver::MAX_SPEED;
    }
    m_motor->drive(speed);
}
void CHN203::brake()
{
    lock();

    m_currentTask = NONE;

    unlock();

    m_motor->brake();
}

/*
 * PID controller tuning.
 */

void CHN203::tunePositionPID(float kp, float ki, float kd)
{
    kp = countsToRotation(kp);
    ki = countsToRotation(ki);
    kd = countsToRotation(kd);

    lock();

    m_pidPosition.setTuning(kp, ki, kd);

    unlock();
}

void CHN203::tuneSpeedPID(float kp, float ki, float kd)
{
    // TODO: adjust to take window into account.
    kp = countsToRotation(kp);
    ki = countsToRotation(ki);
    kd = countsToRotation(kd);

    lock();

    m_pidSpeed.setTuning(kp, ki, kd);

    unlock();
}

/*
 * Rotation control functions.
 */

void CHN203::rotate(float turn)
{
    // Convert turn into encoder counts.
    long counts = rotationToCounts(turn);

    lock();

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

    unlock();
}
void CHN203::rotateRad(float angle) { rotate(angle / M_2_PI); }
void CHN203::rotateDeg(float angle) { rotate(angle / 360.0); }

float CHN203::getPosition()
{
    lock();

    float turn = countsToRotation(m_encoder.getCount());

    unlock();

    return turn;
}
float CHN203::getPositionDeg() { return 360.0 * getPosition(); }
float CHN203::getPositionRad() { return M_2_PI * getPosition(); }

/*
 * Speed control functions.
 */

void CHN203::setSpeed(float speed)
{
    // Convert speed to amount of counts per second.
    long countsPerSecond = rotationToCounts(speed);

    lock();

    m_currentTask = SPEED;
    m_setCountsPerSecond = countsPerSecond;

    unlock();
}
void CHN203::setSpeedRad(float speed) { setSpeed(speed / M_2_PI); }
void CHN203::setSpeedDeg(float speed) { setSpeed(speed / 360.0); }
void CHN203::setSpeedRPM(float speed) { setSpeed(speed / 60.0); }

float CHN203::getSpeed()
{
    lock();

    float speed = m_speed;

    unlock();

    return speed;
}
float CHN203::getSpeedRad() { return getSpeed() * M_2_PI; }
float CHN203::getSpeedDeg() { return getSpeed() * 360.0; }
float CHN203::getSpeedRPM() { return getSpeed() * 60.0; }

/*
 * Helper functions.
 */

float CHN203::countsToRotation(float counts)
{
    return counts / (ENCODER_COUNTS * m_ratio);
}

float CHN203::rotationToCounts(float rotation)
{
    return rotation * ENCODER_COUNTS * m_ratio;
}

void CHN203::lock()
{
    m_lock.lock();
}

void CHN203::unlock()
{
    m_lock.unlock();
}

/*
 * Main controller function.
 */

void CHN203::motorControl(void *board)
{
    // Get board pointer.
    CHN203 *volatile p = static_cast<struct CHN203 *volatile>(board);

    // Last task being executed.
    CHN203::ControlTaskType lastTask = NONE;

    // Speed calculation.
    const int WINDOW = 10;
    int windowIndex = 0;
    long countsHistory[WINDOW] = {0};

    // Execution frequency parameters. See:
    // <https://www.freertos.org/xtaskdelayuntiltask-control.html>
    const TickType_t xFrequency = pdMS_TO_TICKS(p->m_intervalMs);
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1)
    {
        // Wait for the next cycle.
        xTaskDelayUntil(&xLastWakeTime, xFrequency);

        // Lock board.
        p->lock();

        // Get current task.
        CHN203::ControlTaskType task = p->m_currentTask;

        if (lastTask != task)
        {
            // Reset control parameters after task change.
            lastTask = task;
            p->m_pidPosition.reset();
            p->m_pidSpeed.reset();
        }

        // Get current counts and counts change during window.
        long counts = p->m_encoder.getCount();
        long dt = counts - countsHistory[windowIndex];

        if (task == POSITION)
        {
            // Position control.
            long setCounts = p->m_setCounts;
            float output = p->m_pidPosition.compute(counts, setCounts);

            const long MAX_ERROR = 1;
            long error = setCounts - counts;
            if (-MAX_ERROR <= error && error <= MAX_ERROR)
            {
                // If error is small enough, brake motor.
                p->m_motor->brake();
            }
            else
            {
                // If error is too great, keep driving.
                p->m_motor->drive(output);
            }
        }
        else if (task == SPEED)
        {
            // Speed control.
            float setpoint = p->m_setCountsPerSecond * WINDOW * p->m_interval;
            float output = p->m_pidSpeed.compute(dt, setpoint);
            p->m_motor->drive(output);
        }

        // Calculate speed.
        float countsPerSecond = dt / (WINDOW * p->m_interval);
        p->m_speed = p->countsToRotation(countsPerSecond);

        // Unlock board.
        p->unlock();

        // Update counts history.
        countsHistory[windowIndex] = counts;
        windowIndex++;
        if (windowIndex >= WINDOW)
        {
            windowIndex = 0;
        }
    }
}
