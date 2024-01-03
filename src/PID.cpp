#include <Arduino.h>
#include <PID.h>

PID::PID(float kp, float ki, float kd, float sampleTime)
{
    m_sampleTime = sampleTime;
    setTuning(kp, ki, kd);
}

void PID::setTuning(float kp = 0, float ki = 0, float kd = 0)
{
    m_kp = kp;

    // Store given parameters.
    m_kiOrg = ki;
    m_kdOrg = kd;

    // Correct tuning according to sample time.
    m_ki = ki * m_sampleTime;
    m_kd = kd / m_sampleTime;
}

bool PID::setSampleTime(float sampleTime)
{
    if (sampleTime <= 0)
        // Sample time must be positive.
        return false;

    // Store new sample time.
    m_sampleTime = sampleTime;

    // Correct tuning according to new sample time.
    setTuning(m_kp, m_kiOrg, m_kdOrg);

    return true;
}

float PID::getSampleTime()
{
    return m_sampleTime;
}

bool PID::setBounds(float lo, float hi)
{
    if (lo >= hi)
    {
        // Lower bound must be smaller than upper bound.
        return false;
    }

    m_lo = lo;
    m_hi = hi;
    return true;
}

void PID::resetBounds()
{
    m_lo = 0;
    m_hi = 0;
}

void PID::setDirection(Direction direction)
{
    m_direction = direction;
}

float PID::compute(float input, float setpoint)
{
    // Error and derivative terms.
    float error = setpoint - input;
    float dInput = input - m_lastInput;

    // Calculate and clamp integral term.
    m_integral += m_ki * error;
    m_integral = clamp(m_integral);

    // Calculate and clamp output.
    float output = (m_kp * error) + m_integral - (m_kd * dInput);
    output = clamp(output);

    // Remember input.
    m_lastInput = input;

    // Invert output if PID in reverse direction.
    if (m_direction == REVERSE)
        output = -output;

    return output;
}

void PID::reset()
{
    m_lastInput = 0;
    m_integral = 0;
}

float PID::clamp(float v)
{
    if (m_lo == m_hi)
    {
        // Bounds not set, no clamping.
        return v;
    }

    // Clamp.
    if (v < m_lo)
        return m_lo;
    if (v > m_hi)
        return m_hi;
    return v;
}

#ifdef AUTO_PID

// TODO: implement AutoPID

#endif