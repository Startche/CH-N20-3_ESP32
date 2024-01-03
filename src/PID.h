#pragma once

/*
 * Based on the Arduino PID Library by Brett Beauregard:
 * <https://github.com/br3ttb/Arduino-PID-Library/tree/master>
 * And the accompanying writeup:
 * <http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/>
 */

// TODO: separate into different library.

class PID
{
public:
    PID(float kp = 0, float ki = 0, float kd = 0, float sampleTime = 20e-3);

    // Set controller tuning.
    void setTuning(float kp, float ki, float kd);

    // Change sample time.
    // Return true if sample time is updated, false otherwise.
    // Sample time must be positive.
    bool setSampleTime(float sampleTime);
    float getSampleTime();

    // Set minimum and maximum control values or remove bounds.
    // setBounds returns true if bounds are updated, false otherwise.
    // For bounds to be updated, hi > lo must be true.
    bool setBounds(float lo, float hi);
    void resetBounds();

    enum Direction
    {
        DIRECT,  // Positive input -> Positive output.
        REVERSE, // Positive input -> negative output.
    };

    // Change direction.
    void setDirection(Direction direction);

    // Compute next control value given input and setpoint.
    // Assumes constant time elapsed between computations.
    float compute(float input, float setpoint);

    // Resets internal variables.
    void reset();

private:
    // Parameters given by user.
    float m_kiOrg;
    float m_kdOrg;

    // Corrected parameters.
    float m_kp;
    float m_ki;
    float m_kd;

    // Sample time, in seconds.
    float m_sampleTime;

    // Direction.
    Direction m_direction = DIRECT;

    // Bounds.
    float m_lo = 0;
    float m_hi = 0;

    // Last seen input.
    float m_lastInput = 0;

    // Integral term.
    float m_integral = 0;

    // Clamp value to bounds.
    float clamp(float v);
};

#ifdef INC_TASK_H
/*
 * If FreeRTOS tasks header included, define AutoPID.
 * AutoPID automatically reads inputs and setpoints and
 * writes the output at the correct intervals.
 */
#define AUTO_PID

// TODO: implement AutoPID

class AutoPID : PID
{
};

#endif