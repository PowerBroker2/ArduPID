#include "ArduPID.h"

TeensyPID pid;

float setpoint = 100.0f;
float input = 0.0f;
float output = 0.0f;

IntervalTimer controlTimer;

void controlLoop()
{
    input += output * 0.01f;
    output = pid.compute(setpoint, input);
}

void setup()
{
    Serial.begin(115200);
    delay(1000);

    pid.begin(
        2.0f,
        1.0f,
        0.1f,
        0.001f,
        -255.0f,
        255.0f,
        TeensyPID::P_ON_E,
        TeensyPID::D_ON_M,
        TeensyPID::FORWARD,
        0.0f
    );

    pid.setWindupLimits(-200.0f, 200.0f);
    pid.setDeadband(-0.01f, 0.01f);

    // Choose anti-windup method here:
    pid.setAntiWindupMode(TeensyPID::CLAMP_HEADROOM);
    // pid.setAntiWindupMode(TeensyPID::CONDITIONAL_INTEGRATION);
    // pid.setAntiWindupMode(TeensyPID::BACK_CALCULATION);
    // pid.setBackCalculationGain(0.5f);

    controlTimer.begin(controlLoop, 1000);
}

void loop()
{
    static elapsedMillis timer;

    if (timer > 100)
    {
        timer = 0;

        pid.debug(Serial,
                  "PID",
                  TeensyPID::PRINT_INPUT |
                  TeensyPID::PRINT_OUTPUT |
                  TeensyPID::PRINT_P |
                  TeensyPID::PRINT_I |
                  TeensyPID::PRINT_D,
                  setpoint,
                  input,
                  output);
    }
}
