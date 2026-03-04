#include "ArduPID.h"
#include "FiltAndCal.h"

float fs = 100.0f; // sample rate

ArduPID pid;
ButterworthFilter sys;

void setup()
{
    Serial.begin(115200);
    delay(1000);

    pid.setSetpoint(1);
    pid.setDtS(0.1);
    pid.setTunings(2, 1, 1);

    sys.configureLPF(1, fs, 0.1);
}

void loop()
{
    float out    = pid.getSignals().out;
    float sysOut = sys.process(out);
    pid.compute(sysOut);
    pid.debug();

    delay((1.0 / fs) * 1e3);
}
