#include "ArduPID.h"
#include "ArduHelpers.h"

ArduPID::ArduPID()
{
    
}

void ArduPID::reset()
{
    
}

void ArduPID::setTunings(float _kp, float _ki, float _kd)
{
    // TODO: Force abs() instead of early return?
    if ((_kp < 0) || (_ki < 0) || (_kd < 0))
        return;

    config.kp = _kp;
    config.ki = _ki * (float)config.dt / 1e6;
    config.kd = _kd * 1e6              / (float)config.dt;
}

void ArduPID::setAntiWindupGain(float _kaw)
{
    if (_kaw < 0)
        return;
    
    config.kaw = _kaw * (float)config.dt / 1e6;
}

void ArduPID::setPMode(PMode _pMode)
{
    config.pMode = _pMode;

    if (_pMode == PMode::P_ON_E)
        config.pModeMix = 1.0;
    else
        config.pModeMix = 0.0;
}

void ArduPID::setPercentPonE(float p)
{
    config.pModeMix = constrain_val(p / 100.0, 0, 1);
}

void ArduPID::setPercentPonM(float p)
{
    config.pModeMix = constrain_val(1.0 - (p / 100.0), 0, 1);
}

void ArduPID::setDMode(DMode _dMode)
{
    config.dMode = _dMode;
}

void ArduPID::setAntiWindMode(AntiWindMode _aWMode)
{
    config.aWMode = _aWMode;
}

void ArduPID::setOutputLimits(float min, float max)
{
    if (max > min)
    {
        config.outMin = min;
        config.outMax = max;
    }
}

void ArduPID::setPLimits(float min, float max)
{
    if (max > min)
    {
        config.pMin = min;
        config.pMax = max;
    }
}

void ArduPID::setILimits(float min, float max)
{
    if (max > min)
    {
        config.iMin = min;
        config.iMax = max;
    }
}

void ArduPID::setDLimits(float min, float max)
{
    if (max > min)
    {
        config.dMin = min;
        config.dMax = max;
    }
}

void ArduPID::setDirection(Direction dir)
{
    config.direction = dir;
}

void ArduPID::setDtUs(uint32_t _dt)
{
    config.dt = _dt;
}

void ArduPID::setDtMs(float _dt)
{
    if (_dt > 0)
        config.dt = _dt * 1e3;
}

void ArduPID::setDtS(float _dt)
{
    if (_dt > 0)
        config.dt = _dt * 1e6;
}

void ArduPID::setSetpoint(float _setpoint)
{
    signals.setpoint = _setpoint;
}

float ArduPID::compute(float input)
{
    signals.input = input;
    signals.currT = micros();

    if ((signals.currT - signals.prevT) < config.dt)
        return signals.out;

    signals.prevT = signals.currT;

    signals.error = signals.setpoint - signals.input;
    float dInput  = signals.input    - signals.prevInput;
    float dError  = signals.error    - signals.prevError;




    // TODO: incorporate mode mixing
    if (config.pMode == PMode::P_ON_E)
        signals.p = config.kp * signals.error;
    else
        signals.p -= config.kp * dInput;
    
    if (config.pMin != config.pMax)
        signals.p = constrain_val(signals.p, config.pMin, config.pMax);




    signals.i += config.ki * signals.error;

    if (config.aWMode == AntiWindMode::BACK_CALC)
    {
        float dActuator = signals.out - signals.input;
        signals.i -= config.kaw * dActuator;
    }
    else if (config.aWMode == AntiWindMode::CLEGG)
    {
        if ((signals.error == 0)                              ||
            ((signals.error < 0) && (signals.prevError >= 0)) ||
            ((signals.error > 0) && (signals.prevError <= 0)))
            signals.i = 0;
    }

    if (config.iMin != config.iMax)
        signals.i = constrain_val(signals.i, config.iMin, config.iMax);




    if (config.dMode == DMode::D_ON_E)
        signals.d = -config.kd * dError;
    else
        signals.d = -config.kd * dInput;

    if (config.dMin != config.dMax)
        signals.d = constrain_val(signals.d, config.dMin, config.dMax);
    



    signals.out = signals.p + signals.i + signals.d;

    if (config.outMin != config.outMax)
        signals.out = constrain_val(signals.out, config.outMin, config.outMax);
    



    signals.prevError = signals.error;
    signals.prevInput = signals.input;

    return signals.out;
}

float ArduPID::getkp()       const { return config.kp;                                       }
float ArduPID::getki()       const { return config.ki / (float)config.dt * 1e6;              }
float ArduPID::getkd()       const { return config.kd / 1e6              * (float)config.dt; }
float ArduPID::getP()        const { return signals.p;                                       }
float ArduPID::getI()        const { return signals.i;                                       }
float ArduPID::getD()        const { return signals.d;                                       }
float ArduPID::getInput()    const { return signals.input;                                   }
float ArduPID::getSetpoint() const { return signals.setpoint;                                }
float ArduPID::getOutput()   const { return signals.out;                                     }

void ArduPID::debug(      Stream& s,
                    const char*   name,
                          uint16_t mask,
                          float   setpoint,
                          float   input,
                          float   output)
{
    // s.print(name);
    // s.print(" ");

    // if (mask & PRINT_INPUT)    { s.print("IN: ");  s.print(input);    s.print(" "); }
    // if (mask & PRINT_OUTPUT)   { s.print("OUT: "); s.print(out);      s.print(" "); }
    // if (mask & PRINT_SETPOINT) { s.print("SP: ");  s.print(setpoint); s.print(" "); }
    // if (mask & PRINT_P)        { s.print("P: ");   s.print(_pOut);    s.print(" "); }
    // if (mask & PRINT_I)        { s.print("I: ");   s.print(_iOut);    s.print(" "); }
    // if (mask & PRINT_D)        { s.print("D: ");   s.print(_dOut);    s.print(" "); }

    // s.println();
}
