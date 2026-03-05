#include "ArduPID.h"
#include "ArduHelpers.h"

ArduPID::ArduPID()
{
    
}

void ArduPID::reset()
{
    // TODO: better handle kick due to reset
    signals.error     = 0.0;
    signals.prevError = 0.0;
    signals.dError    = 0.0;

    signals.input     = 0.0;
    signals.prevInput = 0.0;
    signals.dInput    = 0.0;

    signals.p = 0.0;
    signals.i = 0.0;
    signals.d = 0.0;

    signals.out = 0.0;

    signals.prevT = micros();
    signals.currT = signals.prevT;
}

void ArduPID::setConfig(const Config& _config)
{
    config = _config;
}

void ArduPID::setName(const char* _name)
{
    if (_name == nullptr)
        return;

    strncpy(config.name,
            _name,
            sizeof(config.name) - 1);

    config.name[sizeof(config.name) - 1] = '\0';
}

void ArduPID::setTunings(float _kp, float _ki, float _kd)
{
    config.kp = fabs(_kp);
    config.ki = fabs(_ki);
    config.kd = fabs(_kd);
}

void ArduPID::setAntiWindupGain(float _kaw)
{
    if (_kaw < 0)
        return;
    
    config.kaw = _kaw;
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
    if (config.direction != dir)
    {
        config.direction = dir;
        reset();
    }
}

void ArduPID::setDtUs(uint32_t _dt)
{
    if (config.dt != _dt)
    {
        config.dt  = _dt;
        config.dts = (float)config.dt / 1e6;
        reset();
    }
}

void ArduPID::setDtMs(float _dt)
{
    if ((_dt > 0) && (config.dt != (uint32_t)(_dt * 1e3)))
    {
        config.dt  = _dt * 1e3;
        config.dts = (float)config.dt / 1e6;
        reset();
    }
}

void ArduPID::setDtS(float _dt)
{
    if ((_dt > 0) && (config.dt != (uint32_t)(_dt * 1e6)))
    {
        config.dt  = _dt * 1e6;
        config.dts = (float)config.dt / 1e6;
        reset();
    }
}

void ArduPID::setSetpoint(float _setpoint)
{
    signals.setpoint = _setpoint;
}

void ArduPID::computeP()
{
    // TODO: incorporate mode mixing
    if (config.pMode == PMode::P_ON_E)
        signals.p = config.direction * config.kp * signals.error;
    else
        signals.p -= config.direction * config.kp * signals.dInput;
    
    if (config.pMin != config.pMax)
        signals.p = constrain_val(signals.p, config.pMin, config.pMax);
}

void ArduPID::computeI()
{
    signals.i += config.direction * config.ki * config.dts * signals.error;

    if ((config.aWMode == AntiWindMode::BACK_CALC) && (config.outMin != config.outMax))
    {
        float outunsat  = signals.p + signals.i + signals.d;
        float outsat    = constrain_val(outunsat, config.outMin, config.outMax);
        float dActuator = outunsat - outsat;
        
        signals.i -= config.direction * config.kaw * dActuator;
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
}

void ArduPID::computeD()
{
    if (config.dMode == DMode::D_ON_E)
        signals.d = -((config.direction * config.kd) / config.dts) * signals.dError;
    else
        signals.d = -((config.direction * config.kd) / config.dts) * signals.dInput;

    if (config.dMin != config.dMax)
        signals.d = constrain_val(signals.d, config.dMin, config.dMax);
}

float ArduPID::compute(float input)
{
    signals.input = input;
    signals.currT = micros();

    if ((signals.currT - signals.prevT) < config.dt)
        return signals.out;

    signals.prevT = signals.currT;

    signals.error  = signals.setpoint - signals.input;
    signals.dInput = signals.input    - signals.prevInput;
    signals.dError = signals.error    - signals.prevError;

    computeP();
    computeD();
    computeI(); // Compute I last to make back-calc anti-windup easier to implement
    
    signals.out = signals.p + signals.i + signals.d;

    if (config.outMin != config.outMax)
        signals.out = constrain_val(signals.out, config.outMin, config.outMax);
    
    signals.prevError = signals.error;
    signals.prevInput = signals.input;

    return signals.out;
}

ArduPID::Config  ArduPID::getConfig()  const { return config;  }
ArduPID::Signals ArduPID::getSignals() const { return signals; }

void ArduPID::debug(uint16_t mask,
                    Stream&  stream)
{
    if (mask & PRINT_INPUT)    { stream.print(config.name);  stream.print("_IN ");  }
    if (mask & PRINT_OUTPUT)   { stream.print(config.name);  stream.print("_OUT "); }
    if (mask & PRINT_SETPOINT) { stream.print(config.name);  stream.print("_SP ");  }
    if (mask & PRINT_KP)       { stream.print(config.name);  stream.print("_kp ");  }
    if (mask & PRINT_KI)       { stream.print(config.name);  stream.print("_ki ");  }
    if (mask & PRINT_KD)       { stream.print(config.name);  stream.print("_kd ");  }
    if (mask & PRINT_P)        { stream.print(config.name);  stream.print("_P ");   }
    if (mask & PRINT_I)        { stream.print(config.name);  stream.print("_I ");   }
    if (mask & PRINT_D)        { stream.print(config.name);  stream.print("_D ");   }

    if (mask & PRINT_INPUT)    { stream.print(signals.input);    stream.print(" "); }
    if (mask & PRINT_OUTPUT)   { stream.print(signals.out);      stream.print(" "); }
    if (mask & PRINT_SETPOINT) { stream.print(signals.setpoint); stream.print(" "); }
    if (mask & PRINT_KP)       { stream.print(config.kp);        stream.print(" "); }
    if (mask & PRINT_KI)       { stream.print(config.ki);        stream.print(" "); }
    if (mask & PRINT_KD)       { stream.print(config.kd);        stream.print(" "); }
    if (mask & PRINT_P)        { stream.print(signals.p);        stream.print(" "); }
    if (mask & PRINT_I)        { stream.print(signals.i);        stream.print(" "); }
    if (mask & PRINT_D)        { stream.print(signals.d);        stream.print(" "); }

    stream.println();
}
