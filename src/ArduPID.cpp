#include "ArduPID.h"

TeensyPID::TeensyPID()
{
    _kp = _ki = _kd = 0.0f;
    _dt = 0.001f;

    _integral = 0.0f;
    _prevError = 0.0f;
    _prevInput = 0.0f;

    _pOut = _iOut = _dOut = 0.0f;

    _bias = 0.0f;

    _outMin = -1.0f;
    _outMax = 1.0f;

    _windMin = -1000.0f;
    _windMax = 1000.0f;

    _deadMin = 0.0f;
    _deadMax = 0.0f;

    _antiWindupMode = CLAMP_HEADROOM;
    _kaw = 1.0f;

    _direction = FORWARD;

    setModes(P_ON_E, D_ON_M);
}

void TeensyPID::begin(float kp,
                      float ki,
                      float kd,
                      float dtSeconds,
                      float outMin,
                      float outMax,
                      PMode pMode,
                      DMode dMode,
                      Direction direction,
                      float bias)
{
    setDt(dtSeconds);
    setDirection(direction);
    setBias(bias);
    setOutputLimits(outMin, outMax);
    setTunings(kp, ki, kd);
    setModes(pMode, dMode);
    reset();
}

void TeensyPID::reset()
{
    _integral = 0.0f;
    _prevError = 0.0f;
    _prevInput = 0.0f;
}

void TeensyPID::setTunings(float kp, float ki, float kd)
{
    _kp = kp * (float)_direction;
    _ki = ki * _dt * (float)_direction;
    _kd = kd / _dt * (float)_direction;
}

void TeensyPID::setModes(PMode pMode, DMode dMode)
{
    _computeP = (pMode == P_ON_E)
        ? &TeensyPID::pOnError
        : &TeensyPID::pOnMeasurement;

    _computeD = (dMode == D_ON_E)
        ? &TeensyPID::dOnError
        : &TeensyPID::dOnMeasurement;
}

void TeensyPID::setOutputLimits(float min, float max)
{
    if (max > min)
    {
        _outMin = min;
        _outMax = max;
    }
}

void TeensyPID::setWindupLimits(float min, float max)
{
    if (max > min)
    {
        _windMin = min;
        _windMax = max;
    }
}

void TeensyPID::setDeadband(float min, float max)
{
    if (max >= min)
    {
        _deadMin = min;
        _deadMax = max;
    }
}

void TeensyPID::setBias(float bias)
{
    _bias = bias;
}

void TeensyPID::setDirection(Direction dir)
{
    _direction = dir;
}

void TeensyPID::setDt(float dtSeconds)
{
    if (dtSeconds > 0)
        _dt = dtSeconds;
}

void TeensyPID::setAntiWindupMode(AntiWindupMode mode)
{
    _antiWindupMode = mode;
}

void TeensyPID::setBackCalculationGain(float kaw)
{
    if (kaw >= 0)
        _kaw = kaw;
}

float TeensyPID::compute(float setpoint, float input)
{
    float error = setpoint - input;

    if (error >= _deadMin && error <= _deadMax)
        error = 0.0f;

    float dInput = input - _prevInput;
    float dError = error - _prevError;

    _pOut = (this->*_computeP)(error, dInput);
    _dOut = (this->*_computeD)(dError, dInput);

    float iTemp = _integral + 0.5f * _ki * (error + _prevError);
    iTemp = constrain(iTemp, _windMin, _windMax);

    float baseOutput = _bias + _pOut + _dOut;
    float output = baseOutput + iTemp;

    if (_antiWindupMode == CLAMP_HEADROOM)
    {
        float iMax = _outMax - baseOutput;
        float iMin = _outMin - baseOutput;
        iTemp = constrain(iTemp, iMin, iMax);
        output = baseOutput + iTemp;
        _integral = iTemp;
    }
    else if (_antiWindupMode == CONDITIONAL_INTEGRATION)
    {
        if (output <= _outMax && output >= _outMin)
            _integral = iTemp;

        output = baseOutput + _integral;
    }
    else if (_antiWindupMode == BACK_CALCULATION)
    {
        float unclamped = output;

        if (output > _outMax)
            output = _outMax;
        else if (output < _outMin)
            output = _outMin;

        float correction = _kaw * (output - unclamped);
        _integral = iTemp + correction;
    }

    output = constrain(output, _outMin, _outMax);

    _iOut = _integral;

    _prevError = error;
    _prevInput = input;

    return output;
}

// --- Mode Implementations ---

float TeensyPID::pOnError(float error, float)
{
    return _kp * error;
}

float TeensyPID::pOnMeasurement(float, float dInput)
{
    return -_kp * dInput;
}

float TeensyPID::dOnError(float dError, float)
{
    return -_kd * dError;
}

float TeensyPID::dOnMeasurement(float, float dInput)
{
    return -_kd * dInput;
}

// --- Accessors ---

float TeensyPID::P() const { return _pOut; }
float TeensyPID::I() const { return _iOut; }
float TeensyPID::D() const { return _dOut; }
float TeensyPID::Bias() const { return _bias; }

// --- Debug ---

void TeensyPID::debug(Stream& s,
                      const char* name,
                      uint8_t mask,
                      float setpoint,
                      float input,
                      float output)
{
    s.print(name);
    s.print(" ");

    if (mask & PRINT_INPUT)    { s.print("IN: ");  s.print(input);    s.print(" "); }
    if (mask & PRINT_OUTPUT)   { s.print("OUT: "); s.print(output);   s.print(" "); }
    if (mask & PRINT_SETPOINT) { s.print("SP: ");  s.print(setpoint); s.print(" "); }
    if (mask & PRINT_BIAS)     { s.print("B: ");   s.print(_bias);    s.print(" "); }
    if (mask & PRINT_P)        { s.print("P: ");   s.print(_pOut);    s.print(" "); }
    if (mask & PRINT_I)        { s.print("I: ");   s.print(_iOut);    s.print(" "); }
    if (mask & PRINT_D)        { s.print("D: ");   s.print(_dOut);    s.print(" "); }

    s.println();
}
