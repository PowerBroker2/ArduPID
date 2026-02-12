#include "ArduPID.h"




void ArduPID::begin(      float*       _input,
                          float*       _output,
                          float*       _setpoint,
                          float        _p,
                          float        _i,
                          float        _d,
                    const pOn&         _pOn,
                    const dOn&         _dOn,
                    const dir&         _direction,
                          unsigned int _minSamplePeriodMs,
                          float        _bias)
{
    input    = _input;
    output   = _output;
    setpoint = _setpoint;

    setCoefficients(_p, _i, _d);
    setPOn(_pOn);
    setDOn(_dOn);
    setBias(_bias);
    setDirection(_direction);
    setSampleTime(_minSamplePeriodMs);

    modeType = ON;
    reset();
}





void ArduPID::start()
{
    if (modeType != ON)
    {
        modeType = ON;
        reset();
    }
}




void ArduPID::reset()
{
    curError    = 0;
    curSetpoint = 0;
    curInput    = 0;

    lastError    = 0;
    lastSetpoint = 0;
    lastInput    = 0;

    pOut = 0;
    iOut = 0;
    dOut = 0;

    timer.start();
}





void ArduPID::stop()
{
    if (modeType != OFF)
        modeType = OFF;
}




void ArduPID::doCompute(uint32_t timeDiff)
{
    kp = pIn;

    if (timeDiff > 0)
    {
        ki = iIn * (timeDiff / 1000.0);
        kd = dIn * (1000.0 / timeDiff); // go to inf if timeDiff == 0
    }
    else
    {
        ki = 0.0;
        kd = 0.0;
    }

    if (direction == BACKWARD)
    {
        kp *= -1;
        ki *= -1;
        kd *= -1;
    }

    lastInput    = curInput;
    lastSetpoint = curSetpoint;
    lastError    = curError;

    curInput    = *input;
    curSetpoint = *setpoint;
    curError    = curSetpoint - curInput;

    float dInput = *input - lastInput;
    float dError = curError - lastError;

    if (pOnType == P_ON_E)
        pOut = kp * curError;
    else if (pOnType == P_ON_M)
        pOut = -kp * dInput;

    if (dOnType == D_ON_E)
        dOut = -kd * dError;
    else if (dOnType == D_ON_M)
        dOut = -kd * dInput;

    float iTemp = (iIn == 0.0) ? 0.0 : iOut + (ki * ((curError + lastError) / 2.0)); // Trapezoidal integration
    iTemp       = constrain(iTemp, windupMin, windupMax);       // Prevent integral windup

    float outTemp = bias + pOut + dOut;                           // Output without integral
    float iMax    = constrain(outputMax - outTemp, 0, outputMax); // Maximum allowed integral term before saturating output
    float iMin    = constrain(outputMin - outTemp, outputMin, 0); // Minimum allowed integral term before saturating output

    iOut = constrain(iTemp, iMin, iMax);

    outTemp += iOut;
    outTemp  = constrain(outTemp, outputMin, outputMax);
    *output  = outTemp;

}




void ArduPID::compute()
{
    if (timer.fire() && modeType == ON)
    {
        doCompute(timer.timeDiff);
    }
}




void ArduPID::setOutputLimits(float min,
                              float max)
{
    if (max > min)
    {
        outputMax = max;
        outputMin = min;

        if (modeType == ON)
            *output = constrain(*output, outputMin, outputMax);
    }
}




void ArduPID::setWindUpLimits(float min,
                              float max)
{
    if (max > min)
    {
        windupMax = max;
        windupMin = min;
    }
}




void ArduPID::setDeadBand(float min,
                          float max)
{
    if (max >= min)
    {
        deadBandMax = max;
        deadBandMin = min;
    }
}




void ArduPID::setPOn(const pOn& _pOn)
{
    pOnType = _pOn;
}




void ArduPID::setDOn(const dOn& _dOn)
{
    dOnType = _dOn;
}




void ArduPID::setBias(float _bias)
{
    bias = _bias;
}




void ArduPID::setCoefficients(float _p,
                              float _i,
                              float _d)
{
    if (_p >= 0 && _i >= 0 && _d >= 0)
    {
        pIn = _p;
        iIn = _i;
        dIn = _d;
    }
}




void ArduPID::setDirection(const dir& _direction)
{
    direction = _direction;

    if (modeType == ON)
        reset();
}




void ArduPID::reverse()
{
    if (direction == FORWARD)
        direction = BACKWARD;
    else if (direction == BACKWARD)
        direction = FORWARD;

    if (modeType == ON)
        reset();
}




void ArduPID::setSampleTime(unsigned int _minSamplePeriodMs)
{
    timer.begin(_minSamplePeriodMs);
}




float ArduPID::B()
{
    return bias;
}




float ArduPID::P()
{
    return pOut;
}




float ArduPID::I()
{
    return iOut;
}




float ArduPID::D()
{
    return dOut;
}




void ArduPID::debug(      Stream* stream,
                    const char*   controllerName,
                          byte    mask)
{
    if (mask & PRINT_INPUT)
    {
        stream->print(controllerName);
        stream->print("_input ");
    }
    
    if (mask & PRINT_OUTPUT)
    {
        stream->print(controllerName);
        stream->print("_output ");
    }
        
    if (mask & PRINT_SETPOINT)
    {
        stream->print(controllerName);
        stream->print("_setpoint ");
    }
        
    if (mask & PRINT_BIAS)
    {
        stream->print(controllerName);
        stream->print("_bias ");
    }
        
    if (mask & PRINT_P)
    {
        stream->print(controllerName);
        stream->print("_P ");
    }
        
    if (mask & PRINT_I)
    {
        stream->print(controllerName);
        stream->print("_I ");
    }
        
    if (mask & PRINT_D)
    {
        stream->print(controllerName);
        stream->print("_D ");
    }
    
    stream->println();
        
    if (mask & PRINT_INPUT)
    {
        stream->print(*input);
        stream->print(" ");
    }
    
    if (mask & PRINT_OUTPUT)
    {
        stream->print(*output);
        stream->print(" ");
    }
    
    if (mask & PRINT_SETPOINT)
    {
        stream->print(*setpoint);
        stream->print(" ");
    }
    
    if (mask & PRINT_BIAS)
    {
        stream->print(bias);
        stream->print(" ");
    }
    
    if (mask & PRINT_P)
    {
        stream->print(pOut);
        stream->print(" ");
    }
    
    if (mask & PRINT_I)
    {
        stream->print(iOut);
        stream->print(" ");
    }
    
    if (mask & PRINT_D)
    {
        stream->print(dOut);
        stream->print(" ");
    }
    
    stream->println();
}
