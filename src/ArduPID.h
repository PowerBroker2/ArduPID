#pragma once
#include <Arduino.h>

class TeensyPID
{
public:
    enum PMode { P_ON_E, P_ON_M };
    enum DMode { D_ON_E, D_ON_M };
    enum Direction { FORWARD = 1, BACKWARD = -1 };
    enum AntiWindupMode { CLAMP_HEADROOM, CONDITIONAL_INTEGRATION, BACK_CALCULATION };

    // Debug masks
    static const uint8_t PRINT_INPUT    = 0x01;
    static const uint8_t PRINT_OUTPUT   = 0x02;
    static const uint8_t PRINT_SETPOINT = 0x04;
    static const uint8_t PRINT_BIAS     = 0x08;
    static const uint8_t PRINT_P        = 0x10;
    static const uint8_t PRINT_I        = 0x20;
    static const uint8_t PRINT_D        = 0x40;

    TeensyPID();

    void begin(float kp,
               float ki,
               float kd,
               float dtSeconds,
               float outMin,
               float outMax,
               PMode pMode = P_ON_E,
               DMode dMode = D_ON_M,
               Direction direction = FORWARD,
               float bias = 0.0f);

    void reset();

    void setTunings(float kp, float ki, float kd);
    void setModes(PMode pMode, DMode dMode);
    void setOutputLimits(float min, float max);
    void setWindupLimits(float min, float max);
    void setDeadband(float min, float max);
    void setBias(float bias);
    void setDirection(Direction dir);
    void setDt(float dtSeconds);
    void setAntiWindupMode(AntiWindupMode mode);
    void setBackCalculationGain(float kaw);

    float compute(float setpoint, float input);

    float P() const;
    float I() const;
    float D() const;
    float Bias() const;

    void debug(Stream& stream,
               const char* name,
               uint8_t mask,
               float setpoint,
               float input,
               float output);

private:
    // Mode handlers
    float pOnError(float error, float dInput);
    float pOnMeasurement(float error, float dInput);
    float dOnError(float dError, float dInput);
    float dOnMeasurement(float dError, float dInput);

    float (TeensyPID::*_computeP)(float, float);
    float (TeensyPID::*_computeD)(float, float);

    // Gains
    float _kp, _ki, _kd;
    float _dt;

    // State
    float _integral;
    float _prevError;
    float _prevInput;

    float _pOut, _iOut, _dOut;
    float _bias;

    float _outMin, _outMax;
    float _windMin, _windMax;
    float _deadMin, _deadMax;

    AntiWindupMode _antiWindupMode;
    float _kaw;

    Direction _direction;
};
