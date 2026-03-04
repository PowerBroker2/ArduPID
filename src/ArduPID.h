#pragma once
#include <Arduino.h>

class ArduPID
{
public:
    enum PMode        { P_ON_E,      P_ON_M };
    enum DMode        { D_ON_E,      D_ON_M };
    enum AntiWindMode { NONE,        BACK_CALC, CLEGG };
    enum Direction    { FORWARD = 1, BACKWARD = -1 };

    struct Config
    {
        char name[32] = "PID";

        PMode        pMode     = P_ON_E;
        DMode        dMode     = D_ON_M;
        AntiWindMode aWMode    = NONE;
        Direction    direction = FORWARD;

        float pModeMix = 1.0;
    
        uint32_t dt  = 1e5;
        float    dts = 0.1;

        float kp  = 0.0;
        float ki  = 0.0;
        float kd  = 0.0;
        float kaw = 0.0;

        float pMin = 0.0;
        float pMax = 0.0;
        float iMin = 0.0;
        float iMax = 0.0;
        float dMin = 0.0;
        float dMax = 0.0;

        float outMin = 0.0;
        float outMax = 0.0;
    };

    struct Signals
    {
        uint32_t prevT = 0;
        uint32_t currT = 0;

        float error     = 0;
        float prevError = 0;
        float dError    = 0;

        float input     = 0;
        float prevInput = 0;
        float dInput    = 0;

        float setpoint = 0;
        float out      = 0;

        float p = 0;
        float i = 0;
        float d = 0;
    };

    // Debug masks
    static const uint16_t PRINT_INPUT    = 0x1 << 0;
    static const uint16_t PRINT_OUTPUT   = 0x1 << 1;
    static const uint16_t PRINT_SETPOINT = 0x1 << 2;
    static const uint16_t PRINT_KP       = 0x1 << 3;
    static const uint16_t PRINT_KI       = 0x1 << 4;
    static const uint16_t PRINT_KD       = 0x1 << 5;
    static const uint16_t PRINT_P        = 0x1 << 6;
    static const uint16_t PRINT_I        = 0x1 << 7;
    static const uint16_t PRINT_D        = 0x1 << 8;

    ArduPID();

    void reset();

    void setConfig(const Config& _config);
    void setName(const char* _name);
    void setTunings(float _kp, float _ki, float _kd);
    void setAntiWindupGain(float _kaw);
    void setPMode(PMode _pMode);
    void setPercentPonE(float p);
    void setPercentPonM(float p);
    void setDMode(DMode _dMode);
    void setAntiWindMode(AntiWindMode _aWMode);
    void setOutputLimits(float min, float max);
    void setPLimits(float min, float max);
    void setILimits(float min, float max);
    void setDLimits(float min, float max);
    void setDirection(Direction dir);
    void setDtUs(uint32_t _dt);
    void setDtMs(float _dt);
    void setDtS(float _dt);
    void setSetpoint(float _setpoint);

    float compute(float _input);

    Config  getConfig()  const;
    Signals getSignals() const;

    void debug(uint16_t mask   = PRINT_SETPOINT | PRINT_OUTPUT,
               Stream&  stream = Serial);

private:
    Config  config;
    Signals signals;

    void computeP();
    void computeI();
    void computeD();
};
