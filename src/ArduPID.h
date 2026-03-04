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
        PMode        pMode;
        DMode        dMode;
        AntiWindMode aWMode;
        Direction    direction;

        float pModeMix;
    
        uint32_t dt;

        float kp;
        float ki;
        float kd;
        float kaw;

        float pMin;
        float pMax;
        float iMin;
        float iMax;
        float dMin;
        float dMax;

        float outMin;
        float outMax;
    };

    struct Signals
    {
        uint32_t prevT;
        uint32_t currT;

        float error;
        float prevError;

        float input;
        float prevInput;

        float setpoint;
        float out;
        float p;
        float i;
        float d;
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

    float getkp()       const;
    float getki()       const;
    float getkd()       const;
    float getP()        const;
    float getI()        const;
    float getD()        const;
    float getInput()    const;
    float getSetpoint() const;
    float getOutput()   const;
    float getDtUs()     const;
    float getDtMs()     const;
    float getDtS()      const;

    void debug(      Stream& stream,
               const char*   name,
                     uint16_t mask,
                     float   setpoint,
                     float   input,
                     float   output);

private:
    Config  config;
    Signals signals;
};
