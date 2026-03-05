#pragma once
#include <Arduino.h>

/**
 * @brief Configurable discrete PID controller for Arduino-based systems.
 *
 * ArduPID implements a flexible PID controller suitable for embedded control
 * applications such as motor control, robotics, environmental regulation,
 * and feedback stabilization loops.
 *
 * The controller supports multiple algorithm variants commonly used in
 * industrial and embedded control systems:
 *
 * - Proportional-on-error or proportional-on-measurement
 * - Derivative-on-error or derivative-on-measurement
 * - Multiple anti-windup strategies
 * - Configurable fixed sample time
 * - Output and term-level saturation limits
 *
 * Internally, the controller operates using a fixed update period (`dt`)
 * and maintains its own timing using `micros()`. Calls to compute() faster
 * than the configured sample period will return the previous output without
 * updating the controller state.
 *
 * The controller is designed for lightweight embedded targets and uses
 * simple floating-point arithmetic compatible with typical Arduino-class
 * microcontrollers.
 */
class ArduPID
{
public:
    /**
     * @brief Proportional control mode.
     *
     * Determines how the proportional term reacts to system signals.
     *
     * - P_ON_E: Proportional term reacts to control error (setpoint − measurement).
     * - P_ON_M: Proportional term reacts to measurement changes.
     *
     * Proportional-on-measurement can reduce overshoot and improve disturbance
     * rejection in some systems.
     */
    enum PMode        { P_ON_E,      P_ON_M };

    /**
     * @brief Derivative control mode.
     *
     * Determines how the derivative term is computed.
     *
     * - D_ON_E: derivative of the error signal.
     * - D_ON_M: derivative of the measurement signal.
     *
     * Derivative-on-measurement is typically preferred because it avoids
     * derivative kick when the setpoint changes abruptly.
     */
    enum DMode        { D_ON_E,      D_ON_M };

    /**
     * @brief Anti-windup strategy for the integrator.
     *
     * Integral windup occurs when the controller output saturates and the
     * integral term continues to accumulate error, causing slow recovery
     * once the actuator leaves saturation.
     *
     * Supported methods:
     *
     * - NONE: No anti-windup protection.
     * - BACK_CALC: Back-calculation using actuator saturation feedback.
     * - CLEGG: Clegg integrator, which resets the integral when the error
     *   crosses zero.
     */
    enum AntiWindMode { NONE,        BACK_CALC, CLEGG };

    /**
     * @brief Controller direction.
     *
     * Determines the sign relationship between the control error and the
     * controller output.
     *
     * - FORWARD: Positive error increases controller output.
     * - BACKWARD: Positive error decreases controller output.
     *
     * Reverse-acting controllers are required when the actuator causes
     * an inverse response in the plant (e.g., cooling systems).
     */
    enum Direction    { FORWARD = 1, BACKWARD = -1 };

    /**
     * @brief Configuration parameters for the PID controller.
     *
     * This structure contains all tunable parameters and configuration
     * settings that define controller behavior.
     *
     * Users may modify individual parameters using setter methods or
     * replace the entire configuration using setConfig().
     */
    struct Config
    {
        /** @brief Controller identifier used in debug output streams. */
        char name[32] = "PID";

        /** @brief Proportional mode selection (error-based or measurement-based). */
        PMode        pMode     = P_ON_E;

        /** @brief Derivative computation mode. */
        DMode        dMode     = D_ON_M;

        /** @brief Selected anti-windup strategy. */
        AntiWindMode aWMode    = NONE;

        /** @brief Direction of controller response. */
        Direction    direction = FORWARD;

        /**
         * @brief Mixing factor for proportional control.
         *
         * Allows blending between proportional-on-error and
         * proportional-on-measurement implementations.
         *
         * 1.0 → fully proportional-on-error  
         * 0.0 → fully proportional-on-measurement
         */
        float pModeMix = 1.0;
    
        /** @brief Controller update period in microseconds. */
        uint32_t dt  = 1e5;

        /** @brief Controller update period in seconds. */
        float    dts = 0.1;

        /** @brief Proportional gain coefficient. */
        float kp  = 0.0;

        /** @brief Integral gain coefficient. */
        float ki  = 0.0;

        /** @brief Derivative gain coefficient. */
        float kd  = 0.0;

        /**
         * @brief Back-calculation anti-windup gain.
         *
         * Determines how aggressively the integrator compensates
         * for actuator saturation when BACK_CALC mode is enabled.
         */
        float kaw = 0.0;

        /** @brief Minimum proportional contribution limit. */
        float pMin = 0.0;

        /** @brief Maximum proportional contribution limit. */
        float pMax = 0.0;

        /** @brief Minimum integral contribution limit. */
        float iMin = 0.0;

        /** @brief Maximum integral contribution limit. */
        float iMax = 0.0;

        /** @brief Minimum derivative contribution limit. */
        float dMin = 0.0;

        /** @brief Maximum derivative contribution limit. */
        float dMax = 0.0;

        /** @brief Minimum allowed controller output. */
        float outMin = 0.0;

        /** @brief Maximum allowed controller output. */
        float outMax = 0.0;
    };

    /**
     * @brief Runtime signals and internal state of the PID controller.
     *
     * This structure stores the dynamic variables used during control
     * computation, including previous measurements, derivatives, and
     * individual PID contributions.
     *
     * These values can be inspected using getSignals() for debugging,
     * tuning, or telemetry purposes.
     */
    struct Signals
    {
        /** @brief Timestamp of the previous controller update (microseconds). */
        uint32_t prevT = 0;

        /** @brief Timestamp of the current controller update (microseconds). */
        uint32_t currT = 0;

        /** @brief Current control error (setpoint − measurement). */
        float error     = 0;

        /** @brief Previous control error. */
        float prevError = 0;

        /** @brief Error derivative used when derivative-on-error is enabled. */
        float dError    = 0;

        /** @brief Current measured process variable. */
        float input     = 0;

        /** @brief Previous measurement. */
        float prevInput = 0;

        /** @brief Measurement derivative used for derivative-on-measurement mode. */
        float dInput    = 0;

        /** @brief Desired target value for the controlled variable. */
        float setpoint = 0;

        /** @brief Current controller output. */
        float out      = 0;

        /** @brief Proportional contribution to the controller output. */
        float p = 0;

        /** @brief Integral contribution to the controller output. */
        float i = 0;

        /** @brief Derivative contribution to the controller output. */
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

    /**
     * @brief Construct a new PID controller instance.
     *
     * Initializes the controller with default configuration values.
     */
    ArduPID();

    /**
     * @brief Reset the controller internal state.
     *
     * Clears accumulated integrator state and historical signals.
     * This is useful when restarting control loops or when system
     * conditions change significantly.
     */
    void reset();

    /**
     * @brief Replace the entire controller configuration.
     *
     * @param _config New configuration structure.
     */
    void setConfig(const Config& _config);

    /**
     * @brief Assign a human-readable name to the controller.
     *
     * This name is used when printing debug telemetry.
     */
    void setName(const char* _name);

    /**
     * @brief Set the PID gain coefficients.
     *
     * @param _kp Proportional gain
     * @param _ki Integral gain
     * @param _kd Derivative gain
     */
    void setTunings(float _kp, float _ki, float _kd);

    /**
     * @brief Set anti-windup back-calculation gain.
     *
     * Used when BACK_CALC anti-windup mode is enabled.
     */
    void setAntiWindupGain(float _kaw);

    /**
     * @brief Set proportional control mode.
     */
    void setPMode(PMode _pMode);

    /**
     * @brief Set percentage of proportional control acting on error.
     */
    void setPercentPonE(float p);

    /**
     * @brief Set percentage of proportional control acting on measurement.
     */
    void setPercentPonM(float p);

    /**
     * @brief Set derivative calculation mode.
     */
    void setDMode(DMode _dMode);

    /**
     * @brief Select anti-windup strategy.
     */
    void setAntiWindMode(AntiWindMode _aWMode);

    /**
     * @brief Set limits for the final controller output.
     */
    void setOutputLimits(float min, float max);

    /**
     * @brief Set limits for the proportional term.
     */
    void setPLimits(float min, float max);

    /**
     * @brief Set limits for the integral term.
     */
    void setILimits(float min, float max);

    /**
     * @brief Set limits for the derivative term.
     */
    void setDLimits(float min, float max);

    /**
     * @brief Set controller direction.
     *
     * Changing direction resets internal PID contributions to prevent
     * large transient outputs caused by sign inversion.
     */
    void setDirection(Direction dir);

    /**
     * @brief Set controller timestep in microseconds.
     */
    void setDtUs(uint32_t _dt);

    /**
     * @brief Set controller timestep in milliseconds.
     */
    void setDtMs(float _dt);

    /**
     * @brief Set controller timestep in seconds.
     */
    void setDtS(float _dt);

    /**
     * @brief Set desired control setpoint.
     */
    void setSetpoint(float _setpoint);

    /**
     * @brief Execute one PID update.
     *
     * Computes a new controller output using the current input measurement.
     * If called faster than the configured sampling period, the previous
     * output is returned and no update occurs.
     *
     * @param _input Current process variable measurement.
     * @return Controller output value.
     */
    float compute(float _input);

    /**
     * @brief Retrieve the current controller configuration.
     */
    Config  getConfig()  const;

    /**
     * @brief Retrieve current controller signals and internal state.
     */
    Signals getSignals() const;

    /**
     * @brief Print selected debug telemetry.
     *
     * Debug output can include inputs, outputs, gains, and individual
     * PID components depending on the mask flags provided.
     */
    void debug(uint16_t mask   = PRINT_SETPOINT | PRINT_OUTPUT,
               Stream&  stream = Serial);

private:
    Config  config;
    Signals signals;

    /** @brief Compute proportional contribution. */
    void computeP();

    /** @brief Compute integral contribution. */
    void computeI();

    /** @brief Compute derivative contribution. */
    void computeD();
};