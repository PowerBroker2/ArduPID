# ArduPID

[![GitHub version](https://badge.fury.io/gh/PowerBroker2%2FArduPID.svg)](https://badge.fury.io/gh/PowerBroker2%2FArduPID) [![arduino-library-badge](https://www.ardu-badge.com/badge/ArduPID.svg?)](https://www.ardu-badge.com/ArduPID)  

PID library for Arduino with improved accuracy and configurability compared to the legacy Arduino PID library.

---

## What is PID?

PID stands for **Proportional, Integral, Derivative**, and is a common feedback control strategy. A PID controller continuously calculates an **error** value as the difference between a desired setpoint and a measured process variable, and applies corrections via three terms:

- **P (Proportional):** Reacts proportionally to the error.  
- **I (Integral):** Accumulates past errors to eliminate steady-state offset.  
- **D (Derivative):** Predicts future error based on its rate of change to dampen oscillations.  

PID controllers can stabilize control for systems such as motors, servos, temperature, or even cruise control in vehicles.

---

## How Does it Work?

See this explanation video: [How PID Controllers Work](https://www.youtube.com/watch?v=OqvrYNJvtaU)

---

## How to Tune a PID

A detailed guide can be found here: [PID Tuning Guide](https://pidexplained.com/how-to-tune-a-pid-controller/)

---

## Getting Started with ArduPID

### 1. Include the Library

```cpp
#include "ArduPID.h"
```

### 2. Instantiate a Controller and Create Variables

```cpp
ArduPID myController;

float setpoint = 512;
float input;
float output;

float kp = 1.0;
float ki = 0.0;
float kd = 0.0;
```

### 3. Configure the Controller in `setup()`

```cpp
void setup()
{
    Serial.begin(115200);

    // Set PID gains
    myController.setTunings(kp, ki, kd);

    // Optional configuration
    myController.setOutputLimits(0, 255);   // Clip output to actuator range
    myController.setILimits(-100, 100);     // Prevent integral wind-up
    myController.setPLimits(-255, 255);     // Optional P term limits
    myController.setDLimits(-50, 50);       // Optional D term limits
    myController.setDirection(ArduPID::FORWARD); // Set output direction
}
```

### 4. Update the Controller in `loop()`

```cpp
void loop()
{
    // Read sensor input
    input = analogRead(A0); // Replace with your sensor reading

    // Compute PID output
    output = myController.compute(input);

    // Debug information (prints selected signals to Serial)
    myController.debug(ArduPID::PRINT_OUTPUT   |
                       ArduPID::PRINT_SETPOINT);

    // Send output to actuator
    analogWrite(3, output); // Replace with your actuator control
}
```

## Configuration Commands

- `setDirection(Direction dir)` – Set output direction. Resets PID state if changed.
- `setTunings(float kp, float ki, float kd)` – Set PID gains (all positive values).
- `setOutputLimits(float min, float max)` – Clip the PID output to specified range.
- `setPLimits(float min, float max)` – Limit the proportional term.
- `setILimits(float min, float max)` – Limit the integral term to prevent wind-up.
- `setDLimits(float min, float max)` – Limit the derivative term.
- `setPMode(PMode mode)` – Choose P_ON_E (proportional on error) or P_ON_M (proportional on measurement).
- `setDMode(DMode mode)` – Choose derivative on error or measurement.
- `setAntiWindMode(AntiWindMode mode)` – Choose anti-windup strategy: NONE, BACK_CALC, or CLEGG.
- `setDtMs(float dt)` / `setDtUs(uint32_t dt)` / `setDtS(float dt)` – Set sample time. Resets PID state if changed.
- `setSetpoint(float sp)` – Update target value for PID.
- `reset()` – Resets internal PID state (P, I, D, errors, timestamps). Useful when changing direction, sample time, or after major system changes.
- `debug(uint16_t mask, Stream &stream)` – Print selected PID signals to serial for plotting or monitoring.
