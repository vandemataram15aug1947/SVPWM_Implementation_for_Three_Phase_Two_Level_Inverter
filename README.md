# SVPWM_Implementation_for_Three_Phase_Two_Level_Inverter

# 🔲 Space Vector Pulse Width Modulation (SVPWM)

## 📘 Introduction

Space Vector Pulse Width Modulation (SVPWM) is an advanced PWM technique used to control three-phase inverters more efficiently than traditional sinusoidal PWM. By treating the inverter as a single unit, SVPWM synthesizes the desired output voltage vector through a combination of switching states. This approach improves DC bus utilization, reduces total harmonic distortion (THD), and enhances overall system efficiency.

---

## 🧠 Theory

SVPWM treats the three-phase inverter output as a single rotating vector in a two-dimensional α-β plane (also called the Clarke transformation plane). Instead of controlling three separate phase voltages directly, SVPWM computes an equivalent voltage vector `Vref` that represents the desired three-phase voltages at that moment in time.

- Use 6 active and 2 zero voltage vectors  
- Divide space vector plane into 6 sectors (each 60° wide)  
- Represent the desired voltage vector `Vref` using time-weighted combinations of adjacent active vectors and zero vectors within one PWM cycle

---

## 🧮 Step-by-Step Mathematical Explanation

### Clarke Transformation (abc → αβ)

Convert three-phase quantities into 2D orthogonal components:

### Given,

```math
V_a = V_m \cos(\omega t)
```

```math
V_b = V_m \cos(\omega t - 120^\circ)
```

```math
V_c = V_m \cos(\omega t + 120^\circ)
```

Clarke transform gives:

```math
V_\alpha = V_a
```

```math
V_\beta = \frac{1}{\sqrt{3}} (V_b - V_c)
```

Then, construct the space vector:

```math
\vec{V}_{ref} = V_\alpha + j V_\beta
```

---

### Determine the Sector

Calculate the angle \( \theta \) of the reference vector:

```math
\theta = \tan^{-1}\left(\frac{V_\beta}{V_\alpha}\right)
```

Use this angle to determine in which of the 6 sectors the vector lies.

---

# SVPWM Vector Times Calculation

This project calculates the vector times $T_1$, $T_2$, and $T_0$ for Space Vector Pulse Width Modulation (SVPWM). The SVPWM technique is used to control three-phase inverters and optimize the output voltage to drive the motor effectively. The calculation of the vector times is based on decomposing the reference vector $\vec{V}_{ref}$ into adjacent vectors $\vec{V}_1$, $\vec{V}_2$, and the zero vector $\vec{V}_0$.

## Formulae for Calculation

The vector times are computed based on the following parameters:
- $T_s$: PWM period
- $V_{dc}$: DC bus voltage
- $|\vec{V}_{ref}|$: Magnitude of the reference vector
- $\theta$: Angle of $\vec{V}_{ref}$ within the sector

## ⏱️ Time Calculations for Space Vector PWM (SVPWM)

In Space Vector Pulse Width Modulation, the time durations for active vectors **$\vec{V}_1$**, **$\vec{V}_2$**, and the zero vector are computed as follows for a given sampling period $T_s$:

### Duration of Vector $\vec{V}_1$ (Time $T_1$)

```math
T_1 = \frac{T_s \cdot |\vec{V}_{\text{ref}}|}{V_{dc}} \cdot \sin\left(\frac{\pi}{3} - \theta\right)
```

### Duration of Vector $\vec{V}_2$ (Time $T_2$)

```math
T_2 = \frac{T_s \cdot |\vec{V}_{\text{ref}}|}{V_{dc}} \cdot \sin(\theta)
```

### Duration of Vector $T_0$ (Zero vector time):

    ```math
    T_0 = T_s - T_1 - T_2
    ```

### Explanation:
- $T_1$: The time duration for the adjacent vector $\vec{V}_1$.
- $T_2$: The time duration for the adjacent vector $\vec{V}_2$.
- $T_0$: The time for the zero vector $\vec{V}_0$, which is used for symmetry.

### Sector Consideration:
- The reference vector $\vec{V}_{ref}$ lies in one of the six sectors of the hexagonal voltage space.
- The angle $\theta$ represents the position of the reference vector within a specific sector. This angle is used to calculate the respective vector times.

### How to Use:
1. **Input Parameters**:
    - Define the PWM period $T_s$.
    - Set the DC bus voltage $V_{dc}$.
    - Input the magnitude $|\vec{V}_{ref}|$ of the reference vector.
    - Input the angle $\theta$ of the reference vector in the specific sector.

2. **Calculation**:
    - Use the above formulae to compute the values of $T_1$, $T_2$, and $T_0$.

3. **Output**:
    - The calculated times $T_1$, $T_2$, and $T_0$ can be used in generating PWM signals to control the inverter or motor drive.

## Example:

For a given scenario where:
- $V_{dc} = 400 \, \text{V}$
- $|\vec{V}_{ref}| = 230 \, \text{V}$
- $\theta = 30^\circ$ (in radians: $\theta = \pi / 6$)
- $T_s = 20 \, \mu s$

You can calculate the vector times using the above formulae.


---


# Code Explanation

## Include Required Header Files

```c
#include "stdint.h"         /* Standard integer types */
#include "math.h"           /* Standard math library */
#include "IQmathLib.h"      /* TI's fixed-point math library */

#include "device.h"         /* Device-specific configuration */
#include "ADC.h"            /* ADC configuration and data handling */
#include "eQEP.h"           /* Quadrature encoder interface */

#include "CLARKE.h"         /* Clarke transformation (abc → αβ) */
#include "PARK.h"           /* Park transformation (αβ → dq) */
#include "iPARK.h"          /* Inverse Park transformation (dq → αβ) */
#include "iCLARKE.h"        /* Inverse Clarke transformation (αβ → abc) */
#include "SVPWM.h"          /* Space Vector PWM generation */
```

## Headers and Functionality

Here is a list of the headers used in this project and their specific functionality:

- **`"stdint.h"`**: Standard integer types (such as `int32_t`, `uint16_t`) for improved portability across platforms.
- **`"math.h"`**: Standard math library providing functions like `sin()`, `cos()`, and `atan2()` for performing trigonometric calculations.
- **`"IQmathLib.h"`**: TI's fixed-point math library, enabling efficient fixed-point mathematical operations.
- **`"device.h"`**: Device-specific configuration, such as microcontroller settings, peripheral initialization, and clock configuration.
- **`"ADC.h"`**: ADC driver functions for configuring and handling Analog-to-Digital Conversion (ADC) to acquire motor currents and rotor speed.
- **`"eQEP.h"`**: Quadrature Encoder Pulse (eQEP) interface, used for acquiring encoder data to monitor rotor position and speed.
- **`"CLARKE.h"`**: Clarke transformation (abc → αβ), used to convert a 3-phase system to a 2-phase system in motor control.
- **`"PARK.h"`**: Park transformation (αβ → dq), which is used to convert the αβ frame to the dq frame for motor control, aligned with the rotor.
- **`"PI.h"`**: Contains the implementation of Proportional-Integral (PI) controllers, which are used for feedback control in motor control systems. The PI controller calculates the error between the reference and measured values (e.g., motor speed or position), and adjusts the control signal to minimize this error, ensuring stable motor operation.
- **`"iPARK.h"`**: Inverse Park transformation (dq → αβ), which converts the dq frame back to the αβ frame.
- **`"iCLARKE.h"`**: Inverse Clarke transformation (αβ → abc), which converts the 2-phase system back to the 3-phase system for motor drive.
- **`"SVPWM.h"`**: Space Vector Pulse Width Modulation (SVPWM) generation, used to control motor voltage using space vector modulation techniques.

---


## 🔁 FOC Control Loop Summary

### Code Implementation

### Global Variables
```c
/* Global Variables */
float i_alpha, i_beta;      /* Alpha-beta current components */
float i_d, i_q;             /* Direct-quadrature currents */
float v_d = 0, v_q = 0;     /* Direct-quadrature voltage components */
float v_alpha, v_beta;      /* Inverse Clarke transform results */
float theta;                /* Rotor position from encoder */
float omega_actual;         /* Actual rotor speed */
float omega_ref = 1500;     /* Reference speed in RPM */
float i_d_ref = 0.0f;       /* d-axis current reference */
float i_q_ref = 2.0f;       /* q-axis current reference */
```

### Encoder Read (Position Feedback)
```c
/* Encoder Read Function */
/* Reads the current rotor position from encoder */
float encoder_read() {
    return get_encoder_position();  /* Read encoder feedback */
}
```

### Clarke Transform (ABC to αβ)
```c
/* Clarke Transform Function */
/* Converts 3-phase current (i_a, i_b, i_c) to 2-phase orthogonal (i_alpha, i_beta) */
void clarke_transform(float i_a, float i_b, float i_c) {
    i_alpha = i_a;
    i_beta = (i_a + 2.0f * i_b) / sqrtf(3.0f);  /* Assuming balanced system */
}
```

### Park Transform (αβ → dq)
```c
/* Park Transform Function */
/* Converts (i_alpha, i_beta) into rotating reference frame components (i_d, i_q) */
void park_transform(float theta) {
    float sin_theta = sinf(theta);  /* Calculate sin(theta) */
    float cos_theta = cosf(theta);  /* Calculate cos(theta) */
    
    i_d = i_alpha * cos_theta + i_beta * sin_theta;    /* d-axis current */
    i_q = -i_alpha * sin_theta + i_beta * cos_theta;   /* q-axis current */
}
```

### Speed PI Controller
```c
/* Speed PI Controller */
/* Implements a basic PI controller for speed regulation */
void speed_control(float omega_actual) {
    static float integral = 0;           /* Integral term storage */
    float error = omega_ref - omega_actual;  /* Speed error */
    float Kp = 0.01f, Ki = 0.005f;       /* PI controller gains */
    
    integral += error * Ki;              /* Integrate error */
    integral = fminf(fmaxf(integral, -1.0f), 1.0f);  /* Anti-windup: limit integral */
    
    v_q = (Kp * error) + integral;       /* Output q-axis voltage command */
}

/* Current PI Controller */
/* Controls i_d and i_q to track reference values */
void current_pi_control() {
    static float int_d = 0, int_q = 0;   /* Integral terms */
    float Kp = 0.2f, Ki = 0.01f;         /* PI gains for current loops */
    
    float error_d = i_d_ref - i_d;       /* Error in d-axis current */
    float error_q = i_q_ref - i_q;       /* Error in q-axis current */
    
    int_d += error_d * Ki;               /* Integrate d error */
    int_q += error_q * Ki;               /* Integrate q error */
    
    int_d = fminf(fmaxf(int_d, -1.0f), 1.0f);  /* Anti-windup for d */
    int_q = fminf(fmaxf(int_q, -1.0f), 1.0f);  /* Anti-windup for q */
    
    v_d = (Kp * error_d) + int_d;        /* Output d-axis voltage */
    v_q = (Kp * error_q) + int_q;        /* Output q-axis voltage */
}
```

### Inverse Park Transform (dq → αβ)
```c
/* Inverse Park Transform Function */
/* Converts (v_d, v_q) from rotating to stationary reference frame (v_alpha, v_beta) */
void inverse_park_transform(float theta) {
    float sin_theta = sinf(theta);      /* Calculate sin(theta) */
    float cos_theta = cosf(theta);      /* Calculate cos(theta) */
    
    v_alpha = v_d * cos_theta - v_q * sin_theta;  /* Alpha-axis voltage */
    v_beta = v_q * cos_theta + v_d * sin_theta;   /* Beta-axis voltage */
}
```

### SVPWM Generation
```c
#include "driverlib.h"
#include "math.h"

/* SVPWM Generation */
/* Generates PWM duty cycles based on v_alpha and v_beta */
void generate_svpwm_pulses(float v_alpha, float v_beta) {
    float v_ref = sqrtf(v_alpha * v_alpha + v_beta * v_beta);  /* Magnitude of voltage vector */
    float angle = atan2f(v_beta, v_alpha);                     /* Angle of voltage vector */
    
    if (angle < 0) angle += 2.0f * M_PI;                       /* Wrap angle into [0, 2π] */

    int sector_num = (int)(angle / (M_PI / 3.0f));             /* Determine sector (0 to 5) */
    float T = 1.0f;                                            /* Normalized PWM period */
    float t1 = 0, t2 = 0, t0 = 0;                              /* Time durations */
    float sqrt3 = sqrtf(3.0f);

    /* Calculate t1 and t2 based on sector */
    switch (sector_num) {
        case 0:
            t1 = sqrt3 * v_ref * sinf(M_PI / 3.0f - angle);
            t2 = sqrt3 * v_ref * sinf(angle);
            break;
        case 1:
            angle -= M_PI / 3.0f;
            t1 = sqrt3 * v_ref * sinf(M_PI / 3.0f - angle);
            t2 = sqrt3 * v_ref * sinf(angle);
            break;
        case 2:
            angle -= 2 * M_PI / 3.0f;
            t1 = sqrt3 * v_ref * sinf(M_PI / 3.0f - angle);
            t2 = sqrt3 * v_ref * sinf(angle);
            break;
        case 3:
            angle -= M_PI;
            t1 = sqrt3 * v_ref * sinf(M_PI / 3.0f - angle);
            t2 = sqrt3 * v_ref * sinf(angle);
            break;
        case 4:
            angle -= 4 * M_PI / 3.0f;
            t1 = sqrt3 * v_ref * sinf(M_PI / 3.0f - angle);
            t2 = sqrt3 * v_ref * sinf(angle);
            break;
        case 5:
            angle -= 5 * M_PI / 3.0f;
            t1 = sqrt3 * v_ref * sinf(M_PI / 3.0f - angle);
            t2 = sqrt3 * v_ref * sinf(angle);
            break;
    }

    t0 = T - t1 - t2;  /* Zero vector time */

    /* Calculate duty cycles for phases A, B, C */
    float Ta = (t1 + t2 + t0 / 2.0f) / T;
    float Tb = (t2 + t0 / 2.0f) / T;
    float Tc = t0 / 2.0f / T;

    /* Set PWM duty cycles (assumes API exists for this) */
    PWM_setDutyCycle(PWM1_BASE, PWM_OUT_1, Ta);  /* Phase A */
    PWM_setDutyCycle(PWM1_BASE, PWM_OUT_2, Tb);  /* Phase B */
    PWM_setDutyCycle(PWM1_BASE, PWM_OUT_3, Tc);  /* Phase C */
}
```

### Main Loop
```c
/* Main Entry Point */
int main() {
    /* Initialize encoder and peripherals */
    pwm_init();         /* PWM peripheral setup */
    adc_init();         /* ADC initialization for i_a, i_b, i_c */
    
    while (1) {
        /* Step 1: Read actual rotor position */

        /* Step 2: Read 3-phase currents */
        float i_a = read_adc_current_A();
        float i_b = read_adc_current_B();
        float i_c = read_adc_current_C();

        /* Step 3: Clarke and Park transforms */
        clarke_transform(i_a, i_b, i_c);
        park_transform(theta);

        /* Step 4: Speed PI controller (if speed control is used) */
        omega_actual = calculate_speed_from_encoder();
        speed_control(omega_actual);  /* Updates v_q */

        /* Step 5: Current PI controller */
        current_pi_control();         /* Updates v_d and v_q */

        /* Step 6: Inverse Park Transform */
        inverse_park_transform(theta);

        /* Step 7: SVPWM signal generation */
        generate_svpwm_pulses(v_alpha, v_beta);
    }

    return 0;
}

```
