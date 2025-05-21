# SVPWM Implementation for Three-Phase Two-Level Inverter

## Introduction

Space Vector Pulse Width Modulation (SVPWM) is an advanced PWM technique widely used to control three-phase voltage source inverters. Unlike traditional sinusoidal PWM, SVPWM treats the inverter as a single unit and manipulates the output voltage vector directly in the two-dimensional α-β stationary reference frame.

**Key benefits of SVPWM:**

- Improved DC bus voltage utilization  
- Reduced total harmonic distortion (THD)  
- Enhanced inverter efficiency and output waveform quality  

This technique is commonly employed in motor drives, renewable energy systems, and various power electronics applications.

---

## SVPWM Principle

SVPWM represents the inverter output voltage as a space vector in the α-β plane, synthesized by switching between the nearest active voltage vectors and zero vectors within each PWM cycle.

- The α-β plane is divided into six 60° sectors.  
- The reference voltage vector is synthesized by time-weighted switching of two adjacent active vectors and one or two zero vectors.  
- PWM duty cycles for phases A, B, and C are calculated based on these dwell times.

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

```

## SVPWM Generation Code

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

## Explanation

### Voltage Vector Representation

* The inputs `v_alpha` and `v_beta` represent the voltage vector in stationary reference frame.
* `v_ref` is the magnitude and `angle` is the orientation of this vector.

### Sector Identification

* The vector space is divided into 6 sectors (each 60°).
* Based on `angle`, the corresponding sector is identified using:

  ```c
  int sector_num = (int)(angle / (M_PI / 3.0f));
  ```

### Timing Calculations

* In each sector, the duration for the two adjacent vectors (`t1` and `t2`) and zero vector (`t0`) are calculated.
* These times determine the voltage vector placement in one switching cycle.

### Duty Cycle Computation

* Ta, Tb, and Tc represent the on-time fraction for each phase.
* The sum of `t1`, `t2`, and `t0` equals the switching period `T` (normalized to 1).

### PWM Output

* The duty cycles are applied to the three inverter legs (A, B, C).
* API `PWM_setDutyCycle()` is assumed to map these duty cycles into actual PWM registers.

## Integration

Use this function in your motor control or inverter firmware loop after computing the `v_alpha` and `v_beta` outputs from the inverse Park transform.

## Dependencies

* `math.h` for trigonometric functions.
* `driverlib.h` for PWM hardware interface (specific to your MCU platform).

## License

MIT or any appropriate open-source license as per your project.
