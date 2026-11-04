4 st# Turret Tracker: Math, Smart Filtering & Tuning Guide

This document outlines the control architecture, mathematical models, smart filters, and tuning procedures for the **Turret Tracker** in [`TurretTracker.java`](src/main/java/org/firstinspires/ftc/teamcode/subsystems/TurretTracker.java).

---

## 1. Overview & Control Philosophy

1. **Vision as the Primary Control Driver**: Whenever an AprilTag cluster is in view, the camera pixel error directly drives the turret with fast, responsive PD control.
2. **Elimination of Wild Oscillations (Velocity Damping)**:
   - Damping is applied against the turret motor's physical velocity (`turretMotor.getVelocity()`) rather than a noisy, lag-prone pixel derivative.
   - Filter lag is reduced (`FILTER_ALPHA = 0.25`), preventing the phase delay that previously caused wild left/right oscillations.
3. **Pinpoint Target Lock on Loss (No Blind Sweeping)**:
   - When AprilTags are visible, the system continuously updates and memorizes the target's world/field heading:
     $$\theta_{\text{target\_world}} = \text{normalize}\left(\theta_{\text{bot}} + \theta_{\text{turret\_rel}} + \Delta\theta_{\text{vision}}\right)$$
   - When vision drops, **blind sweeping is completely disabled**. Instead, the turret uses the GoBilda Pinpoint IMU to calculate the exact relative angle needed to remain locked onto the last known target heading as the robot moves and turns.
4. **Dynamic Chassis Rotation Feedforward**:
   - As the robot rotates, a real-time gyro feedforward counteracts the chassis yaw rate ($\text{deg/s}$), keeping the turret steadily aimed at the target without waiting for error build-up.

---

## 2. Mathematical Control Models

### 2.1 Fast Vision PD Control (with Velocity Damping)
When tags are in view (`pixelErrorX != -99999`):
$$e_{\text{filtered}}(t) = \alpha \cdot e_{\text{filtered}}(t - \Delta t) + (1 - \alpha) \cdot e_{\text{raw}}(t)$$
$$u_{\text{vision}} = \left(K_{p,\text{vision}} \cdot e_{\text{filtered}}\right) - \left(K_{d,\text{vision}} \cdot v_{\text{turret}}\right)$$
- If $|u| > 0.01$ but below static friction threshold $\text{MIN\_MOVE\_POWER} = 0.06$, power is boosted to $\text{MIN\_MOVE\_POWER}$ to ensure immediate, snappy motion.
- $K_{p,\text{vision}} = 0.0038$: Fast response mapping pixel error to motor power.
- $K_{d,\text{vision}} = 0.00015$: Dampens motor kinetic energy directly to stop overshoot and oscillation.

### 2.2 Pinpoint Target Lock (When Vision Drops)
When tags drop out of view (`pixelErrorX == -99999`):
$$\theta_{\text{desired\_turret\_deg}} = \text{normalize}\left(\theta_{\text{target\_world}} - \theta_{\text{bot}}\right)$$
$$\text{desiredTicks} = \text{clamp}\left(\theta_{\text{desired\_turret\_deg}} \times \text{TICKS\_PER\_DEGREE}, \; -450, \; 450\right)$$
$$e_{\text{ticks}} = \text{desiredTicks} - \text{currentTicks}$$
$$u_{\text{hold}} = \left(K_{p,\text{hold}} \cdot e_{\text{ticks}}\right) + \left(K_{d,\text{hold}} \cdot \frac{de_{\text{ticks}}}{dt}\right) + u_{\text{gyro\_ff}}$$
- Holds target alignment for up to `VISION_TIMEOUT_SECONDS` (5.0s) after loss before returning to idle.

### 2.3 Real-Time Gyro Feedforward
Calculated from the Pinpoint heading:
$$\omega_{\text{bot}} = \frac{\text{normalize}(\theta_{\text{bot}}(t) - \theta_{\text{bot}}(t - \Delta t))}{\Delta t}$$
$$u_{\text{gyro\_ff}} = -\omega_{\text{bot}} \times \left(\frac{\text{TICKS\_PER\_DEGREE}}{1000}\right) \times K_{p,\text{gyro\_ff}}$$
- Counteracts chassis yaw instantaneously as the driver steers the robot.

---

## 3. Dashboard Tuning Guide (`@Config`)

Edit live in **FTC Dashboard** (`http://192.168.43.1:8080/dash`):

| Parameter | Default | Effect | How to Tune |
|:---|:---|:---|:---|
| `KP_VISION` | `0.0038` | Speed of vision tracking | If turret doesn't track fast enough, increase toward `0.0045`. If it overshoots, reduce toward `0.0030`. |
| `KD_VISION` | `0.00015` | Oscillation damping | Increase toward `0.00025` if the turret twitches or oscillates left/right around the center. |
| `MIN_MOVE_POWER` | `0.06` | Static friction boost | Increase if the motor buzzes or stalls when making small fine adjustments. |
| `KP_HOLD` | `0.0065` | Pinpoint position hold | Increase if the turret lags behind while holding target after vision drops. |
| `KP_FEEDFORWARD_GYRO` | `1.0` | Gyro counter-rotation | Increase if the turret drifts when the robot spins in place; decrease if it over-corrects. |
| `FILTER_ALPHA` | `0.25` | Smoothing weight | Keep low ($0.15 - 0.35$) for snappy tracking. Higher values cause lag and induce hunting. |

---

## 4. Telemetry States

- `VISION`: AprilTags in camera view; active vision PD control + live Pinpoint target world heading updating.
- `PINPOINT_HOLD`: Vision lost; actively holding orientation onto the last known target coordinates using Pinpoint IMU.
- `IDLE`: Target timed out (> 5s); turret stopped.
