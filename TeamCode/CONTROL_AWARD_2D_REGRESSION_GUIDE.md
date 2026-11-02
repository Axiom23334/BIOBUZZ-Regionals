# FTC Control Award Submission: 2-Dimensional Aim & Shot Ballistics

**Team:** BIOBUZZ Regionals  
**System:** Autonomous & TeleOp Turret Tracking & 2D Ballistic Regression  
**Code Reference:** [`ShooterSystem.java`](src/main/java/org/firstinspires/ftc/teamcode/subsystems/ShooterSystem.java), [`TurretTracker.java`](src/main/java/org/firstinspires/ftc/teamcode/subsystems/TurretTracker.java), [`ShooterRegressionTuning.java`](src/main/java/org/firstinspires/ftc/teamcode/ShooterRegressionTuning.java)

---

## 1. Executive Summary

In competition robotics, hitting high targets consistently requires solving for trajectory curvature and air resistance across a wide range of distances and angles. Rather than relying on a static launch velocity or single lookup table, our robot implements:
1. **Geometric 1x4 AprilTag Cluster Midpoint Reconstruction**: Reconstructs the true geometric center of the goal cluster even when only 1, 2, or 3 tags are in camera view.
2. **Dual-Triangulation Optical Distance Fusion**: Blends perspective focal-length geometry with 3D pose solve data.
3. **2-Dimensional Coupled Regression**: Dynamically outputs both optimal **Flywheel RPM** ($v_0$) and **Hood Launch Angle** ($\theta$) as functions of distance, bearing, and target yaw.
4. **Sub-Pixel Turret Azimuth Tracking with IMU Stabilization**: Centers the turret on the goal with user-tunable pixel offsets and Pinpoint IMU rotation damping.

---

## 2. 1x4 Cluster Midpoint Reconstruction Math

The goal target comprises four AprilTags placed horizontally in a 1x4 row with known inter-tag center spacing ($S_{\text{physical}} = 6.0\text{ inches}$). In match play, field obstacles and camera frame limits often occlude 1 to 3 tags. 

### 2.1 Virtual Row Midpoint Formula
Each tag $i \in \{0, 1, 2, 3\}$ has a known index relative to the cluster start ID. Let $N$ be the number of currently detected tags ($1 \le N \le 4$).
The centroid of detected tags is:
$$\bar{x} = \frac{1}{N}\sum_{i=1}^N x_i, \quad \bar{\text{idx}} = \frac{1}{N}\sum_{i=1}^N \text{index}_i$$

The true geometric center of the 4-tag row is at index $1.5$. Using the measured inter-tag pixel spacing $S_{\text{px}}$, the true cluster center $X_{\text{centre}}$ is mathematically reconstructed:
$$X_{\text{centre}} = \bar{x} + (1.5 - \bar{\text{idx}}) \cdot S_{\text{px}}$$

### 2.2 Dynamic Pairwise Spacing ($S_{\text{px}}$)
When two or more tags are visible ($a, b$ with indices $i_a < i_b$):
$$S_{\text{px}} = \frac{1}{P}\sum_{a < b} \frac{x_b - x_a}{i_b - i_a}$$
If only a single tag is visible ($P = 0$), the algorithm safely falls back to the last known verified spacing $S_{\text{last}}$, maintaining accuracy through partial occlusions.

---

## 3. Optical Distance Triangulation

Using the pinhole camera projection model with ThriftyCam calibrated focal length ($f_x = 578.272\text{ px}$ on $640\times 480$ resolution):
$$d_{\text{optical}} = \frac{S_{\text{physical}} \times f_x}{S_{\text{px}}} = \frac{6.0 \times 578.272}{S_{\text{px}}}$$

To maximize robustness against lighting and tag edge degradation, we compute a weighted fusion between optical triangulation and SDK 3D PnP pose estimation ($d_{\text{pnp}} = \text{ftcPose.range}$):
$$d_{\text{fused}} = \begin{cases} 
0.6 \cdot d_{\text{pnp}} + 0.4 \cdot d_{\text{optical}} & \text{if } \text{PnP pose available} \\
d_{\text{optical}} & \text{if only 2D centers detected}
\end{cases}$$
Clamped to operational bounds: $d \in [12.0\text{ in}, 160.0\text{ in}]$.

---

## 4. 2-Dimensional Coupled Ballistic Regression

Projectiles fired from a spinning dual-wheel launcher follow non-parabolic trajectories due to drag and Magnus effect. Rather than holding flywheel RPM constant, our system simultaneously solves for launch velocity and trajectory angle:

### 4.1 Dimension 1: Flywheel Target RPM ($v_0$)
Flywheel kinetic energy requirements scale non-linearly with distance to overcome aerodynamic drag:
$$\text{RPM}_{\text{target}}(d) = R_0 + c_{R,1} \cdot d + c_{R,2} \cdot d^2$$
- $R_0 = 3200.0\text{ RPM}$ (Point-blank base velocity)
- $c_{R,1} = 5.5\text{ RPM/in}$ (Linear range expansion coefficient)
- $c_{R,2} = 0.015\text{ RPM/in}^2$ (Quadratic drag compensation)
- Clamped within mechanical safety bounds: $[2800, 4200]\text{ RPM}$.

### 4.2 Dimension 2: Hood Pitch Position ($\theta_{\text{hood}}$)
The hood servo angle controls the release angle $\theta \in [0.0, 1.0]$. The regression combines distance, perceived spacing deviation, target bearing angle $\beta$, and wall perspective yaw $\psi$:
$$\begin{aligned}
\theta_{\text{hood}}(d, \beta, \psi) &= H_0 + c_{H,1} \cdot d + c_{H,2} \cdot d^2 \\
&\quad + c_{\text{spacing}} \cdot (S_{\text{ref}} - S_{\text{px}}) \\
&\quad + c_{\text{bearing}} \cdot |\beta| + c_{\text{yaw}} \cdot |\psi|
\end{aligned}$$
- $H_0 = 0.38$ (Base hood launch angle)
- $c_{H,1} = 0.0035$ /in, $c_{H,2} = 0.00001$ /in$^2$
- $c_{\text{spacing}} = 0.0018$ /px (Optical scale correction)
- $c_{\text{bearing}} = 0.0006$ /deg (Compensates for azimuth entry angle)
- $c_{\text{yaw}} = 0.0005$ /deg (Compensates for perspective foreshortening of the goal face)

---

## 5. Azimuth Tracking & User Aim Offset

Turret horizontal tracking uses closed-loop proportional-derivative control driven by the reconstructed horizontal error:
$$e_x = (320 - X_{\text{centre}}) + \Delta_{\text{aim\_offset}}$$
$$u_{\text{turret}} = K_{p,\text{vision}} \cdot e_{x,\text{filtered}} - K_{d,\text{vision}} \cdot v_{\text{motor}} + u_{\text{gyro\_ff}}$$

- **Tunable Aim Offset ($\Delta_{\text{aim\_offset}}$)**: Directly configurable in degrees/pixels to compensate for turret camera mounting biases and mechanical wheel bias.
- **Pinpoint Target Lock**: When tags drop out of view, the system transitions to holding the last known field-centric target heading rather than sweeping blindly:
  $$\theta_{\text{turret\_target}} = \text{normalize}(\theta_{\text{target\_world}} - \theta_{\text{bot\_imu}})$$

---

## 6. Empirical Tuning Protocol (`ShooterRegressionTuning.java`)

To calibrate the 2D regression model empirically:

1. **Station the Robot at Known Distances**:
   - Place robot at $48''$, $72''$, $96''$, and $120''$ from the center of the target cluster.
2. **Execute OpMode [`ShooterRegressionTuning`](src/main/java/org/firstinspires/ftc/teamcode/ShooterRegressionTuning.java)**:
   - **Y / A**: Adjust hood pitch fine ($\pm 0.001$) or coarse ($\pm 0.01$ with bumper).
   - **D-pad UP / DOWN**: Adjust flywheel RPM fine ($\pm 25$) or coarse ($\pm 100$ with bumper).
   - **D-pad LEFT / RIGHT**: Adjust turret horizontal offset ($\pm 1.0\text{ px}$).
   - **Right Trigger**: Feed ball through intake/transfer to fire a test shot.
3. **Record Optimal Parameters**:
   - Telemetry outputs live `Distance (in)`, `Base Pitch`, `Target Flywheel RPM`, and `Turret Aim Offset (px)`.
   - Fit recorded $(d, \text{RPM})$ and $(d, \text{Pitch})$ coordinates into quadratic regression to set $R_0, c_{R,1}, c_{R,2}$ and $H_0, c_{H,1}, c_{H,2}$ live on FTC Dashboard.
4. **Button X**: Re-engages automatic 2D regression for immediate verification at any arbitrary court position.
