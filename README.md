# LAB1: Kalman Filter / SLAM

## Table of Contents
- [LAB1: Kalman Filter / SLAM](#lab1-kalman-filter--slam)
  - [Table of Contents](#table-of-contents)
  - [Project Overview](#project-overview)
  - [System Architecture](#system-architecture)
  - [Setup](#setup)
    - [Running](#running)
  - [Part 1: EKF Odometry Fusion](#part-1-ekf-odometry-fusion)
    - [1.1 Wheel Odometry](#11-wheel-odometry)
      - [Robot Parameters](#robot-parameters)
      - [Wheel Displacement](#wheel-displacement)
      - [ICC (Instantaneous Center of Curvature) Kinematics](#icc-instantaneous-center-of-curvature-kinematics)
      - [Robot Velocity](#robot-velocity)
    - [1.2 Extended Kalman Filter](#12-extended-kalman-filter)
      - [1.2.1 State Vector Design](#121-state-vector-design)
      - [1.2.2 Motion Model (Prediction)](#122-motion-model-prediction)
      - [1.2.3 Measurement Model (Correction)](#123-measurement-model-correction)
      - [1.2.4 EKF Algorithm](#124-ekf-algorithm)
      - [1.2.5 Coordinate Frames](#125-coordinate-frames)
      - [1.2.6 Noise Covariance](#126-noise-covariance)
    - [1.3 Experimental Validation](#13-experimental-validation)
      - [Experimental Setup](#experimental-setup)
      - [Filter Validation](#filter-validation)
      - [Trajectory Comparison](#trajectory-comparison)
      - [Time Series Analysis](#time-series-analysis)
  - [Part 2: ICP Odometry Refinement](#part-2-icp-odometry-refinement)
  - [Part 3: Full SLAM with slam\_toolbox](#part-3-full-slam-with-slam_toolbox)
  - [Part 4: Results](#part-4-results)
  - [Part 5: Discussion](#part-5-discussion)
  - [Conclusion](#conclusion)

---

## Project Overview

<!-- TODO: Add project overview -->

---

## System Architecture

<!-- TODO: Add system architecture diagram -->

---

## Setup

```bash
# Clone the repository
git clone https://github.com/pboon09/FRA532-Autonomous-Mobile-Robot.git -b lab1

cd FRA532-Autonomous-Mobile-Robot

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build

# Source workspace
source install/setup.bash

# Add to bashrc (optional)
echo "source ~/FRA532-Autonomous-Mobile-Robot/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Running

**Terminal 1: Play ROSBag**
```bash
cd FRA532-Autonomous-Mobile-Robot
 
# Sequence 00 - Empty Hallway
ros2 bag play FRA532_LAB1_DATASET/fibo_floor3_seq00 --clock

# Sequence 01 - Non-Empty Hallway with Sharp Turns
ros2 bag play FRA532_LAB1_DATASET/fibo_floor3_seq01 --clock

# Sequence 02 - Non-Empty Hallway with Non-Aggressive Motion
ros2 bag play FRA532_LAB1_DATASET/fibo_floor3_seq02 --clock
```

**Terminal 2: Choose which part you want to see the result**
```bash
ros2 launch turtle_bringup part1.launch.py
```

---

## Part 1: EKF Odometry Fusion

This section presents a sensor fusion approach combining wheel odometry and IMU orientation using an Extended Kalman Filter (EKF). The wheel odometry provides position and velocity estimates through encoder measurements, while the IMU supplies heading corrections to compensate for accumulated drift.

**References:**
- Columbia University CS4733 - [ICC Kinematics](https://www.cs.columbia.edu/~allen/F17/NOTES/icckinematics.pdf)

### 1.1 Wheel Odometry

#### Robot Parameters

| Parameter | Symbol | Value | Description |
|-----------|--------|-------|-------------|
| Wheel radius | r | 0.033 m | TurtleBot3 Burger wheel radius |
| Track width | b | 0.160 m | Distance between wheel centers |

#### Wheel Displacement

The wheel displacements are computed from encoder position changes:

```math
\Delta s_r = \Delta \theta_r \cdot r, \quad \Delta s_l = \Delta \theta_l \cdot r
```

where $\Delta \theta_r$ and $\Delta \theta_l$ represent the angular displacement of the right and left wheels in radians.

#### ICC (Instantaneous Center of Curvature) Kinematics

For differential drive robots, the ICC method provides accurate pose integration by computing the instantaneous turning center.

**Heading change:**

```math
\Delta \theta = \frac{\Delta s_r - \Delta s_l}{b}
```

**Turning radius:**

```math
R = \frac{b}{2} \cdot \frac{\Delta s_l + \Delta s_r}{\Delta s_r - \Delta s_l}
```

**ICC coordinates:**

```math
ICC_x = x - R \sin(\theta), \quad ICC_y = y + R \cos(\theta)
```

**Pose update (general case):**

```math
\begin{bmatrix} x' \\ y' \end{bmatrix} = \begin{bmatrix} \cos\Delta\theta & -\sin\Delta\theta \\ \sin\Delta\theta & \cos\Delta\theta \end{bmatrix} \begin{bmatrix} x - ICC_x \\ y - ICC_y \end{bmatrix} + \begin{bmatrix} ICC_x \\ ICC_y \end{bmatrix}
```

```math
\theta' = \theta + \Delta\theta
```

**Pose update (straight line, $|\Delta s_r - \Delta s_l| < \epsilon$):**

When the robot moves straight, $R \to \infty$. The pose update simplifies to:

```math
x' = x + \frac{\Delta s_l + \Delta s_r}{2} \cos\theta, \quad y' = y + \frac{\Delta s_l + \Delta s_r}{2} \sin\theta
```

#### Robot Velocity

The linear and angular velocities are derived from wheel displacements:

```math
v = \frac{\Delta s_r + \Delta s_l}{2 \Delta t}, \quad \omega = \frac{\Delta s_r - \Delta s_l}{b \cdot \Delta t}
```

**Position Delta vs Direct Velocity Measurement:**

The robot velocity is computed from **encoder position deltas** rather than the velocity field in JointState messages. This approach is preferred because encoder position is the **primary data source** from the hardware. The JointState velocity field is not used for two reasons: first, it **may be empty or unavailable** depending on the robot configuration; second, when present, the velocity is **already derived from position deltas**, so using it would add an unnecessary layer of indirection.

### 1.2 Extended Kalman Filter

#### 1.2.1 State Vector Design

The EKF estimates a 3-dimensional state vector:

```math
\mu = \begin{bmatrix} x \\ y \\ \theta \end{bmatrix}
```

| State | Description |
|-------|-------------|
| x | Position in x-axis (m) |
| y | Position in y-axis (m) |
| θ | Heading angle (rad) |

**Why these 3 states?**
- A ground robot is constrained to planar motion, so only 2D position and heading are needed
- Roll, pitch, and z-height are assumed constant (flat floor assumption)
- These states fully describe the robot's configuration in the odom frame

**Why not include velocities $(v, \omega)$?**

In this EKF formulation, velocities are treated as **control inputs**, not states:
- Wheel encoders directly measure $(v, \omega)$
- The EKF uses these as inputs to the motion model (prediction step)
- To estimate velocities as states, we would need **independent velocity measurements** for the correction step
- Since the IMU only provides heading $\theta$, there is no velocity measurement to fuse
- Including unmeasured states would only add uncertainty without improving the estimate

#### 1.2.2 Motion Model (Prediction)

**Control Input:**

```math
u_t = \begin{bmatrix} v \\ \omega \end{bmatrix}
```

**Why use $(v, \omega)$ instead of wheel odometry pose $(x, y, \theta)$?**

Using velocities allows the EKF to **perform its own integration**, maintaining a separate state that can be **corrected by IMU**. Using the integrated pose directly would **replace** the EKF state with wheel odometry, **bypassing fusion** entirely.

**State Transition (Unicycle Model):**

```math
\bar{\mu}_t = f(\mu_{t-1}, u_t) = \begin{bmatrix} x + v \cos\theta \cdot \Delta t \\ y + v \sin\theta \cdot \Delta t \\ \theta + \omega \cdot \Delta t \end{bmatrix}
```

**State Jacobian:**

```math
F_t = \frac{\partial f}{\partial \mu} = \begin{bmatrix} 1 & 0 & -v \sin\theta \cdot \Delta t \\ 0 & 1 & v \cos\theta \cdot \Delta t \\ 0 & 0 & 1 \end{bmatrix}
```

#### 1.2.3 Measurement Model (Correction)

**Measurement Vector:**

The IMU provides orientation as a quaternion. Only the yaw component is extracted:

```math
z_t = \begin{bmatrix} \theta_{IMU} \end{bmatrix}
```

**Why not use wheel odometry $(x, y, \theta)$ as measurements?**

Wheel odometry pose is derived from the **same encoder data** used in prediction. Using it as a measurement would **double-count** the same information. Measurements must come from **independent sensors**. The IMU provides heading independently of wheel encoders.

**IMU Data Selection:**

| IMU Data | Used | Rationale |
|----------|------|-----------|
| Orientation (yaw) | Yes | Heading reference independent of wheel slip |
| Orientation (roll, pitch) | No | Ground robot assumes planar motion |
| Angular velocity | No | Wheel encoders provide more accurate ω |
| Linear acceleration | No | Double integration causes drift; wheel odometry is superior |

**IMU Offset Handling:**

The IMU yaw is zeroed at startup by storing the initial reading as an offset. All subsequent measurements are relative to this initial heading, aligning the IMU frame with the odometry frame.

**Measurement Function:**

```math
h(\bar{\mu}_t) = \theta
```

**Measurement Jacobian:**

```math
H_t = \frac{\partial h}{\partial \mu} = \begin{bmatrix} 0 & 0 & 1 \end{bmatrix}
```

#### 1.2.4 EKF Algorithm

**Prediction Step:**

1. State prediction:
```math
\bar{\mu}_t = f(\mu_{t-1}, u_t)
```

2. Covariance prediction:
```math
\bar{\Sigma}_t = F_t \Sigma_{t-1} F_t^T + Q_t
```

**Correction Step:**

1. Innovation (Measurement Residual):
```math
y_t = z_t - h(\bar{\mu}_t)
```

2. Innovation covariance:
```math
S_t = H_t \bar{\Sigma}_t H_t^T + R_t
```

3. Kalman gain:
```math
K_t = \bar{\Sigma}_t H_t^T S_t^{-1}
```

4. State update:
```math
\mu_t = \bar{\mu}_t + K_t y_t
```

5. Covariance update:
```math
\Sigma_t = (I - K_t H_t) \bar{\Sigma}_t
```

**Angle Normalization:**

The heading angle $\theta$ is normalized to $[-\pi, \pi]$ after each update using:

```math
\theta = \text{atan2}(\sin\theta, \cos\theta)
```

#### 1.2.5 Coordinate Frames

**Frame Tree:**

```
odom (world-fixed, drifts over time)
  │
  └──► base_footprint (robot body frame)
          │
          └──► base_link, sensors, wheels...
```

The EKF publishes the `odom → base_footprint` transform because the estimate is based solely on wheel odometry and IMU, which accumulate drift over time. The `map` frame would imply global accuracy, which requires external localization (SLAM, GPS).

#### 1.2.6 Noise Covariance

**What is Process and Measurement Noise?**

The Kalman filter models two sources of uncertainty:

| Noise Type | Symbol | Source | Purpose |
|------------|--------|--------|---------|
| **Process Noise** | Q | Motion model imperfection | Accounts for unmodeled dynamics (wheel slip, terrain variation) |
| **Measurement Noise** | R | Sensor imperfection | Accounts for sensor noise and bias |

Without noise covariances, the filter cannot balance prediction vs. measurement. Q represents how much we **distrust** the motion model per timestep; R represents how much we **distrust** the sensor measurement.

**Covariance Matrices in This Work:**

```math
Q_t = \begin{bmatrix} Q_{xx} & 0 & 0 \\ 0 & Q_{yy} & 0 \\ 0 & 0 & Q_{\theta\theta} \end{bmatrix}, \quad R_t = \begin{bmatrix} R_{\theta\theta} \end{bmatrix}
```

**Trust Interpretation:**

| Value | Meaning |
|-------|---------|
| Lower Q | More trust in motion model (prediction) |
| Higher Q | Less trust in motion model, faster response to measurement |
| Lower R | More trust in measurement |
| Higher R | Less trust in measurement, smoother estimate, relies more on prediction |

**What Really Affects the Estimate in This 3-State Model?**

The IMU measures **only heading $\theta$**, which determines which states can be corrected.

**Why Only One Correction Source?**

- **Wheel odometry** $(v, \omega)$ drives the **prediction step** as control input
- **IMU** $\theta$ drives the **correction step** as measurement

Wheel odometry cannot be a measurement because it already defines the motion model. Using it for both would double-count information.

**What happens to $x$, $y$, $\theta$ during correction?**

Starting from the Kalman gain (Section 1.2.4):

```math
K_t = \bar{\Sigma}_t H_t^T (H_t \bar{\Sigma}_t H_t^T + R_t)^{-1}
```

With $H = [0, 0, 1]$:

```math
H_t \bar{\Sigma}_t H_t^T = \Sigma_{\theta\theta}, \quad \bar{\Sigma}_t H_t^T = \begin{bmatrix} \Sigma_{x\theta} \\ \Sigma_{y\theta} \\ \Sigma_{\theta\theta} \end{bmatrix}
```

```math
\therefore K_t = \begin{bmatrix} K_x \\ K_y \\ K_\theta \end{bmatrix} = \begin{bmatrix} \frac{\Sigma_{x\theta}}{\Sigma_{\theta\theta} + R_t} \\ \frac{\Sigma_{y\theta}}{\Sigma_{\theta\theta} + R_t} \\ \frac{\Sigma_{\theta\theta}}{\Sigma_{\theta\theta} + R_t} \end{bmatrix}
```

Since cross-covariances $\Sigma_{x\theta}, \Sigma_{y\theta} \approx 0$, we have $K_x \approx 0$ and $K_y \approx 0$.

**Key insight:** No matter what $Q_{xx}$, $Q_{yy}$ values we choose, x and y receive almost no correction because their Kalman gains are approximately zero.

**How do x and y change if no one corrects them?**

From the state update:

```math
\mu_t = \bar{\mu}_t + K_t y_t
```

With $K_x \approx 0$, $K_y \approx 0$:

```math
x_t \approx \bar{x}_t, \quad y_t \approx \bar{y}_t
```

Position states follow the **prediction exactly** (wheel odometry integration). The EKF does not correct position directly.

However, correcting $\theta$ **indirectly improves** position because future predictions use the corrected heading:

```math
\bar{x}_t = x_{t-1} + v \cos\theta_{t-1} \cdot \Delta t, \quad \bar{y}_t = y_{t-1} + v \sin\theta_{t-1} \cdot \Delta t
```

**What happens to $\theta$ when we change Q and R?**

From $K_\theta = \Sigma_{\theta\theta} / (\Sigma_{\theta\theta} + R)$ and the covariance prediction $\bar{\Sigma}_{\theta\theta} \approx \Sigma_{\theta\theta} + Q_{\theta\theta}$:

Increasing $Q_{\theta\theta}$ causes $\Sigma_{\theta\theta}$ to grow faster during prediction. A larger $\Sigma_{\theta\theta}$ in the numerator produces a larger $K_\theta$, which applies a stronger correction toward the IMU measurement.

Increasing $R$ adds more to the denominator $(\Sigma_{\theta\theta} + R)$, which produces a smaller $K_\theta$. A smaller gain means weaker correction and smoother estimates that rely more on prediction.

The ratio $Q_{\theta\theta}/R$ determines filter behavior; doubling both produces identical response.

**Conclusion**

With only IMU heading as correction source, only $Q_{\theta\theta}$ and $R$ affect the state estimate.

For $Q_{xx}$ and $Q_{yy}$, changing these values affects the covariance prediction:

```math
\bar{\Sigma}_{xx} = \Sigma_{xx,t-1} + Q_{xx}, \quad \bar{\Sigma}_{yy} = \Sigma_{yy,t-1} + Q_{yy}
```

However, these covariances do not appear in the Kalman gain $K$ because $H = [0, 0, 1]$ selects only $\Sigma_{\theta\theta}$. The state update $\mu = \bar{\mu} + K \cdot y$ remains unchanged regardless of $\Sigma_{xx}$ or $\Sigma_{yy}$. Therefore, tuning $Q_{xx}$ or $Q_{yy}$ only inflates the covariance matrix without changing the actual position estimates.

**Future Extension:** If position measurements were added (e.g., GPS), then $H$ would observe $x$ and $y$, making $K_x$ and $K_y$ non-zero. In that case, $Q_{xx}$ and $Q_{yy}$ would become meaningful tuning parameters.

### 1.3 Experimental Validation

<p align="center">
  <img src="media/part1_demo.gif" width="100%">
</p>


This section validates the EKF implementation using recorded bag files from a TurtleBot3 Burger navigating FIBO Floor 3 corridors.

#### Experimental Setup

**Dataset:**
| Sequence | Description | Duration | Samples |
|----------|-------------|----------|---------|
| seq00 | Empty hallway | 525s | 10,504 |
| seq01 | Non-empty hallway with sharp turns | 393s | 7,854 |
| seq02 | Non-empty hallway with non-aggressive motion | 599s | 11,975 |

**EKF Parameters:**
| Parameter | Value |
|-----------|-------|
| Q | diag(0.0, 0.0, 0.01) |
| R | 0.1 |
| Σ₀ | diag(0.0, 0.0, 0.1) |
| Wheel radius | 0.033 m |
| Track width | 0.160 m |

#### Filter Validation

Without external ground truth, we validate filter correctness using **innovation statistics**.  
- **Zero-mean**: Prediction matches measurement on average (filter is unbiased)
- **Small standard deviation**: Consistent with expected sensor noise (filter is consistent)

Validation Reference:
> "The KF design is considered consistent if the estimation error is unbiased (zero-mean)" - [Normalized Innovation Squared (NIS)](https://kalman-filter.com/normalized-innovation-squared/)

>"The basic idea of covariance-matching techniques is that the sample covariance of innovations should be consistent with its theoretical value" - [Adaptive Kalman Filtering, PMC](https://pmc.ncbi.nlm.nih.gov/articles/PMC8638515/)

**Innovation Statistics:**

| Sequence | Mean (°) | Std (°) | Max (°) |
|----------|----------|---------|---------|
| seq00 | -0.001 | 0.113 | 0.55 |
| seq01 | -0.018 | 0.196 | 1.70 |
| seq02 | -0.003 | 0.119 | 2.36 |

The near-zero mean confirms the filter is unbiased. The small standard deviation (~0.1-0.2°) indicates the prediction closely matches IMU measurements. This validates proper filter operation per the Normalized Innovation Squared (NIS) consistency test, which is the recommended approach when ground truth is unavailable.

#### Trajectory Comparison

**Sequence 00:**

![seq00 trajectory](part1_ekf_odom/figures/seq00/trajectory.png)

**Sequence 01:**

![seq01 trajectory](part1_ekf_odom/figures/seq01/trajectory.png)

**Sequence 02:**

![seq02 trajectory](part1_ekf_odom/figures/seq02/trajectory.png)

#### Time Series Analysis

**Sequence 00:**

![seq00 time series](part1_ekf_odom/figures/seq00/time_series.png)

**Sequence 01:**

![seq01 time series](part1_ekf_odom/figures/seq01/time_series.png)

**Sequence 02:**

![seq02 time series](part1_ekf_odom/figures/seq02/time_series.png)

---

## Part 2: ICP Odometry Refinement

<!-- TODO: Add ICP implementation details -->

---

## Part 3: Full SLAM with slam_toolbox

<!-- TODO: Add SLAM implementation details -->

---

## Part 4: Results

<!-- TODO: Add trajectory plots and maps -->

---

## Part 5: Discussion

<!-- TODO: Add comparison and analysis -->

---

## Conclusion

<!-- TODO: Add conclusion -->
