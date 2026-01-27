# LAB1: Kalman Filter / SLAM

## Table of Contents
- [LAB1: Kalman Filter / SLAM](#lab1-kalman-filter--slam)
  - [Table of Contents](#table-of-contents)
  - [Project Overview](#project-overview)
  - [System Architecture](#system-architecture)
  - [Setup](#setup)
    - [Running](#running)
  - [Part 1: EKF Odometry Fusion](#part-1-ekf-odometry-fusion)
    - [Overview](#overview)
    - [1.1 Wheel Odometry from Joint States](#11-wheel-odometry-from-joint-states)
      - [Wheel Displacement](#wheel-displacement)
      - [ICC (Instantaneous Center of Curvature) Kinematics](#icc-instantaneous-center-of-curvature-kinematics)
      - [Robot Velocity](#robot-velocity)
    - [1.2 Odometry Covariance Model](#12-odometry-covariance-model)
      - [Step-by-Step Matrix Calculation](#step-by-step-matrix-calculation)
    - [1.3 Twist Covariance](#13-twist-covariance)
      - [Step-by-Step Matrix Calculation](#step-by-step-matrix-calculation-1)
    - [1.4 Extended Kalman Filter](#14-extended-kalman-filter)
    - [1.5 Implementation](#15-implementation)
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
ros2 bag play FRA532_LAB1_DATASET/fibo_floor3_seq00

# Sequence 01 - Non-Empty Hallway with Sharp Turns
ros2 bag play FRA532_LAB1_DATASET/fibo_floor3_seq01

# Sequence 02 - Non-Empty Hallway with Non-Aggressive Motion
ros2 bag play FRA532_LAB1_DATASET/fibo_floor3_seq02
```

---

## Part 1: EKF Odometry Fusion

### Overview

This part implements sensor fusion between wheel odometry and IMU using an Extended Kalman Filter (EKF).

**References:**
- **Forward Kinematics (ICC Method):** Columbia University CS4733 Lecture Notes - [ICC Kinematics](https://www.cs.columbia.edu/~allen/F17/NOTES/icckinematics.pdf)
- **Covariance Propagation:** Correll, N. (2022). *Introduction to Autonomous Robots*. Chapter 8: Uncertainty and Error Propagation - [Error Propagation](https://eng.libretexts.org/Bookshelves/Mechanical_Engineering/Introduction_to_Autonomous_Robots_(Correll)/08:_Uncertainty_and_Error_Propagation/8.02:_Error_Propogation)
- **Odometry Error Model:** Chong, K. S., & Kleeman, L. (1997). *Accurate Odometry and Error Modelling for a Mobile Robot* - [PDF](https://www.cs.cmu.edu/~motionplanning/papers/sbp_papers/kalman/chong_accurate_odometry_error.pdf)

### 1.1 Wheel Odometry from Joint States

We compute wheel odometry from `/joint_states` which provides wheel encoder positions and velocities at 20 Hz.

#### Wheel Displacement

$$\Delta s_r = \Delta \theta_r \cdot r$$

$$\Delta s_l = \Delta \theta_l \cdot r$$

Where:
- $\Delta \theta_r, \Delta \theta_l$ = change in wheel encoder positions (radians)
- $r$ = wheel radius (0.033 m for TurtleBot3 Burger)

#### ICC (Instantaneous Center of Curvature) Kinematics

For differential drive robots, we use the ICC method for accurate pose integration:

**Heading change:**

$$\Delta \theta = \frac{\Delta s_r - \Delta s_l}{b}$$

**Turning radius:**

$$R = \frac{b}{2} \cdot \frac{\Delta s_l + \Delta s_r}{\Delta s_r - \Delta s_l}$$

**ICC point:**

$$ICC_x = x - R \cdot \sin(\theta)$$

$$ICC_y = y + R \cdot \cos(\theta)$$

**Pose update:**

$$\begin{bmatrix} x' \\ y' \end{bmatrix} = \begin{bmatrix} \cos(\Delta\theta) & -\sin(\Delta\theta) \\ \sin(\Delta\theta) & \cos(\Delta\theta) \end{bmatrix} \begin{bmatrix} x - ICC_x \\ y - ICC_y \end{bmatrix} + \begin{bmatrix} ICC_x \\ ICC_y \end{bmatrix}$$

$$\theta' = \theta + \Delta\theta$$

**Special case (straight line, $|\Delta s_r - \Delta s_l| < \epsilon$):**

When the robot moves straight, $R \to \infty$ and we use:

$$\begin{bmatrix} x' \\ y' \end{bmatrix} = \begin{bmatrix} x \\ y \end{bmatrix} + \frac{\Delta s_l + \Delta s_r}{2} \begin{bmatrix} \cos(\theta) \\ \sin(\theta) \end{bmatrix}$$

Where $b$ = track width (0.160 m for TurtleBot3 Burger).

#### Robot Velocity

$$v = \frac{v_r + v_l}{2}$$

$$\omega = \frac{v_r - v_l}{b}$$

### 1.2 Odometry Covariance Model

We implement a wheel-level error model using standard EKF Jacobian-based covariance propagation:

$$\Sigma_{p'} = F_p \cdot \Sigma_p \cdot F_p^T + F_{\Delta} \cdot \Sigma_{\Delta} \cdot F_{\Delta}^T$$

#### Step-by-Step Matrix Calculation

**Given values at each timestep:**
- $\Delta s_r$, $\Delta s_l$ = wheel displacements (meters)
- $\Delta\theta = \frac{\Delta s_r - \Delta s_l}{b}$ = heading change
- $\theta_{mid} = \theta + \frac{\Delta\theta}{2}$ = midpoint heading
- $k_r = k_l = 0.1$ = slip coefficients

---

**Step 1: Build Pose Jacobian $F_p$ (3×3)**

$F_p$ is the Jacobian of the new pose with respect to the previous pose:

$$F_p = \begin{bmatrix} \frac{\partial x'}{\partial x} & \frac{\partial x'}{\partial y} & \frac{\partial x'}{\partial \theta} \\ \frac{\partial y'}{\partial x} & \frac{\partial y'}{\partial y} & \frac{\partial y'}{\partial \theta} \\ \frac{\partial \theta'}{\partial x} & \frac{\partial \theta'}{\partial y} & \frac{\partial \theta'}{\partial \theta} \end{bmatrix}$$

From the rotation matrix in pose update, taking partial derivatives:

$$F_p = \begin{bmatrix} \cos(\Delta\theta) & -\sin(\Delta\theta) & 0 \\ \sin(\Delta\theta) & \cos(\Delta\theta) & 0 \\ 0 & 0 & 1 \end{bmatrix}$$

---

**Step 2: Build Wheel Displacement Jacobian $F_{\Delta}$ (3×2)**

$F_{\Delta}$ is the Jacobian of pose change with respect to wheel displacements $[\Delta s_r, \Delta s_l]^T$.

Using midpoint approximation for pose change:
- $\Delta x \approx \frac{\Delta s_r + \Delta s_l}{2} \cos(\theta_{mid})$
- $\Delta y \approx \frac{\Delta s_r + \Delta s_l}{2} \sin(\theta_{mid})$
- $\Delta \theta = \frac{\Delta s_r - \Delta s_l}{b}$

Taking partial derivatives:

$$F_{\Delta} = \begin{bmatrix} \frac{\partial \Delta x}{\partial \Delta s_r} & \frac{\partial \Delta x}{\partial \Delta s_l} \\ \frac{\partial \Delta y}{\partial \Delta s_r} & \frac{\partial \Delta y}{\partial \Delta s_l} \\ \frac{\partial \Delta \theta}{\partial \Delta s_r} & \frac{\partial \Delta \theta}{\partial \Delta s_l} \end{bmatrix} = \begin{bmatrix} \frac{\cos(\theta_{mid})}{2} & \frac{\cos(\theta_{mid})}{2} \\ \frac{\sin(\theta_{mid})}{2} & \frac{\sin(\theta_{mid})}{2} \\ \frac{1}{b} & -\frac{1}{b} \end{bmatrix}$$

---

**Step 3: Build Wheel Covariance $\Sigma_{\Delta}$ (2×2)**

$$\Sigma_{\Delta} = \begin{bmatrix} k_r \cdot |\Delta s_r| + \epsilon & 0 \\ 0 & k_l \cdot |\Delta s_l| + \epsilon \end{bmatrix}$$

Where $\epsilon = 10^{-6}$ prevents singularity.

---

**Step 4: Propagate Previous Covariance**

$$\text{Term}_1 = F_p \cdot \Sigma_p \cdot F_p^T$$

This rotates the previous pose covariance by $\Delta\theta$.

---

**Step 5: Add Process Noise from Wheels**

$$\text{Term}_2 = F_{\Delta} \cdot \Sigma_{\Delta} \cdot F_{\Delta}^T$$

---

**Step 6: Final Covariance Update**

$$\Sigma_{p'} = \text{Term}_1 + \text{Term}_2$$

The result is a (3×3) covariance matrix:
$$\Sigma_{p'} = \begin{bmatrix} \sigma_x^2 & \sigma_{xy} & \sigma_{x\theta} \\ \sigma_{xy} & \sigma_y^2 & \sigma_{y\theta} \\ \sigma_{x\theta} & \sigma_{y\theta} & \sigma_\theta^2 \end{bmatrix}$$

For ROS Odometry message, we extract diagonal elements:
- `pose.covariance[0]` = $\sigma_x^2$
- `pose.covariance[7]` = $\sigma_y^2$
- `pose.covariance[35]` = $\sigma_\theta^2$

### 1.3 Twist Covariance

For instantaneous velocity covariance, we use the same Jacobian-based error propagation:

$$\Sigma_{twist} = J_{vel} \cdot \Sigma_{wheel} \cdot J_{vel}^T$$

#### Step-by-Step Matrix Calculation

**Given values:**
- $v_r = \omega_r \cdot r$ = right wheel linear velocity
- $v_l = \omega_l \cdot r$ = left wheel linear velocity
- $b = 0.160$ m = track width
- $k_r = k_l = 0.1$ = slip coefficients

---

**Step 1: Build Velocity Jacobian $J_{vel}$ (3×2)**

Maps wheel velocities $[v_r, v_l]^T$ to robot velocities $[v_x, v_y, \omega]^T$:

$$J_{vel} = \begin{bmatrix} \frac{\partial v_x}{\partial v_r} & \frac{\partial v_x}{\partial v_l} \\ \frac{\partial v_y}{\partial v_r} & \frac{\partial v_y}{\partial v_l} \\ \frac{\partial \omega}{\partial v_r} & \frac{\partial \omega}{\partial v_l} \end{bmatrix}$$

From the velocity equations $v_x = \frac{v_r + v_l}{2}$ and $\omega = \frac{v_r - v_l}{b}$:

$$J_{vel} = \begin{bmatrix} \frac{1}{2} & \frac{1}{2} \\ 0 & 0 \\ \frac{1}{b} & -\frac{1}{b} \end{bmatrix}$$

---

**Step 2: Build Wheel Velocity Covariance $\Sigma_{wheel}$ (2×2)**

$$\Sigma_{wheel} = \begin{bmatrix} k_r \cdot |v_r| + \epsilon & 0 \\ 0 & k_l \cdot |v_l| + \epsilon \end{bmatrix}$$

---

**Step 3: Compute Final Twist Covariance (3×3)**

$$\Sigma_{twist} = J_{vel} \cdot \Sigma_{wheel} \cdot J_{vel}^T$$

---

**Step 4: Extract for ROS Message**

For ROS Odometry twist covariance (6×6 matrix as 36-element array):
- `twist.covariance[0]` = $\sigma_{v_x}^2$
- `twist.covariance[7]` = $\sigma_{v_y}^2$ (0 for diff-drive, no lateral motion)
- `twist.covariance[35]` = $\sigma_\omega^2$

### 1.4 Extended Kalman Filter

<!-- TODO: Add EKF equations (predict and correct steps) -->

### 1.5 Implementation

The wheel odometry node is implemented in Python:
- **File:** `src/turtle_odometry/scripts/turtle_wheel_odometry.py`
- **Input:** `/joint_states` (sensor_msgs/JointState)
- **Output:** `/odom` (nav_msgs/Odometry)

The EKF node is implemented in C++:
- **File:** `src/turtle_ekf/src/ekf_node.cpp`
- **Input:** `/odom`, `/imu`
- **Output:** `/odometry/filtered`, TF: `odom` -> `base_footprint`

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
