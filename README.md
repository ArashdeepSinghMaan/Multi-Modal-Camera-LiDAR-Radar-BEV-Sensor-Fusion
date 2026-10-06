# Multi-Modal Camera–LiDAR–Radar BEV Sensor Fusion

A ROS 2-based perception system that transforms **camera, LiDAR, and Radar measurements into a common Bird's-Eye View (BEV) representation** for 3D object detection, sensor fusion, and multi-object tracking.

---

# 1. Project Overview

Autonomous vehicles observe the environment using multiple complementary sensors.

```text
Camera
  |
  +-- semantic information
  +-- appearance
  +-- texture
  +-- 2D localization

LiDAR
  |
  +-- 3D geometry
  +-- depth
  +-- spatial structure

Radar
  |
  +-- range
  +-- radial velocity
  +-- Doppler
  +-- RCS
```

The challenge is to combine these measurements into a common representation.

This project uses **Bird's-Eye View (BEV)** as the common coordinate space.

```text
Camera
   |
   v
Image Features
   |
   v
Camera → BEV
   |
   +------------------+
                      |
LiDAR → BEV ----------+----> BEV Fusion
                      |
Radar → BEV ----------+
                      |
                      v
                BEV Representation
                      |
                      v
                3D Detection
                      |
                      v
                 Tracking
```

---

# 2. Main Objectives

- Understand BEV representations
- Transform heterogeneous sensors into a common coordinate frame
- Project camera information into BEV
- Project LiDAR into BEV
- Project Radar measurements into BEV
- Fuse heterogeneous sensor information
- Incorporate Radar velocity into BEV
- Detect 3D objects
- Track objects over time
- Evaluate individual sensors against fused perception

---

# 3. Why BEV?

Images are naturally represented in perspective:

```text
             Camera

        ----------------
        \              /
         \            /
          \          /
           \        /
            \______/
```

LiDAR and Radar measurements are naturally 3D/spatial.

BEV converts everything into a top-down representation:

```text
                FRONT

        +--------------------+
        |                    |
        |       CAR          |
        |      ████          |
        |                    |
        |                    |
        |   CAR              |
        |  ████              |
        |                    |
        |       EGO          |
        |       ███          |
        +--------------------+

                 REAR
```

This makes spatial fusion significantly easier.

---

# 4. Target BEV Grid

Define:

```text
X:
-10 m → 80 m

Y:
-30 m → 30 m

Resolution:
0.2 m/cell
```

Example:

```yaml
bev:
  x_min: -10.0
  x_max: 80.0

  y_min: -30.0
  y_max: 30.0

  resolution: 0.2
```

The exact values depend on the application.

---

# 5. Sensor Coordinate Systems

The system will maintain:

```text
map
 |
 +-- odom
      |
      +-- base_link
             |
             +-- camera_link
             |
             +-- lidar_link
             |
             +-- radar_link
```

All measurements are transformed into:

```text
base_link
```

or a dedicated:

```text
bev_frame
```

---

# 6. Camera-to-BEV Transformation

Camera images use perspective projection.

The camera projection model is:

\[
p=K[R|t]P
\]

where:

\[
K=
\begin{bmatrix}
f_x&0&c_x\\
0&f_y&c_y\\
0&0&1
\end{bmatrix}
\]

To transform image information into BEV, the system can initially use a ground-plane assumption.

For a ground point:

\[
Z=0
\]

the perspective transformation can be represented using a homography:

\[
p_{image}=Hp_{BEV}
\]

and therefore:

\[
p_{BEV}=H^{-1}p_{image}
\]

---

# 7. Camera BEV Pipeline

```text
Camera Image
     |
     v
YOLO / Feature Extractor
     |
     v
2D Detections / Features
     |
     v
Camera Calibration
     |
     v
Ground Projection
     |
     v
BEV Features
```

Example output:

```text
Car:
BEV center = (24.2, -3.1)
Width = 1.9 m
Length = 4.3 m
Confidence = 0.91
```

---

# 8. LiDAR-to-BEV

LiDAR points are already spatial.

For each point:

\[
(x,y,z)
\]

map:

\[
x_{bev}=
\frac{x-x_{min}}{resolution}
\]

\[
y_{bev}=
\frac{y-y_{min}}{resolution}
\]

Each point contributes to the BEV representation.

Possible BEV channels:

```text
Height
Intensity
Point density
Ground probability
Obstacle probability
```

---

# 9. Radar-to-BEV

Radar points provide:

```text
x
y
z
radial velocity
RCS
SNR
```

Radar can therefore create BEV channels such as:

```text
Radar occupancy
Radar velocity
Radar RCS
Radar point density
Radar confidence
```

Example:

```text
BEV Radar Tensor

Channel 0 → occupancy
Channel 1 → radial velocity
Channel 2 → RCS
Channel 3 → point density
```

---

# 10. Doppler as a BEV Feature

This is one of the main advantages of Radar.

For each Radar point:

\[
v_r =
\frac{\lambda f_D}{2}
\]

where:

- \(\lambda\) = Radar wavelength
- \(f_D\) = Doppler frequency shift

The BEV representation can preserve this velocity information.

Example:

```text
                 FRONT

          +----------------+
          |       ↑↑       |
          |      CAR       |
          |                |
          |                |
          |  ← CAR         |
          |                |
          +----------------+
```

The arrows represent estimated radial motion.

---

# 11. Multi-Modal BEV Tensor

The initial representation can contain:

```text
Camera:
    semantic features

LiDAR:
    height
    density
    intensity

Radar:
    occupancy
    velocity
    RCS
```

Combined representation:

\[
F_{BEV}
=
[F_{camera},F_{lidar},F_{radar}]
\]

---

# 12. Fusion Strategies

The project will investigate three levels.

## Early Fusion

Concatenate sensor features:

\[
F=[F_C,F_L,F_R]
\]

Then process using a neural network.

---

## Intermediate Fusion

Each sensor has an encoder:

```text
Camera → Encoder
LiDAR  → Encoder
Radar  → Encoder
```

Then:

```text
Camera features
       +
LiDAR features
       +
Radar features
       |
       v
Fusion Network
```

---

## Late Fusion

Each sensor produces detections independently:

```text
Camera → detections
LiDAR  → detections
Radar  → detections
             |
             v
      Detection Fusion
```

The initial implementation should use late fusion because it is easier to debug.

Intermediate BEV fusion becomes the advanced stage.

---

# 13. Baseline Architecture

```text
                 CAMERA
                    |
                 YOLO
                    |
                2D Boxes
                    |
                    v
                 Camera
                  BEV
                    |
                    |
                    +----------------+
                                     |
                 LiDAR               |
                   |                 |
             Point Processing        |
                   |                 |
                LiDAR BEV            |
                   |                 |
                   +--------+--------+
                            |
                 Radar      |
                   |        |
             Point Processing
                   |
               Radar BEV
                   |
                   +--------+
                            |
                            v
                       BEV Fusion
                            |
                            v
                    3D Object Detection
                            |
                            v
                         Tracking
```

---

# 14. 3D Object Representation

Each detected object:

```text
id
class
x
y
z
length
width
height
yaw
velocity
confidence
```

Example:

```yaml
object:
  id: 17
  class: car

  position:
    x: 25.2
    y: -3.2
    z: 0.8

  dimensions:
    length: 4.4
    width: 1.9
    height: 1.6

  yaw: 1.54

  velocity:
    vx: 6.2
    vy: 0.1

  confidence: 0.93
```

---

# 15. Detection Head

The BEV detector can initially predict:

```text
Objectness
Class
Center
Dimensions
Orientation
Velocity
```

For each BEV cell:

\[
P(object)
\]

\[
P(class)
\]

\[
(x,y,z)
\]

\[
(l,w,h)
\]

\[
yaw
\]

\[
(v_x,v_y)
\]

---

# 16. Tracking

The output of the BEV detector is passed into a multi-object tracker.

```text
BEV Detection
      |
      v
Prediction
      |
      v
Data Association
      |
      v
Kalman Update
      |
      v
Track Management
```

State:

\[
x =
[x,y,v_x,v_y,a_x,a_y]^T
\]

Future versions may include:

\[
z,v_z
\]

---

# 17. ROS 2 Architecture

```text
Camera Driver
      |
      v
/camera/image
      |
      v
camera_bev_node
      |
      +----------------+
                       |
LiDAR Driver           |
      |                |
      v                |
/lidar/points          |
      |                |
      v                |
lidar_bev_node         |
      |                |
      +--------+-------+
               |
Radar Driver  |
      |       |
      v       |
/radar/points |
      |       |
      v       |
radar_bev_node
      |
      +-------+
              |
              v
         bev_fusion_node
              |
              v
        /bev/detections
              |
              v
         tracker_node
              |
              v
       /tracked_objects
```

---

# 18. Proposed ROS 2 Topics

```text
/camera/image_raw
/camera/camera_info

/lidar/points

/radar/points

/bev/camera_features
/bev/lidar_features
/bev/radar_features

/bev/fused_features

/bev/detections
/tracked_objects

/tf
/tf_static
```

---

# 19. Dataset

The dataset should contain synchronized:

```text
Camera
LiDAR
Radar
Calibration
Timestamps
3D annotations
```

Suitable datasets can include multimodal autonomous-driving datasets such as:

- nuScenes
- View-of-Delft
- TJ4DRadSet
- RADIal
- Astyx HiRes2019

The exact dataset should be selected based on sensor availability and licensing.

---

# 20. Synchronization

The three sensors must be synchronized.

```text
Camera:
t = 10.021 s

LiDAR:
t = 10.019 s

Radar:
t = 10.023 s
```

Use timestamp-aware synchronization.

ROS2 can use:

```text
message_filters
```

or an explicit temporal buffer.

---

# 21. Calibration

Required calibration:

```text
Camera intrinsics
Camera ↔ LiDAR
Camera ↔ Radar
LiDAR ↔ Radar
```

The transformation is:

\[
P_B=RP_A+t
\]

All sensors must ultimately be represented in the same coordinate frame.

---

# 22. Evaluation

The system will compare:

```text
Camera only
LiDAR only
Radar only
Camera + LiDAR
Camera + Radar
LiDAR + Radar
Camera + LiDAR + Radar
```

This is critical for demonstrating the actual benefit of sensor fusion.

---

# 23. Detection Metrics

```text
Precision
Recall
mAP
3D IoU
BEV IoU
Center error
Depth error
Velocity error
```

---

# 24. Tracking Metrics

```text
MOTA
MOTP
IDF1
HOTA
ID switches
Track fragmentation
```

---

# 25. BEV-Specific Metrics

Measure:

```text
BEV center error
BEV IoU
Orientation error
Velocity error
Range error
```

---

# 26. Ablation Study

Run:

```text
Camera
   ↓
Camera + LiDAR
   ↓
Camera + Radar
   ↓
LiDAR + Radar
   ↓
Camera + LiDAR + Radar
```

Measure:

```text
mAP
BEV AP
HOTA
IDF1
Velocity error
FPS
Latency
```

This will demonstrate whether Radar actually improves perception.

---

# 27. Radar-Specific Experiment

Compare:

```text
Camera + LiDAR
```

against:

```text
Camera + LiDAR + Radar
```

especially for:

```text
Moving objects
Long range
Low visibility
Partial occlusion
High relative velocity
```

The hypothesis is that Radar should particularly improve **motion estimation and dynamic-object tracking**.

---

# 28. Runtime Optimization

Target platform:

```text
NVIDIA Jetson Orin
```

Optimization:

```text
PyTorch
   ↓
ONNX
   ↓
TensorRT
   ↓
FP16
```

Additional optimization:

- CUDA preprocessing
- TensorRT inference
- ROS2 intra-process communication
- Reduced memory copies
- Point-cloud downsampling
- BEV resolution optimization

---

# 29. Target Performance

Initial targets:

```text
Camera input: 30 FPS
LiDAR input: 10–20 Hz
Radar input: 10–20 Hz

BEV perception: >20 FPS
End-to-end latency: <50 ms
```

Final performance must be measured experimentally.

---

# 30. Development Roadmap

## Phase 1

```text
[ ] Dataset setup
[ ] Sensor synchronization
[ ] Calibration
[ ] ROS2 playback
```

## Phase 2

```text
[ ] Camera BEV
[ ] LiDAR BEV
[ ] Radar BEV
```

## Phase 3

```text
[ ] Late fusion
[ ] Detection fusion
[ ] Tracking
```

## Phase 4

```text
[ ] BEV tensor representation
[ ] Intermediate feature fusion
[ ] Neural BEV detector
```

## Phase 5

```text
[ ] Radar velocity integration
[ ] Temporal fusion
[ ] Motion prediction
```

## Phase 6

```text
[ ] TensorRT
[ ] Jetson Orin
[ ] Runtime profiling
```

---

# 31. Future Extensions

- BEVFusion
- BEVFormer-style architecture
- Transformer-based fusion
- CenterPoint
- PointPillars
- Radar-camera feature fusion
- Radar velocity-aware attention
- Temporal BEV fusion
- Occupancy prediction
- 3D trajectory prediction

---

# 32. Skills Demonstrated

This project demonstrates:

### Perception

- Camera perception
- LiDAR perception
- Radar perception
- 3D detection

### Sensor Fusion

- Coordinate transformations
- Calibration
- BEV representation
- Multimodal fusion

### Tracking

- State estimation
- Data association
- Motion estimation
- Temporal fusion

### Deployment

- ROS2
- CUDA
- TensorRT
- Jetson Orin

---

# 33. Definition of Done

```text
[ ] Camera BEV representation
[ ] LiDAR BEV representation
[ ] Radar BEV representation
[ ] Sensor calibration
[ ] Temporal synchronization
[ ] Camera-LiDAR fusion
[ ] Camera-Radar fusion
[ ] LiDAR-Radar fusion
[ ] Full three-sensor fusion
[ ] 3D object detection
[ ] Velocity estimation
[ ] Multi-object tracking
[ ] Quantitative evaluation
[ ] Ablation study
[ ] TensorRT deployment
[ ] Jetson benchmark
```

---

# 34. Portfolio Outcome

The final project should demonstrate:

```text
Camera
   +
LiDAR
   +
Radar
   ↓
Common BEV Representation
   ↓
Multi-Modal Fusion
   ↓
3D Perception
   ↓
Object Tracking
   ↓
Motion Understanding
```

The project is intended to demonstrate practical knowledge of **modern multi-modal 3D perception for ADAS and autonomous systems**.
