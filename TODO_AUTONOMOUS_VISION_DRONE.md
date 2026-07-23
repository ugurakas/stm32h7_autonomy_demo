# Autonomous Object-Tracking Drone with Live Video Streaming - TODO

> **Goal:** A fully autonomous drone that streams live camera feed to a user mobile/desktop app and performs real-time image processing to detect and follow a targeted object.

---

## 🟢 PHASE 1: Hardware Enablement (STM32H7 Firmware)

### Camera & Video Pipeline
- [ ] **1.1 DCMI Camera Driver** — Digital Camera Interface (DCMI) peripheral driver for STM32H7
    - Register-level configuration of DCMI for parallel camera sensors (OV2640/OV5640)
    - VSYNC/HSYNC/PIXCLK timing
    - Cropping and windowing support
- [ ] **1.2 Camera Sensor Driver (OV2640/OV5640)** — I2C-based sensor configuration
    - SCCB protocol over I2C for register writes
    - Resolution config (QVGA 320x240 for processing, VGA 640x480 for streaming)
    - JPEG output mode for bandwidth efficiency
    - FPS control (target 15-30 FPS)
- [ ] **1.3 DMA Double-Buffer for Camera** — Continuous frame capture
    - DMA2 stream configuration for DCMI
    - Double-buffering (ping-pong) for zero-copy frame acquisition
    - Frame-complete interrupt with callback
- [ ] **1.4 ESP32/ESP8266 WiFi Module Driver** — UART-based AT command bridge
    - WiFi station mode configuration
    - TCP/UDP socket client
    - HTTP/MJPEG streaming server on the coprocessor
    - OTA update channel

### Video Encoding & Streaming
- [ ] **1.5 MJPEG Streaming Server** — Embedded HTTP server on ESP32
    - JPEG compression on STM32 (accelerated or software)
    - MJPEG-over-HTTP boundary separator framing
    - Configurable quality vs bandwidth (10-90% JPEG quality)
    - Multi-client support (max 2-3 concurrent streams)
- [ ] **1.6 Video Buffer Management** — Ring buffer for frame queuing
    - 3-5 frame buffer pool
    - Frame drop policy on buffer full
    - Timestamp embedding per frame
- [ ] **1.7 Low-Latency Streaming Protocol** — UDP-based custom protocol
    - RTP-like packetization with sequence numbers
    - Fragment large JPEG frames (max MTU 1460 bytes)
    - NAK-based retransmission for keyframes

### Communication Back-Channel
- [ ] **1.8 Bidirectional Command/Telemetry Link** — Concurrent with video
    - Telemetry multiplexing over the same WiFi link
    - Command packets from app → drone (target selection, mode change)
    - Telemetry packets from drone → app (battery, GPS, altitude, heading)
    - Prioritization: commands > telemetry > video frames

---

## 🟡 PHASE 2: Object Detection & Tracking (Image Processing)

### Onboard Image Processing (STM32H7)
- [ ] **2.1 Color-Based Object Detection** — Fast, lightweight detector
    - HSV color thresholding
    - Morphological operations (erode/dilate) for noise removal
    - Contour/blob detection for object isolation
    - Bounding box computation (center X/Y, width, height)
- [ ] **2.2 Object Tracking Filter** — State estimation for tracked object
    - Kalman Filter (linear constant velocity model) for object position
    - Prediction: where will the object be in the next frame?
    - Update: correct prediction with new detection
    - Handles temporary occlusion (2-3 seconds)
- [ ] **2.3 Feature-Based Tracking (Optical Flow)** — Lucas-Kanade tracker
    - Corner detection (fast/Shi-Tomasi) for feature points
    - Sparse optical flow between frames
    - Object motion vector calculation
    - Fallback to color detection when features are lost
- [ ] **2.4 CNN-Based Detection (TensorFlow Lite Micro)** — For advanced targets
    - TFLite model conversion and quantization (int8)
    - Model deployment on STM32H7 (1MB RAM budget)
    - Person/face/car detection models (~300KB model size)
    - Inference at 1-5 FPS with CMSIS-NN optimizations
- [ ] **2.5 Object Re-Identification** — Maintain lock on correct target
    - Feature vector extraction (color histogram + HOG)
    - Simple similarity matching between frames
    - Target confidence scoring
    - Re-acquisition after target loss

### Image Processing Pipeline
- [ ] **2.6 Frame Preprocessing** — Before detection
    - Color space conversion (RGB → HSV, RGB → Gray)
    - Downscaling (QVGA 320x240 for processing)
    - Histogram equalization (poor lighting compensation)
    - Rolling shutter correction
- [ ] **2.7 ROI (Region of Interest) Tracking** — Optimize processing
    - Only process the region around the tracked object
    - ROI expands on low confidence (search mode)
    - ROI shrinks on high confidence (track mode)
    - Reduces processing load by 60-80%

---

## 🔵 PHASE 3: Autonomous Target Following & Control

### Target-Follow Controller
- [ ] **3.1 Visual Servoing Controller** — Position-based visual servoing (PBVS)
    - Input: object position error (pixels from image center)
    - Output: yaw rate, pitch rate, throttle adjustments
    - Proportional control on X-error → yaw rate
    - Proportional control on Y-error → pitch/altitude
    - Proportional control on object size → throttle (distance keeping)
- [ ] **3.2 Distance Keeping Logic** — Maintain safe follow distance
    - Target distance estimation (from known object size or sonar)
    - Desired follow distance: configurable (2m / 5m / 10m)
    - Brake when target stops (throttle reduction)
    - Accelerate when target moves away
- [ ] **3.3 Obstacle Avoidance Override** — Safety during following
    - Ultrasonic/ToF sensor for forward obstacle detection
    - Pause follow and hover on obstacle < 2m
    - Lateral avoidance (move left/right) if path is blocked
    - Resume follow after obstacle cleared
- [ ] **3.4 Smooth Trajectory Generation** — Jerk-limited motion
    - Minimum-jerk trajectory for velocity commands
    - Rate limiting on yaw/pitch/throttle changes
    - Target velocity smoothing with low-pass filter
    - Prevents motion sickness and aggressive maneuvers

### Autonomous Behaviors
- [ ] **3.5 Target Acquisition** — Search and lock
    - Hover at current position and scan yaw (360° sweep)
    - Report "target acquired" via telemetry
    - Auto-start following on acquisition
    - Manual target selection via app (tap-to-track)
- [ ] **3.6 Target Loss Handling** — Graceful degradation
    - Timer-based: 1s = increase search area, 3s = return to last known position
    - 5s = abort follow, hover in place, notify app
    - 10s = Return-to-Home (RTH) if GPS available
- [ ] **3.7 Return-to-Home** — Safe return on critical failure
    - GPS-based position hold and return
    - Altitude gain during return (clear obstacles)
    - Auto-land at home position
    - Geofence enforcement (max follow distance 100m)

---

## 🟠 PHASE 4: Mobile/Desktop User Application

### Application Features
- [ ] **4.1 Live Video Viewer** — The core UI
    - MJPEG/JPEG-over-UDP stream decoding
    - Hardware-accelerated rendering (OpenGL ES / Metal)
    - Frame rate display (FPS counter)
    - Resolution and quality indicator
    - Buffering indicator (latency bar)
- [ ] **4.2 Telemetry Dashboard** — Real-time data overlay
    - Battery voltage (with percentage)
    - Altitude (barometer + GPS)
    - GPS coordinates + map view
    - Flight mode, armed status
    - Signal strength (RSSI)
    - Connection status
- [ ] **4.3 Touch-to-Track** — User selects target
    - Tap on video feed → send coordinates to drone
    - Drone computes object position from tap point
    - Visual feedback: bounding box on tracked object
    - Double-tap to release target
- [ ] **4.4 Flight Controls Overlay** — Manual override
    - Virtual joystick for yaw/throttle (right stick)
    - Virtual joystick for pitch/roll (left stick)
    - Arm/Disarm button
    - Takeoff/Land/Hold buttons
    - Emergency stop (kill switch)
    - Autonomous mode toggle
- [ ] **4.5 Video Recording & Capture** — Save footage
    - Record video to device storage
    - Snapshot capture (JPEG still)
    - Geotagging for captured media
    - Playback mode for recorded flights

### Application Platforms
- [ ] **4.6 iOS Application** (Swift/SwiftUI + AVFoundation)
    - MJPEG stream parser
    - Metal rendering pipeline
    - CoreLocation for user map position
    - Background task handling
- [ ] **4.7 Android Application** (Kotlin/Jetpack Compose + CameraX)
    - MJPEG stream parser
    - SurfaceView/TextureView rendering
    - Fused Location Provider
    - Background service architecture
- [ ] **4.8 Web/Desktop Application** (Electron or React + Node.js)
    - WebSocket-based MJPEG streaming
    - Canvas-based rendering
    - Map integration (Leaflet/Mapbox)
    - Cross-platform (Windows/macOS/Linux)
- [ ] **4.9 Companion Backend Server** (optional)
    - WebRTC signaling server for low-latency video
    - Telemetry relay and logging
    - Multi-drone support
    - REST API for flight data retrieval

---

## 🔴 PHASE 5: System Integration & Testing

### Hardware Integration
- [ ] **5.1 Camera Module Integration** — Physical connection
    - OV2640/OV5640 camera module wiring (DCMI + I2C)
    - ESP32 module (UART + power) for WiFi
    - Power budgeting: camera + ESP32 + STM32H7
    - EMI shielding for video signal integrity
- [ ] **5.2 End-to-End Video Pipeline** — From sensor to app
    - Camera capture @ 15 FPS → DMA to SRAM
    - JPEG compression on STM32 (or passthrough to ESP32)
    - WiFi transmission → App reception and rendering
    - Latency measurement target: <200ms end-to-end
- [ ] **5.3 Sensor Fusion with Vision** — IMU + GPS + Camera
    - IMU attitude (roll/pitch/yaw) for horizon stabilization
    - GPS position for geo-referencing tracked objects
    - Barometer altitude for height keeping
    - Camera optical flow for velocity estimation (VIO)

### Testing
- [ ] **5.4 Ground Testing**
    - Camera streaming range test (WiFi distance)
    - Object detection accuracy test (various lighting)
    - Latency measurement under load
    - Power consumption profiling
- [ ] **5.5 Flight Testing**
    - Target following with slow-moving person
    - Target following with fast-moving vehicle
    - Target loss and re-acquisition
    - RTH failsafe test
    - Battery endurance test (with camera + WiFi)
- [ ] **5.6 Safety Validation**
    - Obstacle avoidance collision test
    - Geofence boundary enforcement
    - Kill switch responsiveness
    - Low-battery auto-land test
    - Communication loss behavior

---

## 📊 System Architecture Overview

```
┌──────────────────────────────────────────────┐
│                 STM32H743                     │
│  ┌──────────┐  ┌─────────────────────────┐   │
│  │ DCMI     │  │ Image Processing         │   │
│  │ Camera   │──│ • Color Detection        │   │
│  │ Capture  │  │ • Optical Flow           │   │
│  │ + DMA    │  │ • TFLite CNN (optional)  │   │
│  └──────────┘  │ • Object Tracking Filter │   │
│                └────────────┬────────────┘   │
│  ┌──────────┐               │                │
│  │ Sensor   │  ┌────────────▼────────────┐   │
│  │ Fusion   │──│ Visual Servoing +       │   │
│  │ (IMU/GPS)│  │ Follow Controller       │   │
│  └──────────┘  └────────────┬────────────┘   │
│                             │                │
│  ┌──────────┐  ┌────────────▼────────────┐   │
│  │ Motor    │  │ Motor Mixer + PWM Out   │   │
│  │ Mixer    │◄─│ (Flight Control)        │   │
│  └──────────┘  └─────────────────────────┘   │
│                             │                │
│  ┌──────────────────────────▼──────────────┐ │
│  │ UART Bridge to ESP32 (Video + Command) │ │
│  └─────────────────────────────────────────┘ │
└──────────────────────┬───────────────────────┘
                       │ UART @ 921600 baud
┌──────────────────────▼───────────────────────┐
│              ESP32 / ESP8266                  │
│  ┌─────────────────────────────────────────┐ │
│  │ • TCP/IP Stack                          │ │
│  │ • MJPEG HTTP Server                     │ │
│  │ • UDP Video Streaming                   │ │
│  │ • Command/Telemetry Relay               │ │
│  │ • WiFi Access Point or Station Mode     │ │
│  └─────────────────────────────────────────┘ │
└──────────────────────┬───────────────────────┘
                       │ WiFi
        ┌──────────────┴──────────────┐
        │                             │
┌───────▼───────┐           ┌─────────▼──────┐
│  iOS App      │           │  Android App   │
│  (Swift)      │           │  (Kotlin)      │
└───────────────┘           └────────────────┘
```

## Priority Matrix

| Feature | Effort | Impact | Priority |
|---------|--------|--------|----------|
| DCMI Camera + DMA | Medium | Critical | P0 |
| ESP32 WiFi Bridge | Medium | Critical | P0 |
| MJPEG Streaming | Medium | Critical | P0 |
| Color Object Detection | Low | High | P0 |
| Visual Servoing Control | Medium | Critical | P0 |
| Mobile App (iOS/Android) | High | Critical | P0 |
| Touch-to-Track | Medium | High | P1 |
| Kalman Filter Tracking | Medium | High | P1 |
| Obstacle Avoidance | Medium | High | P1 |
| Optical Flow Tracking | High | Medium | P2 |
| TFLite CNN Detection | High | Medium | P2 |
| Return-to-Home | Medium | Medium | P2 |
| Video Recording | Medium | Low | P3 |
| Web Dashboard | High | Low | P3 |
| Multi-Drone Support | High | Low | P4 |

---

## Milestones

| Milestone | Target | Deliverable |
|-----------|--------|-------------|
| M1: Raw Video | Week 2 | Camera → DMA → WiFi → App shows live feed |
| M2: Object Detection | Week 4 | App shows bounding box on tracked object |
| M3: Follow Me | Week 6 | Drone autonomously follows a person in open field |
| M4: Robust Tracking | Week 8 | Tracking works in varied lighting, occlusions handled |
| M5: Production Ready | Week 12 | Full app, safety features, reliable following |
