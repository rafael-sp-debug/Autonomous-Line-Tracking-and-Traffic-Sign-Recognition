# puzzlebot_traffic_nav — Autonomous Line Tracking and Traffic Sign Recognition

A ROS 2 package that drives a **Puzzlebot** differential-drive robot through a full autonomous navigation circuit. The system detects and follows a guide line while simultaneously identifying and reacting to traffic signs ("give way", "stop", "turn_right", "turn_left", "go_straight", "roadwork") and traffic lights. 

The robot relies on a custom YOLOv11 Convolutional Neural Network for vision and a dynamic Proportional-Integral-Derivative (PID) controller for stable movement, orchestrated by a robust finite-state machine.

> Built for the *Integration of Robotics and Intelligent Systems* course (Group 501) at Tecnológico de Monterrey, Campus Estado de México. Real-hardware tested on the Puzzlebot platform utilizing a Jetson Nano and an external laptop.

![ROS 2](https://img.shields.io/badge/ROS_2-Humble-22314E?logo=ros)
![Python](https://img.shields.io/badge/Python-3.10-3776AB?logo=python&logoColor=white)
![YOLO](https://img.shields.io/badge/YOLO-v11-00FFFF?logo=yolo&logoColor=black)
![OpenCV](https://img.shields.io/badge/OpenCV-Image_Processing-5C3EE8?logo=opencv&logoColor=white)

---
[![Pickandplace](https://img.youtube.com/vi/8Jr8yP2CqAU/0.jpg)](https://www.youtube.com/watch?v=8Jr8yP2CqAU)

|  | |
|---|---|
| **Task** | Autonomous line following, intersection navigation, and dynamic speed reduction based on traffic signs and lights. |
| **Sensors** | On-board Puzzlebot camera streaming to a Jetson Nano. |
| **Vision System** | YOLOv11 for object detection and OpenCV for line segmentation and arrow direction classification. |
| **Control** | Dynamic PID controller with angular compensation. |

---

## 1. System Architecture

The project is strictly modular, distributed between the Jetson Nano (control and basic vision) and an external laptop (YOLO inference) to optimize processing. 

<img width="634" height="308" alt="image" src="https://github.com/user-attachments/assets/3b75864c-cb22-4118-b2b3-09190b8e7cba" />

| Node | Role | Key Functions |
|---|---|---|
| **`line_detector`** | Tracks the path | Converts images to grayscale, applies Otsu's binary thresholding, and uses morphological closing to calculate the lateral error and orientation angle. |
| **`pid_controller`** | Movement generation | Subscribes to the error topics and dynamically adjusts linear and angular velocities within a safe range. |
| **`yolo_detection`** | Perception | Runs YOLOv11 inferences to publish `/signal` and `/color` topics. Filters out predictions with a confidence score below 0.5. |
| **`state_machine_controller`** | Decision making | Consolidates velocity commands and perception flags into a finite-state machine to publish final `/cmd_vel` instructions. |

<img width="667" height="326" alt="image" src="https://github.com/user-attachments/assets/6adf99d7-c2ce-4661-892c-de825415241e" />

The system transitions between six primary states based on the environment: **Advance** (base line-following), **Reduction** (25% speed decrease for roadworks or yellow lights), **Ahead** (crosses intersections linearly), **Turn_left**, **Turn_right**, and **Stop** (halts completely for red lights or stop signs).

## 2. Technical Approach

*   **Adaptive Control:** The line-following algorithm uses a proportional control that integrates angular compensation. The control law applied is u(t) = Kp*e(t) + Kθ*e(θ).
*   **Directional Ambiguity Resolution:** Because "turn_left" and "turn_right" signs share strong visual similarities, the vision node applies binarization and extracts contours using `cv2.moments` to accurately determine arrow direction based on the centroid's position.
*   **Time-Consolidated Detections:** To prevent false positives caused by noise, the traffic light detector requires the color state to remain consistent for at least 1.5 seconds before validating the detection and executing a state change.

## 3. Experimental Results

The package was validated on physical hardware through rigorous scenario testing.

*   **Full Circuit Navigation:** The robot successfully completed the full circuit in 4 out of 5 tests.
*   **Lap Times:** Successful navigation cycles were completed in approximately 92 to 94 seconds.
*   **Detection Precision:** Stop, Give Way, Go Ahead, and Roadwork signs achieved a 100% detection precision rate.
*   **Traffic Lights:** Red and Green lights were detected with 100% accuracy, while the Yellow light achieved 90% accuracy due to its brief display interval.

## 4. Strengths & Limitations

**Strengths**
*   The dynamically adjusted proportional controller significantly reduces movement oscillations, resulting in highly stable navigation.
*   The YOLOv11 model, trained on a custom dataset tailored specifically to the physical track, provides highly reliable real-time decision-making.
*   The line detection pipeline proves robust in standard lighting conditions.

**Limitations**
*   The system's accuracy degrades under poor lighting or highly reflective conditions, which can obscure the guide line.
*   Turn signs (left/right) occasionally cause confusion within the neural network due to their high visual similarity, dropping detection precision to 70-80%.
*   Network latency between the Jetson Nano and the inference laptop can delay reaction times, especially when multiple robots are operating simultaneously.

***
**Authors:** Rafael Soto Padilla (A01799296).
**References:** Manchester Robotics. (2025). Final Challenge.
***
