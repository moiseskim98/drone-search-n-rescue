# Automated Drone-Based Search and Rescue System for Drowning Detection

## Overview
Drowning is a leading cause of accidental death, particularly along shorelines. This project aims to develop an **Automated Drone-Based Search and Rescue System**, addressing challenges such as complex environments, weather, and response time, by utilizing autonomous navigation, real-time object detection, and lifebuoy deployment for rapid response.

## System Functionality

### Simulation Phase
The simulation was conducted using **Webots**, an open-source robotics simulator, to model a virtual coastline. The system tested functionalities like:
- **Autonomous Navigation**
- **Object Detection** using computer vision
- **Lifebuoy Deployment** triggered by scripts

**MAVROS** and **Ardupilot** were used to control the drone in both autonomous and remote modes. Telemetry feedback, including GPS position and battery status, was transmitted through the **MavLink** protocol.

### Real-Life Application
Building on simulation results, the real-world application involved:
- **YOLOv7** for real-time drowning victim detection
- A **depth camera** for enhanced environmental perception
- **Speech Recognition** for basic commands like "yes" and "no"
- **NERV** framework for simulating and integrating real-world conditions
- A **gripper** for assistive actions, like delivering supplies

Extensive testing was conducted to validate module performance in controlled settings.

## Features
- **Autonomous Missions**: Rapid deployment for rescue operations
- **Advanced Sensor Integration**: Depth cameras and computer vision for accurate detection
- **Real-time Control**: Interactive interfaces with ground control stations
- **Simulation Testing**: Refining capabilities through Webots and NERV

## Future Prospects
- Improved system integration for seamless real-world application
- Expanded testing in varied rescue scenarios
- Development of additional modules to enhance mission capabilities

## Installation

To get started with the project:

1. Clone the repository:
   ```bash
   git clone https://github.com/yourusername/drone-search-n-rescue.git


![image](https://github.com/user-attachments/assets/ee95b53f-9c48-40e9-b865-41415b8e734f)

Slide for the project:https://www.canva.com/design/DAGVBoA6hbM/Ai6BIfBj4ClON4PuPVQ88Q/edit
