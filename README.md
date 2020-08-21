# UAV-Search

## Backstory

This project is done as part of my graduation project as a senior aerospace engineering student at Cairo University. The project is also a step towards participating in the "UAV Challenge - Medical Rescue" competition in Australia (that was meant to be on septemper, 2020 but now is planned to take place on 2021 due to SARS-CoV-2 "COVID-19" outbreak)

## What is this?

- [Video Demo](https://www.youtube.com/watch?v=-0mwb_KjFy8)
- This is an ongoing project to implement a "search & rescue" mission with a VTOL aircraft.
- It's meant to be running on an on-board computer (which is currently decided to be the Nvidia Jetson Nano) that would work with the Pixhawk controller running the PX4 firmware [Communication are done via the MAVlink protocol using the MAVSDK-Python library].
- Right now, I have implemented the following:
  - **A\* 2D path planning:**
    - Currently designed to run as an 'offline' planner that would only run once at the beginning of the mission, knowing the map, the no-fly zones and the goal/target location, the planner plans a path to the goal location avoiding the no-fly zones.
    - Returned path waypoints are uploaded to the Pixhawk controller as Mission Waypoints via MAVSDK.
  - **Object Detection:**
    - Designed to detect objects [humans in the case of UAV Challenge] using a camera mounted on the aircraft.
    - Currently implemented with Tensorflow2, the code can use mainly ***three*** different models which are:
      1. YOLOv3 & YOLOv3-Tiny - running with tensorflow2 (will be optimized with tensorRT in the future to run even faster on the on-board computer)
      2. SSD-mobilenet-COCO (all versions) & SSDlite from the tensorflow detection model zoo [not optimized]
      3. A TensorRT optimized model of the original SSDlite-mobilenet model. *[currently runs at >10fps on Nvidia Jetson Nano]*
  - **Camera-Based Localization of Detected Objects:**
    - [Video Demo](https://www.youtube.com/watch?v=lc4bMRS7228)
    - Currently implements localization based on the simple triangle-similarity method, assumming the camera is always facing directly downwards.
    - Located objects pixel center is transformed into the global coordinates, based on the UAV location at the time of capturing the image and the yaw angle of the UAV.

## Clone Notes
- This repository contains some files that are tracked by "git-lfs" (git large file system). To clone the full version of these files you need to `git clone <repo-link>` then navigate to the repo directory and `git lfs pull` to pull the full version of the lfs tracked files into your directory.