# Project Title

Object detection utilizing YOLOv8 and ROS2 Jazzy

## Description

This project integrates ROS2 to interface with a camera for real-time image capture. YOLOv8 (You Only Look Once version 8) is utilized for efficient image processing and object detection, enabling the identification and classification of objects in the captured video feed. The system allows seamless communication between the camera and processing module, providing a powerful tool for real-time computer vision applications.

### Dependencies

* ROS 2 (Foxy, Galactic, Humble, or later): Install according to your OS. * Installation Guide
* Python 3.8+: Ensure Python is installed (python3 --version)
* OpenCV (cv2): Install via pip install opencv-python
* ROS 2 Python Client Library (rclpy): Installed with ROS 2
* sensor_msgs: Part of ROS 2 message packages
* cv_bridge (for OpenCV-ROS integration): Install via sudo apt install ros-$* {ROS_DISTRO}-cv-bridge
* Ultralytics YOLOv8: Install via pip install ultralytics
* NumPy: Install via pip install numpy


## Authors


Ali Kamran  
[Email](mailto:akamran038@gmail.com)

## Version History

* 0.1
    * Initial Release


## Acknowledgments

* [ROS2 Jazzy](https://docs.ros.org/en/jazzy/index.html)
* [YOLOv8](https://github.com/ultralytics/ultralytics/blob/main/docs/en/models/yolov8.md)
* [OpenCV](https://docs.opencv.org/4.x/index.html)
