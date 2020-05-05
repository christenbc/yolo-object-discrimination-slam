# yolo_object_discrimination_slam

This is an Open Source package.

This ROS package contains:
- laser_darknet_filter:
This is the engine of the algorithm

- struct_gmapping:
This is a modification of http://wiki.ros.org/gmapping in order to make laser_darknet_filter work properly

Dependencies: https://github.com/leggedrobotics/darknet_ros

Technical note: From YoloObjectDetector.cpp in line 588, instead of "if (num > 0 && num <= 100) {" replace with "if (num >= 0 && num <= 100) {".

https://www.youtube.com/watch?v=92vfkuiwe_Y

This repository was made as part of my MSc's thesis.

Implemented in ROS for a higher-fidelity approach in autonomous mobile robot exploration using LIDAR sensors and a depth camera integrated with the MiR100 robot. In addition, by using state-of-the-art deep learning object detector like YOLOv3, the system filtered out undesired objects from the navigation map.

Please, do not forget to referenciate me if you use any piece of my work.
The documentation is still in process to be completed.
