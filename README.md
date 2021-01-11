# AdaptiveCruiseControl

# Overview

* Uses a Faster R-CNN to detect highway lanes, cars, and traffic lights using videos from the KITTIdataset
* Uses various image processing and camera calibration techniques to determine detected objects’ position in 3D
space in relation to the host car
* Contains implementation of a PID based control system in MATLAB SIMULINK to operate a modern cruise control system with
lane keeping
* Uses Signal Temporal Logic specifications for model checking and robustness testing

# Run

To run this code, open a new Matlab 2020 project and import the project files. The main file to run is the ACCBreach_MP1_b.m in the MP1_b_source folder. Any file with MP1_a in the file name is strictly used for PID controller implmentation for the host vehicle. All files in the MP1_b_source folder use the PID control system in tandem with the vision system that is implemented using a faster R-CNN. The model was trained and calibrated using the KITTI dataset for highway driving.
