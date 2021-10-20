autoware_lidar_camera_fusion_sample
====

This package is for

* Example of internal calibration of an RGB camera
* Example of external calibration between an RGB camera and an LiDAR

based on [Autoware.AI](https://github.com/Autoware-AI).

Gazebo-related files are prepared to try a procedure of each calibration in Gazebo simulation.

# Gazebo Environment

There are in the environment
* Checker board
* RGB camera(Tiny orange box)
* Velodyne(VLP-16, HDL-32).

You may insert other objects in the environment.

If you use Autoware in docker container, you can execute Gazebo simulation in your host.
To execute it, `$ roslaunch autoware_lidar_camera_fusion_sample sensors.launch`.


Some topics are published from gazebo simulatior and next topics are used in explanation.

* /rgb_camera/image_raw
* /rgb_camera/camera_info_nominal
* /velodyne_points2.

`/rgb_camera/image_raw` is raw image from orange RGB-camera.
And `/rgb_camera/camera_info_nominal` includes metadata of images from RGB-camera.
However, the metadata should be obtained via internal calibration and published as `/rgb_camera/camera_info`.
You use `/rgb_camera/camera_info_nominal` as criteria for results of internal calibration.

# Internal calibration of RGB camera
After executing `sensors.launch`, 
`$ rosrun autoware_camera_lidar_calibrator cameracalibrator.py --square 0.25 --size 5x4 image:=/rgb_camera/image_raw`.

Then you rotate and move the checkerboard in gazebo simulation.
As default, 40 valid images are necessary to execute internal calibration.
All the images should be different each other for its parameter such as position, size or skew.
For efficient capturing, you set a position of the board at first and rotate it.
And after you get the several images on the position, move the board and get new images.

Now, you can push a button of "CALIBRATE" which is colored by blue.
Results is saved by pushing a button of "SAVE".
Try to compare the results and '/rgb_camera/camera_info_nominal'.


<img src=doc/interalcalib.png width=50%>


# External calibration between an RGB camera and a LiDAR

<img src=doc/gazebo.png width=50%>
<img src=doc/rviz.png width=50%>
