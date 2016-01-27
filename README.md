# voxel

A ROS package for the Voxel stereo camera.

## Setup

TODO: At these moment, this is more a scratch pad for the various drivers that I have been trying. In the future, down-select to just one driver.

usb_cam

```bash
sudo apt-get install ros-jade-usb-cam
```

libuvc_camera

```bash
clone libuvc
clone libuvc_ros
```

uvc_camera:

```bash
git clone https://github.com/ktossell/camera_umd ~/camera_umd
ln -s ~/camera_umd/uvc_camera ~/catkin_ws/src/uvc_camera
rosdep install uvc_camera
# password and y's
cd ~/catkin_ws
catkin_make
```

gscam:

```bash
git clone https://github.com/ros-drivers/gscam ~/catkin_ws/src/gscam
rosdep install gscam
# password and y's
cd ~/catkin_ws
catkin_make
```

this package:

```bash
git clone https://github.com/bluerobotics/voxel.git ~/catkin_ws/src/voxel
# rosdep install voxel
# password and y's
cd ~/catkin_ws
catkin_make
```

## Calibration

The calibration process removes distortion from the individual cameras. This process is essential for producing a quality disparity image.

To calibrate the cameras, first print out the [8x6 checkerboard](extra/checkerboard-8x6.pdf) on an 8.5x11 inch piece of paper. Attach the checkerboard printout to something firm like a clipboard. Then run the following:

```bash
roslaunch voxel calibrate.launch
```

The calibration process measures a range of four key attributes: X, Y, size, and skew. Hold the checkerboard up in front of the cameras and vary each of the parameters. (So move the checkboard left and right, up and down, father and closer, and at different angles.) Make sure that the entire calibration grid stays within view of both cameras. You'll know the calibration process is working when you can see a colored grid in each camera view and when the size of attribute bars are increasing.

![Calibration Screenshot](extra/calibration-screenshot.jpg)

When the calibration process has gathered enough information about a particular attribute, the cooresponding bar will turn green. With practice, the calibration process should take about one minute to complete. Hit the "calibrate" button once all four bars are green. After a few moments, new rectified images should be displayed. Hit the "commit" button to complete the process.

For more information, check out the [ROS Stereo Calibration Tutorial](http://wiki.ros.org/camera_calibration/Tutorials/StereoCalibration) as well as the [OpenCV Camera Calibration](http://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html) documentation.

## Usage

To start the cameras:

```bash
roslaunch voxel voxel.launch
```

You now have available topics such as `stereo/left/image_rect_color`, `stereo/left/image_rect_color`, and `/stereo/points`. To quickly view a camera image and the resulting point cloud, try out the included rviz configuration:

```bash
roslaunch voxel rviz.launch
```

## Change History

This project uses [semantic versioning](http://semver.org/).

## v0.1.0 - 2016/02/tbd

* Initial release.
