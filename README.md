# voxel-ros-pkg

A ROS package for the Blue Robotics [Voxel Stereo Camera](https://www.bluerobotics.com/store/electronics/voxel-stereo-camera/).

## Setup

These setup instructions assume that you have Ubuntu 14.04, have ROS Jade or ROS Indigo installed, and have a catkin workspace at `~/catkin_ws`. If you don't, follow the [Installing and Configuring Your ROS Environment](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment) tutorial before proceeding. The following specific packages should be installed if they aren't already:

```bash
sudo apt-get update
sudo apt-get install ros-jade-ros-base ros-jade-image-common ros-jade-image-transport-plugins ros-jade-image-pipeline ros-jade-usb-cam
```

Now you can install the voxel package:

```bash
git clone https://github.com/bluerobotics/voxel-ros-pkg.git ~/catkin_ws/src/voxel

# setup udev rules
sudo cp ~/catkin_ws/src/voxel/extra/99-voxel-camera.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo service udev restart && sudo udevadm trigger
```

## Usage

To start the cameras:

```bash
roslaunch voxel voxel.launch
```

You now have available topics such as `stereo/left/image_rect_color`, `stereo/left/image_rect_color`, and `/stereo/points`. You'll likely also want to include the `voxel_camera` macro in your urdf file. Look at the [voxel_standalone.urdf.xacro](description/voxel_standalone.urdf.xacro) file for an example of how to do that.

To quickly view a camera image and the resulting point cloud, try out the included rviz configuration. Click the "Point Cloud" saved camera view in the Views pane on the right. You should now be able to your objects in 3D! Keep in mind that objects closer than 70 cm won't show up in the point cloud with this camera arrangement.

```bash
roslaunch voxel rviz.launch
```

![Rviz Screenshot](extra/rviz-screenshot.jpg)

## Calibration

The calibration process removes distortion from the individual cameras and surrounding enclosure. This process is essential for producing a quality disparity image. While this package includes generic calibration files for the voxel camera, performing a calibration for each unique voxel camera can improve the quality of the resulting point could.

To calibrate the cameras, first print out the [8x6 checkerboard](extra/checkerboard-8x6.pdf) on an 8.5x11 inch piece of paper. Attach the checkerboard printout to something firm like a clipboard. (Add a bit of tape to the bottom two corners to keep the page flat.) Then run the following:

```bash
roslaunch voxel calibrate.launch
```

The calibration process measures a range of four key attributes: X, Y, size, and skew. Hold the checkerboard up in front of the cameras and vary each of the parameters. (So move the checkerboard left and right, up and down, father and closer, and at different angles.) Make sure that the entire calibration grid stays within view of both cameras. You'll know the calibration process is working when you can see a colored grid in each camera view and when the size of attribute bars are increasing.

![Calibration Screenshot](extra/calibration-screenshot.jpg)

When the calibration process has gathered enough information about a particular attribute, the corresponding bar will turn green. With practice, the calibration process should take about one minute to complete. Hit the "calibrate" button once all four bars are green. After a few moments, new rectified images should be displayed. Real-world straight edges should now appear straight in the video feed.

Hit the "commit" button to complete the process. This saves the calibration information locally to `~/.ros/camera_info/`. Check out [voxel.launch](launch/voxel.launch) for an example of how to specify the calibration files.

For more information, check out the [ROS Stereo Calibration Tutorial](http://wiki.ros.org/camera_calibration/Tutorials/StereoCalibration) as well as the [OpenCV Camera Calibration](http://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html) documentation.

## Video Modes

The ELP module used in the voxel camera has these video modes:



## Tips and Tricks

Some helpful debugging commands:

* Quick check of available cameras: `ls /dev | grep video`
* Quick check of the support video modes of a camera: `v4l2-ctl --list-formats-ext -d /dev/video0`
* Camera viewer with extensive camera configuration: `guvcview` or `guvcview -d /dev/video0`
* View udev attributes to create different udev rules: `udevadm info -a -n /dev/video0` (also see [this syntax guide](http://www.reactivated.net/writing_udev_rules.html#syntax))

## Change History

This project uses [semantic versioning](http://semver.org/).

## v0.2.0 - TBD

* Added calibration files
* Added udev rules for consistent left/right camera acquisition
* Now using [ELP-1MP2CAM001](http://www.amazon.com/gp/product/B00VG32EC2?psc=1&redirect=true&ref_=ox_sc_sfl_title_1&smid=A1X8N7RHCK4F35) dual-lens camera

## v0.1.0 - 2016/01/27

* Initial release
* Stereo camera working with `gscam` and two [ELP-USBFHD01M](http://www.amazon.com/ELP-Driver-Camera-Module-ELP-USBFHD01M-L21/dp/B00KA7WSSU/ref=pd_sim_147_2?ie=UTF8&dpID=41HNP%2BZXJuL&dpSrc=sims&preST=_AC_UL160_SR160%2C160_&refRID=0K7CKWSDSNFEWPV613WY) cameras
