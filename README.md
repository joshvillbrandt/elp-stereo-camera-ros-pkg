# elp-stereo-camera-ros-pkg

A ROS driver for the ELP Dual Lens stereo camera.

## Documentation

Documentation is available on the ROS wiki: http://wiki.ros.org/elp_stereo_camera

## Install from Source

These setup instructions assume that you have Ubuntu 14.04, have ROS Indigo or ROS Jade installed, and have a catkin workspace at `~/catkin_ws`. If you don't, follow the [Installing and Configuring Your ROS Environment](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment) tutorial before proceeding. The following specific packages should be installed if they aren't already:

```bash
sudo apt-get update
sudo apt-get install ros-jade-ros-base ros-jade-image-common ros-jade-image-transport-plugins ros-jade-image-pipeline ros-jade-usb-cam
```

Now you can install the elp_stereo_camera package:

```bash
git clone https://github.com/joshvillbrandt/elp-stereo-camera-ros-pkg.git  ~/catkin_ws/src/elp_stereo_camera

# setup udev rules
sudo cp ~/catkin_ws/src/elp_stereo_camera/debian/99-elp-stereo-camera.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo service udev restart && sudo udevadm trigger
```

## Change History

This project uses [semantic versioning](http://semver.org/).

### v1.0.0 - TBD

* Renamed package
* Moved documentation to the ROS wiki
* Added package to ROS apt repository

### v0.2.0 - 2016/01/29

* Added calibration files
* Added udev rules for consistent left/right camera acquisition
* Now using `usb_cam` and a [ELP-1MP2CAM001](http://www.amazon.com/gp/product/B00VG32EC2) dual-lens camera

### v0.1.0 - 2016/01/27

* Initial release
* Stereo camera working with `gscam` and two [ELP-USBFHD01M](http://www.amazon.com/dp/B00KA7WSSU) cameras
