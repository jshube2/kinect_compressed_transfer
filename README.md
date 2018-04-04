This ros package allows you to stream RGBD data from a remote sensor with improved framerates and bandwith consumption.

It subscribes to the rgb, depth, and camera info topics specified in kinect_synchronized_transfer.launch
and transfers them to new local and synchronized topics ideal for use by RGBDSLAM or other time-sensitive ros packages.

During my testing, using this package improved framerates 5x and reduced bandwidth consumption by 10x
when used with a kinect RGBD sensor outputting at QVGA resolution.

To install:

```
cd ~/catkin_ws/src
git clone https://github.com/jshube2/kinect_compressed_transfer.git
cd ~/catkin_ws
catkin_make
```
Then to run, first make sure your RGBD image topics are being published.

Then:

```
cd ~/catkin_ws
source devel/setup.bash
roslaunch kinect_compressed_transfer kinect_synchronized_transfer.launch
```

Enjoy!
