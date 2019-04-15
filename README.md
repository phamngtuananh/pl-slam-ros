# PL-SLAM ROS Wrapper (modified)
- Some bad compilation and run time bugs are adjusted
- Merge all necessary libraries (StVO-PL and PL-SLAM)
- Added ROS publish and visualization

## Requirements
Recommended:
- ROS Kinetic
- Eigen 3.3.4
- G2O branch `20160424_git`
- OpenCV 3.3.1 (installed as `ros-kinetic-opencv3`)

## How to build
```
# Clone and build
cd
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone https://github.com/phamngtuananh
```

Before continuing, go to dependencies/pl-slam/src/slamConfig.cpp, line 70-71, and adjust the paths to your installation path.

```
chmod +x build_dependencies.sh
./build_dependencies.sh
cd ..
catkin_make

# Run
source devel/setup.bash
rosrun pl_slam_ros plslam
```

Below is the README from the original author

# PL-SLAM ROS Wrapper #

This code contains a simple ROS wrapper for it's use along with our stereo visual SLAM by using both point and line segment features project:

[https://github.com/rubengooj/pl-slam](https://github.com/rubengooj/pl-slam)

Notice that for representation purposes we are using our MRPT-based visualizer, but it can be used with any other representation class by modifying the project.

### Usage:

./plslam  <params_file>  <config_file>
   <params_file> - file with the camera configuration
   <config_file> - file with the VO configuration (optional)


### PerceptIn Sensor

We also provide a modified version of the following repository: 

[https://github.com/Shuailing-Li/PerceptIn_ROS](https://github.com/Shuailing-Li/PerceptIn_ROS) 

to stream the PerceptIn stereo sensor. The repository can be found in:

[https://github.com/rubengooj/PerceptIn_ROS](https://github.com/rubengooj/PerceptIn_ROS)  
