# ME5413_Final_Project

NUS ME5413 Autonomous Mobile Robotics Final Project
> Authors: [Christina Lee](https://github.com/ldaowen), [Dongen Li](https://github.com/nuslde), [Yuhang Han](https://github.com/yuhang1008), and [Shuo Sun](https://github.com/SS47816)

![Ubuntu 20.04](https://img.shields.io/badge/OS-Ubuntu_20.04-informational?style=flat&logo=ubuntu&logoColor=white&color=2bbc8a)
![ROS Noetic](https://img.shields.io/badge/Tools-ROS_Noetic-informational?style=flat&logo=ROS&logoColor=white&color=2bbc8a)
![C++](https://img.shields.io/badge/Code-C++-informational?style=flat&logo=c%2B%2B&logoColor=white&color=2bbc8a)
![Python](https://img.shields.io/badge/Code-Python-informational?style=flat&logo=Python&logoColor=white&color=2bbc8a)

![cover_image](media/cover.gif)

## Dependencies

* System Requirements:
  * Ubuntu 20.04 (18.04 not yet tested)
  * ROS Noetic (Melodic not yet tested)
  * C++11 above
  * CMake: 3.0.2 above
* This repo is self-contained, only depending on standard ROS pkgs:
  * `jsk_recognition_msgs`
  * `visualization_msgs`
  * `geometry_msgs`
  * `sensor_msgs`
  * `nav_msgs`
  * `std_msgs`
  * `roscpp`
  * `rospy`
  * `tf2_ros`
  * `tf2_eigen`
  * `tf2_geometry_msgs`
  * `tf`
  * `joy`
  * `cv_bridge`
  * `image_transport`
  * `rosbridge_server`

## Installation

This repo is a ros workspace, containing two rospkgs:

* `interactive_tools`
* `me5413_world`

**Note:** If you are working on this project, it is encouraged to fork this repository and work on your own fork instead!

```bash
# clone the repo to somewhere (assuming home here `~/`)
cd
git clone https://github.com/NUS-Advanced-Robotics-Centre/ME5413_Final_Project.git
cd ME5413_Final_Project

# install dependencies
rosdep install --from-paths src --ignore-src -r -y

# build
catkin_make
# source 
source devel/setup.bash
```

## Usage

```bash
roslaunch 
```

## Contribution

You are welcome contributing to this repo by opening a pull-request

We are following:

* [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html),
* [C++ Core Guidelines](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#main),
* [ROS C++ Style Guide](http://wiki.ros.org/CppStyleGuide)

## License

The `ME5413_Final_Project` is released under the [MIT License](https://github.com/NUS-Advanced-Robotics-Centre/ME5413_Final_Project/blob/main/LICENSE)