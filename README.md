<!-- markdownlint-disable MD024 -->

# ME5413 Final Project Group 10

This is our implementation of the [ME5413_Final_Project](https://github.com/NUS-Advanced-Robotics-Centre/ME5413_Final_Project). For the original project description and instructions, please refer to the [ORIGINAL_README.md](ORIGINAL_README.md) file.

## Group Members

> Sorted in alphabetical order

[Cao Chenyu](https://github.com/ruziniuuuuu), [Li Zhangjin](https://github.com/Lizhangjin), Wang Yuanlong, Zhao Huaiyi, [Zhao Xu](https://github.com/AeroEmbedAutoTechJohn), [Zhu Rong](https://github.com/Celia0731)

## Introduction

This project focuses on implementing the Jackal robot to map an environment and navigate to a goal location. As a bonus objective, the robot is required to detect a dynamic object (number 3 on a box) and navigate to its location.

The project consists of four main components:

- **Mapping:** The primary mapping method used is the `Fast-Lio` package. Other mapping methods, such as `gmapping`, `Cartographer`, `A-LOAM`, and `F-LOAM`, are also tested for performance comparison.

- **Perception (Object Detection):** Two methods are used to detect the object (number 3 on the box) using the camera information:
  - Template matching method provided by the OpenCV library.
  - The `find_object_2d` package, which uses the `ORB` (Oriented FAST and Rotated BRIEF) method.
  
- **Localization:** Localization is performed using the `AMCL` (Adaptive Monte Carlo Localization) method.

- **Navigation:** The global planner used is `A*`, and the local planner used is `Teb`.

By integrating these components, the Jackal robot is capable of autonomously mapping the environment, detecting the dynamic object, and navigating to the desired goal location.

### Flowchart


### Mapping

### Object Detection

### Localizatioin

### Navigation

## Repo Structure

```plaintext

├── README.md
├── ORIGINAL_README.md
├── assets
├── src
│   ├── costmap_prohibition_layer -> Prohibit the robot from entering the area
│   ├── pcd_to_map -> Convert pcd file to map 
│   ├── FAST-LIO -> Fast-Lio mapping
│   ├── livox_ros_driver -> Livox LIDAR driver (necessary for Fast-Lio)
│   ├── interactive_tools -> Interactive tools for manipulating RViz panels
│   ├── jackal_description -> Jackal robot description
│   ├── jackal_navigation -> Jackal navigation
│   ├── me5413_world -> Main implementations of the project
│   │   ├── include -> Header files
│   │   │   ├── me5413_world -> Header files for the main implementations
│   │   ├── launch -> Launch files
│   │   │   ├── amcl.launch
│   │   │   ├── fast_lio.launch
│   │   │   ├── find_box.launch
│   │   │   ├── include
│   │   │   │   └── spawn_jackal.launch
│   │   │   ├── main.launch
│   │   │   ├── manual.launch
│   │   │   ├── mapping.launch
│   │   │   ├── move_base.launch
│   │   │   ├── navigation.launch
│   │   │   └── world.launch  
│   │   ├── src -> C++ scripts
│   │   │   ├── object_spawner_gz_plugin.cpp -> Gazebo plugin for spawning the object (Boxes)
│   │   │   ├── goal_publisher_node.cpp -> Publish the goal location (when goal is not box)
│   │   │   ├── box_explore_node.cpp -> Explore the box location (when goal is box)
│   │   │   └── template_matching.cpp -> Template matching method for object detection
│   │   └── scripts -> Python scripts  
│   │       ├── sift_detection_node_py.py -> SIFT method for object detection
│   │       └── template_matching_node_py.py -> Template matching method for object detection
```

Some other packages used in this project are not included in this repo. Please refer to the installation section for more information.

## Installation

First, clone this repository into your workspace:

```bash
cd ~
git clone https://github.com/ruziniuuuuu/ME5413_Final_Project_Group10.git
cd ME5413_Final_Project_Group10
```

Then, install all dependencies:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

Finally, build the workspace:

```bash
catkin_make
source devel/setup.bash
```

To properly load the gazebo world, you will need to have the necessary model files in the `~/.gazebo/models/` directory.

There are two sources of models needed:

- [Gazebo official models](https://github.com/osrf/gazebo_models)
  
  ```bash
  # Create the destination directory
  cd
  mkdir -p .gazebo/models

  # Clone the official gazebo models repo (assuming home here `~/`)
  git clone https://github.com/osrf/gazebo_models.git

  # Copy the models into the `~/.gazebo/models` directory
  cp -r ~/gazebo_models/* ~/.gazebo/models
  ```

- [Our customized models](https://github.com/NUS-Advanced-Robotics-Centre/ME5413_Final_Project/tree/main/src/me5413_world/models)

  ```bash
  # Copy the customized models into the `~/.gazebo/models` directory
  cp -r ~/ME5413_Final_Project/src/me5413_world/models/* ~/.gazebo/models
  ```

Other than the packages provided in the original project description, the following packages are also required:

- Teb Local Planner

    ```bash
    sudo apt-get install ros-noetic-teb-local-planner
    ```

- find_object_2d

    ```bash
    sudo apt-get install ros-noetic-find-object-2d
    ```
