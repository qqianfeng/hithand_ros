# Hithand_ros
This repo aims to provide hithand catkin packages for running it in the gazebo simulation.

## Introduction
This repo runs the gazebo simulation with robot model of Hithand.  

![Alt text](docs/HIT_Hand_II_444x600.jpg?raw=true "Title")
## Directory layout

    .
    ├── hithand_description                   
    ├── ├── meshes                        # 3d models of dae, stl
    ├── ├── urdf                          # xacro, urdf and sdf files
    ├── ├── launch 
    ├── hithand_gazebo 
    ├── ├── launch 
    ├── ├── worlds
    ├── hithand_control  
    ├── ├── launch 
    ├── ├── config
    ├── Models                    # robot model file for inserting in gazebo
    ├── Experiments               #  folder for running experiment in Neurorobotic platform
    └── README.md
    

Contact:
qian.feng@tum.de

## To Run
```
cp -r hithand_description hithand_control hithand_gazebo /path/to/catkin_ws/src 
cd /path/to/catkin_ws 
catkin_make 
```
