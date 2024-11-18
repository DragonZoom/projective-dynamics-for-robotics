# Projective Dynamics for Robotics

**This version of the project is in early development: only a simple visualization of the robot model is available**

This project implements a simulation of projective dynamics for robotic applications. It provides a ROS2 package named pd_sim that simulates deformable objects, such as tires, using projective dynamics and integrates it with Robot Operating System (ROS) software stack - ROS 2.

## Overview
The pd_sim package uses projective dynamics to simulate deformable bodies in real-time. It utilizes OpenGL and GLFW for rendering, GLEW for OpenGL extension handling, and TinyGLTF for loading GLTF models.

## Installation
### With Docker

To build and run the project using Docker, follow these steps:

1. **Build the Docker Image**

From the root of the repository build the Docker image:
```
docker build -t projective-dynamics-for-robotics .
```

2. **Run the Docker Container**

Run the container and start the simulation:
```
docker run -it --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --env="MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --gpus all projective-dynamics-for-robotics
```
At this point project is ready to launch. Don't forget `source install/setup.bash`. \
_TODO: there is no video card visible inside the docker, rendering takes place via LLVM pipe_

### Prerequisites
* [ROS 2](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) (tested with ROS2 jazzy)
* OpenGL
* GLFW
* GLEW
* GLM

### Build
Clone the repository:
```
git clone https://github.com/DragonZoom/projective-dynamics-for-robotics
cd projective-dynamics-for-robotics
```

Install missing dependencies:
```
rosdep install -i --from-path src --rosdistro jazzy -y
```

Build the package (`--cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON` to generate compile commands for Clangd):
```
colcon build --symlink-install --parallel-workers 4 --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

ROS validation:
```
ros2 doctor --report
```

## Usage
### Running the Simulator
Source the setup script
* linux:
```
source install/setup.bash
```
* Windows:
```
.\install\setup.ps1
```
Run the simulation node:
```
ros2 launch pd_sim pd_sim_launch.py robot_model:=models/robot_simple.gltf
```

### Parameters
* `robot_model`: Path to the GLTF file representing the robot model

## Mesh Format
The simulator uses the GLTF format for 3D models. Custom attributes are used to define simulation properties:

* `__IS_DEFORMABLE`: Indicates that the mesh is deformable and requires simulation.
* `__IS_ROTATING_SOURCE`: Marks the parts of the model where motor rotation forces are applied.


## Additional Notes
* Build only one package
```
colcon build --packages-select my_package
```

* Create own packege
```
ros2 pkg create --build-type ament_cmake --license Apache-2.0 --node-name my_node my_package
```

* Simulation run with WSL (enable NVIDIA)
```
MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA gz sim
```

## References
* [Projective dynamics: fusing constraint projections for fast simulation](https://dl.acm.org/doi/10.1145/2601097.2601116)
* ROS 2 [tutorials](https://docs.ros.org/en/jazzy/Tutorials.html)


## License
This project is licensed under the Apache License 2.0. See the `LICENSE` file for details.
