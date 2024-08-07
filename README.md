# Giskardpy_ros
A motion control framework for ROS implemented using [Giskardpy](https://github.com/SemRoCo/giskardpy/tree/giskard_library).

## Installation instructions for Ubuntu 24.04 + Jazzy

### ROS Workspace
First, install [Giskardpy](https://github.com/SemRoCo/giskardpy/tree/giskard_library) and switch to your virtual env.
```
workon giskardpy
```
Setup workspace
```
source /opt/ros/$ROS_DISTRO/setup.bash
mkdir -p ~/giskard_ws/src && cd ~/giskard_ws
colcon build
cd src
git clone https://github.com/SemRoCo/giskardpy_ros.git -b ros2-jazzy-main
cd ..
vcs import src < src/giskardpy_ros/$ROS_DISTRO.repos 
rosdep update --rosdistro=$ROS_DISTRO
sudo apt-get update
rosdep install --from-paths ./ -i -y --rosdistro ${ROS_DISTRO}
colcon build
source ~/giskard_ws/install/setup.bash
```
### Tutorials
https://github.com/SemRoCo/giskardpy/wiki

### How to cite
Stelter, Simon, Georg Bartels, and Michael Beetz. "An open-source motion planning framework for mobile manipulators using constraint-based task space control with linear MPC." 2022 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). IEEE, 2022.

