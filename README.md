# Mobile-Manipulator-Simulator
(Forked from Compliant Control and Application)
The repository integrates the mobile robot Mir_robot and the UR5e manipulator, providing both velocity and torque control interface for the manipulator, and velocity interface for the mobile car.

## Compile
```bash
mkdir catkin_ws/src && cd catkin_ws/src
```
```bash
git clone 
```
```bash
sudo chmod +x install_dependencies.sh && ./install_dependencies.sh
```
```bash
catkin build (or cd .. && catkin_make)
```
If there is come up with some errors during compile process, you might need to install some ros packages for support this project, I recommend you to run following command or search the compile error in Google or Baidu and so on.
```bash
rosdep install -i --from-path src --rosdistro noetic --ignore-src -r -y
```
## Check

using the following command to check the self-defined controller. Like:`cartesian_velocity_controller`

```bash
rospack plugins --attrib=plugin controller_interface
```

## Run

### 1. Impedance control under wall environment
To launch gazebo Environment:
```bash
roslaunch mir_gazebo mm_torque_control.launch
```
for cases of free world with no wall:
```bash
roslaunch mir_gazebo mm_torque_control_emplyworld.launch
```

To launch the UDE-based dynamic motion/force control: 
```bash
roslaunch impedance Impedance.launch
```

### 2. Hybrid Admittance Control under wall environment
```bash
roslaunch mir_gazebo mm_vel_control.launch
```
```
roslaunch admittance vel_HybridAdmittance.launch
```