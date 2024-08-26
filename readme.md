<div id="top"></div>

<br />

## Install the repo:

The installation assumes that the system has ROS2 Humble installed and sourced.

First use CycloneDDS as a middleware 

```sh
sudo apt install ros-humble-rmw-cyclonedds-cpp
```
It is highly recommended to add the following line to your bashrc file
```sh
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp 
```
else, run the command at every new terminal.

Install dependencies:

```sh
rosdep install -r --from-paths . --ignore-src --rosdistro humble -y
```

Build the repo:
```sh
colcon build --event-handlers desktop_notification- status- --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 2
```
Note that building some of the nodes is a highly computation intensive thus if the build process fails change parallel-workers to 1.
<br />

##  Running the Simulation

main file to run the simulation: (Terminal 1)
```sh
ros2 launch  spot_arm_gazebo main.launch.py show_detection:='true' close_rviz:='false'
```
Running the task node:(Terminal 2)

```sh
ros2 run move_arm cns_task
```
Executing the commands:(Terminal 3)

```sh
 ros2 topic pub /test_cmd std_msgs/msg/String "{data: "3"}" -1
 ```
 The commands should be run in accordance to the menu printout in the task node (sending "0" will print the menu).


## Additional options:
 Setting show_detection:='false' will disable the detection results output which can be run
 seperatly in an additional consol via the command
```sh
ros2 run image_tools showimage --ros-args --remap /image:=/detection_results
```
Activating and deactivating the suction cups
```sh
ros2 service call /custom_switch3 std_srvs/srv/SetBool data:\ true\
```
set true to activate and false to deactivate (/custom_switch1>4).

Link attacher:
```sh
ros2 service call /ATTACHLINK linkattacher_msgs/srv/AttachLink "{model1_name: 'spot', link1_name: 'left_finger_link', model2_name: 'Chip', link2_name: 'link_7'}"
```
Link detacher:
```sh
ros2 service call /DETACHLINK linkattacher_msgs/srv/DetachLink "{model1_name: 'spot', link1_name: 'left_finger_link', model2_name: 'Chip', link2_name: 'link_7'}"
```