# natnet_client_ros
Natnet client for publishing Optitrack pose into ROS.  
This is the ROS wrapped package for https://github.com/ShigiDono/natnetclient.git  
This package connect to the Optitrack server and publishes pose information as ROS messages.

# Install
Clone this repo to your catkin_ws and build it.
```bash
cd ~/catkin_ws/src
git clone https://github.com/mktk1117/natnet_client_ros.git
cd natnet_catkin_ros && git submodule update --init --recursive
cd ~/catkin_ws
catkin build natnet_client_ros
```

# Settings
In the config folder, you should make a config.yaml
It would be like this.
```yaml
rigid_bodies:
    'marker1':
        pose: marker1/pose
        child_frame_id: marker1/base_link
        parent_frame_id: world

client_ip: 192.168.1.48
server_ip: 192.168.1.35
data_port: 1511
comm_port: 1510
read_rate: 1200
publish_rate: 100
```
The `rigid_bodies` are the name of the rigid_bodies that are defined in the Optitrack software.  
`pose` is the name of the topic where the pose message is published. 
`child_frame_id` and `parent_frame_id` is the name of the frames used in tf messages.  
`client_ip` is the ip address of your computer and `server_ip` is the ip address of the computer where the Optitrack software is running.
`data_port`, `comm_port`, and `read_rate` are the settings used in the Optitrack software.
`publish_rate` defines the publishing rate of the pose and tf messages.

# Usage
First, modify the config.yaml.
Then, launch the node with launch file.
```bash
roslaunch natnet_client_ros natnet_client.launch
```
