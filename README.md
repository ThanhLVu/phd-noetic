phd

**** Note on launching bounding box real-life ROS-Unity setup ****

1. Edit the ROS_IP in working/param directory
2. Launch the cameras nodes:  roslaunch realsense2_camera rs_camera.launch
3. Launch camera nodes:       rosrun working combine_cameras_real
4. Launch ROS-TCP-Endpoint:   roslaunch working endpoint.launch 

-- Side note: ROS-TCP-Endpoint package version cloned on 1-9-2021, earlier version might requires changes to endpoint.launch

