phd

**** Note on launching bounding box real-life ROS-Unity setup ****

1. Edit the ROS_IP in working/param directory
2. Launch the cameras nodes:  roslaunch realsense2_camera rs_camera.launch
3. Launch camera nodes:       rosrun working combine_cameras_real
4. Launch ROS-TCP-Endpoint:   roslaunch working endpoint.launch 

-- Side note: ROS-TCP-Endpoint package version cloned on 1-9-2021, earlier version might requires changes to endpoint.launch

**Note for future Thanh**
After a pull from this repository, remember to pull the halo_project_hub again.

git clone https://github.com/tonydle/halo_project_hub.git
cd halo_project_hub
git checkout feature/gazebo-model-with-cameras

Camera calibration
+ Hardware: QR code, 2 camera
+ Start cameras: 
    roslaunch working multiple_cameras.launch
+ Start qr tracking:
    roslaunch working qr_track_real_multi_cam.launch
+ Make sure base_link to qr_link exist
+ Start calibration node:
    rosrun working multi_camera_calibration_qr_real
+ Manually note the result into the urdf 

SSH to HALO NUC:
+ ssh command: 
    ssh tony@192.168.0.10 -X
+ Password: iceandfire
+ HALO Subnet Router Password:
    IceAndFire!@1029
    
Truck simulation
+ Truck + 3d lidar empty environment launch
    roslaunch working 3d_lidar_sim.launch

Vigir package for point cloud to mesh
+ Get the following packages: vigir_perception, vigir_perception_msgs, vigir_utilities
+ In the vigir_perception package, delete all other files except vigir_point_cloud_proc
+ Github links:
    https://github.com/team-vigir/vigir_perception
    https://github.com/team-vigir/vigir_perception_msgs
    https://github.com/team-vigir/vigir_utilities
+ In vigir_point_cloud_proc/include/vigir_point_cloud_proc/cloud_to_mesh.h
    Line 123 mls.setSearchRadius(x); where x >= the distance of your points from each other
    Line 186 greedy.setSearchRadius(y); where y >= x
    Modify x and y depends on the point cloud density
+ For NextOre project: 
    x = 0.3, y = 0.5
    These values are not fixed and can be changed depending on the point cloud
    
HALO Annotation:
+ After have the both cameras on:
    rosrun working combine_cameras_real
    
Modbus 
+ Slave Simulator (https://www.modbusdriver.com/diagslave.html)
+ Git clone modified ros-modbus-device-driver 
    git clone https://github.com/ThanhLVu/modified-ros-modbus-device-driver.git
+ To start the Modbus simulated slave
    Change directory to path/diagslave/x86_64-linux-gnu
    sudo ./diagslave -m tcp
+ To start ros and modbus connection
+ Check ~/.bashrc for IP address setting
+ Start roscore
+ Run ros-modbus-device-driver
    rosrun ros_modbus_device_driver modbus_device_driver.py _mapping:=path/to/config/file
+ Run talker C++ code (can be replace with command line rostopic pub)
    rosrun working modbus_talker	 

