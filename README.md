# PX4-Gazebo

1. Build plugin [realsense_gazebo_plugin](https://github.com/pal-robotics/realsense_gazebo_plugin)
2. Copy library *librealsense_gazebo_plugin.so* from folder *devel/lib* to */opt/ros/version/lib*
3. Copy package d435_camera to Tools/sitl_gazebo/models into foler [PX4](https://github.com/PX4/PX4-Autopilot)
4. Add to file iris.sdf
```
<include>
    <uri>model://d435_camera</uri>
    <pose>0 0 -0.04 0 1.5707 0</pose>
</include>
<joint name="d435_camera_joint" type="fixed">
    <child>realsense2_camera::link</child>
    <parent>iris::base_link</parent>
</joint>
```
- pose is *x y z roll pitch yaw*
- realsense2_camera is name of camera in file *my_sdf.sdf* line 3:
```
<model name='realsense2_camera'
```
- Folder contain file report position format ***.txt***, please create directory the same link in file subscriber.cpp.**
```
outfile0.open("/home/nam97/data_file/gps.txt");
```
- write to file: x, y, z , minute, seconds
```
outfile0 << var_gps_pose.pose.position.x << "\t" << var_gps_pose.pose.position.y << "\t" << var_gps_pose.pose.position.z << '\t' << ltm->tm_min << " : " << ltm->tm_sec << endl;
```
##### Add realsensen camera with generate file .sdf
1. Dowload [realsense_ros](https://github.com/IntelRealSense/realsense-ros)
2. Copy ```_d435.gazebo.xacro``` to ```realsense-ros/realsense2_description/urdf/d435.urdf.xacro```
3. Edit file ```d435.urdf.xacro```
- replace
```
<xacro:include filename="$(find realsense2_description)/urdf/_usb_plug.urdf.xacro" />
<xacro:include filename="$(find realsense2_description)/urdf/_d435.gazebo.xacro" />
```
```
<xacro:macro name="sensor_d435" params="parent *origin name:=camera use_nominal_extrinsics:=true add_plug:=false topics_ns:=camera publish_pointcloud:=true" >
```
- add gazebo plugin before
```
   </xacro:macro>
</robot>
```
```
<xacro:gazebo_d435 camera_name="${name}" reference_link="${name}_link" topics_ns="${topics_ns}" depth_optical_frame="${name}_depth_optical_frame" color_optical_frame="${name}_color_optical_frame" infrared1_optical_frame="${name}_left_ir_optical_frame" infrared2_optical_frame="${name}_right_ir_optical_frame" publish_pointcloud="${publish_pointcloud}"/>
```
file ```test_d435_camera.urdf.xacro```
```
<xacro:arg name="use_nominal_extrinsics" default="false"/>
to default="true"
```
5. Run cmd
```
cd realsensen_ros
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
rosrun xacro xacro realsense2_description/urdf/test_d435_camera.urdf.xacro > file_name.urdf
sync
gz sdf -p ./file_name.urdf > my_sdf.sdf
sync
```

# Tool

1. Generate ArUco marker.

- In folder _tool/generate_ run command: ```chmod +x ./auto.sh``` at the first run
- Run command: ```./auto.sh```
- Run file executed: ```out``` and enter parameter as instructions on the terminal.

2. Generate report.
- Not ready.

# Add new marker Aruco to gazebo

1. Copy image generated above step into folder ```Tool/sitl_gazebo/models/aruco_visual_marker_4/textures```
2. Modify ```scripts/aruco_visual_marker_4_marker.material``` with file name of image.
```
texture_unit
{
	texture aruco_mark_23.png
}
```
