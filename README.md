# PX4-Gazebo

1. Copy plugin [realsense_gazebo_plugin](https://github.com/pal-robotics/realsense_gazebo_plugin) to src folder.
2. Copy package d435_camera to Tools/sitl_gazebo/models into foler [PX4](https://github.com/PX4/PX4-Autopilot).
3. Add to file iris.sdf.
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
- pose is **x y z roll pitch yaw**  
- realsense2_camera is name of camera in file my_sdf.sdf line 3:
```<model name='realsense2_camera'```
**- Folder contain file report position format ***.txt***, please create directory the same link in file subscriber.cpp.**
```
outfile0.open("/home/nam97/data_file/gps.txt");
```
- write to file: x, y, z , minute, seconds
```
outfile0 << var_gps_pose.pose.position.x << "\t" << var_gps_pose.pose.position.y << "\t" << var_gps_pose.pose.position.z << '\t' << ltm->tm_min << " : " << ltm->tm_sec << endl;
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