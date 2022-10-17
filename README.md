# ROS_test
> Latest update  --  2022.10.12


## Create Test Package
>Dependent

roscpp rospy std_msg 
>Commands
```
catkin_create_pkg test_pkg roscpp rospy std_msgs
```
## PCL Test
>Dependent

roscpp rospy std_msg #basic

pcl_ros pcl_conversions #ROS和PCL的接口

sensor_msgs #传感器消息
>Commands
```
catkin_create_pkg pcl_test roscpp rospy std_msgs pcl_ros pcl_conversions sensor_msgs
```
然后在package.xml文件中添加：
```
  <build_depend>libpcl-all-dev</build_depend>
  <exec_depend>libpcl-all</exec_depend>
```
在完成源代码编辑后，记得要对CMakeList.txt文件进行修改：
```
add_executable(pcl_test src/main.cpp)
target_link_libraries(pcl_test ${catkin_LIBRARIES})

```

## imu_viz_2d
-[imu_viz_2d](https://zhuanlan.zhihu.com/p/143769628)
>Dependent

roscpp std_msgs #basic

visualization_msgs #可视化

>Commands
```
catkin_create_pkg imu_viz_2d roscpp visualization_msgs std_msgs
```
对CMakeList.txt文件进行修改
```
add_executable(imu_viz_2d src/imu.cpp)
target_link_libraries(imu_viz_2d ${catkin_LIBRARIES})

add_executable(imu_frame src/frame.cpp)
target_link_libraries(imu_frame ${catkin_LIBRARIES})
```