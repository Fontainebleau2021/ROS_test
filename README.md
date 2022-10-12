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

pcl_ros pcl_conversions #ROS and PCL

sensor_msgs
>Commands
```
catkin_create_pkg pcl_test roscpp rospy std_msgs pcl_ros pcl_conversions sensor_msgs
```
