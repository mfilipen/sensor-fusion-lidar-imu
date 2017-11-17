# Sesor fusion

This repository contains the following packages 
 * racecar_description - it is for rendering the dimensional model of the car in RViz
 * razor_imu - it is for visualizing data from [9 Degrees of Freedom - Razor IMU](https://www.sparkfun.com/products/retired/10736)
 * sensor_fusion - it is for fusing odometry data and data IMU data.

Main idea.  Package [hector_mapping](http://wiki.ros.org/hector_mapping) with information from [2D lidar](http://wiki.ros.org/hokuyo_node) provides a two-dimensional map and robot pose (x, y, yaw). When the position of the monitored object changes along the z axis, the algorithm loses localization. The idea of using additional information from other sensors seems interesting. First, data from lidar about orientation (x,y,yaw) and data from magnetrometr (yaw).

## Test enviroment
Test environment was built for collecting data. All blocks have a standard length of 0.3m. The main contour has a rectangular shape. In this way, we can calculate the positions of the robot from the on-board camera.
[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/rrM2ilHYsvM/0.jpg)](https://www.youtube.com/watch?v=rrM2ilHYsvM)

## Datasets
[Google drive link to datasets](https://drive.google.com/drive/folders/1Y8_DFJUcuB-nSq_NuDmEkCzxAicApLjp?usp=sharing) - download it to repository folder.
##### 2017-11-12-20-18-43.bag - dataset_1 for sensor fusion. LIDAR + IMU(default calibration).
[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/lVSDDgokX3g/0.jpg)](https://www.youtube.com/watch?v=lVSDDgokX3g)
Raw data about yaw from lidar(blue) and from IMU(red). There are radians along the y axis and time in seconds along x-axis.
![graph 1](http://filipenko.biz/wp-content/uploads/2017/11/figure_1.png)

##### 2017-11-14-21-53-47.bag - dataset_2 for sensor fusion. LIDAR + IMU(calibrated).
[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/PBC5dq39bkE/0.jpg)](https://www.youtube.com/watch?v=PBC5dq39bkE)
Raw data about yaw from lidar(blue) and from IMU(red). There are in radians along the y axis and time in seconds along x-axis .
![graph 2](http://filipenko.biz/wp-content/uploads/2017/11/figure_2-1.png)

## 9DOF Razor IMU calibration
I found that default calibration does not provide enough precise information (we can see it from first dataset). It was nesesary to do calibration before fusing data. The procedure of calibration was done how it was described [here](http://wiki.ros.org/razor_imu_9dof). I had a lot of problems with installation of programs for calibration on Jetson TX1 (which is used as the basis for the robot). It is was found that there is problem with running Oracle Java (which is requred for run calibration software) on Jetson TX1 (last version which I tryed - [Linux ARM 64 Hard Float ABI](http://www.oracle.com/technetwork/java/javase/downloads/jdk8-downloads-2133151.html)). Finally, calibration was done using another computer.
[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/6SqMOvdJ9xU/0.jpg)](https://www.youtube.com/watch?v=6SqMOvdJ9xU)

## Runnig datasets
[ROS Kinetic Kame](http://wiki.ros.org/kinetic) is used for collecting and run datasets.
#### Console 1
Run ROS master
```bash
roscore
```

#### Console 2
Run bag file with dataset.
```bash
rosbag play 2017-11-12-20-18-43.bag
or
rosbag play 2017-11-14-21-53-47.bag
```
#### Console 3
Run Rviz for visualization.
```bash
rosrun rviz rviz
```

#### Console 4
Download model for vizualization.
```bash
roslaunch racecar_description racecar_model.launch
```
#### Console 5
Download vizualization for IMU and for car orientation.
```bash
roslaunch razor_imu razor-display.launch
```
## Data for fusion
### IMU data - [semsor_msgs/imu](http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html)
```bash
topic: /imu
data: semsor_msgs/imu
```
Compact Message Definition
```bash
std_msgs/Header header
geometry_msgs/Quaternion orientation
float64[9] orientation_covariance # Row major about x, y, z axes

geometry_msgs/Vector3 angular_velocity
float64[9] angular_velocity_covariance # Row major about x, y, z axes

geometry_msgs/Vector3 linear_acceleration
float64[9] linear_acceleration_covariance # Row major x, y z 
```

### Hector_SLAM  - [geometry_msgs/PoseWithCovarianceStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html)

```bash
topic: /poseupdate
data: geometry_msgs/PoseWithCovarianceStamped
```

Compact Message Definition - geometry_msgs/PoseWithCovarianceStamped
```bash
std_msgs/Header header
geometry_msgs/PoseWithCovariance pose
```
Compact Message Definition - geometry_msgs/PoseWithCovariance
```bash
# This represents a pose in free space with uncertainty.
Pose pose
# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance
```
### /poseupdate data
|![pose_x_y](http://filipenko.biz/wp-content/uploads/2017/11/pose_x_y.png) (x,y) - trajectory of the robot| |
|---|---|
|![pose_x_t](http://filipenko.biz/wp-content/uploads/2017/11/pose_x_t.png) x(t) - raw data|   ![pose_y_t](http://filipenko.biz/wp-content/uploads/2017/11/pose_y_t.png) y(t) - raw data| 
|![pose_vx_t](http://filipenko.biz/wp-content/uploads/2017/11/pose_vx_t.png)  dx(t) - numerical first derivative| ![pose_vy_t](http://filipenko.biz/wp-content/uploads/2017/11/pose_vy_t.png)  dy(t) - numerical first derivative|  
|![pose_ax_t](http://filipenko.biz/wp-content/uploads/2017/11/pose_ax_t.png)  d^2 x(t) - numerical second derivative| ![pose_ay_t](http://filipenko.biz/wp-content/uploads/2017/11/pose_ay_t.png)  d^2 y(t) - numerical second derivative|  

### /IMU data
#### /acceleration data
|![acceleration_ax_t](http://filipenko.biz/wp-content/uploads/2017/11/gyro_ax_t.png) ax(t) - raw data|   ![acceleration_ay_t](http://filipenko.biz/wp-content/uploads/2017/11/gyro_ay_t.png) ay(t) - raw data| 
|---|---|
|![acceleration_vx_t](http://filipenko.biz/wp-content/uploads/2017/11/gyro_vx_t.png)  vx(t) - numerical first integral| ![acceleration_vy_t](http://filipenko.biz/wp-content/uploads/2017/11/gyro_vy_t.png)  vy(t) - numerical first integral|  
|![acceleration_ax_t](http://filipenko.biz/wp-content/uploads/2017/11/gyro_x_t.png)  x(t) - numerical second integral| ![acceleration_ay_t](http://filipenko.biz/wp-content/uploads/2017/11/gyro_y_t.png)  y(t) - numerical second integral| 
|![acceleration_ax_t](http://filipenko.biz/wp-content/uploads/2017/11/gyro_y_x.png) (x,y) - integrated data| | 
#### /gyro data
|![gyro_wz](http://filipenko.biz/wp-content/uploads/2017/11/gyro_wz.png) angular_velocity wz(t) - raw data|   ![gyro_wz_integral](http://filipenko.biz/wp-content/uploads/2017/11/gyro_wz_integral.png) yaw(t) - numerical first integral| 
|---|---|
|![gyro_wz_integral_normalization](http://filipenko.biz/wp-content/uploads/2017/11/gyro_wz_integral_normalization.png)  yaw(t) - numerical first integral normalization| |  
#### /magnetometer data
![magnetometer_yaw](http://filipenko.biz/wp-content/uploads/2017/11/magnetometer_yaw.png)
yaw(t) - raw data



