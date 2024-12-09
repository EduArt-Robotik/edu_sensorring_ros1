# EduArt Sensorring Ros Wrapper
The software interface of the EduArt Sensorring is the lightweight and framework independent [edu_lib_sensorring](https://github.com/EduArt-Robotik/edu_lib_sensorring) library. This repository is a wrapper of the library to use the Sensorring measurements in Ros.

>Note: For a Ros2 wrapper see the [edu_sensorring_ros2](https://github.com/EduArt-Robotik/edu_sensorring_ros2) repository.

>Note: The Sensorring library currently only implements the Linux specific SocketCan interface for communication with the sensors. Therefore the Ros node is currently also limited to Linux systems.

# Building and running the node

The node can be run either locally on systems with a Ros environment or in a docker container that is also included in this repository.

## Running the node locally

To run the node locally you need to have Ros installed. Please refer to the official [Ros installation guide](http://wiki.ros.org/noetic/Installation).

### 1. Install the Sensorring library
Use the convenience script to build and install the Sensorring library on your system. Refer to the [edu_lib_sensorring](https://github.com/EduArt-Robotik/edu_lib_sensorring) repository for a detailed description of the build and installation process.

```
mkdir eduart_ws
cd eduart_ws
git clone -b master https://github.com/EduArt-Robotik/edu_lib_sensorring.git
cd edu_lib_sensorring
./cmake/install_release.bash
```
>Note:<br> If the bash script does not execute it may need to be converted to a file with unix style line endings. Run `dos2unix ./cmake/*` for an automatic conversion before executing the script.

### 2. Clone the Sensorring node into your Ros workspace
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_ws
git clone -b master https://github.com/EduArt-Robotik/edu_sensorring_ros1.git
```

### 3. Build the node
The node uses the standard Ros `catkin_make` toolchain.
```
cd ~/catkin_ws
catkin_make -DCATKIN_WHITELIST_PACKAGES="edu_sensorring_ros1"
source ~/catkin_ws/devel/setup.bash
```

### 4. Start the node
To start the node use the included launchfile.
```
roslaunch edu_sensorring_ros1 edu_sensorring_ros1.launch.py
```

>Note:<br> The launchfile [edu_sensorring_ros1.launch.py](launch/edu_sensorring_ros1.launch) uses the parameter set [edu_bot_sensorring_params.yaml](params/edu_bot_sensorring_params.yaml). You likely need to change the parameters to match your hardware configuration. Either create your own launchfile and parameter file or adjust the existing parameter file.


## Running the node in a container

Running the node in a container offers some advantages over natively running the node. It allows you to ...
- ... skip the manual installation of dependencies (e.g. the edu_lib_sensorring)
- ... run the node without installing Ros on your native system
- ... quickly change between different distributions of Ros
- ... autostart the node after boot

To use the container you need to install Docker. Please refer to the official [Docker installation guide](https://docs.docker.com/engine/install/).

### 1. Clone the Sensorring node into your workspace
```
mkdir ~/eduart_ws
cd ~/eduart_ws
git clone -b master https://github.com/EduArt-Robotik/edu_sensorring_ros1.git
```

### 2. Build the container
The container has to be built once after cloning the repository. During the build all dependencies are installed in the container.
```
cd ~/eduart_ws/edu_sensorring_ros1/docker
docker compose build
```

### 3. Start the container
Start the container by running `docker compose up`. The container is configured to restart automatically including after rebooting your system.
```
cd ~/eduart_ws/edu_sensorring_ros1/docker
docker compose up -d
```

You can verify that the container is running by listing all active containers. The list should include the Sensorring container.
```
docker ps
```

>Note:<br>The container uses some environment variables of the host system. Set the following variables according to your setup:<br>- EDU_ROBOT_NAMESPACE: Prefix for the Ros topics and Ros services of the node<br>- ROS_MASTER_URI: Route to Ros master (e.g. "http://192.168.178.100:11311/") <br>- ROS_IP: Own IP address (e.g. "192.168.178.100")

### 4. Stop the container
To stop the container and remove it from the autostart run `docker compose down`.
```
cd ~/eduart_ws/edu_sensorring_ros1/docker
docker compose down
```

Again you can verify that the container has stopped by listing all active containers. The Sensorring container should no longer be listed.
```
docker ps
```

# Ros interface of the node

## 1. Overview
This is a screenshot of the `rqt_graph` while the edu_sensorring_ros1 node is running. In the example the sensor board with the index 0 has a thermal sensor that publishes thermal measurements. Note that the default launchfile also includes a static transform publisher that defines a reference frame for the Sensorring measurements.

<img src="doc/images/rqt_graph.png" width="600"/>

## 2. Publisher

### /sensors/tof_sensors/pcl_raw
There is one publisher that publishes the point cloud data from all Time-of-Flight sensors of the Sensorring. The message type is a [`sensor_msgs/msg/PointCloud2`](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/PointCloud2.html). There is currently no mechanism to publish the individual measurements of each sensor.

### /sensors/thermal_sensor_\<idx\>/falsecolor
There is one publisher for each sensor that publishes a falsecolor image of the thermal measurement. The message type is [`sensor_msgs/msg/Image`](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html).

### /sensors/thermal_sensor_\<idx\>/grayscale
There is one publisher for each thermal sensor that publishes a grayscale image of the thermal measurement. The message type is [`sensor_msgs/msg/Image`](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html).

## 3. Subscriber
There are currently no subscribers in the edu_sensorring_ros1 node.

## 4. Services

### /startThermalCalibration
The service `startThermalCalibration` starts the calibration process of the thermal sensors. The calibration compensates a vignette effect that is apparent in the thermal measurements.

```
rosservice call /edu_sensorring_ros1_node/startThermalCalibration "window: 20"
```

>Thermal calibration:<br>1. Run the Sensorring node for 10 to 20 minutes that all sensors reach a steady operating temperature.<br>2. Place an object in front of the thermal sensors that covers the whole sensor area and has a uniform surface and uniform temperature, e.g. a cardboard box. Don't touch the side of your calibration target that is facing the sensor directly before the calibration.<br>3. Call the thermal calibration service. The window size defines how many thermal frames are averaged for the calibration. Recommended values are 20 to 100 frames.<br>4. Wait for the calibration to finish. If the `use_calibration_file` parameter is set to `true` the calibration values are automatically stored in a file and loaded at each start of the node.

### /stopThermalCalibration
The service `stopThermalCalibration` stops the calibration process of the thermal sensors in case the process takes too long.
```
rosservice call /edu_sensorring_ros1_node/stopThermalCalibration "stop: true"
```
