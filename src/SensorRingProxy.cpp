#include "SensorRingProxy.hpp"

#include <sensor_msgs/PointField.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <algorithm>

namespace sensorring {

SensorRingProxy::SensorRingProxy(const std::string& node_name) 
    : MeasurementObserver(), _shutdown(false), _nh(node_name) {
}

SensorRingProxy::~SensorRingProxy() {
}

ros::NodeHandle* SensorRingProxy::getNodeHandle(){
    return &_nh;
}

bool SensorRingProxy::run(manager::ManagerParams params, const std::string& tf_name) {
    _manager = std::make_unique<manager::MeasurementManager>(params, static_cast<MeasurementObserver*>(this));

    // Prepare PointCloud2 message
    _pc2_msg = sensor_msgs::PointCloud2();
    _pc2_msg.header.frame_id = tf_name;
    _pc2_msg.height = 1;
    _pc2_msg.point_step = 4 * sizeof(float); // Dimensions per zone (x, y, z, sigma) * bytes per dimension (float32 -> 4 bytes)
    _pc2_msg.is_bigendian = true;
    _pc2_msg.is_dense = true;

    sensor_msgs::PointField field_x, field_y, field_z, field_sigma;

    field_x.name = "x";
    field_x.offset = 0;
    field_x.datatype = sensor_msgs::PointField::FLOAT32;
    field_x.count = 1;

    field_y.name = "y";
    field_y.offset = 4;
    field_y.datatype = sensor_msgs::PointField::FLOAT32;
    field_y.count = 1;

    field_z.name = "z";
    field_z.offset = 8;
    field_z.datatype = sensor_msgs::PointField::FLOAT32;
    field_z.count = 1;

    field_sigma.name = "sigma";
    field_sigma.offset = 12;
    field_sigma.datatype = sensor_msgs::PointField::FLOAT32;
    field_sigma.count = 1;

    _pc2_msg.fields = {field_x, field_y, field_z, field_sigma};

    // Create PointCloud2 publisher
    _pointcloud_pub = _nh.advertise<sensor_msgs::PointCloud2>("/sensors/tof_sensors/pcl_raw", 1);

    // Prepare image publishers for each active thermal sensor
    int i = 0;
    for (const auto& sensor_bus : _manager->getParams().ring_params.bus_param_vec) {
        for (const auto& sensor_board : sensor_bus.board_param_vec) {
            if (sensor_board.enable_thermal) {
                // =================== Temperature image ===================
                std::string sensor_name = "thermal_sensor_" + std::to_string(i) + "/grayscale";

                auto img_pub = _nh.advertise<sensor_msgs::Image>("/sensors/" + sensor_name, 1);
                sensor_msgs::Image img_msg;
                img_msg.header.frame_id = sensor_name;
                img_msg.height = 32;
                img_msg.width = 32;
                img_msg.encoding = "mono8";
                img_msg.is_bigendian = false;
                img_msg.step = img_msg.width;

                _img_msg_vec.push_back(img_msg);
                _img_pub_vec.push_back(img_pub);

                // =================== False color image ===================
                sensor_name = "thermal_sensor_" + std::to_string(i) + "/falsecolor";

                auto colorimg_pub = _nh.advertise<sensor_msgs::Image>("/sensors/" + sensor_name, 1);
                sensor_msgs::Image colorimg_msg;
                colorimg_msg.header.frame_id = sensor_name;
                colorimg_msg.height = 32;
                colorimg_msg.width = 32;
                colorimg_msg.encoding = "rgb8";
                colorimg_msg.is_bigendian = false;
                colorimg_msg.step = colorimg_msg.width * 3;

                _colorimg_msg_vec.push_back(colorimg_msg);
                _colorimg_pub_vec.push_back(colorimg_pub);
            }
            i++;
        }
    }

    // Send static transforms for each sensor
    tf2_ros::StaticTransformBroadcaster tf_broadcaster;

    i = 0;
    for (const auto& sensor_bus : _manager->getParams().ring_params.bus_param_vec) {
        for (const auto& sensor_board : sensor_bus.board_param_vec) {
            geometry_msgs::TransformStamped t;
            t.header.stamp = ros::Time::now();
            t.header.frame_id = tf_name;
            t.child_frame_id = "sensor_" + std::to_string(i);

            t.transform.translation.x = sensor_board.tof_params.translation.x();
            t.transform.translation.y = sensor_board.tof_params.translation.y();
            t.transform.translation.z = sensor_board.tof_params.translation.z();

            tf2::Quaternion q;
            q.setRPY(
                sensor_board.tof_params.rotation.x() * M_PI / 180.0,
                sensor_board.tof_params.rotation.y() * M_PI / 180.0,
                sensor_board.tof_params.rotation.z() * M_PI / 180.0);
            t.transform.rotation.x = q.x();
            t.transform.rotation.y = q.y();
            t.transform.rotation.z = q.z();
            t.transform.rotation.w = q.w();

            tf_broadcaster.sendTransform(t);
            i++;
        }
    }

    // Set up ROS services
    _start_thermal_calib_srv = _nh.advertiseService(
        _nh.getNamespace() + "/startThermalCalibration", &SensorRingProxy::startThermalCalibration, this);
    _stop_thermal_calib_srv = _nh.advertiseService(
        _nh.getNamespace() + "/stopThermalCalibration", &SensorRingProxy::stopThermalCalibration, this);

    // Force first state update
    onStateChange(_manager->getWorkerState());

    bool success = _manager->startMeasuring();

    if (success) {
        ros::Rate loop_rate(10);
        while (!_shutdown && ros::ok()) {
            ros::spinOnce();
            loop_rate.sleep();
        }

        success = _manager->stopMeasuring();
    }

    return static_cast<int>(success);
}

void SensorRingProxy::onStateChange(const WorkerState state) {
    switch (state) {
        case WorkerState::Initialized:
            ROS_DEBUG("New MeasurementManager state: initialized");
            break;
        case WorkerState::Running:
            ROS_DEBUG("New MeasurementManager state: running");
            break;
        case WorkerState::Shutdown:
            ROS_INFO("New MeasurementManager state: shutdown");
            _shutdown = true;
            break;
        case WorkerState::Error:
            ROS_ERROR("New MeasurementManager state: error");
            _shutdown = true;
            break;
    }
}

bool SensorRingProxy::isShutdown() {
    return _shutdown;
}

void SensorRingProxy::onTofMeasurement(const measurement::TofMeasurement measurement) {
    if (measurement.size > 0) {
        _pc2_msg.header.stamp = ros::Time::now();
        _pc2_msg.width = measurement.size;
        _pc2_msg.row_step = _pc2_msg.width * _pc2_msg.point_step;
        _pc2_msg.data.resize(_pc2_msg.row_step * _pc2_msg.height);

        float* data_ptr = reinterpret_cast<float*>(_pc2_msg.data.data());

        for (size_t i = 0; i < _pc2_msg.width; i++) {
            data_ptr[i * 4 + 0] = static_cast<float>(measurement.point_data_transformed[i].x()); // x
            data_ptr[i * 4 + 1] = static_cast<float>(measurement.point_data_transformed[i].y()); // y
            data_ptr[i * 4 + 2] = static_cast<float>(measurement.point_data_transformed[i].z()); // z
            data_ptr[i * 4 + 3] = static_cast<float>(measurement.point_sigma[i]);                // sigma
        }

        _pointcloud_pub.publish(_pc2_msg);
    }
}

void SensorRingProxy::onOutputLog(const LogVerbosity verbosity, const std::string msg){
    switch (verbosity) {
        case LogVerbosity::Debug:
            ROS_DEBUG("%s", msg.c_str());
            break;
        case LogVerbosity::Info:
            ROS_INFO("%s", msg.c_str());
            break;
        case LogVerbosity::Warning:
            ROS_WARN("%s", msg.c_str());
            break;
        case LogVerbosity::Error:
            ROS_ERROR("%s", msg.c_str());
            break;
    }
}

void SensorRingProxy::onThermalMeasurement(const std::size_t idx, const measurement::ThermalMeasurement measurement) {
    // Prepare and publish grayscale image
    auto& img_msg = _img_msg_vec.at(idx);
    size_t size = img_msg.width * img_msg.height;
    img_msg.header.stamp = ros::Time::now();
    img_msg.data.resize(size);
    std::copy(measurement.grayscale_img.data.begin(), measurement.grayscale_img.data.end(), img_msg.data.begin());
    _img_pub_vec[idx].publish(img_msg);

    // Prepare and publish false color image
    const std::uint8_t* color_data_ptr = measurement.falsecolor_img.data.begin()->begin();
    auto& colorimg_msg = _colorimg_msg_vec.at(idx);
    size *= 3;
    colorimg_msg.header.stamp = ros::Time::now();
    colorimg_msg.data.resize(size);
    //std::copy(measurement.falsecolor_img.data[0].begin(), measurement.falsecolor_img.data[0].end(), colorimg_msg.data.begin());
    std::copy_n(color_data_ptr, size, colorimg_msg.data.data());
    _colorimg_pub_vec[idx].publish(colorimg_msg);
}

bool SensorRingProxy::stopThermalCalibration(sensorring_ros1::StopThermalCalibration::Request& request,
                                             sensorring_ros1::StopThermalCalibration::Response& response) {
    if (request.stop) {
        response.output = _manager->stopThermalCalibration();
    } else {
        response.output = false;
    }
    return true;
}

bool SensorRingProxy::startThermalCalibration(sensorring_ros1::StartThermalCalibration::Request& request,
                                              sensorring_ros1::StartThermalCalibration::Response& response) {
    response.output = _manager->startThermalCalibration(static_cast<std::size_t>(request.window));
    return true;
}

}