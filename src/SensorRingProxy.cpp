#include "SensorRingProxy.hpp"

#include <sensor_msgs/PointField.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <algorithm>

namespace eduart{

namespace sensorring {

SensorRingProxy::SensorRingProxy(const std::string& node_name) :
_nh(node_name), MeasurementClient(), LoggerClient() {
	logger::Logger::getInstance()->registerClient(this);
}

SensorRingProxy::~SensorRingProxy(){
	if(_manager)
		_manager->unregisterClient(this);
	logger::Logger::getInstance()->unregisterClient(this);
}

ros::NodeHandle* SensorRingProxy::getNodeHandle(){
    return &_nh;
}

bool SensorRingProxy::run(std::unique_ptr<manager::MeasurementManager> manager, const std::string& tf_name, light::LightMode initial_light_mode, std::uint8_t red, std::uint8_t green, std::uint8_t blue) {

    // create MeasurementManager
	_manager = std::move(manager);
    _manager->registerClient(this);
    _manager->setLight(initial_light_mode, red, green, blue);

    // Prepare PointCloud2 message
	sensor_msgs::PointCloud2 pc2_msg;
    pc2_msg.header.frame_id = tf_name;
    pc2_msg.height = 1;
    pc2_msg.point_step = sizeof(measurement::PointData);
    pc2_msg.is_bigendian = true;
    pc2_msg.is_dense = true;

    sensor_msgs::PointField field_x;
    field_x.name = "x";
    field_x.offset = 0;
    field_x.datatype = sensor_msgs::PointField::FLOAT32;
    field_x.count = 1;

    sensor_msgs::PointField field_y;
    field_y.name = "y";
    field_y.offset = 4;
    field_y.datatype = sensor_msgs::PointField::FLOAT32;
    field_y.count = 1;

    sensor_msgs::PointField field_z;
    field_z.name = "z";
    field_z.offset = 8;
    field_z.datatype = sensor_msgs::PointField::FLOAT32;
    field_z.count = 1;

	sensor_msgs::PointField field_sigma;
	field_sigma.name = "sigma";
	field_sigma.offset = 16;
	field_sigma.datatype = sensor_msgs::PointField::FLOAT32;
	field_sigma.count = 1;

	sensor_msgs::PointField field_dist;
	field_dist.name = "raw_distance";
	field_dist.offset = 12;
	field_dist.datatype = sensor_msgs::PointField::FLOAT32;
	field_dist.count = 1;

	sensor_msgs::PointField field_idx;
	field_idx.name = "sensor_idx";
	field_idx.offset = 20;
	field_idx.datatype = sensor_msgs::PointField::INT32;
	field_idx.count = 1;

    pc2_msg.fields = {field_x, field_y, field_z, field_sigma, field_dist, field_idx};

	_pc2_msg_raw = pc2_msg;
	_pc2_msg_transformed = pc2_msg;

    // Create PointCloud2 publisher
    _pointcloud_pub_raw         = _nh.advertise<sensor_msgs::PointCloud2>("/sensors/tof_sensors/pcl_raw", 1);
    _pointcloud_pub_transformed = _nh.advertise<sensor_msgs::PointCloud2>("/sensors/tof_sensors/pcl_transformed", 1);

    // Prepare image publishers for each active thermal sensor
    int i = 0;
    for (const auto& sensor_bus : _manager->getParams().ring_params.bus_param_vec) {
        for (const auto& sensor_board : sensor_bus.board_param_vec) {
            if (sensor_board.thermal_params.enable) {
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
    i = 0;
    tf2_ros::StaticTransformBroadcaster tf_broadcaster;
    for (const auto& sensor_bus : _manager->getParams().ring_params.bus_param_vec) {
        for (const auto& sensor_board : sensor_bus.board_param_vec) {
            geometry_msgs::TransformStamped t;
            t.header.stamp = ros::Time::now();
            t.header.frame_id = tf_name;
            t.child_frame_id = "sensor_" + std::to_string(i);

            t.transform.translation.x = sensor_board.translation.x();
            t.transform.translation.y = sensor_board.translation.y();
            t.transform.translation.z = sensor_board.translation.z();

            tf2::Quaternion q;
            q.setRPY(
                sensor_board.rotation.x() * M_PI / 180.0,
                sensor_board.rotation.y() * M_PI / 180.0,
                sensor_board.rotation.z() * M_PI / 180.0);
            t.transform.rotation.x = q.x();
            t.transform.rotation.y = q.y();
            t.transform.rotation.z = q.z();
            t.transform.rotation.w = q.w();

            tf_broadcaster.sendTransform(t);

			// prepare publisher for individual sensor pointclouds
			pc2_msg.header.frame_id = t.child_frame_id;
			_pc2_msg_individual_vec.push_back(pc2_msg);
			_pointcloud_pub_individual_vec.push_back(_nh.advertise<sensor_msgs::PointCloud2>("/sensors/tof_sensors/pcl_individual/sensor_" + std::to_string(i), 1));
            
            i++;
        }
    }

    // Set up ROS services
    _start_thermal_calib_srv = _nh.advertiseService(
        _nh.getNamespace() + "/startThermalCalibration", &SensorRingProxy::startThermalCalibration, this);
    _stop_thermal_calib_srv = _nh.advertiseService(
        _nh.getNamespace() + "/stopThermalCalibration", &SensorRingProxy::stopThermalCalibration, this);

    // Force first state update
    onStateChange(_manager->getManagerState());

    bool success = _manager->startMeasuring();

    if (success) {
        ros::Rate loop_rate(10);
        while (_manager->isMeasuring() && ros::ok()) {
            ros::spinOnce();
            loop_rate.sleep();
        }

        success = _manager->stopMeasuring();
    }

    return static_cast<int>(success);
}

void SensorRingProxy::onStateChange(const manager::ManagerState state) {
    if(state < manager::ManagerState::Error){
		ROS_DEBUG_STREAM("New MeasurementManager state: " << state);
	}else{
		ROS_ERROR_STREAM("New MeasurementManager state: " << state);
	}
}

void SensorRingProxy::onRawTofMeasurement(std::vector<measurement::TofMeasurement> measurement_vec){
	if(!measurement_vec.empty()){

        int idx = 0;
		auto now = ros::Time::now();

        std::size_t point_count = 0;
		for(const auto& measurement : measurement_vec){
			point_count += measurement.point_cloud.size();

            // prepare individual pc2 messages
			auto& msg           = _pc2_msg_individual_vec.at(idx);
			msg.header.stamp	= now;
			msg.width			= measurement.point_cloud.size();
			msg.row_step		= msg.width * msg.point_step;
			msg.data.resize(msg.row_step);
			idx++;
		}

        // prepare combined pc2 message
        _pc2_msg_raw.header.stamp   = now;
        _pc2_msg_raw.width          = point_count;
        _pc2_msg_raw.row_step       = _pc2_msg_raw.width * _pc2_msg_raw.point_step;
        _pc2_msg_raw.data.resize(_pc2_msg_raw.row_step);

		idx = 0;
		std::uint8_t* data_ptr = _pc2_msg_raw.data.data();
		for(const auto& measurement : measurement_vec){
			// prepare combined measurement
			data_ptr = packPointData(measurement, data_ptr);

			// prepare and publish individual measurement
			auto& msg = _pc2_msg_individual_vec.at(idx);
			std::uint8_t* data_ptr_individual =  msg.data.data();
			packPointData(measurement, data_ptr_individual);
			_pointcloud_pub_individual_vec.at(idx).publish(msg);
			idx++;
		}

        // publish combined measurement
        _pointcloud_pub_raw.publish(_pc2_msg_raw);
    }
}

void SensorRingProxy::onTransformedTofMeasurement(std::vector<measurement::TofMeasurement> measurement_vec){
	if(!measurement_vec.empty()){

        std::size_t point_count = 0;
		for(const auto& measurement : measurement_vec){
			point_count += measurement.point_cloud.size();
		}

        _pc2_msg_transformed.header.stamp   = ros::Time::now();
        _pc2_msg_transformed.width          = point_count;
        _pc2_msg_transformed.row_step       = _pc2_msg_transformed.width * _pc2_msg_transformed.point_step;
        _pc2_msg_transformed.data.resize(_pc2_msg_transformed.row_step );

		std::uint8_t* data_ptr = _pc2_msg_transformed.data.data();
		for(const auto& measurement : measurement_vec){
			data_ptr = packPointData(measurement, data_ptr);
		}

        _pointcloud_pub_transformed.publish(_pc2_msg_transformed);
    }
}

void SensorRingProxy::onOutputLog(const logger::LogVerbosity verbosity, const std::string msg){
    switch (verbosity) {
        case logger::LogVerbosity::Debug:
            ROS_DEBUG("%s", msg.c_str());
            break;
        case logger::LogVerbosity::Info:
            ROS_INFO("%s", msg.c_str());
            break;
        case logger::LogVerbosity::Warning:
            ROS_WARN("%s", msg.c_str());
            break;
        case logger::LogVerbosity::Error:
            ROS_ERROR("%s", msg.c_str());
            break;
    }
}

void SensorRingProxy::onThermalMeasurement(std::vector<measurement::ThermalMeasurement> measurement_vec) {
    int idx = 0;
	for(const auto& measurement : measurement_vec){
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
        idx++;
    }
}

bool SensorRingProxy::stopThermalCalibration(edu_sensorring_ros1::StopThermalCalibration::Request& request,
                                             edu_sensorring_ros1::StopThermalCalibration::Response& response) {
    if (request.stop) {
        response.output = _manager->stopThermalCalibration();
    } else {
        response.output = false;
    }
    return true;
}

bool SensorRingProxy::startThermalCalibration(edu_sensorring_ros1::StartThermalCalibration::Request& request,
                                              edu_sensorring_ros1::StartThermalCalibration::Response& response) {
    response.output = _manager->startThermalCalibration(static_cast<std::size_t>(request.window));
    return true;
}

std::uint8_t* SensorRingProxy::packPointData(const measurement::TofMeasurement& src, std::uint8_t* dst)
{
    for (const auto& p : src.point_cloud) {
        float* f = reinterpret_cast<float*>(dst);
        f[0] = static_cast<float>(p.point.data[0]);
        f[1] = static_cast<float>(p.point.data[1]);
        f[2] = static_cast<float>(p.point.data[2]);
        f[3] = static_cast<float>(p.raw_distance);
        f[4] = static_cast<float>(p.sigma);
        reinterpret_cast<int32_t*>(f + 5)[0] = p.user_idx;
        dst += sizeof(measurement::PointData);
	}

	return dst;
}

}

}