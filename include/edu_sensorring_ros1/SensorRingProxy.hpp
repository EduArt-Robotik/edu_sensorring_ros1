#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <edu_sensorring_ros1/StartThermalCalibration.h>
#include <edu_sensorring_ros1/StopThermalCalibration.h>

#include <sensorring/logger/Logger.hpp>
#include <sensorring/MeasurementManager.hpp>

#include <vector>
#include <memory>
#include <string>

namespace eduart{

namespace sensorring {

    class SensorRingProxy : public manager::MeasurementClient, logger::LoggerClient {
    public:
        SensorRingProxy(const std::string& node_name);

        ~SensorRingProxy();

        ros::NodeHandle* getNodeHandle();

        //bool run(manager::ManagerParams params, const std::string& tf_name);
        bool run(std::unique_ptr<manager::MeasurementManager> manager, const std::string& tf_name, light::LightMode initial_light_mode = light::LightMode::Off, std::uint8_t red = 0, std::uint8_t green = 0, std::uint8_t blue = 0);

        bool isShutdown();

        void onStateChange(const manager::ManagerState state) override;

        void onRawTofMeasurement(const std::vector<measurement::TofMeasurement>& measurement_vec) override;

        void onTransformedTofMeasurement(const std::vector<measurement::TofMeasurement>& measurement_vec) override;

        void onThermalMeasurement(const std::vector<measurement::ThermalMeasurement>& measurement_vec) override;

        void onOutputLog(const logger::LogVerbosity verbosity, const std::string& msg) override;

    private:

        bool stopThermalCalibration(edu_sensorring_ros1::StopThermalCalibration::Request& request,
                                    edu_sensorring_ros1::StopThermalCalibration::Response& response);

        bool startThermalCalibration(edu_sensorring_ros1::StartThermalCalibration::Request& request,
                                     edu_sensorring_ros1::StartThermalCalibration::Response& response);

        std::uint8_t* packPointData(const measurement::TofMeasurement& src, std::uint8_t* dst);

        std::unique_ptr<manager::MeasurementManager> _manager;

        sensor_msgs::PointCloud2 _pc2_msg_raw;
        ros::Publisher _pointcloud_pub_raw;

        sensor_msgs::PointCloud2 _pc2_msg_transformed;
        ros::Publisher _pointcloud_pub_transformed;

        std::vector<sensor_msgs::PointCloud2> _pc2_msg_individual_vec;
        std::vector<ros::Publisher> _pointcloud_pub_individual_vec;

        std::vector<sensor_msgs::Image> _img_msg_vec;
        std::vector<ros::Publisher> _img_pub_vec;

        std::vector<sensor_msgs::Image> _colorimg_msg_vec;
        std::vector<ros::Publisher> _colorimg_pub_vec;

        ros::NodeHandle _nh;
        ros::ServiceServer _start_thermal_calib_srv;
        ros::ServiceServer _stop_thermal_calib_srv;
    };
}

}