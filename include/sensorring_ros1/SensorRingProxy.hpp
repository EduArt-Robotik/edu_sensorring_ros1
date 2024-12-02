#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensorring/MeasurementManager.hpp>
#include <sensorring_ros1/StartThermalCalibration.h>
#include <sensorring_ros1/StopThermalCalibration.h>

#include <vector>
#include <memory>
#include <string>

namespace sensorring {

    class SensorRingProxy : public eduart::manager::MeasurementObserver {
    public:
        SensorRingProxy(const std::string& node_name);

        ~SensorRingProxy();

        ros::NodeHandle* getNodeHandle();

        bool run(eduart::manager::ManagerParams params, const std::string& tf_name);

        bool isShutdown();

        void onStateChange(const eduart::manager::WorkerState state) override;

        void onTofMeasurement(const eduart::measurement::TofMeasurement measurement) override;

        void onThermalMeasurement(const std::size_t idx, const eduart::measurement::ThermalMeasurement measurement) override;

        void onOutputLog(const eduart::logger::LogVerbosity verbosity, const std::string msg) override;

    private:

        bool stopThermalCalibration(sensorring_ros1::StopThermalCalibration::Request& request,
                                    sensorring_ros1::StopThermalCalibration::Response& response);

        bool startThermalCalibration(sensorring_ros1::StartThermalCalibration::Request& request,
                                     sensorring_ros1::StartThermalCalibration::Response& response);

        bool _shutdown;
        std::unique_ptr<eduart::manager::MeasurementManager> _manager;

        sensor_msgs::PointCloud2 _pc2_msg;
        ros::Publisher _pointcloud_pub;

        std::vector<sensor_msgs::Image> _img_msg_vec;
        std::vector<ros::Publisher> _img_pub_vec;

        std::vector<sensor_msgs::Image> _colorimg_msg_vec;
        std::vector<ros::Publisher> _colorimg_pub_vec;

        ros::NodeHandle _nh;
        ros::ServiceServer _start_thermal_calib_srv;
        ros::ServiceServer _stop_thermal_calib_srv;
    };
}