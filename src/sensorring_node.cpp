#include <ros/ros.h>
#include <string>
#include <vector>
#include <array>
#include <chrono>
#include "SensorRingProxy.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "sensorring");
    
    // Create SensorRing Node
    auto measurement_node = std::make_shared<sensorring::SensorRingProxy>("sensorring");
    ROS_INFO("Starting the sensorring node");

    std::string tf_name;
    ring::RingParams ring_params;
    manager::ManagerParams manager_params;

    // Get SensorRing parameters
    int timeout_ms = 0;
    std::string param_namespace = "/pointcloud_sensor";
    measurement_node->getNodeHandle()->param(param_namespace + "/base_setup/timeout_ms", timeout_ms, 1000);
    measurement_node->getNodeHandle()->param(param_namespace + "/base_setup/tf_name", tf_name, std::string("base_sensorring"));
    measurement_node->getNodeHandle()->param(param_namespace + "/base_setup/print_topology", manager_params.print_topology, true);
    measurement_node->getNodeHandle()->param(param_namespace + "/base_setup/frequency_tof_hz", manager_params.frequency_tof_hz, 5.0);
    measurement_node->getNodeHandle()->param(param_namespace + "/base_setup/frequency_thermal_hz", manager_params.frequency_thermal_hz, 1.0);
    if(timeout_ms > 0){
        ring_params.timeout = std::chrono::milliseconds(timeout_ms);
    } else{
        ROS_WARN("Invalid timeout value. Using default value of 1000 ms.");
        ring_params.timeout = std::chrono::milliseconds(1000);
    }

    bool thermal_auto_min_max, thermal_use_eeprom_file, thermal_use_calibration_file;
    std::string thermal_eeprom_dir, thermal_calibration_dir;
    double thermal_t_min, thermal_t_max;

    measurement_node->getNodeHandle()->param(param_namespace + "/thermal_config/auto_min_max", thermal_auto_min_max, true);
    measurement_node->getNodeHandle()->param(param_namespace + "/thermal_config/use_eeprom_file", thermal_use_eeprom_file, false);
    measurement_node->getNodeHandle()->param(param_namespace + "/thermal_config/use_calibration_file", thermal_use_calibration_file, false);
    measurement_node->getNodeHandle()->param(param_namespace + "/thermal_config/eeprom_file_dir", thermal_eeprom_dir, std::string(""));
    measurement_node->getNodeHandle()->param(param_namespace + "/thermal_config/calibration_file_dir", thermal_calibration_dir, std::string(""));
    measurement_node->getNodeHandle()->param(param_namespace + "/thermal_config/scale_t_min_deg", thermal_t_min, 15.0);
    measurement_node->getNodeHandle()->param(param_namespace + "/thermal_config/scale_t_max_deg", thermal_t_max, 25.0);

    int nr_of_can_interfaces;
    measurement_node->getNodeHandle()->param(param_namespace + "/topology/nr_of_interfaces", nr_of_can_interfaces, 1);

    // Get parameters for every CAN interface
    param_namespace += "/topology/can_interfaces";
    for (int i = 0; i < nr_of_can_interfaces; i++) {
        bus::BusParams bus_params;

        std::string interface_param_name = "/can_interface_" + std::to_string(i);
        measurement_node->getNodeHandle()->param(param_namespace + interface_param_name + "/interface_name", bus_params.interface_name, std::string("can0"));
        std::string orientation_str;
        measurement_node->getNodeHandle()->param(param_namespace + interface_param_name + "/orientation", orientation_str, std::string("none"));
        int nr_of_sensors;
        measurement_node->getNodeHandle()->param(param_namespace + interface_param_name + "/nr_of_sensors", nr_of_sensors, 1);

        sensor::SensorOrientation orientation = sensor::SensorOrientation::none;
        if (orientation_str == "left") orientation = sensor::SensorOrientation::left;
        if (orientation_str == "right") orientation = sensor::SensorOrientation::right;

        // Get parameters for every sensor on the current CAN interface
        interface_param_name += "/sensors";
        for (int j = 0; j < nr_of_sensors; j++) {
            std::string sensor_param_name = "/sensor_" + std::to_string(j);

            bool enable_tof, enable_thermal;
            measurement_node->getNodeHandle()->param(param_namespace + interface_param_name + sensor_param_name + "/get_tof", enable_tof, true);
            measurement_node->getNodeHandle()->param(param_namespace + interface_param_name + sensor_param_name + "/get_thermal", enable_thermal, false);

            std::vector<double> rotation(3, 0.0), translation(3, 0.0);
            measurement_node->getNodeHandle()->param(param_namespace + interface_param_name + sensor_param_name + "/rotation", rotation, rotation);
            measurement_node->getNodeHandle()->param(param_namespace + interface_param_name + sensor_param_name + "/translation", translation, translation);

            sensor::TofSensorParams tof_params;
            if (rotation.size() == 3) {
                std::copy(rotation.begin(), rotation.end(), tof_params.rotation.data.begin());
            } else {
                ROS_ERROR_STREAM("Rotation vector of sensor " << j << " on interface " 
                                << bus_params.interface_name << " has wrong length!");
            }
            if (translation.size() == 3) {
                std::copy(translation.begin(), translation.end(), tof_params.translation.data.begin());
            } else {
                ROS_ERROR_STREAM("Translation vector of sensor " << j << " on interface " 
                                << bus_params.interface_name << " has wrong length!");
            }

            sensor::ThermalSensorParams thermal_params;
            thermal_params.rotation = tof_params.rotation;
            thermal_params.translation = tof_params.translation;
            thermal_params.orientation = orientation;
            thermal_params.auto_min_max = thermal_auto_min_max;
            thermal_params.use_eeprom_file = thermal_use_eeprom_file;
            thermal_params.use_calibration_file = thermal_use_calibration_file;
            thermal_params.eeprom_dir = thermal_eeprom_dir;
            thermal_params.calibration_dir = thermal_calibration_dir;
            thermal_params.t_min_deg_c = thermal_t_min;
            thermal_params.t_max_deg_c = thermal_t_max;

            sensor::LightParams led_params;
            led_params.orientation = orientation;

            // Create sensor board
            sensor::SensorBoardParams board_params;
            board_params.idx = j;
            board_params.enable_tof = enable_tof;
            board_params.enable_thermal = enable_thermal;
            board_params.tof_params = tof_params;
            board_params.thermal_params = thermal_params;
            board_params.led_params = led_params;

            bus_params.board_param_vec.push_back(board_params);
        }
        ring_params.bus_param_vec.push_back(bus_params);
    }
    manager_params.ring_params = ring_params;

    // Run ROS node
    bool success = measurement_node->run(manager_params, tf_name);
    ros::shutdown();

    return success ? 0 : 1;
}
