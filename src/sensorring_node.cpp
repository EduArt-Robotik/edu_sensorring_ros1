#include <string>
#include <vector>
#include <array>
#include <chrono>
#include <stdexcept>

#include <ros/ros.h>
#include "SensorRingProxy.hpp"

using namespace eduart;

int main(int argc, char** argv) {
    ros::init(argc, argv, "sensorring");
    
    // Create SensorRing Node
    auto measurement_node = std::make_shared<sensorring::SensorRingProxy>("edu_sensorring_ros1_node");
    auto nh = measurement_node->getNodeHandle();
    ROS_INFO("Starting the sensorring node");

    std::string tf_name;
    ring::RingParams ring_params;
    manager::ManagerParams manager_params;

    // Get SensorRing parameters
    int timeout_ms = 0;
    std::string param_namespace = "/pointcloud_sensor";
    nh->param(param_namespace + "/base_setup/timeout_ms", timeout_ms, 1000);
    nh->param(param_namespace + "/base_setup/enable_brs", manager_params.enable_brs, true);
    nh->param(param_namespace + "/base_setup/tf_name", tf_name, std::string("base_sensorring"));
    nh->param(param_namespace + "/base_setup/print_topology", manager_params.print_topology, true);
    nh->param(param_namespace + "/base_setup/enforce_topology", manager_params.enforce_topology, false);
    nh->param(param_namespace + "/base_setup/frequency_tof_hz", manager_params.frequency_tof_hz, 5.0);
    nh->param(param_namespace + "/base_setup/frequency_thermal_hz", manager_params.frequency_thermal_hz, 1.0);
    if(timeout_ms > 0){
        ring_params.timeout = std::chrono::milliseconds(timeout_ms);
    } else{
        ROS_WARN("Invalid timeout value. Using default value of 1000 ms.");
        ring_params.timeout = std::chrono::milliseconds(1000);
    }

    bool thermal_auto_min_max, thermal_use_eeprom_file, thermal_use_calibration_file;
    std::string thermal_eeprom_dir, thermal_calibration_dir;
    double thermal_t_min, thermal_t_max;

    nh->param(param_namespace + "/thermal_config/auto_min_max", thermal_auto_min_max, true);
    nh->param(param_namespace + "/thermal_config/use_eeprom_file", thermal_use_eeprom_file, false);
    nh->param(param_namespace + "/thermal_config/use_calibration_file", thermal_use_calibration_file, false);
    nh->param(param_namespace + "/thermal_config/eeprom_file_dir", thermal_eeprom_dir, std::string(""));
    nh->param(param_namespace + "/thermal_config/calibration_file_dir", thermal_calibration_dir, std::string(""));
    nh->param(param_namespace + "/thermal_config/scale_t_min_deg", thermal_t_min, 15.0);
    nh->param(param_namespace + "/thermal_config/scale_t_max_deg", thermal_t_max, 25.0);

    int nr_of_can_interfaces;
    nh->param(param_namespace + "/topology/nr_of_interfaces", nr_of_can_interfaces, 1);

    int light_initial_mode_code;
    std::vector<int> light_color;
	nh->param(param_namespace + "/led_config/initial_mode", light_initial_mode_code, 0);
	nh->param(param_namespace + "/led_config/initial_color", light_color, std::vector<int>{0, 0, 0});
	light::LightMode light_initial_mode = static_cast<light::LightMode>(light_initial_mode_code);
	
	if(light_color.size() != 3){
		throw std::runtime_error("Light color vector has wrong length! Expected 3 values for RGB color.");
	}

	for(const auto& color_value : light_color){
		if(color_value < 0 || color_value > 255){
			throw std::runtime_error("Light color values must be in the range [0, 255]!");
		}
	}

    // Get parameters for every CAN interface
    param_namespace += "/topology/can_interfaces";
    int sensor_idx = 0;
    for (int i = 0; i < nr_of_can_interfaces; i++) {

        bus::BusParams bus_params;

        std::string interface_param_name = "/can_interface_" + std::to_string(i);
        std::string interface_type_str;
        std::string orientation_str;
        int nr_of_sensors;
        nh->param(param_namespace + interface_param_name + "/interface_type", interface_type_str, std::string("undefined"));
        nh->param(param_namespace + interface_param_name + "/interface_name", bus_params.interface_name, std::string("can0"));
        nh->param(param_namespace + interface_param_name + "/orientation", orientation_str, std::string("none"));
        nh->param(param_namespace + interface_param_name + "/nr_of_sensors", nr_of_sensors, 1);

        if(interface_type_str == "socketcan"){
			bus_params.type = com::InterfaceType::SOCKETCAN;
		}else if(interface_type_str == "usbtingo"){
			bus_params.type = com::InterfaceType::USBTINGO;
		}else{
			bus_params.type = com::InterfaceType::UNDEFINED;
		}

        sensor::Orientation orientation         = sensor::Orientation::none;
        if (orientation_str == "left") orientation      = sensor::Orientation::left;
        if (orientation_str == "right") orientation     = sensor::Orientation::right;

        // Get parameters for every sensor on the current CAN interface
        interface_param_name += "/sensors";
        for (int j = 0; j < nr_of_sensors; j++) {

            sensor::SensorBoardParams board_params;

            // Get parameters for the current sensor board
            bool enable_tof, enable_thermal, enable_light;
            std::string sensor_param_name = "/sensor_" + std::to_string(j);
            nh->param(param_namespace + interface_param_name + sensor_param_name + "/enable_tof", enable_tof, true);
            nh->param(param_namespace + interface_param_name + sensor_param_name + "/enable_thermal", enable_thermal, false);
            nh->param(param_namespace + interface_param_name + sensor_param_name + "/enable_light", enable_light, false);

            std::vector<double> rotation(3, 0.0), translation(3, 0.0);
            nh->param(param_namespace + interface_param_name + sensor_param_name + "/rotation", rotation, rotation);
            nh->param(param_namespace + interface_param_name + sensor_param_name + "/translation", translation, translation);
            
            if (rotation.size() == 3) {
                std::copy(rotation.begin(), rotation.end(), board_params.rotation.data.begin());
            } else {
				throw std::invalid_argument("Rotation vector of sensor " + std::to_string(j) + " on interface " + bus_params.interface_name + " has wrong length!");
            }
            if (translation.size() == 3) {
                std::copy(translation.begin(), translation.end(), board_params.translation.data.begin());
            } else {
				throw std::invalid_argument("Translation vector of sensor " + std::to_string(j) + " on interface " + bus_params.interface_name + " has wrong length!");
            }

            sensor::TofSensorParams tof_params;
			tof_params.enable		= enable_tof;
			tof_params.user_idx		= sensor_idx;

            sensor::ThermalSensorParams thermal_params;
            thermal_params.enable               = enable_thermal;
            thermal_params.user_idx				= sensor_idx;
            thermal_params.orientation          = orientation;
            thermal_params.auto_min_max         = thermal_auto_min_max;
            thermal_params.use_eeprom_file      = thermal_use_eeprom_file;
            thermal_params.use_calibration_file = thermal_use_calibration_file;
            thermal_params.eeprom_dir           = thermal_eeprom_dir;
            thermal_params.calibration_dir      = thermal_calibration_dir;
            thermal_params.t_min_deg_c          = thermal_t_min;
            thermal_params.t_max_deg_c          = thermal_t_max;

            sensor::LightParams light_params;
            light_params.enable = enable_light;
            light_params.orientation = orientation;

            // Create sensor board
            board_params.tof_params = tof_params;
            board_params.thermal_params = thermal_params;
            board_params.light_params = light_params;

            bus_params.board_param_vec.push_back(board_params);
            sensor_idx++;
        }
        ring_params.bus_param_vec.push_back(bus_params);
    }
    manager_params.ring_params = ring_params;

    // Run ROS node
    auto measurement_manager = std::make_unique<manager::MeasurementManager>(manager_params);
    bool success = measurement_node->run(std::move(measurement_manager), tf_name, light_initial_mode, light_color[0], light_color[1], light_color[2]);
    ros::shutdown();

    return success ? 0 : 1;
}
