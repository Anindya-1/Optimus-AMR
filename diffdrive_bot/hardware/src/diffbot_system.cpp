#include "diffdrive_bot/diffbot_system.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include <string>

namespace diffdrive_bot
{
    hardware_interface::CallbackReturn DiffBotSystemHardware::on_init(const hardware_interface::HardwareInfo &info)
    {    
        // executes on_init on base class, if fails -> returns error
        if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        l_drive_.drive_name = (info_.hardware_parameters["l_drive_name"]);
        l_drive_.serial_port = (info_.hardware_parameters["l_serial_port"]);
        l_drive_.baud = std::stoi(info_.hardware_parameters["l_baud"]);
        l_drive_.stop_bit = std::stoi(info_.hardware_parameters["l_stop_bit"]);
        l_drive_.data_bit = std::stoi(info_.hardware_parameters["l_data_bit"]);
        l_drive_.parity = ((info_.hardware_parameters["l_parity"]).c_str())[0];
        l_drive_.slave_id = std::stoi(info_.hardware_parameters["l_slave_id"]);
        l_drive_.gear_ratio = std::stoi(info_.hardware_parameters["l_gear_ratio"]);

        r_drive_.drive_name = (info_.hardware_parameters["r_drive_name"]);
        r_drive_.serial_port = (info_.hardware_parameters["r_serial_port"]);
        r_drive_.baud = std::stoi(info_.hardware_parameters["r_baud"]);
        r_drive_.stop_bit = std::stoi(info_.hardware_parameters["r_stop_bit"]);
        r_drive_.data_bit = std::stoi(info_.hardware_parameters["r_data_bit"]);
        r_drive_.parity = ((info_.hardware_parameters["r_parity"]).c_str())[0];
        r_drive_.slave_id = std::stoi(info_.hardware_parameters["r_slave_id"]);
        r_drive_.gear_ratio = std::stoi(info_.hardware_parameters["r_gear_ratio"]);

        for (const hardware_interface::ComponentInfo & joint : info_.joints)
        {            
            if (joint.command_interfaces.size() != 1)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("HardwareInterface"),
                    "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
                    joint.command_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("UnrealInterface"),
                    "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
                    joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces.size() != 2)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("UnrealInterface"),
                    "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
                    joint.state_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("HardwareInterface"),
                    "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
                    joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("HardwareInterface"),
                    "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
                    joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
                return hardware_interface::CallbackReturn::ERROR;
            }

            // if (joint.state_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
            // {
            //     RCLCPP_FATAL(
            //         rclcpp::get_logger("HardwareInterface"),
            //         "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
            //         joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
            //     return hardware_interface::CallbackReturn::ERROR;
            // }
            
        }

        return hardware_interface::CallbackReturn::SUCCESS;   
    }

    hardware_interface::CallbackReturn DiffBotSystemHardware::on_configure(const rclcpp_lifecycle::State & previous_state)
    {
        RCLCPP_INFO(rclcpp::get_logger("HardwareInterface"), "Successfully configured!");        
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn DiffBotSystemHardware::on_activate(const rclcpp_lifecycle::State & previous_state)
    {
        l_drive_.initialize(l_drive_.serial_port, l_drive_.baud, l_drive_.parity, l_drive_.data_bit, l_drive_.stop_bit, l_drive_.slave_id, l_drive_.gear_ratio);
        r_drive_.initialize(r_drive_.serial_port, r_drive_.baud, r_drive_.parity, r_drive_.data_bit, r_drive_.stop_bit, r_drive_.slave_id, r_drive_.gear_ratio);
        
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn DiffBotSystemHardware::on_deactivate(const rclcpp_lifecycle::State & previous_state)
    {
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> DiffBotSystemHardware::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        
        for (auto i=0u; i < info_.joints.size(); i++)
        // {
        //     state_interfaces.emplace_back(hardware_interface::StateInterface(
        //         info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
        //     state_interfaces.emplace_back(hardware_interface::StateInterface(
        //         info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
        // }

        state_interfaces.emplace_back(hardware_interface::StateInterface(
            l_drive_.drive_name, hardware_interface::HW_IF_VELOCITY, &l_drive_.joint_velocity));

        state_interfaces.emplace_back(hardware_interface::StateInterface(
            r_drive_.drive_name, hardware_interface::HW_IF_VELOCITY, &r_drive_.joint_velocity));

        state_interfaces.emplace_back(hardware_interface::StateInterface(
            l_drive_.drive_name, hardware_interface::HW_IF_POSITION, &l_drive_.position));

        state_interfaces.emplace_back(hardware_interface::StateInterface(
            r_drive_.drive_name, hardware_interface::HW_IF_POSITION, &r_drive_.position));

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> DiffBotSystemHardware::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        // for (auto i=0u; i < info_.joints.size(); i++)
        // {
        //     command_interfaces.emplace_back(hardware_interface::CommandInterface(
        //         info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
        // }

        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            l_drive_.drive_name, hardware_interface::HW_IF_VELOCITY, &l_drive_.cmd_velocity));

        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            r_drive_.drive_name, hardware_interface::HW_IF_VELOCITY, &r_drive_.cmd_velocity));

        return command_interfaces;
    }

    hardware_interface::return_type DiffBotSystemHardware::read(const rclcpp::Time &/*time*/, const rclcpp::Duration &/*period*/)
    {
        l_drive_.joint_velocity = l_drive_.getSpeed();
        r_drive_.joint_velocity = r_drive_.getSpeed();
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type DiffBotSystemHardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration &/*period*/)
    {
        l_drive_.setSpeed(l_drive_.cmd_velocity);
        r_drive_.setSpeed(-1*r_drive_.cmd_velocity);
        return hardware_interface::return_type::OK;
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  diffdrive_bot::DiffBotSystemHardware, hardware_interface::SystemInterface)