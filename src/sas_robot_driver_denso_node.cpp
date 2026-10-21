/*
# Copyright (c) 2016-2022 Murilo Marques Marinho
#
#    This file is part of sas_robot_driver_denso.
#
#    sas_denso_communcation is free software: you can redistribute it and/or modify
#    it under the terms of the GNU Lesser General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    sas_robot_driver_denso is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU Lesser General Public License for more details.
#
#    You should have received a copy of the GNU Lesser General Public License
#    along with sas_robot_driver_denso.  If not, see <https://www.gnu.org/licenses/>.
#
# ################################################################
#
#   Author: Murilo M. Marinho, email: murilomarinho@ieee.org
#
# ################################################################*/
// Modifications Copyright (c) 2026 Hung-Ching Huang (qlin1806@g.ecc.u-tokyo.ac.jp)
// 2026-10-21: Port the node to ROS 2 Jazzy and validate startup parameters.
// 2026-10-21: Expose startup-only read_only mode, defaulting to false.

#include <cmath>
#include <exception>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <sas_common/sas_common.hpp>
#include <sas_core/eigen3_std_conversions.hpp>
#include <sas_robot_driver/sas_robot_driver_ros.hpp>
#include <sas_robot_driver_denso/sas_robot_driver_denso.h>


/*********************************************
 * SIGNAL HANDLER
 * *******************************************/
#include<signal.h>
static std::atomic_bool kill_this_process(false);
void sig_int_handler(int)
{
    kill_this_process = true;
}

int main(int argc, char** argv)
{
    int exit_code = 1;
    try
    {
        if(signal(SIGINT, sig_int_handler) == SIG_ERR ||
           signal(SIGTERM, sig_int_handler) == SIG_ERR)
        {
            throw std::runtime_error("Error setting the shutdown signal handlers.");
        }
        rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
        auto node = std::make_shared<rclcpp::Node>("sas_robot_driver_denso_node");

        RCLCPP_INFO(node->get_logger(), "Loading parameters.");
        sas::RobotDriverDensoConfiguration robot_driver_denso_configuration{};
        rcl_interfaces::msg::ParameterDescriptor read_only_descriptor;
        read_only_descriptor.description = "Monitor joints without commanding the robot; set only at startup.";
        read_only_descriptor.read_only = true;
        robot_driver_denso_configuration.read_only = node->declare_parameter<bool>(
            "read_only", false, read_only_descriptor);
        if(robot_driver_denso_configuration.read_only)
        {
            RCLCPP_WARN(node->get_logger(),
                        "read_only=true: target commands are ignored; motor, speed, slave-mode "
                        "and arm-ownership commands are disabled.");
        }
        sas::get_ros_parameter(node,"robot_ip_address",robot_driver_denso_configuration.ip_address);
        sas::get_ros_parameter(node,"robot_port",robot_driver_denso_configuration.port);
        sas::get_ros_parameter(node,"robot_speed",robot_driver_denso_configuration.speed);
        sas::RobotDriverROSConfiguration robot_driver_ros_configuration{};
        sas::get_ros_parameter(node,"thread_sampling_time_sec",robot_driver_ros_configuration.thread_sampling_time_sec);
        sas::get_ros_parameter(node,"q_min",robot_driver_ros_configuration.q_min);
        sas::get_ros_parameter(node,"q_max",robot_driver_ros_configuration.q_max);
        robot_driver_ros_configuration.robot_driver_provider_prefix = node->get_fully_qualified_name();

        if(robot_driver_denso_configuration.ip_address.empty())
            throw std::runtime_error("robot_ip_address must not be empty.");
        if(robot_driver_denso_configuration.port < 1 || robot_driver_denso_configuration.port > 65535)
            throw std::runtime_error("robot_port must be between 1 and 65535.");
        if(!std::isfinite(robot_driver_denso_configuration.speed) || robot_driver_denso_configuration.speed <= 0.)
            throw std::runtime_error("robot_speed must be finite and positive.");
        if(!std::isfinite(robot_driver_ros_configuration.thread_sampling_time_sec) ||
           robot_driver_ros_configuration.thread_sampling_time_sec <= 0.)
            throw std::runtime_error("thread_sampling_time_sec must be finite and positive.");
        const auto& q_min = robot_driver_ros_configuration.q_min;
        const auto& q_max = robot_driver_ros_configuration.q_max;
        if(q_min.size() != 6 || q_max.size() != 6)
            throw std::runtime_error("q_min and q_max must each contain six joint limits in radians.");
        for(size_t i = 0; i < q_min.size(); ++i)
        {
            if(!std::isfinite(q_min[i]) || !std::isfinite(q_max[i]) || q_min[i] >= q_max[i])
                throw std::runtime_error("Joint limits must be finite with q_min < q_max for every joint.");
        }

        RCLCPP_INFO(node->get_logger(), "Instantiating DENSO robot (b-CAP).");
        auto robot_driver_denso = std::make_shared<sas::RobotDriverDenso>(
            robot_driver_denso_configuration, &kill_this_process);
        robot_driver_denso->set_joint_limits({sas::std_vector_double_to_vectorxd(q_min),
                                             sas::std_vector_double_to_vectorxd(q_max)});
        sas::RobotDriverROS robot_driver_ros(node,
                                             robot_driver_denso,
                                             robot_driver_ros_configuration,
                                             &kill_this_process);
        exit_code = robot_driver_ros.control_loop();
        // The installed SAS loop catches driver exceptions and still returns 0.
        // Only an explicit process shutdown counts as a successful loop exit.
        if(exit_code == 0 && !kill_this_process)
            exit_code = 1;
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("sas_robot_driver_denso"), "Exception: %s", e.what());
    }

    rclcpp::shutdown();
    return exit_code;
}
