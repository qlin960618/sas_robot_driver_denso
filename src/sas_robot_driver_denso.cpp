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
// 2026-10-21: Keep read-only sessions free of motion and arm-control commands.

#include "sas_robot_driver_denso/sas_robot_driver_denso.h"

#include <sas_core/sas_clock.hpp>

#include <dqrobotics/utils/DQ_Math.h>
#include "../../src/sas_driver_bcap.h"

#include <vector>
#include <memory>

namespace sas
{
const int RobotDriverDenso::SLAVE_MODE_JOINT_CONTROL        = 0x102;
const int RobotDriverDenso::SLAVE_MODE_END_EFFECTOR_CONTROL = 0x103;

RobotDriverDenso::RobotDriverDenso(const RobotDriverDensoConfiguration &configuration, std::atomic_bool *break_loops):
    RobotDriver(break_loops),
    configuration_(configuration),
    bcap_driver_(new DriverBcap(configuration.ip_address, configuration.port))
{
    joint_positions_.resize(6);
    end_effector_pose_.resize(7);
    joint_positions_buffer_.resize(8,0);
    end_effector_pose_euler_buffer_.resize(7,0);
    end_effector_pose_homogenous_transformation_buffer_.resize(10,0);
}

VectorXd RobotDriverDenso::get_joint_positions()
{
    const bool worked = bcap_driver_->get_joint_positions(joint_positions_buffer_);
    if(!worked)
    {
        throw std::runtime_error("Error in DensoRobotDriver::get_joint_positions. bCapDriver::" + bcap_driver_->get_last_error_info());
    }
    Map<VectorXd> joint_positions_(joint_positions_buffer_.data(),6);
    return deg2rad(joint_positions_);
}

VectorXd RobotDriverDenso::_get_end_effector_pose_homogenous_transformation()
{
    const bool worked = bcap_driver_->get_end_effector_pose_homogenous_transformation(end_effector_pose_homogenous_transformation_buffer_);
    if(!worked)
    {
        throw std::runtime_error("Error in DensoRobotDriver::_get_end_effector_pose_homogenous_transformation. bCapDriver::" + bcap_driver_->get_last_error_info());
    }
    Map<VectorXd> end_effetor_pose(end_effector_pose_homogenous_transformation_buffer_.data(),10);
    return end_effetor_pose;
}

double RobotDriverDenso::_sign(const double& a) const
{
    if(a>0)
        return 1;
    else
        return -1;
}

DQ RobotDriverDenso::_homogenous_vector_to_dq(const VectorXd& homogenousvector) const
{
    Vector3d translation;
    translation(0)= homogenousvector(0)/1000.0;
    translation(1)= homogenousvector(1)/1000.0;
    translation(2)= homogenousvector(2)/1000.0;
    Vector3d v1;
    v1(0) = homogenousvector(3);
    v1(1) = homogenousvector(4);
    v1(2) = homogenousvector(5);
    v1.normalize();
    Vector3d v2;
    v2(0) = homogenousvector(6);
    v2(1) = homogenousvector(7);
    v2(2) = homogenousvector(8);
    v2.normalize();
    Vector3d v0 = v1.cross(v2);
    Quaterniond q;

    const double& r11 = v0(0);
    const double& r21 = v0(1);
    const double& r31 = v0(2);

    const double& r12 = v1(0);
    const double& r22 = v1(1);
    const double& r32 = v1(2);

    const double& r13 = v2(0);
    const double& r23 = v2(1);
    const double& r33 = v2(2);

    q.w() = 0.5*sqrt(1+r11+r22+r33);
    q.x() = 0.5*_sign(r32-r23)*sqrt(r11-r22-r33+1);
    q.y() = 0.5*_sign(r13-r31)*sqrt(r22-r33-r11+1);
    q.z() = 0.5*_sign(r21-r12)*sqrt(r33-r11-r22+1);
    q.normalize();
    DQ rot(q.w(),q.x(),q.y(),q.z());
    DQ trans = 1+0.5*E_*(translation(0)*i_+translation(1)*j_+translation(2)*k_);
    DQ pose = trans*rot;
    return pose;
}

DQ RobotDriverDenso::get_end_effector_pose_dq()
{
    const bool worked = bcap_driver_->get_end_effector_pose_homogenous_transformation(end_effector_pose_homogenous_transformation_buffer_);
    if(!worked)
    {
        throw std::runtime_error("FAILED in DensoRobotDriver::get_end_effector_pose_dq(). Error in BCAPDriver::" + bcap_driver_->get_last_error_info());
    }
    Map<VectorXd> homogenousvector(end_effector_pose_homogenous_transformation_buffer_.data(),10);
    return _homogenous_vector_to_dq(homogenousvector);
}

void RobotDriverDenso::set_target_joint_positions(const VectorXd &desired_joint_positions_rad)
{
    if(configuration_.read_only)
        return;

    const VectorXd desired_joint_positions_deg = rad2deg(desired_joint_positions_rad);
    std::vector<double> joint_positions_local_buffer(desired_joint_positions_deg.data(), desired_joint_positions_deg.data() + 6);
    if(!bcap_driver_->set_joint_positions(joint_positions_local_buffer))
    {
        throw std::runtime_error("FAILED in DensoRobotDriver::set_joint_positions(). Error in BCAPDriver::" + bcap_driver_->get_last_error_info());
    }
}


/*VectorXd DensoRobotDriver::set_and_get_joint_positions(const VectorXd &desired_joint_positions_rad)
{
    const VectorXd desired_joint_positions_deg = rad2deg(desired_joint_positions_rad);

    std::vector<double> set_joint_positions_local_buffer(desired_joint_positions_deg.data(), desired_joint_positions_deg.data() + 6);
    const bool worked = bcap_driver_->set_and_get_joint_positions(set_joint_positions_local_buffer, joint_positions_buffer_);
    if(!worked)
    {
        throw std::runtime_error("FAILED in DensoRobotDriver::set_and_get_joint_positions(). Error in BCAPDriver::" + bcap_driver_->get_last_error_info());
    }
    Map<VectorXd> joint_positions(joint_positions_buffer_.data(),6);
    return joint_positions;
}*/

VectorXd RobotDriverDenso::_dq_to_homogenous_vector(const DQ& pose) const
{
    const double& n  = pose.q(0);
    const double& ex = pose.q(1);
    const double& ey = pose.q(2);
    const double& ez = pose.q(3);

    Vector3d v1;
    v1(0) = 2*(ex*ey-n*ez);
    v1(1) = 2*(n*n+ey*ey)-1;
    v1(2) = 2*(ey*ez+n*ex);

    Vector3d v2;
    v2(0) = 2*(ex*ez+n*ey);
    v2(1) = 2*(ey*ez-n*ex);
    v2(2) = 2*(n*n+ez*ez)-1;

    DQ trans = pose.translation();

    VectorXd homogenousvector(10);
    homogenousvector(0) = 1000.0*trans.q(1);
    homogenousvector(1) = 1000.0*trans.q(2);
    homogenousvector(2) = 1000.0*trans.q(3);

    v1 = v1.normalized();
    homogenousvector(3) = v1(0);
    homogenousvector(4) = v1(1);
    homogenousvector(5) = v1(2);

    v2 = v2.normalized();
    homogenousvector(6) = v2(0);
    homogenousvector(7) = v2(1);
    homogenousvector(8) = v2(2);

    //Figure selected automatically
    homogenousvector(9) = -1;

    return homogenousvector;
}

bool RobotDriverDenso::set_end_effector_pose_dq(const DQ& pose)
{
    if(configuration_.read_only)
        return false;

    VectorXd homogenousvector = _dq_to_homogenous_vector(pose);

    std::vector<double> end_effector_pose_local_buffer(homogenousvector.data(), homogenousvector.data() + 10);
    return bcap_driver_->set_end_effector_pose_homogenous_transformation(end_effector_pose_local_buffer);
}

void RobotDriverDenso::connect()
{
    _connect();

    // Monitoring must also work near zero, including while the arm is stationary.
    if(configuration_.read_only)
        return;

    //We need to keep reading from the DENSO robot until we get
    //useful information
    VectorXd local_joint_positions = VectorXd::Zero(6);
    while(local_joint_positions.norm() < 1 && not (*break_loops_))
    {
        local_joint_positions = get_joint_positions();
    }
}

void RobotDriverDenso::deinitialize()
{
    if(configuration_.read_only)
        return;

    //10 ms clock
    sas::Clock clock(0.01);
    clock.init();
    _slave_mode_off();
    clock.blocking_sleep_seconds(3.);
    _set_speed(1.,1.,1.);
    clock.blocking_sleep_seconds(3.);
    _motor_off();
    clock.blocking_sleep_seconds(3.);
}

void RobotDriverDenso::initialize()
{
    if(configuration_.read_only)
        return;

    //10 ms clock
    sas::Clock clock(0.01);
    clock.init();
    _motor_on();
    clock.safe_sleep_seconds(6., break_loops_);
    _set_speed(configuration_.speed, configuration_.speed, configuration_.speed);
    clock.safe_sleep_seconds(6., break_loops_);
    _slave_mode_on(sas::RobotDriverDenso::SLAVE_MODE_JOINT_CONTROL);
    clock.safe_sleep_seconds(6., break_loops_);
}

void RobotDriverDenso::_connect()
{
    bool worked; //bCap communication error code

    worked = bcap_driver_->open();
    if(!worked)
    {
        throw std::runtime_error("  FAILED TO Open() IN FUNCTION Connect(). Error code = " + bcap_driver_->get_last_error_info());
    }

    worked = bcap_driver_->service_start();
    if(!worked)
    {
        throw std::runtime_error("  FAILED TO ServiceStart() IN FUNCTION Connect(). Error code = " + bcap_driver_->get_last_error_info());
    }

    worked = bcap_driver_->controller_connect();
    if(!worked)
    {
        throw std::runtime_error("  FAILED TO ControllerConnect() IN FUNCTION Connect(). Error code = " + bcap_driver_->get_last_error_info());
    }

    bcap_driver_->initialize_controller_variable_handles();

    worked = bcap_driver_->get_robot();
    if(!worked)
    {
        throw std::runtime_error("  FAILED TO GetRobot() IN FUNCTION Connect(). Error code = " + bcap_driver_->get_last_error_info());
    }

    if(!configuration_.read_only)
    {
        worked = bcap_driver_->take_arm();
        if(!worked)
        {
            throw std::runtime_error("  FAILED TO TakeArm() IN FUNCTION Connect(). Error code = " + bcap_driver_->get_last_error_info());
        }
    }
}

void RobotDriverDenso::_motor_on()
{
    bool worked; //bCap communication error code

    worked = bcap_driver_->set_motor_state(true);
    if(!worked)
    {
        throw std::runtime_error("  FAILED TO SetMotorState() IN FUNCTION MotorOn(). Error code = " + bcap_driver_->get_last_error_info());
    }
}

void RobotDriverDenso::_set_speed(const float& speed, const float& acceleration, const float& deacceleration)
{
    bool worked; //bCap communication error code

    worked = bcap_driver_->set_speed(speed,acceleration,deacceleration);
    if(!worked)
    {
        throw std::runtime_error("  FAILED TO SetSpeed() IN FUNCTION SetSpeed(). Error code = " + bcap_driver_->get_last_error_info());
    }
}

void RobotDriverDenso::_slave_mode_on(int mode)
{
    bool worked; //bCap communication error code

    worked = bcap_driver_->set_slave_mode(mode);
    if(!worked)
    {
        throw std::runtime_error("  FAILED TO SlaveModeOn() IN FUNCTION SlaveModeOn(). Error code = " + bcap_driver_->get_last_error_info());
    }
}

void RobotDriverDenso::_motor_off() noexcept
{
    bcap_driver_->set_motor_state(false);
}

void RobotDriverDenso::_slave_mode_off() noexcept
{
    bcap_driver_->set_slave_mode(0);
}

void RobotDriverDenso::disconnect()
{
    if(!configuration_.read_only)
        bcap_driver_->give_arm();
    bcap_driver_->release_robot();
    bcap_driver_->controller_disconnect();
    bcap_driver_->service_stop();
    bcap_driver_->close();
}

RobotDriverDenso::~RobotDriverDenso()=default;

}


