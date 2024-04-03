/**
 * @file rrbot_angle_controller.h
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2024-04-03
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#pragma once

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <pluginlib/class_list_macros.h>

#include <control_toolbox/pid.h>

class RRBotAngleController : public controller_interface::MultiInterfaceController<hardware_interface::EffortJointInterface, 
                                                                                hardware_interface::ImuSensorInterface>
{
public:
    bool init(hardware_interface::RobotHW * robot_hw, ros::NodeHandle & controller_nh) override;
    void update(const ros::Time & time, const ros::Duration & period) override;
    void starting(const ros::Time & time) override;
    void stopping(const ros::Time & time) override;

private:
    std::vector<hardware_interface::JointHandle> joint_handles_;
    hardware_interface::ImuSensorHandle imu_handle_;

    control_toolbox::Pid pid_controller_;
};


