/**
 * @file rrbot_hw_sim.h
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2024-04-01
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#pragma once

#include <gazebo_ros_control/default_robot_hw_sim.h>

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/imu_sensor_interface.h>

struct ImuData{
    gazebo::physics::LinkPtr link_ptr;
    double orientation[4];
    double orientation_covariance[9];
    double angular_velocity[3];
    double angular_velocity_covariance[9];
    double linear_acceleration[3];
    double linear_acceleration_covariance[9];
};



/**
 * @brief DefaultRobotHWSim这个类本身已经实现了HW，但是它考虑通用性，代码写的比较复杂
 *  我们继承DefaultRobotHWSim，然后简单地重写几个函数
 *  一方面是为了学习和理解
 *  另一方面也是可以加入一些额外的功能
 *  此外，继承DefaultRobotHWSim，我们可以直接使用DefaultRobotHWSim中的一些成员变量和函数
 * 
 * @ref https://github.com/ros-simulation/gazebo_ros_pkgs/blob/noetic-devel/gazebo_ros_control/src/default_robot_hw_sim.cpp
 */
class RRBotHWSim : public gazebo_ros_control::DefaultRobotHWSim
{
public:
    /**
     * @brief 重写initSim函数，初始化HW，只初始化我们需要的接口
     * 
     * @param robot_namespace 
     * @param model_nh 
     * @param parent_model 
     * @param urdf_model 
     * @param transmissions 
     * @return true 
     * @return false 
     */
    bool initSim(const std::string& robot_namespace,
                 ros::NodeHandle model_nh,
                 gazebo::physics::ModelPtr parent_model,
                 const urdf::Model *const urdf_model,
                 std::vector<transmission_interface::TransmissionInfo> transmissions) override;

    /**
     * @brief 重写readSim函数，读取我们需要的、来自gazebo仿真环境的数据，并且做数据处理
     * 
     * @param time 
     * @param period 
     */
    void readSim(ros::Time time, ros::Duration period) override;

    /**
     * @brief 重写writeSim函数，将我们的控制数据写入到gazebo仿真环境中
     * 
     * @param time 
     * @param period 
     */
    void writeSim(ros::Time time, ros::Duration period) override;

private:
    
    hardware_interface::ImuSensorInterface imu_sensor_interface_;

    ImuData imu_data_;

};


