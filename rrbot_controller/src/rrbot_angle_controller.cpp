/**
 * @file rrbot_angle_controller.cpp
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2024-04-03
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "rrbot_controller/rrbot_angle_controller.h"
#include <Eigen/Geometry>

bool RRBotAngleController::init(hardware_interface::RobotHW * robot_hw, ros::NodeHandle & controller_nh){

    auto * eff_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
    std::vector<std::string> joint_names{"joint1", "joint2"};
    for(const auto & joint_name : joint_names){
        joint_handles_.push_back(eff_joint_interface->getHandle(joint_name));
    }

    imu_handle_ = robot_hw->get<hardware_interface::ImuSensorInterface>()->getHandle("imu_sensor"); // imu_sensor corresponds to the name in rrbot_hw_sim.cpp

    return true;

}

void RRBotAngleController::starting(const ros::Time & time){
    ROS_INFO("Starting RRBotAngleController");
}

void RRBotAngleController::stopping(const ros::Time & time){
    ROS_INFO("Stopping RRBotAngleController");
}

void RRBotAngleController::update(const ros::Time & time, const ros::Duration & period){
    // read imu data
    auto imu_data = imu_handle_.getOrientation();
    Eigen::Quaterniond imu_quat;
    for(size_t i = 0; i<4; i++){
        imu_quat.coeffs()(i) = imu_data[i];
    }

    // convert quaternion to euler angles
    Eigen::Vector3d euler_angles = imu_quat.toRotationMatrix().eulerAngles(2, 1, 0);

    double target_angle = 0.5;
    double error = target_angle - euler_angles(0);
    
    double gain = 1.0;

    joint_handles_[0].setCommand(gain * error);
    joint_handles_[1].setCommand(gain * error);
}


PLUGINLIB_EXPORT_CLASS(RRBotAngleController, controller_interface::ControllerBase)