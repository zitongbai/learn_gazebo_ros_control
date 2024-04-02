/**
 * @file rrbot_hw_sim.cpp
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2024-04-01
 * 
 * @copyright Copyright (c) 2024
 * 
 */


#include "rrbot_gazebo/rrbot_hw_sim.h"
#include <gazebo_ros_control/gazebo_ros_control_plugin.h>

bool RRBotHWSim::initSim(const std::string& robot_namespace,
                 ros::NodeHandle model_nh,
                 gazebo::physics::ModelPtr parent_model,
                 const urdf::Model *const urdf_model,
                 std::vector<transmission_interface::TransmissionInfo> transmissions){
    bool ret = DefaultRobotHWSim::initSim(robot_namespace, model_nh, parent_model, urdf_model, transmissions);

    // IMU interface
    hardware_interface::ImuSensorHandle::Data imu_set_data;
    imu_set_data.name = "imu_sensor";
    imu_set_data.frame_id = "imu";
    imu_set_data.orientation = &imu_data_.orientation[0];
    imu_set_data.orientation_covariance = &imu_data_.orientation_covariance[0];
    imu_set_data.angular_velocity = &imu_data_.angular_velocity[0];
    imu_set_data.angular_velocity_covariance = &imu_data_.angular_velocity_covariance[0];
    imu_set_data.linear_acceleration = &imu_data_.linear_acceleration[0];
    imu_set_data.linear_acceleration_covariance = &imu_data_.linear_acceleration_covariance[0];
    imu_sensor_interface_.registerHandle(hardware_interface::ImuSensorHandle(imu_set_data));
    registerInterface(&imu_sensor_interface_);

    imu_data_.link_ptr = parent_model->GetLink("imu");

    return ret;
}

void RRBotHWSim::readSim(ros::Time time, ros::Duration period){
    // read joint data from gazebo 
    // these lines are copied from default_robot_hw_sim.cpp
    // ref: https://github.com/ros-simulation/gazebo_ros_pkgs/blob/f9e1a4607842afa5888ef01de31cd64a1e3e297f/gazebo_ros_control/src/default_robot_hw_sim.cpp#L261-L283
    for(unsigned int j=0; j < n_dof_; j++){
        double position = sim_joints_[j]->Position(0);
        if (joint_types_[j] == urdf::Joint::PRISMATIC){
        joint_position_[j] = position;
        }
        else{
        joint_position_[j] += angles::shortest_angular_distance(joint_position_[j], position);
        }
        joint_velocity_[j] = sim_joints_[j]->GetVelocity(0);
        joint_effort_[j] = sim_joints_[j]->GetForce((unsigned int)(0));
    }

    // read imu data
    ignition::math::Pose3d imu_pose = imu_data_.link_ptr->WorldPose();
    imu_data_.orientation[0] = imu_pose.Rot().X();
    imu_data_.orientation[1] = imu_pose.Rot().Y();
    imu_data_.orientation[2] = imu_pose.Rot().Z();
    imu_data_.orientation[3] = imu_pose.Rot().W(); // in ROS, 'w' in quaternion is often last. 
    ignition::math::Vector3d imu_pose_dot = imu_data_.link_ptr->WorldAngularVel();
    imu_data_.angular_velocity[0] = imu_pose_dot.X();
    imu_data_.angular_velocity[1] = imu_pose_dot.Y();
    imu_data_.angular_velocity[2] = imu_pose_dot.Z();
    ignition::math::Vector3d gravity = {0.0, 0.0, -9.81};
    ignition::math::Vector3d imu_accel = imu_data_.link_ptr->WorldLinearAccel() - imu_pose.Rot().RotateVectorReverse(gravity);
    imu_data_.linear_acceleration[0] = imu_accel.X();
    imu_data_.linear_acceleration[1] = imu_accel.Y();
    imu_data_.linear_acceleration[2] = imu_accel.Z();

}


void RRBotHWSim::writeSim(ros::Time time, ros::Duration period){
    DefaultRobotHWSim::writeSim(time, period);
}

PLUGINLIB_EXPORT_CLASS(RRBotHWSim, gazebo_ros_control::RobotHWSim)
GZ_REGISTER_MODEL_PLUGIN(gazebo_ros_control::GazeboRosControlPlugin)  // Default plugin


