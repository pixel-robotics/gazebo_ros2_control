// Copyright 2021 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <map>
#include <memory>
#include <string>
#include <vector>
#include <utility>

#include "gazebo_ros2_control/gazebo_system.hpp"
#include "gazebo/common/common.hh"

#include "hardware_interface/types/hardware_interface_type_values.hpp"

class gazebo_ros2_control::GazeboSystemPrivate
{
public:
  GazeboSystemPrivate() = default;

  ~GazeboSystemPrivate() = default;

  /// \brief Degrees od freedom.
  size_t n_dof_;

  /// \brief Gazebo Model Ptr.
  gazebo::physics::ModelPtr parent_model_;

  /// \brief last time the write method was called.
  rclcpp::Time last_update_sim_time_ros_;

  /// \brief vector with the joint's names.
  std::vector<std::string> joint_names_;

  /// \brief vector with the control method defined in the URDF for each joint.
  std::vector<GazeboSystemInterface::ControlMethod> joint_control_methods_;

  /// \brief handles to the joints from within Gazebo
  std::vector<gazebo::physics::JointPtr> sim_joints_;

  /// \brief vector with the current joint position
  std::vector<double> joint_position_;

  /// \brief vector with the current joint velocity
  std::vector<double> joint_velocity_;

  /// \brief vector with the current cmd joint position
  std::vector<double> joint_position_cmd_;

  /// \brief vector with the current cmd joint velocity
  std::vector<double> joint_velocity_cmd_;

  /// \brief state interfaces that will be exported to the Resource Manager
  std::vector<hardware_interface::StateInterface> state_interfaces_;

  /// \brief command interfaces that will be exported to the Resource Manager
  std::vector<hardware_interface::CommandInterface> command_interfaces_;
};

namespace gazebo_ros2_control
{

bool GazeboSystem::initSim(
  rclcpp::Node::SharedPtr & model_nh,
  gazebo::physics::ModelPtr parent_model,
  const hardware_interface::HardwareInfo & hardware_info,
  sdf::ElementPtr sdf)
{
  this->dataPtr = std::make_unique<GazeboSystemPrivate>();
  this->dataPtr->last_update_sim_time_ros_ = rclcpp::Time();

  this->nh_ = model_nh;
  this->dataPtr->parent_model_ = parent_model;
  gazebo::physics::PhysicsEnginePtr physics = gazebo::physics::get_world()->Physics();

  std::string physics_type_ = physics->GetType();
  if (physics_type_.empty()) {
    RCLCPP_ERROR(this->nh_->get_logger(), "No physics engine configured in Gazebo.");
    return false;
  }

  registerJoints(hardware_info, parent_model);

  if (this->dataPtr->n_dof_ == 0) {
    RCLCPP_WARN_STREAM(this->nh_->get_logger(), "There is no joint or sensor available");
    return false;
  }

  joint_controller_ = this->dataPtr->parent_model_->GetJointController();
  joint_controller_->Reset();
  steering_joint_ = this->dataPtr->parent_model_->GetJoint(hardware_info.joints[1].name);
  traction_joint_ = this->dataPtr->parent_model_->GetJoint(hardware_info.joints[2].name);
  joint_controller_->AddJoint(steering_joint_);
  joint_controller_->AddJoint(traction_joint_);

  double steering_P = std::stod(hardware_info.joints[1].parameters.at("P"));
  double steering_I = std::stod(hardware_info.joints[1].parameters.at("I"));
  double steering_D = std::stod(hardware_info.joints[1].parameters.at("D"));
  double traction_P = std::stod(hardware_info.joints[2].parameters.at("P"));
  double traction_I = std::stod(hardware_info.joints[2].parameters.at("I"));
  double traction_D = std::stod(hardware_info.joints[2].parameters.at("D"));
  joint_controller_->SetVelocityPID(steering_joint_->GetScopedName(), gazebo::common::PID(traction_P, traction_I, traction_D));
  joint_controller_->SetPositionPID(traction_joint_->GetScopedName(), gazebo::common::PID(steering_P, steering_I, steering_D));

  return true;
}

void GazeboSystem::registerJoints(
  const hardware_interface::HardwareInfo & hardware_info,
  gazebo::physics::ModelPtr parent_model)
{
  this->dataPtr->n_dof_ = hardware_info.joints.size();

  this->dataPtr->joint_names_.resize(this->dataPtr->n_dof_);
  this->dataPtr->joint_control_methods_.resize(this->dataPtr->n_dof_);
  this->dataPtr->joint_position_.resize(this->dataPtr->n_dof_);
  this->dataPtr->joint_velocity_.resize(this->dataPtr->n_dof_);
  this->dataPtr->joint_position_cmd_.resize(this->dataPtr->n_dof_);
  this->dataPtr->joint_velocity_cmd_.resize(this->dataPtr->n_dof_);

  for (unsigned int j = 0; j < this->dataPtr->n_dof_; j++) {
    std::string joint_name = this->dataPtr->joint_names_[j] = hardware_info.joints[j].name;

    gazebo::physics::JointPtr simjoint = parent_model->GetJoint(joint_name);
    this->dataPtr->sim_joints_.push_back(simjoint);
    if (!simjoint) {
      RCLCPP_WARN_STREAM(
        this->nh_->get_logger(), "Skipping joint in the URDF named '" << joint_name <<
          "' which is not in the gazebo model.");
      continue;
    }

    // Accept this joint and continue configuration
    RCLCPP_INFO_STREAM(this->nh_->get_logger(), "Loading joint: " << joint_name);

    RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\tCommand:");

    // register the command handles
    for (unsigned int i = 0; i < hardware_info.joints[j].command_interfaces.size(); i++) {
      if (hardware_info.joints[j].command_interfaces[i].name == "position") {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t position");
        this->dataPtr->joint_control_methods_[j] |= POSITION;
        this->dataPtr->command_interfaces_.emplace_back(
          joint_name,
          hardware_interface::HW_IF_POSITION,
          &this->dataPtr->joint_position_cmd_[j]);
      }
      if (hardware_info.joints[j].command_interfaces[i].name == "velocity") {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t velocity");
        this->dataPtr->joint_control_methods_[j] |= VELOCITY;
        this->dataPtr->command_interfaces_.emplace_back(
          joint_name,
          hardware_interface::HW_IF_VELOCITY,
          &this->dataPtr->joint_velocity_cmd_[j]);
      }
    }

    RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\tState:");

    // register the state handles
    for (unsigned int i = 0; i < hardware_info.joints[j].state_interfaces.size(); i++) {
      if (hardware_info.joints[j].state_interfaces[i].name == "position") {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t position");
        this->dataPtr->state_interfaces_.emplace_back(
          joint_name,
          hardware_interface::HW_IF_POSITION,
          &this->dataPtr->joint_position_[j]);
      }
      if (hardware_info.joints[j].state_interfaces[i].name == "velocity") {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t velocity");
        this->dataPtr->state_interfaces_.emplace_back(
          joint_name,
          hardware_interface::HW_IF_VELOCITY,
          &this->dataPtr->joint_velocity_[j]);
      }
    }
  }
}

CallbackReturn
GazeboSystem::on_init(const hardware_interface::HardwareInfo & actuator_info)
{
  if (hardware_interface::SystemInterface::on_init(actuator_info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
GazeboSystem::export_state_interfaces()
{
  return std::move(this->dataPtr->state_interfaces_);
}

std::vector<hardware_interface::CommandInterface>
GazeboSystem::export_command_interfaces()
{
  return std::move(this->dataPtr->command_interfaces_);
}

CallbackReturn GazeboSystem::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  return CallbackReturn::SUCCESS;
}

CallbackReturn GazeboSystem::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type GazeboSystem::read()
{
  for (unsigned int j = 0; j < this->dataPtr->joint_names_.size(); j++) {
    if (this->dataPtr->sim_joints_[j]) {
      this->dataPtr->joint_position_[j] = this->dataPtr->sim_joints_[j]->Position(0);
      this->dataPtr->joint_velocity_[j] = this->dataPtr->sim_joints_[j]->GetVelocity(0);
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type GazeboSystem::write()
{
  // Get the simulation time and period
  gazebo::common::Time gz_time_now = this->dataPtr->parent_model_->GetWorld()->SimTime();
  rclcpp::Time sim_time_ros(gz_time_now.sec, gz_time_now.nsec);
  rclcpp::Duration sim_period = sim_time_ros - this->dataPtr->last_update_sim_time_ros_;

  for (unsigned int j = 0; j < this->dataPtr->joint_names_.size(); j++) {
    if (this->dataPtr->sim_joints_[j]) {
      if (this->dataPtr->joint_control_methods_[j] & POSITION) {
        joint_controller_->SetPositionTarget(steering_joint_->GetScopedName(), this->dataPtr->joint_position_cmd_[j]);
      } 
      if (this->dataPtr->joint_control_methods_[j] & VELOCITY) {
        joint_controller_->SetVelocityTarget(traction_joint_->GetScopedName(), this->dataPtr->joint_velocity_cmd_[j]);
      }
    }
  }
  joint_controller_->Update();
  this->dataPtr->last_update_sim_time_ros_ = sim_time_ros;

  return hardware_interface::return_type::OK;
}
}  // namespace gazebo_ros2_control

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(
  gazebo_ros2_control::GazeboSystem, gazebo_ros2_control::GazeboSystemInterface)
