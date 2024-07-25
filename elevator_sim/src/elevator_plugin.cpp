#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

namespace gazebo
{
  class ElevatorPlugin : public ModelPlugin
  {
    public: ElevatorPlugin() : ModelPlugin() 
    {
      gzmsg << "ElevatorPlugin constructor called" << std::endl;
    }

    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
      gzmsg << "ElevatorPlugin Load function called" << std::endl;
      
      // Store the pointer to the model
      this->model = _parent;

      // Listen to the update event
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&ElevatorPlugin::OnUpdate, this));

      // Initialize ROS node
      if (!rclcpp::ok()) {
        rclcpp::init(0, nullptr);
      }
      this->node = std::make_shared<rclcpp::Node>("elevator_plugin");

      // Create a subscriber
      this->sub = this->node->create_subscription<std_msgs::msg::Int32>(
        "elevator_command", 10,
        std::bind(&ElevatorPlugin::CommandCallback, this, std::placeholders::_1));

      // Initialize positions
      this->currentPosition = this->model->WorldPose().Pos().Z();
      this->targetPosition = this->currentPosition;

      gzmsg << "ElevatorPlugin loaded successfully" << std::endl;
    }

    public: void OnUpdate()
    {
      // Move the model towards the target position
      ignition::math::Pose3d pose = this->model->WorldPose();
      double currentZ = pose.Pos().Z();
      double diff = this->targetPosition - currentZ;

      if (std::abs(diff) > 0.01)  // If not close enough to target
      {
        double newZ = currentZ + std::copysign(std::min(0.01, std::abs(diff)), diff);
        this->model->SetWorldPose(ignition::math::Pose3d(pose.Pos().X(), pose.Pos().Y(), newZ, 
                                  pose.Rot().Roll(), pose.Rot().Pitch(), pose.Rot().Yaw()));
        gzmsg << "Moving elevator. Current Z: " << currentZ << ", Target Z: " << this->targetPosition << std::endl;
      }
    }

    private: void CommandCallback(const std_msgs::msg::Int32::SharedPtr msg)
    {
      // Move the elevator by the specified number of units
      this->targetPosition = this->currentPosition + static_cast<double>(msg->data);
      this->currentPosition = this->targetPosition;
      gzmsg << "Received command. New target position: " << this->targetPosition << std::endl;
    }

    private: physics::ModelPtr model;
    private: event::ConnectionPtr updateConnection;
    private: std::shared_ptr<rclcpp::Node> node;
    private: rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub;
    private: double currentPosition;
    private: double targetPosition;
  };

  GZ_REGISTER_MODEL_PLUGIN(ElevatorPlugin)
}