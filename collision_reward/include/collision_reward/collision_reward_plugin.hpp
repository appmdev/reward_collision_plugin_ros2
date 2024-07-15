#ifndef COLLISION_REWARD_PLUGIN_HPP_
#define COLLISION_REWARD_PLUGIN_HPP_

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/ContactManager.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <gazebo/transport/Node.hh>
#include <gazebo/transport/Subscriber.hh>
#include <gazebo/msgs/contacts.pb.h>

#include <memory>
#include <string>
#include <mutex>
#include <atomic>
#include <chrono>

namespace gazebo
{
  class CollisionRewardPluginPrivate;

  class CollisionRewardPlugin : public gazebo::ModelPlugin
  {
  public:
    CollisionRewardPlugin();
    virtual ~CollisionRewardPlugin();
    void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;

  private:
    void OnContacts(ConstContactsPtr &contacts);
    void CheckContacts();
    std::unique_ptr<CollisionRewardPluginPrivate> impl_;
  };

  class CollisionRewardPluginPrivate
  {
  public:
    gazebo_ros::Node::SharedPtr ros_node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr reward_publisher_;
    gazebo::physics::ContactManager *contact_manager_;
    gazebo::physics::WorldPtr world_;
    gazebo::transport::NodePtr gz_node_;
    gazebo::transport::SubscriberPtr contact_sub_;
    std::string model_name_;
    std::string target_collision_;
    std::string collision_object_name_;
    double reward_value_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::atomic<bool> specific_collision_detected_{false};
    std::mutex mutex_;
    std::chrono::steady_clock::time_point last_process_time_;
    std::chrono::milliseconds process_interval_{1000};  // Process contacts every 1000ms (1 Hz)
  };

} // namespace gazebo

#endif  // COLLISION_REWARD_PLUGIN_HPP_
