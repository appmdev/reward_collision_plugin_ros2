#include "collision_reward/collision_reward_plugin.hpp"
#include <gazebo/physics/physics.hh>

namespace gazebo
{
  CollisionRewardPlugin::CollisionRewardPlugin()
  : impl_(std::make_unique<CollisionRewardPluginPrivate>())
  {
    printf("Initiated Collision Reward Plugin !\n");
  }

  CollisionRewardPlugin::~CollisionRewardPlugin()
  {
    // Cleanup if needed
  }

  void CollisionRewardPlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
  {
    impl_->model_name_ = model->GetName();
    if (sdf->HasElement("target_collision"))
    {
      impl_->target_collision_ = sdf->Get<std::string>("target_collision");
    }
    else
    {
      RCLCPP_ERROR(impl_->ros_node_->get_logger(), "target_collision parameter is required.");
      return;
    }

    if (sdf->HasElement("reward_value"))
    {
      impl_->reward_value_ = sdf->Get<double>("reward_value");
    }
    else
    {
      RCLCPP_WARN(impl_->ros_node_->get_logger(), "reward_value parameter not found. Using default value of 1.0.");
      impl_->reward_value_ = 1.0;
    }

    impl_->ros_node_ = gazebo_ros::Node::Get(sdf);

    impl_->reward_publisher_ = impl_->ros_node_->create_publisher<std_msgs::msg::String>("/reward", 10);

    impl_->world_ = model->GetWorld();
    
    impl_->contact_manager_ = impl_->world_->Physics()->GetContactManager();

    impl_->gz_node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
    impl_->gz_node_->Init();

    // Subscribe to the contact topic
    impl_->contact_sub_ = impl_->gz_node_->Subscribe("~/physics/contacts", &CollisionRewardPlugin::OnContacts, this);

    // Initialize last process time to current time
    impl_->last_process_time_ = std::chrono::steady_clock::now();

    // Set up a timer to check contacts at 1 Hz
    impl_->timer_ = impl_->ros_node_->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&CollisionRewardPlugin::CheckContacts, this));
  }

  void CollisionRewardPlugin::OnContacts(ConstContactsPtr &contacts)
  {
    auto now = std::chrono::steady_clock::now();
    if (now - impl_->last_process_time_ < impl_->process_interval_)
    {
      // Ignore this contact message because it's too soon since the last processed one
      return;
    }

    bool specific_collision_detected = false;
    std::string collision_object_name;
    for (int i = 0; i < contacts->contact_size(); ++i)
    {
      const auto &contact = contacts->contact(i);
      if ((contact.collision1().find(impl_->model_name_) != std::string::npos && 
           contact.collision2().find(impl_->target_collision_) != std::string::npos) ||
          (contact.collision2().find(impl_->model_name_) != std::string::npos && 
           contact.collision1().find(impl_->target_collision_) != std::string::npos))
      {
        specific_collision_detected = true;
        collision_object_name = (contact.collision1().find(impl_->model_name_) != std::string::npos) ? contact.collision2() : contact.collision1();
        break;
      }
    }
    impl_->specific_collision_detected_.store(specific_collision_detected, std::memory_order_relaxed);
    impl_->collision_object_name_ = collision_object_name;

    // Update the last process time
    impl_->last_process_time_ = now;
  }

  void CollisionRewardPlugin::CheckContacts()
  {
    bool specific_collision_detected = impl_->specific_collision_detected_.load(std::memory_order_relaxed);
    if (specific_collision_detected)
    {
      auto msg = std::make_shared<std_msgs::msg::String>();
      //msg->data = "Reward: " + std::to_string(impl_->reward_value_) + "; Collided: " + impl_->collision_object_name_;
      msg->data = "Reward: " + std::to_string(impl_->reward_value_) + "; Collided: " + impl_->model_name_;
      impl_->reward_publisher_->publish(*msg);
      RCLCPP_INFO(impl_->ros_node_->get_logger(), "Publishing reward: %s", msg->data.c_str());
    }
  }

  GZ_REGISTER_MODEL_PLUGIN(CollisionRewardPlugin)
}  // namespace gazebo
