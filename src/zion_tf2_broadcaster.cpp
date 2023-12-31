#include "stair_modeling_ros/zion_tf2_broadcaster.h"
#include "stair_modeling_ros/utilities.h"


ZionBroadcaster::ZionBroadcaster() : Node("ZionBroadcaster")
{ 

    loadParams();

    // Initialize the transform broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    rclcpp::QoS qos_profile(1000);
    // qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    tf_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>( "/tf", qos_profile,
        std::bind(&ZionBroadcaster::tfCallback, this, std::placeholders::_1));

    tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  }


void ZionBroadcaster::loadParams()
{
    // Frame IDs
    this->declare_parameter("frame_ids.map_frame", "map");
    this->declare_parameter("frame_ids.base_frame", "zedm_base_link");
    this->declare_parameter("frame_ids.input_cloud_frame", "zedm_left_camera_frame");
    this->declare_parameter("frame_ids.output_cloud_frame", "zedm_base_link_projected");
    this->get_parameter("frame_ids.map_frame", map_frame_);
    this->get_parameter("frame_ids.base_frame", base_frame_);
    this->get_parameter("frame_ids.output_cloud_frame", output_frame_);
}


void ZionBroadcaster::tfCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
{
    (void)msg;
    std::string fromFrameRel = base_frame_;
    std::string toFrameRel = map_frame_;
    geometry_msgs::msg::TransformStamped base2map;
    geometry_msgs::msg::TransformStamped base_projected2map;
    geometry_msgs::msg::TransformStamped base_projected2camera;

    if (tf_buffer_->canTransform(fromFrameRel, toFrameRel, tf2::TimePointZero))
    {
        try {
          base2map = tf_buffer_->lookupTransform(
            toFrameRel, fromFrameRel,
            tf2::TimePointZero);

          // Extract the rotation from existing transform
          tf2::Transform tf2_transform;
          tf2::fromMsg(base2map.transform, tf2_transform);
          tf2::Vector3 translation(tf2_transform.getOrigin());
          tf2::Matrix3x3 rotation_matrix(tf2_transform.getRotation());
          double roll, pitch, yaw;
          rotation_matrix.getRPY(roll, pitch, yaw);
      
          // Create a new transform to broadcast
          base_projected2map.header.stamp = this->get_clock()->now();
          base_projected2map.header.frame_id = toFrameRel;

          base_projected2map.child_frame_id = output_frame_;

          // Set rotation 
          tf2::Quaternion new_rotation;
          new_rotation.setRPY(0, 0, yaw);

          base_projected2map.transform.translation = base2map.transform.translation;

          base_projected2map.transform.rotation = tf2::toMsg(new_rotation);

          tf_broadcaster_->sendTransform(base_projected2map);
          
        }
        catch (const tf2::TransformException & ex) {
          RCLCPP_INFO(
            this->get_logger(), "Could not transform %s to %s: %s",
            fromFrameRel.c_str(), toFrameRel.c_str(), ex.what());
          return;
        }
    }
}



