#include "stair_modeling_ros/zion_tf2_broadcaster.h"
#include "stair_modeling_ros/utilities.h"


ZionBroadcaster::ZionBroadcaster() : Node("ZionBroadcaster")
{ 
    rclcpp::QoS qos_profile(1000);
    // qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    subscription_ = this->create_subscription<tf2_msgs::msg::TFMessage>( "/tf", qos_profile,
        std::bind(&ZionBroadcaster::tfCallback, this, std::placeholders::_1));

    // Initialize the tf instances
    tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // subscribers and publishers
    rclcpp::QoS qos_profile_pcl(1);
    // qos_profile_pcl.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    qos_profile_pcl.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    pcl_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/zedm/zed_node/point_cloud/cloud_registered", qos_profile_pcl,
        std::bind(&ZionBroadcaster::pclCallback, this, std::placeholders::_1));

    pcl_pub_= this->create_publisher<sensor_msgs::msg::PointCloud2>("/stair_modeling_ros/point_cloud/cloud_projected",10);
    pcl_buffer_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
  }

void ZionBroadcaster::tfCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
{
    (void)msg;
    std::string fromFrameRel = "zedm_base_link";
    std::string toFrameRel = "map";
    geometry_msgs::msg::TransformStamped transformStamped;

    geometry_msgs::msg::TransformStamped new_transformStamped;
    if (tf_buffer_->canTransform(fromFrameRel, toFrameRel, tf2::TimePointZero))
    {
        try {
          transformStamped = tf_buffer_->lookupTransform(
            toFrameRel, fromFrameRel,
            tf2::TimePointZero);

          // Extract the rotation from existing transform
          tf2::Transform tf2_transform;
          tf2::fromMsg(transformStamped.transform, tf2_transform);
          tf2::Vector3 translation(tf2_transform.getOrigin());
          tf2::Matrix3x3 rotation_matrix(tf2_transform.getRotation());
          double roll, pitch, yaw;
          rotation_matrix.getRPY(roll, pitch, yaw);
      
          // Create a new transform to broadcast
          new_transformStamped.header.stamp = this->get_clock()->now();
          new_transformStamped.header.frame_id = "map";
          new_transformStamped.child_frame_id = "zedm_base_link_projected";

          // Set rotation 
          tf2::Quaternion new_rotation;
          new_rotation.setRPY(0, 0, yaw);

          new_transformStamped.transform.translation = transformStamped.transform.translation;

          new_transformStamped.transform.rotation = tf2::toMsg(new_rotation);

          tf_broadcaster_->sendTransform(new_transformStamped);
        } 
        catch (const tf2::TransformException & ex) {
          RCLCPP_INFO(
            this->get_logger(), "Could not transform %s to %s: %s",
            toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
          return;
        }
    }
}


void ZionBroadcaster::pclCallback(const sensor_msgs::msg::PointCloud2::SharedPtr pcl_msg)
{
    std::string fromFrameRel = "zedm_base_link";
    std::string toFrameRel = "map";
    geometry_msgs::msg::TransformStamped transformStamped;
    geometry_msgs::msg::TransformStamped new_transformStamped;
    if (tf_buffer_->canTransform(fromFrameRel, toFrameRel, tf2::TimePointZero))
    {
        try {
          transformStamped = tf_buffer_->lookupTransform(
            toFrameRel, fromFrameRel,
            tf2::TimePointZero);

          // Extract the rotation from existing transform
          tf2::Transform tf2_transform;
          tf2::fromMsg(transformStamped.transform, tf2_transform);
          tf2::Vector3 translation(tf2_transform.getOrigin());
          tf2::Matrix3x3 rotation_matrix(tf2_transform.getRotation());
          double roll, pitch, yaw;
          rotation_matrix.getRPY(roll, pitch, yaw);
      
          // Create a new transform to broadcast
          new_transformStamped.header.stamp = this->get_clock()->now();
          new_transformStamped.header.frame_id = fromFrameRel;
          // new_transformStamped.child_frame_id = "zedm_left_camera_frame_projected";
          new_transformStamped.child_frame_id = "zedm_base_link_projected";

          // Set rotation (negating roll and pitch)
          tf2::Quaternion new_rotation;
          // new_rotation.setRPY(-roll, -pitch, 0);
          new_rotation.setRPY(0, 0, yaw);

          new_transformStamped.transform.translation = transformStamped.transform.translation;
          //new_transformStamped.transform.translation = tf2::toMsg(translation);

          new_transformStamped.transform.rotation = tf2::toMsg(new_rotation);

          tf_broadcaster_->sendTransform(new_transformStamped);

          // tf2::doTransform(*pcl_msg,*pcl_buffer_,new_transformStamped);

          c2cp = tf2::transformToEigen(new_transformStamped);

          pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud (new  pcl::PointCloud<pcl::PointXYZRGB>);
          pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud (new  pcl::PointCloud<pcl::PointXYZRGB>);

          // from ros msg 
          pcl::fromROSMsg(*pcl_msg, *input_cloud);
          Utilities::voxelizingDownsample(input_cloud, input_cloud,0.025,0.08);
          Utilities::transformCloud(c2cp,input_cloud,output_cloud);

          pcl::toROSMsg(*output_cloud, *pcl_buffer_);

          pcl_pub_->publish(*pcl_buffer_);
        } 
        catch (const tf2::TransformException & ex) {
          RCLCPP_INFO(
            this->get_logger(), "Could not transform %s to %s: %s",
            toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
          return;
        }
    }
}


