#include "stair_modeling_ros/zion_tf2_broadcaster.h"
#include "stair_modeling_ros/utilities.h"


ZionBroadcaster::ZionBroadcaster() : Node("ZionBroadcaster")
{ 

    loadParams();

    // Initialize the tf instances
    tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // subscribers and publishers
    rclcpp::QoS qos_profile_pcl(1);
    // qos_profile_pcl.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    qos_profile_pcl.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    pcl_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(input_point_cloud_topic_, qos_profile_pcl,
        std::bind(&ZionBroadcaster::pclCallback, this, std::placeholders::_1));

    pcl_pub_= this->create_publisher<sensor_msgs::msg::PointCloud2>(filtered_point_cloud_topic_,10);
    pcl_buffer_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
  }


void ZionBroadcaster::loadParams()
{

    // Voxel Filter Parameters
    this->declare_parameter("voxel_filter.leaf_size_xy", 0.025);
    this->declare_parameter("voxel_filter.leaf_size_z", 0.08);
    this->get_parameter("voxel_filter.leaf_size_xy", leaf_size_xy_);
    this->get_parameter("voxel_filter.leaf_size_z", leaf_size_z_);

    // Crop Box Parameters
    this->declare_parameter("crop_box.min_x", 0.0);
    this->declare_parameter("crop_box.max_x", 2.0);
    this->declare_parameter("crop_box.min_y", -0.8);
    this->declare_parameter("crop_box.max_y", 0.8);
    this->declare_parameter("crop_box.min_z", -3.0);
    this->declare_parameter("crop_box.max_z", 1.0);
    this->get_parameter("crop_box.min_x", min_x_);
    this->get_parameter("crop_box.max_x", max_x_);
    this->get_parameter("crop_box.min_y", min_y_);
    this->get_parameter("crop_box.max_y", max_y_);
    this->get_parameter("crop_box.min_z", min_z_);
    this->get_parameter("crop_box.max_z", max_z_);

    // Topic Names
    this->declare_parameter("topic_names.input_point_cloud_topic", "/zedm/zed_node/point_cloud/cloud_registered");
    this->declare_parameter("topic_names.filtered_point_cloud_topic", "/stair_modeling_ros/point_cloud/cloud_filtered");
    this->get_parameter("topic_names.input_point_cloud_topic", input_point_cloud_topic_);
    this->get_parameter("topic_names.filtered_point_cloud_topic", filtered_point_cloud_topic_);

    // Frame IDs
    this->declare_parameter("frame_ids.map_frame", "map");
    this->declare_parameter("frame_ids.base_frame", "zedm_base_link");
    this->declare_parameter("frame_ids.input_cloud_frame", "zedm_left_camera_frame");
    this->declare_parameter("frame_ids.output_cloud_frame", "zedm_base_link_projected");
    this->get_parameter("frame_ids.map_frame", map_frame_);
    this->get_parameter("frame_ids.base_frame", base_frame_);
    this->get_parameter("frame_ids.input_cloud_frame", input_frame_);
    this->get_parameter("frame_ids.output_cloud_frame", output_frame_);
}

void ZionBroadcaster::pclCallback(const sensor_msgs::msg::PointCloud2::SharedPtr pcl_msg)
{
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

          base_projected2camera = tf_buffer_->lookupTransform(
                output_frame_, input_frame_,
                tf2::TimePointZero);

          c2cp = tf2::transformToEigen(base_projected2camera);

          pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud (new  pcl::PointCloud<pcl::PointXYZRGB>);
          pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud (new  pcl::PointCloud<pcl::PointXYZRGB>);

          // from ros msg 
          pcl::fromROSMsg(*pcl_msg, *input_cloud);
          Utilities::voxelizingDownsample(input_cloud, input_cloud,0.025,0.08);
          Utilities::transformCloud(c2cp,input_cloud,input_cloud);
          Utilities::cropping(input_cloud, output_cloud,
                            min_x_,max_x_,
                            min_y_,max_y_,
                            min_z_,max_z_);

          pcl::toROSMsg(*output_cloud, *pcl_buffer_);
          pcl_buffer_->header.stamp = this->get_clock()->now();
          pcl_buffer_->header.frame_id = output_frame_;
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



