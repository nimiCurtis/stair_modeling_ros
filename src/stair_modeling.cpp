#include "stair_modeling_ros/utilities.h"
#include "stair_modeling_ros/plane.h"
#include "stair_modeling_ros/stair.h"
#include "stair_modeling_ros/stair_modeling.h"

#include <zion_msgs/msg/env_geo_stamped.h>
#include <zion_msgs/msg/env_geo.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <math.h>
#include <iostream>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <sensor_msgs/msg/point_cloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/filters/crop_box.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <Eigen/Dense>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/mls.h>


StairModeling::StairModeling() : Node("StairModeling")
{   
    //load params
    loadParams();

    // subscribers and publishers
    rclcpp::QoS qos_profile_pcl(1);
    // qos_profile_pcl.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    qos_profile_pcl.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    pcl_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(filtered_point_cloud_topic_, qos_profile_pcl,
        std::bind(&StairModeling::pclCallback, this, std::placeholders::_1));

    hull_marker_array_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/stair_modeling_ros/hull_marker_array",10);
    env_geo_pub_ = this->create_publisher<zion_msgs::msg::EnvGeoStamped>("/stair_modeling_ros/env_geo_params",10);

    // init tf instances
    tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // init pcl and buffer
    cloud_.reset(new pcl::PointCloud<pcl::PointXYZRGB>());

    // Initialize to an invalid index
    floor_index_ = -1;  
    level_index_ = -1;

    colors_={
    255., 0.0, 0.0, // red  
    0.0, 255., 0.0, // green
    0.0, 0.0, 255., // blue
    };

}


void StairModeling::loadParams()
{
    // Debug
    this->declare_parameter("debug", false);
    this->get_parameter("debug", debug_);

    // Segmentation Parameters
    this->declare_parameter("segmentation.distance_threshold", 0.05);
    this->declare_parameter("segmentation.max_iterations", 600);
    this->declare_parameter("segmentation.angle_threshold", 5.);
    this->get_parameter("segmentation.distance_threshold", distance_threshold_);
    this->get_parameter("segmentation.max_iterations", max_iterations_);
    this->get_parameter("segmentation.angle_threshold", angle_threshold_);
    
    // Clustering Parameters
    this->declare_parameter("clustering.cluster_tolerance", 0.08);
    this->declare_parameter("clustering.min_cluster_size", 50);
    this->get_parameter("clustering.cluster_tolerance", cluster_tolerance_);
    this->get_parameter("clustering.min_cluster_size", min_cluster_size_);

    // Floor Finding Parameters
    this->declare_parameter("floor_finding.k_neighbors", 50);
    this->get_parameter("floor_finding.k_neighbors", k_neighbors_);
    
    // Average X Calculation Parameters
    this->declare_parameter("avg_x_calculation.x_neighbors", 20);
    this->declare_parameter("avg_x_calculation.y_threshold", 0.05);
    this->get_parameter("avg_x_calculation.x_neighbors", x_neighbors_);
    this->get_parameter("avg_x_calculation.y_threshold", y_threshold_);
    
    // Topic Names
    this->declare_parameter("topic_names.filtered_point_cloud_topic", "/stair_modeling_ros/point_cloud/cloud_filtered");
    this->get_parameter("topic_names.filtered_point_cloud_topic", filtered_point_cloud_topic_);
    
    // Frame IDs
    this->declare_parameter("frame_ids.output_cloud_frame", "zedm_base_link_projected");
    this->get_parameter("frame_ids.output_cloud_frame", output_frame_);
}


void StairModeling::printDebug()
{
    debug_msg_ = debug_msg_ + "\n-----";
    RCLCPP_INFO( this->get_logger(), "%s", debug_msg_.c_str());
}

// void StairModeling::computeC2CpTransformations()
// {
//     geometry_msgs::msg::TransformStamped transform_stamped;
//     // Check if transform is available
//     if (tf_buffer_->canTransform(output_frame_id_, input_frame_id_, tf2::TimePointZero, tf2::durationFromSec(1.)))
//     {
//         try
//         {
//             transform_stamped = tf_buffer_->lookupTransform(
//                 output_frame_id_, input_frame_id_,
//                 tf2::TimePointZero);

//             // Convert tf2::Transform to Eigen::Affine3d
//             c2cp = tf2::transformToEigen(transform_stamped);
//             // Inverse the transform
//             cp2c = c2cp.inverse();
//         }
//         catch (tf2::TransformException& ex)
//         {
//         RCLCPP_INFO(
//                     this->get_logger(), "Could not transform %s to %s: %s",
//                     output_frame_id_.c_str(), input_frame_id_.c_str(), ex.what());
//                 return;
//         }
//     }
// }

void StairModeling::getPlanes(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud)
{       
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr outlier_points(new pcl::PointCloud<pcl::PointXYZRGB>);
        *outlier_points=*input_cloud;
        int i = 0;
        int n_points = static_cast<int>(outlier_points->points.size());

        // set segmenter
        pcl::SACSegmentation<pcl::PointXYZRGB> plane_segmenter;
        plane_segmenter.setOptimizeCoefficients(true);
        plane_segmenter.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
        plane_segmenter.setMethodType(pcl::SAC_RANSAC);
        plane_segmenter.setMaxIterations(max_iterations_);
        plane_segmenter.setAxis(Eigen::Vector3f(1, 0, 0));
        plane_segmenter.setDistanceThreshold(distance_threshold_);
        plane_segmenter.setEpsAngle(Utilities::deg2rad(angle_threshold_));

        while ((outlier_points->points.size() > 0.15 * n_points) && i<2) {
            i++;
            debug_msg_ = debug_msg_ + "\nRansac iteration: " + std::to_string(i);

            // get the segmented plane cloud
            pcl::ModelCoefficients::Ptr plane_coefficients(new pcl::ModelCoefficients());
            pcl::PointIndices::Ptr plane_indices(new pcl::PointIndices());
            
            plane_segmenter.setInputCloud(outlier_points);
            plane_segmenter.segment(*plane_indices, *plane_coefficients);

            // if cloud is too small break the loop 
            if (static_cast<int>(plane_indices->indices.size()) < min_cluster_size_){
                RCLCPP_INFO(this->get_logger(), 
                    "Could not estimate a planar model for plane because it too small .\n");
                break;
            }
            else{ 
                // get the inliers and remove points using euclidean clustering
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_points(new pcl::PointCloud<pcl::PointXYZRGB>);
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_points_clustered(new pcl::PointCloud<pcl::PointXYZRGB>);

                // get the plane points
                Utilities::getCloudByInliers(outlier_points, plane_points, plane_indices, false, false); 
                Utilities::euclideanClustering(plane_points,plane_points_clustered);

                /// keep outlier
                Utilities::getCloudByInliers(outlier_points, outlier_points, plane_indices, true, false); 
                debug_msg_ = debug_msg_ + "\nPlane " + std::to_string(i) + " has: " + std::to_string(plane_points->points.size()) + " points";
                
                // get the first plane 
                if (Planes_.size()==0){
                    Plane plane = Plane(plane_points_clustered,plane_coefficients);
                    plane.calcPlaneSlope();
                    Planes_.push_back(plane);
                }
                // get the second plane
                else{
                    // check for the height validation
                    // if height difference is in valid range , pushback the second plane
                    if (Stair::checkValidHeight(Planes_[0].plane_coefficients_->values[3],plane_coefficients->values[3])){
                        Plane plane = Plane(plane_points_clustered,plane_coefficients);
                        plane.calcPlaneSlope();
                        Planes_.push_back(plane);
                    }
                    // if not, keep in while loop
                    else{
                        i--;
                    } 
                } 
            }
        } // while


        // planes features for debug
        for (size_t i = 0; i < Planes_.size(); ++i) {
                        debug_msg_ = debug_msg_ + "\n------- Plane: " + std::to_string(i+1) + "-------";

                        debug_msg_ = debug_msg_ +  
                        "\ncoefficients: [" + std::to_string(Planes_[i].plane_coefficients_->values[0]) + " "
                                            + std::to_string(Planes_[i].plane_coefficients_->values[1]) + " "
                                            + std::to_string(Planes_[i].plane_coefficients_->values[2]) + " "
                                            + std::to_string(Planes_[i].plane_coefficients_->values[3]) + "]";

                        debug_msg_ = debug_msg_ + 
                        "\ncentroid: " +"x: " + std::to_string(Planes_[i].centroid_.x) + " | "
                                      +"y: " + std::to_string(Planes_[i].centroid_.y) + " | "
                                      +"z: " + std::to_string(Planes_[i].centroid_.z);
                        
                        debug_msg_ = debug_msg_ +
                        "\nPCA rotation matrix: " +" \n[" + std::to_string(Planes_[i].plane_dir_.col(0)[0]) + " " + std::to_string(Planes_[i].plane_dir_.col(1)[0])+ " " + std::to_string(Planes_[i].plane_dir_.col(2)[0])
                                                +"\n " + std::to_string(Planes_[i].plane_dir_.col(0)[1]) + " " + std::to_string(Planes_[i].plane_dir_.col(1)[1])+ " " + std::to_string(Planes_[i].plane_dir_.col(2)[1])
                                                +"\n " + std::to_string(Planes_[i].plane_dir_.col(0)[2]) + " " + std::to_string(Planes_[i].plane_dir_.col(1)[2])+ " " + std::to_string(Planes_[i].plane_dir_.col(2)[2])+"]";

                        debug_msg_ = debug_msg_ + 
                        "\nbounding rect center: " +"x: " + std::to_string(Planes_[i].center_.x) + " | "
                                      +"y: " + std::to_string(Planes_[i].center_.y) + " | "
                                      +"z: " + std::to_string(Planes_[i].center_.z);
                        
                        debug_msg_ = debug_msg_ + 
                        "\nwidth: " + std::to_string(Planes_[i].width_) + " | "
                        +"lenght: " + std::to_string(Planes_[i].length_);
                        
                        debug_msg_ = debug_msg_ + "\n-----";

}
}

void StairModeling::reset()
{
    // Clear Planes and Stair if needed
    Planes_.clear();
    debug_msg_ = "debug";
    floor_index_ = -1;  
    level_index_ = -1;
    stair_detected_ = false;
}

void StairModeling::findFloor() 
{

    float min_avg_x = std::numeric_limits<float>::max();

    // Loop through each plane

    if (Planes_.size()>1){
        for (size_t i = 0; i < Planes_.size(); ++i) {
            const auto& plane = Planes_[i];

            // Sort the points based on their x-values
            std::sort(plane.cloud_->points.begin(), plane.cloud_->points.end(),
                    [](const pcl::PointXYZRGB &a, const pcl::PointXYZRGB &b) {
                        return a.x < b.x;
                    });

            // Calculate the average 'X' value of the k_neighbors_ smallest x-values
            float avg_x = 0.0;
            for (int j = 0; j < k_neighbors_; ++j) {
                avg_x += plane.cloud_->points[j].x;
            }
            avg_x /= k_neighbors_;

            // Update the closest plane index based on the avg_x
            ///// continueee from here

            if (avg_x < min_avg_x) {
                min_avg_x = avg_x;
                floor_index_ = static_cast<int>(i);
            }
        }
        if (floor_index_==0){ level_index_ = 1;}
        else{ level_index_ = 0;}
    }else{
        floor_index_=0;
    }
    
    // set planes type 
    if (floor_index_!=-1){Planes_[floor_index_].type_ = 0;}
    if (level_index_!=-1){Planes_[level_index_].type_ = 1;}
    
    // for debug
    debug_msg_ = debug_msg_ + "\nFloor index: " + std::to_string(floor_index_) + " | Level index: " + std::to_string(level_index_); 
}

void StairModeling::checkForValidStair()
{   
    // check there is more then 1 plane
    // and the level length is more then the min
    if(Planes_.size()>1 && Planes_[level_index_].length_>=k_length_min){
        stair_detected_ = true;
    }
}


void::StairModeling::getStair()
{

    // init stair
    Stair_ = Stair(Planes_);  
    Stair_.step_length_ = Stair_.Planes_[level_index_].length_;
    Stair_.step_width_ = Stair_.Planes_[level_index_].width_;
    Stair_.transition_point_.y = Stair_.Planes_[level_index_].center_.y;
    Stair_.transition_point_.z = Stair_.Planes_[level_index_].center_.z;


    float floor_h = Stair_.Planes_[floor_index_].plane_coefficients_->values[3];
    float level_h = Stair_.Planes_[level_index_].plane_coefficients_->values[3];
    
    // set the stair type based on the planes height
    // if floor height is high than the level height (farther from the camera) -> stair ascending
    if (floor_h>level_h){
        Stair_.type_ = 0; // 0 = stair is upward
        debug_msg_ = debug_msg_ + "\nStair type: Up";
        Stair_.transition_point_.x = Stair_.Planes_[level_index_].center_.x - (Stair_.step_length_/2);
        // If upwards, calculate the distance by finding the average x-coordinate
        // of the points below yThreshold in the level plane's cloud
        Stair_.step_distance_ = Utilities::findAvgXForPointsBelowYThreshold(Planes_[level_index_].cloud_, y_threshold_, x_neighbors_, true);
    }else{ // -> stair descending
        Stair_.type_ = 1; // 1 = downwards
        debug_msg_ = debug_msg_ + "\nStair type: Down";
        Stair_.transition_point_.x = Stair_.Planes_[level_index_].center_.x + (Stair_.step_length_/2);
        // compute distance
        // If downwards, calculate the distance by finding the average x-coordinate
        // of the points below yThreshold in the second plane's cloud
        Stair_.step_distance_ = Utilities::findAvgXForPointsBelowYThreshold(Planes_[level_index_].cloud_, y_threshold_, x_neighbors_, false);
    }
    // compute height
    Stair_.step_height_ = fabs(floor_h-level_h);
    // compute angle
    Stair_.step_angle_ = Utilities::rad2deg(std::atan2(Stair_.transition_point_.y,Stair_.transition_point_.x));

    // debug
    debug_msg_ = debug_msg_ + "\nHeight is:  " + std::to_string(Stair_.step_height_ );
    debug_msg_ = debug_msg_ + "\nDistance is:  " + std::to_string(Stair_.step_distance_);
}

void StairModeling::setStairTf()
{
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    geometry_msgs::msg::TransformStamped transformStamped;

    // set transform msg
    tf2::Transform tf2_transform;

    tf2_transform.setIdentity();    
    transformStamped.header.stamp = this->get_clock()->now();
    transformStamped.header.frame_id = output_frame_;
    transformStamped.child_frame_id = "stair";

    //translation
    transformStamped.transform.translation.x = Stair_.transition_point_.x;
    transformStamped.transform.translation.y = Stair_.transition_point_.y;
    transformStamped.transform.translation.z = Stair_.transition_point_.z;

    //rotation
    tf2::Quaternion q_rotation;
    q_rotation.setRPY(0.,0.,0.);
    q_rotation.normalize();

    transformStamped.transform.rotation = tf2::toMsg(q_rotation);
    // sending transform
    tf_broadcaster->sendTransform(transformStamped);
}

void StairModeling::calcPlaneSlope()
{
    Planes_[0].calcPlaneSlope();
    debug_msg_ = debug_msg_ + "\nSlope is:  " + std::to_string(Planes_[0].slope_);
    printDebug();
}

void StairModeling::publishEnvGeo(const std::string& cloud_frame)
{   
    
    zion_msgs::msg::EnvGeoStamped env_geo_stamped_msg; //with or without ::msg ?
    zion_msgs::msg::EnvGeo env_geo_msg;
    
    env_geo_stamped_msg.header.frame_id = cloud_frame;
    env_geo_stamped_msg.header.stamp = this->get_clock()->now();

    // declare env_geo_msg
    if(stair_detected_){
        env_geo_msg.stair.steps = 1;
        env_geo_msg.stair.distance = Stair_.step_distance_;
        env_geo_msg.stair.height = Stair_.step_height_;
        env_geo_msg.stair.angle = Stair_.step_angle_;
        env_geo_msg.stair.length = Stair_.step_length_;
        env_geo_msg.stair.width = Stair_.step_width_;
        env_geo_msg.plane.slope = 0;
    }
    else{
        env_geo_msg.stair.steps = 0;
        if(Planes_.size() == 1){
            env_geo_msg.plane.slope = Planes_[0].slope_;
        }
    }

    // declare env_geo_stamped_msg
    env_geo_stamped_msg.env_geo = env_geo_msg;
    env_geo_pub_->publish(env_geo_stamped_msg);
}

void StairModeling::publishHullsAsMarkerArray(const std::string& cloud_frame)
{

  geometry_msgs::msg::Point point;
  std_msgs::msg::ColorRGBA point_color;
  visualization_msgs::msg::MarkerArray ma;

  if(Planes_.size()>0){
    for (size_t i = 0; i < Planes_.size(); i++){

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = cloud_frame;
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "hull_" + std::to_string(i);
        marker.id = i;
        marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.03;
        marker.scale.y = 0.03;
        marker.scale.z = 0.03;
        marker.color.a = 1.0;

        const int nColor = i % (colors_.size()/3);
        const double r = colors_[nColor*3]*255.0;
        const double g = colors_[nColor*3+1]*255.0;
        const double b = colors_[nColor*3+2]*255.0;
        marker.points.reserve(Planes_[i].hull_->points.size());
        marker.colors.reserve(Planes_[i].hull_->points.size());

        for (size_t j = 1; j < Planes_[i].hull_->points.size(); j++){
        point.x = Planes_[i].hull_->points[j-1].x;
        point.y = Planes_[i].hull_->points[j-1].y;
        point.z = Planes_[i].hull_->points[j-1].z;
        point_color.r = r;
        point_color.g = g;
        point_color.b = b;
        point_color.a = 1.0;
        marker.colors.push_back(point_color);
        marker.points.push_back(point);

        point.x = Planes_[i].hull_->points[j].x;
        point.y = Planes_[i].hull_->points[j].y;
        point.z = Planes_[i].hull_->points[j].z;
        point_color.r = r;
        point_color.g = g;
        point_color.b = b;
        point_color.a = 1.0;
        marker.colors.push_back(point_color);
        marker.points.push_back(point);
        }

        // start to end line:
        point.x = Planes_[i].hull_->points[0].x;
        point.y = Planes_[i].hull_->points[0].y;
        point.z = Planes_[i].hull_->points[0].z;
        point_color.r = r;
        point_color.g = g;
        point_color.b = b;
        point_color.a = 1.0;
        marker.colors.push_back(point_color);
        marker.points.push_back(point);

        point.x = Planes_[i].hull_->points[ Planes_[i].hull_->points.size()-1 ].x;
        point.y = Planes_[i].hull_->points[ Planes_[i].hull_->points.size()-1 ].y;
        point.z = Planes_[i].hull_->points[ Planes_[i].hull_->points.size()-1 ].z;
        point_color.r = r;
        point_color.g = g;
        point_color.b = b;
        point_color.a = 1.0;
        marker.colors.push_back(point_color);
        marker.points.push_back(point);

        marker.frame_locked = false;
        ma.markers.push_back(marker);
    }

    hull_marker_array_pub_->publish(ma);
    }
}

// void StairModeling::transformCloud(const sensor_msgs::msg::PointCloud2::SharedPtr input_cloud,
//                                sensor_msgs::msg::PointCloud2::SharedPtr output_cloud)
// {
//     geometry_msgs::msg::TransformStamped transform_stamped;
//     // Check if transform is available
//     if (tf_buffer_->canTransform(output_frame_id_, input_frame_id_, tf2::TimePointZero, tf2::durationFromSec(0.01)))
//     {
//         try
//         {
//             transform_stamped = tf_buffer_->lookupTransform(
//             output_frame_id_, input_frame_id_,
//             tf2::TimePointZero);

//             tf2::doTransform(*input_cloud,*output_cloud,transform_stamped);

//         }
//         catch (tf2::TransformException& ex)
//         {
//         RCLCPP_INFO(
//                     this->get_logger(), "Could not transform %s to %s: %s",
//                     output_frame_id_.c_str(), input_frame_id_.c_str(), ex.what());
//                 return;
//         }
// };

// }

void StairModeling::pclCallback(const sensor_msgs::msg::PointCloud2::SharedPtr pcl_msg)
{
        reset();

        // from ros msg 
        pcl::fromROSMsg(*pcl_msg, *cloud_);

        // compute the transformation matrix of input_frame<>target_frame
        // computeC2CpTransformations();

        // Voxel downsampling
        // auto start_time = std::chrono::steady_clock::now(); // Start the timer
        // Utilities::voxelizingDownsample(cloud_, cloud_,leaf_size_xy_,leaf_size_z_);
        // auto end_time = std::chrono::steady_clock::now(); // End the timer
        // auto elapsed_time_voxel = std::chrono::duration_cast<std::chrono::duration<double>>(end_time - start_time).count();
        // std::cout << "Execution voxel time: " << elapsed_time_voxel << " seconds" << std::endl;

        // transform cloud to 'projceted' coordinate system 
        // Utilities::transformCloud(c2cp,cloud_,cloud_);

        // Cropping


        // RANSAC-based plane segmentation
        getPlanes(cloud_);

        // find the floor plane
        findFloor();
        checkForValidStair();

        // if stair detected set the stair instance and compute geometric parameters
        if(stair_detected_){
            getStair();
            setStairTf();
        }

        rclcpp::Time stamp = this->get_clock()->now();
        
        // Publish the processed point cloud and custom msgs
        publishHullsAsMarkerArray(output_frame_);
        publishEnvGeo(output_frame_);

        // debug msg
        if(debug_){
            printDebug();
        }
}

