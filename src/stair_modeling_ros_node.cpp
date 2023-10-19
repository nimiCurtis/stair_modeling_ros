// system
#include <sys/resource.h>

// ros
#include <rclcpp/rclcpp.hpp>

// custom
#include "stair_modeling_ros/stair_modeling.h"
#include "stair_modeling_ros/zion_tf2_broadcaster.h"
#include "stair_modeling_ros/cloud_processor.h"


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // build nodes
  auto broadcaster_node = std::make_shared<ZionBroadcaster>();
  auto cloud_processor_node = std::make_shared<CloudProcessor>();
  auto stair_node =  std::make_shared<StairModeling>();

  
  // run executor
  auto executor1 = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor1->add_node(broadcaster_node);
  auto executor1_thread = std::thread(
    [&](){
      int nice = -5;
      setpriority(PRIO_PROCESS, gettid(),nice);
      executor1->spin();
    }
  );

  auto executor2 = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor2->add_node(cloud_processor_node);
  auto executor2_thread = std::thread(
    [&](){
      int nice = -5;
      setpriority(PRIO_PROCESS, gettid(),nice);
      executor2->spin();
    }
  );

  auto executor3 = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor3->add_node(stair_node);
  auto executor3_thread = std::thread(
    [&](){
      int nice = -5;
      setpriority(PRIO_PROCESS, gettid(),nice);
      executor3->spin();
    }
  );

  executor1_thread.join();
  executor2_thread.join();
  executor3_thread.join();
  
  // shutdown
  rclcpp::shutdown();
  return 0;
}