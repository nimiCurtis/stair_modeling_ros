// system
#include <sys/resource.h>
#include <memory>

// ros
#include <rclcpp/rclcpp.hpp>

// custom
#include "stair_modeling_ros/stair_modeling_component.hpp"
#include "stair_modeling_ros/zion_broadcaster_component.hpp"
#include "stair_modeling_ros/cloud_processor_component.hpp"


int main(int argc, char * argv[])
{ 
  // Force flush of the stdout buffer.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  
  // Initialize any global resources needed by the middleware and the client library.
  rclcpp::init(argc, argv);

  // // build nodes
    rclcpp::NodeOptions options;

    auto brodcaster = std::make_shared<stair_modeling::ZionBroadcaster>(options);
    auto processor = std::make_shared<stair_modeling::CloudProcessor>(options);
    auto stair_modeling = std::make_shared<stair_modeling::StairModeling>(options);

  // // run executor
  auto executor1 = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor1->add_node(brodcaster);
  auto executor1_thread = std::thread(
    [&](){
      int nice = -5;
      setpriority(PRIO_PROCESS, gettid(),nice);
      executor1->spin();
    }
  );

  auto executor2 = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor2->add_node(processor);
  auto executor2_thread = std::thread(
    [&](){
      int nice = -5;
      setpriority(PRIO_PROCESS, gettid(),nice);
      executor2->spin();
    }
  );

  auto executor3 = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor3->add_node(stair_modeling);
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