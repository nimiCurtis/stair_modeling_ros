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
  // auto broadcaster_node = std::make_shared<ZionBroadcaster>();
  // auto cloud_processor_node = std::make_shared<CloudProcessor>();
  // auto stair_node =  std::make_shared<StairModeling>();

  // // run executor
  // auto executor1 = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  // executor1->add_node(broadcaster_node);
  // auto executor1_thread = std::thread(
  //   [&](){
  //     int nice = -5;
  //     setpriority(PRIO_PROCESS, gettid(),nice);
  //     executor1->spin();
  //   }
  // );

  // auto executor2 = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  // executor2->add_node(cloud_processor_node);
  // auto executor2_thread = std::thread(
  //   [&](){
  //     int nice = -5;
  //     setpriority(PRIO_PROCESS, gettid(),nice);
  //     executor2->spin();
  //   }
  // );

  // auto executor3 = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  // executor3->add_node(stair_node);
  // auto executor3_thread = std::thread(
  //   [&](){
  //     int nice = -5;
  //     setpriority(PRIO_PROCESS, gettid(),nice);
  //     executor3->spin();
  //   }
  // );

  // executor1_thread.join();
  // executor2_thread.join();
  // executor3_thread.join();

  // // shutdown
  // rclcpp::shutdown();
  // return 0;

  // Create an executor that will be responsible for execution of callbacks for a set of nodes.
  // With this version, all callbacks will be called from within this thread (the main one).
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;

  // Add some nodes to the executor which provide work for the executor during its "spin" function.
  // An example of available work is executing a subscription callback, or a timer callback.
  auto brodcaster = std::make_shared<stair_modeling::ZionBroadcaster>(options);
  exec.add_node(brodcaster);
  auto processor = std::make_shared<stair_modeling::CloudProcessor>(options);
  exec.add_node(processor);
  auto stair_modeling = std::make_shared<stair_modeling::StairModeling>(options);
  exec.add_node(stair_modeling);


  // spin will block until work comes in, execute work as it becomes available, and keep blocking.
  // It will only be interrupted by Ctrl-C.
  exec.spin();

  rclcpp::shutdown();


}