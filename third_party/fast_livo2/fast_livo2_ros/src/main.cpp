#include <runner.hpp>
#include <rclcpp/rclcpp.hpp>
#include <csignal>
#include <cstdio>
#include <iostream>
namespace {
void normal_stop(int){fast_livo2_core::request_mapping_stop();}
int mapping_failure_exit(const char* first_error)
{
  std::fprintf(stderr, "V19 mapping failure: %.512s\n", first_error);
  try { rclcpp::shutdown(); }
  catch (const std::exception& secondary) {
    std::fprintf(stderr, "V19 secondary ROS shutdown failure: %.512s\n", secondary.what());
  } catch (...) {
    std::fprintf(stderr, "V19 secondary ROS shutdown failure: non-std exception\n");
  }
  return 2;
}
}

int main(int argc, char **argv)
{
  // Drain the existing mapper while its ROS context is still valid. The
  // default rclcpp SIGINT handler shuts the context down before that drain.
  // Only lifecycle signalling changes; estimator arithmetic is unchanged.
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT,normal_stop);
  std::signal(SIGTERM,normal_stop);
  std::signal(SIGUSR1,normal_stop);
  try {
  auto nh = std::make_shared<rclcpp::Node>("laserMapping");
  fast_livo2_core::run_mapping(nh);
  rclcpp::shutdown();
  return 0;
  }catch(const std::exception& e){return mapping_failure_exit(e.what());}
  catch(...){return mapping_failure_exit("non-std exception");}
}
