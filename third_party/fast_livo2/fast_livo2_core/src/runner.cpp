#include <fast_livo2_core/runner.hpp>

#include "LIVMapper.h"
#include <csignal>
#include <atomic>

#include <image_transport/image_transport.hpp>

namespace fast_livo2_core
{

namespace {static_assert(std::atomic<bool>::is_always_lock_free,"normal stop must be signal-safe lock-free");std::atomic<bool> normal_stop{false};}
void request_mapping_stop() noexcept {normal_stop.store(true,std::memory_order_relaxed);}
bool mapping_stop_requested() noexcept {return normal_stop.load(std::memory_order_relaxed);}
void run_mapping(const std::shared_ptr<rclcpp::Node> &node)
{
  auto core_node = node;
  image_transport::ImageTransport image_transport(core_node);
  LIVMapper mapper(core_node, core_node->get_name());
  mapper.initializeSubscribersAndPublishers(core_node, image_transport);
  mapper.run(core_node);
}

}  // namespace fast_livo2_core
