// Startup-only readonly inspection of the installed Jazzy/Fast DDS DataWriter.
// No entity creation, QoS writes, publishing, waits or callbacks in this helper.
//
// r119_read_writer fills out[0..13] from the live writer QoS:
//   [0] publish_mode kind    [1] reliability kind  [2] durability kind
//   [3] history kind         [4] history depth
//   [5] max_samples          [6] max_instances     [7] max_samples_per_instance
//   [8] allocated_samples    [9] history_memory_policy
//   [10] data_sharing kind   [11] max_blocking_time seconds
//   [12] max_blocking_time nanoseconds
//   [13] 1 iff the writer topic name equals "rt" + rmw topic name
// Status: 0 = filled, 1 = invalid arguments/width, 2 = not rmw_fastrtps_cpp,
// 3 = no DataWriter, 4 = unexpected exception.
// r119_parse_profile returns the profile publish mode kind, or -1 bad args,
// -2 XML load failure, -3 missing publisher attributes, -4 unexpected exception.
#include <cstdint>
#include <cstring>
#include <rcl/publisher.h>
#include <rmw_fastrtps_shared_cpp/custom_publisher_info.hpp>
#include <fastrtps/xmlparser/XMLProfileManager.h>
#include <fastrtps/attributes/PublisherAttributes.h>

extern "C" int r119_read_writer(uintptr_t pointer, int64_t* out, size_t width) noexcept {
  if (!pointer || !out || width != 14) return 1;
  try {
    auto* publisher = reinterpret_cast<rcl_publisher_t*>(pointer);
    auto* rmw = rcl_publisher_get_rmw_handle(publisher);
    if (!rmw || !rmw->implementation_identifier ||
        std::strcmp(rmw->implementation_identifier, "rmw_fastrtps_cpp") != 0 ||
        !rmw->data) return 2;
    auto* info = static_cast<CustomPublisherInfo*>(rmw->data);
    if (!info->data_writer_) return 3;
    const auto& q = info->data_writer_->get_qos();
    out[0] = q.publish_mode().kind;
    out[1] = q.reliability().kind;
    out[2] = q.durability().kind;
    out[3] = q.history().kind;
    out[4] = q.history().depth;
    out[5] = q.resource_limits().max_samples;
    out[6] = q.resource_limits().max_instances;
    out[7] = q.resource_limits().max_samples_per_instance;
    out[8] = q.resource_limits().allocated_samples;
    out[9] = q.endpoint().history_memory_policy;
    out[10] = q.data_sharing().kind();
    out[11] = q.reliability().max_blocking_time.seconds;
    out[12] = q.reliability().max_blocking_time.nanosec;
    out[13] = (info->data_writer_->get_topic()->get_name() ==
               std::string("rt") + std::string(rmw->topic_name));
    return 0;
  } catch (...) { return 4; }
}

extern "C" int r119_parse_profile(const char* path, const char* topic) noexcept {
  if (!path || !topic) return -1;
  try {
    using namespace eprosima::fastrtps;
    if (xmlparser::XMLProfileManager::loadXMLFile(path) != xmlparser::XMLP_ret::XML_OK) return -2;
    PublisherAttributes attrs;
    if (xmlparser::XMLProfileManager::fillPublisherAttributes(topic, attrs, false) !=
        xmlparser::XMLP_ret::XML_OK) return -3;
    return static_cast<int>(attrs.qos.m_publishMode.kind);
  } catch (...) { return -4; }
}
