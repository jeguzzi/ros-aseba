#include "asebaros/asebaros_node_ros2.h"
#include "asebaros/asebaros_node_manager_ros2.h"
#include "asebaros/utils.h"

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

AsebaROS2Node::AsebaROS2Node(AsebaDashelHub *hub, AsebaROS2 *manager,
                             rclcpp::Node *ros_node, const unsigned id,
                             const std::string name,
                             const Aseba::TargetDescription *description,
                             const std::string &namespace_,
                             bool include_id_in_events, bool is_connected)
    : AsebaROSNode(hub, manager, id, name, description, namespace_,
                   include_id_in_events, is_connected),
      ros_node(ros_node), pubs(), subs(), services() {
  desc_pub = ros_node->create_publisher<NodeDescriptionMsg>(
            ros_name("description"), rclcpp::QoS(1).transient_local());
  services.push_back(
      ros_node->create_service<asebaros_msgs::srv::GetDescription>(
          ros_name("get_description"),
          std::bind(&AsebaROS2Node::get_description_cb, this, _1, _2, _3)));
  services.push_back(ros_node->create_service<asebaros_msgs::srv::SetVariable>(
      ros_name("set_variable"),
      std::bind(&AsebaROS2Node::set_variable_cb, this, _1, _2, _3)));
  services.push_back(ros_node->create_service<asebaros_msgs::srv::GetVariable>(
      ros_name("get_variable"),
      std::bind(&AsebaROS2Node::get_variable_cb, this, _1, _2, _3)));
}

bool AsebaROS2Node::publish_event(const Aseba::UserMessage *aseba_message) {
  auto pub = get_publisher_for(aseba_message);
  if (pub) {
    asebaros_msgs::msg::Event event;
    rclcpp::Clock ros_clock(RCL_ROS_TIME);
    event.stamp = ros_clock.now();
    event.source = aseba_message->source;
    event.data = aseba_message->data;
    pub->publish(event);
    return true;
  }
  return false;
}

void AsebaROS2Node::reset_publishers() { pubs.clear(); }

void AsebaROS2Node::create_subscribers() {
  subs.clear();
  unsigned i = 0;
  for (const auto &event : script->common_definitions.events) {
    subs.push_back(ros_node->create_subscription<asebaros_msgs::msg::Event>(
        ros_name(EVENTS_NS + narrow(event.name)), 100,
        [this, i](asebaros_msgs::msg::Event::SharedPtr msg) {
          got_event_message_cb(i, msg);
        }));
    i++;
  }
}

std::shared_ptr<rclcpp::Publisher<asebaros_msgs::msg::Event>>
AsebaROS2Node::get_publisher_for(const Aseba::UserMessage *asebaMessage) {
  unsigned type = asebaMessage->type;
  // known, send on a named channel
  std::unique_lock<std::mutex> lock(mutex);
  // std::shared_lock<std::shared_timed_mutex> lock(mutex);
  if (pubs.count(type) == 0) {
    if (!(script &&
          (asebaMessage->type < script->common_definitions.events.size()))) {
      pubs[type] = nullptr;
      // LOG_WARN("no script or %d above event size", asebaMessage->type);
    } else {
      const std::wstring &name = script->common_definitions.events[type].name;
      pubs[type] = ros_node->create_publisher<asebaros_msgs::msg::Event>(
          ros_name(EVENTS_NS + narrow(name)), 100);
    }
  }
  return pubs[type];
}

void AsebaROS2Node::set_variable_cb(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<asebaros_msgs::srv::SetVariable::Request> req,
    const std::shared_ptr<asebaros_msgs::srv::SetVariable::Response> res) {
  set_variable(req->variable, req->data, true);
}

void AsebaROS2Node::get_variable_cb(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<asebaros_msgs::srv::GetVariable::Request> req,
    const std::shared_ptr<asebaros_msgs::srv::GetVariable::Response> res) {
  res->data = get_variable(req->variable);
}

void AsebaROS2Node::get_description_cb(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<asebaros_msgs::srv::GetDescription::Request> req,
    const std::shared_ptr<asebaros_msgs::srv::GetDescription::Response> res) {
  get_description(req, res);
}

void AsebaROS2Node::publish_description(const NodeDescriptionMsg & msg) {
  desc_pub->publish(msg);
}

std::string AsebaROS2Node::absolute_namespace(const std::string & name) const {
  std::string ns = std::string(ros_node->get_namespace());
  if (name.substr(0, 1) != "/" && ns.substr(0, ns.size()) != "/") {
    ns += "/";
  }
  return ns + name;
}
