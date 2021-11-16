#include "asebaros/asebaros_node_manager_ros2.h"
#include "asebaros/asebaros_node_ros2.h"
#include "asebaros/utils.h"

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
// using std::chrono_literals::operator""ms;
using std::chrono_literals::operator""s;
// ------------- ROS Callbacks / ROS specific functions

static rclcpp::NodeOptions node_options() {
  rclcpp::NodeOptions node_options;
  node_options.allow_undeclared_parameters(true);
  node_options.automatically_declare_parameters_from_overrides(true);
  return node_options;
}

unsigned AsebaROS2::get_port() {
  // TODO(Jerome): Add to description
  // listens to incoming connection on this port
  rclcpp::Parameter param("port", ASEBA_DEFAULT_PORT);
  get_parameter("port", param);
  return param.as_int();
}

bool AsebaROS2::get_loop() {
  // TODO(Jerome): Add to description
  // makes the switch transmit messages back to the send, not only forward them.
  rclcpp::Parameter param("loop", false);
  get_parameter("loop", param);
  return param.as_bool();
}

AsebaROS2::AsebaROS2()
    : rclcpp::Node("asebaros", "", node_options()),
      AsebaROS(get_port(), !get_loop())
#if DIAGNOSTICS
      ,
      updater(this)
#endif
{
  std::string script_path = init_params();
  if (!script_path.empty()) {
    read_default_script_from_path(script_path);
  }
  anonPub = create_publisher<asebaros_msgs::msg::AnonymousEvent>(
      "aseba/anonymous_events", 100);
  // auto cb = std::bind(&AsebaROS2::, this, _1);
  anonSub = create_subscription<asebaros_msgs::msg::AnonymousEvent>(
      "aseba/anonymous_events", 100,
      [this](asebaros_msgs::msg::AnonymousEvent::SharedPtr msg) {
        get_anonymous_event_cb(msg);
      });
  nodes_pub = create_publisher<NodeListMsg>("aseba/nodes", rclcpp::QoS(1).transient_local());
  s.push_back(create_service<asebaros_msgs::srv::LoadScript>(
      "aseba/load_script",
      std::bind(&AsebaROS2::load_script_cb, this, _1, _2, _3)));
  s.push_back(create_service<asebaros_msgs::srv::GetNodeList>(
      "aseba/get_nodes", std::bind(&AsebaROS2::get_node_list_cb, this, _1, _2, _3)));
#if DIAGNOSTICS
  // updater.add("Aseba Network", this, &AsebaROS2::update_diagnostics);
  updater.add("Aseba Network", [this](diagnostic_updater::DiagnosticStatusWrapper &stat) {
    update_diagnostics(stat);
  });
#endif

  log_initialized();
};

void AsebaROS2::set_connected_target(const std::string &target) {
  AsebaROS::set_connected_target(target);
#if DIAGNOSTICS
  updater.setHardwareID(target);
#endif

}

AsebaROSNode *AsebaROS2::add_asebaros_node(unsigned id, const std::string &name,
                                           const std::string &ns, bool include_id_in_events) {
  auto node =
      std::dynamic_pointer_cast<AsebaROSNode>(std::make_shared<AsebaROS2Node>(
          &hub, this, this, id, name, &nodes.at(id), ns, include_id_in_events));
  asebaros_nodes[id] = node;
#if DIAGNOSTICS
  updater.add("Aseba Node " + name + " " + std::to_string(id), node.get(),
              &AsebaROSNode::update_diagnostics);
#endif
  return node.get();
  // auto it = asebaros_nodes.emplace(
  //     std::piecewise_construct, std::forward_as_tuple(id),
  //     std::forward_as_tuple(this, id, name, &nodes.at(id), ns,
  //     include_id_in_events));
  // return &(it.first->second);
}

void AsebaROS2::start_pinging() {
  timer = create_wall_timer(1s, std::bind(&AsebaROS2::update, this));
}

bool AsebaROS2::set_constant_from_param(Aseba::NamedValue &constant,
                                        const std::string &constant_name) {
  std::string param_name = "script.constants." + constant_name;
  rclcpp::Parameter param;
  if (get_parameter(param_name, param)) {
    constant.value = param.as_int();
    return true;
  }
  return false;
}

void AsebaROS2::set_constant_to_param(const Aseba::NamedValue &constant,
                                      const std::string &constant_name) {
  std::string param_name = "script.constants." + constant_name;
  set_parameter(rclcpp::Parameter(param_name, constant.value));
}

std::string AsebaROS2::init_params() {
  // TODO(Jerome):  Add to description
  // Additional targets are any valid Dashel targets.
  rclcpp::Parameter targets_param("targets", std::vector<std::string>());
  get_parameter("targets", targets_param);
  additionalTargets = targets_param.as_string_array();
  rclcpp::Parameter reload_script_on_reconnect_param(
      "script.reload_on_reconnect", false);
  get_parameter("script.reload_on_reconnect", reload_script_on_reconnect_param);
  reload_script_on_reconnect = reload_script_on_reconnect_param.as_bool();
  rclcpp::Parameter shutdown_on_unconnect_param("shutdown_on_unconnect", false);
  if (get_parameter("shutdown_on_unconnect", shutdown_on_unconnect_param))
    shutdown_on_unconnect = shutdown_on_unconnect_param.as_bool();
  rclcpp::Parameter reset_on_closing_param("reset_on_closing", false);
  if (get_parameter("reset_on_closing", reset_on_closing_param))
    reset_on_closing = reset_on_closing_param.as_bool();
  rclcpp::Parameter set_id_variable_param("set_id_variable", false);
  if (get_parameter("set_id_variable", set_id_variable_param)) {
    set_id_variable = set_id_variable_param.as_bool();
  }
  rclcpp::Parameter max_target_protocol_version_param(
      "highest_acceptable_protocol_version", ASEBA_PROTOCOL_VERSION);
  if (get_parameter("highest_acceptable_protocol_version", max_target_protocol_version_param)) {
    aseba_max_target_protocol_version = max_target_protocol_version_param.as_int();
  }
  if (aseba_max_target_protocol_version < ASEBA_PROTOCOL_VERSION) {
    aseba_max_target_protocol_version = ASEBA_PROTOCOL_VERSION;
    set_parameter(rclcpp::Parameter("highest_acceptable_protocol_version", ASEBA_PROTOCOL_VERSION));
  }
  auto params = list_parameters({"nodes"}, 3);
  for (auto &prefix : params.prefixes) {
    import_node_config(prefix);
  }
  rclcpp::Parameter file_path_param;
  if (get_parameter("script.path", file_path_param)) {
    return file_path_param.as_string();
  }
  return "";
}

void AsebaROS2::import_node_config(const std::string &prefix) {
  rclcpp::Parameter param;
  std::string name = "";
  int id = -1;
  if (get_parameter(prefix + ".name", param)) {
    name = param.as_string();
  }
  if (get_parameter(prefix + ".id", param)) {
    id = param.as_int();
  }
  if (get_parameter(prefix + ".accept", param)) {
    nodes_configs.accept.set_config(name, id, param.as_bool());
  }
  if (get_parameter(prefix + ".prefix", param)) {
    nodes_configs.prefix.set_config(name, id, param.as_string());
  }
  if (get_parameter(prefix + ".namespace", param)) {
    nodes_configs.name.set_config(name, id, param.as_string());
  }
  if (get_parameter(prefix + ".id_variable", param)) {
    nodes_configs.id_variable.set_config(name, id, param.as_string());
  }
  if (get_parameter(prefix + ".include_id_in_events", param)) {
    nodes_configs.include_id_in_events.set_config(name, id, param.as_bool());
  }
  if (get_parameter(prefix + ".maximal_number", param)) {
    nodes_configs.maximal_number_of_nodes.set_config(name, id, param.as_int());
  }
}

void AsebaROS2::set_script_param(const std::string & path) {
  set_parameter(rclcpp::Parameter("script.path", path));
}

void AsebaROS2::publish_node_list(const NodeListMsg & msg) {
  nodes_pub->publish(msg);
}

void AsebaROS2::load_script_cb(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<asebaros_msgs::srv::LoadScript::Request> req,
    const std::shared_ptr<asebaros_msgs::srv::LoadScript::Response> res) {
    load_script(req, res);
}

void AsebaROS2::get_node_list_cb(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<asebaros_msgs::srv::GetNodeList::Request> req,
    const std::shared_ptr<asebaros_msgs::srv::GetNodeList::Response> res) {
    get_node_list(req, res);
}

void AsebaROS2::publish_anonymous_event(
    const Aseba::UserMessage *aseba_message) {
  // unknown, send on the anonymous channel
  asebaros_msgs::msg::AnonymousEvent event;
  rclcpp::Clock ros_clock(RCL_ROS_TIME);
  event.stamp = ros_clock.now();
  event.source = aseba_message->source;
  event.type = aseba_message->type;
  event.data = aseba_message->data;
  anonPub->publish(event);
}

// int main(int argc, char *argv[]) {
//   rclcpp::init(argc, argv);
//   rclcpp::NodeOptions node_options;
//   node_options.allow_undeclared_parameters(true);
//   node_options.automatically_declare_parameters_from_overrides(true);
//   auto node = std::make_shared<rclcpp::Node>("asebaros", "", node_options);
//   rclcpp::Parameter loop_param("loop", false);
//   node->get_parameter("loop", loop_param);
//   rclcpp::Parameter port_param("port", ASEBA_DEFAULT_PORT);
//   node->get_parameter("port", port_param);
//   AsebaROS2 asebaROS(port_param.as_int(), loop_param.as_bool(), node);
//   asebaROS.wait_for_connection();
//   asebaROS.run();
//   rclcpp::spin(node);
//   asebaROS.stop();
//   LOG_INFO("Shutting down");
//   rclcpp::shutdown();
//   return 0;
// }

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AsebaROS2>();
  node->wait_for_connection();
  node->run();
  rclcpp::spin(node);
  node->stop();
  LOG_INFO("Shutting down");
  rclcpp::shutdown();
  return 0;
}
