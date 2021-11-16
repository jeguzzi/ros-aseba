#include <set>

#include <signal.h>
#include "asebaros/asebaros_node_manager_ros1.h"
#include "asebaros/asebaros_node_ros1.h"
#include "asebaros/utils.h"

using std::chrono_literals::operator""s;
// ------------- ROS Callbacks / ROS specific functions

static unsigned get_port() {
  // listens to incoming connection on this port
  int port;
  ros::param::param<int>("~port", port, ASEBA_DEFAULT_PORT);
  // NOTE: not using nh to avoid initialization race
  // nh.param<int>("port", port, ASEBA_DEFAULT_PORT);
  return port;
}

static bool get_loop() {
  // makes the switch transmit messages back to the send, not only forward them.
  bool loop;
  ros::param::param<bool>("~loop", loop, false);
  LOG_INFO("Loop %d", loop);
  // NOTE: not using nh to avoid initialization race
  // nh.param<bool>("loop", loop, false);
  return loop;
}

AsebaROS1::AsebaROS1()
    :
      n("aseba"), nh("~"),
      anonPub(n.advertise<asebaros_msgs::AnonymousEvent>("anonymous_events", 100)),
      nodes_pub(n.advertise<NodeListMsg>("nodes", 1, true)),
      AsebaROS(get_port(), !get_loop())
#if DIAGNOSTICS
      ,
      updater()
#endif
{
  anonSub = n.subscribe<asebaros_msgs::AnonymousEvent>(
      "anonymous_events", 100,
      [this](const asebaros_msgs::AnonymousEventConstPtr &event) {
        get_anonymous_event_cb(event);
      });
  std::string script_path = init_params();
  if (!script_path.empty()) {
    read_default_script_from_path(script_path);
  }
  // script
  s.push_back(
      n.advertiseService("load_script", &AsebaROS1::load_script_cb, this));
  // nodes
  s.push_back(
      n.advertiseService("get_nodes", &AsebaROS1::get_node_list_cb, this));
#if DIAGNOSTICS
  // updater.add("Aseba Network", this, &AsebaROS1::update_diagnostics);
  updater.add("Aseba Network",
              [this](diagnostic_updater::DiagnosticStatusWrapper &stat) {
                update_diagnostics(stat);
              });
#endif
};

void AsebaROS1::set_connected_target(const std::string &target) {
  AsebaROS::set_connected_target(target);
#if DIAGNOSTICS
  updater.setHardwareID(target);
#endif

  log_initialized();
}

AsebaROSNode *AsebaROS1::add_asebaros_node(unsigned id, const std::string &name,
                                           const std::string &ns, bool include_id_in_events) {
  auto node =
      std::dynamic_pointer_cast<AsebaROSNode>(std::make_shared<AsebaROS1Node>(
          &hub, this, id, name, &nodes.at(id), ns, include_id_in_events));
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

void AsebaROS1::start_pinging() {
  timer = n.createTimer(ros::Duration(1), [this](const ros::TimerEvent &) {
    update();
#if DIAGNOSTICS
    updater.update();
#endif
  });
}

bool AsebaROS1::set_constant_from_param(Aseba::NamedValue &constant,
                                        const std::string &constant_name) {
  std::string param_name = "script/constants/" + constant_name;
  if (nh.getParam(param_name, constant.value)) {
    return true;
  }
  return false;
}

void AsebaROS1::set_constant_to_param(const Aseba::NamedValue &constant,
                                      const std::string &constant_name) {
  std::string param_name = "script/constants/" + constant_name;
  nh.setParam(param_name, constant.value);
}

std::string AsebaROS1::init_params() {
  // Additional targets are any valid Dashel targets.
  nh.param<std::vector<std::string>>("targets", additionalTargets,
                                     std::vector<std::string>());
  nh.param<bool>("reload_script_on_reconnect", reload_script_on_reconnect,
                 false);
  nh.param<bool>("shutdown_on_unconnect", shutdown_on_unconnect, false);
  nh.param<bool>("reset_on_closing", reset_on_closing, false);
  nh.param<bool>("set_id_variable", set_id_variable, false);
  int value = ASEBA_PROTOCOL_VERSION;
  nh.param<int>("highest_acceptable_protocol_version", value, ASEBA_PROTOCOL_VERSION);
  if (value < ASEBA_PROTOCOL_VERSION) {
    value = ASEBA_PROTOCOL_VERSION;
    nh.setParam("highest_acceptable_protocol_version", ASEBA_PROTOCOL_VERSION);
  }
  aseba_max_target_protocol_version = value;

  std::vector<std::string> params;
  nh.getParamNames(params);
  std::set<std::string> imported_params;
  std::string::size_type n;
  for (auto &param : params) {
    n = param.find("nodes/");
    if (n == std::string::npos)
      continue;
    std::string key = param.substr(n + 6);
    n = key.find("/");
    if (n == std::string::npos)
      continue;
    key = key.substr(0, n);
    if (imported_params.count(key))
      continue;
    import_node_config(key);
    imported_params.insert(key);
  }
  std::string file_path;
  if (nh.getParam("script/path", file_path)) {
    return file_path;
  }
  return "";
}

void AsebaROS1::import_node_config(const std::string &prefix) {
  std::string type, name, item_prefix, id_variable;
  bool accept;
  int id = -1;
  int maximal_number_of_nodes = -1;
  bool include_id_in_events = false;
  nh.param<std::string>("nodes/" + prefix + "/name", type, "");
  nh.param<int>("nodes/" + prefix + "/id", id, -1);
  if (nh.getParam("nodes/" + prefix + "/accept", accept)) {
    nodes_configs.accept.set_config(type, id, accept);
  }
  if (nh.getParam("nodes/" + prefix + "/prefix", item_prefix)) {
    nodes_configs.prefix.set_config(type, id, item_prefix);
  }
  if (nh.getParam("nodes/" + prefix + "/namespace", name)) {
    nodes_configs.name.set_config(type, id, name);
  }
  if (nh.getParam("nodes/" + prefix + "/id_variable", id_variable)) {
    nodes_configs.id_variable.set_config(type, id, id_variable);
  }
  if (nh.getParam("nodes/" + prefix + "/include_id_in_events", include_id_in_events)) {
    nodes_configs.include_id_in_events.set_config(type, id, id_variable);
  }
  if (nh.getParam("nodes/" + prefix + "/maximal_number", maximal_number_of_nodes)) {
    nodes_configs.maximal_number_of_nodes.set_config(type, id, maximal_number_of_nodes);
  }
}

bool AsebaROS1::load_script_cb(asebaros_msgs::LoadScript::Request &req,
                               asebaros_msgs::LoadScript::Response &res) {
  return load_script(&req, &res);
}

bool AsebaROS1::get_node_list_cb(asebaros_msgs::GetNodeList::Request &req,
                                 asebaros_msgs::GetNodeList::Response &res) {
  return get_node_list(&req, &res);
}

void AsebaROS1::publish_anonymous_event(
    const Aseba::UserMessage *aseba_message) {
  // unknown, send on the anonymous channel
  asebaros_msgs::AnonymousEvent event;
  event.stamp = ros::Time::now();
  event.source = aseba_message->source;
  event.type = aseba_message->type;
  event.data = aseba_message->data;
  anonPub.publish(event);
}

void AsebaROS1::set_script_param(const std::string & path) {
  nh.setParam("script/path", path);
}

void AsebaROS1::publish_node_list(const NodeListMsg & msg) {
  nodes_pub.publish(msg);
}

AsebaROS1::~AsebaROS1() {
  LOG_INFO("Deleting");
}

static std::function<void(int)> shutdown_handler;
void signal_handler(int signal) { shutdown_handler(signal); }

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "aseba");
  AsebaROS1 node;
  node.wait_for_connection();
  node.run();
  shutdown_handler = [&node](int sig) {
    node.stop();
    LOG_INFO("Shutting down");
    ros::shutdown();
  };
  signal(SIGINT, signal_handler);
  LOG_INFO("Running");
  ros::spin();
  node.stop();
  LOG_INFO("Shutting down");
  return 0;
}
