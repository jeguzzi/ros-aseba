#include <chrono> // NOLINT
#include <sstream>
#include <stdexcept>
#include <algorithm>

#include "asebaros/asebaros_node_manager.h"
#include "asebaros/utils.h"

#include "libxml/parser.h"
#include "transport/dashel_plugins/dashel-plugins.h"
using std::chrono_literals::operator""ms;

// ------------- Utilities

std::string AsebaROS::variable_id_for_node(const std::string &type,
                                           unsigned id) {
  return nodes_configs.id_variable.get_config(type, id);
}

int AsebaROS::number_of_nodes(const std::string type) {
  if (type.empty()) return (int) asebaros_nodes.size();
  int n = 0;
  for (auto &it : asebaros_nodes) {
    if (it.second->type() == type) n++;
  }
  return n;
}

bool AsebaROS::should_ignore_node(const std::string &type, unsigned id) {
  if (asebaros_nodes.count(id)) return true;
  if (!nodes_configs.accept.get_config(type, id)) return true;
  for (const auto & t : std::vector<std::string>{type, ""}) {
    int maximal_number = nodes_configs.maximal_number_of_nodes.get_config(t, id);
    if (maximal_number > 0 && maximal_number <= number_of_nodes(t))
      return true;
  }
  return false;
}

std::string AsebaROS::namespace_for_node(const std::string &type, unsigned id) {
  std::string name = nodes_configs.name.get_config(type, id);
  if (name != "") {
    return name;
  }
  std::string prefix = nodes_configs.prefix.get_config(type, id);
  if (prefix != "") {
    return prefix + std::to_string(id);
  }
  return "";
}

void AsebaROS::set_connected_target(const std::string &target) {
  connected_to = target;
}

void AsebaROS::update() {
  if (connected_to != "") {
    pingNetwork();
  }
}

void AsebaROS::update_script_constants(AsebaScript *script,
                                       bool set_parameters) {
  for (Aseba::NamedValue &constant : script->common_definitions.constants) {
    std::string constant_name = narrow(constant.name);
    if (!set_constant_from_param(constant, constant_name) && set_parameters) {
      set_constant_to_param(constant, constant_name);
    }
  }
}

void AsebaROS::read_default_script_from_path(const std::string &script_path) {
  LOG_INFO("Will try to read script from %s", script_path.c_str());
  default_script = AsebaScript::from_file(script_path);
  if (default_script) {
    update_script_constants(default_script.get(), true);
  }
}

bool AsebaROS::wait_for_connection() {
  bool connected = false;
  set_connected_target("");
  if (additionalTargets.empty()) {
    return false;
  }
  while (ok() && !connected) {
    for (auto &target : additionalTargets) {
      LOG_INFO("Connecting %s", target.c_str());
      try {
        connectTarget(target);
        connected = true;
      } catch (Dashel::DashelException e) {
        LOG_ERROR("Error while connecting %s: %s", target.c_str(), e.what());
      }
    }
    if (!connected) {
      LOG_WARN("Could not connect to any target. Sleep for 1 second and retry");
      sleep_for_s(1);
    }
  }
  sleep_for_s(1);
  return true;
}

void AsebaROS::forward_event_to_ros(const Aseba::UserMessage *aseba_message) {
  // does not need locking, called by other member function already within lock
  // if different, we are currently loading a new script, publish on anonymous
  // channel
  if (asebaros_nodes.count(aseba_message->source)) {
    auto node = asebaros_nodes.at(aseba_message->source);
    if (node->publish_event(aseba_message))
      return;
  }
  publish_anonymous_event(aseba_message);
}

void AsebaROS::log_initialized() {
  LOG_INFO("Initialized AsebaROS: will connect to Aseba nodes%s with config\n%s",
           default_script ? ", ready to load a default script" : "",
           nodes_configs.description().c_str());
}

std::vector<int16_t> AsebaROS::query_variable(unsigned nodeId, unsigned pos,
                                              unsigned length) {
  // create query
  const GetVariableQueryKey key(nodeId, pos);
  GetVariableQueryValue query;
  std::unique_lock<std::mutex> lock(mutex);
  getVariableQueries[key] = &query;
  lock.unlock();

  Aseba::GetVariables msg(nodeId, pos, length);
  hub.sendMessage(&msg, true);

  // wait 100 ms, considering the possibility of spurious wakes
  lock.lock();
  bool result = query.cond.wait_for(lock, 100ms) == std::cv_status::no_timeout;
  // remove key and return answer
  getVariableQueries.erase(key);
  if (result) {
    return query.data;
  }
  return std::vector<int16_t>();
}

// ------------- NodeManager / Dashel

void AsebaROS::sendMessage(const Aseba::Message &message) {
  hub.sendMessage(&message, false);
}

void AsebaROS::has_been_disconnected() {
  set_connected_target("");
  for (auto &it : asebaros_nodes) {
    it.second->set_connected(false);
  }
  has_updated_nodes();
  if (shutdown_on_unconnect) {
    LOG_WARN("Disconnected from target: will shutdown");
    shutdown();
  } else {
    LOG_WARN("Disconnected from target: will wait for reconnection");
    wait_for_connection();
  }
}

void AsebaROS::nodeConnected(unsigned nodeId) {
  std::unique_lock<std::mutex> lock(mutex);
  if (asebaros_nodes.count(nodeId)) {
    auto node = asebaros_nodes.at(nodeId);
    lock.unlock();
    if (!node->get_connected()) {
      if (reload_script_on_reconnect) {
        node->reload_script();
      }
      node->set_connected(true);
      has_updated_nodes();
    }
  }
}

void AsebaROS::nodeDisconnected(unsigned nodeId) {
  std::unique_lock<std::mutex> lock(mutex);
  if (asebaros_nodes.count(nodeId)) {
    LOG_INFO("Aseba node %d unconnected", nodeId);
    asebaros_nodes.at(nodeId)->set_connected(false);
    has_updated_nodes();
  }
}

void AsebaROS::nodeDescriptionReceived(unsigned nodeId) {
  // does not need locking, called by parent object
  std::string name = narrow(nodes.at(nodeId).name);
  std::string ns = namespace_for_node(name, nodeId);
  bool ignore = should_ignore_node(name, nodeId);
  bool include_id_in_events = nodes_configs.include_id_in_events.get_config(name, nodeId);
  LOG_INFO("Received %s description of an Aseba node with name %s and id %d",
           (nodes[nodeId].isComplete() ? "a complete" : "an uncomplete"),
           name.data(), nodeId);
  if (ignore) {
    LOG_INFO("Will ignore node %d", nodeId);
    return;
  }
  if (asebaros_nodes.count(nodeId)) {
    LOG_WARN("Will ignore description as node %d was already known", nodeId);
    return;
  }
  AsebaROSNode *node = add_asebaros_node(nodeId, name, ns, include_id_in_events);
  LOG_INFO("Has connected to a new Aseba node for with %d and namespace %s", nodeId, ns.c_str());
  sleep_for_ms(200);
  if (set_id_variable) {
    std::string variable = variable_id_for_node(name);
    if (!variable.empty()) {
      // This compensate partial remapping of asebaswitch
      node->set_variable(variable, nodeId, false);
    }
  }
  if (default_script) {
    node->load_script(default_script, false);
  } else {
    node->has_updated_description();
  }
  has_updated_nodes();
}

void AsebaROS::nodeProtocolVersionMismatch(unsigned nodeId, const std::wstring &nodeName,
                                           uint16_t protocolVersion) {
    if (protocolVersion < ASEBA_MIN_TARGET_PROTOCOL_VERSION) {
      LOG_WARN("Connected node %d of type %s: protocol version %d"
               " is lower than the minimal accepted version %d",
                nodeId, narrow(nodeName).c_str(), protocolVersion,
                ASEBA_MIN_TARGET_PROTOCOL_VERSION);
    } else {
      LOG_WARN("Connected node %d of type %s: protocol version %d"
               " is higher than the maximal accepted version %d",
                nodeId, narrow(nodeName).c_str(), protocolVersion,
                aseba_max_target_protocol_version);
    }
}

void AsebaROS::processAsebaMessage(Aseba::Message *message) {
  // scan this message for nodes descriptions

  Aseba::Description *description = dynamic_cast<Aseba::Description *>(message);
  if (description && description->protocolVersion > ASEBA_PROTOCOL_VERSION &&
      description->protocolVersion <= aseba_max_target_protocol_version) {
          description->protocolVersion = ASEBA_PROTOCOL_VERSION;
  }

  Aseba::NodesManager::processMessage(message);

  // needs locking, called by Dashel hub
  std::lock_guard<std::mutex> lock(mutex);

  // if user message, send to D-Bus as well
  Aseba::UserMessage *userMessage = dynamic_cast<Aseba::UserMessage *>(message);
  if (userMessage)
    forward_event_to_ros(userMessage);

  // if variables, check for pending answers
  Aseba::Variables *variables = dynamic_cast<Aseba::Variables *>(message);
  if (variables) {
    const GetVariableQueryKey queryKey(variables->source, variables->start);
    GetVariableQueryMap::const_iterator queryIt(
        getVariableQueries.find(queryKey));
    if (queryIt != getVariableQueries.end()) {
      queryIt->second->data = variables->variables;
      queryIt->second->cond.notify_one();
    } else {
      LOG_WARN("received Variables from node %d, pos %d, but no "
               "corresponding query was found",
               variables->source, variables->start);
    }
  }
}

// ------------- Main


// hub for dashel
AsebaROS::AsebaROS(unsigned port, bool forward) :
  hub(this, port, forward),
  connected_to(""),
  default_script(nullptr),
  aseba_max_target_protocol_version(ASEBA_PROTOCOL_VERSION) {
  xmlInitParser();
  Dashel::initPlugins();
}

AsebaROS::~AsebaROS() { xmlCleanupParser(); }

void AsebaROS::run() {
  hub.startThread();
  start_pinging();
}

void AsebaROS::stop() {
  set_connected_target("");
  if (reset_on_closing) {
    std::lock_guard<std::mutex> lock(mutex);
    for (const auto &it : asebaros_nodes) {
      Aseba::Reset msg_r(it.first);
      LOG_INFO("Reset Aseba node %d", it.first);
      // HACK(Jerome): try to get why sometimes ROS1 does not send the hub message
      // hub.sendMessage(&msg_r, true);
      hub.sendMessage(&msg_r, false);
      sleep_for_s(1);
    }
  }
  hub.stopThread();
}

#ifdef DIAGNOSTICS
void AsebaROS::update_diagnostics(
    diagnostic_updater::DiagnosticStatusWrapper &stat) {
  if (connected_to == "") {
    stat.summary(DiagnosticStatus::WARN, "Connecting");
    return;
  }
  stat.summary(DiagnosticStatus::OK, "Connected");
  std::vector<unsigned> connected_nodes;
  std::vector<unsigned> unconnected_nodes;
  std::vector<unsigned> ignored_nodes;
  for (const auto &node : nodes) {
    if (!asebaros_nodes.count(node.first)) {
      ignored_nodes.push_back(node.first);
    } else if (asebaros_nodes.at(node.first)->get_connected()) {
      connected_nodes.push_back(node.first);
    } else {
      unconnected_nodes.push_back(node.first);
    }
  }
  stat.add("Connected nodes", list_repr(connected_nodes));
  stat.add("Ignored nodes", list_repr(ignored_nodes));
  stat.add("Unconnected nodes", list_repr(unconnected_nodes));
}
#endif

void AsebaROS::update_script_constants(
    AsebaScript *script, bool set_parameters,
    const std::vector<ConstantMsg> &constants) {
  for (Aseba::NamedValue &constant : script->common_definitions.constants) {
    std::string constant_name = narrow(constant.name);
    const auto &it = std::find_if(std::begin(constants), std::end(constants),
                                  [constant_name](const ConstantMsg &c) {
                                    return c.name == constant_name;
                                  });
    if (it != std::end(constants)) {
      constant.value = it->value;
    }
    if (set_parameters) {
      set_constant_to_param(constant, constant_name);
    }
  }
}

void AsebaROS::get_anonymous_event_cb(const AnonymousEventMsgPtr &event) {
  // does not need locking, does not touch object's members
  if (event->source == 0) {
    // forward only messages with source 0, which means, originating from
    // this computer
    Aseba::UserMessage userMessage(event->type, event->data);
    hub.sendMessage(&userMessage, true);
  }
}

void AsebaROS::has_updated_nodes() {
  NodeListMsg msg;
  for (auto &it : asebaros_nodes) {
    msg.nodes.push_back(it.second->to_msg());
  }
  publish_node_list(msg);
}

bool AsebaROS::load_script(LoadScriptRequestPtr req, LoadScriptResponsePtr res) {
  LOG_INFO("Will try to read script from %s", req->file_name.c_str());
  auto script = AsebaScript::from_file(req->file_name);
  if (!script)
    return false;
  update_script_constants(script.get(), req->set_as_default, req->constants);
  std::lock_guard<std::mutex> lock(mutex);
  if (req->set_as_default) {
    LOG_INFO("Set as default script");
    set_script_param(req->file_name);
    default_script = script;
  }
  for (auto &it : asebaros_nodes) {
    if (req->node_ids.size() == 0 ||
        std::find(req->node_ids.begin(), req->node_ids.end(), it.first) !=
            req->node_ids.end()) {
      it.second->load_script(script, true);
    }
  }
  return true;
}

bool AsebaROS::get_node_list(GetNodeListRequestPtr req, GetNodeListResponsePtr res) {
  std::lock_guard<std::mutex> lock(mutex);
  for (const auto &it : asebaros_nodes) {
    res->nodes.push_back(it.second->to_msg());
  }
  for (const auto &it : nodes) {
    if (asebaros_nodes.count(it.first) == 0) {
      NodeMsg msg;
      msg.id = it.first;
      msg.ignored = true;
      res->nodes.push_back(msg);
    }
  }
  return true;
}
