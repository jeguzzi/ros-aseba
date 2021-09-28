#include "asebaros/asebaros_node.h"
#include "asebaros/asebaros_node_manager.h"
#include "asebaros/utils.h"

typedef std::vector<std::unique_ptr<Aseba::Message>> MessageVector;

void AsebaROSNode::set_connected(bool value) { is_connected = value; }

void AsebaROSNode::reload_script() {
  if (script != nullptr) {
    load_script(script, false);
  }
}

bool AsebaROSNode::load_script(const std::shared_ptr<AsebaScript> new_script,
                               bool should_lock) {
  bool success = false;
  if (!new_script)
    return false;
  Aseba::VariablesMap user_variables;
  Aseba::BytecodeVector bytecode;
  success = new_script->compile(id, description, user_variables, bytecode);
  if (!success)
    return false;

  MessageVector messages;
  auto bytes = std::vector<uint16_t>(bytecode.begin(), bytecode.end());
  sendBytecode(messages, id, bytes);
  for (auto it = messages.begin(); it != messages.end(); ++it) {
    hub->sendMessage((*it).get(), should_lock);
  }
  Aseba::Run msg(id);
  hub->sendMessage(&msg, should_lock);
  LOG_INFO("Loaded script (%lu bytes) to node %d ",
           bytecode.end() - bytecode.begin(), id);
  unsigned _;
  // std::unique_lock<std::mutex> lock(mutex);
  std::unique_lock<std::mutex> lock(mutex);
  script = new_script;
  variables = description->getVariablesMap(_);
  variables.insert(user_variables.begin(), user_variables.end());
  create_subscribers();
  reset_publishers();
  has_updated_description();
  return true;
}

std::string AsebaROSNode::ros_name(const std::string &name) const {
  return (namespace_.size() ? namespace_ + "/" : "") + "aseba/" + name;
}

void AsebaROSNode::set_variable(const std::string &name,
                                const std::vector<int16_t> &value,
                                bool should_lock) {
  std::wstring wname = widen(name);
  // lock the access to the member methods
  unsigned position;
  {
    // std::shared_lock<std::mutex> lock(mutex);
    std::unique_lock<std::mutex> lock(mutex);
    if (variables.count(wname)) {
      position = variables.at(wname).first;
    } else {
      return;
    }
  }
  Aseba::SetVariables msg(id, position, value);
  hub->sendMessage(&msg, should_lock);
}

void AsebaROSNode::set_variable(const std::string &name, int16_t value,
                                bool should_lock) {
  std::vector<int16_t> data{value};
  set_variable(name, data, should_lock);
}

std::vector<int16_t> AsebaROSNode::get_variable(const std::string &name) {
  unsigned position, length;
  // lock the access to the member methods, wait will unlock the underlying
  // mutex
  std::wstring wname = widen(name);
  {
    // get information about variable
    std::unique_lock<std::mutex> lock(mutex);
    // std::shared_lock<std::mutex> lock(mutex);
    if (variables.count(wname)) {
      position = variables.at(wname).first;
      length = variables.at(wname).second;
    } else {
      LOG_WARN("Unknown variable %s of node %d", name.c_str(), id);
      return {};
    }
  }
  auto data = manager->query_variable(id, position, length);
  if (!data.size()) {
    LOG_ERROR("querying variable %s of node %d did not return a valid answer "
              "within 100ms",
              name.c_str(), id);
  }
  return data;
}

void AsebaROSNode::fill_description(NodeDescriptionMsg * msg) {
  msg->id = id;
  msg->name = narrow(description->name);
  // std::shared_lock<std::shared_timed_mutex> lock(mutex);
  if (script) {
    auto &cs = script->common_definitions.constants;
    auto &es = script->common_definitions.events;
    std::transform(
        cs.begin(), cs.end(), std::back_inserter(msg->constants),
        [](Aseba::NamedValue constant) -> ConstantMsg {
          ConstantMsg cmsg;
          cmsg.name = narrow(constant.name);
          cmsg.value = constant.value;
          return cmsg;
        });
    std::transform(es.begin(), es.end(), std::back_inserter(msg->events),
                   [](Aseba::NamedValue event) -> std::string {
                     return narrow(event.name);
                   });
  }
  std::transform(
      variables.begin(), variables.end(), std::back_inserter(msg->variables),
      [](std::pair<const std::wstring, std::pair<unsigned, unsigned>> &p)
          -> std::string { return narrow(p.first); });
}

void AsebaROSNode::get_description(GetDescriptionRequestPtr req, GetDescriptionResponsePtr res) {
  std::unique_lock<std::mutex> lock(mutex);
  fill_description(&(res->description));
}

// ROS Callbacks
void AsebaROSNode::got_event_message_cb(
    const uint16_t event_id, const EventMsgPtr & msg) {
  if (msg->source == 0) {
    LOG_DEBUG("known event %d received from ROS for node %d", event_id, id);
    // forward only messages with source 0, which means, originating from this
    // computer
    Aseba::VariablesDataVector data = msg->data;
    if (include_id_in_events) {
      data.insert(data.begin(), id);
    }
    Aseba::UserMessage userMessage(event_id, data);
    hub->sendMessage(&userMessage, true);
  }
}

NodeMsg AsebaROSNode::to_msg() const {
  NodeMsg msg;
  msg.id = id;
  msg.name_space = namespace_;
  // msg.name_space = absolute_namespace(namespace_);
  msg.name = narrow(description->name);
  msg.ignored = false;
  msg.running = (script != nullptr);
  msg.connected = is_connected;
  return msg;
}

#ifdef DIAGNOSTICS
void AsebaROSNode::update_diagnostics(
    diagnostic_updater::DiagnosticStatusWrapper &stat) {
  if (is_connected) {
    stat.summary(DiagnosticStatus::OK, "Connected");
  } else {
    stat.summary(DiagnosticStatus::WARN, "Not Connected");
  }
  stat.add("Type", name);
  stat.add("Id", id);
  stat.add("ROS Namespace", absolute_namespace(namespace_));
  stat.add("Loaded script", script ? script->source : "-");
}
#endif

void AsebaROSNode::has_updated_description() {
  NodeDescriptionMsg msg;
  fill_description(&msg);
  publish_description(msg);
}
