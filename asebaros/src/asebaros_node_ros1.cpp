#include "asebaros/asebaros_node_ros1.h"
#include "asebaros/asebaros_node_manager_ros1.h"
#include "asebaros/utils.h"

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

AsebaROS1Node::AsebaROS1Node(AsebaDashelHub *hub, AsebaROS1 *manager,
                             const unsigned id, const std::string name,
                             const Aseba::TargetDescription *description,
                             const std::string &namespace_,
                             bool include_id_in_events, bool is_connected)
    : AsebaROSNode(hub, manager, id, name, description, namespace_,
                   include_id_in_events, is_connected),
      pubs(), subs(), services(), n(""), nh("~"),
      desc_pub(n.advertise<NodeDescriptionMsg>(ros_name("description"), 1, true)) {
  services.push_back(n.advertiseService(
      ros_name("get_description"), &AsebaROS1Node::get_description_cb, this));
  services.push_back(n.advertiseService(ros_name("set_variable"),
                                        &AsebaROS1Node::set_variable_cb, this));
  services.push_back(n.advertiseService(ros_name("get_variable"),
                                        &AsebaROS1Node::get_variable_cb, this));
}

bool AsebaROS1Node::publish_event(const Aseba::UserMessage *aseba_message) {
  auto pub = get_publisher_for(aseba_message);
  if (pub) {
    asebaros_msgs::Event event;
    event.stamp = ros::Time::now();
    event.source = aseba_message->source;
    event.data = aseba_message->data;
    pub->publish(event);
    return true;
  }
  return false;
}

void AsebaROS1Node::reset_publishers() { pubs.clear(); }

void AsebaROS1Node::create_subscribers() {
  subs.clear();
  unsigned i = 0;
  for (const auto &event : script->common_definitions.events) {
    subs.push_back(n.subscribe<asebaros_msgs::Event>(
        ros_name(EVENTS_NS + narrow(event.name)), 100,
        [this, i](const asebaros_msgs::EventConstPtr &event) {
          got_event_message_cb(i, event);
        }));
    i++;
  }
}

ros::Publisher *
AsebaROS1Node::get_publisher_for(const Aseba::UserMessage *asebaMessage) {
  unsigned type = asebaMessage->type;
  // known, send on a named channel
  std::unique_lock<std::mutex> lock(mutex);
  // std::shared_lock<std::shared_timed_mutex> lock(mutex);
  if (pubs.count(type) == 0) {
    if (!(script &&
          (asebaMessage->type < script->common_definitions.events.size()))) {
      return nullptr;
    } else {
      const std::wstring &name = script->common_definitions.events[type].name;
      pubs[type] = n.advertise<asebaros_msgs::Event>(
          ros_name(EVENTS_NS + narrow(name)), 100);
    }
  }
  return &(pubs[type]);
}

bool AsebaROS1Node::set_variable_cb(asebaros_msgs::SetVariable::Request &req,
                                    asebaros_msgs::SetVariable::Response &res) {
  set_variable(req.variable, req.data, true);
  return true;
}
bool AsebaROS1Node::get_variable_cb(asebaros_msgs::GetVariable::Request &req,
                                    asebaros_msgs::GetVariable::Response &res) {
  res.data = get_variable(req.variable);
  return true;
}

bool AsebaROS1Node::get_description_cb(
    asebaros_msgs::GetDescription::Request &req,
    asebaros_msgs::GetDescription::Response &res) {
  get_description(&req, &res);
  return true;
}

void AsebaROS1Node::publish_description(const NodeDescriptionMsg & msg) {
  desc_pub.publish(msg);
}

std::string AsebaROS1Node::absolute_namespace(const std::string & name) const {
  return n.resolveName(name);
}
