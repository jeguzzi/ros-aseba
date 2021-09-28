#ifndef ASEBAROS_INCLUDE_ASEBAROS_ASEBAROSNODE_H
#define ASEBAROS_INCLUDE_ASEBAROS_ASEBAROSNODE_H

#include <mutex>
// #include <shared_mutex>
#include <string>
#include <vector>

#if DIAGNOSTICS
#if (ROS_VERSION_M == 2)
#include <diagnostic_updater/diagnostic_updater.hpp>
using diagnostic_msgs::msg::DiagnosticStatus;
#else
#include "diagnostic_updater/diagnostic_updater.h"
using diagnostic_msgs::DiagnosticStatus;
#endif
#endif

#include "common/msg/NodesManager.h"
#include "common/msg/msg.h"

#include "aseba_script.h"
#include "utils.h"
#include "aseba_dashel_hub.h"

#if (ROS_VERSION_M == 2)
#include "asebaros_msgs/msg/event.hpp"
#include "asebaros_msgs/msg/node.hpp"
#include "asebaros_msgs/msg/constant.hpp"
#include "asebaros_msgs/msg/node_description.hpp"
#include "asebaros_msgs/srv/get_description.hpp"
typedef asebaros_msgs::msg::Event EventMsg;
typedef asebaros_msgs::msg::Node NodeMsg;
typedef asebaros_msgs::msg::Event::SharedPtr EventMsgPtr;
typedef asebaros_msgs::msg::Constant ConstantMsg;
typedef asebaros_msgs::msg::NodeDescription NodeDescriptionMsg;
typedef std::shared_ptr<asebaros_msgs::srv::GetDescription::Request> GetDescriptionRequestPtr;
typedef std::shared_ptr<asebaros_msgs::srv::GetDescription::Response> GetDescriptionResponsePtr;
#else
#include "asebaros_msgs/Event.h"
#include "asebaros_msgs/Node.h"
#include "asebaros_msgs/Constant.h"
#include "asebaros_msgs/GetDescription.h"
#include "asebaros_msgs/NodeDescription.h"
typedef asebaros_msgs::Event EventMsg;
typedef asebaros_msgs::Node NodeMsg;
typedef asebaros_msgs::EventConstPtr EventMsgPtr;
typedef asebaros_msgs::Constant ConstantMsg;
typedef asebaros_msgs::NodeDescription NodeDescriptionMsg;
typedef asebaros_msgs::GetDescription::Request * GetDescriptionRequestPtr;
typedef asebaros_msgs::GetDescription::Response * GetDescriptionResponsePtr;
#endif  // ROS_VERSION


#define EVENTS_NS "events/"

class AsebaROS;

class AsebaROSNode {

public:
  AsebaROSNode(AsebaDashelHub * hub, AsebaROS * manager, const unsigned id, const std::string name,
               const Aseba::TargetDescription *description,
               const std::string &namespace_,
               bool include_id_in_events, bool is_connected = true)
      : hub(hub), manager(manager), id(id), name(name), namespace_(namespace_),
        description(description), script(nullptr),
        include_id_in_events(include_id_in_events), is_connected(is_connected) {
    unsigned _;
    variables = description->getVariablesMap(_);
  };

  NodeMsg to_msg() const;
  void has_updated_description();

#if DIAGNOSTICS
  void update_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);
#endif
  bool load_script(const std::shared_ptr<AsebaScript> script, bool should_lock);
  void reload_script();
  void set_connected(bool);
  bool get_connected() {
    return is_connected;
  }
  void set_variable(const std::string &name, const std::vector<int16_t> &value,
                    bool should_lock = true);
  void set_variable(const std::string &name, int16_t value,
                    bool should_lock = true);
  virtual bool publish_event(const Aseba::UserMessage *aseba_message) = 0;
  std::string type() const {
    return name;
  }
protected:
  AsebaDashelHub * hub;
  AsebaROS * manager;
  const unsigned id;
  const std::string name;
  const std::string namespace_;
  const Aseba::TargetDescription *description;
  std::shared_ptr<AsebaScript> script;
  bool include_id_in_events;
  bool is_connected;
  Aseba::VariablesMap variables;
  mutable std::mutex mutex;


  // mutable std::shared_timed_mutex mutex;
  std::string ros_name(const std::string &topic_name) const;
  void fill_description(NodeDescriptionMsg * msg);

  void get_description(GetDescriptionRequestPtr req, GetDescriptionResponsePtr res);
  std::vector<int16_t> get_variable(const std::string &name);
  void got_event_message_cb(const uint16_t event_id,
                            const EventMsgPtr & msg);

  virtual std::string absolute_namespace(const std::string &) const = 0;
  virtual void create_subscribers() = 0;
  virtual void reset_publishers() = 0;
  virtual void publish_description(const NodeDescriptionMsg & msg) = 0;
};

#endif // ASEBAROS_INCLUDE_ASEBAROS_ASEBAROSNODE_H
