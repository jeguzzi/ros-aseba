#ifndef ASEBAROS_INCLUDE_ASEBAROS_ASEBAROSNODEMANAGER_H_
#define ASEBAROS_INCLUDE_ASEBAROS_ASEBAROSNODEMANAGER_H_

#include <condition_variable>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <iomanip>

#if DIAGNOSTICS
#if (ROS_VERSION_M == 2)
#include <diagnostic_updater/diagnostic_updater.hpp>
using diagnostic_msgs::msg::DiagnosticStatus;
#else
#include "diagnostic_updater/diagnostic_updater.h"
using diagnostic_msgs::DiagnosticStatus;
#endif
#endif

#if (ROS_VERSION_M == 2)
#include "asebaros_msgs/msg/constant.hpp"
#include "asebaros_msgs/msg/anonymous_event.hpp"
#include "asebaros_msgs/msg/node_list.hpp"
#include "asebaros_msgs/srv/get_node_list.hpp"
#include "asebaros_msgs/srv/load_script.hpp"
typedef asebaros_msgs::msg::Constant ConstantMsg;
typedef asebaros_msgs::msg::AnonymousEvent::SharedPtr AnonymousEventMsgPtr;
typedef asebaros_msgs::msg::NodeList NodeListMsg;
typedef std::shared_ptr<asebaros_msgs::srv::GetNodeList::Request> GetNodeListRequestPtr;
typedef std::shared_ptr<asebaros_msgs::srv::GetNodeList::Response> GetNodeListResponsePtr;
typedef std::shared_ptr<asebaros_msgs::srv::LoadScript::Request> LoadScriptRequestPtr;
typedef std::shared_ptr<asebaros_msgs::srv::LoadScript::Response> LoadScriptResponsePtr;
#else
#include "asebaros_msgs/Constant.h"
#include "asebaros_msgs/AnonymousEvent.h"
#include "asebaros_msgs/NodeList.h"
#include "asebaros_msgs/GetNodeList.h"
#include "asebaros_msgs/LoadScript.h"
typedef asebaros_msgs::Constant ConstantMsg;
typedef asebaros_msgs::AnonymousEventConstPtr AnonymousEventMsgPtr;
typedef asebaros_msgs::NodeList NodeListMsg;
typedef asebaros_msgs::GetNodeList::Request * GetNodeListRequestPtr;
typedef asebaros_msgs::GetNodeList::Response * GetNodeListResponsePtr;
typedef asebaros_msgs::LoadScript::Request * LoadScriptRequestPtr;
typedef asebaros_msgs::LoadScript::Response * LoadScriptResponsePtr;
#endif  // ROS_VERSION


#include "common/msg/NodesManager.h"
#include "common/msg/TargetDescription.h"
#include "common/msg/msg.h"
#include "compiler/compiler.h"

#include "aseba_dashel_hub.h"
#include "asebaros_node.h"
#include "aseba_script.h"

class AsebaROS : public Aseba::NodesManager {
  friend class AsebaROSNode;

protected:
  template <typename T> struct NodesConfig {
    std::map<std::string, std::map<int, T>> config;
    const T default_value;
    NodesConfig(T default_value) : config(), default_value(default_value){};

    T get_config(std::string name, int id) {
      if (!config.count(name)) {
        if (name.empty()) {
          return default_value;
        }
        return get_config("", id);
      }
      if (!config.at(name).count(id)) {
        if (id == -1) {
          // This should not happen
          LOG_ERROR("Could not get config for %s", name.c_str());
          return default_value;
        }
        return get_config(name, -1);
      }
      return config.at(name).at(id);
    }

    void set_config(std::string name, int id, T value) {
      config[name][id] = value;
    }

    std::string description() const {
      std::stringstream desc;
      bool has_bottom = false;
      for (const auto &i : config) {
        std::string type = i.first.size() ? i.first : "*";
        for (const auto &j : i.second) {
          std::string id = j.first >= 0 ? std::to_string(j.first) : "*";
          desc << std::right << std::setw (10) << type << " | " << std::setw (10) << std::left << id << j.second << std::endl;
          if(type=="*" && id == "*") {
            has_bottom = true;
          }
        }
      }
      if (!has_bottom) {
        std::string type = "*";
        desc << std::right << std::setw (10) << type << " | " << std::setw (10) << std::left << type << default_value << std::endl;
      }
      return desc.str();
    }
  };

  struct NodesConfigs {
    NodesConfig<std::string> prefix;
    NodesConfig<std::string> name;
    NodesConfig<std::string> id_variable;
    NodesConfig<int> maximal_number_of_nodes;
    NodesConfig<bool> include_id_in_events;
    NodesConfig<bool> accept;
    NodesConfigs() :
      prefix(""), name(""), id_variable(""),
      maximal_number_of_nodes(-1), include_id_in_events(false), accept(true) {};

    std::string description ( ) const {
      std::stringstream desc;
      desc << "accept:" << std::endl << accept.description();
      desc << "maximal number:" << std::endl <<  maximal_number_of_nodes.description();
      desc << "include id in events:" << std::endl <<  include_id_in_events.description();
      desc << "name:" << std::endl << name.description();
      desc << "prefix:" << std::endl << prefix.description();
      desc << "id variable:" << std::endl << id_variable.description();
      return desc.str();
    }

  };



  NodesConfigs nodes_configs;

  class GetVariableQueryKey {
  public:
    GetVariableQueryKey(unsigned nodeId, unsigned pos)
        : nodeId(nodeId), pos(pos) {}
    bool operator<(const GetVariableQueryKey &that) const {
      return (nodeId < that.nodeId && pos < that.pos);
    }
    unsigned nodeId;
    unsigned pos;
  };
  struct GetVariableQueryValue {
    typedef std::vector<int16_t> DataVector;
    DataVector data;
    std::condition_variable cond;
  };
  typedef std::map<GetVariableQueryKey, GetVariableQueryValue *>
      GetVariableQueryMap;
  GetVariableQueryMap getVariableQueries;

  std::map<unsigned, std::shared_ptr<AsebaROSNode>> asebaros_nodes;
  /// hub is the network interface for dashel peers
  AsebaDashelHub hub;
  /// mutex for protecting accesses from hub
  std::mutex mutex;

  std::string connected_to;
  std::shared_ptr<AsebaScript> default_script;

  // parameters
  bool shutdown_on_unconnect;
  bool reset_on_closing;
  bool set_id_variable;
  bool reload_script_on_reconnect;
  uint16_t aseba_max_target_protocol_version;

protected:
  int number_of_nodes(const std::string type);
  void has_updated_nodes();
  void update_script_constants(
      AsebaScript *script, bool set_parameters,
      const std::vector<ConstantMsg> &constants);
  void forward_event_to_ros(const Aseba::UserMessage *asebaMessage);
  // Node manager callbacks
  virtual void sendMessage(const Aseba::Message &message);
  virtual void nodeDescriptionReceived(unsigned nodeId);
  void nodeConnected(unsigned nodeId);
  void nodeDisconnected(unsigned nodeId);
  // Utilities
  std::string namespace_for_node(const std::string &kind, unsigned id);
  std::string variable_id_for_node(const std::string &kind, unsigned id = -1);
  bool should_ignore_node(const std::string &kind, unsigned id);
  std::vector<std::string> additionalTargets;
  std::vector<int16_t> query_variable(unsigned nodeId, unsigned pos,
                                      unsigned length);

  void update();
  void connectTarget(const std::string &target) { hub.connect(target); }
  void read_default_script_from_path(const std::string &script_path);
  void log_initialized();
  // ROS1-2 specific [virtual] methods
  virtual void start_pinging() = 0;
  virtual bool set_constant_from_param(Aseba::NamedValue &constant,
                                       const std::string &param_name) = 0;
  virtual void set_constant_to_param(const Aseba::NamedValue &constant,
                                     const std::string &param_name) = 0;
  virtual std::string init_params() = 0;
  virtual void import_node_config(const std::string &prefix) = 0;
  virtual AsebaROSNode *add_asebaros_node(unsigned id, const std::string &name,
                                          const std::string &ns, bool include_id_in_events) = 0;
  virtual void publish_node_list(const NodeListMsg & msg) = 0;
  virtual void set_script_param(const std::string & path) = 0;
  virtual void
  publish_anonymous_event(const Aseba::UserMessage *aseba_message) = 0;

  void update_script_constants(AsebaScript *script, bool set_parameters);
#if DIAGNOSTICS
  void update_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);
#endif
  void get_anonymous_event_cb(const AnonymousEventMsgPtr &event);
  bool get_node_list(GetNodeListRequestPtr req, GetNodeListResponsePtr res);
  bool load_script(LoadScriptRequestPtr req, LoadScriptResponsePtr res);
  void nodeProtocolVersionMismatch(unsigned nodeId, const std::wstring &nodeName,
                                   uint16_t protocolVersion);
public:
  // dashel callbacks
  void processAsebaMessage(Aseba::Message *message);
  void has_been_disconnected();
  // main
  AsebaROS(unsigned port, bool forward);
  ~AsebaROS();
  void run();
  void stop();
  bool wait_for_connection();
  virtual void set_connected_target(const std::string &target);
};

#endif // ASEBAROS_INCLUDE_ASEBAROS_ASEBAROSNODEMANAGER_H_
