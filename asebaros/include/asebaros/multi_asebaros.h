#ifndef ASEBAROS_INCLUDE_ASEBAROS_ASEBAROS_H_
#define ASEBAROS_INCLUDE_ASEBAROS_ASEBAROS_H_

#include <map>
#include <vector>
#include <string>
#include <memory>
#include <thread>
#include <mutex>

#include "dashel/dashel.h"
#include "common/msg/msg.h"
#include "common/msg/NodesManager.h"
#include "compiler/compiler.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "asebaros_msgs/msg/aseba_event.hpp"
#include "asebaros_msgs/msg/aseba_anonymous_event.hpp"
#include "asebaros_msgs/msg/aseba_node_list.hpp"
#include "asebaros_msgs/msg/aseba_node.hpp"
#include "asebaros_msgs/srv/load_scripts.hpp"
#include "asebaros_msgs/srv/get_node_list.hpp"
#include "asebaros_msgs/srv/get_node_ids.hpp"
#include "asebaros_msgs/srv/get_node_name.hpp"
#include "asebaros_msgs/srv/get_variable_list.hpp"
#include "asebaros_msgs/srv/set_variable.hpp"
#include "asebaros_msgs/srv/get_variable.hpp"
#include "asebaros_msgs/srv/get_event_id.hpp"
#include "asebaros_msgs/srv/get_event_name.hpp"

using asebaros_msgs::srv::LoadScripts;
using asebaros_msgs::srv::GetNodeList;
using asebaros_msgs::srv::GetNodeIds;
using asebaros_msgs::srv::GetNodeName;
using asebaros_msgs::srv::GetVariableList;
using asebaros_msgs::srv::SetVariable;
using asebaros_msgs::srv::GetVariable;
using asebaros_msgs::srv::GetVariableList;
using asebaros_msgs::srv::GetEventId;
using asebaros_msgs::srv::GetEventName;
using asebaros_msgs::msg::AsebaEvent;
using asebaros_msgs::msg::AsebaAnonymousEvent;
using asebaros_msgs::msg::AsebaNodeList;
using asebaros_msgs::msg::AsebaNode;

class AsebaROS;

class AsebaDashelHub: public Dashel::Hub {
 private:
  std::unique_ptr<std::thread> thread;  //! thread for the hub
  AsebaROS* asebaROS;  //!< pointer to aseba ROS
  bool forward;  //!< should we only forward messages instead of transmit them back to the sender

 public:
  /*! Creates the hub, listen to TCP on port, and creates a DBus interace.
  @param port port on which to listen for incoming connections
  @param forward should we only forward messages instead of transmit them back to the sender
  */
  AsebaDashelHub(AsebaROS* asebaROS, unsigned port, bool forward);

  /*! Sends a message to Dashel peers.
  Does not delete the message, should be called by the main thread.
  @param message aseba message to send
  @param sourceStream originate of the message, if from Dashel.
  */
  void sendMessage(const Aseba::Message *message, bool doLock, Dashel::Stream* sourceStream = 0);

  //! run the hub
  void operator()();
  //! start the hub thread
  void startThread();
  //! stop the hub thread and wait for its termination
  void stopThread();

 protected:
  virtual void connectionCreated(Dashel::Stream *stream);
  virtual void incomingData(Dashel::Stream *stream);
  virtual void connectionClosed(Dashel::Stream *stream, bool abnormal);
};


typedef std::vector<std::shared_ptr<rclcpp::ServiceBase>> ServiceServers;
// typedef std::vector<std::shared_ptr<rclcpp::Publisher<AsebaEvent>>> Publishers;
// typedef std::vector<std::shared_ptr <rclcpp::Subscription<AsebaEvent>>> Subscribers;

typedef std::vector<std::map<unsigned,std::shared_ptr<rclcpp::Publisher<AsebaEvent>>>> Publishers;
typedef std::vector<std::map<unsigned,std::shared_ptr <rclcpp::Subscription<AsebaEvent>>>> Subscribers;
typedef std::vector<std::unique_ptr<Aseba::Message>> MessageVector;

class AsebaROS: public Aseba::NodesManager, public rclcpp::Node {
 protected:
  typedef std::map<std::string, unsigned> NodesNamesMap;
  typedef std::map<std::string, Aseba::VariablesMap> UserDefinedVariablesMap;
  class GetVariableQueryKey {
   public:
    GetVariableQueryKey(unsigned nodeId, unsigned pos) : nodeId(nodeId), pos(pos) { }
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

  typedef std::map<GetVariableQueryKey, GetVariableQueryValue*> GetVariableQueryMap;

  ServiceServers s;  //!< all services of this class

  //!< anonymous publisher, for aseba events with no associated name
  std::shared_ptr<rclcpp::Publisher<AsebaAnonymousEvent>> anonPub;
  //!< anonymous subscriber, for aseba events with no associated name
  std::shared_ptr<rclcpp::Subscription<AsebaAnonymousEvent>> anonSub;
  rclcpp::TimerBase::SharedPtr timer;
  Publishers pubs;  //!< publishers for known events
  Subscribers subs;  //!< subscribers for known events

  AsebaDashelHub hub;  //!< hub is the network interface for dashel peers


  Aseba::CommonDefinitions commonDefinitions;  //!< description of aseba constants and events
  NodesNamesMap nodesNames;  //!< the name of all nodes
  UserDefinedVariablesMap userDefinedVariablesMap;  //!< the name of the user-defined variables
  GetVariableQueryMap getVariableQueries;  //!< all get variable queries

  bool shutdown_on_unconnect;

 protected:
  void loadScript(
    const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<LoadScripts::Request> req,
  const std::shared_ptr<LoadScripts::Response> res);

  void getNodeList(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<GetNodeList::Request> req,
  const std::shared_ptr<GetNodeList::Response> res);

  void getNodeIds(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<GetNodeIds::Request> req,
  const std::shared_ptr<GetNodeIds::Response> res);

  void getNodeName(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<GetNodeName::Request> req,
  const std::shared_ptr<GetNodeName::Response> res);

  void getVariableList(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<GetVariableList::Request> req,
  const std::shared_ptr<GetVariableList::Response> res);

  void setVariable(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<SetVariable::Request> req,
  const std::shared_ptr<SetVariable::Response> res);

  void getVariable(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<GetVariable::Request> req,
  const std::shared_ptr<GetVariable::Response> res);

  void getEventId(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<GetEventId::Request> req,
  const std::shared_ptr<GetEventId::Response> res);

  void getEventName(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<GetEventName::Request> req,
  const std::shared_ptr<GetEventName::Response> res);

  // utility
  bool getNodePosFromNames(const std::string& nodeName, const std::string& variableName,
  unsigned* nodeId, unsigned* pos) const;
  void sendEventOnROS(const Aseba::UserMessage* asebaMessage);

  // callbacks
  virtual void sendMessage(const Aseba::Message& message);
  void nodeDescriptionReceived(unsigned nodeId);
  void eventReceived(const AsebaAnonymousEvent::SharedPtr msg);
  void knownEventReceived(const uint16_t id, const uint16_t nodeId, const AsebaEvent::SharedPtr event);

  // void knownEventReceived(const AsebaEvent::SharedPtr event);

  void create_subscribers(unsigned nodeId);
  void updateConstantsFromROS();

  std::vector<unsigned> get_node_ids(const std::wstring& name);
  std::shared_ptr<rclcpp::Publisher<AsebaEvent>> pubFor(const Aseba::UserMessage* asebaMessage);
  void loadScriptToTarget(unsigned nodeId);
  void compile_script();
  void read_script_header();
  void load(Aseba::BytecodeVector&, unsigned int);
  void load_script_to(unsigned int);
  void load_script_to(std::vector<uint16_t>);
  void publish_nodes();
  std::map <std::string, Aseba::BytecodeVector> bytecode;
  std::string node_name(unsigned int);
  std::string script_path;
  std::set<uint16_t> running_nodes;
  std::shared_ptr<rclcpp::Publisher<AsebaNodeList>> nodes_pub;

  std::map<unsigned, std::string> namespaces;
  std::string topic(unsigned id, const std::string & name);
  std::string namespace_for_node(unsigned id, std::string kind = std::string(""));
  bool ignore_node(unsigned int);
  bool manage_single_node;

 public:
  AsebaROS(unsigned port, bool forward, rclcpp::NodeOptions);
  ~AsebaROS();
  void pingCallback();
  void run();
  void processAsebaMessage(Aseba::Message *message);
  void unconnect();
  void stopAllNodes();
  void connectTarget(const std::string& target) { hub.connect(target); }
  std::mutex mutex;  //!< mutex for protecting accesses from hub
};

#endif  // ASEBAROS_INCLUDE_ASEBAROS_ASEBAROS_H_
