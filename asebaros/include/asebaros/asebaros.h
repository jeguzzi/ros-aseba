#ifndef ASEBAROS_INCLUDE_ASEBAROS_ASEBAROS_H_
#define ASEBAROS_INCLUDE_ASEBAROS_ASEBAROS_H_

#include <condition_variable>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "asebaros_msgs/AsebaAnonymousEvent.h"
#include "asebaros_msgs/AsebaEvent.h"
#include "asebaros_msgs/AsebaConstant.h"
#include "asebaros_msgs/AsebaNodeList.h"
#include "asebaros_msgs/GetEventId.h"
#include "asebaros_msgs/GetEventName.h"
#include "asebaros_msgs/GetNodeIds.h"
#include "asebaros_msgs/GetNodeList.h"
#include "asebaros_msgs/GetNodeName.h"
#include "asebaros_msgs/GetVariable.h"
#include "asebaros_msgs/GetVariableList.h"
#include "asebaros_msgs/LoadScripts.h"
#include "asebaros_msgs/SetVariable.h"
#include "common/msg/NodesManager.h"
#include "common/msg/msg.h"
#include "compiler/compiler.h"
#include "dashel/dashel.h"
#include "diagnostic_updater/diagnostic_updater.h"
#include "diagnostic_updater/publisher.h"
#include "ros/ros.h"
#include "std_msgs/String.h"

using namespace asebaros_msgs;

class AsebaROS;

class AsebaDashelHub : public Dashel::Hub {
private:
  /// thread for the hub
  std::unique_ptr<std::thread> thread;
  /// pointer to aseba ROS
  AsebaROS *asebaROS;
  /// should we only forward messages instead of transmit them back to the
  /// sender
  bool forward;

public:
  /**
   * Creates the hub, listen to TCP on port, and creates a DBus interace.
   * @param port     port on which to listen for incoming connections
   * @param forward  should we only forward messages instead of transmit them
   * back to the sender
   */
  AsebaDashelHub(AsebaROS *asebaROS, unsigned port, bool forward);

  /** Sends a message to Dashel peers.
   * Does not delete the message, should be called by the main thread.
   * @param   message aseba message to send
   * @param   sourceStream originate of the message, if from Dashel.
   */
  void sendMessage(const Aseba::Message *message, bool doLock,
                   Dashel::Stream *sourceStream = 0);

  /// run the hub
  void operator()();
  /// start the hub thread
  void startThread();
  /// stop the hub thread and wait for its termination
  void stopThread();

protected:
  virtual void connectionCreated(Dashel::Stream *stream);
  virtual void incomingData(Dashel::Stream *stream);
  virtual void connectionClosed(Dashel::Stream *stream, bool abnormal);
};

class AsebaROS : public Aseba::NodesManager {
  typedef std::map<std::string, unsigned> NodesNamesMap;
  typedef std::map<std::string, Aseba::VariablesMap> UserDefinedVariablesMap;
  typedef std::vector<ros::ServiceServer> ServiceServers;
  typedef std::vector<std::map<unsigned, ros::Publisher>> Publishers;
  typedef std::vector<std::map<unsigned, ros::Subscriber>> Subscribers;
  typedef std::vector<std::unique_ptr<Aseba::Message>> MessageVector;

protected:
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

  /// node handler of this class
  ros::NodeHandle n;
  ros::NodeHandle nh;
  /// all services of this class
  ServiceServers s;
  /// anonymous publisher, for aseba events with no associated name
  ros::Publisher anonPub;
  /// anonymous subscriber, for aseba events with no associated name
  ros::Subscriber anonSub;
  /// publishers for known events
  Publishers pubs;
  /// subscribers for known events
  Subscribers subs;

  /// hub is the network interface for dashel peers
  AsebaDashelHub hub;
  /// mutex for protecting accesses from hub
  std::mutex mutex;

  /// description of aseba constants and events
  Aseba::CommonDefinitions commonDefinitions;
  /// the name of all nodes
  NodesNamesMap nodesNames;
  /// the name of the user-defined variables
  UserDefinedVariablesMap userDefinedVariablesMap;
  /// all get variable queries
  GetVariableQueryMap getVariableQueries;

  bool shutdown_on_unconnect;
  // TODO(Jerome): review -> no more fanout in ros2
  bool fanout;

protected:
  bool loadScript(LoadScripts::Request &req, LoadScripts::Response &res);
  bool getNodeList(GetNodeList::Request &req, GetNodeList::Response &res);
  bool getNodeIds(GetNodeIds::Request &req, GetNodeIds::Response &res);
  bool getNodeName(GetNodeName::Request &req, GetNodeName::Response &res);
  bool getVariableList(GetVariableList::Request &req,
                       GetVariableList::Response &res);
  bool setVariable(SetVariable::Request &req, SetVariable::Response &res);
  bool getVariable(GetVariable::Request &req, GetVariable::Response &res);
  bool getEventId(GetEventId::Request &req, GetEventId::Response &res);
  bool getEventName(GetEventName::Request &req, GetEventName::Response &res);

  // utility
  bool getVariableInfo(const std::string &nodeName,
                       const std::string &variableName, unsigned &nodeId,
                       unsigned &pos, unsigned &length) const;
  void sendEventOnROS(const Aseba::UserMessage *asebaMessage);

  // callbacks
  virtual void sendMessage(const Aseba::Message &message);
  void nodeDescriptionReceived(unsigned nodeId);
  // TODO(Jerome): review, ROS2 version does not use it.
  // void nodeDisconnected(unsigned nodeId);
  void eventReceived(const AsebaAnonymousEventConstPtr &event);
  void knownEventReceived(const uint16_t id, const uint16_t nodeId,
                          const AsebaEventConstPtr &event);

  std::vector<unsigned> get_node_ids(const std::wstring &name);
  ros::Publisher pubFor(const Aseba::UserMessage *asebaMessage);
  void loadScriptToTarget(unsigned nodeId);
  void compile_script();
  void read_script_header();
  void load(Aseba::BytecodeVector &, unsigned int);
  void load_script_to(unsigned int);
  void load_script_to(std::vector<uint16_t>);
  void publish_nodes();
  std::map<std::string, Aseba::BytecodeVector> bytecode;
  std::string node_name(unsigned int);
  std::string script_path;
  std::set<uint16_t> running_nodes;
  ros::Publisher nodes_pub;
  std::map<unsigned, std::string> namespaces;
  std::string topic(unsigned id, const std::string &name);
  std::string namespace_for_node(unsigned id, std::string kind = std::string(""));

  void create_subscribers(unsigned nodeId);

  std::map<unsigned, std::string> names;
  bool ignore_node(unsigned int);
  bool manage_single_node;

public:
  AsebaROS(unsigned port, bool forward);
  ~AsebaROS();

  void pingCallback(const ros::TimerEvent &e);
  void run();
  void processAsebaMessage(Aseba::Message *message);
  void unconnect();
  void stopAllNodes();
  void update_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);
  void connectTarget(const std::string &target) { hub.connect(target); }
};

#endif // ASEBAROS_INCLUDE_ASEBAROS_ASEBAROS_H_
