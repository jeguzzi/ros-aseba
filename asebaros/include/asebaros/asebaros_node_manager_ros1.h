#ifndef ASEBAROS_INCLUDE_ASEBAROS_ASEBAROSNODEMANAGERROS1_H_
#define ASEBAROS_INCLUDE_ASEBAROS_ASEBAROSNODEMANAGERROS1_H_

#include "asebaros/asebaros_node_manager.h"

#include "ros/ros.h"

#include "asebaros_msgs/AnonymousEvent.h"
#include "asebaros_msgs/GetNodeList.h"
#include "asebaros_msgs/LoadScript.h"

class AsebaROS1 : public AsebaROS {
  typedef std::vector<ros::ServiceServer> ServiceServers;

public:
  AsebaROS1();
  ~AsebaROS1();

protected:
#if DIAGNOSTICS
  diagnostic_updater::Updater updater;
#endif
  /// node handler of this class
  ros::NodeHandle n;
  ros::NodeHandle nh;
  /// all services of this class
  ServiceServers s;
  /// anonymous publisher, for aseba events with no associated name
  ros::Publisher anonPub;
  /// anonymous subscriber, for aseba events with no associated name
  ros::Subscriber anonSub;
  ros::Publisher nodes_pub;
  ros::Timer timer;

  void set_connected_target(const std::string &target);

  bool load_script_cb(asebaros_msgs::LoadScript::Request &req, asebaros_msgs::LoadScript::Response &res);
  bool get_node_list_cb(asebaros_msgs::GetNodeList::Request &req, asebaros_msgs::GetNodeList::Response &res);

  // ROS1-2 specific [virtual] methods
  void start_pinging();
  bool set_constant_from_param(Aseba::NamedValue &constant,
                               const std::string &param_name);
  void set_constant_to_param(const Aseba::NamedValue &constant,
                             const std::string &param_name);
  std::string init_params();
  void init_ros();
  void import_node_config(const std::string &prefix);
  void publish_anonymous_event(const Aseba::UserMessage *aseba_message);
  AsebaROSNode *add_asebaros_node(unsigned id, const std::string &name,
                                  const std::string &ns, bool include_id_in_events);
  void publish_node_list(const NodeListMsg & msg);
  void set_script_param(const std::string & path);
  void publish_description(const NodeDescriptionMsg & msg);
};

#endif /* end of include guard:                                                \
          ASEBAROS_INCLUDE_ASEBAROS_ASEBAROSNODEMANAGERROS1_H_ */
