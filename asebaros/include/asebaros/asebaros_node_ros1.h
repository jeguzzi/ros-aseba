#ifndef ASEBAROS_INCLUDE_ASEBAROS_ASEBAROSNODEROS1_H
#define ASEBAROS_INCLUDE_ASEBAROS_ASEBAROSNODEROS1_H

#include <string>
#include <vector>

#include "asebaros/asebaros_node_manager_ros1.h"
#include "asebaros/asebaros_node.h"
#include "ros/ros.h"

#include "common/msg/NodesManager.h"
#include "common/msg/msg.h"

#include "asebaros_msgs/GetDescription.h"
#include "asebaros_msgs/GetVariable.h"
#include "asebaros_msgs/SetVariable.h"
#include "aseba_script.h"
#include "utils.h"

class AsebaROS1;

class AsebaROS1Node : public AsebaROSNode {

  typedef std::vector<ros::ServiceServer> ServiceServers;
  typedef std::vector<ros::Subscriber> Subscribers;
  typedef std::map<unsigned, ros::Publisher> Publishers;

public:
  AsebaROS1Node(AsebaDashelHub *hub, AsebaROS1 *manager, const unsigned id,
                const std::string name,
                const Aseba::TargetDescription *description,
                const std::string &namespace_, bool include_id_in_events,
                bool is_connected = true);
  // Superclass virtual methods
  bool publish_event(const Aseba::UserMessage *aseba_message);

protected:
  ros::NodeHandle n;
  ros::NodeHandle nh;
  Publishers pubs;
  /// anonymous subscriber, for aseba events with no associated name
  Subscribers subs;
  ServiceServers services;
  ros::Publisher desc_pub;
  ros::Publisher *get_publisher_for(const Aseba::UserMessage *asebaMessage);
  // ROS Callbacks
  bool set_variable_cb(asebaros_msgs::SetVariable::Request &req,
                       asebaros_msgs::SetVariable::Response &res);
  bool get_variable_cb(asebaros_msgs::GetVariable::Request &req,
                       asebaros_msgs::GetVariable::Response &res);
  bool get_description_cb(asebaros_msgs::GetDescription::Request &req,
                          asebaros_msgs::GetDescription::Response &res);

  // Superclass virtual methods
  virtual std::string absolute_namespace(const std::string &) const;
  void create_subscribers();
  void reset_publishers();
  void publish_description(const NodeDescriptionMsg & msg);
};

#endif // ASEBAROS_INCLUDE_ASEBAROS_ASEBAROSNODEROS1_H
