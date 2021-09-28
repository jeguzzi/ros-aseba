#ifndef ASEBAROS_INCLUDE_ASEBAROS_ASEBAROSNODEROS2_H
#define ASEBAROS_INCLUDE_ASEBAROS_ASEBAROSNODEROS2_H

#include <string>
#include <vector>

#include "asebaros/asebaros_node.h"
#include "asebaros/asebaros_node_manager_ros2.h"
#include "rclcpp/rclcpp.hpp"

#include "common/msg/NodesManager.h"
#include "common/msg/msg.h"

#include "asebaros_msgs/srv/get_description.hpp"
#include "asebaros_msgs/srv/get_variable.hpp"
#include "asebaros_msgs/srv/set_variable.hpp"
#include "aseba_script.h"
#include "utils.h"

class AsebaROS2;

class AsebaROS2Node : public AsebaROSNode {

  typedef std::vector<
      std::shared_ptr<rclcpp::Subscription<EventMsg>>>
      Subscribers;
  typedef std::map<
      unsigned, std::shared_ptr<rclcpp::Publisher<EventMsg>>>
      Publishers;
  typedef std::vector<std::shared_ptr<rclcpp::ServiceBase>> ServiceServers;

public:
  AsebaROS2Node(AsebaDashelHub *hub, AsebaROS2 *manager, rclcpp::Node *ros_node,
                const unsigned id, const std::string name,
                const Aseba::TargetDescription *description,
                const std::string &namespace_, bool include_id_in_events,
                bool is_connected = true);
  // Superclass virtual methods
  bool publish_event(const Aseba::UserMessage *aseba_message);

protected:

  rclcpp::Node *ros_node;
  Publishers pubs;
  Subscribers subs;
  ServiceServers services;
  std::shared_ptr<rclcpp::Publisher<NodeDescriptionMsg>> desc_pub;
  std::shared_ptr<rclcpp::Publisher<EventMsg>>
  get_publisher_for(const Aseba::UserMessage *asebaMessage);
  // ROS Callbacks
  void set_variable_cb(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<asebaros_msgs::srv::SetVariable::Request> req,
      const std::shared_ptr<asebaros_msgs::srv::SetVariable::Response> res);
  void get_variable_cb(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<asebaros_msgs::srv::GetVariable::Request> req,
      const std::shared_ptr<asebaros_msgs::srv::GetVariable::Response> res);
  void get_description_cb(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<asebaros_msgs::srv::GetDescription::Request> req,
      const std::shared_ptr<asebaros_msgs::srv::GetDescription::Response> res);

  // Superclass virtual methods
  virtual std::string absolute_namespace(const std::string &) const;
  void create_subscribers();
  void reset_publishers();
  void publish_description(const NodeDescriptionMsg & msg);
};

#endif // ASEBAROS_INCLUDE_ASEBAROS_ASEBAROSNODEROS2_H
