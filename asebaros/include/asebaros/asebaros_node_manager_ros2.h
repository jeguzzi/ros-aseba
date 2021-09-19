#ifndef ASEBAROS_INCLUDE_ASEBAROS_ASEBAROSNODEMANAGERROS2_H_
#define ASEBAROS_INCLUDE_ASEBAROS_ASEBAROSNODEMANAGERROS2_H_

#include "asebaros/asebaros_node_manager.h"

#include "rclcpp/rclcpp.hpp"

#include "asebaros_msgs/msg/anonymous_event.hpp"
#include "asebaros_msgs/srv/get_node_list.hpp"
#include "asebaros_msgs/srv/load_script.hpp"

class AsebaROS2 : public rclcpp::Node, public AsebaROS {
  typedef std::vector<std::shared_ptr<rclcpp::ServiceBase>> ServiceServers;

public:
  AsebaROS2();

protected:
#if DIAGNOSTICS
  diagnostic_updater::Updater updater;
#endif
  /// all services of this class
  ServiceServers s;
  /// anonymous publisher, for aseba events with no associated name
  std::shared_ptr<rclcpp::Publisher<asebaros_msgs::msg::AnonymousEvent>>
      anonPub;
  /// anonymous subscriber, for aseba events with no associated name
  std::shared_ptr<rclcpp::Subscription<asebaros_msgs::msg::AnonymousEvent>>
      anonSub;
  std::shared_ptr<rclcpp::Publisher<NodeListMsg>> nodes_pub;
  rclcpp::TimerBase::SharedPtr timer;
  void set_connected_target(const std::string &target);

  void load_script_cb(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<asebaros_msgs::srv::LoadScript::Request> req,
      const std::shared_ptr<asebaros_msgs::srv::LoadScript::Response> res);

  void get_node_list_cb(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<asebaros_msgs::srv::GetNodeList::Request> req,
      const std::shared_ptr<asebaros_msgs::srv::GetNodeList::Response> res);

  unsigned get_port();
  bool get_loop();

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
  void publish_node_list(const NodeListMsg &msg);
  void set_script_param(const std::string &path);
};

#endif /* end of include guard:                                                \
          ASEBAROS_INCLUDE_ASEBAROS_ASEBAROSNODEMANAGERROS2_H_ */
