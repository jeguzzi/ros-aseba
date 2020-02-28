// Copyright 2019 JG

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "dashel/dashel.h"

class AsebaROS;

class AsebaDashelHub: public Dashel::Hub
{
private:
	AsebaROS* asebaROS; //!< pointer to aseba ROS
	bool forward; //!< should we only forward messages instead of transmit them back to the sender

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
	// void sendMessage(const Aseba::Message *message, bool doLock, Dashel::Stream* sourceStream = 0);

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




class AsebaROS : public rclcpp::Node {
 public:
  AsebaROS(): Node("asebaros") {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
  }
 private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char * argv[]) {
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<AsebaROS>());
        rclcpp::shutdown();
        return 0;
}
