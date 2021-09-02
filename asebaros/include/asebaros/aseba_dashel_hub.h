#ifndef ASEBAROS_INCLUDE_ASEBAROS_ASEBADASHELHUB_H_
#define ASEBAROS_INCLUDE_ASEBAROS_ASEBADASHELHUB_H_

#include <memory>
#include <thread>

#include "common/msg/msg.h"
#include "dashel/dashel.h"

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

#endif // ASEBAROS_INCLUDE_ASEBAROS_ASEBADASHELHUB_H_
