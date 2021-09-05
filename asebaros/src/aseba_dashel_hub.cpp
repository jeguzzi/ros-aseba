#include <chrono> // NOLINT
#include <sstream>
#include <stdexcept>

#include "asebaros/asebaros_node_manager.h"
#include "asebaros/utils.h"

static std::wstring asebaMsgToString(const Aseba::Message *message) {
  std::wostringstream oss;
  message->dump(oss);
  return oss.str();
}

//------------ AsebaDashelHub ------------ //
AsebaDashelHub::AsebaDashelHub(AsebaROS *asebaROS, unsigned port, bool forward)
    : Dashel::Hub(), asebaROS(asebaROS), forward(forward) {
  std::ostringstream oss;
  oss << "tcpin:port=" << port;
  Dashel::Hub::connect(oss.str());
  LOG_INFO("Created AsebaDashelHub %d %d", port, forward);
}

void AsebaDashelHub::sendMessage(const Aseba::Message *message, bool doLock,
                                 Dashel::Stream *sourceStream) {
  // dump if requested, but only if it's not forwarding
  if (!sourceStream) {
    LOG_DEBUG("Sending aseba message: %s",
              narrow(asebaMsgToString(message)).c_str());
  }

  // Might be called from the ROS thread, not the Hub thread, need to lock
  if (doLock)
    lock();

  // write on all connected streams
  for (auto it = dataStreams.begin(); it != dataStreams.end(); ++it) {
    Dashel::Stream *destStream(*it);
    if ((forward) && (destStream == sourceStream)) {
      continue;
    }
    try {
      message->serialize(destStream);
      destStream->flush();
    } catch (Dashel::DashelException e) {
      // if this stream has a problem, ignore it for now, and let Hub call
      // connectionClosed later.
      LOG_ERROR("AsebaDashelHub: error while writing message");
    }
  }
  if (doLock)
    unlock();
}

void AsebaDashelHub::operator()() {
  try {
    Hub::run();
  } catch (Dashel::DashelException e) {
    LOG_ERROR("Hub::run exception %s \n", e.what());
  }
  shutdown();
}

void AsebaDashelHub::startThread() {
  thread = std::make_unique<std::thread>(std::ref(*this));
}

void AsebaDashelHub::stopThread() {
  Hub::stop();
  thread->join();
  thread = nullptr;
}

// the following method run in the blocking reception thread
void AsebaDashelHub::incomingData(Dashel::Stream *stream) {
  // receive message
  Aseba::Message *message = nullptr;
  try {
    message = Aseba::Message::receive(stream);
  } catch (Dashel::DashelException e) {
    // if this stream has a problem, ignore it for now, and let Hub call
    // connectionClosed later.
    LOG_ERROR("AsebaDashelHub: error while writing message %s \n", e.what());
  }
  if (message) {
    LOG_DEBUG("Received aseba message: %s",
              narrow(asebaMsgToString(message)).c_str());
    // send message to Dashel peers
    sendMessage(message, false, stream);
    // process message for ROS peers, the receiver will delete it
    asebaROS->processAsebaMessage(message);
    // free the message
    delete message;
  }
}

// TODO(Jerome): Is this still needed in aseba >= 5, as we are already using pingNetwork?
void AsebaDashelHub::connectionCreated(Dashel::Stream *stream) {
  LOG_INFO("Incoming connection from %s", stream->getTargetName().c_str());
  asebaROS->set_connected_target(stream->getTargetName());
  if (dataStreams.size() == 1) {
    // Note: on some robot such as the marXbot, because of hardware
    // constraints this might not work. In this case, an external
    // hack is required
    Aseba::GetDescription getDescription;
    LOG_INFO("Broadcast a description query");
    sendMessage(&getDescription, false);
  }
}

void AsebaDashelHub::connectionClosed(Dashel::Stream *stream, bool abnormal) {
  if (abnormal) {
    LOG_WARN("Abnormal connection closed to %s : %s",
             stream->getTargetName().c_str(), stream->getFailReason().c_str());
    asebaROS->has_been_disconnected();
  } else {
    LOG_INFO("Normal connection closed to %s", stream->getTargetName().c_str());
  }
}
