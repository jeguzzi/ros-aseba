#include "asebaros/multi_asebaros.h"
#include <stdexcept>
#include <vector>
#include <chrono>
#include <sstream>

#include "libxml/parser.h"
#include "libxml/tree.h"
#include "transport/dashel_plugins/dashel-plugins.h"

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
using std::chrono_literals::operator""ms;
using std::chrono_literals::operator""s;
// UTF8 to wstring
std::wstring widen(const char *src) {
  const size_t destSize(mbstowcs(0, src, 0)+1);
  std::vector<wchar_t> buffer(destSize, 0);
  mbstowcs(&buffer[0], src, destSize);
  return std::wstring(buffer.begin(), buffer.end() - 1);
}
std::wstring widen(const std::string& src) {
  return widen(src.c_str());
}
// wstring to UTF8
std::string narrow(const wchar_t* src) {
  const size_t destSize(wcstombs(0, src, 0)+1);
  std::vector<char> buffer(destSize, 0);
  wcstombs(&buffer[0], src, destSize);
  return std::string(buffer.begin(), buffer.end() - 1);
}
std::string narrow(const std::wstring& src) {
  return narrow(src.c_str());
}


// AsebaDashelHub

AsebaDashelHub::AsebaDashelHub(AsebaROS* asebaROS, unsigned port, bool forward):
  Dashel::Hub(),
  asebaROS(asebaROS),
  forward(forward) {
  std::ostringstream oss;
  oss << "tcpin:port=" << port;
  Dashel::Hub::connect(oss.str());
}

static std::wstring asebaMsgToString(const Aseba::Message *message) {
  std::wostringstream oss;
  message->dump(oss);
  return oss.str();
}

void AsebaDashelHub::sendMessage(const Aseba::Message *message, bool doLock,
    Dashel::Stream* sourceStream) {
  // dump if requested
  RCLCPP_DEBUG(asebaROS->get_logger(), "Sending aseba message: %s",
    narrow(asebaMsgToString(message)).c_str());

  // Might be called from the ROS thread, not the Hub thread, need to lock
  if (doLock) lock();

  // write on all connected streams
  for (auto it = dataStreams.begin(); it != dataStreams.end(); ++it) {
    Dashel::Stream* destStream(*it);
    if ((forward) && (destStream == sourceStream)) continue;
    try {
      message->serialize(destStream);
      destStream->flush();
    }
    catch (Dashel::DashelException e) {
      // if this stream has a problem, ignore it for now, and let Hub call connectionClosed later.
      RCLCPP_ERROR(asebaROS->get_logger(), "error while writing message");
    }
  }
  if (doLock) unlock();
}

void AsebaDashelHub::operator()() {
  try {
    Hub::run();
  }
  catch (Dashel::DashelException e) {
    RCLCPP_ERROR(asebaROS->get_logger(), "Hub::run exception %s \n", e.what());
  }
  // cerr << "hub returned" << endl;
  rclcpp::shutdown();
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
  Aseba::Message *message = 0;
  try {
    message = Aseba::Message::receive(stream);
  }
  catch (Dashel::DashelException e) {
    // if this stream has a problem, ignore it for now, and let Hub call connectionClosed later.
    RCLCPP_ERROR(asebaROS->get_logger(), "error while writing message %s \n", e.what());
    return;
  }
  // send message to Dashel peers
  sendMessage(message, false, stream);
  // process message for ROS peers, the receiver will delete it
  asebaROS->processAsebaMessage(message);
  // free the message
  // TODO(jerome): use unique ptr?
  // auto message = std::shared_ptr<Aseba::Message>(Aseba::Message::receive(stream));
  delete message;
}

void AsebaDashelHub::connectionCreated(Dashel::Stream *stream) {
  RCLCPP_WARN(asebaROS->get_logger(), "Incoming connection from %s",
    stream->getTargetName().c_str());
  if (dataStreams.size() == 1) {
    // Note: on some robot such as the marXbot, because of hardware
    // constraints this might not work. In this case, an external
    // hack is required
    Aseba::GetDescription getDescription;
    sendMessage(&getDescription, false);
  }
}

void AsebaDashelHub::connectionClosed(Dashel::Stream* stream, bool abnormal) {
  if (abnormal) {
    RCLCPP_WARN(asebaROS->get_logger(), "Abnormal connection closed to %s : %s",
      stream->getTargetName().c_str(), stream->getFailReason().c_str());
    asebaROS->unconnect();
  } else {
    RCLCPP_WARN(asebaROS->get_logger(), "Normal connection closed to %s",
      stream->getTargetName().c_str());
  }
}

// AsebaROS

void AsebaROS::load(Aseba::BytecodeVector &bytecode, unsigned int node_id)
{
  RCLCPP_WARN(get_logger(), "Loading script to node with id %d (%d bytes)", node_id, bytecode.end()- bytecode.begin());
  MessageVector messages;
  auto bytes = std::vector<uint16_t>(bytecode.begin(), bytecode.end());

  sendBytecode(messages, node_id, bytes);
  for (auto it = messages.begin(); it != messages.end(); ++it) {
    hub.sendMessage((*it).get(), false);
  }
  Aseba::Run msg(node_id);
  hub.sendMessage(&msg, false);
  create_subscribers(node_id);
  running_nodes.push_back(node_id);
  publish_nodes();
}

void AsebaROS::publish_nodes()
{
  AsebaNodeList msg;
  for ( auto node_id : running_nodes)
  {
    AsebaNode node_msg;
    node_msg.id = node_id;
    // TODO: should save this
    node_msg.name_space = nameForId(node_id);
    node_msg.name = node_name(node_id);
    msg.nodes.push_back(node_msg);
  }
  nodes_pub->publish(msg);
}

void AsebaROS::load_script_to(unsigned int node_id)
{
  if(ignore(node_id))
  {
    RCLCPP_WARN(get_logger(), "Will not load script to node %d (to be ignored)");
    return;
  }
  // RCLCPP_WARN(get_logger(), "Get name");
  std::string name = node_name(node_id);
  // RCLCPP_WARN(get_logger(), "Got name %s", name.c_str());
  RCLCPP_WARN(get_logger(), "Will load script to node with id %d and name %s", node_id, name.c_str());
  if(!bytecode.count(name))
  {
    RCLCPP_WARN(get_logger(), "No script available for node with id %d and name %d", node_id, name.c_str());
    return;
  }
  load(bytecode[name], node_id);
}

void AsebaROS::load_script_to_all()
{
  for (auto const& pair : bytecode)
  {
    std::string name = pair.first;
    for (const auto & node_id : getNodeIds(widen(name)))
    {
      if(ignore(node_id)) continue;
      load(bytecode[name], node_id);
    }
  }
}

void AsebaROS::read_script_header()
{

  RCLCPP_WARN(get_logger(), "Reading script %s", script_path.c_str());

  xmlDoc *doc = xmlReadFile(script_path.c_str(), NULL, 0);
  if (!doc) {
    RCLCPP_ERROR(get_logger(), "Cannot read XML from file %s", script_path.c_str());
    throw std::invalid_argument("Cannot read XML from file");
  }

  xmlNode *domRoot = xmlDocGetRootElement(doc);
  // clear existing data
  mutex.lock();
  commonDefinitions.events.clear();
  commonDefinitions.constants.clear();
  userDefinedVariablesMap.clear();
  pubs.clear();
  subs.clear();
  bytecode.clear();
  running_nodes.clear();
  mutex.unlock();
  if (!xmlStrEqual(domRoot->name, BAD_CAST("network"))) {
    RCLCPP_ERROR(get_logger(), "root node is not \"network\", XML considered as invalid");
  } else {
    for (xmlNode *domNode = xmlFirstElementChild(domRoot); domNode; domNode = domNode->next) {
      // std::cerr << "node " << domNode->name << std::endl;
      if (domNode->type == XML_ELEMENT_NODE) {
         if (xmlStrEqual(domNode->name, BAD_CAST("event"))) {
          // get attributes
          xmlChar *name = xmlGetProp(domNode, BAD_CAST("name"));
          if (!name) RCLCPP_WARN(get_logger(), "missing \"name\" attribute in \"event\" entry");
          xmlChar *size = xmlGetProp(domNode, BAD_CAST("size"));
          if (!size) RCLCPP_WARN(get_logger(), "missing \"size\" attribute in \"event\" entry");
          // add event
          if (name && size) {
            int eventSize(atoi((const char *)size));
            if (eventSize > ASEBA_MAX_EVENT_ARG_SIZE) {
              RCLCPP_ERROR(get_logger(), "Event %s has a length %d larger than maximum %d",
                name, eventSize, ASEBA_MAX_EVENT_ARG_SIZE);
              break;
            } else {
              std::lock_guard<std::mutex> lock(mutex);
              commonDefinitions.events.push_back(Aseba::NamedValue(widen((const char *)name),
                eventSize));
            }
          }
          // free attributes
          if (name) xmlFree(name);
          if (size) xmlFree(size);
        } else if (xmlStrEqual(domNode->name, BAD_CAST("constant"))) {
          // get attributes
          xmlChar *name = xmlGetProp(domNode, BAD_CAST("name"));
          if (!name) RCLCPP_WARN(get_logger(),
            "missing \"name\" attribute in \"constant\" entry");
          xmlChar *value = xmlGetProp(domNode, BAD_CAST("value"));
          if (!value) RCLCPP_WARN(get_logger(),
            "missing \"value\" attribute in \"constant\" entry");
            // add constant if attributes are valid
          if (name && value) {
            int constant_value = atoi((const char *)value);
            std::string constant_name = std::string((const char *)name);
            rclcpp::Parameter param;
            std::string param_name = "script.constants." + constant_name;
            if (get_parameter(param_name, param))
            {
              constant_value = param.as_int();
            }
            std::lock_guard<std::mutex> lock(mutex);
            commonDefinitions.constants.push_back(Aseba::NamedValue(widen((const char *)name), constant_value));
          }
          // free attributes
          if (name) xmlFree(name);
          if (value) xmlFree(value);
        } else if (!xmlStrEqual(domNode->name, BAD_CAST("node"))) {
          RCLCPP_WARN(get_logger(), "Unknown XML node seen in .aesl file: %s", domNode->name);
        }
      }
    }
    // release memory
    xmlFreeDoc(doc);
    mutex.lock();
    for (size_t i = 0; i < commonDefinitions.events.size(); ++i) {
      pubs.push_back(std::map<unsigned, std::shared_ptr<rclcpp::Publisher<AsebaEvent>>>());
      subs.push_back(std::map<unsigned, std::shared_ptr<rclcpp::Subscription<AsebaEvent>>>());
    }
    mutex.unlock();
  }
  // for (auto &ev : commonDefinitions.constants)
  // {
  //   RCLCPP_WARN(get_logger(), "Constant %s with value %d", narrow(ev.name).c_str(), ev.value);
  // }
  // for (auto &ev : commonDefinitions.events)
  // {
  //   RCLCPP_WARN(get_logger(), "Event %s", narrow(ev.name).c_str());
  // }

  RCLCPP_WARN(get_logger(), "Read script");

}

void AsebaROS::compile_script()
{
  RCLCPP_WARN(get_logger(), "Will compile script");
  xmlDoc *doc = xmlReadFile(script_path.c_str(), NULL, 0);
  if (!doc) {
    RCLCPP_ERROR(get_logger(), "Cannot read XML from file %s", script_path.c_str());
    throw std::invalid_argument("Cannot read XML from file");
  }

  xmlNode *domRoot = xmlDocGetRootElement(doc);
  // load new data
  if (!xmlStrEqual(domRoot->name, BAD_CAST("network"))) {
    RCLCPP_ERROR(get_logger(), "root node is not \"network\", XML considered as invalid");
  } else {
    for (xmlNode *domNode = xmlFirstElementChild(domRoot); domNode; domNode = domNode->next) {
      // std::cerr << "node " << domNode->name << std::endl;
      if (domNode->type == XML_ELEMENT_NODE) {
        if (xmlStrEqual(domNode->name, BAD_CAST("node"))) {
          // get attributes, child and content
          xmlChar *name = xmlGetProp(domNode, BAD_CAST("name"));
          if (!name) {
            RCLCPP_WARN(get_logger(), "missing \"name\" attribute in \"node\" entry");
          } else {
            const std::string _name((const char *)name);
            if(bytecode.count(_name))
            {
              // We have already compiled it
              xmlFree(name);
              continue;
            }
            std::vector<unsigned> nodeIds = getNodeIds(widen(_name));
            if(nodeIds.empty())
            {
              xmlFree(name);
              continue;
            }
            xmlChar * text = xmlNodeGetContent(domNode);
            if (!text) {
              RCLCPP_WARN(get_logger(), "missing text in \"node\" entry");
            } else {
              std::wistringstream is(widen((const char *)text));
              Aseba:: Error error;
              unsigned nodeId = nodeIds[0];
              unsigned allocatedVariablesCount;
              mutex.lock();
              Aseba::Compiler compiler;
              compiler.setTargetDescription(getDescription(nodeId));
              compiler.setCommonDefinitions(&commonDefinitions);
              Aseba::BytecodeVector _bytecode;
              //TODO: we have assumed only 1 name, in general it should be a map name -> bytecode
              bool result = compiler.compile(is, _bytecode, allocatedVariablesCount, error);
              mutex.unlock();
              if(!result)
              {
                RCLCPP_ERROR(get_logger(), "compilation of %s for node %s failed: %s",script_path.c_str(), _name.c_str(),
                             narrow(error.toWString()).data());
                continue;
              }
              bytecode[_name] = _bytecode;
              RCLCPP_WARN(get_logger(), "Compiled script for id %d", _bytecode.end() - _bytecode.begin());

              mutex.lock();
              userDefinedVariablesMap[_name] = *compiler.getVariablesMap();
              mutex.unlock();

              // free attribute and content
              xmlFree(text);
            }
            xmlFree(name);
          }
        }
      }
    }
    // release memory
    xmlFreeDoc(doc);
  }
}

void AsebaROS::loadScript(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<LoadScripts::Request> req,
  const std::shared_ptr<LoadScripts::Response> res)
{
  script_path = req->file_name;
  read_script_header();
  load_script_to_all();
}

void AsebaROS::getNodeList(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<GetNodeList::Request> req,
  const std::shared_ptr<GetNodeList::Response> res) {
    std::lock_guard<std::mutex> lock(mutex);
    transform(nodesNames.begin(), nodesNames.end(), back_inserter(res->node_list),
      bind(&NodesNamesMap::value_type::first, _1));
}

void AsebaROS::getNodeId(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<GetNodeId::Request> req,
  const std::shared_ptr<GetNodeId::Response> res) {
  std::lock_guard<std::mutex> lock(mutex);
  NodesNamesMap::const_iterator nodeIt(nodesNames.find(req->node_name));
  if (nodeIt != nodesNames.end()) {
    res->node_id = nodeIt->second;
  } else {
    RCLCPP_ERROR(get_logger(), "node %s does not exists", req->node_name.c_str());
    throw std::invalid_argument("Node does not exists");
  }
}

std::string AsebaROS::node_name(unsigned int node_id)
{
  // HACK:
  // TODO: Understand and check all locks. Which are called from ROS ...
  // With the lock below, it blocks the second update
  // std::lock_guard<std::mutex> lock(mutex);
  auto node_it(nodes.find(node_id));
  if (node_it != nodes.end()) {
    return narrow(node_it->second.name);
  }
  return "";
}

void AsebaROS::getNodeName(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<GetNodeName::Request> req,
  const std::shared_ptr<GetNodeName::Response> res) {
  std::string name = node_name(req->node_id);
  if (name != "") {
    res->node_name = name;
  } else {
    RCLCPP_ERROR(get_logger(), "node %d does not exists", req->node_id);
    throw std::invalid_argument("Node does not exists");
  }
}

struct ExtractNameVar {
  std::string operator()(const std::pair<std::wstring, std::pair<unsigned, unsigned> > p) const {
    return narrow(p.first);
  }
};

struct ExtractNameDesc {
  std::string operator()(const Aseba::TargetDescription::NamedVariable& nv) const {
    return narrow(nv.name);
  }
};

void AsebaROS::getVariableList(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<GetVariableList::Request> req,
  const std::shared_ptr<GetVariableList::Response> res) {
  std::lock_guard<std::mutex> lock(mutex);
  NodesNamesMap::const_iterator nodeIt(nodesNames.find(req->node_name));
  if (nodeIt != nodesNames.end()) {
    // search if we have a user-defined variable map?
    auto userVarMapIt(userDefinedVariablesMap.find(req->node_name));
    if (userVarMapIt != userDefinedVariablesMap.end()) {
      // yes, us it
      const Aseba::VariablesMap& variablesMap(userVarMapIt->second);
      transform(variablesMap.begin(), variablesMap.end(),
      back_inserter(res->variable_list), ExtractNameVar());
    } else {
      // no, then only show node-defined variables
      const unsigned nodeId(nodeIt->second);
      const NodesMap::const_iterator descIt(nodes.find(nodeId));
      const Aseba::NodesManager::Node& description(descIt->second);
      transform(description.namedVariables.begin(), description.namedVariables.end(),
      back_inserter(res->variable_list), ExtractNameDesc());
    }
  } else {
    RCLCPP_ERROR(get_logger(), "node %s does not exists", req->node_name.c_str());
    throw std::invalid_argument("Node does not exists");
  }
}

void AsebaROS::setVariable(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<SetVariable::Request> req,
  const std::shared_ptr<SetVariable::Response> res) {
  // lock the access to the member methods
  unsigned nodeId, pos;
  mutex.lock();
  getNodePosFromNames(req->node_name, req->variable_name, &nodeId, &pos);
  mutex.unlock();
  Aseba::SetVariables msg(nodeId, pos, req->data);
  hub.sendMessage(&msg, true);
}

void AsebaROS::getVariable(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<GetVariable::Request> req,
  const std::shared_ptr<GetVariable::Response> res) {
  unsigned nodeId, pos;
  // lock the access to the member methods, wait will unlock the underlying mutex
  std::unique_lock<std::mutex> lock(mutex);
  // get information about variable
  getNodePosFromNames(req->node_name, req->variable_name, &nodeId, &pos);
  bool ok;
  unsigned length = getVariableSize(nodeId, widen(req->variable_name), &ok);
  if (!ok) throw std::invalid_argument("Cannot read variable size");

  // create query
  const GetVariableQueryKey key(nodeId, pos);
  GetVariableQueryValue query;
  getVariableQueries[key] = &query;
  lock.unlock();

  // send message, outside lock to avoid deadlocks
  Aseba::GetVariables msg(nodeId, pos, length);
  hub.sendMessage(&msg, true);

  // system_time const timeout(get_system_time()+posix_time::milliseconds(100));
  // wait 100 ms, considering the possibility of spurious wakes
  bool result;
  lock.lock();
  while (query.data.empty()) {
    if (query.cond.wait_for(lock, 100ms) == std::cv_status::timeout) break;
  }

  // remove key and return answer
  getVariableQueries.erase(key);
  if (result) {
    res->data = query.data;
  } else {
    RCLCPP_ERROR(get_logger(),
      "read of node %s, variable %s did not return a valid answer within 100ms",
      req->node_name.c_str(), req->variable_name.c_str());
    throw std::invalid_argument("Timeout");
  }
}

void AsebaROS::getEventId(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<GetEventId::Request> req,
  const std::shared_ptr<GetEventId::Response> res) {
  // needs locking, called by ROS's service thread
  std::lock_guard<std::mutex> lock(mutex);
  size_t id;
  if (commonDefinitions.events.contains(widen(req->name), &id)) {
    res->id = id;
  }
  throw std::invalid_argument("Event unknown");
}

void AsebaROS::getEventName(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<GetEventName::Request> req,
  const std::shared_ptr<GetEventName::Response> res) {
  // needs locking, called by ROS's service thread
  std::lock_guard<std::mutex> lock(mutex);
  if (req->id < commonDefinitions.events.size()) {
    res->name = narrow(commonDefinitions.events[req->id].name);
  }
  throw std::invalid_argument("Event unknown");
}

void AsebaROS::getNodePosFromNames(const std::string& nodeName, const std::string& variableName,
  unsigned* nodeId, unsigned* pos) const {
  // does not need locking, called by other member function already within the lock

  // make sure the node exists
  NodesNamesMap::const_iterator nodeIt(nodesNames.find(nodeName));
  if (nodeIt == nodesNames.end()) {
    RCLCPP_ERROR(get_logger(), "node %s does not exists", nodeName.c_str());
    throw std::invalid_argument("Node does not exists");
  }
  *nodeId = nodeIt->second;
  *pos = unsigned(-1);

  // check whether variable is user-defined
  auto userVarMapIt(userDefinedVariablesMap.find(nodeName));
  if (userVarMapIt != userDefinedVariablesMap.end()) {
    const Aseba::VariablesMap& userVarMap(userVarMapIt->second);
    const Aseba::VariablesMap::const_iterator userVarIt(userVarMap.find(widen(variableName)));
    if (userVarIt != userVarMap.end()) {
      *pos = userVarIt->second.first;
    }
  }

  // if variable is not user-defined, check whether it is provided by this node
  if (*pos == unsigned(-1)) {
    bool ok;
    *pos = getVariablePos(*nodeId, widen(variableName), &ok);
    if (!ok) {
      RCLCPP_ERROR(get_logger(), "variable %s does not exists in node %s",
        variableName.c_str(), nodeName.c_str());
      throw std::invalid_argument("Varible does not exists");
    }
  }
}

void AsebaROS::sendEventOnROS(const Aseba::UserMessage* asebaMessage) {
  // does not need locking, called by other member function already within lock
  // if different, we are currently loading a new script, publish on anonymous channel
  if (ignore(asebaMessage->source)) return;
  if ((pubs.size() == commonDefinitions.events.size()) &&
      (asebaMessage->type < commonDefinitions.events.size())) {
    // known, send on a named channel
    auto event = AsebaEvent();
    rclcpp::Clock ros_clock(RCL_ROS_TIME);
    event.stamp = ros_clock.now();
    event.source = asebaMessage->source;
    event.data = asebaMessage->data;
    pubFor(asebaMessage)->publish(event);
    // pubs[asebaMessage->type]->publish(event);
  } else {
    // unknown, send on the anonymous channel
    auto event = AsebaAnonymousEvent();
    rclcpp::Clock ros_clock(RCL_ROS_TIME);
    event.stamp = ros_clock.now();
    event.source = asebaMessage->source;
    event.type = asebaMessage->type;
    event.data = asebaMessage->data;
    anonPub->publish(event);
  }
}

void AsebaROS::nodeDescriptionReceived(unsigned nodeId) {
  // does not need locking, called by parent object
  RCLCPP_WARN(get_logger(), "Received description of node %d (%d)", nodeId, nodes[nodeId].isComplete());
  std::string name = narrow(nodes.at(nodeId).name);
  nodesNames[name] = nodeId;
  rclcpp::sleep_for(std::chrono::seconds(1));
  if(!bytecode.count(name)) compile_script();
  else RCLCPP_WARN(get_logger(), "Script already compiled for name %s", name.c_str());
  RCLCPP_WARN(get_logger(), "Will try to load script");
  //CHANGED: Let us load a script if available!
  load_script_to(nodeId);
}

void AsebaROS::eventReceived(const AsebaAnonymousEvent::SharedPtr event) {
  // does not need locking, does not touch object's members
  if (event->source == 0) {
    // forward only messages with source 0, which means, originating from this computer
    Aseba::UserMessage userMessage(event->type, event->data);
    hub.sendMessage(&userMessage, true);
  }
}
void AsebaROS::knownEventReceived(const uint16_t id, const uint16_t nodeId, const AsebaEvent::SharedPtr event) {
  // does not need locking, does not touch object's members
  if (event->source == 0) {
    RCLCPP_DEBUG(get_logger(), "known event %d received from ROS for node %d", id, nodeId);
    // forward only messages with source 0, which means, originating from this computer
    Aseba::VariablesDataVector data = event->data;
    data.insert(data.begin(), nodeId);
    Aseba::UserMessage userMessage(id, data);
    hub.sendMessage(&userMessage, true);
  }
}

void AsebaROS::sendMessage(const Aseba::Message& message) {
  // not sure if use true or false (to lock or not to lock)
  hub.sendMessage(&message, false);
}

AsebaROS::AsebaROS(unsigned port, bool forward, rclcpp::NodeOptions node_options):
  hub(this, port, forward),
  rclcpp::Node("asebaros", "aseba", node_options)
   {  // hub for dashel

  RCLCPP_WARN(get_logger(), "AsebaROS %d", mutex.try_lock());
  mutex.unlock();

  // does not need locking, called by main
  //  ros::Duration(5).sleep();
  // n = rclcpp::Node("asebaros", "asebaros");
  anonPub = create_publisher<AsebaAnonymousEvent>("anonymous_events", 100);
  auto cb = std::bind(&AsebaROS::eventReceived, this, _1);
  anonSub = create_subscription<AsebaAnonymousEvent>("anonymous_events", 100, cb);

  nodes_pub = create_publisher<AsebaNodeList>("nodes", rclcpp::QoS(1).transient_local());

  // script
  s.push_back(create_service<LoadScripts>("load_script",
    std::bind(&AsebaROS::loadScript, this, _1, _2, _3)));
  // nodes
  s.push_back(create_service<GetNodeList>("get_node_list",
    std::bind(&AsebaROS::getNodeList, this, _1, _2, _3)));
  s.push_back(create_service<GetNodeId>("get_node_id",
    std::bind(&AsebaROS::getNodeId, this, _1, _2, _3)));
  s.push_back(create_service<GetNodeName>("get_node_name",
    std::bind(&AsebaROS::getNodeName, this, _1, _2, _3)));
  // variables
  s.push_back(create_service<GetVariableList>("get_variable_list",
    std::bind(&AsebaROS::getVariableList, this, _1, _2, _3)));
  s.push_back(create_service<SetVariable>("set_variable",
    std::bind(&AsebaROS::setVariable, this, _1, _2, _3)));
  s.push_back(create_service<GetVariable>("get_variable",
    std::bind(&AsebaROS::getVariable, this, _1, _2, _3)));
  // events
  s.push_back(create_service<GetEventId>("get_event_id",
    std::bind(&AsebaROS::getEventId, this, _1, _2, _3)));
  s.push_back(create_service<GetEventName>("get_event_name",
    std::bind(&AsebaROS::getEventName, this, _1, _2, _3)));
  rclcpp::Parameter shutdown_on_unconnect_param;
  if (get_parameter("shutdown_on_unconnect", shutdown_on_unconnect_param))
    shutdown_on_unconnect = shutdown_on_unconnect_param.as_bool();
  else
    shutdown_on_unconnect = false;

  // TODO: How to declare params?
  rclcpp::Parameter file_path_param;
   // = declare_parameter("script.path");
  if (get_parameter("script.path", file_path_param))
  {
    script_path = file_path_param.as_string();
    RCLCPP_WARN(get_logger(), "Initializing with script %s", script_path.c_str());
    read_script_header();
  }
  else{
    RCLCPP_WARN(get_logger(), "Initializing without script");
  }

  RCLCPP_WARN(get_logger(), "AsebaROS F %d", mutex.try_lock());
  mutex.unlock();
}

AsebaROS::~AsebaROS() {
  // does not need locking, called by main
  xmlCleanupParser();
}

// void AsebaROS::run() {
//   // does not need locking, called by main
//   // hub.startThread();
//   rclcpp::TimerBase::SharedPtr timer = create_wall_timer(1s,
//       std::bind(&AsebaROS::pingNetwork, this));
//   rclcpp::spin(std::shared_ptr<AsebaROS>(this));
//   // ros::spin();
//   // cerr << "ros returned" << endl;
//   // hub.stopThread();
// }

void AsebaROS::run() {
  hub.startThread();
  timer = create_wall_timer(1s, std::bind(&AsebaROS::pingNetwork, this));
}


void AsebaROS::processAsebaMessage(Aseba::Message *message) {

  // scan this message for nodes descriptions
  Aseba::NodesManager::processMessage(message);

  // needs locking, called by Dashel hub
  // J: why?
  std::lock_guard<std::mutex> lock(mutex);

  // if user message, send to D-Bus as well
  Aseba::UserMessage *userMessage = dynamic_cast<Aseba::UserMessage *>(message);
  if (userMessage) sendEventOnROS(userMessage);

  // if variables, check for pending answers
  Aseba::Variables *variables = dynamic_cast<Aseba::Variables *>(message);
  if (variables) {
    const GetVariableQueryKey queryKey(variables->source, variables->start);
    GetVariableQueryMap::const_iterator queryIt(getVariableQueries.find(queryKey));
    if (queryIt != getVariableQueries.end()) {
    queryIt->second->data = variables->variables;
    queryIt->second->cond.notify_one();
  } else {
      RCLCPP_WARN(get_logger(),
        "received Variables from node %d, pos %d, but no corresponding query was found",
        variables->source, variables->start);
    }
  }
}

void AsebaROS::unconnect() {
  if (shutdown_on_unconnect) {
  RCLCPP_WARN(get_logger(), "Will shutdown the node.");
  rclcpp::shutdown();
  } else {
  RCLCPP_WARN(get_logger(), "Will ignore losing connection.");
  }
}

bool AsebaROS::ignore(unsigned id)
{
  return names[id] == "-";
}

void AsebaROS::stopAllNodes() {
  for (const auto & node : nodes) {

    if (ignore(node.first)) continue;
    Aseba::Reset msg_r(node.first);
    RCLCPP_WARN(get_logger(), "Reset %d", node.first);
    hub.sendMessage(&msg_r, true);
    rclcpp::sleep_for(std::chrono::seconds(1));
  }
}

std::vector<unsigned> AsebaROS::getNodeIds(const std::wstring& name)
{
  // search for all nodes with a given name
  std::vector<unsigned> nodeIds;
  for (const auto & node : nodes)
  {
    if (node.second.name == name and !ignore(node.first))
    {
      nodeIds.push_back(node.first);
    }
  }
  return nodeIds;
}

std::string AsebaROS::nameForId(unsigned id) {
  if ( names[id] != "" ) {
      // ROS_WARN_STREAM("name already recorded " << names[id]);
      return names[id];
  }

  rclcpp::Parameter param;
  std::string param_name = "nodes." + std::to_string(id);
  std::string value = "+";

  std::string prefix = "id_";
  rclcpp::Parameter prefix_param;
  if(get_parameter("nodes.prefix", prefix_param))
  {
    prefix = prefix_param.as_string();
  }
  if (get_parameter(param_name, param))
  {
    value = param.as_string();
  }
  else if (get_parameter("nodes.ANY", param))
  {
    value = param.as_string();
  }
  if (value == "+") {
    names[id] = prefix + std::to_string(id);
  } else {
    names[id] = value;
  }
  return names[id];
}

std::shared_ptr<rclcpp::Publisher<AsebaEvent>> AsebaROS::pubFor(const Aseba::UserMessage* asebaMessage)
{
  unsigned type = asebaMessage->type;
  unsigned source = asebaMessage->source;
  if(pubs[type].count(source) == 0)
  {
    const std::wstring& name(commonDefinitions.events[type].name);
    pubs[type][source] = create_publisher<AsebaEvent>(nameForId(source) + "/events/" + narrow(name), 100);
  }
  return pubs[type][source];
}


// AsebaROS

void AsebaROS::create_subscribers(unsigned node_id) {
  for (size_t i = 0; i < commonDefinitions.events.size(); ++i) {
    const std::wstring& name(commonDefinitions.events[i].name);
    std::string topic = nameForId(node_id) + "/events/" + narrow(name);
    subs[i][node_id] = create_subscription<AsebaEvent>(topic, 100,
        [this, i, node_id](AsebaEvent::SharedPtr event) {
          knownEventReceived(i, node_id, event);
        });
  }
}

// void AsebaROS::loadScriptToTarget(unsigned nodeId) {
//   MessageVector messages;
//   sendBytecode(messages, nodeId, std::vector<uint16_t>(bytecode.begin(), bytecode.end()));
//   for (MessageVector::const_iterator it = messages.begin(); it != messages.end(); ++it) {
//     hub.sendMessage((*it).get(), true);
//   }
//   Aseba::Run msg(nodeId);
//   hub.sendMessage(&msg, true);
//   createSubscribersForTarget(nodeId);
// }

//! Show usage
void dumpHelp(std::ostream &stream, std::string programName) {
  stream << "AsebaROS, connects aseba components together and with ROS, usage:\n";
  stream << programName << " [options] [additional targets]*\n";
  stream << "Options:\n";
  stream << "-l, --loop      : ";
  stream << "makes the switch transmit messages back to the send, not only forward them.\n";
  stream << "-p port         : listens to incoming connection on this port\n";
  stream << "-h, --help      : shows this help\n";
  stream << "ROS_OPTIONS     : see ROS documentation\n";
  stream << "Additional targets are any valid Dashel targets." << std::endl;
}


int main(int argc, char *argv[]) {
  // ros::init(argc, argv, "aseba");
  rclcpp::init(argc, argv);

  // return 0;

  auto args = rclcpp::remove_ros_arguments(argc, argv);
  unsigned port = ASEBA_DEFAULT_PORT;
  bool forward = true;
  std::vector<std::string> additionalTargets;
  for ( auto arg = args.begin() + 1; arg != args.end(); arg++ ) {
    if ((arg->compare("-l") == 0) || (arg->compare("--loop") == 0)) {
      forward = false;
    } else if (arg->compare("-p") == 0) {
      arg++;
      port = stoi(*arg);
    } else if ((arg->compare("-h") == 0) || (arg->compare("--help") == 0)) {
      dumpHelp(std::cout, args[0]);
      return 0;
    } else {
      additionalTargets.push_back(*arg);
    }
  }

  Dashel::initPlugins();
  rclcpp::NodeOptions node_options;
  node_options.allow_undeclared_parameters(true);
  node_options.automatically_declare_parameters_from_overrides(true);
  auto asebaROS = std::make_shared<AsebaROS>(port, forward, node_options);
  bool connected = false;

  RCLCPP_WARN(asebaROS->get_logger(), "Wait for connections");

  while (rclcpp::ok() && !connected) {
    for (auto &target : additionalTargets) {
      RCLCPP_WARN(asebaROS->get_logger(), "Connecting to %s", target.c_str());
      try {
        asebaROS->connectTarget(target);
        connected = true;
      } catch (Dashel::DashelException e) {
        std::cerr << e.what() << std::endl;
      }
    }
    if (!connected) {
      RCLCPP_WARN(asebaROS->get_logger(),
        "Could not connect to any target. Sleep for 1 second and then retry");
      rclcpp::sleep_for(std::chrono::seconds(1));
    }
  }

  rclcpp::sleep_for(std::chrono::seconds(1));

  asebaROS->run();

  rclcpp::spin(asebaROS);

  asebaROS->stopAllNodes();

  RCLCPP_WARN(asebaROS->get_logger(), "Will shutdown node");
  rclcpp::shutdown();
  return 0;
}