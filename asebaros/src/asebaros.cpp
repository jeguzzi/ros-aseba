#include <sstream>

#include "asebaros/asebaros.h"
#include "libxml/parser.h"
#include "libxml/tree.h"
#include "ros/console.h"
#include "transport/dashel_plugins/dashel-plugins.h"

using std::chrono_literals::operator""ms;

// UTF8 to wstring
std::wstring widen(const char *src) {
  const size_t destSize(mbstowcs(0, src, 0) + 1);
  std::vector<wchar_t> buffer(destSize, 0);
  mbstowcs(&buffer[0], src, destSize);
  return std::wstring(buffer.begin(), buffer.end() - 1);
}

std::wstring widen(const std::string &src) { return widen(src.c_str()); }

// wstring to UTF8
std::string narrow(const wchar_t *src) {
  const size_t destSize(wcstombs(0, src, 0) + 1);
  std::vector<char> buffer(destSize, 0);
  wcstombs(&buffer[0], src, destSize);
  return std::string(buffer.begin(), buffer.end() - 1);
}

std::string narrow(const std::wstring &src) { return narrow(src.c_str()); }

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
}

void AsebaDashelHub::sendMessage(const Aseba::Message *message, bool doLock,
                                 Dashel::Stream *sourceStream) {
  // dump if requested
  ROS_DEBUG_STREAM(
      "sending aseba message: " << narrow(asebaMsgToString(message)));

  // Might be called from the ROS thread, not the Hub thread, need to lock
  if (doLock)
    lock();

  // write on all connected streams
  for (auto it = dataStreams.begin(); it != dataStreams.end(); ++it) {
    Dashel::Stream *destStream(*it);
    if ((forward) && (destStream == sourceStream))
      continue;
    try {
      message->serialize(destStream);
      destStream->flush();
    } catch (Dashel::DashelException e) {
      // if this stream has a problem, ignore it for now, and let Hub call
      // connectionClosed later.
      ROS_ERROR("error while writing message");
    }
  }
  if (doLock)
    unlock();
}

void AsebaDashelHub::operator()() {
  try {
    Hub::run();
  } catch (Dashel::DashelException e) {
    ROS_ERROR("Hub::run exception %s \n", e.what());
  }
  ros::shutdown();
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
  // TODO(jerome): use smart ptr?
  // auto message =
  // std::shared_ptr<Aseba::Message>(Aseba::Message::receive(stream));

  // receive message
  Aseba::Message *message = 0;
  try {
    message = Aseba::Message::receive(stream);
  } catch (Dashel::DashelException e) {
    // if this stream has a problem, ignore it for now, and let Hub call
    // connectionClosed later.
    ROS_ERROR("error while writing message %s \n", e.what());
    if (message)
      delete message;
    return;
  }
  // send message to Dashel peers
  sendMessage(message, false, stream);
  // process message for ROS peers, the receiver will delete it
  asebaROS->processAsebaMessage(message);
  // free the message
  delete message;
}

void AsebaDashelHub::connectionCreated(Dashel::Stream *stream) {
  ROS_INFO_STREAM("Incoming connection from " << stream->getTargetName());
  if (dataStreams.size() == 1) {
    // Note: on some robot such as the marXbot, because of hardware
    // constraints this might not work. In this case, an external
    // hack is required
    Aseba::GetDescription getDescription;
    sendMessage(&getDescription, false);
  }
}

// TODO(Jerome): do we really have to close? can we not switch to waiting?
void AsebaDashelHub::connectionClosed(Dashel::Stream *stream, bool abnormal) {
  if (abnormal) {
    ROS_WARN_STREAM("Abnormal connection closed to "
                    << stream->getTargetName() << " : "
                    << stream->getFailReason());
    asebaROS->unconnect();
  } else {
    ROS_INFO_STREAM("Normal connection closed to " << stream->getTargetName());
  }
}

//------------ AsebaROS ------------ //

void AsebaROS::load(Aseba::BytecodeVector &bytecode, unsigned int node_id) {
  ROS_INFO("Loading script to node with id %d (%lu bytes)", node_id,
           bytecode.end() - bytecode.begin());
  MessageVector messages;
  auto bytes = std::vector<uint16_t>(bytecode.begin(), bytecode.end());

  sendBytecode(messages, node_id, bytes);
  for (auto it = messages.begin(); it != messages.end(); ++it) {
    hub.sendMessage((*it).get(), false);
  }
  Aseba::Run msg(node_id);
  hub.sendMessage(&msg, false);
  create_subscribers(node_id);
  running_nodes.insert(node_id);
  publish_nodes();
}

void AsebaROS::publish_nodes() {
  AsebaNodeList msg;
  for (auto node_id : running_nodes) {
    AsebaNode node_msg;
    node_msg.id = node_id;
    // TODO(Jerome): should save this
    node_msg.name_space = namespace_for_node(node_id);
    node_msg.name = node_name(node_id);
    msg.nodes.push_back(node_msg);
  }
  nodes_pub.publish(msg);
}

void AsebaROS::load_script_to(unsigned int node_id) {
  if (ignore_node(node_id)) {
    ROS_WARN("Will not load script to node %d to be ignored", node_id);
    return;
  }
  std::string name = node_name(node_id);
  ROS_INFO("Will try to load a script to node with name %s and id %d",
           name.c_str(), node_id);
  if (!bytecode.count(name)) {
    ROS_WARN("No script available for name %s", name.c_str());
    return;
  }
  load(bytecode[name], node_id);
}

void AsebaROS::load_script_to(std::vector<uint16_t> node_ids) {
  std::set<uint16_t> node_ids_set(node_ids.begin(), node_ids.end());
  for (auto const &pair : bytecode) {
    std::string name = pair.first;
    ROS_WARN("Will load compiled script to nodes with name %s", name.c_str());
    for (const auto &node_id : get_node_ids(widen(name))) {
      if (ignore_node(node_id))
        continue;
      if (node_ids_set.size() && !node_ids_set.count(node_id))
        continue;
      load(bytecode[name], node_id);
    }
  }
}

void AsebaROS::read_script_header() {
  if (script_path.empty())
    return;
  ROS_INFO("Will read script headers in %s", script_path.c_str());
  xmlDoc *doc = xmlReadFile(script_path.c_str(), NULL, 0);
  if (!doc) {
    ROS_ERROR("Cannot read XML from file %s", script_path.c_str());
    return;
    // throw std::invalid_argument("Cannot read XML from file");
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
    ROS_ERROR("root node is not \"network\", XML considered as invalid");
  } else {
    for (xmlNode *domNode = xmlFirstElementChild(domRoot); domNode;
         domNode = domNode->next) {
      // std::cerr << "node " << domNode->name << std::endl;
      if (domNode->type == XML_ELEMENT_NODE) {
        if (xmlStrEqual(domNode->name, BAD_CAST("event"))) {
          // get attributes
          xmlChar *name = xmlGetProp(domNode, BAD_CAST("name"));
          if (!name)
            ROS_WARN("missing \"name\" attribute in \"event\" entry");
          xmlChar *size = xmlGetProp(domNode, BAD_CAST("size"));
          if (!size)
            ROS_WARN("missing \"size\" attribute in \"event\" entry");
          // add event
          if (name && size) {
            int eventSize(atoi((const char *)size));
            if (eventSize > ASEBA_MAX_EVENT_ARG_SIZE) {
              ROS_ERROR("Event %s has a length %d larger than maximum %d", name,
                        eventSize, ASEBA_MAX_EVENT_ARG_SIZE);
              break;
            } else {
              std::lock_guard<std::mutex> lock(mutex);
              commonDefinitions.events.push_back(
                  Aseba::NamedValue(widen((const char *)name), eventSize));
            }
          }
          // free attributes
          if (name)
            xmlFree(name);
          if (size)
            xmlFree(size);
        } else if (xmlStrEqual(domNode->name, BAD_CAST("constant"))) {
          // get attributes
          xmlChar *name = xmlGetProp(domNode, BAD_CAST("name"));
          if (!name)
            ROS_WARN("missing \"name\" attribute in \"constant\" entry");
          xmlChar *value = xmlGetProp(domNode, BAD_CAST("value"));
          if (!value)
            ROS_WARN("missing \"value\" attribute in \"constant\" entry");
          // add constant if attributes are valid
          if (name && value) {
            int constant_value = atoi((const char *)value);
            std::string constant_name = std::string((const char *)name);
            std::string param_name = "script/constants/" + constant_name;
            if (nh.getParam(param_name, constant_value)) {
            } else {
              nh.setParam(param_name, constant_value);
            }
            std::lock_guard<std::mutex> lock(mutex);
            commonDefinitions.constants.push_back(
                Aseba::NamedValue(widen((const char *)name), constant_value));
          }
          // free attributes
          if (name)
            xmlFree(name);
          if (value)
            xmlFree(value);
        } else if (xmlStrEqual(domNode->name, BAD_CAST("keywords"))) {
          continue;
        } else if (!xmlStrEqual(domNode->name, BAD_CAST("node"))) {
          ROS_WARN("Unknown XML node seen in .aesl file: %s", domNode->name);
        }
      }
    }
    // release memory
    xmlFreeDoc(doc);
    mutex.lock();
    for (size_t i = 0; i < commonDefinitions.events.size(); ++i) {
      pubs.push_back(std::map<unsigned, ros::Publisher>());
      subs.push_back(std::map<unsigned, ros::Subscriber>());
    }
    mutex.unlock();
  }
}

void AsebaROS::compile_script() {
  if (script_path.empty())
    return;
  ROS_INFO("Will compile script in file %s", script_path.c_str());
  xmlDoc *doc = xmlReadFile(script_path.c_str(), NULL, 0);
  if (!doc) {
    ROS_ERROR("Cannot read XML from file %s", script_path.c_str());
    return;
    // throw std::invalid_argument("Cannot read XML from file");
  }

  xmlNode *domRoot = xmlDocGetRootElement(doc);
  // load new data
  if (!xmlStrEqual(domRoot->name, BAD_CAST("network"))) {
    ROS_ERROR("root node is not \"network\", XML considered as invalid");
  } else {
    for (xmlNode *domNode = xmlFirstElementChild(domRoot); domNode;
         domNode = domNode->next) {
      // std::cerr << "node " << domNode->name << std::endl;
      if (domNode->type == XML_ELEMENT_NODE) {
        if (xmlStrEqual(domNode->name, BAD_CAST("node"))) {
          // get attributes, child and content
          xmlChar *name = xmlGetProp(domNode, BAD_CAST("name"));
          if (!name) {
            ROS_WARN("missing \"name\" attribute in \"node\" entry");
          } else {
            const std::string _name((const char *)name);
            if (bytecode.count(_name)) {
              // We have already compiled it
              xmlFree(name);
              continue;
            }
            std::vector<unsigned> nodeIds = get_node_ids(widen(_name));
            if (nodeIds.empty()) {
              xmlFree(name);
              continue;
            }
            xmlChar *text = xmlNodeGetContent(domNode);
            if (!text) {
              ROS_WARN("missing text in \"node\" entry");
            } else {
              std::wistringstream is(widen((const char *)text));
              Aseba::Error error;
              unsigned nodeId = nodeIds[0];
              unsigned allocatedVariablesCount;
              mutex.lock();
              Aseba::Compiler compiler;
              compiler.setTargetDescription(getDescription(nodeId));
              compiler.setCommonDefinitions(&commonDefinitions);
              Aseba::BytecodeVector _bytecode;
              // TODO(Jerome): we have assumed only 1 name, in general it should
              // be a map name -> bytecode
              bool result = compiler.compile(is, _bytecode,
                                             allocatedVariablesCount, error);
              mutex.unlock();
              if (!result) {
                ROS_ERROR("compilation of %s for node %s failed: %s",
                          script_path.c_str(), _name.c_str(),
                          narrow(error.toWString()).data());
                continue;
              }
              bytecode[_name] = _bytecode;
              ROS_INFO("Compiled script for nodes with name %s to %lu bytes",
                       _name.data(), _bytecode.end() - _bytecode.begin());

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

bool AsebaROS::loadScript(LoadScripts::Request &req,
                          LoadScripts::Response &res) {
  for (auto &constant : req.constants) {
    nh.setParam("script/constants/" + constant.name, constant.value);
  }
  script_path = req.file_name;
  read_script_header();
  compile_script();
  load_script_to(req.node_ids);
  // TODO(Jerome): should be failable
  return true;
}

bool AsebaROS::getNodeList(GetNodeList::Request &req,
                           GetNodeList::Response &res) {
  std::lock_guard<std::mutex> lock(mutex);
  for (const auto &node : nodes) {
    if (!req.ignored && ignore_node(node.first))
      continue;
    AsebaNode node_msg;
    node_msg.id = node.first;
    node_msg.name_space = namespace_for_node(node.first);
    node_msg.name = narrow(node.second.name);
    res.nodes.push_back(node_msg);
  }
  return true;
}

bool AsebaROS::getNodeIds(GetNodeIds::Request &req, GetNodeIds::Response &res) {
  std::lock_guard<std::mutex> lock(mutex);
  std::vector<unsigned> ids = get_node_ids(widen(req.node_name));
  std::copy(ids.begin(), ids.end(), back_inserter(res.node_ids));
  return true;
}

bool AsebaROS::getNodeName(GetNodeName::Request &req,
                           GetNodeName::Response &res) {
  std::lock_guard<std::mutex> lock(mutex);
  std::string name = node_name(req.node_id);
  if (name != "") {
    res.node_name = name;
    return true;
  } else {
    ROS_ERROR("node %d does not exists", req.node_id);
    return false;
  }
}

struct ExtractNameVar {
  std::string operator()(
      const std::pair<std::wstring, std::pair<unsigned, unsigned>> p) const {
    return narrow(p.first);
  }
};

struct ExtractNameDesc {
  std::string
  operator()(const Aseba::TargetDescription::NamedVariable &nv) const {
    return narrow(nv.name);
  }
};

bool AsebaROS::getVariableList(GetVariableList::Request &req,
                               GetVariableList::Response &res) {
  std::lock_guard<std::mutex> lock(mutex);

  NodesNamesMap::const_iterator nodeIt(nodesNames.find(req.node_name));
  if (nodeIt != nodesNames.end()) {
    // search if we have a user-defined variable map?
    auto userVarMapIt(userDefinedVariablesMap.find(req.node_name));
    if (userVarMapIt != userDefinedVariablesMap.end()) {
      // yes, us it
      const Aseba::VariablesMap &variablesMap(userVarMapIt->second);
      transform(variablesMap.begin(), variablesMap.end(),
                std::back_inserter(res.variable_list), ExtractNameVar());
    } else {
      // no, then only show node-defined variables
      const unsigned nodeId(nodeIt->second);
      const NodesMap::const_iterator descIt(nodes.find(nodeId));
      const Aseba::NodesManager::Node &description(descIt->second);
      transform(description.namedVariables.begin(),
                description.namedVariables.end(),
                std::back_inserter(res.variable_list), ExtractNameDesc());
    }
    return true;
  } else {
    ROS_ERROR_STREAM("node " << req.node_name << " does not exists");
    return false;
  }
}

bool AsebaROS::setVariable(SetVariable::Request &req,
                           SetVariable::Response &res) {
  // lock the access to the member methods
  unsigned nodeId, pos, size;
  mutex.lock();
  bool success =
      getVariableInfo(req.node_name, req.variable_name, nodeId, pos, size);
  mutex.unlock();
  if (!success)
    return false;
  Aseba::SetVariables msg(nodeId, pos, req.data);
  hub.sendMessage(&msg, true);
  return true;
}

bool AsebaROS::getVariable(GetVariable::Request &req,
                           GetVariable::Response &res) {
  unsigned nodeId, pos, length;
  // lock the access to the member methods, wait will unlock the underlying
  // mutex
  std::unique_lock<std::mutex> lock(mutex);
  // get information about variable
  bool ok = getVariableInfo(req.node_name, req.variable_name, nodeId, pos, length);
  if (!ok) {
    return false;
  }
  // TODO(Jerome): this only work for internal. For external I should use the method above
  // Then bring to ROS2 too

  // create query
  const GetVariableQueryKey key(nodeId, pos);
  GetVariableQueryValue query;
  getVariableQueries[key] = &query;
  lock.unlock();

  // send message, outside lock to avoid deadlocks
  Aseba::GetVariables msg(nodeId, pos, length);
  hub.sendMessage(&msg, true);

  // system_time const timeout(get_system_time() +
  // posix_time::milliseconds(100)); wait 100 ms, considering the possibility of
  // spurious wakes

  lock.lock();
  bool result = query.cond.wait_for(lock, 100ms) == std::cv_status::no_timeout;
  // remove key and return answer
  getVariableQueries.erase(key);
  if (result) {
    res.data = query.data;
    return true;
  } else {
    ROS_ERROR("Reading variable %s of node %s did not return a valid answer within 100ms",
              req.variable_name.c_str(), req.node_name.c_str());
    return false;
  }
}

bool AsebaROS::getEventId(GetEventId::Request &req, GetEventId::Response &res) {
  // needs locking, called by ROS's service thread
  std::lock_guard<std::mutex> lock(mutex);
  size_t id;
  if (commonDefinitions.events.contains(widen(req.name), &id)) {
    res.id = id;
    return true;
  }
  return false;
}

bool AsebaROS::getEventName(GetEventName::Request &req,
                            GetEventName::Response &res) {
  // needs locking, called by ROS's service thread
  std::lock_guard<std::mutex> lock(mutex);
  if (req.id < commonDefinitions.events.size()) {
    res.name = narrow(commonDefinitions.events[req.id].name);
    return true;
  }
  return false;
}

bool AsebaROS::getVariableInfo(const std::string &nodeName,
                               const std::string &variableName,
                               unsigned &nodeId, unsigned &pos, unsigned &length) const {
  // does not need locking, called by other member function already within the
  // lock

  // make sure the node exists
  NodesNamesMap::const_iterator nodeIt(nodesNames.find(nodeName));
  if (nodeIt == nodesNames.end()) {
    ROS_ERROR("node %s does not exists", nodeName.c_str());
    return false;
  }
  nodeId = nodeIt->second;
  pos = unsigned(-1);

  // check whether variable is user-defined
  const UserDefinedVariablesMap::const_iterator userVarMapIt(
      userDefinedVariablesMap.find(nodeName));
  if (userVarMapIt != userDefinedVariablesMap.end()) {
    const Aseba::VariablesMap &userVarMap(userVarMapIt->second);
    const Aseba::VariablesMap::const_iterator userVarIt(
        userVarMap.find(widen(variableName)));
    if (userVarIt != userVarMap.end()) {
      pos = userVarIt->second.first;
      length = userVarIt->second.second;
    } else {
    }
  }

  // if variable is not user-defined, check whether it is provided by this node
  if (pos == unsigned(-1)) {
    bool ok;
    pos = getVariablePos(nodeId, widen(variableName), &ok);
    if (!ok) {
      ROS_ERROR("Variable %s does not exists in node %s (%d)",
                variableName.c_str(), nodeName.c_str(), nodeId);
      return false;
    }
    length = getVariableSize(nodeId, widen(variableName), &ok);
    if (!ok) {
      ROS_ERROR("Variable %s has unknown size in node %s (%d)",
                variableName.c_str(), nodeName.c_str(), nodeId);
      return false;
    }
  }
  return true;
}

void AsebaROS::sendEventOnROS(const Aseba::UserMessage *asebaMessage) {
  if (ignore_node(asebaMessage->source))
    return;

  // does not need locking, called by other member function already within lock
  // if different, we are currently loading a new script, publish on anonymous
  // channel
  if ((pubs.size() == commonDefinitions.events.size()) &&
      (asebaMessage->type < commonDefinitions.events.size())) {
    // known, send on a named channel
    AsebaEvent event;
    event.stamp = ros::Time::now();
    event.source = asebaMessage->source;
    event.data = asebaMessage->data;
    pubFor(asebaMessage).publish(event);
    // pubs[asebaMessage->type].publish(event);
  } else {
    // unknown, send on the anonymous channel
    AsebaAnonymousEvent event;
    event.stamp = ros::Time::now();
    event.source = asebaMessage->source;
    event.type = asebaMessage->type;
    event.data = asebaMessage->data;
    anonPub.publish(event);
  }
}

void AsebaROS::nodeDescriptionReceived(unsigned nodeId) {
  // does not need locking, called by parent object
  std::string name = narrow(nodes.at(nodeId).name);
  ROS_INFO("Received %s description of a node with name %s and id %d",
           (nodes[nodeId].isComplete() ? "complete" : "uncomplete"),
           name.data(), nodeId);
  nodesNames[name] = nodeId;
  ros::Duration(1.0).sleep();
  if (!bytecode.count(name))
    compile_script();
  else
    ROS_INFO("Script already compiled for name %s", name.c_str());
  // CHANGED: Let us load a script if available!
  load_script_to(nodeId);
}

void AsebaROS::eventReceived(const AsebaAnonymousEventConstPtr &event) {
  // does not need locking, does not touch object's members
  if (event->source == 0) {
    // forward only messages with source 0, which means, originating from this
    // computer
    Aseba::UserMessage userMessage(event->type, event->data);
    hub.sendMessage(&userMessage, true);
  }
}

void AsebaROS::knownEventReceived(const uint16_t id, const uint16_t nodeId,
                                  const AsebaEventConstPtr &event) {
  // does not need locking, does not touch object's members
  if (event->source == 0) {
    // forward only messages with source 0, which means, originating from this
    // computer
    Aseba::VariablesDataVector data = event->data;
    if (!manage_single_node)
      data.insert(data.begin(), nodeId);
    Aseba::UserMessage userMessage(id, data);
    hub.sendMessage(&userMessage, true);
  }
}

void AsebaROS::sendMessage(const Aseba::Message &message) {
  // not sure if use true or false (to lock or not to lock)
  hub.sendMessage(&message, false);
}

// hub for dashel
AsebaROS::AsebaROS(unsigned port, bool forward)
    : n("aseba"), nh("~"), fanout(true),
      anonPub(n.advertise<AsebaAnonymousEvent>("anonymous_events", 100)),
      anonSub(
          n.subscribe("anonymous_events", 100, &AsebaROS::eventReceived, this)),
      nodes_pub(n.advertise<AsebaNodeList>("nodes", 1, true)),
      hub(this, port, forward) {
  // does not need locking, called by main
  nh.param<bool>("single", manage_single_node, false);
  if (manage_single_node)
    ROS_INFO("Will manage only the first valid connected node");
  if (nh.getParam("script/path", script_path)) {
    ROS_INFO("Initializing with script %s", script_path.c_str());
    read_script_header();
  } else {
    ROS_INFO("Initializing without script");
  }
  // script
  s.push_back(n.advertiseService("load_script", &AsebaROS::loadScript, this));
  // nodes
  s.push_back(
      n.advertiseService("get_node_list", &AsebaROS::getNodeList, this));
  s.push_back(n.advertiseService("get_node_ids", &AsebaROS::getNodeIds, this));
  s.push_back(
      n.advertiseService("get_node_name", &AsebaROS::getNodeName, this));
  // variables
  s.push_back(n.advertiseService("get_variable_list",
                                 &AsebaROS::getVariableList, this));
  s.push_back(n.advertiseService("set_variable", &AsebaROS::setVariable, this));
  s.push_back(n.advertiseService("get_variable", &AsebaROS::getVariable, this));
  // events
  s.push_back(n.advertiseService("get_event_id", &AsebaROS::getEventId, this));
  s.push_back(
      n.advertiseService("get_event_name", &AsebaROS::getEventName, this));
  shutdown_on_unconnect = nh.param<bool>("shutdown_on_unconnect", false);
}

AsebaROS::~AsebaROS() {
  // does not need locking, called by main
  xmlCleanupParser();
}

void AsebaROS::pingCallback(const ros::TimerEvent &) { pingNetwork(); }

// TODO(Jerome): review as different from ROS2 version
void AsebaROS::run() {
  // does not need locking, called by main
  hub.startThread();
  ros::Timer timer =
      n.createTimer(ros::Duration(1), &AsebaROS::pingCallback, this);
  ros::spin();
  hub.stopThread();
}

void AsebaROS::processAsebaMessage(Aseba::Message *message) {
  // scan this message for nodes descriptions
  Aseba::NodesManager::processMessage(message);

  // needs locking, called by Dashel hub
  // TODO(Jerome): why?
  std::lock_guard<std::mutex> lock(mutex);

  // if user message, send to D-Bus as well
  Aseba::UserMessage *userMessage = dynamic_cast<Aseba::UserMessage *>(message);
  if (userMessage)
    sendEventOnROS(userMessage);

  // if variables, check for pending answers
  Aseba::Variables *variables = dynamic_cast<Aseba::Variables *>(message);
  if (variables) {
    const GetVariableQueryKey queryKey(variables->source, variables->start);
    GetVariableQueryMap::const_iterator queryIt(
        getVariableQueries.find(queryKey));
    if (queryIt != getVariableQueries.end()) {
      queryIt->second->data = variables->variables;
      queryIt->second->cond.notify_one();
    } else {
      ROS_WARN_STREAM("received Variables from node "
                      << variables->source << ", pos " << variables->start
                      << ", but no corresponding query was found");
    }
  }
}

void AsebaROS::unconnect() {
  if (shutdown_on_unconnect) {
    ROS_INFO("Will shutdown the node.");
    ros::shutdown();
  } else {
    ROS_INFO("Will ignore losing connection.");
  }
}

void AsebaROS::stopAllNodes() {
  for (const auto &node : nodes) {
    if (ignore_node(node.first))
      continue;
    Aseba::Reset msg_r(node.first);
    ROS_INFO("Reset node with id %d", node.first);
    hub.sendMessage(&msg_r, true);
    ros::Duration(1).sleep();
  }
}

std::vector<unsigned> AsebaROS::get_node_ids(const std::wstring &name) {
  // search for all nodes with a given name
  std::vector<unsigned> nodeIds;
  for (const auto &node : nodes) {
    if (node.second.name == name && !ignore_node(node.first)) {
      nodeIds.push_back(node.first);
    }
  }
  return nodeIds;
}

#define IGNORE "-"

bool AsebaROS::ignore_node(unsigned id) {
  if (manage_single_node && running_nodes.size() &&
      running_nodes.count(id) == 0)
    return true;
  return namespace_for_node(id) == IGNORE;
}

std::string AsebaROS::namespace_for_node(unsigned id, std::string name) {
  if (namespaces.count(id) == 0) {
    if (name.empty())
      name = node_name(id);
    if (name.empty())
      return "";
    std::string ns;
    bool accept;
    // HACK(Jerome): ROS1 does not like - in names
    std::replace(name.begin(), name.end(), '-', '_');
    if (nh.getParam("nodes/" + name + "/" + std::to_string(id), ns)) {
    } else if (nh.getParam("nodes/" + name + "/accept_all", accept) && !accept) {
      ns = IGNORE;
    } else if (nh.getParam("nodes/" + name + "/prefix", ns)) {
      ns = ns + std::to_string(id);
    } else if (name != "*") {
      ns = namespace_for_node(id, "*");
    } else {
      ns = "node" + std::to_string(id);
    }
    if (manage_single_node && ns != "-")
      ns = "";
    namespaces[id] = ns;
    ROS_INFO("Namespace for node with name %s and id %d is %s", name.data(), id,
             ns.data());
  }
  return namespaces[id];
}

std::string AsebaROS::topic(unsigned node_id, const std::string &topic_name) {
  std::string tn = topic_name;
  if (manage_single_node)
    return tn;
  return namespace_for_node(node_id) + "/" + tn;
}

std::string AsebaROS::node_name(unsigned int id) {
  // HACK:
  // TODO(Jerome): Understand and check all locks. Which are called from ROS ...
  // With the lock below, it blocks the second update
  // std::lock_guard<std::mutex> lock(mutex);
  auto node_it(nodes.find(id));
  if (node_it != nodes.end()) {
    return narrow(node_it->second.name);
  }
  return "";
}

ros::Publisher AsebaROS::pubFor(const Aseba::UserMessage *asebaMessage) {
  unsigned type = asebaMessage->type;
  unsigned source = asebaMessage->source;
  if (pubs[type].count(source) == 0) {
    const std::wstring &name(commonDefinitions.events[type].name);
    pubs[type][source] =
        n.advertise<AsebaEvent>(topic(source, narrow(name)), 100);
  }
  return pubs[type][source];
}

void AsebaROS::create_subscribers(unsigned node_id) {
  for (size_t i = 0; i < commonDefinitions.events.size(); ++i) {
    const std::wstring &name(commonDefinitions.events[i].name);
    subs[i][node_id] =
        n.subscribe<AsebaEvent>(topic(node_id, narrow(name)), 100,
                    [this, i, node_id](const AsebaEventConstPtr &event) {
                      knownEventReceived(i, node_id, event);
                    });
  }
}

//! Show usage
void dumpHelp(std::ostream &stream, const char *programName) {
  stream
      << "AsebaROS, connects aseba components together and with ROS, usage:\n";
  stream << programName << " [options] [additional targets]*\n";
  stream << "Options:\n";
  stream << "-l, --loop      : makes the switch transmit messages back to the "
            "send, not only forward them.\n";
  stream << "-p port         : listens to incoming connection on this port\n";
  stream << "-h, --help      : shows this help\n";
  stream << "ROS_OPTIONS     : see ROS documentation\n";
  stream << "Additional targets are any valid Dashel targets." << std::endl;
}

// void nodeConnectedSignal(unsigned nodeId)
// {
// }

// TODO(Jerome): review

// void AsebaROS::nodeDisconnected(unsigned nodeId) {
//   ROS_WARN_STREAM("Node " << nodeId << " has been disconnected");
// }

void AsebaROS::update_diagnostics(
    diagnostic_updater::DiagnosticStatusWrapper &stat) {
  stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "");
  for (const auto &node : nodes) {
    std::ostringstream oss;
    oss << "ignored: " << (ignore_node(node.first) ? "true" : "false")
        << "\ttype: " << narrow(node.second.name)
        << "\tname: " << namespace_for_node(node.first)
        << "\tconnected: " << (node.second.connected ? "true" : "false")
        << "\tcomplete: " << (node.second.isComplete() ? "true" : "false");

    std::ostringstream tss;
    tss << "Node " << node.first;
    stat.add(tss.str(), oss.str());
  }
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "aseba");
  diagnostic_updater::Updater updater;
  updater.setHardwareID("aseba-ros");

  unsigned port = ASEBA_DEFAULT_PORT;
  bool forward = true;
  std::vector<std::string> additionalTargets;

  int argCounter = 1;

  // TODO(Jerome): review as different from ros2
  while (argCounter < argc) {
    const char *arg = argv[argCounter];

    if ((strcmp(arg, "-l") == 0) || (strcmp(arg, "--loop") == 0)) {
      forward = false;
    } else if (strcmp(arg, "-p") == 0) {
      arg = argv[++argCounter];
      port = atoi(arg);
    } else if ((strcmp(arg, "-h") == 0) || (strcmp(arg, "--help") == 0)) {
      dumpHelp(std::cout, argv[0]);
      return 0;
    } else {
      additionalTargets.push_back(argv[argCounter]);
    }
    argCounter++;
  }
  Dashel::initPlugins();
  AsebaROS asebaROS(port, forward);

  updater.add("Aseba Network", &asebaROS, &AsebaROS::update_diagnostics);

  ros::NodeHandle n;

  ros::Timer timer =
      n.createTimer(ros::Duration(1),
                    [&updater](const ros::TimerEvent &) { updater.update(); });

  bool connected = false;
  ROS_INFO("Waiting for connections");
  while (ros::ok() && !connected) {
    for (auto &target : additionalTargets) {
      ROS_INFO("Connecting to %s", target.c_str());
      try {
        asebaROS.connectTarget(target);
        connected = true;
      } catch (Dashel::DashelException e) {
        ROS_ERROR("While connecting to target %s: %s", target.c_str(),
                  e.what());
      }
    }
    if (!connected) {
      ROS_WARN(
          "Could not connect to any target. Sleep for 1 second and then retry");
      ros::Duration(1).sleep();
    }
  }
  // TODO(Jerome): why?
  ros::Duration(1).sleep();

  asebaROS.run();

  asebaROS.stopAllNodes();

  ROS_INFO("Will shutdown node");

  return 0;
}
