#include "common/consts.h"

#include "asebaros/aseba_script.h"
#include "asebaros/utils.h"


#include "libxml/parser.h"
#include "libxml/tree.h"

static bool compile_script(const Aseba::TargetDescription *description,
                           const Aseba::CommonDefinitions *common_definitions,
                           const std::string &code,
                           Aseba::VariablesMap &variable_map,
                           Aseba::BytecodeVector &bytecode) {
  std::wistringstream is(widen(code));
  unsigned allocatedVariablesCount;
  Aseba::Compiler compiler;
  Aseba::Error error;
  compiler.setTargetDescription(description);
  compiler.setCommonDefinitions(common_definitions);
  bool result = compiler.compile(is, bytecode, allocatedVariablesCount, error);
  if (!result) {
    LOG_ERROR("Failed to compile script for node %ls: %ls",
              description->name.c_str(), error.toWString().c_str());
    return false;
  }
  variable_map = *compiler.getVariablesMap();
  return true;
}

static std::shared_ptr<AsebaScript> load_xml(const xmlDoc *doc,
                                             const std::string source) {
  std::shared_ptr<AsebaScript> script = std::make_shared<AsebaScript>();
  xmlNode *domRoot = xmlDocGetRootElement(doc);
  if (!xmlStrEqual(domRoot->name, BAD_CAST("network"))) {
    LOG_ERROR("root node is not \"network\", XML considered as invalid");
    return nullptr;
  }
  for (xmlNode *domNode = xmlFirstElementChild(domRoot); domNode;
       domNode = domNode->next) {
    if (domNode->type == XML_ELEMENT_NODE) {
      if (xmlStrEqual(domNode->name, BAD_CAST("event"))) {
        // get attributes
        xmlChar *name = xmlGetProp(domNode, BAD_CAST("name"));
        if (!name)
          LOG_WARN("missing \"name\" attribute in \"event\" entry");
        xmlChar *size = xmlGetProp(domNode, BAD_CAST("size"));
        if (!size)
          LOG_WARN("missing \"size\" attribute in \"event\" entry");
        // add event
        if (name && size) {
          int eventSize(atoi((const char *)size));
          if (eventSize > ASEBA_MAX_EVENT_ARG_SIZE) {
            LOG_ERROR("Event %s has a length %d larger than maximum %d", name,
                      eventSize, ASEBA_MAX_EVENT_ARG_SIZE);
            xmlFree(name);
            xmlFree(size);
            return nullptr;
          } else {
            script->common_definitions.events.push_back(
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
          LOG_WARN("missing \"name\" attribute in \"constant\" entry");
        xmlChar *value = xmlGetProp(domNode, BAD_CAST("value"));
        if (!value)
          LOG_WARN("missing \"value\" attribute in \"constant\" entry");
        // add constant if attributes are valid
        if (name && value) {
          int constant_value = atoi((const char *)value);
          std::wstring _name = widen((const char *)name);
          script->common_definitions.constants.push_back(
              Aseba::NamedValue(_name, constant_value));
        }
        // free attributes
        if (name)
          xmlFree(name);
        if (value)
          xmlFree(value);
      } else if (xmlStrEqual(domNode->name, BAD_CAST("keywords"))) {
        continue;
      } else if (xmlStrEqual(domNode->name, BAD_CAST("node"))) {
        // get attributes, child and content
        xmlChar *name = xmlGetProp(domNode, BAD_CAST("name"));
        if (!name) {
          LOG_WARN("missing \"name\" attribute in \"node\" entry");
        } else {
          const std::string _name((const char *)name);
          xmlChar *storedId = xmlGetProp(domNode, BAD_CAST("nodeId"));
          unsigned id = 0;
          if (storedId) {
            id = unsigned(atoi((char *)storedId));
            xmlFree(storedId);
          }
          xmlChar *text = xmlNodeGetContent(domNode);
          if (!text) {
            LOG_WARN("missing text in \"node\" entry");
          } else {
            script->code[_name][id] = std::string((const char *)text);
            xmlFree(text);
          }
          xmlFree(name);
        }
      } else {
        LOG_WARN("Unknown XML node seen in .aesl file: %s", domNode->name);
      }
    }
  }
  script->source = source;
  return script;
}

std::shared_ptr<AsebaScript>
AsebaScript::from_string(const std::string &value) {
  const unsigned char *text = (const unsigned char *)value.c_str();
  xmlDoc *doc = xmlReadDoc(text, NULL, NULL, 0);
  if (!doc) {
    LOG_ERROR("Cannot read XML from string");
    return nullptr;
  }
  return load_xml(doc, "from string");
}

std::shared_ptr<AsebaScript> AsebaScript::from_file(const std::string &path) {
  if (path.empty()) {
    LOG_ERROR("Script path %s is unvalid", path.c_str());
    return nullptr;
  }
  // addLOG_INFO("Will read script in %s", path.c_str());
  xmlDoc *doc = xmlReadFile(path.c_str(), NULL, 0);
  if (!doc) {
    LOG_ERROR("Cannot read XML from file %s", path.c_str());
    return nullptr;
  }
  return load_xml(doc, path);
}

bool AsebaScript::compile(unsigned node_id,
                          const Aseba::TargetDescription *description,
                          Aseba::VariablesMap &variable_map,
                          Aseba::BytecodeVector &bytecode) {
  std::string name = narrow(description->name);
  if (!code.count(name)) {
    LOG_WARN("No node in script for name %s: won't compile and load to Aseba "
             "node %d",
             name.c_str(), node_id);
    return false;
  }
  std::string aesl_code;
  if (code[name].count(node_id)) {
    aesl_code = code[name].at(node_id);
  } else if (code[name].count(0)) {
    aesl_code = code[name].at(0);
  } else if (code[name].size()) {
    aesl_code = code[name].begin()->second;
  } else {
    LOG_WARN("No node in script for name %s and id %d: : won't compile and "
             "load to Aseba node %d",
             name.c_str(), node_id, node_id);
  }
  return compile_script(description, &common_definitions, aesl_code,
                        variable_map, bytecode);
}
