#ifndef ASEBAROS_INCLUDE_ASEBAROS_ASEBASCRIPT_H
#define ASEBAROS_INCLUDE_ASEBAROS_ASEBASCRIPT_H

#include <map>
#include <memory>

#include "compiler/compiler.h"

class AsebaScript {

  typedef std::map<std::string, std::map<unsigned, std::string>> CodeMap;

public:
  static std::shared_ptr<AsebaScript> from_string(const std::string &value);
  static std::shared_ptr<AsebaScript> from_file(const std::string &path);

  bool compile(unsigned node_id, const Aseba::TargetDescription *description,
               Aseba::VariablesMap &variable_map,
               Aseba::BytecodeVector &bytecode);
  AsebaScript() : common_definitions(), code(){};
  // user events and constants
  Aseba::CommonDefinitions common_definitions;
  // type -> (id -> aesl text)
  CodeMap code;
  std::string source;
};

#endif // ASEBAROS_INCLUDE_ASEBAROS_ASEBASCRIPT_H
