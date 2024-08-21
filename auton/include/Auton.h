#pragma once

#include <algorithm>
#include <functional>
#include <string>
#include <vector>

struct Inst {
    std::string name;
    std::vector<char> argData;
};

struct InstDef {
    std::string name;
    std::vector<std::string> args;
    void* fn;
};

struct Step {
    int x;
    int y;
    std::vector<Inst> insts;
};

class Auton {
  public:
    std::string name;
    std::vector<Step> steps;
    std::vector<InstDef> instDefs;

    template <typename T>
    void setInstructionFunction(const std::string& name, T fn) {
      InstDef inst = *std::find_if(instDefs.begin(), instDefs.end(), [name](InstDef inst) -> bool { return inst.name == name; });

      inst.fn = new std::function<void(int, int)>(fn);
    }
};
