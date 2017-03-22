#ifndef GATHER_H
#define GATHER_H

#include <sstream>

#include "task.h"

class Gather : public Task {
  public:
    Gather(std::string problem, int objCount, int * objectives);

    virtual Status operator()();

    virtual std::string str() const;
    virtual std::string details() const;

};

inline Gather::Gather(std::string problem, int objCount,
    int * objectives) :
    Task(problem, objCount, objCount, objectives){
}

inline Status Gather::operator()() {
  gatherSolutions();
  return status_;
}

inline std::string Gather::str() const {
  return std::string("Gathering task");
}

inline std::string Gather::details() const {
  std::stringstream ss(str());
  ss << " is " << status_ << std::endl;
  return ss.str();
}

#endif /* GATHER_H */
