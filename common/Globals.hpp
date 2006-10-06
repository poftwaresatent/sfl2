#ifndef NPM_GLOBALS_HPP
#define NPM_GLOBALS_HPP

#include <boost/shared_ptr.hpp>

namespace sfl {
  class Condition;
  class Mutex;
}

namespace npm {
  boost::shared_ptr<sfl::Condition> SimulatorIdleCondition();
  boost::shared_ptr<sfl::Mutex> SimulatorMutex();
}

#endif // NPM_GLOBALS_HPP
