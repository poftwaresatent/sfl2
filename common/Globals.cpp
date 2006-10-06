#include "Globals.hpp"
#include <sfl/util/Pthread.hpp>
#include <iostream>

using namespace sfl;
using namespace boost;
using namespace std;

namespace npm {
  
  shared_ptr<Condition> SimulatorIdleCondition()
  {
    static shared_ptr<Condition> instance;
    if( ! instance){
      instance = Condition::Create("simulator_idle");
      if( ! instance){
	cerr << "npm::SimulatorIdleCondition(): Condition::Create() failed\n";
	exit(EXIT_FAILURE);
      }
    }
    return instance;
  }
  
  shared_ptr<Mutex> SimulatorMutex()
  {
    static shared_ptr<Mutex> instance;
    if( ! instance){
      instance = Mutex::Create("simulator_mutex");
      if( ! instance){
	cerr << "npm::SimulatorMutex(): Mutex::Create() failed\n";
	exit(EXIT_FAILURE);
      }
    }
    return instance;
  }
  
}
