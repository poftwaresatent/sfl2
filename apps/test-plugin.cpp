#include <npm/Plugin.hpp>
#include <iostream>

using namespace std;


int main (int argc, char ** argv)
{
  if (argc < 2) {
    cout << "Please specify some plugin file names on the command line\n";
    return 42;
  }
  for (int ii (1); ii < argc; ++ii) {
    npm::Plugin plugin;
    if (plugin.load (argv[ii], true, cout)) {
      cout << "loaded " << argv[ii] << "\n";
    }
  }
  return 0;
}