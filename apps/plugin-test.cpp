#include <npm/Plugin.hpp>
#include <iostream>


int npm_plugin_init (void)
{
  std::cout << "Hello from the test plugin!\n";
  return 0;
}


void npm_plugin_fini (void)
{
  std::cout << "Byebye from the test plugin!\n";
}
