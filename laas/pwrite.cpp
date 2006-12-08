#include "phelp.hpp"
#include <genPos/genPosStruct.h>
#include <iostream>
#include <signal.h>

using namespace poster;
using namespace std;

static POSTER_ID poster_id;

static void sighandle(int signum)
{
  exit(EXIT_SUCCESS);
}

static void cleanup()
{
  if(0 != poster_id)
    destroy(poster_id);
}

int main(int argc, char ** argv)
{
  atexit(cleanup);
  if(SIG_ERR == signal(SIGTERM, sighandle)){
    perror("signal(SIGTERM) failed");
    return -1;
  }
  poster_id = create("speedref", sizeof(GENPOS_CART_SPEED));
  for(int numRef(0); true; ++numRef){
    GENPOS_CART_SPEED speedref;
    speedref.numRef = numRef;
    if( ! write(poster_id, &speedref, sizeof(GENPOS_CART_SPEED)))
      return -2;
    cout << "wrote " << speedref.numRef << "\n";
    usleep(400000);
  }
}
