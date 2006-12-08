#include "phelp.hpp"
#include <genPos/genPosStruct.h>
#include <iostream>

using namespace poster;
using namespace std;

int main(int argc, char ** argv)
{
  POSTER_ID poster_id(find("speedref", sizeof(GENPOS_CART_SPEED)));
  if(0 == poster_id)
    return -1;
  while(true){
    GENPOS_CART_SPEED speedref;
    if( ! read(poster_id, &speedref, sizeof(GENPOS_CART_SPEED)))
      return -2;
    cout << "read " << speedref.numRef << "\n";
    usleep(400000);
  }
}
