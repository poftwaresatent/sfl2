/*
 * Copyright (c) 2008 Roland Philippsen <roland DOT philippsen AT gmx DOT net>
 *
 * BSD license:
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of
 *    contributors to this software may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHORS AND CONTRIBUTORS ``AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR THE CONTRIBUTORS TO THIS SOFTWARE BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ROSHAL.hpp"
#include <boost/shared_ptr.hpp>

#include "ros/node.h"

#include <err.h>
#include <signal.h>
#include <unistd.h>
#include <iostream>

using namespace std;
using namespace boost;


static void cleanup();

static shared_ptr<ROSHAL> hal;
static double qdot[2];
static size_t qdot_len(2);


int main(int argc, char **argv)
{
  ros::init(argc, argv);
  if ( -1 == atexit(cleanup))
    err(EXIT_FAILURE, "atexit()");
  hal.reset(new ROSHAL(2, &cerr));
  qdot[0] = 0.2;
  qdot[1] = 0.05;
  while (true) {
    if (0 != hal->speed_set(qdot, &qdot_len))
      errx(EXIT_FAILURE, "hal->speed_set() failed");
    usleep(1000000);
  }
  return EXIT_SUCCESS;
}


static void cleanup()
{
  if (0 != hal.get()) {
    qdot[0] = 0;
    qdot[1] = 0;
    if (0 != hal->speed_set(qdot, &qdot_len))
      warnx("cleanup(): hal->speed_set() failed");
  }
  ros::fini();
}
