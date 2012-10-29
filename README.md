# Sunflower2 Mobile Robot Library and Nepumuk Simulator

**For the Impatient**

Build everything using the provided script:

    ./build-stage.sh

By default, this builds the software underneath the build/ directory
and installs it under the stage/ directory. This includes the
`stage/bin/nepumuk` binary, which is a simulator. To try it out,
launch one of the example scripts also installed there.

Documentation (if not included in the tarball you downloaded) can be
created if you have Doxygen and Graphviz installed:

    cd build
    make doc

This will create `html/index.html` underneath the top-level source
directory.

```
----------------------------------------------------------------------------
Copyright (C) 2004, Ecole Polytechnique Federale de Lausanne. All rights reserved.
Copyright (C) 2005, LAAS-CNRS. All rights reserved.
Copyright (C) 2007, ETH Zurich. All rights reserved.

Author: Roland Philippsen
Contributors: Viet Nguyen, Frederic Pont, Sascha Kolski, Agostino
              Martinelli, Kristijan Macek, and others

Most of this software is released under the GNU General Public License
(GPL), version 2. The files not covered by GPL are released under the
new BSD license. See the LICENSE.GPLv2 and LICENSE.BSD files for more
information.
----------------------------------------------------------------------------
```

## Introduction

Sunflower contains mobile robotic programming abstractions developed
mainly at the Autonomous Systems Lab (ASL), EPFL (now at ETH
Zurich). It is written in C++ and provides a framework of classes and
utilities for common tasks and patterns in mobile robotics. As a
concrete application of those abstractions, sunflower contains
obstacle avoidance code developed during a [PhD thesis][phd-thesis] at
the ASL (see chapter 3).

[phd-thesis]: http://library.epfl.ch/theses/?nr=3146


## Compiling

This software has been compiled under Linux, Mac OS X, OpenBSD, and
even once or twice under Windows. For a basic setup, you need

- a C++ compiler (tested with GCC 3 and 4)
- the [Boost][] smart_ptr and bind libraries

[Boost]: http://www.boost.org/

In order to build the Nepumuk simulator which comes bundled with
Sunflower, you will also need

- OpenGL, GLU, and GLUT development packages

We use GNU Automake with Libtool and Autoconf to provide a
cross-platform configure script. The recommended way of building is in
a separate directory:

    $ mkdir build
    $ cd build
    $ ../configure --prefix=/install/path
    $ make
    $ make install

The provided `build-statge.sh` script will take care of this for you,
and install everything underneath the top-level source directory in a
subdirectory called `stage.` The behavior of `build-stage.sh` script
can be controlled by passing arguments to it, read the output of
`./build-stage.sh -h` for more details.

Running `configure` manually gives you more fine-grained control,
however. Some of the commonly used configure options are

    --enable-debug       switches on debug options when you use GCC
    --enable-pedantic    switches on pedantic warnings when you use GCC
    --with-boost=PATH    lets you specify a non-standard location of the
                         required boost library
    --without-png        disable PNG support (problematic on Mac OS X)

Read the output of `configure --help` for more details.


## Running

If you have built the Nepumuk simulator (should happen automatically
if you have all required prerequisites), then you can start running
the examples that get installed into the `bin` directory.

In Nepumuk's GL window, press `c` for continuous mode, `SPACE` for
step-by-step simulation, and `q` to exit the program. Some more
keybindings exist, check the `apps/Simulator.cpp` code.


## Documentation

Sunflower uses [Doxygen][] to provide documentation through comments
inside the sourcecode. You can either rely on the fact that the vast
majority of these comments is in the class declarations (ie header
files), or use the `make doc` target which generates the HTML
documentation and places it underneath the top-level source directory
into a directory called `html.`

[Doxygen]: http://www.doxygen.org

## Project Layout

* sfl/util/

  Basic classes and other utilities.

* sfl/api/

  Mobile robotic abstractions, some with default implementations,
  others in form of interfaces.

* sfl/dwa/

  An implementation of the Dynamic Window Approach to obstacle
  avoidance, modified for differential drive robots and with
  pre-calculated lookup-tables for very fast execution.

* sfl/bband/

  An implementation of a radically simplified Elastic Band approach to
  on-line path modification, aimed at keeping computations very
  lightweight but sacrifycing accuracy.

* sfl/gplan/
  An early attempt at generalized grid-based path planners, only useful
  for it's implementation of the NF1 planner. (The [E-Star project][estar]
  provides a more powerful approach but has not been integrated with
  libsunflower yet.)

  [estar]: https://github.com/poftwaresatent/estar

* sfl/expo/

  The integrated path planning and obstacle avoidance system used
  during the Swiss National Exhibition expo.02. For 6 months it
  controlled 10 Robox tour guide robots at the Robotics pavillion.

* npm/

  The core classes for the Nepumuk simulator.

* npm/gfx

  Graphics output of Nepumuk.

* npm/ext

  Nepumuk's (currently simplistic) extension mechanism.

* apps/

  Sourcecode for executables.


## References

```
@phdthesis{philippsen:2004:thesis,
  author = {Philippsen, Roland},
  title  = {Motion Planning and Obstacle Avoidance for Mobile Robots
           in Highly Cluttered Dynamic Environments},
  school = {Ecole Polytechnique F\'ed\'erale de Lausanne},
  year   = 2004
}
```
