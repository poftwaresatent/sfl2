/* 
 * Copyright (C) 2012 Roland Philippsen. All rights reserved.
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307
 * USA
 */

#ifndef NPM_EXT_PYROB_PYNPM_HPP
#define NPM_EXT_PYROB_PYNPM_HPP

#include <Python.h>


namespace npm {
class RobotServer;
}

namespace pynpm {

void init(void);

PyObject *createServer(npm::RobotServer *server);

}

#endif // NPM_EXT_PYROB_PYNPM_HPP
