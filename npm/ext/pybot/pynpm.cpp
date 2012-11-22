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

#include <Python.h>
#include <npm/RobotServer.hpp>
#include <iostream>


namespace pynpm {
  
  typedef struct {
    PyObject_HEAD
    npm::RobotServer *server_;
  } ServerObject;
  
  
  static PyTypeObject ServerType = {
    PyObject_HEAD_INIT(NULL)
    0,                         /*ob_size*/
    "npm.Server",              /*tp_name*/
    sizeof(ServerObject),      /*tp_basicsize*/
    0,                         /*tp_itemsize*/
    0,                         /*tp_dealloc*/
    0,                         /*tp_print*/
    0,                         /*tp_getattr*/
    0,                         /*tp_setattr*/
    0,                         /*tp_compare*/
    0,                         /*tp_repr*/
    0,                         /*tp_as_number*/
    0,                         /*tp_as_sequence*/
    0,                         /*tp_as_mapping*/
    0,                         /*tp_hash */
    0,                         /*tp_call*/
    0,                         /*tp_str*/
    0,                         /*tp_getattro*/
    0,                         /*tp_setattro*/
    0,                         /*tp_as_buffer*/
    Py_TPFLAGS_DEFAULT,        /*tp_flags*/
    "Nepumuk RobotServer",     /* tp_doc */
  };
  
  
  static PyMethodDef ServerMethods[] = {
    {NULL}  /* Sentinel */
  };
  
  
  PyMODINIT_FUNC initpynpm(void) 
  {
    PyObject *module;
    ServerType.tp_new = PyType_GenericNew;
    if (PyType_Ready(&ServerType) < 0) {
      std::cerr << "ERROR: failed to initialize pynpm (ServerType not ready)\n";
      return;
    }
    
    module = Py_InitModule3("pynpm", ServerMethods, "Python extension module for Nepumuk.");
    Py_INCREF(&ServerType);
    PyModule_AddObject(module, "Server", (PyObject *)&ServerType);
  }
  
  
  void init(void)
  {
    if (NULL == getenv("PYTHONPATH")) {
      setenv("PYTHONPATH", ".", 1);
    }
    std::cerr << "DBG pynpm::init: initializing python\n";
    Py_Initialize();
    std::cerr << "DBG pynpm::init: loading pynpm\n";
    initpynpm();
  }
  
  
  PyObject *createServer(npm::RobotServer *server)
  {
    return 0;
  }
  
}
