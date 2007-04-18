/* 
 * Copyright (C) 2007
 * Swiss Federal Institute of Technology, Lausanne. All rights reserved.
 * 
 * Developed at the Autonomous Systems Lab.
 * Visit our homepage at http://asl.epfl.ch/
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

#ifndef NPM_PDEBUG_HPP
#define NPM_PDEBUG_HPP

#include <sfl/util/pdebug.hpp>

#ifdef NPM_VERBOSE_DEBUG
# define NPM_DEBUG
# define PVDEBUG PDEBUG_OUT
#else // ! NPM_VERBOSE_DEBUG
# define PVDEBUG PDEBUG_OFF
#endif // NPM_VERBOSE_DEBUG

#ifdef NPM_DEBUG
# define PDEBUG PDEBUG_OUT
#else // ! NPM_DEBUG
# define PDEBUG PDEBUG_OFF
#endif // NPM_DEBUG

#endif // NPM_PDEBUG_HPP
