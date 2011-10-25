/* +---------------------------------------------------------------------------+
   |                       Recursive World Toolkit                             |
   |                                                                           |
   |   Copyright (C) 2011  Jose Luis Blanco Claraco                            |
   |                                                                           |
   |     RWLC is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   RWLC is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with RWLC.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */

#pragma once

#include <iostream>
#include <sstream>
#include <string>
#include <stdexcept>


// Useful macros ---------------------
#if defined(__BORLANDC__)
#   define _CURRENT_FUNC_       __FUNC__
#else
#   define _CURRENT_FUNC_       __FUNCTION__
#endif

#define RWLASSERT(_F) \
    { \
        if (!(_F)) \
        { \
            std::stringstream s; \
            s << _CURRENT_FUNC_ << ":" << __LINE__ << ": Assert failed: " << #_F; \
            throw std::runtime_error(s.str()); \
        } \
    } \

#ifdef _DEBUG
#   define RWLASSERT_DEBUG(_F) RWLASSERT(_F)
#else
#   define RWLASSERT_DEBUG(_F)
#endif

/** Usage: RWL_MESSAGE << "blah" << x << endl;
  */
#define RWL_MESSAGE std::cout << "[" << _CURRENT_FUNC_ << "] "

// Define a decl. modifier for printf-like format checks at compile time:
#ifdef __GNUC__
#	define RWL_printf_format_check(_FMT_,_VARARGS_)  __attribute__ ((__format__ (__printf__, _FMT_,_VARARGS_)))
#else
#	define RWL_printf_format_check(_FMT_,_VARARGS_)
#endif

namespace rwl
{
	/** Returns the numeric representation of a string, or raises an exception if it's not a valid number */
	double str2num(const std::string &s);

}

