/* +---------------------------------------------------------------------------+
   |                       Recursive World Toolkit                             |
   |                                                                           |
   |   Copyright (C) 2011  Jose Luis Blanco Claraco                            |
   |                                                                           |
   |      RWT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |    RWT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with  RWT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */

#include "rwt.h"

#include <mrpt/system/string_utils.h>
#include <mrpt/system/filesystem.h>

using namespace rwt;
using namespace std;

bool rwt::compile_and_run_rwt_program(const std::string &file, RWT_Program &out_program, RWT_World & out_world, bool enable_file_caching)
{
	const string sCompiledFile = mrpt::system::fileNameChangeExtension( file, "crwt.gz" );

	// // Check for compiled file
	if (enable_file_caching && mrpt::system::fileExists(sCompiledFile))
	{
		// Load:
		if (rwt::load_rwt_world(out_world,sCompiledFile,file))
			return true;

		// If we are here it's because the compiled file doesn't exists or is outdated (source file changed).
	}

	// Regenerate:
	if (!compile_rwt_program(file,out_program)) return false;
	if (!run_rwt_program(out_program,out_world)) return false;

	// Save to cache:
	if (enable_file_caching)
	{
		if (!rwt::save_rwt_world(out_world, sCompiledFile,file))
			std::cerr << "Warning: Couldn't save compiled world to '" << sCompiledFile << "' (ignoring and going on)\n";
	}

	return true;
}
