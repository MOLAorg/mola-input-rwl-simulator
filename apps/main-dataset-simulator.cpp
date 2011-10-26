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

#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/string_utils.h>

#include "rwt.h"

using namespace rwt;
using namespace std;

// Aux funcs:
string completePath(const string &abs_or_relative_path, const string &script_base_filename);

// ---------------
//      Main
// ---------------
int main(int argc, char**argv)
{
	try
	{
		if (argc!=2)
		{
			cout << "Usage:\n" << argv[0] << " dataset-script.cfg\n";
			return 1;
		}

		const string sCfgFil = string(argv[1]);
		ASSERT_FILE_EXISTS_(sCfgFil)

		// Parse cfg file:
		mrpt::utils::CConfigFile  cfg(sCfgFil);

		// Get input RWL file:
		string sFil = cfg.read_string("world","input","", true /* fail if not found*/ );
		sFil = completePath(sFil,sCfgFil);

		// Compile ----------------------------
		cout << "[1>] Compiling '"<< sFil << "'...\n"; cout.flush();
		RWT_Program program;
		if (!compile_rwt_program(sFil,program))
		{
			cerr << "*ERROR* Program compilation failed\n";
			return 1;
		}
		cout << "[1<] Compilation succeeded!\n";

		// Run ----------------------------
		RWT_World the_world;

		cout << "[2>] Building world...\n"; cout.flush();
		if (!run_rwt_program(program,the_world))
		{
			cerr << "*ERROR* Program compilation failed\n";
			return 1;
		}
		cout << "[2<] World contruction succeeded!\n";
		// Stats:
		if (1)
		{
			cout << "# landmarks    : " << the_world.landmarks.size() << endl;
			cout << "# way-points   : " << the_world.nodes.size() << endl;
			cout << "# path segments: " << the_world.graph.edges.size() << endl;
		}

		// Display ----------------------------
		mrpt::gui::CDisplayWindow3D win3D("RWL compilation result",640,480);
		win3D.getDefaultViewport()->setCustomBackgroundColor( mrpt::utils::TColorf(0.2f,0.2f,0.2f) );

		mrpt::opengl::CSetOfObjectsPtr gl_world = mrpt::opengl::CSetOfObjects::Create(); // Create smart pointer to new object
		rwt::world_to_opengl(the_world, *gl_world);

		mrpt::opengl::COpenGLScenePtr &scene = win3D.get3DSceneAndLock();
		scene->insert( gl_world );
		win3D.unlockAccess3DScene();

		win3D.repaint();
		win3D.waitForKey();

		return 0;
	} catch (exception &e) {
		cerr << "Exception: " << e.what() << endl;
		return -1;
	}
}


// Aux. func for adding a path prefix as needed.
string completePath(const string &abs_or_relative_path, const string &script_base_filename)
{
	const string sFil = mrpt::system::trim(abs_or_relative_path);

	// Is it an absolute path?
	if (!sFil.empty() && ( sFil[0]=='/' || sFil[0]=='\\' || sFil[1]==':' ) ) {
		// It's absolute, we're done.
		return sFil;
	}
	else {
		// It's relative to the config file path:
		return mrpt::system::filePathSeparatorsToNative(
			mrpt::system::extractFileDirectory(script_base_filename) +
			string("/") +
			sFil );
	}
}

