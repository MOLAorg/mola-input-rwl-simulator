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

#include <mrpt/utils/CConfigFile.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/string_utils.h>
#include <mrpt/random.h>

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

		// Run the program to build the world ----------------
		RWT_World the_world;

		const int random_seed = cfg.read_int("world","random_seed",-1 /* default val */);
		if (random_seed<0)
		     mrpt::random::randomGenerator.randomize();
		else mrpt::random::randomGenerator.randomize(random_seed);


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
		mrpt::gui::CDisplayWindow3DPtr win3D;

		const bool  show_live_3D = cfg.read_bool("path","show_live_3D", true);

		if (show_live_3D )
		{
			win3D = mrpt::gui::CDisplayWindow3D::Create("RWL compilation result",640,480);
			win3D->getDefaultViewport()->setCustomBackgroundColor( mrpt::utils::TColorf(0.2f,0.2f,0.2f) );

			mrpt::opengl::CSetOfObjectsPtr gl_world = mrpt::opengl::CSetOfObjects::Create(); // Create smart pointer to new object
			rwt::world_to_opengl(the_world, *gl_world);

			mrpt::opengl::COpenGLScenePtr &scene = win3D->get3DSceneAndLock();
			scene->insert( gl_world );
			win3D->unlockAccess3DScene();

			win3D->repaint();
			//win3D->waitForKey();
		}

		// --------------------------------------------------------
		// Generate path
		// --------------------------------------------------------
		cout << "[3>] Generating path...\n"; cout.flush();

		std::vector<mrpt::math::TPoint3D>  simulation_waypoints;

		// 2) A sequence of node IDs, corresponding to the 0-based indices of the RWL
		//      program "NODE" primitives:
		//  source_node_IDs = ID_1 ID_2 ID_3 .... ID_N
		const string sSourceNodeIDs = cfg.read_string("path","source_node_IDs","");
		if (!sSourceNodeIDs.empty())
		{
			vector<string> lst;
			mrpt::system::tokenize(sSourceNodeIDs," \t,",lst);
			ASSERTMSG_(!lst.empty(), "[path].source_node_IDs: Expected list of IDs!")

			simulation_waypoints.reserve(lst.size());

			for (size_t i=0;i<lst.size();i++)
			{
				const size_t idx = static_cast<size_t>( rwt::str2num(lst[i]) );
				ASSERT_BELOW_(idx,the_world.nodes.size())

				float x,y,z;
				the_world.nodes.getPoint(idx,x,y,z);
				simulation_waypoints.push_back( mrpt::math::TPoint3D(x,y,z) );
			}
		}
		else
		{
			// ...
		}

		cout << "[3<] Path generated OK: " << simulation_waypoints.size() << " waypoints.\n"; cout.flush();

		// load the rest of options:
		// ------------------------------
		RWT_PathOptions     pathParams;
		pathParams.max_step_lin = cfg.read_double("path","max_step_lin", 0.10);
		pathParams.max_step_ang = mrpt::utils::DEG2RAD( cfg.read_double("path","max_step_ang", 10) );

		RWT_SensorOptions   sensorParams;

		RWT_OutputOptions   outputParams;
		outputParams.win3D = win3D;
		outputParams.show_live_3D = win3D.present();
		outputParams.show_live_3D_sleep_ms = cfg.read_int("path","show_live_3D_sleep_ms", 10);


		if (outputParams.is_binary)
		{
			THROW_EXCEPTION("TO DO")
		}
		else
		{
			//const string sOutOdometry = cfg.read_string("dataset-format","output_text_sensor", "", true /* fail if not present */) ;

			const string sOutSensor = cfg.read_string("dataset-format","output_text_sensor", "", true /* fail if not present */) ;
			outputParams.output_text_sensor.open(sOutSensor.c_str());
			ASSERTMSG_(outputParams.output_text_sensor.is_open(), mrpt::format("Couldn't open output file: '%s'",sOutSensor.c_str() ) )

			const string sOutGT     = cfg.read_string("dataset-format","output_text_gt", "", true /* fail if not present */) ;
			outputParams.output_text_groundtruth.open(sOutGT.c_str());
			ASSERTMSG_(outputParams.output_text_groundtruth.is_open(), mrpt::format("Couldn't open output file: '%s'",sOutGT.c_str() ) )
			outputParams.output_text_groundtruth <<
				"# STEP     X       Y        Z        QR        QX      QY      QZ     \n"
				"# --------------------------------------------------------------------\n";
		}

		// And launch simulation:
		// ----------------------------
		cout << "[4>] Running simulator...\n"; cout.flush();

		const int random_seed_sensor = cfg.read_int("sensor","random_seed",-1 /* default val */);
		if (random_seed_sensor<0)
		     mrpt::random::randomGenerator.randomize();
		else mrpt::random::randomGenerator.randomize(random_seed_sensor);


		simulate_rwt_dataset(simulation_waypoints, the_world, pathParams, sensorParams, outputParams);

		cout << "[4<] Simulation done!\n"; cout.flush();

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

