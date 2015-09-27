/* +---------------------------------------------------------------------------+
   |                       Recursive World Toolkit                             |
   |                                                                           |
   |   Copyright (C) 2011-2015  Jose Luis Blanco Claraco                       |
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
#include <mrpt/graphs/dijkstra.h>

#include "rwt.h"

using namespace rwt;
using namespace std;

// Aux funcs:
string completePath(const string &abs_or_relative_path, const string &script_base_filename);


double graph_edge_weight(const RWT_adjacency_graph& graph, const mrpt::utils::TNodeID id_from, const mrpt::utils::TNodeID id_to, const RWT_adjacency_graph::edge_t &edge);
RWT_World *global_the_world; // Aux var used by graph_edge_weight

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

		// Compile & run the "world" model ----------------------------
		const int random_seed = cfg.read_int("world","random_seed",-1 /* default val */);
		if (random_seed<0)
		     mrpt::random::randomGenerator.randomize();
		else mrpt::random::randomGenerator.randomize(random_seed);


		cout << "[1>] Compiling and running '"<< sFil << "'...\n"; cout.flush();
		RWT_Program program;
		RWT_World the_world;
		global_the_world = &the_world; // used by aux. global functions

		if (!rwt::compile_and_run_rwt_program(sFil,program, the_world))
		{
			cerr << "*ERROR* Program compilation/run failed\n";
			return 1;
		}
		cout << "[1<] World generated or loaded succeeded!\n";

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
			gl_world->setName("world");

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
		const string sSourceNodePathIDs = cfg.read_string("path","source_node_path_IDs","");

		if (!sSourceNodeIDs.empty())
		{
			vector<string> lst;
			mrpt::system::tokenize(sSourceNodeIDs," \t,\r\n",lst);
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
		else if (!sSourceNodePathIDs.empty())
		{
			vector<string> lst;
			mrpt::system::tokenize(sSourceNodePathIDs," \t,\r\n",lst);
			ASSERTMSG_(!lst.empty(), "[path].source_node_path_IDs: Expected list of IDs!")

			simulation_waypoints.reserve(lst.size());

			// Go thru the list of waypoints given by the user, then complement it
			//  by finding the topological paths between them:
			std::vector<size_t> lst_idxs;
			size_t prev_idx = INVALID_NODEID;
			for (size_t i=0;i<lst.size();i++)
			{
				// "lst[i]" can be:
				//  - "#"        : A number
				//  - "RANDOM"   : We pick a random target node
				//  - "RANDOM*#" : We pick a sequene of ## random target nodes
				lst_idxs.clear(); // This will hold the sequence/unique IDs

				if (mrpt::system::strCmpI(lst[i],"RANDOM"))
				{
					size_t idx;
					mrpt::random::randomGenerator.drawUniformUnsignedIntRange(idx, /*min/max:*/ 0,the_world.nodes.size()-1);
					lst_idxs.push_back(idx);
				}
				else if (mrpt::system::strStartsI(lst[i],"RANDOM*"))
				{
					ASSERTMSG_(lst[i].size()>7,"RANDOM*# expects # of random nodes!")

					const size_t nRndNodes = static_cast<size_t>( rwt::str2num( lst[i].substr(7) ) );
					for (size_t k=0;k<nRndNodes;k++)
					{
						size_t idx;
						mrpt::random::randomGenerator.drawUniformUnsignedIntRange(idx, /*min/max:*/ 0,the_world.nodes.size()-1);
						lst_idxs.push_back(idx);
					}
				}
				else
				{
					const size_t idx = static_cast<size_t>( rwt::str2num(lst[i]) );
					ASSERT_BELOW_(idx,the_world.nodes.size())

					lst_idxs.push_back(idx);
				}

				for (size_t j=0;j<lst_idxs.size();j++)
				{
					const size_t idx = lst_idxs[j];

					// ==== Go to node "idx": ====
					// Directly move to the first node...
					if (prev_idx==INVALID_NODEID)
					{
						float x,y,z;
						the_world.nodes.getPoint(idx,x,y,z);
						simulation_waypoints.push_back( mrpt::math::TPoint3D(x,y,z) );

						// Save current node idx for the next iter:
						prev_idx = idx;
					}
					else
					{
						// ...and go on with the topological path:
						// Create a spanning tree with Dijkstra to find out the path between: prev_idx -> idx.
						const mrpt::graphs::CDijkstra<RWT_adjacency_graph> tree(
							the_world.graph,
							prev_idx,
							&graph_edge_weight
							);

						// Get path:
						mrpt::graphs::CDijkstra<RWT_adjacency_graph>::edge_list_t  lst_edges;
						tree.getShortestPathTo(idx, lst_edges);

						ASSERTMSG_( !lst_edges.empty(), mrpt::format("ERROR: No topological path found between %u <-> %u", static_cast<unsigned int>(prev_idx),static_cast<unsigned int>(idx) ) )

						for (mrpt::graphs::CDijkstra<RWT_adjacency_graph>::edge_list_t::const_iterator it=lst_edges.begin();it!=lst_edges.end();++it)
						{
							const mrpt::utils::TPairNodeIDs pair_ids = *it;
							const mrpt::utils::TNodeID  next_id = pair_ids.first==prev_idx ? pair_ids.second : pair_ids.first;

							float x,y,z;
							the_world.nodes.getPoint(next_id,x,y,z);
							simulation_waypoints.push_back( mrpt::math::TPoint3D(x,y,z) );

							prev_idx = next_id;
						}
					}
				} // end for "j" over "lst_idxs"
			} // end for "i"
		}
		else
		{
			// ...
		}

		cout << "[3<] Path generated OK: " << simulation_waypoints.size() << " waypoints.\n"; cout.flush();

		// load the rest of options:
		// ------------------------------
		const string sOutFilesPrefix = cfg.read_string("dataset-format","output_files_prefix", "OUT_", true /* fail if not present */) ;

		RWT_PathOptions     pathParams;
		pathParams.max_step_lin = cfg.read_double("path","max_step_lin", 0.10);
		pathParams.max_step_ang = mrpt::utils::DEG2RAD( cfg.read_double("path","max_step_ang", 10) );

		RWT_SensorOptions   sensorParams(cfg);
		sensorParams.sOutFilesPrefix=sOutFilesPrefix;
		sensorParams.observations_as_c_structs = sensorParams.cfg_file.read_bool("dataset-format","observations_as_c_structs", false);
		

		RWT_OutputOptions   outputParams;
		outputParams.win3D = win3D;
		outputParams.show_live_3D = win3D.present();
		outputParams.show_live_3D_decimate = cfg.read_int("path","show_live_3D_decimate", outputParams.show_live_3D_decimate);
		outputParams.show_live_3D_sleep_ms = cfg.read_int("path","show_live_3D_sleep_ms", outputParams.show_live_3D_sleep_ms);


		if (outputParams.is_binary)
		{
			THROW_EXCEPTION("TO DO")
		}
		else
		{
			cout << "Saving map ground truth and preparing result files..."; cout.flush();

			const string sOutSensor = sOutFilesPrefix + string("_SENSOR.txt");
			outputParams.output_text_sensor.open(sOutSensor.c_str());
			ASSERTMSG_(outputParams.output_text_sensor.is_open(), mrpt::format("Couldn't open output file: '%s'",sOutSensor.c_str() ) )
			outputParams.output_text_sensor <<
				"% STEP     LANDMARK_ID      {...SENSOR SPECIFIC DATA...}  \n"
				"% -------------------------------------------------------------------\n";

			const string sOutGT = sOutFilesPrefix + string("_GT_PATH.txt");
			outputParams.output_text_groundtruth.open(sOutGT.c_str());
			ASSERTMSG_(outputParams.output_text_groundtruth.is_open(), mrpt::format("Couldn't open output file: '%s'",sOutGT.c_str() ) )
			outputParams.output_text_groundtruth <<
				"% Ground truth path of the SENSOR, and the ROBOT                     \n"
				"% STEP     X       Y        Z        QR        QX      QY      QZ     |    X       Y        Z        QR        QX      QY      QZ   \n"
				"% ----------------------------------------------------------------------------------------------------------------------------------------\n";

			const string sOutGTMap = sOutFilesPrefix + string("_GT_MAP.txt");
			std::ofstream fOutGTMap(sOutGTMap.c_str());
			ASSERTMSG_(fOutGTMap.is_open(), mrpt::format("Couldn't open output file: '%s'",sOutGTMap.c_str() ) )
			fOutGTMap <<
				"% Landmark ground truth global positions (first row is for LM index=0, next is 1 and so on)\n"
				"%    X             Y              Z        \n"
				"% -----------------------------------------\n";

			const string sOutWayPointsMatlab = sOutFilesPrefix + string("_DRAW_WAYPOINTS.m");
			std::ofstream fOutWayPointsMatlab(sOutWayPointsMatlab.c_str());
			ASSERTMSG_(fOutWayPointsMatlab.is_open(), mrpt::format("Couldn't open output file: '%s'",sOutWayPointsMatlab.c_str() ) )

			RWT_SaveMatlabOptions matlab_opts;
			matlab_opts.nodes_size = 4;
			rwt::save_rwt_as_matlab_script(the_world,fOutWayPointsMatlab, matlab_opts);


			const std::vector<float> & lm_xs = the_world.landmarks.getPointsBufferRef_x();
			const std::vector<float> & lm_ys = the_world.landmarks.getPointsBufferRef_y();
			const std::vector<float> & lm_zs = the_world.landmarks.getPointsBufferRef_z();

			const size_t N = lm_xs.size();
			for (unsigned int i=0;i<N;i++)
				fOutGTMap << mrpt::format("%14f %14f %14f\n",lm_xs[i],lm_ys[i],lm_zs[i]);

			cout << "Done.\n"; cout.flush();
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
		string basedir = mrpt::system::extractFileDirectory(script_base_filename);
		if (!basedir.empty() && (*basedir.rbegin()!='/' && *basedir.rbegin()!='\\') )
			basedir.push_back('/');

		return mrpt::system::filePathSeparatorsToNative(basedir+sFil);
	}
}

double graph_edge_weight(const RWT_adjacency_graph& graph, const mrpt::utils::TNodeID id_from, const mrpt::utils::TNodeID id_to, const RWT_adjacency_graph::edge_t &edge)
{
	using mrpt::utils::square;

	mrpt::math::TPoint3Df p1,p2;
	global_the_world->nodes.getPoint(id_from,p1.x,p1.y,p1.z);
	global_the_world->nodes.getPoint(id_to,  p2.x,p2.y,p2.z);

	return square(p1.x-p2.x)+square(p1.y-p2.y)+square(p1.z-p2.z);
}
