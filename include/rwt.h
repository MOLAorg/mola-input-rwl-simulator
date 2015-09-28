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

#pragma once

#include <cstdlib>
#include <cmath>
#include <vector>
#include <deque>
#include <map>
#include <iostream>
#include <sstream>
#include <string>
#include <stdexcept>
#include <limits>
#include <stack>

#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DQuat.h> // To fix missing hdr in older mrpt network of poses 
#include <mrpt/math/lightweight_geom_data.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/utils/CFileGZOutputStream.h>
#include <mrpt/graphs.h>
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/utils/ci_less.h>

#include "rwt-utils.h"  // Useful MACROS, etc.

namespace rwt
{
	using std::vector;
	using std::string;

	/** @name RWT data types
	    @{ */

	// Recursive World Language (RWT) primitives:
	enum RWT_primitive_t
	{
		PRIM_INVALID = -1,

		PRIM_X_SET, PRIM_Y_SET, PRIM_Z_SET, PRIM_YAW_SET, PRIM_PITCH_SET, PRIM_ROLL_SET,
		PRIM_X_INC, PRIM_Y_INC, PRIM_Z_INC, PRIM_YAW_INC, PRIM_PITCH_INC, PRIM_ROLL_INC,
		PRIM_X_DEC, PRIM_Y_DEC, PRIM_Z_DEC, PRIM_YAW_DEC, PRIM_PITCH_DEC, PRIM_ROLL_DEC,
		PRIM_PUSH, PRIM_POP,
		PRIM_LANDMARK, PRIM_LANDMARK_RANDOM,
		PRIM_NODE,
		PRIM_CALL,
		PRIM_RANDOMIZE
	};

	struct RWT_command
	{
		RWT_primitive_t  primitive;
		vector<string>   args;
	};

	/** A sequence of primitives and their arguments */
	struct RWT_List
	{
		vector<RWT_command>  cmds;
	};

	typedef std::map<std::string,RWT_List,mrpt::utils::ci_less> TListSet; //!< A set of RWT_Lists, indexed by name (case insensitive)

	/** A complete RWT program, comprising any number of lists */
	struct RWT_Program
	{
		void clear() { lists.clear(); }

		TListSet  lists;
	};

	struct RWT_graph_edge
	{
	};
	typedef mrpt::graphs::CDirectedGraph<RWT_graph_edge> RWT_adjacency_graph;

	/** The generated world, with landmarks and way-point nodes */
	struct RWT_World
	{
		void clear()
		{
			landmarks.clear();
			nodes.clear();
			graph.edges.clear();
		}

		mrpt::maps::CSimplePointsMap  landmarks; //!< XYZ coordinates of all landmarks, accesible thru a KD-tree
		mrpt::maps::CSimplePointsMap  nodes;     //!< XYZ coordinates of all nodes, accesible thru a KD-tree
		RWT_adjacency_graph           graph;     //!< graph.edges contain the existing paths between \a nodes above
	};

	/** To be used with rwt::world_to_opengl */
	struct WRL_RenderOptions
	{
		/** Aux struct with the parameters of each XYZ corner */
		struct CornerParams
		{
			CornerParams() : scale(1), line_width(1) { }
			CornerParams(double _scale,double _line_width=1.0) : scale(_scale), line_width(_line_width) { }
			double scale;
			double line_width;
		};

		/** Ctor */
		WRL_RenderOptions() :
			show_world_origin_corner   (true),
			world_origin_corner_params (1.,3.)
		{ }

		bool          show_world_origin_corner;
		CornerParams  world_origin_corner_params;
	}; // end of WRL_RenderOptions


	/** Used in rwt::save_rwt_as_matlab_script **/
	struct RWT_SaveMatlabOptions
	{
		RWT_SaveMatlabOptions() :
			nodes_size(7.0),
			edges_width(1.0)
		{}

		double nodes_size;  //!< Set to 0 to disable drawing waypoint nodes.
		double edges_width; //!< Set to 0 to disable edges.
	}; // end of RWT_SaveMatlabOptions


	struct RWT_PathOptions
	{
		RWT_PathOptions() :
			max_step_lin(0.1),
			max_step_ang(mrpt::utils::DEG2RAD(10))
		{ }

		double max_step_lin; //!< meters
		double max_step_ang; //!< radians
	}; // end of RWT_PathOptions

	struct RWT_SensorOptions
	{
		RWT_SensorOptions(mrpt::utils::CConfigFileBase &_cfg_file)
			: cfg_file(_cfg_file), observations_as_c_structs(false)
		{
		}

		mrpt::utils::CConfigFileBase &cfg_file; //!< Source of extra parameters for loading upon construction
		std::string  sOutFilesPrefix; //!< For dumping extra sets of parameters used in simulation, for future reference.
		bool observations_as_c_structs;

	}; // end of RWT_SensorOptions

	struct RWT_OutputOptions
	{
		RWT_OutputOptions() :
			win3D(),
			show_live_3D(false),
			show_live_3D_decimate(1),
			show_live_3D_sleep_ms(10),
			is_binary   (false)
		{
		}

		mrpt::gui::CDisplayWindow3DPtr win3D; //!< If not an empty smart pointer, the window to show simulation live.

		bool  show_live_3D; //!< true: Update the robot path, etc. in the 3D view
		unsigned int show_live_3D_decimate;
		int   show_live_3D_sleep_ms;

		bool  is_binary; // true: use outfile_bin_rawlog; false: use outfile_text_*
		// The following file streams must be opened by the caller to \a simulate_rwt_dataset

		// Sensor: Binary output:
		mrpt::utils::CFileGZOutputStream  output_bin_rawlog;

		// Sensor: Text output:
		std::ofstream   output_text_sensor;
		std::ofstream   output_text_odometry;

		// Path ground truth:
		std::ofstream   output_text_groundtruth;

	}; // end of RWT_OutputOptions

	/** @} */  // ----------- end of data types ---------------


	/** @name RWT API
	    @{ */

	/** Compile an input RWT file into a program and run it to generate the synthetic world.
	  *  This function is equivalent to successive calls to \a compile_rwt_program() and 
	  *  \a run_rwt_program(), plus the possibility of caching the generated world in cache files
	  *  to avoid running exactly the same program more than once.
	  *
	  *  \return false on any error, and dump info to std::cerr
	  */
	bool compile_and_run_rwt_program(const std::string &file, RWT_Program &out_program, RWT_World & out_world, bool enable_file_caching = true);

	/** Compile an input RWT file into a program.
	  *  \return false on any error, and dump info to std::cerr
	  */
	bool compile_rwt_program(const std::string &file, RWT_Program &out_program);

	/** Runs an RWT program and generates its corresponding World.
	  *  \return false on any error, and dump info to std::cerr
	  */
	bool run_rwt_program(const RWT_Program &program, RWT_World & out_world);

	/** Build an OpenGL representation of the world.
	  * These opengl objects are created with names for allowing the user to manipulate them:
	  *   - "node_labels": The text labels of all node IDs. It's initially invisible, get it and call setVisibility(true) to show.
	  *   - "landmarks": All landmarks as a point cloud.
	  */
	void world_to_opengl(const RWT_World &world, mrpt::opengl::CSetOfObjects &out_gl, const WRL_RenderOptions &renderOpts = WRL_RenderOptions() );

	/** Load a world from a compiled .crwt.gz file \sa compile_and_run_rwt_program, save_rwt_world 
	  * If a source file (.rwt) is provided in \a source_file, its HASH will be compared with that within the 
	  * compiled file to assure that the program hasn't changed since compilation, returning false otherwise.
	  * \return false on any error  */
	bool load_rwt_world(RWT_World &world, const std::string &crwt_file, const std::string &source_file = std::string());

	/** Save a world to a compiled .crwt.gz file \sa compile_and_run_rwt_program, load_rwt_world 
	  * If \a source_file is provided, a HASH will be saved (see \a load_rwt_world() for its utility).
	  * \return false on any error  */
	bool save_rwt_world(const RWT_World &world, const std::string &file, const std::string &source_file = std::string());

	/** Simulate an arbitrarily complex robot path (given an input sequence of waypoints),
	  * a world model and a set of sensor and output parameters.
	  */
	void simulate_rwt_dataset(
		const std::vector<mrpt::math::TPoint3D>    waypoints,
		const RWT_World                          & world,
		const RWT_PathOptions                    & pathParams,
		const RWT_SensorOptions                  & sensorParams,
		RWT_OutputOptions                        & outputParams
		);

	/** Save a MATLAB script with a representation of nodes,edges, etc. **/
	void save_rwt_as_matlab_script(
		const RWT_World             & world,
		std::ostream                & o,
		const RWT_SaveMatlabOptions & matlabDrawParams
		);


	/** @} */  // ----------- end of API ---------------


} // end of namespace
