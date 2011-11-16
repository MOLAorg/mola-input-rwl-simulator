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
#include <mrpt/math/lightweight_geom_data.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/slam/CSimplePointsMap.h>
#include <mrpt/utils/CFileGZOutputStream.h>
#include <mrpt/graphs.h>
#include <mrpt/gui/CDisplayWindow3D.h>

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

		mrpt::slam::CSimplePointsMap  landmarks; //!< XYZ coordinates of all landmarks, accesible thru a KD-tree
		mrpt::slam::CSimplePointsMap  nodes;     //!< XYZ coordinates of all nodes, accesible thru a KD-tree
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
			: cfg_file(_cfg_file)
		{ 
		}

		mrpt::utils::CConfigFileBase &cfg_file; //!< Source of extra parameters for loading upon construction
		std::string  sOutFilesPrefix; //!< For dumping extra sets of parameters used in simulation, for future reference.

	}; // end of RWT_SensorOptions

	struct RWT_OutputOptions
	{
		RWT_OutputOptions() :
			win3D(),
			show_live_3D(false),
			show_live_3D_sleep_ms(10),
			is_binary   (false)
		{
		}

		mrpt::gui::CDisplayWindow3DPtr win3D; //!< If not an empty smart pointer, the window to show simulation live.

		bool  show_live_3D; //!< true: Update the robot path, etc. in the 3D view
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

	/** Compile an input RWT file into a program.
	  *  \return false on any error, and dump info to std::cerr
	  */
	bool compile_rwt_program(const std::string &file, RWT_Program &out_program);

	/** Runs an RWT program and generates its corresponding World.
	  *  \return false on any error, and dump info to std::cerr
	  */
	bool run_rwt_program(const RWT_Program &program, RWT_World & out_world);

	/** Build an OpenGL representation of the world */
	void world_to_opengl(const RWT_World &world, mrpt::opengl::CSetOfObjects &out_gl, const WRL_RenderOptions &renderOpts = WRL_RenderOptions() );

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


	/** @} */  // ----------- end of API ---------------


} // end of namespace
