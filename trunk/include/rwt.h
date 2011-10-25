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

#include <mrpt/slam/CSimplePointsMap.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/math/lightweight_geom_data.h>
#include <mrpt/graphs.h>
#include <mrpt/opengl/CSetOfObjects.h>

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
		PRIM_LANDMARK,
		PRIM_NODE,
		PRIM_CALL
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

	};

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


	/** @} */  // ----------- end of API ---------------


} // end of namespace
