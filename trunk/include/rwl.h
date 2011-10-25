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

#include "rwl-utils.h"  // Useful MACROS, etc.

namespace rwl
{
	using std::vector;
	using std::string;

	/** @name RWL data types
	    @{ */

	// Recursive World Language (RWL) primitives:
	enum RWL_primitive_t
	{
		PRIM_INVALID = -1,

		PRIM_X_SET, PRIM_Y_SET, PRIM_Z_SET, PRIM_YAW_SET, PRIM_PITCH_SET, PRIM_ROLL_SET,
		PRIM_X_INC, PRIM_Y_INC, PRIM_Z_INC, PRIM_YAW_INC, PRIM_PITCH_INC, PRIM_ROLL_INC,
		PRIM_PUSH, PRIM_POP,
		PRIM_LANDMARK,
		PRIM_NODE,
		PRIM_CALL
	};

	struct RWL_command
	{
		RWL_primitive_t  primitive;
		vector<string>   args;
	};

	/** A sequence of primitives and their arguments */
	struct RWL_List
	{
		vector<RWL_command>  cmds;
	};

	typedef std::map<std::string,RWL_List,mrpt::utils::ci_less> TListSet; //!< A set of RWL_Lists, indexed by name (case insensitive)

	/** A complete RWL program, comprising any number of lists */
	struct RWL_Program
	{
		void clear() { lists.clear(); }

		TListSet  lists;
	};

	struct RWL_graph_edge
	{
	};
	typedef mrpt::graphs::CDirectedGraph<RWL_graph_edge> RWL_adjacency_graph;

	/** The generated world, with landmarks and way-point nodes */
	struct RWL_World
	{
		void clear()
		{
			landmarks.clear();
			nodes.clear();
			graph.edges.clear();
		}

		mrpt::slam::CSimplePointsMap  landmarks; //!< XYZ coordinates of all landmarks, accesible thru a KD-tree
		mrpt::slam::CSimplePointsMap  nodes;     //!< XYZ coordinates of all nodes, accesible thru a KD-tree
		RWL_adjacency_graph           graph;     //!< graph.edges contain the existing paths between \a nodes above
	};

	/** To be used with rwl::world_to_opengl */
	struct WRL_RenderOptions
	{
		/** Ctor */
		WRL_RenderOptions()
		{ }


	};

	/** @} */  // ----------- end of data types ---------------


	/** @name RWL API
	    @{ */

	/** Compile an input RWL file into a program.
	  *  \return false on any error, and dump info to std::cerr
	  */
	bool compile_rwl_program(const std::string &file, RWL_Program &out_program);

	/** Runs an RWL program and generates its corresponding World.
	  *  \return false on any error, and dump info to std::cerr
	  */
	bool run_rwl_program(const RWL_Program &program, RWL_World & out_world);

	/** Build an OpenGL representation of the world */
	void world_to_opengl(const RWL_World &world, mrpt::opengl::CSetOfObjects &out_gl, const WRL_RenderOptions &renderOpts = WRL_RenderOptions() );


	/** @} */  // ----------- end of API ---------------


} // end of namespace
