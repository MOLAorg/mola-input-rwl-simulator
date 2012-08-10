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

#include <mrpt/random.h>

using namespace rwt;
using namespace std;
using namespace mrpt::random;

struct RWT_run_context
{
	struct StackableContext
	{
		mrpt::math::TPose3D cursor; //!< SE(3) poses with rotations parameterized as yaw,pitch,roll
		size_t              current_node;
	};

	size_t                            current_node;  //!< As index of out_world.nodes
	mrpt::poses::CPose3D              cursor;    //!< A SE(3) pose: 3x3 matrix + 3 translation
	std::stack<StackableContext>      cntx_stack;
	const RWT_Program                &program;
	RWT_World                        &out_world;

	/** Ctor */
	RWT_run_context(const RWT_Program &_program, RWT_World &_world) :
		current_node (0),
		cursor    (),
		program   (_program),
		out_world (_world)
	{
		out_world.clear();
		out_world.nodes.insertPoint(0,0,0); // The first node is always at the origin!
	}

	void push_context()
	{
		StackableContext d;
		d.cursor = mrpt::math::TPose3D(this->cursor);
		d.current_node = this->current_node;
		cntx_stack.push( d );
	}

	void pop_context()
	{
		if (cntx_stack.empty())
			throw std::runtime_error("*ERROR* POP: Stack underflow!\n");

		const StackableContext &d = cntx_stack.top();
		this->cursor = mrpt::poses::CPose3D(d.cursor);
		this->current_node = d.current_node;

		cntx_stack.pop();
	}

	/** Evaluate 1st argument as a number */
	double eval1st(const RWT_command &cmd)
	{
		if (cmd.args.empty())
			throw std::runtime_error("*ERROR* Evaluating primitive: Argument expected but none provided.");
		return rwt::str2num(cmd.args[0]);
	}
};

bool recursive_run_rwt_program(
	const TListSet::const_iterator &it_lst,
	RWT_run_context & cntx
	);


bool run_rwt_cmd(
	const RWT_command &cmd,
	RWT_run_context & cntx )
{
	using mrpt::utils::DEG2RAD;

	switch (cmd.primitive)
	{
	case PRIM_X_SET:
		cntx.cursor.x( cntx.eval1st(cmd) );
		break;
	case PRIM_Y_SET:
		cntx.cursor.y( cntx.eval1st(cmd) );
		break;
	case PRIM_Z_SET:
		cntx.cursor.z( cntx.eval1st(cmd) );
		break;
	case PRIM_YAW_SET:
		{
			const double val = DEG2RAD( cntx.eval1st(cmd) );
			double y,p,r;
			cntx.cursor.getYawPitchRoll(y,p,r);
			cntx.cursor.setYawPitchRoll(val,p,r);
		}
		break;
	case PRIM_PITCH_SET:
		{
			const double val = DEG2RAD( cntx.eval1st(cmd) );
			double y,p,r;
			cntx.cursor.getYawPitchRoll(y,p,r);
			cntx.cursor.setYawPitchRoll(y,val,r);
		}
		break;
	case PRIM_ROLL_SET:
		{
			const double val = DEG2RAD( cntx.eval1st(cmd) );
			double y,p,r;
			cntx.cursor.getYawPitchRoll(y,p,r);
			cntx.cursor.setYawPitchRoll(y,p,val);
		}
		break;

	case PRIM_X_INC:
		cntx.cursor+=mrpt::poses::CPose3D( cntx.eval1st(cmd), 0 ,0, 0,0,0 );
		break;
	case PRIM_Y_INC:
		cntx.cursor+=mrpt::poses::CPose3D( 0, cntx.eval1st(cmd), 0, 0,0,0 );
		break;
	case PRIM_Z_INC:
		cntx.cursor+=mrpt::poses::CPose3D( 0,0, cntx.eval1st(cmd), 0,0,0 );
		break;
	case PRIM_YAW_INC:
		cntx.cursor+=mrpt::poses::CPose3D( 0,0, 0, DEG2RAD(cntx.eval1st(cmd)), 0,0 );
		break;
	case PRIM_PITCH_INC:
		cntx.cursor+=mrpt::poses::CPose3D( 0,0, 0, 0, DEG2RAD(cntx.eval1st(cmd)), 0);
		break;
	case PRIM_ROLL_INC:
		cntx.cursor+=mrpt::poses::CPose3D( 0,0, 0, 0, 0, DEG2RAD(cntx.eval1st(cmd)) );
		break;

	case PRIM_X_DEC:
		cntx.cursor+=mrpt::poses::CPose3D( -cntx.eval1st(cmd), 0 ,0, 0,0,0 );
		break;
	case PRIM_Y_DEC:
		cntx.cursor+=mrpt::poses::CPose3D( 0, -cntx.eval1st(cmd), 0, 0,0,0 );
		break;
	case PRIM_Z_DEC:
		cntx.cursor+=mrpt::poses::CPose3D( 0,0, -cntx.eval1st(cmd), 0,0,0 );
		break;
	case PRIM_YAW_DEC:
		cntx.cursor+=mrpt::poses::CPose3D( 0,0, 0, -DEG2RAD(cntx.eval1st(cmd)), 0,0 );
		break;
	case PRIM_PITCH_DEC:
		cntx.cursor+=mrpt::poses::CPose3D( 0,0, 0, 0, -DEG2RAD(cntx.eval1st(cmd)), 0);
		break;
	case PRIM_ROLL_DEC:
		cntx.cursor+=mrpt::poses::CPose3D( 0,0, 0, 0, 0, -DEG2RAD(cntx.eval1st(cmd)) );
		break;

	case PRIM_PUSH:
		cntx.push_context();
		break;
	case PRIM_POP:
		cntx.pop_context();
		break;
	case PRIM_LANDMARK:
		{
			if (cmd.args.size()!=3)
				throw std::runtime_error("*ERROR* LM: Three arguments expected!\n");

			// Get relative coords:
			const double lx = rwt::str2num(cmd.args[0]);
			const double ly = rwt::str2num(cmd.args[1]);
			const double lz = rwt::str2num(cmd.args[2]);

			// Convert to global:
			double gx,gy,gz;
			cntx.cursor.composePoint(lx,ly,lz, gx,gy,gz);
			cntx.out_world.landmarks.insertPointFast(gx,gy,gz); // "Fast" means the KD-tree will not be marked as invalid, but we don't mind that here.

			//RWT_MESSAGE << "LM: " << gx << ", " << gy << ", " << gz << endl;
		}
		break;
	case PRIM_LANDMARK_RANDOM:
		{
			if (cmd.args.size()!=6)
				throw std::runtime_error("*ERROR* LM_RANDOM: 6 arguments expected!\n");

			// Get relative coords:
			const double min_x = rwt::str2num(cmd.args[0]);
			const double max_x = rwt::str2num(cmd.args[1]);
			const double min_y = rwt::str2num(cmd.args[2]);
			const double max_y = rwt::str2num(cmd.args[3]);
			const double min_z = rwt::str2num(cmd.args[4]);
			const double max_z = rwt::str2num(cmd.args[5]);

			// Draw sample:
			const double lx = randomGenerator.drawUniform(min_x,max_x);
			const double ly = randomGenerator.drawUniform(min_y,max_y);
			const double lz = randomGenerator.drawUniform(min_z,max_z);

			// Convert to global:
			double gx,gy,gz;
			cntx.cursor.composePoint(lx,ly,lz, gx,gy,gz);
			cntx.out_world.landmarks.insertPointFast(gx,gy,gz); // "Fast" means the KD-tree will not be marked as invalid, but we don't mind that here.
		}
		break;
	case PRIM_NODE:
		{
			if (cmd.args.size()!=0 && cmd.args.size()!=3 )
				throw std::runtime_error("*ERROR* NODE: None or three arguments expected!\n");

			// Get relative coords:
			double lx=0,ly=0,lz=0;
			if (cmd.args.size()==3)
			{
				lx = rwt::str2num(cmd.args[0]);
				ly = rwt::str2num(cmd.args[1]);
				lz = rwt::str2num(cmd.args[2]);
			}
			// Convert to global:
			double gx,gy,gz;
			cntx.cursor.composePoint(lx,ly,lz, gx,gy,gz);

			// Don't insert if there's already another one very close
			// which may actually be the same except for round-off errors:
			bool skip = false;
			size_t revisit_index = string::npos;
			if (!cntx.out_world.nodes.empty())
			{
				float out_sqr_err;
				revisit_index = cntx.out_world.nodes.kdTreeClosestPoint3D(gx,gy,gz, out_sqr_err);
				skip = (out_sqr_err<mrpt::utils::square(1e-1));
			}

			// Save so we can create an arc:
			const size_t prev_node = cntx.current_node;

			if (!skip)
			{
				cntx.out_world.nodes.insertPoint(gx,gy,gz);
				//RWT_MESSAGE << "NODE: " << gx << ", " << gy << ", " << gz << " from " << cntx.cursor  << endl;
				cntx.current_node = cntx.out_world.nodes.size()-1;
			}
			else
			{
				// We have revisited an old node:
				// RWT_MESSAGE << "NODE *SKIPPING*: " << gx << ", " << gy << ", " << gz << " from " << cntx.cursor  << endl;
				cntx.current_node = revisit_index;
			}

			// Create arc:
			RWT_graph_edge edge_val; /* dummy edge value */
			cntx.out_world.graph.insertEdge( prev_node,cntx.current_node, edge_val );
		}
		break;
	case PRIM_CALL:
		{
			// Recursively call other lists:
			if (cmd.args.size()!=1 && cmd.args.size()!=2 )
				throw std::runtime_error("*ERROR* CALL: Invalid number of arguments!\n");

			const string &callListName = cmd.args.back();
			size_t num_calls = 1;

			if (cmd.args.size()==2)
			{
				const string &sNumReps = mrpt::system::trim(cmd.args[0]);
				if (sNumReps.empty() || sNumReps[0]!='*')
					throw std::runtime_error("*ERROR* CALL: Expected '*N' for number of repetitions!\n");

				num_calls = rwt::str2num(sNumReps.substr(1));
			}
			//RWT_MESSAGE << "CALL: " << num_calls << " times LIST: '" << callListName << "'\n";

			const TListSet::const_iterator it_lst = cntx.program.lists.find(callListName);

			if (it_lst==cntx.program.lists.end())
				throw std::runtime_error(mrpt::format("*ERROR* CALL: Unknown list name: '%s'\n",callListName.c_str()));

			for (size_t i=0;i<num_calls;i++)
				recursive_run_rwt_program(it_lst,cntx);
		}
		break;
	case PRIM_RANDOMIZE:
		{
			if (cmd.args.size()!=0 && cmd.args.size()!=1 )
				throw std::runtime_error("*ERROR* RANDOMIZE: None or one arguments expected!\n");

			if (cmd.args.size()==1)
			{
				const double fSeed = rwt::str2num(cmd.args[0]);
				randomGenerator.randomize(static_cast<uint32_t>(fSeed));					
			}
			else
			{
				randomGenerator.randomize();					
			}
		}
		break;
	default:
		cerr << "Unknown primitive (!!)\n";
		return false;
	};
	return true;
}

bool recursive_run_rwt_program(
	const TListSet::const_iterator &it_lst,
	RWT_run_context & cntx
	)
{
	const RWT_List & lst = it_lst->second;
	//RWT_MESSAGE << "Running list: " << it_lst->first << endl;

	for (size_t i=0;i<lst.cmds.size();i++)
		run_rwt_cmd(lst.cmds[i],cntx);

	return true;
} // end of recursive_run_rwt_program

/** Runs an RWT program and generates its corresponding World.
  *  \return false on any error, and dump info to std::cerr
  */
bool rwt::run_rwt_program(const RWT_Program &program, RWT_World & out_world)
{
	RWT_run_context  cntx(program,out_world);

	// Start at the "main" list and run recursively:
	TListSet::const_iterator itMain = program.lists.find("main");

	if (itMain == program.lists.end())
	{
		RWT_MESSAGE << "*ERROR* Program has no 'main' list!\n";
		return false;
	}

	return recursive_run_rwt_program(itMain,cntx);
} // end of run_rwt_program
