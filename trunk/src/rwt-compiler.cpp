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

#include <mrpt/utils/CTextFileLinesParser.h>
#include <mrpt/system/string_utils.h>

using namespace rwt;
using namespace std;

struct TStrings2Primitive
{
	const char *str;
	const RWT_primitive_t prim;
};

const size_t primitive_strings_count = 25;
const TStrings2Primitive primitive_strings[primitive_strings_count]  = {
	{ "X=", PRIM_X_SET },
	{ "Y=", PRIM_Y_SET },
	{ "Z=", PRIM_Z_SET },
	{ "YAW=", PRIM_YAW_SET },
	{ "PITCH=", PRIM_PITCH_SET },
	{ "ROLL=", PRIM_ROLL_SET },

	{ "X+=", PRIM_X_INC },
	{ "Y+=", PRIM_Y_INC },
	{ "Z+=", PRIM_Z_INC },
	{ "YAW+=", PRIM_YAW_INC },
	{ "PITCH+=", PRIM_PITCH_INC },
	{ "ROLL+=", PRIM_ROLL_INC },

	{ "X+=", PRIM_X_DEC },
	{ "Y+=", PRIM_Y_DEC },
	{ "Z+=", PRIM_Z_DEC },
	{ "YAW+=", PRIM_YAW_DEC },
	{ "PITCH+=", PRIM_PITCH_DEC },
	{ "ROLL+=", PRIM_ROLL_DEC },

	{ "PUSH", PRIM_PUSH },
	{ "POP", PRIM_POP },

	{ "LM", PRIM_LANDMARK },
	{ "LM_RANDOM", PRIM_LANDMARK_RANDOM },

	{ "NODE", PRIM_NODE },
	{ "CALL", PRIM_CALL },

	{ "RANDOMIZE", PRIM_RANDOMIZE},
};

/** Compile an input RWT file into a program.
  *  \return false on any error, and dump info to std::cerr
  */
bool rwt::compile_rwt_program(const std::string &file, RWT_Program &out_program)
{
	using mrpt::system::strCmpI;
	using mrpt::system::tokenize;
	using mrpt::system::trim;

	out_program.clear();

	try
	{
		mrpt::utils::CTextFileLinesParser  flp(file);

		flp.enableCommentFilters(
			false, // filter_MATLAB_comments
			true,  // filter_C_comments
			false  // filter_SH_comments
		);

		std::istringstream ss;

		// The parser state machine is really simple: It can be either within or without
		//  a "LIST" block at any time:
		enum TState
		{
			stIdle,
			stList
		};

		TState    parser_state = stIdle;
		RWT_List  parser_cur_list; // The list being compile right now.
		string    parser_cur_list_name;

		while (flp.getNextLine(ss))
		{
			string line = ss.str();
			// Remove in-line comments:
			{
				const size_t p=line.find("//");
				if (p!=string::npos)
					line.resize(p-1);
			}
			line = trim(line);

			vector<string> toks;
			tokenize(line," ,\t",toks);
			if (toks.empty()) continue;

			if (strCmpI("LIST",toks[0]))
			{
				// "LIST": Start command
				if (parser_state!=stIdle)
					throw std::runtime_error(mrpt::format("%u: LIST within another LIST is not allowed\n",static_cast<unsigned int>(flp.getCurrentLineNumber())) );

				// we need exactly one argument: the list name:
				if (toks.size()!=2)
					throw std::runtime_error(mrpt::format("%u: LIST needs one argument\n",static_cast<unsigned int>(flp.getCurrentLineNumber())) );

				const string sListName = trim(toks[1]);
				if (out_program.lists.count(sListName)!=0)
					throw std::runtime_error(mrpt::format("%u: LIST name was already used\n",static_cast<unsigned int>(flp.getCurrentLineNumber())) );

				parser_state=stList; // we are now within a list.
				parser_cur_list = RWT_List();
				parser_cur_list_name = sListName;

			}
			else if (strCmpI("ENDLIST",toks[0]))
			{
				if (parser_state!=stList)
					throw std::runtime_error(mrpt::format("%u: ENDLIST out of LIST block is not allowed\n",static_cast<unsigned int>(flp.getCurrentLineNumber())) );

				//RWT_MESSAGE << "Created list: " << parser_cur_list_name << endl;
				out_program.lists[parser_cur_list_name] = parser_cur_list;

				parser_state=stIdle; // we are now outside a list.
				parser_cur_list_name="";
			}
			else if (parser_state==stList)
			{
				// Parse the inner contents of a list:
				const string sFirst = toks[0];
				vector<string> args = toks;

				args.erase(args.begin());  // Remove the first one, so the rest are arguments.

				RWT_primitive_t primitive_id = PRIM_INVALID;

				for (size_t i=0;i<primitive_strings_count;i++)
				{
					const TStrings2Primitive &p = primitive_strings[i];
					if (strCmpI(p.str,sFirst))
					{
						primitive_id = p.prim;
						break;
					}
				}

				if (primitive_id == PRIM_INVALID)
					throw std::runtime_error(mrpt::format("%u: No known primitive found in this line:\n Line: '%s'\n",static_cast<unsigned int>(flp.getCurrentLineNumber()),line.c_str() ));

				// append (quick):
				parser_cur_list.cmds.resize( parser_cur_list.cmds.size()+1 );
				RWT_command &new_cmd = parser_cur_list.cmds.back();
				new_cmd.primitive = primitive_id;
				new_cmd.args = args;
			}
			else
			{
				// Parse stuff out of a list:

			}

		}; // end while

		return true;
	}
	catch (exception &e)
	{
		if (strlen(e.what()))
			cerr << e.what() << endl;
		return false;
	}
}

