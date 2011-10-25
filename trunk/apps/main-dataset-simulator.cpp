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

//#include <mrpt/utils/CTextFileLinesParser.h>
//#include <mrpt/system/string_utils.h>
//#include <mrpt/slam/CSimplePointsMap.h>
//#include <mrpt/poses/CPose3D.h>
//#include <mrpt/math/lightweight_geom_data.h>
//#include <mrpt/graphs.h>

#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/opengl.h>

#include "rwt.h"

using namespace rwt;
using namespace std;

// ---------------
//      Main
// ---------------
int main(int argc, char**argv)
{
	try
	{
		if (argc!=2)
		{
			cout << "Usage:\n" << argv[0] << " input_file.rwt\n";
			return 1;
		}

		const string sFil = string(argv[1]);

		// Compile ----------------------------
		cout << "Compiling...\n"; cout.flush();
		RWT_Program program;
		if (!compile_rwt_program(sFil,program))
		{
			cerr << "*ERROR* Program compilation failed\n";
			return 1;
		}
		cout << "Compilation succeeded!\n";

		// Run ----------------------------
		RWT_World the_world;

		cout << "Building world...\n"; cout.flush();
		if (!run_rwt_program(program,the_world))
		{
			cerr << "*ERROR* Program compilation failed\n";
			return 1;
		}
		cout << "World contruction succeeded!\n";
		// Stats:
		if (1)
		{
			cout << "# landmarks    : " << the_world.landmarks.size() << endl;
			cout << "# way-points   : " << the_world.nodes.size() << endl;
			cout << "# path segments: " << the_world.graph.edges.size() << endl;
		}

		// Display ----------------------------
		mrpt::gui::CDisplayWindow3D win3D("RWC compiled world",640,480);
		win3D.getDefaultViewport()->setCustomBackgroundColor( mrpt::utils::TColorf(0.2f,0.2f,0.2f) );

		mrpt::opengl::CPointCloudPtr gl_LMs =mrpt::opengl::CPointCloud::Create();
		gl_LMs->loadFromPointsMap( &the_world.landmarks);
		gl_LMs->setPointSize(2.0);
		gl_LMs->setColor_u8(220,220,220);

		mrpt::opengl::CPointCloudPtr gl_Nodes =mrpt::opengl::CPointCloud::Create();
		gl_Nodes->loadFromPointsMap( &the_world.nodes );
		gl_Nodes->setPointSize(5.0);
		gl_Nodes->setColor_u8(255,0,0);

		mrpt::opengl::CSetOfLinesPtr gl_edges =mrpt::opengl::CSetOfLines::Create();
		gl_edges->setLineWidth(1);
		gl_edges->setColor_u8( mrpt::utils::TColor(0,0,220));

		for (RWT_adjacency_graph::const_iterator it=the_world.graph.begin();it!=the_world.graph.end();++it)
		{
			const size_t idx1 = it->first.first;
			float x1,y1,z1;
			the_world.nodes.getPoint(idx1, x1,y1,z1);

			const size_t idx2 = it->first.second;
			float x2,y2,z2;
			the_world.nodes.getPoint(idx2, x2,y2,z2);

			gl_edges->appendLine(x1,y1,z1, x2,y2,z2);
		}

		mrpt::opengl::COpenGLScenePtr &scene = win3D.get3DSceneAndLock();

		scene->insert( gl_LMs );
		scene->insert( gl_Nodes );
		scene->insert( gl_edges );

		win3D.unlockAccess3DScene();
		win3D.repaint();
		win3D.waitForKey();


		return 0;
	} catch (exception &e) {
		cerr << "Exception: " << e.what() << endl;
		return -1;
	}
}


