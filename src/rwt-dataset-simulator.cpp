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

#include "rwt.h"
#include "rwt-sensor-simulators.h" // Declaration of abstract (and particular) sensor classes.

#include <mrpt/system/threads.h>  // for sleep()
#include <mrpt/system/datetime.h>
#include <mrpt/poses/CPose3DInterpolator.h>
#include <mrpt/math/geometry.h>
#include <mrpt/opengl/stock_objects.h>

using namespace rwt;
using namespace std;


// This is the main body of the simulator:
void rwt::simulate_rwt_dataset(
	const std::vector<mrpt::math::TPoint3D>    waypoints,
	const RWT_World                          & world,
	const RWT_PathOptions                    & pathParams,
	const RWT_SensorOptions                  & sensorParams,
	RWT_OutputOptions                        & outputParams
	)
{
	using namespace mrpt::utils; // Needed by the CStream "<<" operator.

	const size_t nWayPoints = waypoints.size();
	const bool   isBinary   = outputParams.is_binary;  // (output file format)

	SimulContext sim;
	// Set extra params:
	sim.show_live_3D = outputParams.show_live_3D;
	sim.win3D        = outputParams.win3D;


	// Create a SE(3) pose interpolator passing thru all the waypoints:
	// --------------------------------------------------------------------
	mrpt::poses::CPose3DInterpolator  simul_path;

	simul_path.setInterpolationMethod( mrpt::poses::CPose3DInterpolator::imSplineSlerp );
	simul_path.setMaxTimeInterpolation( 2.0 );

	const mrpt::system::TTimeStamp t0 =  mrpt::system::now();

	const mrpt::system::TTimeStamp At_between_waypoints =  mrpt::system::secondsToTimestamp(1.0); // 1 second

	double total_path_len = 0;

	{
		mrpt::math::CMatrixDouble33 ROT;  // 3x3 rot matrix
		mrpt::system::TTimeStamp t = t0;
		for (size_t i=0;i<nWayPoints;i++)
		{
			const bool is_last_pt = (i==(nWayPoints-1));

			const mrpt::math::TPoint3D &cur_pt  = waypoints[i];

			if (!is_last_pt)
			{
				// Create a SE(3) pose placed at cur_pt and heading towards next_pt with "upwards" looking at +Z:
				const mrpt::math::TPoint3D &next_pt = waypoints[i+1];

				// +X is the forward direction:
				mrpt::math::TPoint3D vec_x = next_pt-cur_pt;
				const double vec_x_norm = vec_x.norm();
				total_path_len+=vec_x_norm;
				if (vec_x_norm) vec_x*= 1.0/vec_x_norm;

				mrpt::math::TPoint3D vec_y(-vec_x.y,vec_x.x,0); 
				const double vec_y_norm = vec_y.norm();
				ASSERT_(vec_y_norm>1e-6) // Otherwise -> TODO!
				vec_y*= 1.0/vec_y_norm;

				mrpt::math::TPoint3D vec_z;// =  vec_x (x) vec_y
				mrpt::math::crossProduct3D(vec_x,vec_y, vec_z);

				const double vec_x_norm2 = vec_x.norm();
				const double vec_y_norm2 = vec_y.norm();
				const double vec_z_norm2 = vec_z.norm();
				ASSERT_BELOW_(std::abs(vec_x_norm2-1),1e-6)
				ASSERT_BELOW_(std::abs(vec_y_norm2-1),1e-6)
				ASSERT_BELOW_(std::abs(vec_z_norm2-1),1e-6)

				ROT(0,0) = vec_x.x; ROT(0,1) = vec_y.x;  ROT(0,2) = vec_z.x;
				ROT(1,0) = vec_x.y; ROT(1,1) = vec_y.y;  ROT(1,2) = vec_z.y;
				ROT(2,0) = vec_x.z; ROT(2,1) = vec_y.z;  ROT(2,2) = vec_z.z;
			}
			else
			{
				// Last waypoint: Reuse last rotation matrix
			}

			const mrpt::poses::CPose3D p(ROT,cur_pt);

			// Trick: For spline interpolator to work we need to add an extra time steps before the beginning
			// and after the end:
			if (i==0) simul_path.insert(t-At_between_waypoints, p);

			simul_path.insert(t, p);

			if (is_last_pt) simul_path.insert(t+At_between_waypoints, p);

			//cout << t << " -> " << p << endl;
			t+=At_between_waypoints;
		}
	}


	// Create sensor:
	// --------------------------------------
	const string sType = sensorParams.cfg_file.read_string("sensor","type","",true);
	SensorSimulBasePtr sensor;

	if (mrpt::system::strCmpI(sType,"camera"))
		sensor = SensorSimulBasePtr(new SensorSimul_Camera(world,sensorParams) );
	else if (mrpt::system::strCmpI(sType,"camera_range"))
		sensor = SensorSimulBasePtr(new SensorSimul_CameraRange(world,sensorParams) );
	else if (mrpt::system::strCmpI(sType,"stereo_camera"))
		sensor = SensorSimulBasePtr(new SensorSimul_CameraStereo(world,sensorParams) );
	else if (mrpt::system::strCmpI(sType,"cartesian_sensor"))
		sensor = SensorSimulBasePtr(new SensorSimul_Cartesian(world,sensorParams) );
	else if (mrpt::system::strCmpI(sType,"range_bearing"))
		sensor = SensorSimulBasePtr(new SensorSimul_RangeBearing(world,sensorParams) );
	else if (mrpt::system::strCmpI(sType,"relative_poses"))
		sensor = SensorSimulBasePtr(new SensorSimul_RelativePoses(world,sensorParams) );	
	else
		throw std::runtime_error( mrpt::format("ERROR: Unknown sensor type: %s",sType.c_str() ).c_str() );

	// Write human-readable textual description of data output:
	sensor->describeObservation(outputParams);

	mrpt::opengl::CRenderizablePtr gl_robot; // Cache this pointer to avoid looking for it with each iteration.

	// Approx # steps from user params:
	const size_t nDesiredTimeSteps = 1 + total_path_len/pathParams.max_step_lin;
	const mrpt::system::TTimeStamp t_last = simul_path.rbegin()->first;
	const double At_total_tim = mrpt::system::timeDifference(t0,t_last);
	const mrpt::system::TTimeStamp At_step = (t_last-t0)/nDesiredTimeSteps;

	cout << endl; // start new line for simulation state

	unsigned int nStepForDecimateGUI = 0;

	// Simulation takes places in the domain of "time":
	for (mrpt::system::TTimeStamp t = t0; t<(t_last+1e-4) ; t+=At_step)
	{
		// Simulate sensor readings at current location:
		// -----------------------------------------------
		mrpt::obs::CObservationPtr  new_obs_bin;
		std::string                  new_obs_txt;

		bool valid_interp;
		simul_path.interpolate(t,sim.curPose,valid_interp);

		if (!valid_interp)  {
			std::cerr << "Invalid interpolation for t=" << t << "\r";  // repeat error in the same line
			sim.warning_no_interpolation_count++;
			continue;
		}


		if (sensorParams.observations_as_c_structs)
			outputParams.output_text_sensor << "my_observation_struct_t  observations_" << sim.step_count << "[] = {" << endl;

		mrpt::poses::CPose3DQuat sensor_GT_Pose;
		const mrpt::poses::CPose3DQuat robot_GT_Pose = mrpt::poses::CPose3DQuat(sim.curPose);
		// -------------------------------------------
		// Here happens the real sensor simulation:
		sensor->simulate(sim, isBinary, new_obs_bin, new_obs_txt, sensor_GT_Pose);
		// -------------------------------------------

		if (sensorParams.observations_as_c_structs)
			new_obs_txt += string(" };\n");


		// Update 3D view?
		// ----------------------------
		if (outputParams.show_live_3D && outputParams.win3D->isOpen())
		{
			// Lock:
			bool do_repaint = false;

			if (outputParams.win3D->keyHit())
			{
				mrpt::opengl::COpenGLScenePtr &scene = outputParams.win3D->get3DSceneAndLock();

				const int c = outputParams.win3D->getPushedKey();
				switch (c)
				{
				case 27:
				case 'q':
				case 'Q':
					outputParams.show_live_3D = false;
					break;

				case 'i':
				case 'I':
					{
						mrpt::opengl::CRenderizablePtr obj = mrpt::opengl::CSetOfObjectsPtr(scene->getByName("world"))->getByName("node_labels");
						if (obj) obj->setVisibility( !obj->isVisible() );
						do_repaint=true;
						break;
					}

				case 'l':
				case 'L':
					{
						mrpt::opengl::CRenderizablePtr obj = mrpt::opengl::CSetOfObjectsPtr(scene->getByName("world"))->getByName("landmarks");
						if (obj) obj->setVisibility( !obj->isVisible() );
						do_repaint=true;
						break;
					}
				};

				// Unlock:
				outputParams.win3D->unlockAccess3DScene();
			} // process key hits


			if (++nStepForDecimateGUI>=outputParams.show_live_3D_decimate)
			{
				nStepForDecimateGUI=0;
				do_repaint=true;
		
				mrpt::opengl::COpenGLScenePtr &scene = outputParams.win3D->get3DSceneAndLock();
				
				if (!gl_robot)
				{
					gl_robot = scene->getByName("robot");
					if (!gl_robot)
					{
						mrpt::opengl::CSetOfObjectsPtr gl_rob = mrpt::opengl::stock_objects::CornerXYZSimple(1.0f,2.0f);
						gl_rob->setName("robot");
						scene->insert(gl_rob);
						gl_robot = gl_rob;
					}
				}
				// Update robot representation pose:
				gl_robot->setPose(sim.curPose);

				outputParams.win3D->setCameraPointingToPoint( sim.curPose.x(), sim.curPose.y(), sim.curPose.z() );

				// Unlock:
				outputParams.win3D->unlockAccess3DScene();
			}
			
			if (do_repaint)
			{
				outputParams.win3D->repaint();
				mrpt::system::sleep( outputParams.show_live_3D_sleep_ms );
			}
		}

		// Save observations & groundtruth to files:
		// -----------------------------------------------
		if (isBinary)
		       outputParams.output_bin_rawlog << new_obs_bin;
		else   outputParams.output_text_sensor << new_obs_txt;

		// GT pose: Save as quaternion since its meaning is clear and unambiguous for everyone:
		outputParams.output_text_groundtruth << mrpt::format("%6u %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n",
			static_cast<unsigned int>(sim.step_count),
			sensor_GT_Pose.x(),sensor_GT_Pose.y(),sensor_GT_Pose.z(),
			sensor_GT_Pose.quat().r(), sensor_GT_Pose.quat().x(), sensor_GT_Pose.quat().y(), sensor_GT_Pose.quat().z(), 
			robot_GT_Pose.x(), robot_GT_Pose.y(), robot_GT_Pose.z(),
			robot_GT_Pose.quat().r(), robot_GT_Pose.quat().x(), robot_GT_Pose.quat().y(), robot_GT_Pose.quat().z() );


		// Put here so the "continue" in case of not interpol. does not increment it:
		sim.step_count++;

		if ((sim.step_count % 500)==0)
		{
			cout << mrpt::format("Simulation step %7u | Progress: %.03f%%    \r",
				static_cast<unsigned int>(sim.step_count),
				100*mrpt::system::timeDifference(t0,t)/At_total_tim );
			cout.flush();
		}

	} // end for every "t"

	// Give an opportunity for delayed-output:
	sensor->endSimul(outputParams);

	if (sim.warning_no_observation_count)
		cerr << "WARNING: Zero observations were detected by the sensor during " << sim.warning_no_observation_count << " frames.\n";

	if (sim.warning_no_interpolation_count)
		cerr << "WARNING: Couldn't interpolate path for " << sim.warning_no_interpolation_count << " time steps.\n";
}
