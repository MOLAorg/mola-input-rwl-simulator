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
#include "rwt-sensor-simulators.h" // Declaration of abstract (and particular) sensor classes.

#include <mrpt/system/threads.h>  // for sleep()

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

	SimulContext sim;
	// Set extra params:
	sim.show_live_3D = outputParams.show_live_3D;
	sim.win3D        = outputParams.win3D;

	const size_t nWayPoints = waypoints.size();
	const bool   isBinary   = outputParams.is_binary;

	// Create sensor:
	SensorSimulBasePtr sensor = SensorSimulBasePtr(new SensorSimul_Camera(world,sensorParams) );

	mrpt::opengl::CRenderizablePtr gl_robot; // Cache this pointer to avoid looking for it with each iteration.

	for ( ; sim.next_waypoint<nWayPoints; ++sim.step_count)
	{
		// Simulate sensor readings at current location:
		// -----------------------------------------------
		mrpt::slam::CObservationPtr  new_obs_bin;
		std::string                  new_obs_txt;

		sensor->simulate(sim, isBinary, new_obs_bin, new_obs_txt );

		// Update 3D view?
		// ----------------------------
		if (outputParams.show_live_3D)
		{
			// Lock:
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

			// Unlock:
			outputParams.win3D->unlockAccess3DScene();

			outputParams.win3D->repaint();
			mrpt::system::sleep( outputParams.show_live_3D_sleep_ms );
		}

		// Save observations & groundtruth to files:
		// -----------------------------------------------
		if (isBinary)
		       outputParams.output_bin_rawlog << new_obs_bin;
		else   outputParams.output_text_sensor << new_obs_txt;

		// GT pose: Save as quaternion since its meaning is clear and unambiguous for everyone:
		const mrpt::poses::CPose3DQuat curPoseQuat = mrpt::poses::CPose3DQuat(sim.curPose);
		outputParams.output_text_groundtruth << mrpt::format("%6u %f %f %f %f %f %f %f\n",
			static_cast<unsigned int>(sim.step_count),
			curPoseQuat.x(),curPoseQuat.y(),curPoseQuat.z(),
			curPoseQuat.quat().r(), curPoseQuat.quat().x(), curPoseQuat.quat().y(), curPoseQuat.quat().z() );

		// Move the robot:
		// -----------------------------------------------
		// Next waypoint Absolute coords
		const mrpt::math::TPoint3D & nextWp = waypoints[sim.next_waypoint];

		// In spherical coords:
		double wp_r, wp_yaw, wp_pitch;
		sim.curPose.sphericalCoordinates(nextWp, wp_r, wp_yaw, wp_pitch);

		// Are we already heading towards the next waypoint?
		if (std::abs(wp_yaw)<1e-3 && std::abs(wp_pitch)<1e-3)
		{
			// Move straight:
			const double move_dist = std::min(wp_r, pathParams.max_step_lin );
			sim.curPose += mrpt::poses::CPose3D(move_dist,0,0, 0,0,0);
		}
		else
		{
			const double maxR = pathParams.max_step_ang;

			// Rotate:
			const double Ayaw   = std::min( std::max(-maxR,wp_yaw  ), maxR);
			const double Apitch = std::min( std::max(-maxR,wp_pitch), maxR);

			sim.curPose += mrpt::poses::CPose3D(0,0,0, Ayaw,Apitch,0);
		}

		// Close enough?
		if ( sim.curPose.distance3DTo(nextWp.x,nextWp.y,nextWp.z)<1e-3 )
		{
			++sim.next_waypoint;
		}

	} // end while

}
