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

#include <mrpt/obs.h>
#include <memory>  // for auto_ptr<>

using namespace rwt;
using namespace std;


struct SimulContext
{
	SimulContext() :
		curPose       ( ),
		step_count    (0),
		next_waypoint (0)
	{ }

	mrpt::poses::CPose3D curPose;  // The current robot pose in the world.
	size_t   step_count;
	size_t   next_waypoint;  // The waypoint we're right now heading to.
};


/** Virtual base class for all sensor simulator */
struct SensorSimulBase
{
	SensorSimulBase(const RWT_World & world, const RWT_SensorOptions & sensorParams) :
		m_world(world),
		m_sensorParams(sensorParams)
	{ }

	const RWT_World          & m_world;
	const RWT_SensorOptions  & m_sensorParams;

	virtual void simulate(
		const SimulContext           & sim,
		const bool                     is_binary,
		mrpt::slam::CObservationPtr  & out_observation_bin,
		std::string                  & out_observation_text
		) = 0;
}; // end of SensorSimulBase

typedef std::auto_ptr<SensorSimulBase>  SensorSimulBasePtr;

/** Virtual base class for all sensor simulator */
struct SensorSimul_Camera : public SensorSimulBase
{
	SensorSimul_Camera(const RWT_World & world, const RWT_SensorOptions & sensorParams) :
		SensorSimulBase(world,sensorParams)
	{ }

	virtual void simulate(
		const SimulContext           & sim,
		const bool                     is_binary,
		mrpt::slam::CObservationPtr  & out_observation_bin,
		std::string                  & out_observation_text
		)
	{
		//this->m_world.


	}
}; // end of SensorSimul_Camera


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

	const size_t nWayPoints = waypoints.size();
	const bool   isBinary   = outputParams.is_binary;

	// Create sensor:
	SensorSimulBasePtr sensor = SensorSimulBasePtr(new SensorSimul_Camera(world,sensorParams) );


	for ( ; sim.next_waypoint<nWayPoints; ++sim.step_count)
	{
		// Simulate sensor readings at current location:
		// -----------------------------------------------
		mrpt::slam::CObservationPtr  new_obs_bin;
		std::string                  new_obs_txt;

		sensor->simulate(sim, isBinary, new_obs_bin, new_obs_txt );

		// Save to files:
		// -----------------------------------------------
		if (isBinary)
		{
			outputParams.output_bin_rawlog << new_obs_bin;
		}
		else
		{
			outputParams.output_text_sensor << new_obs_txt;
		}


		// Move the robot:
		// -----------------------------------------------
		++sim.next_waypoint;

	} // end while

}
