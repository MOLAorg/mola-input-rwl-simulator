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

#include <mrpt/obs.h>
#include <memory>  // for auto_ptr<>

namespace rwt
{

	struct SimulContext
	{
		SimulContext() :
			curPose       ( ),
			step_count    (0),
			warning_no_observation_count   (0),
			warning_no_interpolation_count (0),

			show_live_3D  (false),
			win3D()
		{ }

		mrpt::poses::CPose3D curPose;  // The current robot pose in the world.
		size_t   step_count;
		size_t   warning_no_observation_count;
		size_t   warning_no_interpolation_count;

		bool show_live_3D;
		mrpt::gui::CDisplayWindow3DPtr win3D;
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
			SimulContext                 & sim,
			const bool                    is_binary,
			mrpt::slam::CObservationPtr  & out_observation_bin,
			std::string                  & out_observation_text,
			mrpt::poses::CPose3DQuat     & out_GT_sensor_pose
			) = 0;
	}; // end of SensorSimulBase

	typedef std::auto_ptr<SensorSimulBase>  SensorSimulBasePtr;

} // end namespace

// -----------------------------------------
//     Include all sensors:
// -----------------------------------------
#include "rwt-sensor-camera.h"

