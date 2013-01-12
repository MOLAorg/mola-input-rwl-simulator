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

#include <mrpt/random.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/utils/CConfigFile.h>

namespace rwt
{
	using namespace std;
	using namespace mrpt::utils;
	using namespace mrpt::math;

	struct TrangeBearingSensorObsData
	{
		double range, yaw, pitch;  // Sensed point
	};

	/** Sensor implementation: A generic sensor giving (range, yaw, pitch) measurements.
	  */
	struct SensorSimul_RangeBearing : public SensorSimulBase
	{
		float                 m_minRange,m_maxRange;       //!< In meters
		double                m_fov_h, m_fov_v; //!< In radians (Field of view)
		mrpt::poses::CPose3D  m_sensor_pose_on_robot;
		double                m_range_noise_std,m_yaw_noise_std,m_pitch_noise_std;
		unsigned int          m_check_min_features_per_frame;

		SensorSimul_RangeBearing(
			const RWT_World & world,
			const RWT_SensorOptions & sensorParams
			) :
				SensorSimulBase(world,sensorParams),
				m_minRange(0), m_maxRange(20),
				m_fov_h(DEG2RAD(180)),m_fov_v(DEG2RAD(120)),
				m_sensor_pose_on_robot(0,0,0,DEG2RAD(0),DEG2RAD(0),DEG2RAD(0)),
				m_range_noise_std(0),m_yaw_noise_std(0),m_pitch_noise_std(0),
				m_check_min_features_per_frame(0)
		{
			m_minRange = sensorParams.cfg_file.read_double("sensor","minRange",m_minRange);
			m_maxRange = sensorParams.cfg_file.read_double("sensor","maxRange",m_maxRange);
			m_fov_h = DEG2RAD( sensorParams.cfg_file.read_double("sensor","fov_h",RAD2DEG(m_fov_h)) );
			m_fov_v = DEG2RAD( sensorParams.cfg_file.read_double("sensor","fov_v",RAD2DEG(m_fov_v)) );
			
			m_range_noise_std = sensorParams.cfg_file.read_double("sensor","range_noise_std",m_range_noise_std);
			m_yaw_noise_std = DEG2RAD( sensorParams.cfg_file.read_double("sensor","yaw_noise_std",RAD2DEG(m_yaw_noise_std)) );
			m_pitch_noise_std = DEG2RAD( sensorParams.cfg_file.read_double("sensor","pitch_noise_std",RAD2DEG(m_pitch_noise_std)) );

			m_check_min_features_per_frame = sensorParams.cfg_file.read_uint64_t("sensor","check_min_features_per_frame",m_check_min_features_per_frame);
		}

		virtual void simulate(
			SimulContext                 & sim,
			const bool                    is_binary,
			mrpt::slam::CObservationPtr  & out_observation_bin,
			std::string                  & out_observation_text,
			mrpt::poses::CPose3DQuat     & out_GT_sensor_pose
			)
		{
			mrpt::poses::CPose3D cam_pose(mrpt::poses::UNINITIALIZED_POSE);
			cam_pose.composeFrom(sim.curPose, m_sensor_pose_on_robot);

			out_GT_sensor_pose = mrpt::poses::CPose3DQuat(cam_pose);

			// Get the list of closest LMs using the KD-tree:
			vector<pair<size_t,float> > nearby_LMs;
			this->m_world.landmarks.kdTreeRadiusSearch3D(
				cam_pose.x(), cam_pose.y(), cam_pose.z(),
				mrpt::utils::square(m_maxRange+0.1),
				nearby_LMs);

			// Convert to sensor-centric coordinates:
			vector<pair<size_t,TrangeBearingSensorObsData> >  lst_observed_landmarks;
			lst_observed_landmarks.reserve(nearby_LMs.size());

			for (size_t i=0;i<nearby_LMs.size();i++)
			{
				const size_t idxLM = nearby_LMs[i].first;
				float gx,gy,gz;
				this->m_world.landmarks.getPoint(idxLM, gx,gy,gz);

				double range,yaw,pitch;
				cam_pose.sphericalCoordinates(TPoint3D(gx,gy,gz), range,yaw,pitch);

				if (range<m_minRange || range>m_maxRange || std::abs(yaw)>m_fov_h || std::abs(pitch)>m_fov_v)
					continue;

				// Add noise:
				range += mrpt::random::randomGenerator.drawGaussian1D(0,m_range_noise_std);
				yaw += mrpt::random::randomGenerator.drawGaussian1D(0,m_yaw_noise_std);
				pitch += mrpt::random::randomGenerator.drawGaussian1D(0,m_pitch_noise_std);

				TrangeBearingSensorObsData obs;
				obs.range = range;
				obs.yaw = yaw;
				obs.pitch = pitch;
				lst_observed_landmarks.push_back( std::pair<size_t,TrangeBearingSensorObsData>(idxLM,obs) );
			}

			if (lst_observed_landmarks.size()<m_check_min_features_per_frame)
				throw std::runtime_error(mrpt::format("ERROR: Minimum number of features established in 'check_min_features_per_frame' does not hold: #feats=%u at step=%u",static_cast<unsigned int>(lst_observed_landmarks.size()), static_cast<unsigned int>(sim.step_count) ));

			if (sim.show_live_3D && sim.win3D)
			{
				mrpt::gui::CDisplayWindow3D* win = const_cast<mrpt::gui::CDisplayWindow3D*>(sim.win3D.pointer()); // Well...yeah!
				win->addTextMessage(
					5,5,
					mrpt::format("Observed LMs: %u",static_cast<unsigned int>(lst_observed_landmarks.size()) ),
					mrpt::utils::TColorf(1,1,1), "mono", 10, mrpt::opengl::NICE,
					1000 /* unique ID */ );
			}

			// Warning: No LM observed:
			if (lst_observed_landmarks.size()==0)
				sim.warning_no_observation_count++;

			// Output:
			if (is_binary)
			{
				THROW_EXCEPTION("TODO")
			}
			else
			{
				out_observation_text.clear();

				for (size_t i=0;i<lst_observed_landmarks.size();i++)
				{
					if (!m_sensorParams.observations_as_c_structs)
						out_observation_text+= mrpt::format("%6u %6u %10.3f %10.3f %10.3f\n",
							static_cast<unsigned int>(sim.step_count),
							static_cast<unsigned int>(lst_observed_landmarks[i].first),
							lst_observed_landmarks[i].second.range,
							lst_observed_landmarks[i].second.yaw,
							lst_observed_landmarks[i].second.pitch
							);
					else
						out_observation_text+= mrpt::format(" {%6u,%10.3f,%10.3f,%10.3f}\n",
							static_cast<unsigned int>(lst_observed_landmarks[i].first),
							lst_observed_landmarks[i].second.range,
							lst_observed_landmarks[i].second.yaw,
							lst_observed_landmarks[i].second.pitch
							);
				}
			}
		}

		virtual void describeObservation(RWT_OutputOptions & outputParams)
		{
			outputParams.output_text_sensor <<
				"%                           RANGE(m)  YAW(rad)  PITCH(rad)           \n"
				"% -------------------------------------------------------------------\n";
		}

	}; // end of SensorSimul_RangeBearing

}
