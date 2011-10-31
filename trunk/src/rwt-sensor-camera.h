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

#include <mrpt/vision/pinhole.h>
#include <mrpt/random.h>
#include <mrpt/utils/CConfigFile.h>

namespace rwt
{
	using namespace std;
	using namespace mrpt::utils;

	/** Sensor implementation: Camera
	  */
	struct SensorSimul_Camera : public SensorSimulBase
	{
		float                 m_maxRange;       //!< In meters
		mrpt::poses::CPose3D  m_camera_pose_on_robot;
		mrpt::utils::TCamera  m_camera_params;  //!< Camera description
		float                 m_camera_pixel_noise_std;

		SensorSimul_Camera(const RWT_World & world, const RWT_SensorOptions & sensorParams) :
			SensorSimulBase(world,sensorParams),
			m_maxRange(20),
			m_camera_pose_on_robot(0,0,0,DEG2RAD(-90),0,DEG2RAD(-90)),
			m_camera_pixel_noise_std(0)
		{
			m_camera_params.ncols = 640;
			m_camera_params.nrows = 480;

			m_camera_params.cx(m_camera_params.ncols>>1);
			m_camera_params.cy(m_camera_params.nrows>>1);

			m_camera_params.fx(m_camera_params.ncols);
			m_camera_params.fy(m_camera_params.ncols);

			// Save output param files:
			const string sOutCAMCALIB = sensorParams.sOutFilesPrefix + string("_CAMCALIB.txt");
			m_camera_params.saveToConfigFile("CAMERA", mrpt::utils::CConfigFile(sOutCAMCALIB)); 
		}

		virtual void simulate(
			const SimulContext           & sim,
			const bool                     is_binary,
			mrpt::slam::CObservationPtr  & out_observation_bin,
			std::string                  & out_observation_text,
			mrpt::poses::CPose3DQuat     & out_GT_sensor_pose
			)
		{
			const mrpt::poses::CPose3D cam_pose = sim.curPose + m_camera_pose_on_robot;
			out_GT_sensor_pose = mrpt::poses::CPose3DQuat(cam_pose);

			// Get the list of closest LMs using the KD-tree:
			vector<pair<int,float> > nearby_LMs;
			this->m_world.landmarks.kdTreeRadiusSearch3D(
				cam_pose.x(), cam_pose.y(), cam_pose.z(),
				m_maxRange+0.1,
				nearby_LMs);

			// Convert to pixel coordinates, and leave only those that fall within the camera image limits:
			vector<pair<size_t,TPixelCoordf> >  lst_observed_landmarks;
			lst_observed_landmarks.reserve(nearby_LMs.size());

			for (size_t i=0;i<nearby_LMs.size();i++)
			{
				const int idxLM = nearby_LMs[i].first;
				float gx,gy,gz;
				this->m_world.landmarks.getPoint(idxLM, gx,gy,gz);

				double lx,ly,lz;
				cam_pose.inverseComposePoint(gx,gy,gz, lx,ly,lz);

				if (lz>0)
				{
					TPixelCoordf px;
					mrpt::vision::pinhole::projectPoint_with_distortion(
						mrpt::math::TPoint3D(lx,ly,lz),
						m_camera_params, px,
						true /* no need to filter points behind our back; done already */
						);

					// Add noise:
					px.x += mrpt::random::randomGenerator.drawGaussian1D(0,m_camera_pixel_noise_std);
					px.y += mrpt::random::randomGenerator.drawGaussian1D(0,m_camera_pixel_noise_std);

					if (px.x>0 && px.x<m_camera_params.ncols &&
					    px.y>0 && px.y<m_camera_params.nrows )
					{
						lst_observed_landmarks.push_back( std::make_pair<size_t,TPixelCoordf>(idxLM,px) );
					}
				}
			}

			if (sim.show_live_3D && sim.win3D)
			{
				mrpt::gui::CDisplayWindow3D* win = const_cast<mrpt::gui::CDisplayWindow3D*>(sim.win3D.pointer()); // Well...yeah!
				win->addTextMessage(
					5,5,
					mrpt::format("Observed LMs: %u",static_cast<unsigned int>(lst_observed_landmarks.size()) ),
					mrpt::utils::TColorf(1,1,1), "mono", 10, mrpt::opengl::NICE,
					1000 /* unique ID */ );
			}

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
					out_observation_text+= mrpt::format("%6u %6u %10.3f %10.3f\n",
						static_cast<unsigned int>(sim.step_count),
						static_cast<unsigned int>(lst_observed_landmarks[i].first),
						lst_observed_landmarks[i].second.x,
						lst_observed_landmarks[i].second.y
						);
				}
			}
		}
	}; // end of SensorSimul_Camera

}
