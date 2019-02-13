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
   +---------------------------------------------------------------------------+
 */

#pragma once

#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/random.h>
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/vision/pinhole.h>

#include <mrpt/version.h>
#if MRPT_VERSION >= 0x199
#include <mrpt/core/aligned_std_deque.h>
#endif

namespace rwt {
using namespace std;
using namespace mrpt::utils;

/** Sensor implementation: Relative poses (for datasets of pose constraints)
 */
struct SensorSimul_RelativePoses : public SensorSimulBase {
  float m_minRange, m_maxRange; //!< In meters

  // The XYZ components of all poses (for KD-tree search)
  mrpt::maps::CSimplePointsMap m_poses_points;
  // All poses
#if MRPT_VERSION >= 0x199
  mrpt::aligned_std_deque<mrpt::poses::CPose3D> m_poses;
#else
  mrpt::aligned_containers<mrpt::poses::CPose3D>::deque_t m_poses;
#endif

  SensorSimul_RelativePoses(const RWT_World &world,
                            const RWT_SensorOptions &sensorParams)
      : SensorSimulBase(world, sensorParams), m_minRange(0), m_maxRange(20) {
    m_minRange =
        sensorParams.cfg_file.read_double("sensor", "minRange", m_minRange);
    m_maxRange =
        sensorParams.cfg_file.read_double("sensor", "maxRange", m_maxRange);
  }

  virtual void simulate(SimulContext &sim, const bool is_binary,
                        mrpt::obs::CObservation::Ptr &out_observation_bin,
                        std::string &output_text_sensor,
                        mrpt::poses::CPose3DQuat &out_GT_sensor_pose) {
    out_GT_sensor_pose = mrpt::poses::CPose3DQuat(sim.curPose);
    output_text_sensor.clear();

    // Accumulate all points, later on we'll process them all in endSimul()
    m_poses_points.insertPoint(sim.curPose.x(), sim.curPose.y(),
                               sim.curPose.z());
    m_poses.push_back(sim.curPose);
  }

  virtual void endSimul(RWT_OutputOptions &outputParams) {
    const size_t N = m_poses.size();

    for (size_t kf_id = 0; kf_id < N; ++kf_id) {
      // Get the list of closest LMs using the KD-tree:
      vector<pair<size_t, float>> nearby_KFs;

      const mrpt::poses::CPose3D &p = m_poses[kf_id];

      m_poses_points.kdTreeRadiusSearch3D(
          p.x(), p.y(), p.z(), mrpt::square(m_maxRange + 0.1), nearby_KFs);

      for (size_t i = 0; i < nearby_KFs.size(); i++) {
        const size_t seen_kf_id = nearby_KFs[i].first;

        if (seen_kf_id >= kf_id)
          continue; // Do not observe the future!

        const mrpt::poses::CPose3D &seen_p = m_poses[seen_kf_id];

        const mrpt::poses::CPose3D delta = seen_p - p;

        const double dist = delta.norm();
        if (dist < m_minRange || dist > m_maxRange)
          continue;

        // Output:
        if (outputParams.is_binary) {
          THROW_EXCEPTION("TODO");
        } else {
          outputParams.output_text_sensor.clear();
          const mrpt::poses::CPose3DQuat q = mrpt::poses::CPose3DQuat(delta);

          if (!m_sensorParams.observations_as_c_structs)
            outputParams.output_text_sensor << mrpt::format(
                "%6u %6u %15.8f %15.8f %15.8f %15.8f %15.8f %15.8f  %15.8f "
                "%15.8f %15.8f %15.8f \n",
                static_cast<unsigned int>(kf_id),
                static_cast<unsigned int>(seen_kf_id), delta.x(), delta.y(),
                delta.z(), delta.yaw(), delta.pitch(), delta.roll(),
                q.quat().r(), q.quat().x(), q.quat().y(), q.quat().z());
          else
            outputParams.output_text_sensor << mrpt::format(
                " {%6u, %6u, %15.8f, %15.8f, %15.8f, %15.8f, %15.8f, %15.8f, "
                "%15.8f, %15.8f, %15.8f, %15.8f},\n",
                static_cast<unsigned int>(kf_id),
                static_cast<unsigned int>(seen_kf_id), delta.x(), delta.y(),
                delta.z(), delta.yaw(), delta.pitch(), delta.roll(),
                q.quat().r(), q.quat().x(), q.quat().y(), q.quat().z());
        }
      }
    }
  }

  virtual void describeObservation(RWT_OutputOptions &outputParams) {
    outputParams.output_text_sensor
        << "% REL. POSES: OBSERVED_KEYFRAME_ID     X     Y     Z     YAW   "
           "PITCH   ROLL     QR      QX      QY      QZ \n"
           "% "
           "-------------------------------------------------------------------"
           "---------------------------------------\n";
  }

}; // end of SensorSimul_RelativePoses

} // namespace rwt
