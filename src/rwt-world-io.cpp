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

#include "rwt.h"

#include <iterator>
#include <mrpt/opengl/CRenderizable.h>
#include <mrpt/system/filesystem.h>
#if MRPT_VERSION >= 0x199
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/md5.h>
using mrpt::graphs::TPairNodeIDs;
using mrpt::io::CFileGZInputStream;
using mrpt::system::md5;
#else
#include <mrpt/utils/md5.h>
using mrpt::utils::CFileGZInputStream;
using mrpt::utils::md5;
using namespace mrpt::utils;
#endif

using namespace rwt;
using namespace std;
using namespace mrpt;

std::vector<uint8_t> readFile(const char *filename) {
  // open the file:
  std::ifstream file(filename, std::ios::binary);

  // Stop eating new lines in binary mode!!!
  file.unsetf(std::ios::skipws);

  // get its size:
  std::streampos fileSize;

  file.seekg(0, std::ios::end);
  fileSize = file.tellg();
  file.seekg(0, std::ios::beg);

  // reserve capacity
  std::vector<uint8_t> vec;
  vec.reserve(fileSize);

  // read the data:
  vec.insert(vec.begin(), std::istream_iterator<uint8_t>(file),
             std::istream_iterator<uint8_t>());

  return vec;
}

/** Load a world from a compiled .crwt.gz file \sa compile_and_run_rwt_program,
 * save_rwt_world \return false on any error  */
bool rwt::load_rwt_world(RWT_World &world, const std::string &file,
                         const std::string &source_file) {
  world.clear();

  CFileGZInputStream fi;
  if (!fi.open(file))
    return false;

#if MRPT_VERSION >= 0x199
  auto f = mrpt::serialization::archiveFrom(fi);
#else
  auto &f = fi;
#endif

  string true_src_md5;
  if (!source_file.empty() && mrpt::system::fileExists(source_file)) {
    std::vector<uint8_t> srcFileBin = readFile(source_file.c_str());
    true_src_md5 = md5(srcFileBin);
  }

  // Load:
  try {
    string stored_src_md5;
    f >> stored_src_md5;

    if (!true_src_md5.empty() && true_src_md5 != stored_src_md5) {
      return false; // Source file has changed!
    } else {
      // Keep reading:
      // f >> world.graph.edges;
      {
        uint64_t N;
        f >> N;
        for (size_t i = 0; i < N; i++) {
          uint64_t id1, id2;
          f >> id1 >> id2;
          world.graph.edges.insert(
              world.graph.edges.end(),
              make_pair(TPairNodeIDs(id1, id2), RWT_graph_edge()));
        }
      }
      f >> world.landmarks;
      f >> world.nodes;
      return true;
    }
  } catch (...) {
    return false;
  }
}

/** Save a world to a compiled .crwt.gz file \sa compile_and_run_rwt_program,
 * load_rwt_world \return false on any error  */
bool rwt::save_rwt_world(const RWT_World &world, const std::string &file,
                         const std::string &source_file) {
  mrpt::utils::CFileGZOutputStream fo;
  if (!fo.open(file))
    return false;

#if MRPT_VERSION >= 0x199
  auto f = mrpt::serialization::archiveFrom(fo);
#else
  auto &f = fo;
#endif

  string srcMD5 = "00";
  if (!source_file.empty() && mrpt::system::fileExists(source_file)) {
    std::vector<uint8_t> srcFileBin = readFile(source_file.c_str());
    srcMD5 = md5(srcFileBin);
  }

  // Dump to file:
  // --------------------
  try {
    f << srcMD5;
    {
      uint64_t N = world.graph.edges.size();
      f << N;
      for (RWT_adjacency_graph::const_iterator it = world.graph.edges.begin();
           it != world.graph.edges.end(); ++it)
        f << uint64_t(it->first.first) << uint64_t(it->first.second);
    }
    f << world.landmarks;
    f << world.nodes;
    return true;
  } catch (...) {
    return false;
  }
}
