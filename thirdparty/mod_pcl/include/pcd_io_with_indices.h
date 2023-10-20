/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */

#ifndef PCL_IO_PCD_IO_WITH_INDICES_IMPL_H_
#define PCL_IO_PCD_IO_WITH_INDICES_IMPL_H_

#include <boost/algorithm/string/trim.hpp> // for trim
#include <fstream>
#include <fcntl.h>
#include <string>
#include <cstdlib>
#include <pcl/common/io.h> // for getFields, ...
#include <pcl/console/print.h>
#include <pcl/io/low_level_io.h>
#include <pcl/io/pcd_io.h>

#include <pcl/io/lzf.h>

namespace pcl
{
  class PCDWriterWithIndices : public PCDWriter
  {
    template <typename PointT>
    std::string
    generateHeaderWithIndices(const pcl::PointCloud<PointT> &cloud, const pcl::PointIndices &indices)
    {
      std::ostringstream oss;
      oss.imbue(std::locale::classic());

      oss << "# .PCD v0.7 - Point Cloud Data file format"
             "\nVERSION 0.7"
             "\nFIELDS";

      const auto fields = pcl::getFields<PointT>();

      std::stringstream field_names, field_types, field_sizes, field_counts;
      for (const auto &field : fields)
      {
        if (field.name == "_")
          continue;
        // Add the regular dimension
        field_names << " " << field.name;
        field_sizes << " " << pcl::getFieldSize(field.datatype);
        if ("rgb" == field.name)
          field_types << " "
                      << "U";
        else
          field_types << " " << pcl::getFieldType(field.datatype);
        int count = std::abs(static_cast<int>(field.count));
        if (count == 0)
          count = 1; // check for 0 counts (coming from older converter code)
        field_counts << " " << count;
      }
      oss << field_names.str();
      oss << "\nSIZE" << field_sizes.str()
          << "\nTYPE" << field_types.str()
          << "\nCOUNT" << field_counts.str();
      oss << "\nWIDTH " << indices.indices.size() << "\nHEIGHT " << 1 << "\n";

      oss << "VIEWPOINT " << cloud.sensor_origin_[0] << " " << cloud.sensor_origin_[1] << " " << cloud.sensor_origin_[2] << " " << cloud.sensor_orientation_.w() << " " << cloud.sensor_orientation_.x() << " " << cloud.sensor_orientation_.y() << " " << cloud.sensor_orientation_.z() << "\n";

      // If the user passes in a number of points value, use that instead
      oss << "POINTS " << indices.indices.size() << "\n";

      return (oss.str());
    }

    public:
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template <typename PointT>
    int writeBinaryWithIndices(const std::string &file_name,
                               const pcl::PointCloud<PointT> &cloud,
                               const pcl::PointIndices &indices)
    {
      if (cloud.empty() && indices.indices.empty())
      {
        PCL_WARN("[pcl::PCDWriter::writeBinary] Input point cloud has no data!\n");
      }
      std::ostringstream oss;
      oss << generateHeaderWithIndices<PointT>(cloud, indices) << "DATA binary\n";
      oss.flush();
      const auto data_idx = static_cast<unsigned int>(oss.tellp());

#ifdef _WIN32
      HANDLE h_native_file = CreateFileA(file_name.c_str(), GENERIC_READ | GENERIC_WRITE, 0, NULL, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);
      if (h_native_file == INVALID_HANDLE_VALUE)
      {
        throw pcl::IOException("[pcl::PCDWriter::writeBinary] Error during CreateFile!");
      }
#else
      int fd = io::raw_open(file_name.c_str(), O_RDWR | O_CREAT | O_TRUNC, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
      if (fd < 0)
      {
        throw pcl::IOException("[pcl::PCDWriter::writeBinary] Error during open!");
      }
#endif
      // Mandatory lock file
      boost::interprocess::file_lock file_lock;
      setLockingPermissions(file_name, file_lock);

      auto fields = pcl::getFields<PointT>();
      std::vector<int> fields_sizes;
      std::size_t fsize = 0;
      std::size_t data_size = 0;
      std::size_t nri = 0;
      // Compute the total size of the fields
      for (const auto &field : fields)
      {
        if (field.name == "_")
          continue;

        int fs = field.count * getFieldSize(field.datatype);
        fsize += fs;
        fields_sizes.push_back(fs);
        fields[nri++] = field;
      }
      fields.resize(nri);

      data_size = indices.indices.size() * fsize;

      // Prepare the map
#ifdef _WIN32
      HANDLE fm = CreateFileMappingA(h_native_file, NULL, PAGE_READWRITE, (DWORD)((data_idx + data_size) >> 32), (DWORD)(data_idx + data_size), NULL);
      if (fm == NULL)
      {
        throw pcl::IOException("[pcl::PCDWriter::writeBinary] Error during memory map creation ()!");
      }
      char *map = static_cast<char *>(MapViewOfFile(fm, FILE_MAP_READ | FILE_MAP_WRITE, 0, 0, data_idx + data_size));
      if (map == NULL)
      {
        CloseHandle(fm);
        throw pcl::IOException("[pcl::PCDWriter::writeBinary] Error mapping view of file!");
      }
      CloseHandle(fm);

#else
      // Allocate disk space for the entire file to prevent bus errors.
      const int allocate_res = io::raw_fallocate(fd, data_idx + data_size);
      if (allocate_res != 0)
      {
        io::raw_close(fd);
        resetLockingPermissions(file_name, file_lock);
        PCL_ERROR("[pcl::PCDWriter::writeBinary] raw_fallocate(length=%zu) returned %i. errno: %d strerror: %s\n",
                  data_idx + data_size, allocate_res, errno, strerror(errno));

        throw pcl::IOException("[pcl::PCDWriter::writeBinary] Error during raw_fallocate ()!");
      }

      char *map = static_cast<char *>(::mmap(nullptr, data_idx + data_size, PROT_WRITE, MAP_SHARED, fd, 0));
      if (map == reinterpret_cast<char *>(-1)) // MAP_FAILED)
      {
        io::raw_close(fd);
        resetLockingPermissions(file_name, file_lock);
        throw pcl::IOException("[pcl::PCDWriter::writeBinary] Error during mmap ()!");
      }
#endif

      // Copy the header
      memcpy(&map[0], oss.str().c_str(), data_idx);

      // Copy the data
      char *out = &map[0] + data_idx;
      for (const auto &idx : indices.indices)
      {
        const auto &point = cloud[idx];
        int nrj = 0;
        for (const auto &field : fields)
        {
          memcpy(out, reinterpret_cast<const char *>(&point) + field.offset, fields_sizes[nrj]);
          out += fields_sizes[nrj++];
        }
      }

      // If the user set the synchronization flag on, call msync
#ifndef _WIN32
      //if (map_synchronization_)
      if (false)
        msync(map, data_idx + data_size, MS_SYNC);
#endif

        // Unmap the pages of memory
#ifdef _WIN32
      UnmapViewOfFile(map);
#else
      if (::munmap(map, (data_idx + data_size)) == -1)
      {
        io::raw_close(fd);
        resetLockingPermissions(file_name, file_lock);
        throw pcl::IOException("[pcl::PCDWriter::writeBinary] Error during munmap ()!");
      }
#endif
      // Close file
#ifdef _WIN32
      CloseHandle(h_native_file);
#else
      io::raw_close(fd);
#endif
      resetLockingPermissions(file_name, file_lock);
      return (0);
    }
  };
}

#endif