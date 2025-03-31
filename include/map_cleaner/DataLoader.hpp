#pragma once

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/common/transforms.h>
#include <string>
#include <memory>
#include <fstream>
#include <sstream>
#include <vector>
#include <map_cleaner/utils.hpp>

class DataLoaderBase
{
public:
  typedef std::shared_ptr<DataLoaderBase> Ptr;
  typedef std::shared_ptr<const DataLoaderBase> ConstPtr;

  struct Frame
  {
    size_t idx = 0;
    CloudType::Ptr frame = CloudType::Ptr(new CloudType);
    Eigen::Vector3f t = Eigen::Vector3f::Identity();
    Eigen::Quaternionf r = Eigen::Quaternionf::Identity();
  };
  
  struct FrameInfo
  {
    size_t idx = 0;
    std::string pcd_filename = "";
    Eigen::Vector3f t = Eigen::Vector3f::Identity();
    Eigen::Quaternionf r = Eigen::Quaternionf::Identity();
  };

protected:
  std::vector<FrameInfo> frame_info_buf_;

public:
  size_t getSize() const { return frame_info_buf_.size(); }
  FrameInfo getFrameInfo(const size_t idx) const
  {
    if(idx < frame_info_buf_.size())
      return frame_info_buf_[idx];
    return FrameInfo();
  }
  
  virtual size_t loadFrameInfo(const std::string &pcds_dir, const std::string &pose_file, const int start = 0, const int end = -1) = 0;
  virtual Frame loadFrame(const size_t idx) = 0;
};

class KittiFormatLoader : public DataLoaderBase
{
  Eigen::Affine3f calib_ = Eigen::Affine3f::Identity();
  Eigen::Affine3f calib_inv_ = Eigen::Affine3f::Identity();
  
  Eigen::Affine3f parseLine(const std::string &str)
  {
    int num = 0;
    std::string parse_str;
    std::istringstream i_stream(str);
    
    Eigen::Affine3f res = Eigen::Affine3f::Identity();
    while (getline(i_stream, parse_str, ' '))
    {
      res(num / 4, num %  4) = std::stof(parse_str);
      num++;
    }
    return res;
  }
  
public:
  bool loadKittiCalibration(const std::string &calibration_file)
  {
    std::ifstream ifs(calibration_file);
    std::string csv_line;
    while (getline(ifs, csv_line))
    {
      if(csv_line.find("Tr:") != std::string::npos){
        calib_ = parseLine(csv_line.substr(4));
        calib_inv_ = calib_.inverse();
        return true;
      }
    }
    return false;
  }

  size_t loadFrameInfo(const std::string &pcds_dir, const std::string &pose_file, const int start, const int end) override
  {
    frame_info_buf_.clear();

    int line_num = 0;
    std::ifstream ifs(pose_file);
    std::string csv_line;
    while (getline(ifs, csv_line))
    {
      int tmp = line_num;
      if(tmp >= start){
        if(tmp >= end && start < end)
          break;

        FrameInfo frame_info;
        frame_info.idx = line_num;

        Eigen::Affine3f lidar_pose = calib_inv_ * parseLine(csv_line) * calib_;
        frame_info.t = lidar_pose.translation();
        frame_info.r = Eigen::Quaternionf(lidar_pose.rotation());

        std::stringstream frame_filename_str;
        frame_filename_str << std::setfill('0') << std::right << std::setw(6) << line_num << ".bin";
        frame_info.pcd_filename = pcds_dir + frame_filename_str.str();
    
        frame_info_buf_.push_back(frame_info);
      }

      line_num++;
    }

    return frame_info_buf_.size();
  }

  void loadKittiCloud(const std::string &file_name, CloudType &cloud)
  {
    cloud.clear();
    std::ifstream in(file_name, std::ios::binary | std::ios::ate);
    if(!in.is_open())
      return;

    std::streamsize bytes = in.tellg();
    size_t num_points = bytes / (sizeof(float) * 4);
    cloud.reserve(num_points);

    bool is_dense = true;
    in.seekg(0, std::ios::beg);
    while(in.good() && !in.eof() && cloud.size() < num_points){
      PointType p;
		  in.read((char *)&p.x, sizeof(float));
		  in.read((char *)&p.y, sizeof(float));
		  in.read((char *)&p.z, sizeof(float));
		  in.read((char *)&p.intensity, sizeof(float));

      if(!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
        is_dense = false;

      cloud.push_back(p);
  	}
    cloud.points.shrink_to_fit();
    cloud.is_dense = is_dense;
	  in.close();
  }
  
  Frame loadFrame(const size_t idx) override
  {
    if(idx >= frame_info_buf_.size()){
      return Frame();
    }

    Frame frame;
    CloudType tmp_frame;
    loadKittiCloud(frame_info_buf_[idx].pcd_filename, tmp_frame);
    frame.idx = frame_info_buf_[idx].idx;
    frame.t = frame_info_buf_[idx].t;
    frame.r = frame_info_buf_[idx].r;

    pcl::Indices dummy;
    pcl::removeNaNFromPointCloud(tmp_frame, *frame.frame, dummy);
    
    return frame;
  };
};

class ERASORFormatLoader : public DataLoaderBase
{
  FrameInfo parseLine(const std::string &str)
  {
    int num = 0;
    std::string parse_str;
    std::istringstream i_stream(str);
    FrameInfo frame_info;
    double timestamp;
    Eigen::Vector4f tmp_quat;
    while (getline(i_stream, parse_str, ','))
    {
      switch (num)
      {
      case 0:
        frame_info.idx = std::stoi(parse_str);
        break;
      case 1:
        timestamp = std::stod(parse_str);
        break;
      case 2:
      case 3:
      case 4:
        frame_info.t[num - 2] = std::stof(parse_str);
        break;
      case 5:
      case 6:
      case 7:
      case 8:
        tmp_quat[num - 5] = std::stof(parse_str);
        break;
      default:
        break;
      }
      num++;
    }
    frame_info.r = Eigen::Quaternionf(tmp_quat);
    frame_info.r.normalize();
    return frame_info;
  }

public:
  size_t loadFrameInfo(const std::string &pcds_dir, const std::string &pose_file, const int start, const int end) override
  {
    frame_info_buf_.clear();

    int line_num = 0;
    std::ifstream ifs(pose_file);
    std::string csv_line;
    while (getline(ifs, csv_line))
    {
      int tmp = line_num - 1;
      if(tmp >= start){
        if(tmp >= end && start < end)
          break;

        FrameInfo frame_info = parseLine(csv_line);

        std::stringstream frame_filename_str;
        frame_filename_str << std::setfill('0') << std::right << std::setw(6) << frame_info.idx << ".pcd";
        frame_info.pcd_filename = pcds_dir + frame_filename_str.str();
    
        frame_info_buf_.push_back(frame_info);
      }

      line_num++;
    }
    ROS_INFO_STREAM("Loaded: " << start << " " << end << " " << frame_info_buf_.size());
    return frame_info_buf_.size();
  }
  
  Frame loadFrame(const size_t idx) override
  {
    if(idx >= frame_info_buf_.size()){
      return Frame();
    }

    Frame frame;
    CloudType tmp_frame;
    pcl::io::loadPCDFile<PointType>(frame_info_buf_[idx].pcd_filename, tmp_frame);
    frame.idx = frame_info_buf_[idx].idx;
    frame.t = frame_info_buf_[idx].t;
    frame.r = frame_info_buf_[idx].r;

    pcl::Indices dummy;
    pcl::removeNaNFromPointCloud(tmp_frame, *frame.frame, dummy);
    
    return frame;
  };
};

class GLIMFormatLoader : public DataLoaderBase
{
  const std::string POINTS_FILENAME = "points_compact.bin";
  const std::string INTENSITIES_FILENAME = "intensities_compact.bin";

  FrameInfo parseLine(const std::string &str)
  {
    int num = 0;
    std::string parse_str;
    std::istringstream i_stream(str);
    FrameInfo frame_info;
    double timestamp;
    Eigen::Vector4f tmp_quat;
    while (getline(i_stream, parse_str, ' '))
    {
      switch (num)
      {
      case 0:
        timestamp = std::stod(parse_str);
        break;
      case 1:
      case 2:
      case 3:
        frame_info.t[num - 1] = std::stof(parse_str);
        break;
      case 4:
      case 5:
      case 6:
      case 7:
        tmp_quat[num - 4] = std::stof(parse_str);
        break;
      default:
        break;
      }
      num++;
    }
    frame_info.r = Eigen::Quaternionf(tmp_quat);
    frame_info.r.normalize();
    return frame_info;
  }

public:
  size_t loadFrameInfo(const std::string &pcds_dir, const std::string &pose_file, const int start, const int end) override
  {
    frame_info_buf_.clear();

    int line_num = 0;
    std::ifstream ifs(pose_file);
    std::string csv_line;
    while (getline(ifs, csv_line))
    {
      int tmp = line_num;
      if(tmp >= start){
        if(tmp >= end && start < end)
          break;

        FrameInfo frame_info = parseLine(csv_line);
        frame_info.idx = line_num;

        std::stringstream frame_filename_str;
        frame_filename_str << std::setfill('0') << std::right << std::setw(8) << frame_info.idx << "/"; //frame_dir
        frame_info.pcd_filename = pcds_dir + frame_filename_str.str();
    
        frame_info_buf_.push_back(frame_info);
      }

      line_num++;
    }
    ROS_INFO_STREAM("Loaded: " << start << " " << end << " " << frame_info_buf_.size());
    return frame_info_buf_.size();
  }

  void loadGLIMCloud(const std::string &frame_dir, CloudType &cloud)
  {
    { //read points
      cloud.clear();
      std::ifstream in(frame_dir + POINTS_FILENAME, std::ios::binary | std::ios::ate);
      if(!in.is_open())
        return;

      std::streamsize bytes = in.tellg();
      size_t num_points = bytes / (sizeof(Eigen::Vector3f));
      cloud.reserve(num_points);

      in.seekg(0, std::ios::beg);
      bool is_dense = true;
      while(in.good() && !in.eof() && cloud.size() < num_points){
        Eigen::Vector3f eigen_p;
        in.read((char *)&eigen_p, sizeof(Eigen::Vector3f));

        if(!std::isfinite(eigen_p[0]) || !std::isfinite(eigen_p[1]) || !std::isfinite(eigen_p[2]))
          is_dense = false;

        PointType p;
        p.x = eigen_p[0];
        p.y = eigen_p[1];
        p.z = eigen_p[2];
        p.intensity = 0;
        cloud.push_back(p);
    	}
      cloud.points.shrink_to_fit();
      cloud.is_dense = is_dense;
      in.close();
    }

    { //read intensities
      std::ifstream in(frame_dir + INTENSITIES_FILENAME, std::ios::binary | std::ios::ate);
      if(!in.is_open())
        return;

      std::streamsize bytes = in.tellg();
      size_t num_points = bytes / (sizeof(float));
      
      if(num_points == cloud.size()){
        in.seekg(0, std::ios::beg);
        size_t i = 0;
        while(in.good() && !in.eof() && i < cloud.size()){
          float intensity;
          in.read((char *)&intensity, sizeof(float));
          cloud[i].intensity = intensity;
          ++i;
      	}
      }
      in.close();
    }
  }
  
  Frame loadFrame(const size_t idx) override
  {
    if(idx >= frame_info_buf_.size()){
      return Frame();
    }

    Frame frame;
    CloudType tmp_frame;
    loadGLIMCloud(frame_info_buf_[idx].pcd_filename, tmp_frame);
    frame.idx = frame_info_buf_[idx].idx;
    frame.t = frame_info_buf_[idx].t;
    frame.r = frame_info_buf_[idx].r;
    
    pcl::Indices dummy;
    pcl::removeNaNFromPointCloud(tmp_frame, *frame.frame, dummy);

    return frame;
  };
};
