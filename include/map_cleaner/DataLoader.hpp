#pragma once

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
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
    cloud.reserve(200000);
    std::fstream in(file_name, std::ios::in | std::ios::binary);
    while(in.good() && !in.eof()){
      PointType p;
		  in.read((char *)&p.x, sizeof(float));
		  in.read((char *)&p.y, sizeof(float));
		  in.read((char *)&p.z, sizeof(float));
		  in.read((char *)&p.intensity, sizeof(float));
		  cloud.push_back(p);
  	}
	  in.close();
  }
  
  Frame loadFrame(const size_t idx) override
  {
    if(idx >= frame_info_buf_.size()){
      return Frame();
    }

    Frame frame;
    loadKittiCloud(frame_info_buf_[idx].pcd_filename, *frame.frame);
    frame.idx = frame_info_buf_[idx].idx;
    frame.t = frame_info_buf_[idx].t;
    frame.r = frame_info_buf_[idx].r;
    
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
    pcl::io::loadPCDFile<PointType>(frame_info_buf_[idx].pcd_filename, *frame.frame);
    frame.idx = frame_info_buf_[idx].idx;
    frame.t = frame_info_buf_[idx].t;
    frame.r = frame_info_buf_[idx].r;
    
    return frame;
  };
};