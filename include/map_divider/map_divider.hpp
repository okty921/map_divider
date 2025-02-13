#ifndef MAP_DIVIDER__MAP_DIVIDER_HPP_
#define MAP_DIVIDER__MAP_DIVIDER_HPP_

#ifdef _OPENMP
#include <omp.h>
#endif

#include <memory>
#include <string>
#include <sstream>
#include <fstream>
#include <vector>
#include <cmath>
#include <limits>
#include <iostream>
#include <unordered_map>
#include <queue>
#include <boost/filesystem.hpp>

#include <octomap/octomap.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/voxel_grid.h>
#include <Eigen/Core>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>

struct AngleBin2D {
  int horiz;
  int vert; 

  bool operator==(const AngleBin2D & other) const {
    return horiz == other.horiz && vert == other.vert;
  }
};

namespace std {
  template <>
  struct hash<AngleBin2D> {
    std::size_t operator()(const AngleBin2D & bin) const {
      return std::hash<int>()(bin.horiz) ^ (std::hash<int>()(bin.vert) << 1);
    }
  };
}

class MapDivider : public rclcpp::Node
{
public:
  explicit MapDivider(const rclcpp::NodeOptions & options);

  void setSensorPosition(float x, float y, float z);
  std::vector<geometry_msgs::msg::Point> loadWaypoint(const std::string & file_name);
  void preprocessPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud);
  void occlusionFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr & selected_cloud);
  void processWaypoints();
  pcl::PointCloud<pcl::PointXYZ>::Ptr removeDuplicates(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, float leaf_size);

private:

  // Parameters
  std::string pcd_path_;
  std::string waypoints_path_;
  std::string output_pcd_name_;
  float waypoint_interpolation_step_;
  float sensor_max_range_;
  float sensor_min_elev_deg_;
  float sensor_max_elev_deg_;
  float octomap_resolution_;
  float dilation_radius_;
  float dedup_leaf_size_;
  float map_divide_step_;

  // State Variables
  std::vector<geometry_msgs::msg::Point> waypoints_data_;
  float sensor_pos_x_;
  float sensor_pos_y_;
  float sensor_pos_z_;
};

#endif  // MAP_DIVIDER__MAP_DIVIDER_HPP_
