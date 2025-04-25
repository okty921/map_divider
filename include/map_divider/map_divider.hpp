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

#include <open3d/Open3D.h>

#include <octomap/octomap.h>
#include <Eigen/Core>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>

class MapDivider : public rclcpp::Node
{
public:
  explicit MapDivider(const rclcpp::NodeOptions & options);

  void setSensorPosition(double x, double y, double z);
  std::vector<geometry_msgs::msg::Point> loadWaypoint(const std::string & file_name);
  void preprocessPointCloud(std::shared_ptr<open3d::geometry::PointCloud> & cloud);
  void occlusionFilter(const std::shared_ptr<open3d::geometry::PointCloud> & cloud, std::shared_ptr<open3d::geometry::PointCloud> & selected_cloud);
  void processWaypoints();
  void StatisticalFilter(std::shared_ptr<open3d::geometry::PointCloud> & cloud);
  std::shared_ptr<open3d::geometry::PointCloud> VoxelDownSample(std::shared_ptr<open3d::geometry::PointCloud> & cloud);
private:

  // Parameters
  std::string pcd_path_;
  std::string waypoints_path_;
  std::string output_pcd_name_;
  double waypoint_interpolation_step_;
  double sensor_max_range_;
  double sensor_min_elev_deg_;
  double sensor_max_elev_deg_;
  size_t nb_neighbors_;
  double std_ratio_;
  double octomap_resolution_;
  double dilation_radius_;
  double voxel_size_;
  double map_divide_step_;

  // State Variables
  std::vector<geometry_msgs::msg::Point> waypoints_data_;
  double sensor_pos_x_;
  double sensor_pos_y_;
  double sensor_pos_z_;
};

#endif  // MAP_DIVIDER__MAP_DIVIDER_HPP_
