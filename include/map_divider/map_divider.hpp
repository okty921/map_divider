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
#include <boost/filesystem.hpp>
#include <octomap/octomap.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

struct DivisionWaypoint {
  geometry_msgs::msg::Point pt;
  std::string map_filename;
  geometry_msgs::msg::Quaternion quat;
};

class MapDivider : public rclcpp::Node
{
public:
  explicit MapDivider(const rclcpp::NodeOptions & options);
  void setSensorPosition(float x, float y, float z);
  void processWaypoints();
  pcl::PointCloud<pcl::PointXYZ>::Ptr removeDuplicates(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, float leaf_size);

private:
  std::vector<geometry_msgs::msg::Point> loadWaypoint(const std::string & file_name);
  void preprocessPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud);
  void occlusionFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, 
                       pcl::PointCloud<pcl::PointXYZ>::Ptr & selected_cloud);
  double computeDistance(const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2);
  std::vector<double> computeCumulativeDistances(const std::vector<geometry_msgs::msg::Point> & pts);
  double getCumulativeDistanceEstimate(const geometry_msgs::msg::Point & pt,
                                       const std::vector<geometry_msgs::msg::Point> & loaded_pts,
                                       const std::vector<double> & loaded_cum);
  void saveOutputWaypointsCSV(const std::vector<geometry_msgs::msg::Point> & original_pts,
                              const std::vector<double> & original_rot_x,
                              const std::vector<double> & original_rot_y,
                              const std::vector<double> & original_rot_z,
                              const std::vector<double> & original_rot_w,
                              const std::vector<DivisionWaypoint> & division_waypoints);
  
  std::string pcd_path_;
  std::string waypoints_path_;
  std::string output_pcd_name_;
  std::string output_waypoints_name_;
  float waypoint_interpolation_step_;
  float sensor_max_range_;
  float sensor_min_elev_deg_;
  float sensor_max_elev_deg_;
  float octomap_resolution_;
  float dilation_radius_;
  float dedup_leaf_size_;
  float map_divide_step_;

  std::vector<int> original_ids_;
  std::vector<geometry_msgs::msg::Point> original_pts_;
  std::vector<double> original_rot_x_;
  std::vector<double> original_rot_y_;
  std::vector<double> original_rot_z_;
  std::vector<double> original_rot_w_;

  std::vector<geometry_msgs::msg::Point> waypoints_data_;
  std::vector<geometry_msgs::msg::Quaternion> waypoints_quat_;

  float sensor_pos_x_;
  float sensor_pos_y_;
  float sensor_pos_z_;
};

#endif  // MAP_DIVIDER__MAP_DIVIDER_HPP_
