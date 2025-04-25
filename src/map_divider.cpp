#include "map_divider/map_divider.hpp"

namespace fs = boost::filesystem;

MapDivider::MapDivider(const rclcpp::NodeOptions & options) : Node("map_divider", options)
{
  //Declare parameters
  declare_parameter<std::string>("pcd_path", "");
  declare_parameter<std::string>("waypoints_path", "");
  declare_parameter<std::string>("output_pcd_name", "output");
  declare_parameter<double>("waypoint_interpolation_step", 0.5);
  declare_parameter<double>("sensor_max_range", 50.0);
  declare_parameter<double>("sensor_min_elev_deg", -30.0);
  declare_parameter<double>("sensor_max_elev_deg", 30.0);
  declare_parameter<int>("nb_neighbors", 12);
  declare_parameter<double>("std_ratio", 1.8);
  declare_parameter<double>("octomap_resolution", 0.3);
  declare_parameter<double>("dilation_radius", 0.1);
  declare_parameter<double>("voxel_size", 0.1);
  declare_parameter<double>("map_divide_step", 0.5);

  //Retrive parameters
  get_parameter("pcd_path", pcd_path_);
  get_parameter("waypoints_path", waypoints_path_);
  get_parameter("output_pcd_name", output_pcd_name_);
  get_parameter("waypoint_interpolation_step", waypoint_interpolation_step_);
  get_parameter("sensor_max_range", sensor_max_range_);
  get_parameter("sensor_max_elev_deg", sensor_max_elev_deg_);
  get_parameter("sensor_min_elev_deg", sensor_min_elev_deg_);
  get_parameter("nb_neighbors", nb_neighbors_);
  get_parameter("std_ratio", std_ratio_);
  get_parameter("octomap_resolution", octomap_resolution_);
  get_parameter("dilation_radius", dilation_radius_);
  get_parameter("voxel_size", voxel_size_);
  get_parameter("map_divide_step", map_divide_step_);

  //Log parameter values
  RCLCPP_INFO(get_logger(), "pcd_path: %s", pcd_path_.c_str());
  RCLCPP_INFO(get_logger(), "waypoints_path: %s", waypoints_path_.c_str());
  RCLCPP_INFO(get_logger(), "output_pcd_name: %s", output_pcd_name_.c_str());
  RCLCPP_INFO(get_logger(), "waypoint_interpolation_step: %f", waypoint_interpolation_step_);
  RCLCPP_INFO(get_logger(), "sensor_max_range: %f", sensor_max_range_);
  RCLCPP_INFO(get_logger(), "FOV: [%f, %f] deg", sensor_min_elev_deg_, sensor_max_elev_deg_);
  RCLCPP_INFO(get_logger(), "nb_neighbors: %ld", nb_neighbors_);
  RCLCPP_INFO(get_logger(), "std_ratio: %f", std_ratio_);
  RCLCPP_INFO(get_logger(), "octomap_resolution: %f", octomap_resolution_);
  RCLCPP_INFO(get_logger(), "dilation_radius: %f", dilation_radius_);
  RCLCPP_INFO(get_logger(), "voxel_size: %f", voxel_size_);
  RCLCPP_INFO(get_logger(), "map_divide_step: %f", map_divide_step_);

  //Create output directory if it doesn't exist
  if (!fs::exists(output_pcd_name_))
  {
    if (fs::create_directories(output_pcd_name_))
    {
      RCLCPP_INFO(get_logger(), "Created output directory: %s", output_pcd_name_.c_str());
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "Faild to create output directiry: %s", output_pcd_name_.c_str());
    }
  }

  //Load and interpolate waypoints
  waypoints_data_ = loadWaypoint(waypoints_path_);
  RCLCPP_INFO(get_logger(), "Loaded %ld waypoints (after interpolation)", waypoints_data_.size());

  //Process waypoints to extract and merge map segments
  processWaypoints();

  rclcpp::shutdown();
}

void MapDivider::setSensorPosition(double x, double y, double z)
{
  sensor_pos_x_ = x;
  sensor_pos_y_ = y;
  sensor_pos_z_ = z;
}

std::vector<geometry_msgs::msg::Point> MapDivider::loadWaypoint(const std::string & file_name)
{
  std::ifstream file(file_name);
  if (!file.is_open())
  {
    RCLCPP_WARN(get_logger(), "No waypoints loaded.");
    return {};
  }

  std::vector<geometry_msgs::msg::Point> original_waypoints;
  std::string line;
  std::getline(file, line);

  while (std::getline(file, line))
  {
    if (line.empty())
      continue;
    std::istringstream ss(line);
    std::string token;
    std::vector<std::string> row;
    while (std::getline(ss, token, ','))
      row.push_back(token);
    geometry_msgs::msg::Point pt;
    try {
      pt.x = std::stod(row[1]);
      pt.y = std::stod(row[2]);
      pt.z = std::stod(row[3]);
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "Conversion error for line: %s", line.c_str());
      continue;
    }
    original_waypoints.push_back(pt);
  }

  //Linear interpolation between waypoints
  std::vector<geometry_msgs::msg::Point> interpolated_waypoints;
  if (!original_waypoints.empty())
  {
    interpolated_waypoints.push_back(original_waypoints.front());
    for (size_t i=1; i < original_waypoints.size(); i++)
    {
      const auto & p0 = original_waypoints[i-1];
      const auto & p1 = original_waypoints[i];
      double dx = p1.x - p0.x, dy = p1.y - p0.y, dz = p1.z - p0.z;
      double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
      int num_step = static_cast<int>(std::floor(dist / waypoint_interpolation_step_));
      for (int s=1; s<=num_step; s++)
      {
        double t = (s * waypoint_interpolation_step_) / dist;
        geometry_msgs::msg::Point p;
        p.x = p0.x + t * dx;
        p.y = p0.y + t * dy;
        p.z = p0.z + t * dz;
        interpolated_waypoints.push_back(p);
      }
    }
  }
  return interpolated_waypoints;
}

void MapDivider::preprocessPointCloud(std::shared_ptr<open3d::geometry::PointCloud> & cloud)
{
  open3d::geometry::PointCloud filtered_cloud;
  filtered_cloud.points_.reserve(cloud->points_.size());

  Eigen::Vector3f sensor(sensor_pos_x_, sensor_pos_y_, sensor_pos_z_);
  double min_range_sq = 0.2f * 0.2f;
  double max_range_sq = sensor_max_range_ * sensor_max_range_;

  #pragma omp parallel
  {
    #pragma omp for nowait
    for (size_t i=0; i < cloud->points_.size(); i++) {
      const auto & pt = cloud->points_[i];
      Eigen::Vector3f p(pt(0), pt(1), pt(2));
      double d_sq = (sensor - p).squaredNorm();
      if (d_sq < min_range_sq || d_sq > max_range_sq) 
        continue;
      double horizontal_distance = std::sqrt((p(0) - sensor.x())*(p(0) - sensor.x())+
                                            (p(1) - sensor.y())*(p(1) - sensor.y()));
      double elev_deg = std::atan2(p(2) - sensor.z(), horizontal_distance) * 180.0f / M_PI;
      if (elev_deg < sensor_min_elev_deg_ || elev_deg > sensor_max_elev_deg_)
        continue;
      filtered_cloud.points_.push_back(pt);
    }
  }

  auto filtered_cloud_ptr = std::make_shared<open3d::geometry::PointCloud>(filtered_cloud);
  cloud.swap(filtered_cloud_ptr);
}

void MapDivider::StatisticalFilter(std::shared_ptr<open3d::geometry::PointCloud> & cloud)
{
  open3d::geometry::PointCloud statistical_filterd_cloud;
  statistical_filterd_cloud = *cloud;

  statistical_filterd_cloud.RemoveStatisticalOutliers(nb_neighbors_, std_ratio_);

  auto statistical_filterd_cloud_ptr = std::make_shared<open3d::geometry::PointCloud>(statistical_filterd_cloud);
  cloud.swap(statistical_filterd_cloud_ptr);
}

void MapDivider::occlusionFilter(const std::shared_ptr<open3d::geometry::PointCloud> & cloud,
                                 std::shared_ptr<open3d::geometry::PointCloud> & selected_cloud)
{
  octomap::OcTree tree(octomap_resolution_);

  octomap::point3d bbx_min(-200, -200, -20);
  octomap::point3d bbx_max(200, 200, 20);

  tree.setBBXMin(bbx_min);
  tree.setBBXMax(bbx_max);
  tree.useBBXLimit(true);

  for (const auto & point : cloud->points_)
    tree.updateNode(octomap::point3d(point(0), point(1), point(2)), true);
  tree.updateInnerOccupancy();

  octomap::point3d observation_point(sensor_pos_x_, sensor_pos_y_, sensor_pos_z_);
  for (const auto & point : cloud->points_)
  {
    octomap::point3d target(point(0), point(1), point(2));
    octomap::point3d vec = target - observation_point;
    double distance = vec.norm();

    if (distance < std::numeric_limits<double>::epsilon()) {
      selected_cloud->points_.push_back(point);
      continue;
    }

    octomap::point3d direction = vec * (1.0 / distance);
    octomap::point3d hit_point;
    bool hit_found = tree.castRay(observation_point, direction, hit_point, distance);
    if (hit_found) { 
      double hit_distance = (hit_point - observation_point).norm();
      if (hit_distance < distance - octomap_resolution_)
        continue;
    }
    selected_cloud->points_.push_back(point);
  }

  open3d::geometry::KDTreeFlann kd_tree(*selected_cloud);
  for (const auto & point : cloud->points_)
  {
    std::vector<int> indices;
    std::vector<double> sqr_distance;
    if (kd_tree.SearchRadius(point, dilation_radius_,indices, sqr_distance) > 0)
      selected_cloud->points_.push_back(point);
  }
}

std::shared_ptr<open3d::geometry::PointCloud> MapDivider::VoxelDownSample(std::shared_ptr<open3d::geometry::PointCloud> & cloud)
{
  std::shared_ptr<open3d::geometry::PointCloud> downsampled_cloud;
  auto ptr_cloud = open3d::geometry::PointCloud(*cloud);
  downsampled_cloud = ptr_cloud.VoxelDownSample(voxel_size_);
  return downsampled_cloud;
}

void MapDivider::processWaypoints()
{
  open3d::geometry::PointCloud original_cloud;
  if (open3d::io::ReadPointCloud(pcd_path_, original_cloud)) {
    RCLCPP_INFO(get_logger(), "Loaded map with %ld points", original_cloud.points_.size());
  } else {
    RCLCPP_ERROR(get_logger(), "Failed to load map from: %s", pcd_path_.c_str());
  }

  auto accmulated_cloud = std::make_shared<open3d::geometry::PointCloud>();
  accmulated_cloud->points_.clear();

  geometry_msgs::msg::Point base_wp;
  bool first_wp = true;
  size_t group_index = 0;

  for (size_t i=0; i < waypoints_data_.size(); i++)
  {
    auto cloud_copy = std::make_shared<open3d::geometry::PointCloud>(original_cloud);
    geometry_msgs::msg::Point current_wp = waypoints_data_[i];

    setSensorPosition(current_wp.x, current_wp.y, current_wp.z);
    preprocessPointCloud(cloud_copy);
    StatisticalFilter(cloud_copy);

    auto selected_cloud_ptr = std::make_shared<open3d::geometry::PointCloud>();
    occlusionFilter(cloud_copy, selected_cloud_ptr);
    std::shared_ptr<open3d::geometry::PointCloud> downsampled_cloud;
    downsampled_cloud = VoxelDownSample(selected_cloud_ptr);

    if(first_wp)
    {
      accmulated_cloud->points_.insert(accmulated_cloud->points_.end(), 
                                        downsampled_cloud->points_.begin(),
                                        downsampled_cloud->points_.end());
      base_wp = current_wp;
      first_wp = false;
    }
    else
    {
      double dx = current_wp.x - base_wp.x;
      double dy = current_wp.y - base_wp.y;
      double dz = current_wp.z - base_wp.z;
      double dist = std::sqrt(dx * dx + dy * dy + dz* dz);

      if (dist < map_divide_step_)
      {
        accmulated_cloud->points_.insert(accmulated_cloud->points_.end(),
                                          downsampled_cloud->points_.begin(),
                                          downsampled_cloud->points_.end());
      }
      else
      {
        if (!accmulated_cloud->points_.empty())
        {
          auto vds_cloud = VoxelDownSample(accmulated_cloud);
          
          std::stringstream ss;
          ss << output_pcd_name_ << "/" << output_pcd_name_ << "_" << group_index << ".pcd";
          std::string output_file_ = ss.str();
          open3d::io::WritePointCloud(output_file_, *vds_cloud);
          RCLCPP_INFO(get_logger(), "Save merged map for group %ld with %ld points to %s",
                      group_index, vds_cloud->points_.size(), output_file_.c_str());
          group_index++;
        }
        accmulated_cloud->points_ = selected_cloud_ptr->points_;
        base_wp = current_wp;
      }
    }
  }
  if (!accmulated_cloud->points_.empty())
  {
    auto vds_cloud = VoxelDownSample(accmulated_cloud);
          
    std::stringstream ss;
    ss << output_pcd_name_ << "/" << output_pcd_name_ << "_" << group_index << ".pcd";
    std::string output_file_ = ss.str();
    open3d::io::WritePointCloud(output_file_, *vds_cloud);
    RCLCPP_INFO(get_logger(), "Save merged map for group %ld with %ld points to %s",
                group_index, vds_cloud->points_.size(), output_file_.c_str());

  }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(MapDivider)

