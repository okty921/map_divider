#include "map_divider/map_divider.hpp"
namespace fs = boost::filesystem;
MapDivider::MapDivider(const rclcpp::NodeOptions & options) : Node("map_divider", options)
{
  //Declare parameters
  declare_parameter<std::string>("pcd_path", "");
  declare_parameter<std::string>("waypoints_path", "");
  declare_parameter<std::string>("output_pcd_name", "output");
  declare_parameter<double>("waypoint_interpolation_step", 0.5);
  declare_parameter<double>("sensor_max_range", 35.0);
  declare_parameter<double>("sensor_min_elev_deg", -30.0);
  declare_parameter<double>("sensor_max_elev_deg", 30.0);
  declare_parameter<int>("nb_neighbors", 18);
  declare_parameter<double>("std_ratio", 1.8);
  declare_parameter<double>("hpr_radius", 40000.0);
  declare_parameter<double>("voxel_size", 0.1);
  declare_parameter<double>("map_divide_step", 0.5);
  declare_parameter<double>("save_voxel_size", 0.1);
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
  get_parameter("hpr_radius", hpr_radius_);
  get_parameter("voxel_size", voxel_size_);
  get_parameter("map_divide_step", map_divide_step_);
  get_parameter("voxel_size", save_voxel_size_);
  //Log parameter values
  RCLCPP_INFO(get_logger(), "pcd_path: %s", pcd_path_.c_str());
  RCLCPP_INFO(get_logger(), "waypoints_path: %s", waypoints_path_.c_str());
  RCLCPP_INFO(get_logger(), "output_pcd_name: %s", output_pcd_name_.c_str());
  RCLCPP_INFO(get_logger(), "waypoint_interpolation_step: %f", waypoint_interpolation_step_);
  RCLCPP_INFO(get_logger(), "sensor_max_range: %f", sensor_max_range_);
  RCLCPP_INFO(get_logger(), "FOV: [%f, %f] deg", sensor_min_elev_deg_, sensor_max_elev_deg_);
  RCLCPP_INFO(get_logger(), "nb_neighbors: %ld", nb_neighbors_);
  RCLCPP_INFO(get_logger(), "std_ratio: %f", std_ratio_);
  RCLCPP_INFO(get_logger(), "hpr_radius: %f", hpr_radius_);
  RCLCPP_INFO(get_logger(), "voxel_size: %f", voxel_size_);
  RCLCPP_INFO(get_logger(), "map_divide_step: %f", map_divide_step_);
  RCLCPP_INFO(get_logger(), "voxel_size: %f", save_voxel_size_);
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
open3d::geometry::PointCloud MapDivider::preprocessPointCloud(std::shared_ptr<open3d::geometry::PointCloud> cloud)
{
  open3d::geometry::PointCloud filtered_cloud;
  filtered_cloud.points_.reserve(cloud->points_.size());
  Eigen::Vector3f sensor(sensor_pos_x_, sensor_pos_y_, sensor_pos_z_);
  double min_range_sq = 0.04;
  double max_range_sq = sensor_max_range_ * sensor_max_range_;
  for (const auto &pt : cloud->points_)
  {
    Eigen::Vector3f p(pt(0), pt(1), pt(2));
    double d_sq = (sensor - p).squaredNorm();
    if (d_sq < min_range_sq || d_sq > max_range_sq)
    continue;
    double horizontal_distance = std::sqrt((p(0) - sensor.x())*(p(0) - sensor.x())+
                                           (p(1) - sensor.y())*(p(1) - sensor.y()));
    double elev_deg = std::atan2(p(2) - sensor.z(), horizontal_distance) * 180.0 / M_PI;
    if (elev_deg < sensor_min_elev_deg_ || elev_deg > sensor_max_elev_deg_)
    continue;
    filtered_cloud.points_.push_back(pt);
  }
  return filtered_cloud;
}
void MapDivider::processWaypoints()
{
  clock_t start = clock();

  open3d::geometry::PointCloud original_cloud;
    if (open3d::io::ReadPointCloud(pcd_path_, original_cloud)) {
    RCLCPP_INFO(get_logger(), "Loaded map with %ld points", original_cloud.points_.size());
  } else {
    RCLCPP_ERROR(get_logger(), "Failed to load map from: %s", pcd_path_.c_str());
  }
  auto result_ptr = std::make_shared<open3d::geometry::PointCloud>();
  geometry_msgs::msg::Point base_wp;
  bool first = true;
  size_t group_index = 0;
  for (const auto &wp : waypoints_data_)
  {
    Eigen::Vector3d current_pos(wp.x, wp.y, wp.z);
    setSensorPosition(wp.x, wp.y, wp.z);
    auto [mesh, indices] = original_cloud.HiddenPointRemoval(current_pos, hpr_radius_);
    auto hpr_filtered_cloud = original_cloud.SelectByIndex(indices);
    auto preprocessed_cloud = preprocessPointCloud(hpr_filtered_cloud);
    auto [statistical_filtered_cloud, _2] = preprocessed_cloud.RemoveStatisticalOutliers(nb_neighbors_, std_ratio_);
    auto voxeldown_ptr = statistical_filtered_cloud->VoxelDownSample(voxel_size_);
    if (first)
    {
      result_ptr->points_.insert(result_ptr->points_.end(), voxeldown_ptr->points_.begin(), voxeldown_ptr->points_.end());
      base_wp = wp;
      first = false;
    }
    else
    {
      double dx = wp.x - base_wp.x, dy = wp.y - base_wp.y, dz = wp.z - base_wp.z;
      double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
      if (dist < map_divide_step_)
      {
        result_ptr->points_.insert(result_ptr->points_.end(), voxeldown_ptr->points_.begin(), voxeldown_ptr->points_.end());
      }
      else
      {
        if (!result_ptr->points_.empty())
        {
          auto saved_ptr = result_ptr->VoxelDownSample(save_voxel_size_);

          // saved_ptr->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(0.02, 30));
          // saved_ptr->OrientNormalsConsistentTangentPlane(100);
          // std::vector<double> nn = saved_ptr->ComputeNearestNeighborDistance();
          // double avg_dist = std::accumulate(nn.begin(), nn.end(), 0.0) / static_cast<double>(nn.size());
          // std::vector<double> radii = {1.5 * avg_dist, 2.0 * avg_dist, 3.0 * avg_dist};
          // auto mesh = open3d::geometry::TriangleMesh::CreateFromPointCloudBallPivoting(*saved_ptr, radii);
          // mesh->ComputeVertexNormals();
          // std::stringstream ssm;
          // ssm << output_pcd_name_ << "/" << output_pcd_name_ << "_" << group_index << ".ply";
          // std::string output_file_mesh = ssm.str();
          // open3d::io::WriteTriangleMesh(output_file_mesh, *mesh);

          std::stringstream ss;
          ss << output_pcd_name_ << "/" << output_pcd_name_ << "_" << group_index << ".pcd";
          std::string output_file_ = ss.str();
          open3d::io::WritePointCloud(output_file_, *saved_ptr);
          RCLCPP_INFO(get_logger(), "Save merged map for group %ld with %ld points to %s",
                      group_index, saved_ptr->points_.size(), output_file_.c_str());

          double fitness_score = 0;
          std::vector<int> indices;
          std::vector<double> dist;
          open3d::geometry::KDTreeFlann tree(original_cloud);
          for (size_t i=0; i < saved_ptr->points_.size(); i++) {
            const Eigen::Vector3d &pt = saved_ptr->points_[i];
            int k = tree.SearchKNN(pt, 1, indices, dist);
            if (k>0) {
              fitness_score += sqrt(dist[0]);
            } else {
              RCLCPP_ERROR(get_logger(), "error");
            }
          }
          RCLCPP_INFO(get_logger(), "No %ld fitness_score is %f", group_index, fitness_score/saved_ptr->points_.size()*1000);
          group_index++;
        }
        result_ptr = voxeldown_ptr;
        base_wp = wp;
      }
    }
  }
  if (!result_ptr->points_.empty())
  {
    auto saved_ptr = result_ptr->VoxelDownSample(save_voxel_size_);

    // saved_ptr->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(0.02, 30));
    // saved_ptr->OrientNormalsConsistentTangentPlane(100);
    // std::vector<double> nn = saved_ptr->ComputeNearestNeighborDistance();
    // double avg_dist = std::accumulate(nn.begin(), nn.end(), 0.0) / static_cast<double>(nn.size());
    // std::vector<double> radii = {1.5 * avg_dist, 2.0 * avg_dist, 3.0 * avg_dist};
    // auto mesh = open3d::geometry::TriangleMesh::CreateFromPointCloudBallPivoting(*saved_ptr, radii);
    // mesh->ComputeVertexNormals();
    // std::stringstream ssm;
    // ssm << output_pcd_name_ << "/" << output_pcd_name_ << "_" << group_index << ".ply";
    // std::string output_file_mesh = ssm.str();
    // open3d::io::WriteTriangleMesh(output_file_mesh, *mesh);

    std::stringstream ss;
    std::string output_file_ = ss.str();
    ss << output_pcd_name_ << "/" << output_pcd_name_ << "_" << group_index << ".pcd";
    open3d::io::WritePointCloud(ss.str(), *saved_ptr);
    RCLCPP_INFO(get_logger(), "Save merged map for group %ld with %ld points to %s",
                group_index, saved_ptr->points_.size(), output_file_.c_str());

    double fitness_score = 0;
    std::vector<int> indices;
    std::vector<double> dist;
    open3d::geometry::KDTreeFlann tree(original_cloud);
    for (size_t i=0; i < saved_ptr->points_.size(); i++) {
      const Eigen::Vector3d &pt = saved_ptr->points_[i];
      int k = tree.SearchKNN(pt, 1, indices, dist);
      if (k>0) {
        fitness_score += sqrt(dist[0]);
      }
    }
    RCLCPP_INFO(get_logger(), "No %ld fitness_score is %f", group_index, fitness_score/saved_ptr->points_.size()*1000);

    clock_t end = clock();
    double process_time;
    process_time = static_cast<double>(end - start) / CLOCKS_PER_SEC;
    RCLCPP_INFO(get_logger(), "Process_time is %fsec", process_time);          
  }
}
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(MapDivider)