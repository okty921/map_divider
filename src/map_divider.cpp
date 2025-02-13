#include "map_divider/map_divider.hpp"

constexpr float DEG2RAD = M_PI / 180.0f;
namespace fs = boost::filesystem;

MapDivider::MapDivider(const rclcpp::NodeOptions & options) : Node("map_divider", options)
{
  // Declare parameters
  declare_parameter<std::string>("pcd_path", "");
  declare_parameter<std::string>("waypoints_path", "");
  declare_parameter<std::string>("output_pcd_name", "output");
  declare_parameter<float>("waypoint_interpolation_step", 0.5);
  declare_parameter<float>("sensor_max_range", 50.0);
  declare_parameter<float>("sensor_min_elev_deg", -30.0);
  declare_parameter<float>("sensor_max_elev_deg", 30.0);
  declare_parameter<double>("octomap_resolution", 0.3);
  declare_parameter<double>("dilation_radius", 0.1);
  declare_parameter<float>("dedup_leaf_size", 0.1);
  declare_parameter<float>("map_divide_step", 10.0);

  // Retrieve parameters
  get_parameter("pcd_path", pcd_path_);
  get_parameter("waypoints_path", waypoints_path_);
  get_parameter("output_pcd_name", output_pcd_name_);
  get_parameter("waypoint_interpolation_step", waypoint_interpolation_step_);
  get_parameter("sensor_max_range", sensor_max_range_);
  get_parameter("sensor_min_elev_deg", sensor_min_elev_deg_);
  get_parameter("sensor_max_elev_deg", sensor_max_elev_deg_);
  get_parameter("octomap_resolution", octomap_resolution_);
  get_parameter("dilation_radius", dilation_radius_);
  get_parameter("dedup_leaf_size", dedup_leaf_size_);
  get_parameter("map_divide_step", map_divide_step_);

  // Log parameter values
  RCLCPP_INFO(get_logger(), "pcd_path: %s", pcd_path_.c_str());
  RCLCPP_INFO(get_logger(), "waypoints_path: %s", waypoints_path_.c_str());
  RCLCPP_INFO(get_logger(), "output_pcd_name: %s", output_pcd_name_.c_str());
  RCLCPP_INFO(get_logger(), "waypoint_interpolation_step: %f", waypoint_interpolation_step_);
  RCLCPP_INFO(get_logger(), "sensor_max_range: %f", sensor_max_range_);
  RCLCPP_INFO(get_logger(), "FOV: [%f, %f] deg", sensor_min_elev_deg_, sensor_max_elev_deg_);
  RCLCPP_INFO(get_logger(), "octomap_resolution: %lf", octomap_resolution_);
  RCLCPP_INFO(get_logger(), "dilation_radius: %lf", dilation_radius_);
  RCLCPP_INFO(get_logger(), "dedup_leaf_size: %f", dedup_leaf_size_);
  RCLCPP_INFO(get_logger(), "map_divide_step: %f", map_divide_step_);

  // Create output directory if it doesn't exist
  if (!fs::exists(output_pcd_name_))
  {
    if (fs::create_directories(output_pcd_name_))
    {
      RCLCPP_INFO(get_logger(), "Created output directory: %s", output_pcd_name_.c_str());
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "Failed to create output directory: %s", output_pcd_name_.c_str());
    }
  }

  // Load and interpolate waypoints
  waypoints_data_ = loadWaypoint(waypoints_path_);
  RCLCPP_INFO(get_logger(), "Loaded %ld waypoints (after interpolation)", waypoints_data_.size());

  // Process waypoints to extract and merge map segments
  processWaypoints();

  rclcpp::shutdown();
}

void MapDivider::setSensorPosition(float x, float y, float z)
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
    RCLCPP_INFO(get_logger(), "No waypoints loaded.");
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

  // Linear interpolation between waypoints
  std::vector<geometry_msgs::msg::Point> interpolated_waypoints;
  if (!original_waypoints.empty())
  {
    interpolated_waypoints.push_back(original_waypoints.front());
    for (size_t i = 1; i < original_waypoints.size(); i++)
    {
      const auto & p0 = original_waypoints[i-1];
      const auto & p1 = original_waypoints[i];
      float dx = p1.x - p0.x, dy = p1.y - p0.y, dz = p1.z - p0.z;
      float dist = std::sqrt(dx*dx + dy*dy + dz*dz);
      int num_steps = static_cast<int>(std::floor(dist / waypoint_interpolation_step_));
      for (int s = 1; s <= num_steps; s++)
      {
        float t = (s * waypoint_interpolation_step_) / dist;
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

void MapDivider::preprocessPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud)
{
  auto filtered_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  filtered_cloud->reserve(cloud->points.size());
  
  Eigen::Vector3f sensor(sensor_pos_x_, sensor_pos_y_, sensor_pos_z_);
  float min_range_sq = 0.2f * 0.2f;
  float max_range_sq = sensor_max_range_ * sensor_max_range_;

  #pragma omp parallel
  {
    std::vector<pcl::PointXYZ> local;
    local.reserve(1000);
    #pragma omp for nowait
    for (size_t i = 0; i < cloud->points.size(); i++) {
      const auto & pt = cloud->points[i];
      Eigen::Vector3f p(pt.x, pt.y, pt.z);
      float d_sq = (sensor - p).squaredNorm();
      if (d_sq < min_range_sq || d_sq > max_range_sq)
        continue;
      float horizontal_distance = std::sqrt((p.x() - sensor.x())*(p.x() - sensor.x()) +
                                            (p.y() - sensor.y())*(p.y() - sensor.y()));
      float elev_deg = std::atan2(p.z() - sensor.z(), horizontal_distance) * 180.0f / M_PI;
      if (elev_deg < sensor_min_elev_deg_ || elev_deg > sensor_max_elev_deg_)
        continue;
      local.push_back(pt);
    }
    #pragma omp critical
    {
      filtered_cloud->insert(filtered_cloud->end(), local.begin(), local.end());
    }
  }
  cloud.swap(filtered_cloud);
}

void MapDivider::occlusionFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, 
                                 pcl::PointCloud<pcl::PointXYZ>::Ptr & selected_cloud)
{
  octomap::OcTree tree(octomap_resolution_);
  for (const auto & point : cloud->points)
    tree.updateNode(octomap::point3d(point.x, point.y, point.z), true);
  tree.updateInnerOccupancy();

  octomap::point3d observation_point(sensor_pos_x_, sensor_pos_y_, sensor_pos_z_);
  for (const auto & point : cloud->points)
  {
    octomap::point3d target(point.x, point.y, point.z);
    octomap::point3d vec = target - observation_point;
    double distance = vec.norm();

    if (distance < std::numeric_limits<double>::epsilon()) {
      selected_cloud->points.push_back(point);
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
    selected_cloud->points.push_back(point);
  }

  pcl::KdTreeFLANN<pcl::PointXYZ> kd_tree;
  kd_tree.setInputCloud(selected_cloud->makeShared());
  for (const auto & point : cloud->points)
  {
    std::vector<int> indices;
    std::vector<float> sqr_distances;
    if (kd_tree.radiusSearch(point, dilation_radius_, indices, sqr_distances) > 0)
      selected_cloud->points.push_back(point);
  }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr MapDivider::removeDuplicates(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
                                                                   float leaf_size)
{
  pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
  voxel_filter.setInputCloud(cloud);
  voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
  auto filtered_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  voxel_filter.filter(*filtered_cloud);
  return filtered_cloud;
}

void MapDivider::processWaypoints()
{
  auto original_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path_, *original_cloud) == -1)
  {
    RCLCPP_ERROR(get_logger(), "Failed to load map from: %s", pcd_path_.c_str());
    return;
  }
  RCLCPP_INFO(get_logger(), "Loaded map with %ld points", original_cloud->size());

  auto accumulated_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  accumulated_cloud->points.clear();

  geometry_msgs::msg::Point base_wp;
  bool first_wp = true;
  size_t group_index = 0;

  for (size_t i = 0; i < waypoints_data_.size(); i++)
  {
    auto cloud_copy = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(*original_cloud);
    geometry_msgs::msg::Point current_wp = waypoints_data_[i];
    
    setSensorPosition(current_wp.x, current_wp.y, current_wp.z);
    preprocessPointCloud(cloud_copy);

    auto selected_cloud_ptr = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    occlusionFilter(cloud_copy, selected_cloud_ptr);
    selected_cloud_ptr = removeDuplicates(selected_cloud_ptr, dedup_leaf_size_);

    if (first_wp)
    {
      accumulated_cloud->points.insert(accumulated_cloud->points.end(),
                                         selected_cloud_ptr->points.begin(),
                                         selected_cloud_ptr->points.end());
      base_wp = current_wp;
      first_wp = false;
    }
    else
    {
      float dx = current_wp.x - base_wp.x;
      float dy = current_wp.y - base_wp.y;
      float dz = current_wp.z - base_wp.z;
      float dist = std::sqrt(dx * dx + dy * dy + dz * dz);

      if (dist < map_divide_step_)
      {
        accumulated_cloud->points.insert(accumulated_cloud->points.end(),
                                           selected_cloud_ptr->points.begin(),
                                           selected_cloud_ptr->points.end());
      }
      else
      {
        if (!accumulated_cloud->points.empty())
        {
          auto deduped = removeDuplicates(accumulated_cloud, dedup_leaf_size_);
          deduped->width = deduped->points.size();
          deduped->height = 1;
          deduped->is_dense = true;

          std::stringstream ss;
          ss << output_pcd_name_ << "/" << output_pcd_name_ << "_" << group_index << ".pcd";
          std::string output_file = ss.str();
          pcl::io::savePCDFileASCII(output_file, *deduped);
          RCLCPP_INFO(get_logger(), "Saved merged map for group %ld with %ld points to %s",
                      group_index, deduped->size(), output_file.c_str());
          group_index++;
        }
        accumulated_cloud->points = selected_cloud_ptr->points;
        base_wp = current_wp;
      }
    }
  }
  if (!accumulated_cloud->points.empty())
  {
    auto deduped = removeDuplicates(accumulated_cloud, dedup_leaf_size_);
    deduped->width = deduped->points.size();
    deduped->height = 1;
    deduped->is_dense = true;

    std::stringstream ss;
    ss << output_pcd_name_ << "/" << output_pcd_name_ << "_" << group_index << ".pcd";
    std::string output_file = ss.str();
    pcl::io::savePCDFileASCII(output_file, *deduped);
    RCLCPP_INFO(get_logger(), "Saved merged map for group %ld with %ld points to %s",
                group_index, deduped->size(), output_file.c_str());
  }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(MapDivider)
