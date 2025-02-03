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
  declare_parameter<float>("horiz_res_deg", 1.0);
  declare_parameter<float>("vert_res_deg", 1.0);
  declare_parameter<float>("tube_radius", 1.0);
  declare_parameter<float>("cluster_tolerance", 0.5);
  declare_parameter<int>("min_cluster_size", 1);
  declare_parameter<int>("max_cluster_size", 1000);
  declare_parameter<float>("dedup_leaf_size", 0.1);
  declare_parameter<float>("map_divide_step", 10.0);

  // Get parameters from the parameter server
  get_parameter("pcd_path", pcd_path_);
  get_parameter("waypoints_path", waypoints_path_);
  get_parameter("output_pcd_name", output_pcd_name_);
  get_parameter("waypoint_interpolation_step", waypoint_interpolation_step_);
  get_parameter("sensor_max_range", sensor_max_range_);
  get_parameter("sensor_min_elev_deg", sensor_min_elev_deg_);
  get_parameter("sensor_max_elev_deg", sensor_max_elev_deg_);
  get_parameter("horiz_res_deg", horiz_res_deg_);
  get_parameter("vert_res_deg", vert_res_deg_);
  get_parameter("tube_radius", tube_radius_);
  get_parameter("cluster_tolerance", cluster_tolerance_);
  get_parameter("min_cluster_size", min_cluster_size_);
  get_parameter("max_cluster_size", max_cluster_size_);
  get_parameter("dedup_leaf_size", dedup_leaf_size_);
  get_parameter("map_divide_step", map_divide_step_);

  // Log parameter values
  RCLCPP_INFO(get_logger(), "pcd_path: %s", pcd_path_.c_str());
  RCLCPP_INFO(get_logger(), "waypoints_path: %s", waypoints_path_.c_str());
  RCLCPP_INFO(get_logger(), "output_pcd_name: %s", output_pcd_name_.c_str());
  RCLCPP_INFO(get_logger(), "waypoint_interpolation_step: %f", waypoint_interpolation_step_);
  RCLCPP_INFO(get_logger(), "sensor_max_range: %f", sensor_max_range_);
  RCLCPP_INFO(get_logger(), "FOV: [%f, %f] deg", sensor_min_elev_deg_, sensor_max_elev_deg_);
  RCLCPP_INFO(get_logger(), "horiz_res_deg: %f, vert_res_deg: %f", horiz_res_deg_, vert_res_deg_);
  RCLCPP_INFO(get_logger(), "tube_radius: %f", tube_radius_);
  RCLCPP_INFO(get_logger(), "cluster_tolerance: %f", cluster_tolerance_);
  RCLCPP_INFO(get_logger(), "min_cluster_size: %d, max_cluster_size: %d", min_cluster_size_, max_cluster_size_);
  RCLCPP_INFO(get_logger(), "dedup_leaf_size: %f", dedup_leaf_size_);
  RCLCPP_INFO(get_logger(), "map_divide_step: %f", map_divide_step_);

  // Create the output directory if it does not exist
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

  // Load and interpolate waypoints from CSV file
  waypoints_data_ = loadWaypoint(waypoints_path_);
  RCLCPP_INFO(get_logger(), "Loaded %ld waypoints (after interpolation)", waypoints_data_.size());

  // Start processing the map based on the waypoints
  processWaypoints();

  rclcpp::shutdown();
}

// Set the sensor position based on the current waypoint
void MapDivider::setSensorPosition(float x, float y, float z)
{
  sensor_pos_x_ = x;
  sensor_pos_y_ = y;
  sensor_pos_z_ = z;
}

// Load waypoints from a CSV file and perform linear interpolation
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
  // Skip the header line (e.g., "x,y,z")
  std::getline(file, line);

  while (std::getline(file, line))
  {
    if (line.empty())
      continue;
    std::istringstream ss(line);
    std::string token;
    std::vector<std::string> row;
    while (std::getline(ss, token, ','))
    {
      row.push_back(token);
    }
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

  // Perform linear interpolation between adjacent waypoints using the specified step size
  std::vector<geometry_msgs::msg::Point> interpolated_waypoints;
  if (!original_waypoints.empty())
  {
    interpolated_waypoints.push_back(original_waypoints.front());
    for (size_t i = 1; i < original_waypoints.size(); i++)
    {
      const auto & p0 = original_waypoints[i-1];
      const auto & p1 = original_waypoints[i];
      float dx = p1.x - p0.x;
      float dy = p1.y - p0.y;
      float dz = p1.z - p0.z;
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

// Preprocess the point cloud by filtering based on sensor range and vertical field-of-view
void MapDivider::preprocessPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud)
{
  auto filtered_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  filtered_cloud->reserve(cloud->points.size());
  
  // Create a vector for the sensor position
  Eigen::Vector3f sensor(sensor_pos_x_, sensor_pos_y_, sensor_pos_z_);
  float min_range_sq = 0.2f * 0.2f;
  float max_range_sq = sensor_max_range_ * sensor_max_range_;

  // Use OpenMP to parallelize the filtering process
  #pragma omp parallel
  {
    std::vector<pcl::PointXYZ> local;
    local.reserve(1000);
    #pragma omp for nowait
    for (size_t i = 0; i < cloud->points.size(); i++) {
      const auto & pt = cloud->points[i];
      Eigen::Vector3f p(pt.x, pt.y, pt.z);
      float d_sq = (sensor - p).squaredNorm();
      // Filter out points that are too close or too far
      if (d_sq < min_range_sq || d_sq > max_range_sq)
        continue;
      // Calculate horizontal distance and elevation angle
      float horizontal_distance = std::sqrt((p.x() - sensor.x())*(p.x() - sensor.x()) +
                                            (p.y() - sensor.y())*(p.y() - sensor.y()));
      float elev_deg = std::atan2(p.z() - sensor.z(), horizontal_distance) * 180.0f / M_PI;
      // Filter out points outside the vertical field-of-view
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

// Cluster points using cylindrical bins based on sensor-relative angles
void MapDivider::clusterUsingCylindricalBins(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
                                             pcl::PointCloud<pcl::PointXYZ>::Ptr & selected_cloud,
                                             float tube_radius)
{
  // Clear the output cloud
  selected_cloud->points.clear();
  
  // Create a map to group points by angle bins
  std::unordered_map<AngleBin2D, std::vector<pcl::PointXYZ>> binMap;
  Eigen::Vector3f sensor(sensor_pos_x_, sensor_pos_y_, sensor_pos_z_);
  
  // For each point, compute its horizontal and vertical angles relative to the sensor
  for (const auto & pt : cloud->points)
  {
    Eigen::Vector3f p(pt.x, pt.y, pt.z);
    Eigen::Vector3f rel = p - sensor;
    float horiz = std::atan2(rel.y(), rel.x()) * 180.0f / M_PI;
    if (horiz < 0)
      horiz += 360.0f;
    float vert = std::atan2(rel.z(), std::sqrt(rel.x()*rel.x() + rel.y()*rel.y())) * 180.0f / M_PI;
    AngleBin2D bin { static_cast<int>(std::floor(horiz)),
                      static_cast<int>(std::floor(vert)) };
    binMap[bin].push_back(pt);
  }

  // Process each bin separately
  for (const auto & kv : binMap)
  {
    const AngleBin2D & bin = kv.first;
    const std::vector<pcl::PointXYZ> & pts = kv.second;
    if (pts.empty())
      continue;
    // Calculate the ray from the bin's center angles
    float center_horiz = (bin.horiz + 0.5f) * DEG2RAD;
    float center_vert  = (bin.vert  + 0.5f) * DEG2RAD;
    Eigen::Vector3f ray(std::cos(center_vert)*std::cos(center_horiz),
                        std::cos(center_vert)*std::sin(center_horiz),
                        std::sin(center_vert));

    // Create a candidate cloud and store the distance along the ray for each point
    pcl::PointCloud<pcl::PointXYZ>::Ptr candidate_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    std::vector<float> candidate_ts;
    for (const auto & pt : pts)
    {
      Eigen::Vector3f p(pt.x, pt.y, pt.z);
      Eigen::Vector3f v = p - sensor;
      float t = v.dot(ray);
      if (t <= 0)
        continue;
      float d_perp = (v - t * ray).norm();
      if (d_perp <= tube_radius)
      {
        candidate_cloud->points.push_back(pt);
        candidate_ts.push_back(t);
      }
    }
    if (candidate_cloud->points.empty())
      continue;

    // Perform Euclidean Cluster Extraction on the candidate cloud
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(candidate_cloud);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(cluster_tolerance_);
    ec.setMinClusterSize(min_cluster_size_);
    ec.setMaxClusterSize(max_cluster_size_);
    ec.setSearchMethod(tree);
    ec.setInputCloud(candidate_cloud);
    ec.extract(cluster_indices);

    if (cluster_indices.empty())
      continue;

    // Select the cluster that is closest to the sensor along the ray direction
    float best_cluster_min_t = std::numeric_limits<float>::max();
    std::vector<int> best_cluster_indices;
    for (const auto & indices : cluster_indices)
    {
      float min_t = std::numeric_limits<float>::max();
      for (int idx : indices.indices)
      {
        if (candidate_ts[idx] < min_t)
          min_t = candidate_ts[idx];
      }
      if (min_t < best_cluster_min_t)
      {
        best_cluster_min_t = min_t;
        best_cluster_indices = indices.indices;
      }
    }
    // Add the selected cluster's points to the output cloud
    for (int idx : best_cluster_indices)
    {
      selected_cloud->points.push_back(candidate_cloud->points[idx]);
    }
  }
  selected_cloud->width = selected_cloud->points.size();
  selected_cloud->height = 1;
  selected_cloud->is_dense = true;
}

// Remove duplicate points using a VoxelGrid filter
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

// Process each waypoint: extract, merge, and save maps based on distance traveled
void MapDivider::processWaypoints()
{
  // Load the original point cloud from a PCD file
  auto original_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path_, *original_cloud) == -1) {
    RCLCPP_ERROR(get_logger(), "Failed to load map from: %s", pcd_path_.c_str());
    return;
  }
  RCLCPP_INFO(get_logger(), "Loaded map with %ld points", original_cloud->size());

  // Create an accumulated cloud to merge point clouds from waypoints
  auto accumulated_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  accumulated_cloud->points.clear();

  geometry_msgs::msg::Point base_wp; // Base waypoint for the current group
  bool first_wp = true;
  size_t group_index = 0;

  // Process each interpolated waypoint
  for (size_t i = 0; i < waypoints_data_.size(); i++)
  {
    // Create a copy of the original cloud for this waypoint
    auto cloud_copy = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(*original_cloud);
    geometry_msgs::msg::Point current_wp = waypoints_data_[i];
    
    // Update the sensor position to the current waypoint
    setSensorPosition(current_wp.x, current_wp.y, current_wp.z);
    
    // Preprocess the point cloud (range and FOV filtering)
    preprocessPointCloud(cloud_copy);

    // Extract clusters using cylindrical bin method
    auto selected_cloud_ptr = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    clusterUsingCylindricalBins(cloud_copy, selected_cloud_ptr, tube_radius_);
    
    // Remove duplicate points from the selected cloud
    selected_cloud_ptr = removeDuplicates(selected_cloud_ptr, dedup_leaf_size_);

    if (first_wp)
    {
      // For the first waypoint, simply add all points to the accumulated cloud
      accumulated_cloud->points.insert(accumulated_cloud->points.end(),
                                         selected_cloud_ptr->points.begin(),
                                         selected_cloud_ptr->points.end());
      base_wp = current_wp;
      first_wp = false;
    }
    else
    {
      // Calculate the Euclidean distance from the base waypoint
      float dx = current_wp.x - base_wp.x;
      float dy = current_wp.y - base_wp.y;
      float dz = current_wp.z - base_wp.z;
      float dist = std::sqrt(dx * dx + dy * dy + dz * dz);

      if (dist < map_divide_step_)
      {
        // If distance is less than the specified step, merge the point clouds
        accumulated_cloud->points.insert(accumulated_cloud->points.end(),
                                           selected_cloud_ptr->points.begin(),
                                           selected_cloud_ptr->points.end());
      }
      else
      {
        // When the distance exceeds the threshold, save the accumulated cloud as a PCD file
        if (!accumulated_cloud->points.empty())
        {
          auto deduped = removeDuplicates(accumulated_cloud, dedup_leaf_size_);
          deduped->width = deduped->points.size();
          deduped->height = 1;
          deduped->is_dense = true;

          // Generate output file name in the format: output_pcd_name/output_pcd_name_0.pcd, etc.
          std::stringstream ss;
          ss << output_pcd_name_ << "/" << output_pcd_name_ << "_" << group_index << ".pcd";
          std::string output_file = ss.str();
          pcl::io::savePCDFileASCII(output_file, *deduped);
          RCLCPP_INFO(get_logger(), "Saved merged map for group %ld with %ld points to %s",
                      group_index, deduped->size(), output_file.c_str());
          group_index++;
        }
        // Start a new accumulation group with the current point cloud
        accumulated_cloud->points = selected_cloud_ptr->points;
        base_wp = current_wp;
      }
    }
  }
  // Save any remaining accumulated cloud
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
