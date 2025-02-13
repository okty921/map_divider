#include "map_divider/map_divider.hpp"
#include <fstream>
#include <sstream>
#include <algorithm>
#include <limits>
namespace fs = boost::filesystem;

double MapDivider::computeDistance(const geometry_msgs::msg::Point & p1,
                                   const geometry_msgs::msg::Point & p2)
{
  double dx = p1.x - p2.x, dy = p1.y - p2.y, dz = p1.z - p2.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

std::vector<double> MapDivider::computeCumulativeDistances(const std::vector<geometry_msgs::msg::Point> & pts)
{
  std::vector<double> cum;
  if (pts.empty()) return cum;
  cum.reserve(pts.size());
  cum.push_back(0.0);
  for (size_t i = 1; i < pts.size(); ++i)
    cum.push_back(cum.back() + computeDistance(pts[i], pts[i - 1]));
  return cum;
}

double MapDivider::getCumulativeDistanceEstimate(
    const geometry_msgs::msg::Point & pt,
    const std::vector<geometry_msgs::msg::Point> & loaded_pts,
    const std::vector<double> & loaded_cum)
{
  double bestProj = std::numeric_limits<double>::max(), bestCum = 0.0;
  bool found = false;
  for (size_t i = 0; i + 1 < loaded_pts.size(); ++i)
  {
    const auto & A = loaded_pts[i], & B = loaded_pts[i + 1];
    double vx = B.x - A.x, vy = B.y - A.y, vz = B.z - A.z;
    double len2 = vx * vx + vy * vy + vz * vz;
    if (len2 < 1e-9) continue;
    double wx = pt.x - A.x, wy = pt.y - A.y, wz = pt.z - A.z;
    double t = (wx * vx + wy * vy + wz * vz) / len2;
    if (t >= 0.0 && t <= 1.0)
    {
      geometry_msgs::msg::Point proj;
      proj.x = A.x + t * vx; proj.y = A.y + t * vy; proj.z = A.z + t * vz;
      double d = computeDistance(pt, proj);
      if (d < bestProj)
      {
        bestProj = d;
        bestCum = loaded_cum[i] + t * std::sqrt(len2);
        found = true;
      }
    }
  }
  if (!found)
  {
    double minD = std::numeric_limits<double>::max();
    size_t idx = 0;
    for (size_t i = 0; i < loaded_pts.size(); ++i)
    {
      double d = computeDistance(pt, loaded_pts[i]);
      if (d < minD) { minD = d; idx = i; }
    }
    bestCum = loaded_cum[idx];
  }
  return bestCum;
}

void MapDivider::saveOutputWaypointsCSV(
    const std::vector<geometry_msgs::msg::Point> & original_pts,
    const std::vector<double> & original_rot_x,
    const std::vector<double> & original_rot_y,
    const std::vector<double> & original_rot_z,
    const std::vector<double> & original_rot_w,
    const std::vector<DivisionWaypoint> & division_waypoints)
{
  auto cumulative = computeCumulativeDistances(original_pts);
  std::vector<std::vector<DivisionWaypoint>> segments(original_pts.size());
  for (const auto & dwp : division_waypoints)
  {
    double wpCum = getCumulativeDistanceEstimate(dwp.pt, original_pts, cumulative);
    bool inserted = false;
    for (size_t i = 0; i + 1 < original_pts.size(); ++i)
      if (wpCum >= cumulative[i] && wpCum < cumulative[i + 1])
      { segments[i].push_back(dwp); inserted = true; break; }
    if (!inserted && !original_pts.empty())
      segments.back().push_back(dwp);
  }
  struct MergedData {
    geometry_msgs::msg::Point pt;
    double rx, ry, rz, rw;
    std::string map;
  };
  std::vector<MergedData> merged;
  for (size_t i = 0; i < original_pts.size(); ++i)
  {
    merged.push_back({original_pts[i], original_rot_x[i], original_rot_y[i],
                       original_rot_z[i], original_rot_w[i], ""});
    for (const auto & dwp : segments[i])
      merged.push_back({dwp.pt, dwp.quat.x, dwp.quat.y, dwp.quat.z, dwp.quat.w, dwp.map_filename});
  }
  std::string csvFilename = output_waypoints_name_ + ".csv";
  std::ofstream ofs(csvFilename);
  if (!ofs.is_open())
  {
    RCLCPP_ERROR(get_logger(), "Failed to open CSV file: %s", csvFilename.c_str());
    return;
  }
  ofs << "id,pos_x,pos_y,pos_z,rot_x,rot_y,rot_z,rot_w,command\n";
  for (size_t i = 0; i < merged.size(); ++i)
  {
    ofs << i << ","
        << merged[i].pt.x << "," << merged[i].pt.y << "," << merged[i].pt.z << ","
        << merged[i].rx << "," << merged[i].ry << "," << merged[i].rz << "," << merged[i].rw << ",";
    if (!merged[i].map.empty())
      ofs << "map_change:" << merged[i].map;
    ofs << "\n";
  }
  ofs.close();
  RCLCPP_INFO(get_logger(), "Saved re-indexed waypoints CSV: %s", csvFilename.c_str());
}

MapDivider::MapDivider(const rclcpp::NodeOptions & options)
  : Node("map_divider", options)
{
  declare_parameter<std::string>("pcd_path", "");
  declare_parameter<std::string>("waypoints_path", "");
  declare_parameter<std::string>("output_pcd_name", "output");
  declare_parameter<std::string>("output_waypoints_name", "output");
  declare_parameter<float>("waypoint_interpolation_step", 1.0);
  declare_parameter<float>("sensor_max_range", 50.0);
  declare_parameter<float>("sensor_min_elev_deg", -30.0);
  declare_parameter<float>("sensor_max_elev_deg", 30.0);
  declare_parameter<double>("octomap_resolution", 0.2);
  declare_parameter<double>("dilation_radius", 0.2);
  declare_parameter<float>("dedup_leaf_size", 0.1);
  declare_parameter<float>("map_divide_step", 15.0);

  get_parameter("pcd_path", pcd_path_);
  get_parameter("waypoints_path", waypoints_path_);
  get_parameter("output_pcd_name", output_pcd_name_);
  get_parameter("output_waypoints_name", output_waypoints_name_);
  get_parameter("waypoint_interpolation_step", waypoint_interpolation_step_);
  get_parameter("sensor_max_range", sensor_max_range_);
  get_parameter("sensor_min_elev_deg", sensor_min_elev_deg_);
  get_parameter("sensor_max_elev_deg", sensor_max_elev_deg_);
  get_parameter("octomap_resolution", octomap_resolution_);
  get_parameter("dilation_radius", dilation_radius_);
  get_parameter("dedup_leaf_size", dedup_leaf_size_);
  get_parameter("map_divide_step", map_divide_step_);

  RCLCPP_INFO(get_logger(), "pcd_path: %s", pcd_path_.c_str());
  RCLCPP_INFO(get_logger(), "waypoints_path: %s", waypoints_path_.c_str());
  RCLCPP_INFO(get_logger(), "output_pcd_name: %s", output_pcd_name_.c_str());
  RCLCPP_INFO(get_logger(), "output_waypoints_name: %s", output_waypoints_name_.c_str());
  RCLCPP_INFO(get_logger(), "waypoint_interpolation_step: %f", waypoint_interpolation_step_);
  RCLCPP_INFO(get_logger(), "sensor_max_range: %f", sensor_max_range_);
  RCLCPP_INFO(get_logger(), "sensor_min_elev_deg: %f", sensor_min_elev_deg_);
  RCLCPP_INFO(get_logger(), "sensor_max_elev_deg: %f", sensor_max_elev_deg_);
  RCLCPP_INFO(get_logger(), "octomap_resolution: %lf", octomap_resolution_);
  RCLCPP_INFO(get_logger(), "dilation_radius: %lf", dilation_radius_);
  RCLCPP_INFO(get_logger(), "dedup_leaf_size: %f", dedup_leaf_size_);
  RCLCPP_INFO(get_logger(), "map_divide_step: %f", map_divide_step_);

  if (!fs::exists(output_pcd_name_))
  {
    if (fs::create_directories(output_pcd_name_))
      RCLCPP_INFO(get_logger(), "Created output directory: %s", output_pcd_name_.c_str());
    else
      RCLCPP_ERROR(get_logger(), "Failed to create output directory: %s", output_pcd_name_.c_str());
  }

  waypoints_data_ = loadWaypoint(waypoints_path_);
  RCLCPP_INFO(get_logger(), "Loaded %ld waypoints (after interpolation)", waypoints_data_.size());
  processWaypoints();
  rclcpp::shutdown();
}

void MapDivider::setSensorPosition(float x, float y, float z)
{
  sensor_pos_x_ = x; sensor_pos_y_ = y; sensor_pos_z_ = z;
}

std::vector<geometry_msgs::msg::Point> MapDivider::loadWaypoint(const std::string & file_name)
{
  std::ifstream file(file_name);
  if (!file.is_open())
  {
    RCLCPP_WARN(get_logger(), "Failed to open waypoint file: %s", file_name.c_str());
    return {};
  }
  std::string line;
  std::getline(file, line);
  while (std::getline(file, line))
  {
    if (line.empty()) continue;
    std::istringstream ss(line);
    std::vector<std::string> cols;
    std::string token;
    while (std::getline(ss, token, ',')) cols.push_back(token);
    if (cols.size() < 8)
    {
      RCLCPP_WARN(get_logger(), "Invalid waypoint line: %s", line.c_str());
      continue;
    }
    int id = std::stoi(cols[0]);
    original_ids_.push_back(id);
    geometry_msgs::msg::Point pt;
    pt.x = std::stod(cols[1]); pt.y = std::stod(cols[2]); pt.z = std::stod(cols[3]);
    original_pts_.push_back(pt);
    double rx = std::stod(cols[4]), ry = std::stod(cols[5]), rz = std::stod(cols[6]), rw = std::stod(cols[7]);
    original_rot_x_.push_back(rx); original_rot_y_.push_back(ry);
    original_rot_z_.push_back(rz); original_rot_w_.push_back(rw);
  }
  file.close();

  std::vector<geometry_msgs::msg::Point> interp_points;
  std::vector<geometry_msgs::msg::Quaternion> interp_quats;
  if (!original_pts_.empty())
  {
    interp_points.push_back(original_pts_.front());
    geometry_msgs::msg::Quaternion q_first;
    q_first.x = original_rot_x_.front(); q_first.y = original_rot_y_.front();
    q_first.z = original_rot_z_.front(); q_first.w = original_rot_w_.front();
    interp_quats.push_back(q_first);
    for (size_t i = 1; i < original_pts_.size(); ++i)
    {
      const auto & p0 = original_pts_[i - 1], & p1 = original_pts_[i];
      double dx = p1.x - p0.x, dy = p1.y - p0.y, dz = p1.z - p0.z;
      double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
      int steps = static_cast<int>(std::floor(dist / waypoint_interpolation_step_));
      Eigen::Quaterniond q0(original_rot_w_[i - 1], original_rot_x_[i - 1],
                            original_rot_y_[i - 1], original_rot_z_[i - 1]);
      Eigen::Quaterniond q1(original_rot_w_[i], original_rot_x_[i],
                            original_rot_y_[i], original_rot_z_[i]);
      for (int s = 1; s <= steps; ++s)
      {
        double t = (s * waypoint_interpolation_step_) / dist;
        geometry_msgs::msg::Point ipt;
        ipt.x = p0.x + t * dx; ipt.y = p0.y + t * dy; ipt.z = p0.z + t * dz;
        interp_points.push_back(ipt);
        Eigen::Quaterniond iq = q0.slerp(t, q1);
        geometry_msgs::msg::Quaternion iq_msg;
        iq_msg.x = iq.x(); iq_msg.y = iq.y(); iq_msg.z = iq.z(); iq_msg.w = iq.w();
        interp_quats.push_back(iq_msg);
      }
    }
  }
  waypoints_quat_ = interp_quats;
  return interp_points;
}

void MapDivider::preprocessPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud)
{
  Eigen::Vector3f sensor(sensor_pos_x_, sensor_pos_y_, sensor_pos_z_);
  float min_range_sq = 0.2f * 0.2f, max_range_sq = sensor_max_range_ * sensor_max_range_;
  auto filtered = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  filtered->reserve(cloud->points.size());
#ifdef _OPENMP
  #pragma omp parallel
  {
    std::vector<pcl::PointXYZ> local;
    local.reserve(1000);
    #pragma omp for nowait
    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
      const auto & pt = cloud->points[i];
      Eigen::Vector3f p(pt.x, pt.y, pt.z);
      float d_sq = (p - sensor).squaredNorm();
      if (d_sq < min_range_sq || d_sq > max_range_sq) continue;
      float horiz = std::sqrt((p.x() - sensor.x())*(p.x() - sensor.x()) +
                              (p.y() - sensor.y())*(p.y() - sensor.y()));
      float elev = std::atan2(p.z() - sensor.z(), horiz) * 180.0f / M_PI;
      if (elev < sensor_min_elev_deg_ || elev > sensor_max_elev_deg_) continue;
      local.push_back(pt);
    }
    #pragma omp critical
    { filtered->insert(filtered->end(), local.begin(), local.end()); }
  }
#else
  for (const auto & pt : cloud->points)
  {
    Eigen::Vector3f p(pt.x, pt.y, pt.z);
    float d_sq = (p - sensor).squaredNorm();
    if (d_sq < min_range_sq || d_sq > max_range_sq) continue;
    float horiz = std::sqrt((p.x() - sensor.x())*(p.x() - sensor.x()) +
                            (p.y() - sensor.y())*(p.y() - sensor.y()));
    float elev = std::atan2(p.z() - sensor.z(), horiz) * 180.0f / M_PI;
    if (elev < sensor_min_elev_deg_ || elev > sensor_max_elev_deg_) continue;
    filtered->points.push_back(pt);
  }
#endif
  cloud.swap(filtered);
}

void MapDivider::occlusionFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, 
                                 pcl::PointCloud<pcl::PointXYZ>::Ptr & selected_cloud)
{
  octomap::OcTree tree(octomap_resolution_);
  for (const auto & pt : cloud->points)
    tree.updateNode(octomap::point3d(pt.x, pt.y, pt.z), true);
  tree.updateInnerOccupancy();
  const octomap::point3d sensorPos(sensor_pos_x_, sensor_pos_y_, sensor_pos_z_);
  for (const auto & pt : cloud->points)
  {
    const octomap::point3d target(pt.x, pt.y, pt.z);
    const octomap::point3d vec = target - sensorPos;
    double distance = vec.norm();
    if (distance < 1e-9)
    { selected_cloud->points.push_back(pt); continue; }
    const octomap::point3d direction = vec * (1.0 / distance);
    octomap::point3d hit;
    if (tree.castRay(sensorPos, direction, hit, distance))
    {
      double hitDistance = (hit - sensorPos).norm();
      if (hitDistance < distance - octomap_resolution_) continue;
    }
    selected_cloud->points.push_back(pt);
  }
  pcl::KdTreeFLANN<pcl::PointXYZ> kdTree;
  kdTree.setInputCloud(selected_cloud->makeShared());
  for (const auto & pt : cloud->points)
  {
    std::vector<int> idx;
    std::vector<float> dist2;
    if (kdTree.radiusSearch(pt, dilation_radius_, idx, dist2) > 0)
      selected_cloud->points.push_back(pt);
  }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr MapDivider::removeDuplicates(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, float leaf_size)
{
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud(cloud);
  vg.setLeafSize(leaf_size, leaf_size, leaf_size);
  auto filteredCloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  vg.filter(*filteredCloud);
  return filteredCloud;
}

void MapDivider::processWaypoints()
{
  auto originalCloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path_, *originalCloud) == -1)
  {
    RCLCPP_ERROR(get_logger(), "Failed to load PCD: %s", pcd_path_.c_str());
    return;
  }
  RCLCPP_INFO(get_logger(), "Loaded map with %ld points", originalCloud->size());
  auto accumulatedCloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  accumulatedCloud->points.clear();
  geometry_msgs::msg::Point baseWaypoint;
  bool isFirstWaypoint = true;
  size_t groupIndex = 0;
  std::vector<DivisionWaypoint> divisionWaypoints;
  auto saveGroup = [this](size_t index, const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud) {
    auto deduped = removeDuplicates(cloud, dedup_leaf_size_);
    deduped->width = static_cast<uint32_t>(deduped->points.size());
    deduped->height = 1; deduped->is_dense = true;
    std::stringstream ss;
    ss << output_pcd_name_ << "/" << output_pcd_name_ << "_" << index << ".pcd";
    std::string outFile = ss.str();
    pcl::io::savePCDFileASCII(outFile, *deduped);
    RCLCPP_INFO(get_logger(), "Saved map for group %zu with %ld points to %s",
                index, deduped->size(), outFile.c_str());
  };
  for (size_t i = 0; i < waypoints_data_.size(); ++i)
  {
    const geometry_msgs::msg::Point currentWaypoint = waypoints_data_[i];
    setSensorPosition(currentWaypoint.x, currentWaypoint.y, currentWaypoint.z);
    auto cloudCopy = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(*originalCloud);
    preprocessPointCloud(cloudCopy);
    auto selectedCloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    occlusionFilter(cloudCopy, selectedCloud);
    selectedCloud = removeDuplicates(selectedCloud, dedup_leaf_size_);
    if (isFirstWaypoint)
    {
      accumulatedCloud->points.insert(accumulatedCloud->points.end(),
                                        selectedCloud->points.begin(), selectedCloud->points.end());
      baseWaypoint = currentWaypoint;
      isFirstWaypoint = false;
    }
    else
    {
      double dx = currentWaypoint.x - baseWaypoint.x;
      double dy = currentWaypoint.y - baseWaypoint.y;
      double dz = currentWaypoint.z - baseWaypoint.z;
      double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
      if (distance < map_divide_step_)
        accumulatedCloud->points.insert(accumulatedCloud->points.end(),
                                          selectedCloud->points.begin(), selectedCloud->points.end());
      else
      {
        std::stringstream ss;
        ss << output_pcd_name_ << "/" << output_pcd_name_ << "_" << groupIndex << ".pcd";
        DivisionWaypoint dwp;
        dwp.pt = currentWaypoint;
        dwp.map_filename = ss.str();
        dwp.quat = waypoints_quat_[i];
        divisionWaypoints.push_back(dwp);
        if (!accumulatedCloud->points.empty())
        {
          saveGroup(groupIndex, accumulatedCloud);
          ++groupIndex;
        }
        accumulatedCloud->points = selectedCloud->points;
        baseWaypoint = currentWaypoint;
      }
    }
  }
  if (!accumulatedCloud->points.empty())
    saveGroup(groupIndex, accumulatedCloud);
  saveOutputWaypointsCSV(
      original_pts_,
      original_rot_x_, original_rot_y_,
      original_rot_z_, original_rot_w_,
      divisionWaypoints
  );
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(MapDivider)
