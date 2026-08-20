#include "plan_env/grid_map.h"
#include <cmath>
#include <limits>
#include <string>
#include <tf/transform_broadcaster.h>

GridMap::~GridMap()
{
  // Tear ROS callbacks down before message filters and the TF broadcaster.
  // Relying on global ROS destruction order caused boost::lock_error during
  // roslaunch Ctrl-C after long-running simulations.
  occ_timer_.stop();
  vis_timer_.stop();
  lidar_pose_sub_.shutdown();
  sliding_map_frame_sub_.shutdown();
  cloud_sub_.shutdown();
  if (depth_sub_)
    depth_sub_->unsubscribe();
  if (depth_pose_sub_)
    depth_pose_sub_->unsubscribe();
  sync_image_pose_.reset();
  depth_sub_.reset();
  depth_pose_sub_.reset();
  map_pub_.shutdown();
  map_inf_pub_.shutdown();
  sliding_map_bbox_pub_.shutdown();
  unknown_pub_.shutdown();
  depth_cloud_pub_.shutdown();
  extrinsic_pose_pub_.shutdown();
  tf_broadcaster_.reset();
}

void GridMap::initMap(ros::NodeHandle &nh)
{
  node_ = nh;

  /* get parameter */
  double x_size, y_size, z_size;
  node_.param("grid_map/resolution", mp_.resolution_, -1.0);
  node_.param("grid_map/sliding_map_size_x", x_size, -1.0);
  node_.param("grid_map/sliding_map_size_y", y_size, -1.0);
  node_.param("grid_map/sliding_map_size_z", z_size, -1.0);
  node_.param("grid_map/local_update_range_x", mp_.local_update_range_(0), x_size/2.0);
  node_.param("grid_map/local_update_range_y", mp_.local_update_range_(1), y_size/2.0);
  node_.param("grid_map/local_update_range_z", mp_.local_update_range_(2), z_size/2.0);
  
  node_.param("grid_map/obstacles_inflation_z_up", mp_.obstacles_inflation_z_up, -1.0);
  node_.param("grid_map/obstacles_inflation_z_down", mp_.obstacles_inflation_z_down, -1.0);
  node_.param("grid_map/double_cylinder_radius", mp_.double_cylinder_radius_, -1.0);
  node_.param("grid_map/double_cylinder_offset", mp_.double_cylinder_offset_, 0.0);
  node_.param("grid_map/collision_check_enabled", mp_.collision_check_enabled_, true);
  node_.param("grid_map/map_sliding_en", mp_.map_sliding_en_, true);
  node_.param("grid_map/map_sliding_thresh", mp_.map_sliding_thresh_, mp_.resolution_);

  node_.param("grid_map/fx", mp_.fx_, -1.0);
  node_.param("grid_map/fy", mp_.fy_, -1.0);
  node_.param("grid_map/cx", mp_.cx_, -1.0);
  node_.param("grid_map/cy", mp_.cy_, -1.0);

  node_.param("grid_map/depth_filter_maxdist", mp_.depth_filter_maxdist_, -1.0);
  node_.param("grid_map/depth_filter_mindist", mp_.depth_filter_mindist_, -1.0);
  node_.param("grid_map/depth_filter_margin", mp_.depth_filter_margin_, -1);
  node_.param("grid_map/k_depth_scaling_factor", mp_.k_depth_scaling_factor_, -1.0);
  node_.param("grid_map/skip_pixel", mp_.skip_pixel_, -1);

  node_.param("grid_map/p_hit", mp_.p_hit_, -1.0);
  node_.param("grid_map/p_miss", mp_.p_miss_, -1.0);
  node_.param("grid_map/p_min", mp_.p_min_, -1.0);
  node_.param("grid_map/p_max", mp_.p_max_, -1.0);
  node_.param("grid_map/p_occ", mp_.p_occ_, -1.0);
  node_.param("grid_map/max_ray_length", mp_.max_ray_length_, -0.1);

  node_.param("grid_map/vis_height", mp_.vis_height_, 0.3);
  node_.param("grid_map/show_occ_time", mp_.show_occ_time_, false);

  node_.param("grid_map/frame_id", mp_.frame_id_, string("world"));
  node_.param("grid_map/sliding_map_frame_id", mp_.sliding_map_frame_id_, string("sliding_map"));
  node_.param("grid_map/ground_height", mp_.ground_height_, 0.0);

  node_.param("grid_map/sensor_type", mp_.sensor_type_, string("lidar"));
  node_.param("grid_map/cloud_is_world", mp_.cloud_is_world_, true);
  node_.param("grid_map/need_extrinsic", mp_.need_extrinsic_, true);

  mp_.lidar_extrinsic_ <<
      1.0, 0.0, 0.0, -0.01100,
      0.0, 1.0, 0.0, -0.02329,
      0.0, 0.0, 1.0,  0.04412,
      0.0, 0.0, 0.0,  1.00000;

  mp_.depth_extrinsic_ <<
      0.0,  0.707107, 0.707107, -0.15170,
     -1.0,  0.000000, 0.000000,  0.00000,
      0.0, -0.707107, 0.707107,  0.07510,
      0.0,  0.000000, 0.000000,  1.00000;
      
  if (mp_.sensor_type_ != "lidar" && mp_.sensor_type_ != "depth")
  {
    ROS_ERROR_STREAM("[GridMap] invalid grid_map/sensor_type: " << mp_.sensor_type_
                                                                << ", fallback to lidar.");
    mp_.sensor_type_ = "lidar";
  }

  mp_.resolution_inv_ = 1 / mp_.resolution_;
  mp_.map_origin_ = Eigen::Vector3d(-x_size / 2.0, -y_size / 2.0, mp_.ground_height_);
  mp_.map_size_ = Eigen::Vector3d(x_size, y_size, z_size);

  mp_.prob_hit_log_ = logit(mp_.p_hit_);
  mp_.prob_miss_log_ = logit(mp_.p_miss_);
  mp_.clamp_min_log_ = logit(mp_.p_min_);
  mp_.clamp_max_log_ = logit(mp_.p_max_);
  mp_.min_occupancy_log_ = logit(mp_.p_occ_);
  mp_.unknown_flag_ = 0.01;
  mp_.map_sliding_thresh_vox_ = std::max(1, static_cast<int>(std::ceil(mp_.map_sliding_thresh_ * mp_.resolution_inv_)));

  cout << "hit: " << mp_.prob_hit_log_ << endl;
  cout << "miss: " << mp_.prob_miss_log_ << endl;
  cout << "min log: " << mp_.clamp_min_log_ << endl;
  cout << "max: " << mp_.clamp_max_log_ << endl;
  cout << "thresh log: " << mp_.min_occupancy_log_ << endl;

  for (int i = 0; i < 3; ++i)
    mp_.map_voxel_num_(i) = ceil(mp_.map_size_(i) / mp_.resolution_);

  mp_.map_min_boundary_ = mp_.map_origin_;
  mp_.map_max_boundary_ = mp_.map_origin_ + mp_.map_size_;
  posToIndex(mp_.map_origin_, mp_.map_bound_min_idx_);
  mp_.map_bound_max_idx_ = mp_.map_bound_min_idx_ + mp_.map_voxel_num_ - Eigen::Vector3i::Ones();
  mp_.map_origin_idx_ = mp_.map_bound_min_idx_ + mp_.map_voxel_num_ / 2;
  updateMapBoundaryFromIndex();

  // initialize data buffers

  int buffer_size = mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2);

  md_.occupancy_buffer_ = vector<double>(buffer_size, mp_.clamp_min_log_ - mp_.unknown_flag_);
  md_.occupancy_buffer_inflate_ = vector<char>(buffer_size, 0);
  md_.occupancy_buffer_inflate_cnt_ = vector<int>(buffer_size, 0);
  rebuildInflationOffsets();

  md_.count_hit_and_miss_ = vector<short>(buffer_size, 0);
  md_.count_hit_ = vector<short>(buffer_size, 0);
  md_.flag_rayend_ = vector<char>(buffer_size, -1);
  md_.flag_traverse_ = vector<char>(buffer_size, -1);

  md_.raycast_num_ = 0;

  md_.proj_points_.resize(640 * 480 / mp_.skip_pixel_ / mp_.skip_pixel_);
  md_.proj_points_cnt = 0;

  /* init callback */
  tf_broadcaster_.reset(new tf::TransformBroadcaster);

  if (mp_.sensor_type_ == "depth")
  {
    depth_sub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(node_, "/grid_map/depth", 50));
    depth_pose_sub_.reset(
        new message_filters::Subscriber<nav_msgs::Odometry>(node_, "/grid_map/sensor_pose", 100));

    sync_image_pose_.reset(new message_filters::Synchronizer<SyncPolicyImagePose>(
        SyncPolicyImagePose(100), *depth_sub_, *depth_pose_sub_));
    sync_image_pose_->registerCallback(boost::bind(&GridMap::depthPoseCallback, this, _1, _2));
  }
  else if (mp_.sensor_type_ == "lidar")
  {
    lidar_pose_sub_ =
        node_.subscribe<nav_msgs::Odometry>("/grid_map/sensor_pose", 50, &GridMap::sensorPoseCallback, this);
    cloud_sub_ =
        node_.subscribe<sensor_msgs::PointCloud2>("/grid_map/cloud", 10, &GridMap::cloudCallback, this);
  }

  sliding_map_frame_sub_ =
      node_.subscribe<nav_msgs::Odometry>("/grid_map/body_pose", 50, &GridMap::slidingMapFrameCallback, this);

  occ_timer_ = node_.createTimer(ros::Duration(0.05), &GridMap::updateOccupancyCallback, this);
  vis_timer_ = node_.createTimer(ros::Duration(0.05), &GridMap::visCallback, this);

  map_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/grid_map/occupancy", 10);
  map_inf_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/grid_map/occupancy_inflate", 10);
  sliding_map_bbox_pub_ = node_.advertise<visualization_msgs::Marker>("/grid_map/sliding_map_bbox", 10);

  unknown_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/grid_map/unknown", 10);
  depth_cloud_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/grid_map/depth_cloud", 10);
  extrinsic_pose_pub_ = node_.advertise<nav_msgs::Odometry>("/grid_map/sensor_pose_extrinsic", 10);

  md_.occ_need_update_ = false;
  md_.use_cloud_update_ = false;
  md_.has_first_depth_ = false;
  md_.has_ray_pose_ = false;
  md_.has_cloud_ = false;
  md_.image_cnt_ = 0;
  md_.ray_pos_.setZero();
  md_.sliding_map_frame_pos_.setZero();
  md_.ray_q_ = Eigen::Quaterniond::Identity();

  md_.fuse_time_ = 0.0;
  md_.update_num_ = 0;
  md_.max_fuse_time_ = 0.0;
  md_.local_bound_min_ = mp_.map_bound_min_idx_;
  md_.local_bound_max_ = mp_.map_bound_max_idx_;

  // rand_noise_ = uniform_real_distribution<double>(-0.2, 0.2);
  // rand_noise2_ = normal_distribution<double>(0, 0.2);
  // random_device rd;
  // eng_ = default_random_engine(rd());
}

void GridMap::updateMapBoundaryFromIndex()
{
  mp_.map_bound_min_idx_ = mp_.map_origin_idx_ - mp_.map_voxel_num_ / 2;
  mp_.map_bound_max_idx_ = mp_.map_bound_min_idx_ + mp_.map_voxel_num_ - Eigen::Vector3i::Ones();

  mp_.map_min_boundary_ = mp_.map_bound_min_idx_.cast<double>() * mp_.resolution_;
  mp_.map_max_boundary_ = (mp_.map_bound_max_idx_.cast<double>() + Eigen::Vector3d::Ones()) * mp_.resolution_;
  mp_.map_origin_ = mp_.map_min_boundary_;
}

void GridMap::rebuildInflationOffsets()
{
  const double double_radius = std::max(0.0, mp_.double_cylinder_radius_);
  const int inf_step_xy = ceil(double_radius / mp_.resolution_);
  const int inf_step_z_up = ceil(mp_.obstacles_inflation_z_up / mp_.resolution_);
  const int inf_step_z_down = ceil(mp_.obstacles_inflation_z_down / mp_.resolution_);

  md_.inflate_offsets_.clear();
  for (int x = -inf_step_xy; x <= inf_step_xy; ++x)
    for (int y = -inf_step_xy; y <= inf_step_xy; ++y)
    {
      Eigen::Vector2d offset_xy(x * mp_.resolution_, y * mp_.resolution_);
      if (offset_xy.norm() >= double_radius)
        continue;

      for (int z = -inf_step_z_down; z <= inf_step_z_up; ++z)
        md_.inflate_offsets_.push_back(Eigen::Vector3i(x, y, z));
    }
}

void GridMap::resetAllMapData()
{
  std::fill(md_.occupancy_buffer_.begin(), md_.occupancy_buffer_.end(), mp_.clamp_min_log_ - mp_.unknown_flag_);
  std::fill(md_.occupancy_buffer_inflate_.begin(), md_.occupancy_buffer_inflate_.end(), 0);
  std::fill(md_.occupancy_buffer_inflate_cnt_.begin(), md_.occupancy_buffer_inflate_cnt_.end(), 0);
  std::fill(md_.count_hit_and_miss_.begin(), md_.count_hit_and_miss_.end(), 0);
  std::fill(md_.count_hit_.begin(), md_.count_hit_.end(), 0);
  std::fill(md_.flag_rayend_.begin(), md_.flag_rayend_.end(), -1);
  std::fill(md_.flag_traverse_.begin(), md_.flag_traverse_.end(), -1);
  std::queue<Eigen::Vector3i> empty;
  std::swap(md_.cache_voxel_, empty);
}

void GridMap::hashIdToGlobalIndex(int addr, Eigen::Vector3i& id_g) const
{
  Eigen::Vector3i id_l;
  id_l(0) = addr / (mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2));
  id_l(1) = (addr - id_l(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2)) / mp_.map_voxel_num_(2);
  id_l(2) = addr - id_l(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) -
            id_l(1) * mp_.map_voxel_num_(2);

  for (int i = 0; i < 3; ++i)
  {
    const int min_l = getLocalIndex(mp_.map_bound_min_idx_(i), i);
    int dist = id_l(i) - min_l;
    if (dist < 0)
      dist += mp_.map_voxel_num_(i);
    id_g(i) = mp_.map_bound_min_idx_(i) + dist;
  }
}

void GridMap::updateInflationLayer(const Eigen::Vector3i& id, int delta,
                                   const vector<Eigen::Vector3i>& offsets,
                                   std::vector<int>& cnt_buffer,
                                   std::vector<char>& flag_buffer,
                                   const std::vector<char>* ignore_mask)
{
  for (const auto& offset : offsets)
  {
    const Eigen::Vector3i inf_id = id + offset;
    if (!isInMap(inf_id))
      continue;

    const int addr = toAddress(inf_id);
    if (ignore_mask && (*ignore_mask)[addr])
      continue;

    cnt_buffer[addr] += delta;
    if (cnt_buffer[addr] < 0)
      cnt_buffer[addr] = 0;
    flag_buffer[addr] = cnt_buffer[addr] > 0 ? 1 : 0;
  }
}

void GridMap::updateInflation(const Eigen::Vector3i& id, int delta, const std::vector<char>* ignore_mask)
{
  updateInflationLayer(id, delta, md_.inflate_offsets_, md_.occupancy_buffer_inflate_cnt_,
                       md_.occupancy_buffer_inflate_, ignore_mask);
}

void GridMap::applyOccupancyUpdate(const Eigen::Vector3i& id, double new_log_odds)
{
  if (!isInMap(id))
    return;

  const int addr = toAddress(id);
  const bool was_occ = md_.occupancy_buffer_[addr] > mp_.min_occupancy_log_;
  const bool now_occ = new_log_odds > mp_.min_occupancy_log_;

  md_.occupancy_buffer_[addr] = new_log_odds;
  if (was_occ != now_occ)
    updateInflation(id, now_occ ? 1 : -1);
}

void GridMap::resetCellByAddress(int addr)
{
  Eigen::Vector3i id_g;
  hashIdToGlobalIndex(addr, id_g);
  if (md_.occupancy_buffer_[addr] > mp_.min_occupancy_log_)
    updateInflation(id_g, -1);

  md_.occupancy_buffer_[addr] = mp_.clamp_min_log_ - mp_.unknown_flag_;
  md_.count_hit_[addr] = 0;
  md_.count_hit_and_miss_[addr] = 0;
  md_.flag_rayend_[addr] = -1;
  md_.flag_traverse_[addr] = -1;
}

void GridMap::resetCellByAddressForSliding(int addr, const std::vector<char>& clear_mask)
{
  Eigen::Vector3i id_g;
  hashIdToGlobalIndex(addr, id_g);
  if (md_.occupancy_buffer_[addr] > mp_.min_occupancy_log_)
    updateInflation(id_g, -1, &clear_mask);
}

void GridMap::updateSlidingMap(const Eigen::Vector3d& center)
{
  if (!mp_.map_sliding_en_)
    return;

  Eigen::Vector3i new_origin_idx;
  posToIndex(center, new_origin_idx);
  const Eigen::Vector3i shift_num = new_origin_idx - mp_.map_origin_idx_;
  if (shift_num.cwiseAbs().maxCoeff() < mp_.map_sliding_thresh_vox_)
    return;

  if ((shift_num.cwiseAbs().array() >= mp_.map_voxel_num_.array()).any())
  {
    resetAllMapData();
    mp_.map_origin_idx_ = new_origin_idx;
    updateMapBoundaryFromIndex();
    md_.local_bound_min_ = mp_.map_bound_min_idx_;
    md_.local_bound_max_ = mp_.map_bound_max_idx_;
    return;
  }

  const int buffer_size = mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2);
  std::vector<char> clear_mask(buffer_size, 0);
  std::vector<int> clear_addrs;
  clear_addrs.reserve(buffer_size / 8);

  auto add_clear_addr = [&](const Eigen::Vector3i& id_l) {
    const int addr = toAddressLocal(id_l);
    if (!clear_mask[addr])
    {
      clear_mask[addr] = 1;
      clear_addrs.push_back(addr);
    }
  };

  for (int dim = 0; dim < 3; ++dim)
  {
    const int shift = shift_num(dim);
    if (shift == 0)
      continue;

    if (shift > 0)
    {
      for (int k = 0; k < shift; ++k)
      {
        const int clear_g = mp_.map_bound_min_idx_(dim) + k;
        const int clear_l = getLocalIndex(clear_g, dim);

        for (int a = 0; a < mp_.map_voxel_num_((dim + 1) % 3); ++a)
          for (int b = 0; b < mp_.map_voxel_num_((dim + 2) % 3); ++b)
          {
            Eigen::Vector3i id_l;
            id_l(dim) = clear_l;
            id_l((dim + 1) % 3) = a;
            id_l((dim + 2) % 3) = b;
            add_clear_addr(id_l);
          }
      }
    }
    else
    {
      for (int k = 0; k < -shift; ++k)
      {
        const int clear_g = mp_.map_bound_max_idx_(dim) - k;
        const int clear_l = getLocalIndex(clear_g, dim);

        for (int a = 0; a < mp_.map_voxel_num_((dim + 1) % 3); ++a)
          for (int b = 0; b < mp_.map_voxel_num_((dim + 2) % 3); ++b)
          {
            Eigen::Vector3i id_l;
            id_l(dim) = clear_l;
            id_l((dim + 1) % 3) = a;
            id_l((dim + 2) % 3) = b;
            add_clear_addr(id_l);
          }
      }
    }
  }

  for (int addr : clear_addrs)
    resetCellByAddressForSliding(addr, clear_mask);

  for (int addr : clear_addrs)
  {
    md_.occupancy_buffer_[addr] = mp_.clamp_min_log_ - mp_.unknown_flag_;
    md_.occupancy_buffer_inflate_cnt_[addr] = 0;
    md_.occupancy_buffer_inflate_[addr] = 0;
    md_.count_hit_[addr] = 0;
    md_.count_hit_and_miss_[addr] = 0;
    md_.flag_rayend_[addr] = -1;
    md_.flag_traverse_[addr] = -1;
  }

  mp_.map_origin_idx_ = new_origin_idx;
  updateMapBoundaryFromIndex();
  boundIndex(md_.local_bound_min_);
  boundIndex(md_.local_bound_max_);
}

void GridMap::resetBuffer()
{
  resetAllMapData();
  md_.local_bound_min_ = mp_.map_bound_min_idx_;
  md_.local_bound_max_ = mp_.map_bound_max_idx_;
}

void GridMap::resetBuffer(Eigen::Vector3d min_pos, Eigen::Vector3d max_pos)
{
  Eigen::Vector3i min_id, max_id;
  posToIndex(min_pos, min_id);
  posToIndex(max_pos, max_id);

  boundIndex(min_id);
  boundIndex(max_id);

  for (int x = min_id(0); x <= max_id(0); ++x)
    for (int y = min_id(1); y <= max_id(1); ++y)
    {
      for (int z = min_id(2); z <= max_id(2); ++z)
      {
        resetCellByAddress(toAddress(x, y, z));
      }
    }
}

int GridMap::setCacheOccupancy(Eigen::Vector3d pos, int occ)
{
  if (occ != 1 && occ != 0)
    return INVALID_IDX;

  Eigen::Vector3i id;
  posToIndex(pos, id);
  if (!isInMap(id))
    return INVALID_IDX;

  int idx_ctns = toAddress(id);

  md_.count_hit_and_miss_[idx_ctns] += 1;

  if (md_.count_hit_and_miss_[idx_ctns] == 1)
  {
    md_.cache_voxel_.push(id);
  }

  if (occ == 1)
    md_.count_hit_[idx_ctns] += 1;

  return idx_ctns;
}

void GridMap::projectDepthImage()
{
  // md_.proj_points_.clear();
  md_.proj_points_cnt = 0;

  uint16_t *row_ptr;
  // int cols = current_img_.cols, rows = current_img_.rows;
  int cols = md_.depth_image_.cols;
  int rows = md_.depth_image_.rows;

  double depth;

  Eigen::Matrix3d sensor_r = md_.ray_q_.toRotationMatrix();

  // cout << "rotate: " << md_.ray_q_.toRotationMatrix() << endl;
  // std::cout << "pos in proj: " << md_.ray_pos_ << std::endl;

  if (!md_.has_first_depth_)
  {
    md_.has_first_depth_ = true;
    return;
  }

  Eigen::Vector3d pt_cur, pt_world;
  const double inv_factor = 1.0 / mp_.k_depth_scaling_factor_;

  for (int v = mp_.depth_filter_margin_; v < rows - mp_.depth_filter_margin_; v += mp_.skip_pixel_)
  {
    row_ptr = md_.depth_image_.ptr<uint16_t>(v) + mp_.depth_filter_margin_;

    for (int u = mp_.depth_filter_margin_; u < cols - mp_.depth_filter_margin_; u += mp_.skip_pixel_)
    {
      const uint16_t raw_depth = *row_ptr;
      depth = raw_depth * inv_factor;
      row_ptr = row_ptr + mp_.skip_pixel_;

      // filter depth
      // depth += rand_noise_(eng_);
      // if (depth > 0.01) depth += rand_noise2_(eng_);

      if (raw_depth == 0)
      {
        depth = mp_.max_ray_length_ + 0.1;
      }
      else if (depth < mp_.depth_filter_mindist_)
      {
        continue;
      }
      else if (depth > mp_.depth_filter_maxdist_)
      {
        depth = mp_.max_ray_length_ + 0.1;
      }

      // project to world frame
      pt_cur(0) = (u - mp_.cx_) * depth / mp_.fx_;
      pt_cur(1) = (v - mp_.cy_) * depth / mp_.fy_;
      pt_cur(2) = depth;

      pt_world = sensor_r * pt_cur + md_.ray_pos_;
      // if (!isInMap(pt_world)) {
      //   pt_world = closetPointInMap(pt_world, md_.ray_pos_);
      // }

      md_.proj_points_[md_.proj_points_cnt++] = pt_world;
    }
  }
}

void GridMap::raycastProcess()
{
  // if (md_.proj_points_.size() == 0)
  if (md_.proj_points_cnt == 0)
    return;

  updateSlidingMap(md_.ray_pos_);

  ros::Time t1, t2;

  md_.raycast_num_ += 1;

  int vox_idx;
  double length;

  // bounding box of updated region
  double min_x = mp_.map_max_boundary_(0);
  double min_y = mp_.map_max_boundary_(1);
  double min_z = mp_.map_max_boundary_(2);

  double max_x = mp_.map_min_boundary_(0);
  double max_y = mp_.map_min_boundary_(1);
  double max_z = mp_.map_min_boundary_(2);

  RayCaster raycaster;
  Eigen::Vector3d half = Eigen::Vector3d(0.5, 0.5, 0.5);
  Eigen::Vector3d ray_pt, pt_w;

  for (int i = 0; i < md_.proj_points_cnt; ++i)
  {
    pt_w = md_.proj_points_[i];

    // set flag for projected point

    if (!isInMap(pt_w))
    {
      pt_w = closetPointInMap(pt_w, md_.ray_pos_);

      length = (pt_w - md_.ray_pos_).norm();
      if (length > mp_.max_ray_length_)
      {
        pt_w = (pt_w - md_.ray_pos_) / length * mp_.max_ray_length_ + md_.ray_pos_;
      }
      vox_idx = setCacheOccupancy(pt_w, 0);
    }
    else
    {
      length = (pt_w - md_.ray_pos_).norm();

      if (length > mp_.max_ray_length_)
      {
        pt_w = (pt_w - md_.ray_pos_) / length * mp_.max_ray_length_ + md_.ray_pos_;
        vox_idx = setCacheOccupancy(pt_w, 0);
      }
      else
      {
        vox_idx = setCacheOccupancy(pt_w, 1);
      }
    }

    max_x = max(max_x, pt_w(0));
    max_y = max(max_y, pt_w(1));
    max_z = max(max_z, pt_w(2));

    min_x = min(min_x, pt_w(0));
    min_y = min(min_y, pt_w(1));
    min_z = min(min_z, pt_w(2));

    // raycasting between ray origin and point

    if (vox_idx != INVALID_IDX)
    {
      if (md_.flag_rayend_[vox_idx] == md_.raycast_num_)
      {
        continue;
      }
      else
      {
        md_.flag_rayend_[vox_idx] = md_.raycast_num_;
      }
    }

    raycaster.setInput(pt_w / mp_.resolution_, md_.ray_pos_ / mp_.resolution_);

    while (raycaster.step(ray_pt))
    {
      Eigen::Vector3d tmp = (ray_pt + half) * mp_.resolution_;
      length = (tmp - md_.ray_pos_).norm();

      vox_idx = setCacheOccupancy(tmp, 0);

      if (vox_idx != INVALID_IDX)
      {
        if (md_.flag_traverse_[vox_idx] == md_.raycast_num_)
        {
          break;
        }
        else
        {
          md_.flag_traverse_[vox_idx] = md_.raycast_num_;
        }
      }
    }
  }

  min_x = min(min_x, md_.ray_pos_(0));
  min_y = min(min_y, md_.ray_pos_(1));
  min_z = min(min_z, md_.ray_pos_(2));

  max_x = max(max_x, md_.ray_pos_(0));
  max_y = max(max_y, md_.ray_pos_(1));
  max_z = max(max_z, md_.ray_pos_(2));
  max_z = max(max_z, mp_.ground_height_);

  posToIndex(Eigen::Vector3d(max_x, max_y, max_z), md_.local_bound_max_);
  posToIndex(Eigen::Vector3d(min_x, min_y, min_z), md_.local_bound_min_);
  boundIndex(md_.local_bound_min_);
  boundIndex(md_.local_bound_max_);

  // update occupancy cached in queue
  Eigen::Vector3d local_range_min = md_.ray_pos_ - mp_.local_update_range_;
  Eigen::Vector3d local_range_max = md_.ray_pos_ + mp_.local_update_range_;

  Eigen::Vector3i min_id, max_id;
  posToIndex(local_range_min, min_id);
  posToIndex(local_range_max, max_id);
  boundIndex(min_id);
  boundIndex(max_id);

  // std::cout << "cache all: " << md_.cache_voxel_.size() << std::endl;

  while (!md_.cache_voxel_.empty())
  {

    Eigen::Vector3i idx = md_.cache_voxel_.front();
    int idx_ctns = toAddress(idx);
    md_.cache_voxel_.pop();

    double log_odds_update =
        md_.count_hit_[idx_ctns] >= md_.count_hit_and_miss_[idx_ctns] - md_.count_hit_[idx_ctns] ? mp_.prob_hit_log_ : mp_.prob_miss_log_;

    md_.count_hit_[idx_ctns] = md_.count_hit_and_miss_[idx_ctns] = 0;

    if (log_odds_update >= 0 && md_.occupancy_buffer_[idx_ctns] >= mp_.clamp_max_log_)
    {
      continue;
    }
    else if (log_odds_update <= 0 && md_.occupancy_buffer_[idx_ctns] <= mp_.clamp_min_log_)
    {
      applyOccupancyUpdate(idx, mp_.clamp_min_log_);
      continue;
    }

    bool in_local = idx(0) >= min_id(0) && idx(0) <= max_id(0) && idx(1) >= min_id(1) &&
                    idx(1) <= max_id(1) && idx(2) >= min_id(2) && idx(2) <= max_id(2);
    if (!in_local)
    {
      applyOccupancyUpdate(idx, mp_.clamp_min_log_);
    }

    const double new_log_odds =
        std::min(std::max(md_.occupancy_buffer_[idx_ctns] + log_odds_update, mp_.clamp_min_log_),
                 mp_.clamp_max_log_);
    applyOccupancyUpdate(idx, new_log_odds);
  }
}

Eigen::Vector3d GridMap::closetPointInMap(const Eigen::Vector3d &pt, const Eigen::Vector3d &ray_pos)
{
  Eigen::Vector3d diff = pt - ray_pos;
  Eigen::Vector3d max_tc = mp_.map_max_boundary_ - ray_pos;
  Eigen::Vector3d min_tc = mp_.map_min_boundary_ - ray_pos;

  double min_t = 1000000;

  for (int i = 0; i < 3; ++i)
  {
    if (fabs(diff[i]) > 0)
    {

      double t1 = max_tc[i] / diff[i];
      if (t1 > 0 && t1 < min_t)
        min_t = t1;

      double t2 = min_tc[i] / diff[i];
      if (t2 > 0 && t2 < min_t)
        min_t = t2;
    }
  }

  return ray_pos + (min_t - 1e-3) * diff;
}

void GridMap::visCallback(const ros::TimerEvent & /*event*/)
{

  publishMap();
  publishMapInflate(true);
  publishSlidingMapFrame();
  publishSlidingMapBBox();
  publishDepthCloud();
}

void GridMap::updateOccupancyCallback(const ros::TimerEvent & /*event*/)
{
  if (!md_.occ_need_update_)
    return;

  /* update occupancy */
  // ros::Time t1, t2, t3, t4;
  // t1 = ros::Time::now();

  if (!md_.use_cloud_update_)
    projectDepthImage();
  // t2 = ros::Time::now();
  raycastProcess();
  // t3 = ros::Time::now();

  // t4 = ros::Time::now();

  // cout << setprecision(7);
  // cout << "t2=" << (t2-t1).toSec() << " t3=" << (t3-t2).toSec() << " t4=" << (t4-t3).toSec() << endl;;

  // md_.fuse_time_ += (t2 - t1).toSec();
  // md_.max_fuse_time_ = max(md_.max_fuse_time_, (t2 - t1).toSec());

  // if (mp_.show_occ_time_)
  //   ROS_WARN("Fusion: cur t = %lf, avg t = %lf, max t = %lf", (t2 - t1).toSec(),
  //            md_.fuse_time_ / md_.update_num_, md_.max_fuse_time_);

  md_.occ_need_update_ = false;
  md_.use_cloud_update_ = false;
}

void GridMap::depthPoseCallback(const sensor_msgs::ImageConstPtr &img,
                                const nav_msgs::OdometryConstPtr &pose)
{
  if (mp_.sensor_type_ != "depth")
    return;

  /* get depth image */
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(img, img->encoding);

  if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
  {
    (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, mp_.k_depth_scaling_factor_);
  }
  cv_ptr->image.copyTo(md_.depth_image_);

  // std::cout << "depth: " << md_.depth_image_.cols << ", " << md_.depth_image_.rows << std::endl;

  /* get pose */
  const geometry_msgs::Pose &sensor_pose = pose->pose.pose;
  Eigen::Quaterniond ray_q(sensor_pose.orientation.w, sensor_pose.orientation.x,
                           sensor_pose.orientation.y, sensor_pose.orientation.z);
  if (ray_q.norm() < 1e-6)
    return;
  ray_q.normalize();

  Eigen::Vector3d ray_pos(sensor_pose.position.x, sensor_pose.position.y, sensor_pose.position.z);
  if (mp_.need_extrinsic_)
  {
    const Eigen::Matrix3d pose_r = ray_q.toRotationMatrix();
    ray_pos += pose_r * mp_.depth_extrinsic_.block<3, 1>(0, 3);
    ray_q = Eigen::Quaterniond(pose_r * mp_.depth_extrinsic_.block<3, 3>(0, 0));
    ray_q.normalize();
  }

  nav_msgs::Odometry extrinsic_pose = *pose;
  extrinsic_pose.pose.pose.position.x = ray_pos.x();
  extrinsic_pose.pose.pose.position.y = ray_pos.y();
  extrinsic_pose.pose.pose.position.z = ray_pos.z();
  extrinsic_pose.pose.pose.orientation.x = ray_q.x();
  extrinsic_pose.pose.pose.orientation.y = ray_q.y();
  extrinsic_pose.pose.pose.orientation.z = ray_q.z();
  extrinsic_pose.pose.pose.orientation.w = ray_q.w();
  extrinsic_pose.child_frame_id =
      pose->child_frame_id.empty() ? "sensor_extrinsic" : pose->child_frame_id + "_extrinsic";
  extrinsic_pose_pub_.publish(extrinsic_pose);

  md_.ray_pos_ = ray_pos;
  md_.ray_q_ = ray_q;
  md_.use_cloud_update_ = false;
  updateSlidingMap(md_.ray_pos_);
  if (isInMap(md_.ray_pos_))
  {
    md_.has_ray_pose_ = true;
    md_.update_num_ += 1;
    md_.occ_need_update_ = true;
  }
  else
  {
    md_.occ_need_update_ = false;
  }
}

void GridMap::sensorPoseCallback(const nav_msgs::OdometryConstPtr &pose_msg)
{
  if (mp_.sensor_type_ != "lidar")
    return;

  const geometry_msgs::Pose &sensor_pose = pose_msg->pose.pose;
  Eigen::Quaterniond ray_q(sensor_pose.orientation.w, sensor_pose.orientation.x,
                           sensor_pose.orientation.y, sensor_pose.orientation.z);
  if (ray_q.norm() < 1e-6)
    return;
  ray_q.normalize();

  Eigen::Vector3d ray_pos(sensor_pose.position.x, sensor_pose.position.y, sensor_pose.position.z);
  if (mp_.need_extrinsic_)
  {
    const Eigen::Matrix3d pose_r = ray_q.toRotationMatrix();
    ray_pos += pose_r * mp_.lidar_extrinsic_.block<3, 1>(0, 3);
    ray_q = Eigen::Quaterniond(pose_r * mp_.lidar_extrinsic_.block<3, 3>(0, 0));
    ray_q.normalize();
  }
  if (!std::isfinite(ray_pos.x()) || !std::isfinite(ray_pos.y()) || !std::isfinite(ray_pos.z()))
    return;

  md_.ray_pos_ = ray_pos;
  md_.ray_q_ = ray_q;
  md_.has_ray_pose_ = true;
  updateSlidingMap(md_.ray_pos_);
}

void GridMap::slidingMapFrameCallback(const nav_msgs::OdometryConstPtr &pose)
{
  const geometry_msgs::Point &pos = pose->pose.pose.position;
  md_.sliding_map_frame_pos_ = Eigen::Vector3d(pos.x, pos.y, pos.z);
}

void GridMap::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &img)
{
  if (mp_.sensor_type_ != "lidar")
    return;

  if (!md_.has_ray_pose_)
  {
    ROS_WARN_THROTTLE(1.0, "[GridMap] no /grid_map/sensor_pose yet for lidar cloud update.");
    return;
  }

  pcl::PointCloud<pcl::PointXYZ> latest_cloud;
  pcl::fromROSMsg(*img, latest_cloud);

  md_.has_cloud_ = true;

  if (latest_cloud.points.size() == 0)
    return;

  const Eigen::Matrix3d sensor_r = md_.ray_q_.toRotationMatrix();
  const Eigen::Vector3d ray_pos = md_.ray_pos_;
  if (!std::isfinite(ray_pos.x()) || !std::isfinite(ray_pos.y()) || !std::isfinite(ray_pos.z()))
    return;

  updateSlidingMap(ray_pos);

  md_.proj_points_cnt = 0;

  for (size_t i = 0; i < latest_cloud.points.size(); ++i)
  {
    const pcl::PointXYZ &pt = latest_cloud.points[i];
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z))
      continue;

    Eigen::Vector3d pt_world;
    if (mp_.cloud_is_world_)
    {
      pt_world = Eigen::Vector3d(pt.x, pt.y, pt.z);
    }
    else
    {
      const Eigen::Vector3d pt_sensor(pt.x, pt.y, pt.z);
      pt_world = sensor_r * pt_sensor + ray_pos;
    }
    const Eigen::Vector3d devi = pt_world - ray_pos;
    const double ray_length = devi.norm();
    const bool in_local_range =
        fabs(devi(0)) <= mp_.local_update_range_(0) && fabs(devi(1)) <= mp_.local_update_range_(1) &&
        fabs(devi(2)) <= mp_.local_update_range_(2);
    if (!in_local_range && ray_length <= mp_.max_ray_length_)
      continue;

    if (md_.proj_points_cnt >= static_cast<int>(md_.proj_points_.size()))
      md_.proj_points_.push_back(pt_world);
    else
      md_.proj_points_[md_.proj_points_cnt] = pt_world;

    md_.proj_points_cnt++;
  }

  if (md_.proj_points_cnt == 0)
    return;

  md_.use_cloud_update_ = true;
  md_.occ_need_update_ = true;
}

void GridMap::publishMap()
{

  if (map_pub_.getNumSubscribers() <= 0)
    return;

  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  Eigen::Vector3i min_cut = mp_.map_bound_min_idx_;
  Eigen::Vector3i max_cut = mp_.map_bound_max_idx_;

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z)
      {
        if (md_.occupancy_buffer_[toAddress(x, y, z)] < mp_.min_occupancy_log_)
          continue;

        Eigen::Vector3d pos;
        indexToPos(Eigen::Vector3i(x, y, z), pos);
        if (md_.has_ray_pose_ && pos(2) > md_.ray_pos_(2) + mp_.vis_height_)
          continue;
        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        cloud.push_back(pt);
      }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;
  sensor_msgs::PointCloud2 cloud_msg;

  pcl::toROSMsg(cloud, cloud_msg);
  map_pub_.publish(cloud_msg);
}

void GridMap::publishMapInflate(bool all_info)
{

  if (map_inf_pub_.getNumSubscribers() <= 0)
    return;

  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  Eigen::Vector3i min_cut = mp_.map_bound_min_idx_;
  Eigen::Vector3i max_cut = mp_.map_bound_max_idx_;

  const std::vector<char> &inflate_buffer = md_.occupancy_buffer_inflate_;
  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z)
      {
        if (inflate_buffer[toAddress(x, y, z)] == 0)
          continue;

        Eigen::Vector3d pos;
        indexToPos(Eigen::Vector3i(x, y, z), pos);
        if (md_.has_ray_pose_ && pos(2) > md_.ray_pos_(2) + mp_.vis_height_)
          continue;

        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        cloud.push_back(pt);
      }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;
  sensor_msgs::PointCloud2 cloud_msg;

  pcl::toROSMsg(cloud, cloud_msg);
  map_inf_pub_.publish(cloud_msg);

  // ROS_INFO("pub map");
}

void GridMap::publishSlidingMapFrame()
{
  if (mp_.sliding_map_frame_id_.empty())
    return;

  const ros::Time stamp = ros::Time::now();
  static ros::Time last_stamp;
  static bool have_last_stamp = false;
  if (have_last_stamp && stamp == last_stamp)
    return;
  last_stamp = stamp;
  have_last_stamp = true;

  tf::Transform transform;
  transform.setOrigin(tf::Vector3(md_.sliding_map_frame_pos_.x(), md_.sliding_map_frame_pos_.y(),
                                  md_.sliding_map_frame_pos_.z()));
  transform.setRotation(tf::Quaternion::getIdentity());
  tf_broadcaster_->sendTransform(
      tf::StampedTransform(transform, stamp, mp_.frame_id_, mp_.sliding_map_frame_id_));
}

void GridMap::publishSlidingMapBBox()
{
  if (sliding_map_bbox_pub_.getNumSubscribers() <= 0)
    return;

  visualization_msgs::Marker marker;
  marker.header.frame_id = mp_.frame_id_;
  marker.header.stamp = ros::Time::now();
  marker.ns = "sliding_map";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.04;
  marker.color.r = 0.0;
  marker.color.g = 0.8;
  marker.color.b = 1.0;
  marker.color.a = 1.0;

  const Eigen::Vector3d& min_pt = mp_.map_min_boundary_;
  const Eigen::Vector3d& max_pt = mp_.map_max_boundary_;
  Eigen::Vector3d corners[8] = {
      {min_pt.x(), min_pt.y(), min_pt.z()},
      {max_pt.x(), min_pt.y(), min_pt.z()},
      {max_pt.x(), max_pt.y(), min_pt.z()},
      {min_pt.x(), max_pt.y(), min_pt.z()},
      {min_pt.x(), min_pt.y(), max_pt.z()},
      {max_pt.x(), min_pt.y(), max_pt.z()},
      {max_pt.x(), max_pt.y(), max_pt.z()},
      {min_pt.x(), max_pt.y(), max_pt.z()},
  };

  auto pushPoint = [&](const Eigen::Vector3d& p) {
    geometry_msgs::Point point;
    point.x = p.x();
    point.y = p.y();
    point.z = p.z();
    marker.points.push_back(point);
  };

  const int edges[12][2] = {
      {0, 1}, {1, 2}, {2, 3}, {3, 0},
      {4, 5}, {5, 6}, {6, 7}, {7, 4},
      {0, 4}, {1, 5}, {2, 6}, {3, 7},
  };

  for (const auto& edge : edges)
  {
    pushPoint(corners[edge[0]]);
    pushPoint(corners[edge[1]]);
  }

  sliding_map_bbox_pub_.publish(marker);
}

void GridMap::publishUnknown()
{
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  Eigen::Vector3i min_cut = md_.local_bound_min_;
  Eigen::Vector3i max_cut = md_.local_bound_max_;

  boundIndex(max_cut);
  boundIndex(min_cut);

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z)
      {

        if (md_.occupancy_buffer_[toAddress(x, y, z)] < mp_.clamp_min_log_ - 1e-3)
        {
          Eigen::Vector3d pos;
          indexToPos(Eigen::Vector3i(x, y, z), pos);
          if (md_.has_ray_pose_ && pos(2) > md_.ray_pos_(2) + mp_.vis_height_)
            continue;

          pt.x = pos(0);
          pt.y = pos(1);
          pt.z = pos(2);
          cloud.push_back(pt);
        }
      }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;

  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  unknown_pub_.publish(cloud_msg);
}

bool GridMap::odomValid() { return md_.has_ray_pose_; }

bool GridMap::hasDepthObservation() { return md_.has_first_depth_; }

Eigen::Vector3d GridMap::getOrigin() { return mp_.map_origin_; }

// int GridMap::getVoxelNum() {
//   return mp_.map_voxel_num_[0] * mp_.map_voxel_num_[1] * mp_.map_voxel_num_[2];
// }

void GridMap::getRegion(Eigen::Vector3d &ori, Eigen::Vector3d &size)
{
  ori = mp_.map_origin_, size = mp_.map_size_;
}

// GridMap

void GridMap::publishDepthCloud()
{
  if (depth_cloud_pub_.getNumSubscribers() <= 0)
    return;

  if (md_.proj_points_cnt == 0)
    return;

  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::PointXYZ pt;

  for (int i = 0; i < md_.proj_points_cnt; ++i)
  {
    pt.x = md_.proj_points_[i](0);
    pt.y = md_.proj_points_[i](1);
    pt.z = md_.proj_points_[i](2);
    cloud.push_back(pt);
  }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;

  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  cloud_msg.header.stamp = ros::Time::now();
  depth_cloud_pub_.publish(cloud_msg);
}
