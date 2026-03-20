#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "pcl/filters/radius_outlier_removal.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

struct DepthVoxelKey
{
  int x{0};
  int y{0};
  int z{0};

  bool operator==(const DepthVoxelKey & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct DepthVoxelKeyHash
{
  std::size_t operator()(const DepthVoxelKey & key) const
  {
    const std::uint64_t h1 = static_cast<std::uint64_t>(key.x) * 73856093u;
    const std::uint64_t h2 = static_cast<std::uint64_t>(key.y) * 19349663u;
    const std::uint64_t h3 = static_cast<std::uint64_t>(key.z) * 83492791u;
    return static_cast<std::size_t>(h1 ^ h2 ^ h3);
  }
};

struct DepthVoxelState
{
  int hits{0};
  double first_seen_sec{0.0};
  double last_seen_sec{0.0};
};

class DepthNavFilterNode : public rclcpp::Node
{
public:
  DepthNavFilterNode()
  : Node("depth_nav_filter")
  {
    // 입출력 토픽
    declare_parameter<std::string>("input_topic", "/camera/camera/depth/color/points");
    declare_parameter<std::string>("output_topic", "/camera/camera/depth/color/nav_points");

    // 필터 결과를 base_link 기준으로 내보내면 self-mask / blind-zone을 직관적으로 다루기 쉽다.
    declare_parameter<std::string>("target_frame", "base_link");
    declare_parameter<double>("tf_timeout_sec", 0.05);

    // navigation용으로 쓸 depth의 유효 범위
    declare_parameter<double>("min_x", -0.05);
    declare_parameter<double>("max_x", 1.20);
    declare_parameter<double>("max_abs_y", 1.00);
    declare_parameter<double>("z_min", 0.10);
    declare_parameter<double>("z_max", 1.20);
    declare_parameter<double>("max_range", 1.20);

    // 센서 바로 앞의 반사/엣지 노이즈를 잘라내는 근거리 blind-zone
    declare_parameter<double>("sensor_origin_x", 0.30);
    declare_parameter<double>("sensor_origin_y", 0.00);
    declare_parameter<double>("sensor_blind_radius", 0.18);

    // 로봇 몸체/범퍼/마운트 근처 점을 지우기 위한 self-mask 사각형
    declare_parameter<double>("self_mask_min_x", -0.67);
    declare_parameter<double>("self_mask_max_x", 0.67);
    declare_parameter<double>("self_mask_min_y", -0.44);
    declare_parameter<double>("self_mask_max_y", 0.44);

    // 점군 희소 노이즈 억제
    declare_parameter<double>("voxel_leaf_size", 0.04);
    declare_parameter<double>("ror_radius", 0.10);
    declare_parameter<int>("ror_min_neighbors", 6);

    // 2~3프레임 정도 연속으로 보인 점만 obstacle 후보로 인정
    declare_parameter<double>("temporal_voxel_size", 0.05); // 5cm의 공간(voxel,칸)로 인식 
    declare_parameter<int>("temporal_min_hits", 2); // 2에서 3프레임 인식 후 장애물로 인식 
    declare_parameter<double>("temporal_hit_window_sec", 0.35);
    declare_parameter<double>("temporal_max_stale_sec", 0.50);

    declare_parameter<double>("log_period_sec", 2.0);

    input_topic_ = get_parameter("input_topic").as_string();
    output_topic_ = get_parameter("output_topic").as_string();
    target_frame_ = get_parameter("target_frame").as_string();
    tf_timeout_sec_ = get_parameter("tf_timeout_sec").as_double();

    min_x_ = get_parameter("min_x").as_double();
    max_x_ = get_parameter("max_x").as_double();
    max_abs_y_ = get_parameter("max_abs_y").as_double();
    z_min_ = get_parameter("z_min").as_double();
    z_max_ = get_parameter("z_max").as_double();
    max_range_ = get_parameter("max_range").as_double();
    max_range_sq_ = max_range_ * max_range_;

    sensor_origin_x_ = get_parameter("sensor_origin_x").as_double();
    sensor_origin_y_ = get_parameter("sensor_origin_y").as_double();
    sensor_blind_radius_ = get_parameter("sensor_blind_radius").as_double();
    sensor_blind_radius_sq_ = sensor_blind_radius_ * sensor_blind_radius_;

    self_mask_min_x_ = get_parameter("self_mask_min_x").as_double();
    self_mask_max_x_ = get_parameter("self_mask_max_x").as_double();
    self_mask_min_y_ = get_parameter("self_mask_min_y").as_double();
    self_mask_max_y_ = get_parameter("self_mask_max_y").as_double();

    voxel_leaf_size_ = get_parameter("voxel_leaf_size").as_double();
    ror_radius_ = get_parameter("ror_radius").as_double();
    ror_min_neighbors_ = get_parameter("ror_min_neighbors").as_int();

    temporal_voxel_size_ = get_parameter("temporal_voxel_size").as_double();
    temporal_min_hits_ = get_parameter("temporal_min_hits").as_int();
    temporal_hit_window_sec_ = get_parameter("temporal_hit_window_sec").as_double();
    temporal_max_stale_sec_ = get_parameter("temporal_max_stale_sec").as_double();
    log_period_sec_ = get_parameter("log_period_sec").as_double();

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&DepthNavFilterNode::callback, this, std::placeholders::_1));

    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic_, rclcpp::SensorDataQoS());

    RCLCPP_INFO(
      get_logger(),
      "Depth nav filter: %s -> %s, target=%s, range=%.2fm, blind=%.2fm, temporal_hits=%d",
      input_topic_.c_str(), output_topic_.c_str(), target_frame_.c_str(),
      max_range_, sensor_blind_radius_, temporal_min_hits_);
  }

private:
  DepthVoxelKey to_voxel(const pcl::PointXYZ & point) const
  {
    return DepthVoxelKey{
      static_cast<int>(std::floor(point.x / temporal_voxel_size_)),
      static_cast<int>(std::floor(point.y / temporal_voxel_size_)),
      static_cast<int>(std::floor(point.z / temporal_voxel_size_))
    };
  }

  void cleanup_stale_states(double now_sec)
  {
    std::vector<DepthVoxelKey> stale_keys;
    stale_keys.reserve(voxel_states_.size());

    for (const auto & kv : voxel_states_) {
      if ((now_sec - kv.second.last_seen_sec) > temporal_max_stale_sec_) {
        stale_keys.push_back(kv.first); //오래된 voxel 수집 
      }
    }

    for (const auto & key : stale_keys) {
      voxel_states_.erase(key); // 오래된 voxel 제거 
    }
  }

  bool passes_geometry_filters(const pcl::PointXYZ & point) const
  {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      return false;
    }

    if (point.x < min_x_ || point.x > max_x_) {
      return false;
    }

    if (std::abs(point.y) > max_abs_y_) {
      return false;
    }

    if (point.z < z_min_ || point.z > z_max_) {
      return false;
    }

    const double range_sq = point.x * point.x + point.y * point.y;
    if (range_sq > max_range_sq_) {
      return false;
    }

    // 로봇 footprint 내부/가장자리 점은 자기 몸체 또는 이미 충돌한 상태일 가능성이 높아 제거한다.
    if (point.x >= self_mask_min_x_ && point.x <= self_mask_max_x_ &&
      point.y >= self_mask_min_y_ && point.y <= self_mask_max_y_)
    {
      return false;
    }

    // 카메라 바로 앞의 specular/엣지 노이즈는 대개 매우 가까운 반경에 모이므로 따로 잘라낸다.
    const double dx = point.x - sensor_origin_x_;
    const double dy = point.y - sensor_origin_y_;
    if ((dx * dx + dy * dy) < sensor_blind_radius_sq_) {
      return false;
    }

    return true;
  }

  sensor_msgs::msg::PointCloud2 transform_to_target_frame(
    const sensor_msgs::msg::PointCloud2 & msg) const
  {
    if (msg.header.frame_id == target_frame_) {
      return msg;
    }

    auto tf = tf_buffer_->lookupTransform(
      target_frame_, msg.header.frame_id, msg.header.stamp,
      rclcpp::Duration::from_seconds(tf_timeout_sec_));

    sensor_msgs::msg::PointCloud2 transformed;
    tf2::doTransform(msg, transformed, tf);
    return transformed;
  }

  void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    sensor_msgs::msg::PointCloud2 cloud_in_target;
    try {
      cloud_in_target = transform_to_target_frame(*msg);
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Depth cloud TF 변환 실패 (%s -> %s): %s",
        msg->header.frame_id.c_str(), target_frame_.c_str(), e.what());
      return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(cloud_in_target, *input_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr geometry_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    geometry_filtered->header = input_cloud->header;
    geometry_filtered->points.reserve(input_cloud->size());

    for (const auto & point : input_cloud->points) {
      if (passes_geometry_filters(point)) {
        geometry_filtered->points.push_back(point);
      }
    }

    geometry_filtered->width = static_cast<std::uint32_t>(geometry_filtered->points.size());
    geometry_filtered->height = 1;
    geometry_filtered->is_dense = false;

    pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    if (!geometry_filtered->empty() && voxel_leaf_size_ > 1e-6) {
      pcl::VoxelGrid<pcl::PointXYZ> voxel;
      voxel.setInputCloud(geometry_filtered);
      voxel.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
      voxel.filter(*voxel_filtered);
    } else {
      *voxel_filtered = *geometry_filtered;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr sparse_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    if (!voxel_filtered->empty() && ror_radius_ > 1e-6 && ror_min_neighbors_ > 0) {
      pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;
      ror.setInputCloud(voxel_filtered);
      ror.setRadiusSearch(ror_radius_);
      ror.setMinNeighborsInRadius(ror_min_neighbors_);
      ror.filter(*sparse_filtered);
    } else {
      *sparse_filtered = *voxel_filtered;
    }

    const double now_sec = get_clock()->now().seconds();
    cleanup_stale_states(now_sec);

    std::unordered_set<DepthVoxelKey, DepthVoxelKeyHash> frame_voxels;
    frame_voxels.reserve(sparse_filtered->size());
    for (const auto & point : sparse_filtered->points) {
      frame_voxels.insert(to_voxel(point));
    }

    for (const auto & voxel : frame_voxels) {
      auto it = voxel_states_.find(voxel);
      if (it == voxel_states_.end()) {
        voxel_states_[voxel] = DepthVoxelState{1, now_sec, now_sec};
      } else {
        if ((now_sec - it->second.last_seen_sec) > temporal_hit_window_sec_) {
          it->second.hits = 1;
          it->second.first_seen_sec = now_sec;
        } else {
          it->second.hits += 1;
        }
        it->second.last_seen_sec = now_sec;
      }
    }

    std::unordered_set<DepthVoxelKey, DepthVoxelKeyHash> accepted_voxels;
    accepted_voxels.reserve(frame_voxels.size());
    for (const auto & voxel : frame_voxels) {
      auto it = voxel_states_.find(voxel);
      if (it != voxel_states_.end() && it->second.hits >= temporal_min_hits_) {
        accepted_voxels.insert(voxel);
      }
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    output_cloud->header = sparse_filtered->header;
    output_cloud->points.reserve(sparse_filtered->size());
    for (const auto & point : sparse_filtered->points) {
      if (accepted_voxels.find(to_voxel(point)) != accepted_voxels.end()) {
        output_cloud->points.push_back(point);
      }
    }
    output_cloud->width = static_cast<std::uint32_t>(output_cloud->points.size());
    output_cloud->height = 1;
    output_cloud->is_dense = false;

    sensor_msgs::msg::PointCloud2 out_msg;
    pcl::toROSMsg(*output_cloud, out_msg);
    out_msg.header.stamp = msg->header.stamp;
    out_msg.header.frame_id = target_frame_;
    pub_->publish(out_msg);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), static_cast<int64_t>(log_period_sec_ * 1000.0),
      "depth_nav_filter in=%zu geom=%zu sparse=%zu out=%zu",
      input_cloud->size(), geometry_filtered->size(), sparse_filtered->size(), output_cloud->size());
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string target_frame_;

  double tf_timeout_sec_{0.05};
  double min_x_{-0.05};
  double max_x_{1.20};
  double max_abs_y_{1.00};
  double z_min_{0.10};
  double z_max_{1.20};
  double max_range_{1.20};
  double max_range_sq_{1.44};

  double sensor_origin_x_{0.30};
  double sensor_origin_y_{0.00};
  double sensor_blind_radius_{0.18};
  double sensor_blind_radius_sq_{0.0324};

  double self_mask_min_x_{-0.67};
  double self_mask_max_x_{0.67};
  double self_mask_min_y_{-0.44};
  double self_mask_max_y_{0.44};

  double voxel_leaf_size_{0.04};
  double ror_radius_{0.10};
  int ror_min_neighbors_{6};

  double temporal_voxel_size_{0.05};
  int temporal_min_hits_{2};
  double temporal_hit_window_sec_{0.35};
  double temporal_max_stale_sec_{0.50};
  double log_period_sec_{2.0};

  std::unordered_map<DepthVoxelKey, DepthVoxelState, DepthVoxelKeyHash> voxel_states_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DepthNavFilterNode>());
  rclcpp::shutdown();
  return 0;
}
