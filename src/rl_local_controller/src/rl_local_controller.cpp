#include "rl_local_controller/rl_local_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace rl_local_controller
{

void RlLocalController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;

  // 기본 파라미터를 선언하고 읽는다.
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".max_lin", rclcpp::ParameterValue(max_lin_));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".max_ang", rclcpp::ParameterValue(max_ang_));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".stop_dist", rclcpp::ParameterValue(stop_dist_));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".hard_stop_dist", rclcpp::ParameterValue(hard_stop_dist_));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".creep_speed", rclcpp::ParameterValue(creep_speed_));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".in_place_heading", rclcpp::ParameterValue(in_place_heading_));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".in_place_dist", rclcpp::ParameterValue(in_place_dist_));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".min_turn_rate", rclcpp::ParameterValue(min_turn_rate_));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".rotate_min_side_clearance",
    rclcpp::ParameterValue(rotate_min_side_clearance_));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".rotate_min_front_clearance",
    rclcpp::ParameterValue(rotate_min_front_clearance_));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".escape_forward_speed",
    rclcpp::ParameterValue(escape_forward_speed_));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".escape_forward_turn_scale",
    rclcpp::ParameterValue(escape_forward_turn_scale_));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".escape_use_reverse",
    rclcpp::ParameterValue(escape_use_reverse_));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".escape_reverse_speed",
    rclcpp::ParameterValue(escape_reverse_speed_));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".slow_dist", rclcpp::ParameterValue(slow_dist_));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".heading_gain", rclcpp::ParameterValue(heading_gain_));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".heading_slow_angle", rclcpp::ParameterValue(heading_slow_angle_));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".heading_slow_min_scale", rclcpp::ParameterValue(heading_slow_min_scale_));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".avoid_gain", rclcpp::ParameterValue(avoid_gain_));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".turn_gain", rclcpp::ParameterValue(turn_gain_));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".turn_deadband", rclcpp::ParameterValue(turn_deadband_));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".range_max", rclcpp::ParameterValue(range_max_));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".lookahead_dist", rclcpp::ParameterValue(lookahead_dist_));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".front_angle", rclcpp::ParameterValue(front_angle_));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".side_angle", rclcpp::ParameterValue(side_angle_));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".angle_step", rclcpp::ParameterValue(angle_step_));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".raycast_step", rclcpp::ParameterValue(raycast_step_));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".cost_threshold", rclcpp::ParameterValue(cost_threshold_));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".treat_unknown_as_obstacle",
    rclcpp::ParameterValue(treat_unknown_as_obstacle_));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".enable_footprint_collision_gate",
    rclcpp::ParameterValue(enable_footprint_collision_gate_));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".collision_gate_front_activation_dist",
    rclcpp::ParameterValue(collision_gate_front_activation_dist_));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".collision_gate_side_activation_dist",
    rclcpp::ParameterValue(collision_gate_side_activation_dist_));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".forward_sim_horizon_sec",
    rclcpp::ParameterValue(forward_sim_horizon_sec_));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".forward_sim_dt",
    rclcpp::ParameterValue(forward_sim_dt_));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".rotate_sim_horizon_sec",
    rclcpp::ParameterValue(rotate_sim_horizon_sec_));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".rotate_sim_dt",
    rclcpp::ParameterValue(rotate_sim_dt_));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".micro_progress_sim_horizon_sec",
    rclcpp::ParameterValue(micro_progress_sim_horizon_sec_));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".micro_progress_sim_dt",
    rclcpp::ParameterValue(micro_progress_sim_dt_));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".micro_progress_front_margin",
    rclcpp::ParameterValue(micro_progress_front_margin_));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".micro_progress_side_margin",
    rclcpp::ParameterValue(micro_progress_side_margin_));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".micro_progress_heading_max",
    rclcpp::ParameterValue(micro_progress_heading_max_));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".footprint_collision_cost_threshold",
    rclcpp::ParameterValue(footprint_collision_cost_threshold_));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".use_pid", rclcpp::ParameterValue(use_pid_));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".pid_kp_lin", rclcpp::ParameterValue(pid_kp_lin_));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".pid_ki_lin", rclcpp::ParameterValue(pid_ki_lin_));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".pid_kd_lin", rclcpp::ParameterValue(pid_kd_lin_));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".pid_kp_ang", rclcpp::ParameterValue(pid_kp_ang_));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".pid_ki_ang", rclcpp::ParameterValue(pid_ki_ang_));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".pid_kd_ang", rclcpp::ParameterValue(pid_kd_ang_));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".pid_i_max_lin", rclcpp::ParameterValue(pid_i_max_lin_));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".pid_i_max_ang", rclcpp::ParameterValue(pid_i_max_ang_));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".pid_dt_max", rclcpp::ParameterValue(pid_dt_max_));

  node_->get_parameter(name_ + ".max_lin", max_lin_);
  node_->get_parameter(name_ + ".max_ang", max_ang_);
  node_->get_parameter(name_ + ".stop_dist", stop_dist_);
  node_->get_parameter(name_ + ".hard_stop_dist", hard_stop_dist_);
  node_->get_parameter(name_ + ".creep_speed", creep_speed_);
  node_->get_parameter(name_ + ".in_place_heading", in_place_heading_);
  node_->get_parameter(name_ + ".in_place_dist", in_place_dist_);
  node_->get_parameter(name_ + ".rotate_min_side_clearance", rotate_min_side_clearance_);
  node_->get_parameter(name_ + ".rotate_min_front_clearance", rotate_min_front_clearance_);
  node_->get_parameter(name_ + ".escape_forward_speed", escape_forward_speed_);
  node_->get_parameter(name_ + ".escape_forward_turn_scale", escape_forward_turn_scale_);
  node_->get_parameter(name_ + ".escape_use_reverse", escape_use_reverse_);
  node_->get_parameter(name_ + ".escape_reverse_speed", escape_reverse_speed_);
  node_->get_parameter(name_ + ".slow_dist", slow_dist_);
  node_->get_parameter(name_ + ".heading_gain", heading_gain_);
  node_->get_parameter(name_ + ".heading_slow_angle", heading_slow_angle_);
  node_->get_parameter(name_ + ".heading_slow_min_scale", heading_slow_min_scale_);
  node_->get_parameter(name_ + ".avoid_gain", avoid_gain_);
  node_->get_parameter(name_ + ".turn_gain", turn_gain_);
  node_->get_parameter(name_ + ".turn_deadband", turn_deadband_);
  node_->get_parameter(name_ + ".range_max", range_max_);
  node_->get_parameter(name_ + ".lookahead_dist", lookahead_dist_);
  node_->get_parameter(name_ + ".front_angle", front_angle_);
  node_->get_parameter(name_ + ".side_angle", side_angle_);
  node_->get_parameter(name_ + ".angle_step", angle_step_);
  node_->get_parameter(name_ + ".raycast_step", raycast_step_);
  node_->get_parameter(name_ + ".cost_threshold", cost_threshold_);
  node_->get_parameter(name_ + ".treat_unknown_as_obstacle", treat_unknown_as_obstacle_);
  node_->get_parameter(
    name_ + ".enable_footprint_collision_gate",
    enable_footprint_collision_gate_);
  node_->get_parameter(
    name_ + ".collision_gate_front_activation_dist",
    collision_gate_front_activation_dist_);
  node_->get_parameter(
    name_ + ".collision_gate_side_activation_dist",
    collision_gate_side_activation_dist_);
  node_->get_parameter(
    name_ + ".forward_sim_horizon_sec",
    forward_sim_horizon_sec_);
  node_->get_parameter(
    name_ + ".forward_sim_dt",
    forward_sim_dt_);
  node_->get_parameter(
    name_ + ".rotate_sim_horizon_sec",
    rotate_sim_horizon_sec_);
  node_->get_parameter(
    name_ + ".rotate_sim_dt",
    rotate_sim_dt_);
  node_->get_parameter(
    name_ + ".micro_progress_sim_horizon_sec",
    micro_progress_sim_horizon_sec_);
  node_->get_parameter(
    name_ + ".micro_progress_sim_dt",
    micro_progress_sim_dt_);
  node_->get_parameter(
    name_ + ".micro_progress_front_margin",
    micro_progress_front_margin_);
  node_->get_parameter(
    name_ + ".micro_progress_side_margin",
    micro_progress_side_margin_);
  node_->get_parameter(
    name_ + ".micro_progress_heading_max",
    micro_progress_heading_max_);
  node_->get_parameter(
    name_ + ".footprint_collision_cost_threshold",
    footprint_collision_cost_threshold_);
  node_->get_parameter(name_ + ".use_pid", use_pid_);
  node_->get_parameter(name_ + ".pid_kp_lin", pid_kp_lin_);
  node_->get_parameter(name_ + ".pid_ki_lin", pid_ki_lin_);
  node_->get_parameter(name_ + ".pid_kd_lin", pid_kd_lin_);
  node_->get_parameter(name_ + ".pid_kp_ang", pid_kp_ang_);
  node_->get_parameter(name_ + ".pid_ki_ang", pid_ki_ang_);
  node_->get_parameter(name_ + ".pid_kd_ang", pid_kd_ang_);
  node_->get_parameter(name_ + ".pid_i_max_lin", pid_i_max_lin_);
  node_->get_parameter(name_ + ".pid_i_max_ang", pid_i_max_ang_);
  node_->get_parameter(name_ + ".pid_dt_max", pid_dt_max_);

  base_max_lin_ = max_lin_;
  base_max_ang_ = max_ang_;
  pid_last_time_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());

  // PID 목표 속도 퍼블리셔(학습/디버깅용)
  const std::string desired_topic =
    std::string("/") + node_->get_name() + "/" + name_ + "/desired_cmd";
  desired_cmd_pub_ =
    node_->create_publisher<geometry_msgs::msg::TwistStamped>(desired_topic, 10);

  // PID 파라미터 동적 업데이트 콜백 등록
  param_cb_handle_ = node_->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & params)
    -> rcl_interfaces::msg::SetParametersResult {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      for (const auto & p : params) {
        const auto & n = p.get_name();
        if (n == name_ + ".pid_kp_lin") {
          pid_kp_lin_ = p.as_double();
        } else if (n == name_ + ".pid_ki_lin") {
          pid_ki_lin_ = p.as_double();
        } else if (n == name_ + ".pid_kd_lin") {
          pid_kd_lin_ = p.as_double();
        } else if (n == name_ + ".pid_kp_ang") {
          pid_kp_ang_ = p.as_double();
        } else if (n == name_ + ".pid_ki_ang") {
          pid_ki_ang_ = p.as_double();
        } else if (n == name_ + ".pid_kd_ang") {
          pid_kd_ang_ = p.as_double();
        }
      }
      return result;
    });
}

void RlLocalController::cleanup()
{
  // 별도 자원 정리가 필요하지 않다.
}

void RlLocalController::activate()
{
  // 활성화 단계에서 추가 동작은 없다.
}

void RlLocalController::deactivate()
{
  // 비활성화 단계에서 추가 동작은 없다.
}

void RlLocalController::setPlan(const nav_msgs::msg::Path & path)
{
  // 최신 전역 경로를 저장한다.
  plan_ = path;
  // 새 경로를 받으면 근거리 정렬 상태를 초기화한다.
  align_in_place_mode_ = false;
  last_turn_dir_ = 0;
}

geometry_msgs::msg::TwistStamped RlLocalController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker *)
{
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.stamp = node_->now();
  cmd.header.frame_id = costmap_ros_->getBaseFrameID();

  if (plan_.poses.empty()) {
    // 경로가 없으면 정지한다.
    align_in_place_mode_ = false;
    return cmd;
  }

  // 목표(경로 끝점)까지의 거리: 근거리에서 목표점 직접 추종/제자리 회전 판단에 사용
  const auto & goal_pose = plan_.poses.back().pose.position;
  const double goal_dx = goal_pose.x - pose.pose.position.x;
  const double goal_dy = goal_pose.y - pose.pose.position.y;
  const double dist_to_goal = std::hypot(goal_dx, goal_dy);

  double front = range_max_;
  double left = range_max_;
  double right = range_max_;
  computeSectorDistances(pose, front, left, right);

  double target_x = 0.0;
  double target_y = 0.0;
  const bool near_goal = dist_to_goal <= std::max(0.35, in_place_dist_);
  if (near_goal) {
    // 근거리에서는 경로 중간점(룩어헤드)을 쫓지 않고
    // 최종 목표점을 직접 보게 해서 "목표 주변 원운동"을 줄인다.
    target_x = goal_pose.x;
    target_y = goal_pose.y;
  } else {
    // 목표가 가까워질수록 룩어헤드를 줄여 불필요한 큰 원호 주행을 억제한다.
    // (멀리 있을 때는 충분히 크게 유지해 경로를 부드럽게 따라간다.)
    const double goal_lookahead =
      std::min(lookahead_dist_, std::max(0.35, dist_to_goal * 0.8));
    // 전방 장애물이 가까우면 룩어헤드를 더 줄여 "완만 곡선 후 급턴"을 방지한다.
    const double obstacle_lookahead =
      clamp(front * 0.9, 0.30, lookahead_dist_);
    const double adaptive_lookahead =
      std::min(goal_lookahead, obstacle_lookahead);
    if (!getLookaheadTarget(pose, adaptive_lookahead, target_x, target_y)) {
      return cmd;
    }
  }

  const double yaw = getYaw(pose);
  const double dx = target_x - pose.pose.position.x;
  const double dy = target_y - pose.pose.position.y;
  const double dist_to_target = std::hypot(dx, dy);
  const double heading_error = normalizeAngle(std::atan2(dy, dx) - yaw);
  const double heading_abs = std::abs(heading_error);

  // 근거리 목표에서는 heading 정렬을 우선해 제자리 회전 모드로 진입/해제한다.
  // enter/exit 임계를 분리(히스테리시스)해 경계에서 모드가 반복 전환되며
  // 속도가 떨리는 현상을 줄인다.
  const bool close_target = dist_to_target <= std::max(0.9, in_place_dist_ * 1.8);
  const double align_enter = close_target ?
    std::max(0.30, in_place_heading_ * 0.45) :
    std::max(0.90, in_place_heading_ * 0.90);
  const double align_exit = close_target ?
    std::max(0.12, align_enter * 0.45) :
    std::max(0.30, align_enter * 0.45);
  if (align_in_place_mode_) {
    if (heading_abs < align_exit) {
      align_in_place_mode_ = false;
    }
  } else if (close_target && heading_abs > align_enter) {
    align_in_place_mode_ = true;
  }

  double v_des = 0.0;
  double w_des = 0.0;

  // 좌우 차이 데드밴드로 진동을 줄인다.
  const double lr_diff = left - right;
  const double side_min = std::min(left, right);
  const bool can_in_place_rotate =
    side_min >= rotate_min_side_clearance_ &&
    front >= rotate_min_front_clearance_;

  if (align_in_place_mode_ && !can_in_place_rotate) {
    // 좁은 통로에서는 즉시 제자리 회전을 금지하고 탈출 기동으로 여유 공간을 만든다.
    const bool front_has_room = front > std::max(stop_dist_ + 0.05, hard_stop_dist_ + 0.10);
    if (front_has_room) {
      v_des = std::max(0.0, escape_forward_speed_);
      // 더 넓은 쪽으로 약하게 틀어 측면 여유를 확보한다.
      double turn_dir = 0.0;
      if (std::abs(lr_diff) >= turn_deadband_) {
        turn_dir = (lr_diff > 0.0) ? 1.0 : -1.0;
      } else if (last_turn_dir_ != 0) {
        turn_dir = static_cast<double>(last_turn_dir_);
      }
      if (std::abs(turn_dir) > 0.0) {
        w_des = turn_dir * min_turn_rate_ * clamp(escape_forward_turn_scale_, 0.0, 1.0);
      }
    } else if (escape_use_reverse_) {
      // 전방도 막혀 있으면 매우 저속 후진+회전으로 공간을 다시 확보한다.
      v_des = -std::max(0.0, escape_reverse_speed_);
      double turn_dir = 0.0;
      if (std::abs(lr_diff) >= turn_deadband_) {
        turn_dir = (lr_diff > 0.0) ? 1.0 : -1.0;
      } else {
        turn_dir = heading_error >= 0.0 ? 1.0 : -1.0;
      }
      last_turn_dir_ = (turn_dir >= 0.0) ? 1 : -1;
      w_des = turn_dir * min_turn_rate_;
    } else {
      // 후진 탈출을 비활성화한 경우에는 정지해 상위 복구(BackUp 등)에 맡긴다.
      v_des = 0.0;
      w_des = 0.0;
    }
  } else if (align_in_place_mode_ && front > hard_stop_dist_) {
    // 근거리 정렬 모드: 선속도를 완전히 차단해 원운동 대신 제자리 회전으로 맞춘다.
    v_des = 0.0;
    const double turn_dir = heading_error >= 0.0 ? 1.0 : -1.0;
    // 오차가 작아질수록 각속도도 함께 줄여 목표 방위 근처에서 과회전을 줄인다.
    const double turn_scale = clamp(
      heading_abs / std::max(align_enter, 1e-3), 0.35, 1.0);
    w_des = turn_dir * std::max(min_turn_rate_, turn_gain_ * max_ang_ * turn_scale);
    w_des = clamp(w_des, -max_ang_, max_ang_);
    last_turn_dir_ = (turn_dir >= 0.0) ? 1 : -1;
  } else   if (front < stop_dist_) {
    // 전방이 막혀도 아주 천천히 전진해 재탐색을 유도한다.
    const double hard_stop = std::min(hard_stop_dist_, stop_dist_ * 0.95);
    const double denom = std::max(1e-3, stop_dist_ - hard_stop);
    double creep_scale = 0.0;
    if (front > hard_stop) {
      creep_scale = clamp((front - hard_stop) / denom, 0.0, 1.0);
    }
    v_des = creep_speed_ * creep_scale;

    // 여유가 큰 쪽을 우선해 회전한다.
    double turn_dir = 0.0;

    if (std::abs(lr_diff) < turn_deadband_) {
      // 좌우 차이가 작으면 목표 방향을 기준으로 회전한다.
      if (std::abs(heading_error) > 0.2) {
        turn_dir = heading_error >= 0.0 ? 1.0 : -1.0;
      } else if (last_turn_dir_ != 0) {
        turn_dir = static_cast<double>(last_turn_dir_);
      } else {
        turn_dir = 1.0;
      }
    } else {
      turn_dir = (lr_diff > 0.0) ? 1.0 : -1.0;
    }

    last_turn_dir_ = (turn_dir >= 0.0) ? 1 : -1;
    w_des = turn_dir * turn_gain_ * max_ang_;
  } else {
    // 전방 여유에 따라 전진 속도를 조절한다.
    const double speed_scale = clamp(front / slow_dist_, 0.0, 1.0);
    v_des = max_lin_ * speed_scale;

    // heading 오차가 큰 경우에는 전진 속도를 추가로 줄여 회전 중심을 만들고
    // 짧은 거리 목표에서 "원형으로 도는" 현상을 완화한다.
    if (heading_abs > heading_slow_angle_) {
      const double over = clamp((heading_abs - heading_slow_angle_) /
        std::max(1e-3, M_PI - heading_slow_angle_), 0.0, 1.0);
      const double heading_scale = clamp(1.0 - over, heading_slow_min_scale_, 1.0);
      v_des *= heading_scale;
    }
    if (close_target && heading_abs > align_exit) {
      // 목표 근처에서 heading 오차가 남아 있으면 전진을 강하게 억제한다.
      // 정렬 모드 진입 직전/직후의 원호 주행을 줄이기 위한 보조 안전장치다.
      const double align_scale =
        clamp(1.0 - heading_abs / std::max(align_enter, 1e-3), 0.0, 1.0);
      v_des *= align_scale;
      v_des = std::min(v_des, creep_speed_ * 1.5);
    }

    // 전방 장애물이 가까우면 회전 성분을 선제적으로 키운다.
    const double near_obstacle_band = stop_dist_ + 0.35;
    const double proximity = clamp(
      (near_obstacle_band - front) /
      std::max(1e-3, near_obstacle_band - hard_stop_dist_),
      0.0, 1.0);
    const double heading_boost = 1.0 + 0.9 * proximity;
    const double avoid_boost = 1.0 + 1.2 * proximity;

    // 목표 방향과 장애물 회피를 합쳐 회전 속도를 만든다.
    const double heading_term =
      heading_gain_ * heading_boost * clamp(heading_error / M_PI, -1.0, 1.0);
    double avoid_term = 0.0;
    if (std::abs(lr_diff) >= turn_deadband_) {
      avoid_term = avoid_gain_ * avoid_boost * clamp(lr_diff / range_max_, -1.0, 1.0);
    }
    w_des = clamp((heading_term + avoid_term) * max_ang_, -max_ang_, max_ang_);

    // 근접 장애물 구간에서는 회전 최소각속도를 보장하고 전진을 제한해 접촉을 줄인다.
    if (proximity > 0.0 && heading_abs > 0.20) {
      const double turn_dir = heading_error >= 0.0 ? 1.0 : -1.0;
      const double min_w = min_turn_rate_ * (1.0 + 0.8 * proximity);
      if (std::abs(w_des) < min_w) {
        w_des = turn_dir * min_w;
      }

      const double near_obs_speed_cap =
        std::max(creep_speed_ * 1.5, 0.12 * (1.0 - 0.7 * proximity));
      v_des = std::min(v_des, near_obs_speed_cap);
      if (front < stop_dist_ + 0.10 && heading_abs > 0.35) {
        v_des = 0.0;
      }
    }

    if (std::abs(w_des) > 1e-3) {
      last_turn_dir_ = (w_des >= 0.0) ? 1 : -1;
    }
  }

  // 2차 안전 게이트:
  // 기존 RL 로직은 중심 raycast 기반으로 후보 명령을 만든다.
  // 이번 버전에서는 아무 상황에서나 이 gate를 강하게 돌리지 않는다.
  // 자유 공간에서는 원래 RL 행동을 유지하고,
  // 장애물이 충분히 가까운 구간에서만 "실제 차체 기준" 검사를 개입시킨다.
  if (enable_footprint_collision_gate_ &&
    shouldApplyCollisionGate(front, left, right, v_des, w_des))
  {
    int preferred_turn_dir = 0;
    if (std::abs(lr_diff) >= turn_deadband_) {
      preferred_turn_dir = (lr_diff > 0.0) ? 1 : -1;
    } else if (std::abs(w_des) > 1e-3) {
      preferred_turn_dir = (w_des >= 0.0) ? 1 : -1;
    } else if (std::abs(heading_error) > 1e-3) {
      preferred_turn_dir = (heading_error >= 0.0) ? 1 : -1;
    } else if (last_turn_dir_ != 0) {
      preferred_turn_dir = last_turn_dir_;
    } else {
      preferred_turn_dir = 1;
    }

    const double original_v = v_des;
    const double original_w = w_des;
    const bool found_safe_command =
      selectCollisionFreeCommand(
      pose, front, left, right, heading_abs,
      preferred_turn_dir, original_v, original_w, v_des, w_des);

    if (!found_safe_command) {
      RCLCPP_WARN_THROTTLE(
        logger_, *node_->get_clock(), 2000,
        "footprint collision gate: 후보 명령(v=%.3f, w=%.3f)을 차단했고, 안전한 대체 명령을 찾지 못해 정지합니다.",
        original_v, original_w);
    } else if (
      std::abs(v_des - original_v) > 1e-3 ||
      std::abs(w_des - original_w) > 1e-3)
    {
      RCLCPP_INFO_THROTTLE(
        logger_, *node_->get_clock(), 2000,
        "footprint collision gate: 후보 명령(v=%.3f, w=%.3f)을 더 안전한 명령(v=%.3f, w=%.3f)으로 조정했습니다.",
        original_v, original_w, v_des, w_des);
    }
  }

  // 최종적으로 채택된 회전 방향을 저장해 다음 프레임에서 흔들림을 줄인다.
  if (std::abs(w_des) > 1e-3) {
    last_turn_dir_ = (w_des >= 0.0) ? 1 : -1;
  }

  // PID 이전 단계에서 목표 속도 발행(학습/디버깅용)
  if (desired_cmd_pub_) {
    geometry_msgs::msg::TwistStamped desired;
    desired.header = cmd.header;
    desired.twist.linear.x = v_des;
    desired.twist.angular.z = w_des;
    desired_cmd_pub_->publish(desired);
  }

  // PID를 쓰지 않으면 바로 목표 속도를 출력한다.
  if (!use_pid_) {
    cmd.twist.linear.x = v_des;
    cmd.twist.angular.z = w_des;
    return cmd;
  }

  // PID 기반으로 현재 속도와 목표 속도 차이를 보정한다.
  const auto now = node_->now();
  if (pid_last_time_.nanoseconds() == 0) {
    pid_last_time_ = now;
    pid_prev_err_lin_ = v_des - velocity.linear.x;
    pid_prev_err_ang_ = w_des - velocity.angular.z;
    pid_i_lin_ = 0.0;
    pid_i_ang_ = 0.0;
    cmd.twist.linear.x = v_des;
    cmd.twist.angular.z = w_des;
    return cmd;
  }

  const double dt = (now - pid_last_time_).seconds();
  if (dt <= 0.0 || dt > pid_dt_max_) {
    // 시간 간격이 비정상이면 적분을 초기화하고 목표 속도를 바로 사용한다.
    pid_last_time_ = now;
    pid_prev_err_lin_ = v_des - velocity.linear.x;
    pid_prev_err_ang_ = w_des - velocity.angular.z;
    pid_i_lin_ = 0.0;
    pid_i_ang_ = 0.0;
    cmd.twist.linear.x = v_des;
    cmd.twist.angular.z = w_des;
    return cmd;
  }

  const double err_lin = v_des - velocity.linear.x;
  const double err_ang = w_des - velocity.angular.z;
  pid_i_lin_ = clamp(pid_i_lin_ + err_lin * dt, -pid_i_max_lin_, pid_i_max_lin_);
  pid_i_ang_ = clamp(pid_i_ang_ + err_ang * dt, -pid_i_max_ang_, pid_i_max_ang_);
  const double d_lin = (err_lin - pid_prev_err_lin_) / dt;
  const double d_ang = (err_ang - pid_prev_err_ang_) / dt;

  const double corr_lin =
    pid_kp_lin_ * err_lin + pid_ki_lin_ * pid_i_lin_ + pid_kd_lin_ * d_lin;
  const double corr_ang =
    pid_kp_ang_ * err_ang + pid_ki_ang_ * pid_i_ang_ + pid_kd_ang_ * d_ang;

  double v_out = velocity.linear.x + corr_lin;
  double w_out = velocity.angular.z + corr_ang;

  // 최종 출력은 최대 속도 제한을 따른다.
  v_out = clamp(v_out, -max_lin_, max_lin_);
  w_out = clamp(w_out, -max_ang_, max_ang_);

  pid_prev_err_lin_ = err_lin;
  pid_prev_err_ang_ = err_ang;
  pid_last_time_ = now;

  cmd.twist.linear.x = v_out;
  cmd.twist.angular.z = w_out;
  return cmd;
}

void RlLocalController::simulatePoseStep(
  double & x,
  double & y,
  double & yaw,
  double v,
  double w,
  double dt) const
{
  // 아주 짧은 구간에서는 단순 오일러보다 중간 heading을 쓰는 편이
  // 회전 중 전진 경로를 조금 더 안정적으로 근사한다.
  const double mid_yaw = yaw + 0.5 * w * dt;
  x += std::cos(mid_yaw) * v * dt;
  y += std::sin(mid_yaw) * v * dt;
  yaw = normalizeAngle(yaw + w * dt);
}

bool RlLocalController::isPoseCollisionFree(double x, double y, double yaw) const
{
  auto * costmap = costmap_ros_->getCostmap();
  if (costmap == nullptr) {
    return false;
  }

  // 현재 costmap에 이미 적용된 padded footprint를 그대로 사용한다.
  // 즉 운영 파라미터(footprint_padding)의 효과가 collision gate에도 동일하게 반영된다.
  const auto footprint = costmap_ros_->getRobotFootprint();
  if (footprint.size() < 3) {
    return false;
  }

  nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> checker(costmap);
  const double footprint_cost = checker.footprintCostAtPose(x, y, yaw, footprint);

  // threshold 이상이면 실제 차체 외곽이 lethal / inscribed / unknown 고비용 영역과
  // 닿는다고 보고 통과 불가로 판정한다.
  return footprint_cost < static_cast<double>(footprint_collision_cost_threshold_);
}

bool RlLocalController::evaluateCommandCandidate(
  const geometry_msgs::msg::PoseStamped & pose,
  double v,
  double w,
  double horizon_override_sec,
  double dt_override_sec,
  double & predicted_x,
  double & predicted_y,
  double & predicted_yaw) const
{
  if (!std::isfinite(v) || !std::isfinite(w)) {
    return false;
  }

  double horizon_sec = 0.0;
  double dt = 0.0;
  if (horizon_override_sec > 1e-6 && dt_override_sec > 1e-6) {
    // 일부 deadlock 해소 후보는 기본 forward 후보보다 더 짧은 구간만 본다.
    // 이렇게 하면 "아주 짧게는 안전한" 미세 전진 후보를 과도하게 죽이지 않을 수 있다.
    horizon_sec = horizon_override_sec;
    dt = dt_override_sec;
  } else {
    getSimulationWindowForCommand(v, w, horizon_sec, dt);
  }
  dt = std::max(0.02, dt);
  const int steps = std::max(1, static_cast<int>(std::ceil(
    std::max(dt, horizon_sec) / dt)));

  // 현재 pose는 이미 컨트롤러가 서 있는 위치이므로,
  // 여기서는 "지금 이 명령을 내보냈을 때 다음 순간부터 부딪히는가"를 본다.
  double x = pose.pose.position.x;
  double y = pose.pose.position.y;
  double yaw = getYaw(pose);

  for (int i = 0; i < steps; ++i) {
    simulatePoseStep(x, y, yaw, v, w, dt);
    if (!isPoseCollisionFree(x, y, yaw)) {
      return false;
    }
  }

  // 마지막 예측 pose를 돌려줘 progress score 계산에 재사용한다.
  predicted_x = x;
  predicted_y = y;
  predicted_yaw = yaw;
  return true;
}

bool RlLocalController::shouldApplyCollisionGate(
  double front,
  double left,
  double right,
  double v,
  double w) const
{
  // gate를 항상 켜면 자유공간에서도 전진 후보를 지나치게 많이 죽일 수 있다.
  // 따라서 실제 장애물이 가까운 상황에서만 차체 기반 정밀 검사를 수행한다.
  const double side_min = std::min(left, right);
  const bool near_front_obstacle = front <= collision_gate_front_activation_dist_;
  const bool near_side_obstacle = side_min <= collision_gate_side_activation_dist_;
  const bool translating = std::abs(v) > 1e-3;
  const bool turning = std::abs(w) > min_turn_rate_ * 0.5;

  if (near_front_obstacle || near_side_obstacle) {
    return true;
  }

  // 자유공간에서 아주 약한 회전/전진까지 게이트를 걸면 progress가 쉽게 죽는다.
  // 반대로 큰 병진/회전이 같이 들어가는 명령은 차체 sweep이 커질 수 있어 계속 검사한다.
  return translating && turning;
}

void RlLocalController::getSimulationWindowForCommand(
  double v,
  double w,
  double & horizon_sec,
  double & dt) const
{
  const bool translating = std::abs(v) > 1e-3;

  if (translating) {
    // 전진/후진 명령은 실제 접촉까지 이어질 수 있으므로 조금 더 길게 본다.
    horizon_sec = forward_sim_horizon_sec_;
    dt = forward_sim_dt_;
  } else {
    // 제자리 회전은 너무 긴 horizon을 주면 불필요하게 보수적이 되고,
    // 좌우 짧은 흔들림만 늘 수 있어 더 짧은 구간만 본다.
    horizon_sec = rotate_sim_horizon_sec_;
    dt = rotate_sim_dt_;
  }
}

double RlLocalController::scoreSafeCandidate(
  const geometry_msgs::msg::PoseStamped & pose,
  double predicted_x,
  double predicted_y,
  double predicted_yaw,
  double candidate_v,
  double candidate_w,
  double preferred_turn_dir,
  double original_v,
  double original_w) const
{
  // progress-friendly score:
  // 1) 실제로 goal/lookahead 쪽 거리를 줄이는 후보를 우선
  // 2) heading 오차를 줄이는 후보를 우선
  // 3) 그래도 전진성이 있는 후보를 약간 우선
  // 4) 후진은 최후 수단이므로 강하게 감점
  double score = 0.0;
  const double current_x = pose.pose.position.x;
  const double current_y = pose.pose.position.y;
  const double current_yaw = getYaw(pose);

  double target_x = 0.0;
  double target_y = 0.0;
  if (!getLookaheadTarget(pose, lookahead_dist_, target_x, target_y)) {
    if (!plan_.poses.empty()) {
      target_x = plan_.poses.back().pose.position.x;
      target_y = plan_.poses.back().pose.position.y;
    } else {
      target_x = current_x;
      target_y = current_y;
    }
  }

  const double current_goal_dist = std::hypot(target_x - current_x, target_y - current_y);
  const double predicted_goal_dist = std::hypot(target_x - predicted_x, target_y - predicted_y);
  const double progress_gain = current_goal_dist - predicted_goal_dist;

  const double current_heading_abs = std::abs(normalizeAngle(
      std::atan2(target_y - current_y, target_x - current_x) - current_yaw));
  const double predicted_heading_abs = std::abs(normalizeAngle(
      std::atan2(target_y - predicted_y, target_x - predicted_x) - predicted_yaw));
  const double heading_gain_score = current_heading_abs - predicted_heading_abs;

  // "실제 얼마나 goal 쪽으로 나아갔는가"를 가장 큰 항으로 둔다.
  score += 24.0 * progress_gain;
  // heading이 좋아지는 후보도 가점하되, 전진 거리보다 조금 약하게 둔다.
  score += 3.0 * heading_gain_score;

  if (candidate_v > 0.0) {
    score += 3.0 * (candidate_v / std::max(0.05, max_lin_));
  } else if (candidate_v < 0.0) {
    score -= 8.0;
  } else {
    score -= 1.5;
  }

  if (std::abs(candidate_w) > 1e-3 && preferred_turn_dir != 0.0) {
    if ((candidate_w > 0.0 && preferred_turn_dir > 0.0) ||
      (candidate_w < 0.0 && preferred_turn_dir < 0.0))
    {
      score += 2.0;
    } else {
      score -= 2.0;
    }
  }

  // 원래 명령과 너무 멀리 벌어진 후보는 progress와 자연스러운 추종을 해칠 수 있으므로 소폭 감점.
  score -= 0.8 * std::abs(candidate_v - original_v) / std::max(0.05, max_lin_);
  score -= 0.4 * std::abs(candidate_w - original_w) / std::max(0.1, max_ang_);

  return score;
}

bool RlLocalController::selectCollisionFreeCommand(
  const geometry_msgs::msg::PoseStamped & pose,
  double front,
  double left,
  double right,
  double heading_abs,
  int preferred_turn_dir,
  double original_v,
  double original_w,
  double & v,
  double & w) const
{
  struct CandidateSpec
  {
    double v{0.0};
    double w{0.0};
    double horizon_override_sec{0.0};
    double dt_override_sec{0.0};
  };

  const int turn_dir = preferred_turn_dir >= 0 ? 1 : -1;
  const double desired_turn_mag = clamp(
    std::max(std::abs(original_w), min_turn_rate_), min_turn_rate_, max_ang_);
  const double side_min = std::min(left, right);
  const bool allow_micro_progress =
    original_v > 0.0 &&
    front > (hard_stop_dist_ + micro_progress_front_margin_) &&
    side_min > (rotate_min_side_clearance_ + micro_progress_side_margin_) &&
    heading_abs < micro_progress_heading_max_;

  // 이번 버전은 "첫 번째로 안전한 후보"를 바로 고르지 않는다.
  // 안전한 후보를 모두 모은 뒤, 그중 실제 goal 쪽 진행량이 더 큰
  // progress-friendly 후보를 고른다.
  std::vector<CandidateSpec> candidates;
  candidates.reserve(9);
  candidates.push_back({original_v, original_w, 0.0, 0.0});

  // deadlock 해소용 micro-progress 후보:
  // 원래 명령이 막혔을 때도 "조금만 더 신중하게 전진하면 풀릴 수 있는" 경우를 살리기 위해,
  // 매우 제한된 조건에서만 1~2개의 짧은 전진 후보를 추가한다.
  if (allow_micro_progress) {
    const double micro_v_fast = clamp(
      std::max(creep_speed_, 0.40 * original_v),
      0.0, std::min(max_lin_, 0.12));
    const double micro_v_slow = clamp(
      std::max(creep_speed_, 0.25 * original_v),
      0.0, std::min(max_lin_, 0.08));
    const double micro_w_hold = clamp(original_w, -max_ang_, max_ang_);
    const double micro_w_turn = clamp(
      std::abs(original_w) > 1e-3 ? 1.2 * original_w :
      static_cast<double>(turn_dir) * min_turn_rate_,
      -max_ang_, max_ang_);

    candidates.push_back(
      {micro_v_fast, micro_w_hold, micro_progress_sim_horizon_sec_, micro_progress_sim_dt_});
    candidates.push_back(
      {micro_v_slow, micro_w_turn, micro_progress_sim_horizon_sec_, micro_progress_sim_dt_});
  }

  candidates.push_back({0.0, turn_dir * desired_turn_mag, 0.0, 0.0});
  candidates.push_back({0.0, turn_dir * min_turn_rate_, 0.0, 0.0});
  candidates.push_back({0.0, -turn_dir * desired_turn_mag, 0.0, 0.0});
  candidates.push_back({0.0, -turn_dir * min_turn_rate_, 0.0, 0.0});
  candidates.push_back({-std::max(0.0, escape_reverse_speed_), turn_dir * min_turn_rate_, 0.0, 0.0});
  candidates.push_back({-std::max(0.0, escape_reverse_speed_), -turn_dir * min_turn_rate_, 0.0, 0.0});

  bool found_safe_candidate = false;
  double best_score = -std::numeric_limits<double>::infinity();
  double best_v = 0.0;
  double best_w = 0.0;

  for (const auto & candidate : candidates) {
    if (!escape_use_reverse_ && candidate.v < 0.0) {
      continue;
    }
    double predicted_x = pose.pose.position.x;
    double predicted_y = pose.pose.position.y;
    double predicted_yaw = getYaw(pose);
    if (evaluateCommandCandidate(
        pose, candidate.v, candidate.w,
        candidate.horizon_override_sec, candidate.dt_override_sec,
        predicted_x, predicted_y, predicted_yaw))
    {
      const double score = scoreSafeCandidate(
        pose,
        predicted_x, predicted_y, predicted_yaw,
        candidate.v, candidate.w,
        static_cast<double>(turn_dir), original_v, original_w);

      if (!found_safe_candidate || score > best_score) {
        found_safe_candidate = true;
        best_score = score;
        best_v = candidate.v;
        best_w = candidate.w;
      }
    }
  }

  if (found_safe_candidate) {
    v = best_v;
    w = best_w;
    return true;
  }

  // 모든 대안이 막히면 안전을 위해 정지한다.
  v = 0.0;
  w = 0.0;
  return false;
}

void RlLocalController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  // 속도 제한을 선형/각속도에 동시에 반영한다.
  if (percentage) {
    const double scale = clamp(speed_limit / 100.0, 0.0, 1.0);
    max_lin_ = base_max_lin_ * scale;
    max_ang_ = base_max_ang_ * scale;
    return;
  }

  max_lin_ = std::max(0.0, std::min(speed_limit, base_max_lin_));
}

double RlLocalController::clamp(double x, double lo, double hi) const
{
  if (x < lo) {
    return lo;
  }
  if (x > hi) {
    return hi;
  }
  return x;
}

double RlLocalController::normalizeAngle(double a) const
{
  while (a > M_PI) {
    a -= 2.0 * M_PI;
  }
  while (a < -M_PI) {
    a += 2.0 * M_PI;
  }
  return a;
}

double RlLocalController::getYaw(const geometry_msgs::msg::PoseStamped & pose) const
{
  tf2::Quaternion q;
  tf2::fromMsg(pose.pose.orientation, q);
  return tf2::getYaw(q);
}

bool RlLocalController::getLookaheadTarget(
  const geometry_msgs::msg::PoseStamped & pose,
  double lookahead_dist,
  double & target_x,
  double & target_y) const
{
  if (plan_.poses.empty()) {
    return false;
  }

  const double px = pose.pose.position.x;
  const double py = pose.pose.position.y;

  // 1) 현재 로봇과 가장 가까운 경로 점을 찾는다.
  //    (경로 시작점부터 단순 탐색하면 로봇 뒤쪽 점이 선택될 수 있어 방향이 틀어질 수 있음)
  std::size_t nearest_idx = 0;
  double nearest_dist = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < plan_.poses.size(); ++i) {
    const double dx = plan_.poses[i].pose.position.x - px;
    const double dy = plan_.poses[i].pose.position.y - py;
    const double dist = std::hypot(dx, dy);
    if (dist < nearest_dist) {
      nearest_dist = dist;
      nearest_idx = i;
    }
  }

  // 2) 최근접 점부터 "경로 진행 방향"으로 누적 거리를 더해
  //    룩어헤드 타깃을 계산한다. 세그먼트 중간이면 보간해 좌표를 만든다.
  double accumulated = 0.0;
  for (std::size_t i = nearest_idx; i + 1 < plan_.poses.size(); ++i) {
    const auto & p0 = plan_.poses[i].pose.position;
    const auto & p1 = plan_.poses[i + 1].pose.position;
    const double seg = std::hypot(p1.x - p0.x, p1.y - p0.y);
    if (accumulated + seg >= lookahead_dist) {
      const double remain = std::max(0.0, lookahead_dist - accumulated);
      const double ratio = seg > 1e-6 ? (remain / seg) : 0.0;
      target_x = p0.x + (p1.x - p0.x) * ratio;
      target_y = p0.y + (p1.y - p0.y) * ratio;
      return true;
    }
    accumulated += seg;
  }

  // 룩어헤드 거리가 부족하면 마지막 목표를 사용한다.
  const auto & last = plan_.poses.back();
  target_x = last.pose.position.x;
  target_y = last.pose.position.y;
  return true;
}

double RlLocalController::raycastDistance(
  const geometry_msgs::msg::PoseStamped & pose,
  double angle) const
{
  const auto * costmap = costmap_ros_->getCostmap();
  const double yaw = getYaw(pose);
  const double world_angle = yaw + angle;
  const double start_x = pose.pose.position.x;
  const double start_y = pose.pose.position.y;

  for (double dist = 0.0; dist <= range_max_; dist += raycast_step_) {
    const double wx = start_x + std::cos(world_angle) * dist;
    const double wy = start_y + std::sin(world_angle) * dist;

    unsigned int mx = 0;
    unsigned int my = 0;
    if (!costmap->worldToMap(wx, wy, mx, my)) {
      // 맵 바깥은 장애물로 간주한다.
      return dist;
    }

    const unsigned char cost = costmap->getCost(mx, my);
    if (cost == nav2_costmap_2d::NO_INFORMATION) {
      if (treat_unknown_as_obstacle_) {
        return dist;
      }
      continue;
    }
    if (static_cast<int>(cost) >= cost_threshold_) {
      return dist;
    }
  }

  return range_max_;
}

void RlLocalController::computeSectorDistances(
  const geometry_msgs::msg::PoseStamped & pose,
  double & front,
  double & left,
  double & right) const
{
  front = range_max_;
  left = range_max_;
  right = range_max_;

  for (double a = -side_angle_; a <= side_angle_; a += angle_step_) {
    const double dist = raycastDistance(pose, a);
    if (a >= -front_angle_ && a <= front_angle_) {
      front = std::min(front, dist);
    } else if (a > front_angle_) {
      left = std::min(left, dist);
    } else {
      right = std::min(right, dist);
    }
  }
}

}  // namespace rl_local_controller

PLUGINLIB_EXPORT_CLASS(rl_local_controller::RlLocalController, nav2_core::Controller)
