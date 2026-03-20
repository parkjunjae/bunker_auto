#pragma once

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/time.hpp"
#include "tf2_ros/buffer.h"

namespace rl_local_controller
{

class RlLocalController : public nav2_core::Controller
{
public:
  RlLocalController() = default;
  ~RlLocalController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  void setPlan(const nav_msgs::msg::Path & path) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
  // 값 범위를 제한한다.
  double clamp(double x, double lo, double hi) const;

  // 각도를 -pi~pi로 정규화한다.
  double normalizeAngle(double a) const;

  // 로봇 자세에서 요 각도를 계산한다.
  double getYaw(const geometry_msgs::msg::PoseStamped & pose) const;

  // 전역 경로에서 룩어헤드 목표점을 찾는다.
  bool getLookaheadTarget(
    const geometry_msgs::msg::PoseStamped & pose,
    // 호출 시점의 상황(목표 거리)에 맞게 동적으로 계산된 룩어헤드 거리
    double lookahead_dist,
    double & target_x,
    double & target_y) const;

  // 지정한 각도로 레이캐스팅해 장애물까지의 거리를 구한다.
  double raycastDistance(
    const geometry_msgs::msg::PoseStamped & pose,
    double angle) const;

  // 전방/좌/우 섹터의 최소 장애물 거리를 계산한다.
  void computeSectorDistances(
    const geometry_msgs::msg::PoseStamped & pose,
    double & front,
    double & left,
    double & right) const;

  // 현재 후보 명령(v, w)을 짧은 시간 앞으로 적분해 다음 pose를 계산한다.
  // RL controller의 기본 판단은 중심 raycast이므로, 실제 차체가 닿는지 보려면
  // 명령을 미래로 조금씩 전개해 보는 단계가 추가로 필요하다.
  void simulatePoseStep(
    double & x,
    double & y,
    double & yaw,
    double v,
    double w,
    double dt) const;

  // 특정 pose에 현재 padded footprint를 올렸을 때 costmap과 충돌하는지 검사한다.
  // 중심점이 아니라 실제 차체 polygon 기준으로 안전 여부를 판단하는 핵심 함수다.
  bool isPoseCollisionFree(double x, double y, double yaw) const;

  // 하나의 후보 명령을 여러 스텝 앞까지 시뮬레이션하면서,
  // 중간 어느 pose라도 차체 polygon이 충돌하면 false를 반환한다.
  // 추가로 마지막 예측 pose를 함께 돌려줘 progress score 계산에 재사용한다.
  bool evaluateCommandCandidate(
    const geometry_msgs::msg::PoseStamped & pose,
    double v,
    double w,
    double horizon_override_sec,
    double dt_override_sec,
    double & predicted_x,
    double & predicted_y,
    double & predicted_yaw) const;

  // 현재 상황에서 2차 collision gate를 실제로 켜야 하는지 결정한다.
  // 핵심 아이디어는 "항상 정밀 검사를 돌리지 말고, 장애물이 충분히 가까운 구간에서만
  // 차체 기반 검사를 강하게 개입시키자"는 것이다.
  bool shouldApplyCollisionGate(
    double front,
    double left,
    double right,
    double v,
    double w) const;

  // 명령 종류(전진/회전)에 따라 서로 다른 시뮬레이션 horizon과 dt를 사용한다.
  // 전진은 더 길게, 제자리 회전은 더 짧게 봐서 진행성과 계산량을 함께 맞춘다.
  void getSimulationWindowForCommand(
    double v,
    double w,
    double & horizon_sec,
    double & dt) const;

  // 안전한 후보들 사이에서 무엇을 최종 채택할지 점수화한다.
  // 이번 버전은 단순히 v>0 여부를 보는 대신,
  // "이 후보를 실행했을 때 실제로 goal/lookahead 쪽으로 얼마나 더 나아가는가"를
  // 마지막 예측 pose 기준으로 점수화한다.
  double scoreSafeCandidate(
    const geometry_msgs::msg::PoseStamped & pose,
    double predicted_x,
    double predicted_y,
    double predicted_yaw,
    double candidate_v,
    double candidate_w,
    double preferred_turn_dir,
    double original_v,
    double original_w) const;

  // 원래 후보 명령이 충돌할 때 더 안전한 대체 명령으로 강등한다.
  // 우선순위는:
  // 1) 원래 명령
  // 2) 제자리 회전
  // 3) 반대 방향 회전
  // 4) 필요 시 저속 후진 회전
  // 5) 최종 정지
  bool selectCollisionFreeCommand(
    const geometry_msgs::msg::PoseStamped & pose,
    double front,
    double left,
    double right,
    double heading_abs,
    int preferred_turn_dir,
    double original_v,
    double original_w,
    double & v,
    double & w) const;

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  rclcpp::Logger logger_{rclcpp::get_logger("rl_local_controller")};
  std::string name_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav_msgs::msg::Path plan_;

  // 파라미터(속도/회피/레이캐스팅 설정)
  double max_lin_{0.3};
  double max_ang_{1.0};
  double base_max_lin_{0.3};
  double base_max_ang_{1.0};
  double stop_dist_{0.5};
  // 전방이 너무 가까우면 완전 정지하는 거리
  double hard_stop_dist_{0.25};
  // 막힘 상태에서 아주 천천히 전진하는 속도
  double creep_speed_{0.05};
  // 제자리 회전을 시작하는 heading 오차(rad)
  double in_place_heading_{0.7};
  // 목표에 충분히 가까울 때만 제자리 회전을 허용하는 거리(m)
  double in_place_dist_{0.8};
  // 제자리 회전 최소 각속도(rad/s)
  double min_turn_rate_{0.4};
  // 제자리 회전을 허용할 최소 좌/우 여유 거리(m)
  double rotate_min_side_clearance_{0.45};
  // 제자리 회전을 허용할 최소 전방 여유 거리(m)
  double rotate_min_front_clearance_{0.30};
  // 좁은 구간 탈출용 전진 속도(m/s)
  double escape_forward_speed_{0.06};
  // 탈출 전진 시 회전 비율(0~1)
  double escape_forward_turn_scale_{0.6};
  // 좁은 구간에서 후진 탈출 허용 여부
  bool escape_use_reverse_{true};
  // 좁은 구간 탈출용 후진 속도(m/s)
  double escape_reverse_speed_{0.04};
  double slow_dist_{1.2};
  double heading_gain_{1.0};
  // heading 오차가 클 때 전진 속도를 줄이는 기준 각도(rad)
  double heading_slow_angle_{0.8};
  // heading 오차가 클 때 최소 전진 스케일(0~1)
  double heading_slow_min_scale_{0.2};
  double avoid_gain_{0.8};
  double turn_gain_{1.0};
  // 좌우 차이 데드밴드(회전 방향 흔들림 방지)
  double turn_deadband_{0.2};
  double range_max_{6.0};
  double lookahead_dist_{0.6};
  double front_angle_{0.35};
  double side_angle_{1.2};
  double angle_step_{0.05};
  double raycast_step_{0.05};
  int cost_threshold_{253};
  bool treat_unknown_as_obstacle_{false};

  // 미래 pose 기반 footprint 충돌 게이트 설정
  bool enable_footprint_collision_gate_{true};
  // gate는 장애물이 충분히 가까울 때만 강하게 개입한다.
  double collision_gate_front_activation_dist_{0.55};
  double collision_gate_side_activation_dist_{0.50};
  // 전진 명령은 조금 더 길게 본다.
  double forward_sim_horizon_sec_{0.8};
  double forward_sim_dt_{0.1};
  // 제자리 회전/회전 위주 명령은 더 짧게 본다.
  double rotate_sim_horizon_sec_{0.35};
  double rotate_sim_dt_{0.1};
  // deadlock 해소용 micro-progress 후보는 더 짧게 본다.
  double micro_progress_sim_horizon_sec_{0.30};
  double micro_progress_sim_dt_{0.1};
  // micro-progress 후보를 허용할 최소 여유 조건
  double micro_progress_front_margin_{0.05};
  double micro_progress_side_margin_{0.03};
  double micro_progress_heading_max_{0.65};
  int footprint_collision_cost_threshold_{253};

  // PID 기반 속도 보정(저수준 속도 추종 보완)
  bool use_pid_{true};
  double pid_kp_lin_{1.0};
  double pid_ki_lin_{0.0};
  double pid_kd_lin_{0.1};
  double pid_kp_ang_{2.0};
  double pid_ki_ang_{0.0};
  double pid_kd_ang_{0.1};
  double pid_i_max_lin_{0.5};
  double pid_i_max_ang_{1.0};
  double pid_dt_max_{0.5};

  // PID 내부 상태
  double pid_i_lin_{0.0};
  double pid_i_ang_{0.0};
  double pid_prev_err_lin_{0.0};
  double pid_prev_err_ang_{0.0};
  rclcpp::Time pid_last_time_;

  // PID 파라미터 동적 업데이트 콜백
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  // PID 목표 속도 퍼블리셔(학습/디버깅용)
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr desired_cmd_pub_;

  // 직전 회전 방향(진동 억제용 히스테리시스)
  int last_turn_dir_{0};
  // 근거리 목표 정렬 시 제자리 회전 모드 유지 플래그
  // - enter/exit 임계를 분리해 경계에서 모드가 빠르게 튀는 현상을 줄인다.
  bool align_in_place_mode_{false};
};

}  // namespace rl_local_controller
