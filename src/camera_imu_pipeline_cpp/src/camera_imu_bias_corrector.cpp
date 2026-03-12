#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class ImuBiasCorrector : public rclcpp::Node {
public:
  ImuBiasCorrector() : Node("camera_imu_bias_corrector_cpp") {
    // 입력/출력 토픽
    input_topic_ = declare_parameter<std::string>("input_topic", "/camera/camera/imu");
    output_topic_ = declare_parameter<std::string>("output_topic", "/camera/camera/imu_bias_corrected");
    // IMU 벡터를 변환할 목표 프레임(비워두면 변환 안 함)
    target_frame_ = declare_parameter<std::string>("target_frame", "");
    // TF 조회 타임아웃(초)
    tf_timeout_sec_ = declare_parameter<double>("tf_timeout_sec", 0.05);
    // 정지 상태에서 bias 계산에 사용할 샘플 수
    calib_samples_ = declare_parameter<int>("calib_samples", 1000);
    // 정지 판정 기준(각속도 크기)
    stationary_threshold_ = declare_parameter<double>("stationary_threshold", 0.01);
    // 공분산 값이 클수록 EKF가 덜 신뢰
    gyro_cov_ = declare_parameter<double>("gyro_cov", 0.1);
    accel_cov_ = declare_parameter<double>("accel_cov", 0.1);
    // 보정 중에도 메시지를 계속 내보낼지 여부
    publish_during_calib_ = declare_parameter<bool>("publish_during_calib", true);
    // 초기 교정 후 정지 상태에서 bias를 EMA로 지속 업데이트할지 여부
    // alpha: EMA 가중치 (작을수록 느리게 추적, 온도 드리프트 대응)
    continuous_calib_ = declare_parameter<bool>("continuous_calib", true);
    ema_alpha_ = declare_parameter<double>("ema_alpha", 0.001);

    // 정지 구간 yaw drift 억제 파라미터
    // 기존 threshold는 하위 호환용 기본값으로 남겨 두고,
    // 실제 제어는 LOCK enter/exit 상태기계 파라미터로 수행한다.
    // - yaw_lock_enter_wz: 다시 LOCK할 때 요구하는 낮은 |wz| 기준
    // - yaw_lock_exit_wz: 일반 회전 해제를 위한 필터링된 |wz| 기준
    // - yaw_lock_hard_exit_wz: 강한 회전이면 즉시 UNLOCK하는 raw |wz| 기준
    // - yaw_lock_enter_xy / exit_xy: roll/pitch rate 기반 진입/해제 보조 기준
    // - yaw_lock_enter_accel_dev: LOCK 진입 시 사용할 ||a|-g| 기준
    // - yaw_lock_enter_hold_sec: 다시 LOCK하기 전에 요구하는 연속 안정 시간
    // - yaw_lock_exit_hold_sec: yaw evidence가 해제 기준 이상으로 유지되어야 하는 최소 시간
    // - yaw_lock_exit_xy_hold_sec: xy 기반 보조 해제 조건이 유지되어야 하는 최소 시간
    // - yaw_lock_evidence_*: 저속 yaw를 "실제 회전"으로 승격시키는 evidence score 파라미터
    // - yaw_lock_filter_tau_sec / yaw_lock_accel_filter_tau_sec: gyro/accel EMA 시정수
    yaw_zeroing_enable_ = declare_parameter<bool>("yaw_zeroing_enable", true);
    yaw_zero_threshold_ = declare_parameter<double>("yaw_zero_threshold", 0.05);
    gyro_xy_stationary_threshold_ =
      declare_parameter<double>("gyro_xy_stationary_threshold", 0.05);
    accel_stationary_threshold_ =
      declare_parameter<double>("accel_stationary_threshold", 0.7);
    yaw_lock_enter_wz_ = declare_parameter<double>("yaw_lock_enter_wz", 0.02);
    yaw_lock_exit_wz_ = declare_parameter<double>("yaw_lock_exit_wz", yaw_zero_threshold_);
    yaw_lock_hard_exit_wz_ = declare_parameter<double>("yaw_lock_hard_exit_wz", 0.10);
    yaw_lock_enter_xy_ = declare_parameter<double>("yaw_lock_enter_xy", 0.03);
    yaw_lock_exit_xy_ =
      declare_parameter<double>("yaw_lock_exit_xy", gyro_xy_stationary_threshold_);
    yaw_lock_enter_accel_dev_ =
      declare_parameter<double>("yaw_lock_enter_accel_dev", 0.25);
    yaw_lock_enter_hold_sec_ = declare_parameter<double>("yaw_lock_enter_hold_sec", 0.50);
    yaw_lock_exit_hold_sec_ = declare_parameter<double>("yaw_lock_exit_hold_sec", 0.03);
    yaw_lock_exit_xy_hold_sec_ =
      declare_parameter<double>("yaw_lock_exit_xy_hold_sec", 0.08);
    yaw_lock_evidence_wz_low_ =
      declare_parameter<double>("yaw_lock_evidence_wz_low", 0.008);
    yaw_lock_evidence_wz_high_ =
      declare_parameter<double>("yaw_lock_evidence_wz_high", 0.025);
    yaw_lock_evidence_yaw_ref_rad_ =
      declare_parameter<double>("yaw_lock_evidence_yaw_ref_rad", 0.012);
    yaw_lock_evidence_decay_sec_ =
      declare_parameter<double>("yaw_lock_evidence_decay_sec", 0.40);
    yaw_lock_evidence_unlock_threshold_ =
      declare_parameter<double>("yaw_lock_evidence_unlock_threshold", 0.48);
    yaw_lock_evidence_unlock_hold_sec_ =
      declare_parameter<double>("yaw_lock_evidence_unlock_hold_sec", 0.04);
    yaw_lock_evidence_relock_threshold_ =
      declare_parameter<double>("yaw_lock_evidence_relock_threshold", 0.15);
    yaw_lock_filter_tau_sec_ = declare_parameter<double>("yaw_lock_filter_tau_sec", 0.10);
    yaw_lock_accel_filter_tau_sec_ =
      declare_parameter<double>("yaw_lock_accel_filter_tau_sec", 0.15);
    gravity_mps2_ = declare_parameter<double>("gravity_mps2", 9.81);
    yaw_stationary_cov_ = declare_parameter<double>("yaw_stationary_cov", 1e-4);
    yaw_moving_cov_ = declare_parameter<double>("yaw_moving_cov", gyro_cov_);

    auto qos = rclcpp::SensorDataQoS();
    sub_ = create_subscription<sensor_msgs::msg::Imu>(
      input_topic_, qos,
      std::bind(&ImuBiasCorrector::cb, this, std::placeholders::_1));
    pub_ = create_publisher<sensor_msgs::msg::Imu>(output_topic_, qos);

    // TF 버퍼/리스너 초기화 (필요 시 IMU 벡터 회전에 사용)
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    RCLCPP_INFO(get_logger(), "IMU bias corrector started: input=%s output=%s",
                input_topic_.c_str(), output_topic_.c_str());
    if (!target_frame_.empty()) {
      RCLCPP_INFO(get_logger(), "IMU target_frame is set: %s", target_frame_.c_str());
    }
  }

private:
  // 쿼터니언으로 벡터를 회전
  static inline tf2::Vector3 rotateVec(const tf2::Quaternion &q, double x, double y, double z) {
    tf2::Matrix3x3 m(q);
    tf2::Vector3 v(x, y, z);
    return m * v;
  }

  static inline double ema(double prev, double sample, double dt_sec, double tau_sec) {
    if (tau_sec <= 0.0) {
      return sample;
    }
    const double alpha = dt_sec / (tau_sec + dt_sec);
    return prev + alpha * (sample - prev);
  }

  static inline double clampDt(double dt_sec) {
    if (!(dt_sec > 0.0) || dt_sec > 0.2) {
      return 0.02;
    }
    return dt_sec;
  }

  static inline double clamp01(double value) {
    return std::max(0.0, std::min(1.0, value));
  }

  static inline double normalizeRange(double value, double min_value, double max_value) {
    if (max_value <= min_value) {
      return value > max_value ? 1.0 : 0.0;
    }
    return clamp01((value - min_value) / (max_value - min_value));
  }

  static inline double decayTowardZero(double value, double dt_sec, double tau_sec) {
    if (tau_sec <= 0.0) {
      return 0.0;
    }
    return value * std::exp(-dt_sec / tau_sec);
  }

  void cb(const sensor_msgs::msg::Imu::SharedPtr msg) {
    // IMU 벡터(각속도/가속도)를 target_frame 기준으로 회전
    tf2::Quaternion q_tf;
    bool has_tf = false;
    if (!target_frame_.empty() && msg->header.frame_id != target_frame_) {
      try {
        auto tf = tf_buffer_->lookupTransform(
          target_frame_, msg->header.frame_id, msg->header.stamp,
          rclcpp::Duration::from_seconds(tf_timeout_sec_));
        q_tf = tf2::Quaternion(
          tf.transform.rotation.x,
          tf.transform.rotation.y,
          tf.transform.rotation.z,
          tf.transform.rotation.w);
        has_tf = true;
        tf_warned_ = false;
      } catch (const std::exception &e) {
        if (!tf_warned_) {
          RCLCPP_WARN(get_logger(),
                      "IMU TF lookup failed (%s -> %s): %s",
                      msg->header.frame_id.c_str(), target_frame_.c_str(), e.what());
          tf_warned_ = true;
        }
      }
    }

    // 각속도/가속도 값 준비(필요 시 회전)
    auto av = msg->angular_velocity;
    auto la = msg->linear_acceleration;
    if (has_tf) {
      auto av_rot = rotateVec(q_tf, av.x, av.y, av.z);
      auto la_rot = rotateVec(q_tf, la.x, la.y, la.z);
      av.x = av_rot.x(); av.y = av_rot.y(); av.z = av_rot.z();
      la.x = la_rot.x(); la.y = la_rot.y(); la.z = la_rot.z();
    }

    const double mag = std::sqrt(av.x * av.x + av.y * av.y + av.z * av.z);

    if (!bias_ready_) {
      // target_frame이 설정됐는데 TF를 아직 못 얻었으면 누적하지 않음
      // (잘못된 좌표계 기준으로 bias를 추정하는 것 방지)
      const bool tf_ok = target_frame_.empty() || has_tf;
      if (tf_ok && mag < stationary_threshold_) {
        sum_x_ += av.x;
        sum_y_ += av.y;
        sum_z_ += av.z;
        count_++;
        if (count_ == 1) {
          RCLCPP_INFO(get_logger(), "Calibrating IMU bias... keep robot still.");
        }
        if (count_ >= calib_samples_) {
          // 평균값을 bias로 확정
          bias_x_ = sum_x_ / count_;
          bias_y_ = sum_y_ / count_;
          bias_z_ = sum_z_ / count_;
          bias_ready_ = true;
          RCLCPP_INFO(get_logger(), "IMU bias calibrated: x=%.6f y=%.6f z=%.6f",
                      bias_x_, bias_y_, bias_z_);
        }
      }
    } else if (continuous_calib_ && mag < stationary_threshold_) {
      // 초기 교정 후 정지 상태에서 EMA로 bias 지속 업데이트 (온도 드리프트 추적)
      bias_x_ = (1.0 - ema_alpha_) * bias_x_ + ema_alpha_ * av.x;
      bias_y_ = (1.0 - ema_alpha_) * bias_y_ + ema_alpha_ * av.y;
      bias_z_ = (1.0 - ema_alpha_) * bias_z_ + ema_alpha_ * av.z;
    }

    if (!bias_ready_ && !publish_during_calib_) {
      // 보정 완료 전에는 발행하지 않음
      return;
    }

    auto out = *msg;
    if (has_tf) {
      out.header.frame_id = target_frame_;
    }
    // Orientation은 사용하지 않음을 명시
    out.orientation_covariance[0] = -1.0;
    // EKF 신뢰도를 위해 공분산을 설정
    out.angular_velocity_covariance = {gyro_cov_, 0.0, 0.0,
                                        0.0, gyro_cov_, 0.0,
                                        0.0, 0.0, gyro_cov_};
    out.linear_acceleration_covariance = {accel_cov_, 0.0, 0.0,
                                           0.0, accel_cov_, 0.0,
                                           0.0, 0.0, accel_cov_};

    bool yaw_clamped = false;
    if (bias_ready_) {
      // bias 제거 후 각속도 재발행
      out.angular_velocity.x = av.x - bias_x_;
      out.angular_velocity.y = av.y - bias_y_;
      out.angular_velocity.z = av.z - bias_z_;
      // 가속도는 그대로(필요 시 회전된 값)
      out.linear_acceleration = la;
    } else {
      // 보정 전이라도 회전된 값은 그대로 내보냄
      out.angular_velocity = av;
      out.linear_acceleration = la;
    }

    // 정지 구간에는 yaw를 잠그되, 회전 시작은 즉시/준즉시 해제하는 비대칭 상태기계.
    if (yaw_zeroing_enable_) {
      const double wx = out.angular_velocity.x;
      const double wy = out.angular_velocity.y;
      const double raw_wz = out.angular_velocity.z;
      const double abs_raw_wz = std::fabs(raw_wz);
      const double raw_xy_rate = std::sqrt(wx * wx + wy * wy);
      const double acc_norm = std::sqrt(
        out.linear_acceleration.x * out.linear_acceleration.x +
        out.linear_acceleration.y * out.linear_acceleration.y +
        out.linear_acceleration.z * out.linear_acceleration.z);
      const double raw_acc_dev = std::fabs(acc_norm - gravity_mps2_);

      double stamp_sec = rclcpp::Time(msg->header.stamp).seconds();
      if (!(stamp_sec > 0.0)) {
        stamp_sec = now().seconds();
      }
      double dt_sec = 0.02;
      if (std::isfinite(last_yaw_lock_stamp_sec_)) {
        dt_sec = clampDt(stamp_sec - last_yaw_lock_stamp_sec_);
      }
      last_yaw_lock_stamp_sec_ = stamp_sec;

      if (!yaw_lock_metrics_ready_) {
        yaw_lock_signed_wz_ema_ = raw_wz;
        yaw_lock_abs_wz_ema_ = abs_raw_wz;
        yaw_lock_xy_rate_ema_ = raw_xy_rate;
        yaw_lock_acc_dev_ema_ = raw_acc_dev;
        yaw_lock_metrics_ready_ = true;
      } else {
        yaw_lock_signed_wz_ema_ =
          ema(yaw_lock_signed_wz_ema_, raw_wz, dt_sec, yaw_lock_filter_tau_sec_);
        yaw_lock_abs_wz_ema_ =
          ema(yaw_lock_abs_wz_ema_, abs_raw_wz, dt_sec, yaw_lock_filter_tau_sec_);
        yaw_lock_xy_rate_ema_ =
          ema(yaw_lock_xy_rate_ema_, raw_xy_rate, dt_sec, yaw_lock_filter_tau_sec_);
        yaw_lock_acc_dev_ema_ =
          ema(yaw_lock_acc_dev_ema_, raw_acc_dev, dt_sec, yaw_lock_accel_filter_tau_sec_);
      }

      // 저속 yaw는 별도 상태를 만들지 않고 evidence로 누적한다.
      // deadband를 넘는 yaw가 같은 방향으로 유지되면 signed evidence를 쌓고,
      // yaw가 작아지면 evidence를 0 방향으로 감쇠시킨다.
      const double yaw_over_deadband =
        std::max(0.0, yaw_lock_abs_wz_ema_ - yaw_lock_evidence_wz_low_);
      if (yaw_over_deadband > 0.0) {
        if (std::fabs(yaw_lock_yaw_evidence_rad_) > 1.0e-6 &&
            yaw_lock_yaw_evidence_rad_ * yaw_lock_signed_wz_ema_ < 0.0) {
          yaw_lock_yaw_evidence_rad_ = 0.0;
        }
        yaw_lock_yaw_evidence_rad_ +=
          std::copysign(yaw_over_deadband * dt_sec, yaw_lock_signed_wz_ema_);
      } else {
        yaw_lock_yaw_evidence_rad_ = decayTowardZero(
          yaw_lock_yaw_evidence_rad_, dt_sec, yaw_lock_evidence_decay_sec_);
      }

      const double yaw_evidence_mag =
        normalizeRange(yaw_lock_abs_wz_ema_, yaw_lock_evidence_wz_low_, yaw_lock_evidence_wz_high_);
      const double yaw_evidence_persist =
        normalizeRange(std::fabs(yaw_lock_yaw_evidence_rad_), 0.0, yaw_lock_evidence_yaw_ref_rad_);
      const double yaw_evidence_xy_penalty =
        normalizeRange(yaw_lock_xy_rate_ema_, yaw_lock_enter_xy_, yaw_lock_exit_xy_);

      // score 구성:
      // - magnitude 0.45: 지금 순간의 yaw 크기
      // - persistence 0.45: 같은 방향 yaw가 계속 유지된 증거
      // - xy penalty 0.20: body shake가 크면 evidence 신뢰도 감산
      yaw_lock_evidence_score_ = clamp01(
        0.45 * yaw_evidence_mag +
        0.45 * yaw_evidence_persist -
        0.20 * yaw_evidence_xy_penalty);

      const bool enter_ready =
        yaw_lock_abs_wz_ema_ < yaw_lock_enter_wz_ &&
        yaw_lock_xy_rate_ema_ < yaw_lock_enter_xy_ &&
        yaw_lock_acc_dev_ema_ < yaw_lock_enter_accel_dev_ &&
        yaw_lock_evidence_score_ < yaw_lock_evidence_relock_threshold_;
      const bool hard_exit = abs_raw_wz > yaw_lock_hard_exit_wz_;
      const bool soft_exit_yaw_evidence =
        yaw_lock_evidence_score_ >= yaw_lock_evidence_unlock_threshold_;
      const bool soft_exit_xy = yaw_lock_xy_rate_ema_ > yaw_lock_exit_xy_;

      std::string transition_reason;
      if (yaw_locked_) {
        yaw_lock_enter_accum_sec_ = 0.0;
        if (hard_exit) {
          yaw_locked_ = false;
          yaw_lock_exit_evidence_accum_sec_ = 0.0;
          yaw_lock_exit_xy_accum_sec_ = 0.0;
          transition_reason = "hard_wz";
        } else {
          if (soft_exit_yaw_evidence) {
            yaw_lock_exit_evidence_accum_sec_ += dt_sec;
          } else {
            yaw_lock_exit_evidence_accum_sec_ = 0.0;
          }

          if (soft_exit_xy && yaw_lock_abs_wz_ema_ > yaw_lock_evidence_wz_low_) {
            yaw_lock_exit_xy_accum_sec_ += dt_sec;
          } else {
            yaw_lock_exit_xy_accum_sec_ = 0.0;
          }

          if (yaw_lock_exit_evidence_accum_sec_ >= yaw_lock_evidence_unlock_hold_sec_) {
            yaw_locked_ = false;
            yaw_lock_exit_evidence_accum_sec_ = 0.0;
            yaw_lock_exit_xy_accum_sec_ = 0.0;
            transition_reason = "yaw_evidence";
          } else if (yaw_lock_exit_xy_accum_sec_ >= yaw_lock_exit_xy_hold_sec_) {
            yaw_locked_ = false;
            yaw_lock_exit_evidence_accum_sec_ = 0.0;
            yaw_lock_exit_xy_accum_sec_ = 0.0;
            transition_reason = "xy_hold";
          }
        }
      } else {
        yaw_lock_exit_evidence_accum_sec_ = 0.0;
        yaw_lock_exit_xy_accum_sec_ = 0.0;
        if (enter_ready) {
          yaw_lock_enter_accum_sec_ += dt_sec;
          if (yaw_lock_enter_accum_sec_ >= yaw_lock_enter_hold_sec_) {
            yaw_locked_ = true;
            yaw_lock_enter_accum_sec_ = 0.0;
            transition_reason = "steady_hold";
          }
        } else {
          yaw_lock_enter_accum_sec_ = 0.0;
        }
      }

      yaw_clamped = yaw_locked_;
      if (yaw_clamped) {
        out.angular_velocity.z = 0.0;
      }

      if (yaw_clamped != last_yaw_clamped_) {
        last_yaw_clamped_ = yaw_clamped;
        RCLCPP_INFO(
          get_logger(),
          "yaw_zeroing=%s reason=%s raw_wz=%.5f wz_f=%.5f xy_f=%.5f acc_dev_f=%.5f score=%.3f eyaw=%.5f yaw_cov=%.6f",
          yaw_clamped ? "ON" : "OFF",
          transition_reason.empty() ? "none" : transition_reason.c_str(),
          raw_wz,
          yaw_lock_signed_wz_ema_,
          yaw_lock_xy_rate_ema_,
          yaw_lock_acc_dev_ema_,
          yaw_lock_evidence_score_,
          yaw_lock_yaw_evidence_rad_,
          yaw_clamped ? yaw_stationary_cov_ : yaw_moving_cov_);
      }
    }

    // z축 공분산을 상태별로 나눠 EKF 신뢰도를 제어한다.
    const double yaw_cov = yaw_clamped ? yaw_stationary_cov_ : yaw_moving_cov_;
    out.angular_velocity_covariance = {gyro_cov_, 0.0, 0.0,
                                       0.0, gyro_cov_, 0.0,
                                       0.0, 0.0, yaw_cov};

    pub_->publish(out);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string target_frame_;
  double tf_timeout_sec_{0.05};
  int calib_samples_{1000};
  double stationary_threshold_{0.01};
  double gyro_cov_{0.1};
  double accel_cov_{0.1};
  bool publish_during_calib_{true};
  bool continuous_calib_{true};
  double ema_alpha_{0.001};
  bool tf_warned_{false};
  bool yaw_zeroing_enable_{true};
  double yaw_zero_threshold_{0.05};
  double gyro_xy_stationary_threshold_{0.05};
  double accel_stationary_threshold_{0.7};
  double yaw_lock_enter_wz_{0.02};
  double yaw_lock_exit_wz_{0.05};
  double yaw_lock_hard_exit_wz_{0.10};
  double yaw_lock_enter_xy_{0.03};
  double yaw_lock_exit_xy_{0.05};
  double yaw_lock_enter_accel_dev_{0.25};
  double yaw_lock_enter_hold_sec_{0.50};
  double yaw_lock_exit_hold_sec_{0.03};
  double yaw_lock_exit_xy_hold_sec_{0.08};
  double yaw_lock_evidence_wz_low_{0.008};
  double yaw_lock_evidence_wz_high_{0.025};
  double yaw_lock_evidence_yaw_ref_rad_{0.012};
  double yaw_lock_evidence_decay_sec_{0.40};
  double yaw_lock_evidence_unlock_threshold_{0.48};
  double yaw_lock_evidence_unlock_hold_sec_{0.04};
  double yaw_lock_evidence_relock_threshold_{0.15};
  double yaw_lock_filter_tau_sec_{0.10};
  double yaw_lock_accel_filter_tau_sec_{0.15};
  double gravity_mps2_{9.81};
  double yaw_stationary_cov_{1e-4};
  double yaw_moving_cov_{0.1};
  bool last_yaw_clamped_{false};
  bool yaw_locked_{false};
  bool yaw_lock_metrics_ready_{false};
  double yaw_lock_signed_wz_ema_{0.0};
  double yaw_lock_abs_wz_ema_{0.0};
  double yaw_lock_xy_rate_ema_{0.0};
  double yaw_lock_acc_dev_ema_{0.0};
  double yaw_lock_yaw_evidence_rad_{0.0};
  double yaw_lock_evidence_score_{0.0};
  double yaw_lock_enter_accum_sec_{0.0};
  double yaw_lock_exit_evidence_accum_sec_{0.0};
  double yaw_lock_exit_xy_accum_sec_{0.0};
  double last_yaw_lock_stamp_sec_{std::numeric_limits<double>::quiet_NaN()};

  double sum_x_{0.0};
  double sum_y_{0.0};
  double sum_z_{0.0};
  int count_{0};
  bool bias_ready_{false};
  double bias_x_{0.0};
  double bias_y_{0.0};
  double bias_z_{0.0};

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuBiasCorrector>());
  rclcpp::shutdown();
  return 0;
}
