#include <cmath>
#include <memory>

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
    // - yaw_zeroing_enable: true면 정지로 판단될 때 angular_velocity.z를 0으로 고정
    // - yaw_zero_threshold: |wz|가 이 값보다 작으면 정지 후보
    // - gyro_xy_stationary_threshold: roll/pitch rate가 작아야 정지로 인정
    // - accel_stationary_threshold: | |a|-g |가 작아야 정지로 인정
    // - yaw_stationary_cov: 정지 고정 시 z축 공분산(작을수록 EKF가 0을 더 신뢰)
    // - yaw_moving_cov: 비정지 시 z축 공분산
    yaw_zeroing_enable_ = declare_parameter<bool>("yaw_zeroing_enable", true);
    yaw_zero_threshold_ = declare_parameter<double>("yaw_zero_threshold", 0.03);
    gyro_xy_stationary_threshold_ =
      declare_parameter<double>("gyro_xy_stationary_threshold", 0.05);
    accel_stationary_threshold_ =
      declare_parameter<double>("accel_stationary_threshold", 0.7);
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

    // 정지 판정 시 yaw rate를 0으로 고정해 EKF yaw 드리프트 적분을 차단
    if (yaw_zeroing_enable_) {
      const double wx = out.angular_velocity.x;
      const double wy = out.angular_velocity.y;
      const double wz = out.angular_velocity.z;
      const double acc_norm = std::sqrt(
        out.linear_acceleration.x * out.linear_acceleration.x +
        out.linear_acceleration.y * out.linear_acceleration.y +
        out.linear_acceleration.z * out.linear_acceleration.z);

      const bool low_yaw_rate = std::fabs(wz) < yaw_zero_threshold_;
      const bool low_roll_pitch_rate =
        std::fabs(wx) < gyro_xy_stationary_threshold_ &&
        std::fabs(wy) < gyro_xy_stationary_threshold_;
      const bool accel_near_g =
        std::fabs(acc_norm - gravity_mps2_) < accel_stationary_threshold_;

      yaw_clamped = low_yaw_rate && low_roll_pitch_rate && accel_near_g;
      if (yaw_clamped) {
        out.angular_velocity.z = 0.0;
      }
    }

    // z축 공분산을 상태별로 나눠 EKF 신뢰도를 제어한다.
    const double yaw_cov = yaw_clamped ? yaw_stationary_cov_ : yaw_moving_cov_;
    out.angular_velocity_covariance = {gyro_cov_, 0.0, 0.0,
                                       0.0, gyro_cov_, 0.0,
                                       0.0, 0.0, yaw_cov};
    if (yaw_clamped != last_yaw_clamped_) {
      last_yaw_clamped_ = yaw_clamped;
      RCLCPP_INFO(get_logger(),
                  "yaw_zeroing=%s (wz=%.5f, yaw_cov=%.6f)",
                  yaw_clamped ? "ON" : "OFF",
                  out.angular_velocity.z,
                  yaw_cov);
    }

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
  double yaw_zero_threshold_{0.03};
  double gyro_xy_stationary_threshold_{0.05};
  double accel_stationary_threshold_{0.7};
  double gravity_mps2_{9.81};
  double yaw_stationary_cov_{1e-4};
  double yaw_moving_cov_{0.1};
  bool last_yaw_clamped_{false};

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
