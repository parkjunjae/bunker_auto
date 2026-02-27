from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # This launch runs the "agent PID" policy that tunes RLController PID gains online.
    # Prerequisite: Nav2 controller_server is running with RLController plugin enabled and publishing
    # /controller_server/RLController/desired_cmd.
    model = LaunchConfiguration("model")
    log_dir = LaunchConfiguration("log_dir")
    odom_topic = LaunchConfiguration("odom_topic")
    desired_cmd_topic = LaunchConfiguration("desired_cmd_topic")
    desired_cmd_type = LaunchConfiguration("desired_cmd_type")
    motor_status_topic = LaunchConfiguration("motor_status_topic")
    use_motor_encoder_obs = LaunchConfiguration("use_motor_encoder_obs")
    use_sim_time = LaunchConfiguration("use_sim_time")
    python_exec = LaunchConfiguration("python_exec")
    step_dt = LaunchConfiguration("step_dt")
    gain_scale = LaunchConfiguration("gain_scale")
    gain_lpf_alpha = LaunchConfiguration("gain_lpf_alpha")
    action_deadzone = LaunchConfiguration("action_deadzone")
    straight_freeze_w_ref = LaunchConfiguration("straight_freeze_w_ref")
    straight_freeze_v_ref = LaunchConfiguration("straight_freeze_v_ref")
    stop_freeze_v_ref = LaunchConfiguration("stop_freeze_v_ref")
    stop_freeze_w_ref = LaunchConfiguration("stop_freeze_w_ref")
    rotate_freeze_w_ref = LaunchConfiguration("rotate_freeze_w_ref")
    rotate_freeze_v_ref = LaunchConfiguration("rotate_freeze_v_ref")
    w_ref_lpf_alpha = LaunchConfiguration("w_ref_lpf_alpha")
    w_ref_deadband = LaunchConfiguration("w_ref_deadband")
    dither_v_ref_thresh = LaunchConfiguration("dither_v_ref_thresh")
    dither_w_ref_thresh = LaunchConfiguration("dither_w_ref_thresh")
    w_ref_sign_hold_sec = LaunchConfiguration("w_ref_sign_hold_sec")
    w_ref_abs_max_low_speed = LaunchConfiguration("w_ref_abs_max_low_speed")

    base_args = [
        "--model",
        model,
        "--log-dir",
        log_dir,
        "--odom-topic",
        odom_topic,
        "--desired-cmd-topic",
        desired_cmd_topic,
        "--desired-cmd-type",
        desired_cmd_type,
        "--motor-status-topic",
        motor_status_topic,
        "--use-motor-encoder-obs",
        use_motor_encoder_obs,
        "--step-dt",
        step_dt,
        "--gain-scale",
        gain_scale,
        "--gain-lpf-alpha",
        gain_lpf_alpha,
        "--action-deadzone",
        action_deadzone,
        "--straight-freeze-w-ref",
        straight_freeze_w_ref,
        "--straight-freeze-v-ref",
        straight_freeze_v_ref,
        "--stop-freeze-v-ref",
        stop_freeze_v_ref,
        "--stop-freeze-w-ref",
        stop_freeze_w_ref,
        "--rotate-freeze-w-ref",
        rotate_freeze_w_ref,
        "--rotate-freeze-v-ref",
        rotate_freeze_v_ref,
        "--w-ref-lpf-alpha",
        w_ref_lpf_alpha,
        "--w-ref-deadband",
        w_ref_deadband,
        "--dither-v-ref-thresh",
        dither_v_ref_thresh,
        "--dither-w-ref-thresh",
        dither_w_ref_thresh,
        "--w-ref-sign-hold-sec",
        w_ref_sign_hold_sec,
        "--w-ref-abs-max-low-speed",
        w_ref_abs_max_low_speed,
    ]

    node_no_sim = ExecuteProcess(
        cmd=[python_exec, "-m", "rl_pid_training.run_pid_policy"] + base_args,
        output="screen",
        condition=UnlessCondition(use_sim_time),
    )

    node_sim = ExecuteProcess(
        cmd=[python_exec, "-m", "rl_pid_training.run_pid_policy"] + base_args + ["--use-sim-time"],
        output="screen",
        condition=IfCondition(use_sim_time),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "model",
                default_value="/home/atoz/ca_ws/rl_pid_model_new",
                description="학습된 PPO 모델(.zip) 경로",
            ),
            DeclareLaunchArgument(
                "log_dir",
                default_value="/home/atoz/ca_ws/rl_pid_logs",
                description="CSV 로그 저장 폴더",
            ),
            DeclareLaunchArgument(
                "odom_topic",
                default_value="/odometry/filtered",
                description="오도메트리 토픽",
            ),
            DeclareLaunchArgument(
                "desired_cmd_topic",
                default_value="/controller_server/RLController/desired_cmd",
                description="RLController가 계산한 목표 속도 토픽",
            ),
            DeclareLaunchArgument(
                "desired_cmd_type",
                default_value="auto",
                description="desired_cmd 타입(auto|twist|twist_stamped)",
            ),
            DeclareLaunchArgument(
                "motor_status_topic",
                default_value="/bunker_status",
                description="모터/엔코더 상태 토픽",
            ),
            DeclareLaunchArgument(
                "use_motor_encoder_obs",
                default_value="true",
                description="관측에 모터/엔코더 피처 포함 여부(true/false)",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="시뮬레이션 시간 사용 여부(true/false)",
            ),
            DeclareLaunchArgument(
                "step_dt",
                default_value="0.4",
                description="agent PID 갱신 주기(초), 클수록 덜 민감",
            ),
            DeclareLaunchArgument(
                "gain_scale",
                default_value="0.05",
                description="게인 증감 스케일(작을수록 덜 흔들림)",
            ),
            DeclareLaunchArgument(
                "gain_lpf_alpha",
                default_value="0.08",
                description="게인 저역통과 계수(0~1) 게인 변경을 저역통과해서 “천천히” 변하게 함.",
            ),
            DeclareLaunchArgument(
                "action_deadzone",
                default_value="0.20",
                description="행동 deadzone(절대값 이하면 무시) 작은 정책 출력은 무시해서 미세 떨림 제거.",
            ),
            DeclareLaunchArgument(
                "straight_freeze_w_ref",
                default_value="0.12",
                description="직진 판정용 |w_ref| 임계값(rad/s)",
            ),
            DeclareLaunchArgument(
                "straight_freeze_v_ref",
                default_value="0.08",
                description="직진 판정용 v_ref 임계값(m/s)",
            ),
            DeclareLaunchArgument(
                "stop_freeze_v_ref",
                default_value="0.03",
                description="정지 판정용 |v_ref| 임계값(m/s)",
            ),
            DeclareLaunchArgument(
                "stop_freeze_w_ref",
                default_value="0.05",
                description="정지 판정용 |w_ref| 임계값(rad/s)",
            ),
            DeclareLaunchArgument(
                "rotate_freeze_w_ref",
                default_value="0.20",
                description="제자리 회전 판정용 |w_ref| 임계값(rad/s)",
            ),
            DeclareLaunchArgument(
                "rotate_freeze_v_ref",
                default_value="0.05",
                description="제자리 회전 판정용 |v_ref| 임계값(m/s)",
            ),
            #-----------------
            # 저속 좌우 떨림 억제용 w_ref 가드(로그 기반 기본값)
            DeclareLaunchArgument(
                "w_ref_lpf_alpha",
                default_value="0.10",
                description="w_ref 저역통과 계수(작을수록 더 부드러움)",
            ),
            #-----------------
            DeclareLaunchArgument(
                "w_ref_deadband",
                default_value="0.08",
                description="|w_ref| deadband(rad/s): 이하면 0으로 처리, 아주 작은 회전 명령은 0으로 처리(제자리 미세 떨림 방지).",
            ),
            DeclareLaunchArgument(
                "dither_v_ref_thresh",
                default_value="0.10",
                description="저속 anti-dither 활성화 v_ref 임계값(m/s)",
            ),
            DeclareLaunchArgument(
                "dither_w_ref_thresh",
                default_value="0.20",
                description="anti-dither 대상 최소 |w_ref|(rad/s) 이 값 이하의 작은 회전 요구에는 과민반응하지 않게 함",
            ),
            #-----------------
            DeclareLaunchArgument(
                "w_ref_sign_hold_sec",
                default_value="0.50",
                description="저속 회전 시 w_ref 부호 최소 유지 시간(s)",
            ),
            #-----------------
            DeclareLaunchArgument(
                "w_ref_abs_max_low_speed",
                default_value="0.30",
                description="저속 구간 |w_ref| 상한(rad/s)",
            ),
            #-----------------
            DeclareLaunchArgument(
                "python_exec",
                default_value="python3",
                description="agent PID 실행에 사용할 Python 실행파일(가상환경 사용 시 해당 python)",
            ),
            node_no_sim,
            node_sim,
        ]
    )
