#!/usr/bin/env python3
import argparse
import csv
import math
from pathlib import Path


def moving_average(values, window):
    if window <= 1 or not values:
        return values[:]
    out = []
    acc = 0.0
    q = []
    for v in values:
        q.append(v)
        acc += v
        if len(q) > window:
            acc -= q.pop(0)
        out.append(acc / len(q))
    return out


def to_float(v):
    try:
        if v is None or v == "":
            return math.nan
        return float(v)
    except Exception:
        return math.nan


def load_csv(csv_path):
    with open(csv_path, newline="") as f:
        reader = csv.DictReader(f)
        rows = list(reader)
    if not rows:
        raise RuntimeError("CSV is empty.")
    cols = reader.fieldnames or []
    data = {c: [to_float(r.get(c)) for r in rows] for c in cols}
    return cols, data


def safe_mae(a, b):
    pairs = [(x, y) for x, y in zip(a, b) if not math.isnan(x) and not math.isnan(y)]
    if not pairs:
        return math.nan
    return sum(abs(x - y) for x, y in pairs) / len(pairs)


def main():
    parser = argparse.ArgumentParser(description="Visualize RL PID policy CSV logs.")
    parser.add_argument(
        "--csv",
        default="/home/atoz/ca_ws/rl_pid_logs/pid_policy_20260226_162306.csv",
        help="Path to pid_policy CSV file",
    )
    parser.add_argument(
        "--out",
        default="",
        help="Output PNG path (default: same dir, *.png)",
    )
    parser.add_argument(
        "--reward-ma-window",
        type=int,
        default=50,
        help="Moving-average window size for reward",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="Show plot window interactively",
    )
    args = parser.parse_args()

    try:
        import matplotlib.pyplot as plt
    except ImportError as e:
        raise SystemExit("matplotlib is required. Install it with: pip3 install matplotlib") from e

    csv_path = Path(args.csv).expanduser()
    if not csv_path.exists():
        raise SystemExit(f"CSV not found: {csv_path}")

    cols, data = load_csv(csv_path)
    if "t" not in data:
        raise SystemExit("Column 't' is required.")

    t_abs = data["t"]
    t0 = next((x for x in t_abs if not math.isnan(x)), None)
    if t0 is None:
        raise SystemExit("Column 't' has no valid numeric values.")
    t = [x - t0 if not math.isnan(x) else math.nan for x in t_abs]

    fig, axes = plt.subplots(4, 1, figsize=(14, 12), sharex=True)
    ax0, ax1, ax2, ax3 = axes

    # 1) Velocity tracking
    if "v_ref" in data:
        ax0.plot(t, data["v_ref"], label="v_ref", linewidth=1.8)
    if "v_meas" in data:
        ax0.plot(t, data["v_meas"], label="v_meas", linewidth=1.2)
    if "w_ref_raw" in data:
        ax0.plot(t, data["w_ref_raw"], label="w_ref_raw", linewidth=1.2, alpha=0.7)
    if "w_ref" in data:
        ax0.plot(t, data["w_ref"], label="w_ref", linewidth=1.4)
    if "w_meas" in data:
        ax0.plot(t, data["w_meas"], label="w_meas", linewidth=1.2)
    ax0.set_ylabel("Velocity")
    ax0.set_title("Reference vs Measured (v, w)")
    ax0.grid(True, alpha=0.3)
    ax0.legend(loc="upper right", ncol=3, fontsize=9)

    # 2) PID gains
    gain_cols = ["kp_lin", "ki_lin", "kd_lin", "kp_ang", "ki_ang", "kd_ang"]
    for c in gain_cols:
        if c in data:
            ax1.plot(t, data[c], label=c, linewidth=1.2)
    ax1.set_ylabel("Gain")
    ax1.set_title("PID Gains")
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc="upper right", ncol=3, fontsize=9)

    # 3) Motor / encoder
    if "motor_rpm_mean" in data:
        ax2.plot(t, data["motor_rpm_mean"], label="motor_rpm_mean", linewidth=1.2)
    if "motor_current_mean" in data:
        ax2.plot(t, data["motor_current_mean"], label="motor_current_mean", linewidth=1.2)
    if "encoder_pulse_rate_mean" in data:
        ax2.plot(t, data["encoder_pulse_rate_mean"], label="encoder_pulse_rate_mean", linewidth=1.2)
    ax2.set_ylabel("Motor/Encoder")
    ax2.set_title("Motor / Encoder Signals")
    ax2.grid(True, alpha=0.3)
    ax2.legend(loc="upper right", ncol=3, fontsize=9)

    # 4) Reward
    if "reward" in data:
        reward = data["reward"]
        reward_ma = moving_average(reward, max(1, args.reward_ma_window))
        ax3.plot(t, reward, label="reward", linewidth=1.0, alpha=0.5)
        ax3.plot(t, reward_ma, label=f"reward_ma({args.reward_ma_window})", linewidth=2.0)
    ax3.set_xlabel("Time (s from start)")
    ax3.set_ylabel("Reward")
    ax3.set_title("Reward")
    ax3.grid(True, alpha=0.3)
    ax3.legend(loc="upper right", fontsize=9)

    # Summary metrics
    duration = max(x for x in t if not math.isnan(x))
    mae_v = safe_mae(data.get("v_ref", []), data.get("v_meas", []))
    mae_w = safe_mae(data.get("w_ref", []), data.get("w_meas", []))
    fig.suptitle(
        f"{csv_path.name} | rows={len(t)} | duration={duration:.1f}s | "
        f"MAE(v)={mae_v:.4f} | MAE(w)={mae_w:.4f}",
        fontsize=11
    )

    fig.tight_layout(rect=[0, 0, 1, 0.97])

    out_path = Path(args.out).expanduser() if args.out else csv_path.with_suffix(".png")
    fig.savefig(out_path, dpi=150)
    print(f"Saved plot: {out_path}")

    if args.show:
        plt.show()


if __name__ == "__main__":
    main()
