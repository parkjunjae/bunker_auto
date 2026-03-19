#!/usr/bin/env python3
"""
Cursor-based query tool for append-only JSONL probe logs.

Despite the historical file name, this tool is generic enough to query:
- spin_yaw_probe.py JSONL
- future map_publish_probe.py JSONL

Examples:
  python3 tools/map_event_query.py --file logs/spin.jsonl --record-type spin_exit
  python3 tools/map_event_query.py --file logs/spin.jsonl --after-seq 100 --limit 20
  python3 tools/map_event_query.py --file logs/spin.jsonl --trial-id run_01 --summary
"""

import argparse
import json
from collections import Counter
from pathlib import Path


def parse_bool(text):
    lowered = text.strip().lower()
    if lowered in {"1", "true", "yes", "y", "on"}:
        return True
    if lowered in {"0", "false", "no", "n", "off"}:
        return False
    raise argparse.ArgumentTypeError(f"invalid boolean value: {text}")


def iter_events(path: Path):
    with path.open("r", encoding="utf-8") as fp:
        for lineno, line in enumerate(fp, start=1):
            line = line.strip()
            if not line:
                continue
            try:
                yield json.loads(line)
            except json.JSONDecodeError as exc:
                raise ValueError(f"invalid JSON on line {lineno}: {exc}") from exc


def event_matches(ev, args):
    if args.after_seq is not None and ev.get("seq") is not None and ev["seq"] <= args.after_seq:
        return False
    if (
        args.after_stamp_ns is not None
        and ev.get("stamp_ns") is not None
        and ev["stamp_ns"] <= args.after_stamp_ns
    ):
        return False
    if args.record_type is not None and ev.get("record_type") != args.record_type:
        return False
    if args.trial_id is not None and ev.get("trial_id") != args.trial_id:
        return False
    if args.session_id is not None and ev.get("session_id") != args.session_id:
        return False
    if args.session_index is not None and ev.get("session_index") != args.session_index:
        return False
    if args.session_active is not None and ev.get("session_active") != args.session_active:
        return False
    if args.spin_score_gte is not None:
        score = ev.get("spin_score")
        if score is None or score < args.spin_score_gte:
            return False
    return True


def project_event(ev, fields):
    if not fields:
        return ev
    return {field: ev.get(field) for field in fields}


def summarize_events(events):
    counts = Counter()
    trial_ids = Counter()
    session_ids = []
    for ev in events:
        counts[ev.get("record_type", "unknown")] += 1
        if ev.get("trial_id") is not None:
            trial_ids[ev["trial_id"]] += 1
        if ev.get("record_type") == "spin_exit":
            session_ids.append(
                {
                    "trial_id": ev.get("trial_id"),
                    "session_id": ev.get("session_id"),
                    "session_index": ev.get("session_index"),
                    "duration_sec": ev.get("duration_sec"),
                    "odom_delta_yaw_deg": ev.get("odom_delta_yaw_deg"),
                    "tf_delta_yaw_deg": ev.get("tf_delta_yaw_deg"),
                    "imu_delta_yaw_raw_deg": ev.get("imu_delta_yaw_raw_deg"),
                    "ratio_odom_over_imu_raw": ev.get("ratio_odom_over_imu_raw"),
                    "ratio_tf_over_imu_raw": ev.get("ratio_tf_over_imu_raw"),
                    "pose_minus_tf_delta_yaw_deg": ev.get("pose_minus_tf_delta_yaw_deg"),
                    "mean_abs_pose_tf_yaw_diff_deg": ev.get("mean_abs_pose_tf_yaw_diff_deg"),
                    "peak_abs_pose_tf_yaw_diff_deg": ev.get("peak_abs_pose_tf_yaw_diff_deg"),
                    "mean_pose_tf_pos_err_m": ev.get("mean_pose_tf_pos_err_m"),
                    "peak_pose_tf_pos_err_m": ev.get("peak_pose_tf_pos_err_m"),
                    "peak_spin_score": ev.get("peak_spin_score"),
                    "exit_reason": ev.get("exit_reason"),
                }
            )
    return {
        "event_count": sum(counts.values()),
        "record_type_counts": dict(counts),
        "trial_id_counts": dict(trial_ids),
        "spin_sessions": session_ids,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--file", required=True, type=Path, help="JSONL log file")
    parser.add_argument("--after-seq", type=int, default=None, help="cursor: seq must be greater")
    parser.add_argument(
        "--after-stamp-ns",
        type=int,
        default=None,
        help="cursor: stamp_ns must be greater",
    )
    parser.add_argument("--record-type", default=None, help="filter by record_type")
    parser.add_argument("--trial-id", default=None, help="filter by trial_id")
    parser.add_argument("--session-id", default=None, help="filter by session_id")
    parser.add_argument("--session-index", type=int, default=None, help="filter by session_index")
    parser.add_argument(
        "--session-active",
        type=parse_bool,
        default=None,
        help="filter by session_active true/false",
    )
    parser.add_argument(
        "--spin-score-gte",
        type=float,
        default=None,
        help="filter by minimum spin_score",
    )
    parser.add_argument(
        "--limit",
        type=int,
        default=50,
        help="maximum matching events to print, 0 means no limit",
    )
    parser.add_argument(
        "--fields",
        default="",
        help="comma-separated projection fields, empty prints full event",
    )
    parser.add_argument(
        "--pretty",
        action="store_true",
        help="pretty-print JSON instead of compact one-line JSON",
    )
    parser.add_argument(
        "--summary",
        action="store_true",
        help="print summary of matched events after event output",
    )
    args = parser.parse_args()

    fields = [field.strip() for field in args.fields.split(",") if field.strip()]
    matched = []
    emitted = 0
    unlimited = args.limit <= 0

    for event in iter_events(args.file.expanduser()):
        if not event_matches(event, args):
            continue
        matched.append(event)
        projected = project_event(event, fields)
        if unlimited or emitted < args.limit:
            if args.pretty:
                print(json.dumps(projected, ensure_ascii=True, indent=2, sort_keys=True))
            else:
                print(json.dumps(projected, ensure_ascii=True, sort_keys=True))
            emitted += 1

    if args.summary:
        print(json.dumps(summarize_events(matched), ensure_ascii=True, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
