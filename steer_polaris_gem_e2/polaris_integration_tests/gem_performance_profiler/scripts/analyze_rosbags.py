#!/usr/bin/env python3
"""
Analyze rosbag folders (one per signal) from the error-timing profiler.
Extracts event times and latencies w.r.t. error injection, outputs CSV,
histograms, and a statistics text file.
"""
from __future__ import print_function

import argparse
import os
import sys

import rosbag
import rospy

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# Signal names (must match run_error_timing_profiler.SIGNALS)
SIGNALS = [
    "battery_level",
    "estop",
    "temperature",
    "gps_accuracy",
    "network_strength_low_signal",
    "network_strength_not_connected",
]

# Topic and predicate for "first error-inducing message" per signal
# Predicate: callable(msg) -> bool
def _error_predicate_battery(msg):
    return getattr(msg, "battery_level", 1.0) <= 0.5

def _error_predicate_estop(msg):
    return getattr(msg, "enabled", True)

def _error_predicate_temperature(msg):
    return getattr(msg, "temperature", 0.0) >= 50.0

def _error_predicate_gps(msg):
    return getattr(msg, "accuracy", 0.0) > 200.0

def _error_predicate_signal_low(msg):
    return getattr(msg, "strength", 1) == 2  # LOW_SIGNAL

def _error_predicate_signal_not_connected(msg):
    return getattr(msg, "strength", 1) == 0  # NOT_CONNECTED

ERROR_TOPIC_AND_PREDICATE = {
    "battery_level": ("/battery_level", _error_predicate_battery),
    "estop": ("/estop", _error_predicate_estop),
    "temperature": ("/temperature", _error_predicate_temperature),
    "gps_accuracy": ("/gps_accuracy", _error_predicate_gps),
    "network_strength_low_signal": ("/signal_strength", _error_predicate_signal_low),
    "network_strength_not_connected": ("/signal_strength", _error_predicate_signal_not_connected),
}

SYSTEM_STATE_ERROR = 2   # SystemStateStamped.STATE_ERROR
TASK_PAUSED = 1          # TaskPlannerStatus.STATUS_PAUSED_TASK
GOAL_PREEMPTED = 2       # actionlib_msgs.GoalStatus.PREEMPTED

TOPIC_SYSTEM_STATE = "/system_state"
TOPIC_TASK_PLANNER_STATUS = "/task_planner_status"
TOPIC_NAV_STATUS = "/navigate_waypoints/status"
TOPIC_ACKERMANN = "/gem/ackermann_cmd"


def _stamp_to_sec(stamp):
    """Convert rospy.Time or genpy.Time to float seconds."""
    if stamp is None:
        return None
    if hasattr(stamp, "to_sec"):
        return stamp.to_sec()
    return getattr(stamp, "secs", 0) + getattr(stamp, "nsecs", 0) * 1e-9


def get_time(msg, bag_t):
    """Prefer header.stamp, else bag time. Returns float seconds or None."""
    try:
        if hasattr(msg, "header") and hasattr(msg.header, "stamp"):
            return _stamp_to_sec(msg.header.stamp)
    except Exception:
        pass
    return _stamp_to_sec(bag_t) if bag_t is not None else None


def extract_events_from_bag(bag_path, signal_name):
    """
    Read one bag and return dict with keys:
    time_error_injection, time_system_state_error, time_task_planner_paused,
    time_nav_preempted, time_ackermann_zero.
    Values are float seconds (epoch or relative doesn't matter for latencies) or None if not found.
    """
    out = {
        "time_error_injection": None,
        "time_system_state_error": None,
        "time_task_planner_paused": None,
        "time_nav_preempted": None,
        "time_ackermann_zero": None,
    }
    topic_err, is_error = ERROR_TOPIC_AND_PREDICATE.get(
        signal_name, (None, lambda m: False)
    )
    if not topic_err:
        return out

    with rosbag.Bag(bag_path, "r") as bag:
        for topic, msg, t in bag.read_messages():
            t_sec = _stamp_to_sec(t)
            if t_sec is None:
                continue

            # Error-inducing message (first occurrence)
            if out["time_error_injection"] is None and topic == topic_err and is_error(msg):
                out["time_error_injection"] = get_time(msg, t)

            if topic == TOPIC_SYSTEM_STATE and getattr(msg, "state", None) == SYSTEM_STATE_ERROR:
                if out["time_system_state_error"] is None:
                    out["time_system_state_error"] = get_time(msg, t)

            if topic == TOPIC_TASK_PLANNER_STATUS and getattr(msg, "status", None) == TASK_PAUSED:
                if out["time_task_planner_paused"] is None:
                    out["time_task_planner_paused"] = get_time(msg, t)

            if topic == TOPIC_NAV_STATUS:
                status_list = getattr(msg, "status_list", []) or []
                if status_list:
                    last_status = getattr(status_list[-1], "status", None)
                    if last_status == GOAL_PREEMPTED and out["time_nav_preempted"] is None:
                        out["time_nav_preempted"] = get_time(msg, t)

            if topic == TOPIC_ACKERMANN and getattr(msg, "speed", 1.0) < 0.01:
                if out["time_ackermann_zero"] is None:
                    # No header on AckermannDrive; use bag time
                    out["time_ackermann_zero"] = t_sec

    return out


def analyze_test_dir(test_dir, output_dir=None):
    """Scan test_dir for signal subdirs, analyze each bag, return DataFrame."""
    if output_dir is None:
        output_dir = test_dir
    os.makedirs(output_dir, exist_ok=True)

    rows = []
    for signal_name in SIGNALS:
        subdir = os.path.join(test_dir, signal_name)
        if not os.path.isdir(subdir):
            continue
        bag_files = sorted(
            f for f in os.listdir(subdir)
            if f.endswith(".bag") and f.startswith("run_")
        )
        for bag_file in bag_files:
            bag_path = os.path.join(subdir, bag_file)
            events = extract_events_from_bag(bag_path, signal_name)
            events["signal"] = signal_name
            events["bag"] = bag_path
            events["run"] = bag_file
            rows.append(events)

    df = pd.DataFrame(rows)
    if df.empty:
        return df

    # Reference time for latency: error injection
    t0 = df["time_error_injection"]
    for col in [
        "time_system_state_error",
        "time_task_planner_paused",
        "time_nav_preempted",
        "time_ackermann_zero",
    ]:
        lat_col = "latency_" + col.replace("time_", "")
        df[lat_col] = np.where(
            t0.notna() & df[col].notna(),
            df[col] - t0,
            np.nan,
        )

    return df


def save_csv(df, output_path):
    """Write DataFrame to CSV."""
    df.to_csv(output_path, index=False)
    print("Wrote CSV: {}".format(output_path), file=sys.stderr)


def compute_and_save_stats(df, output_path):
    """
    For each latency column: count valid, mean, std, 95th percentile.
    Write a text file with success rate %, mean, std, 95%ile.
    """
    time_cols = [
        "time_error_injection",
        "time_system_state_error",
        "time_task_planner_paused",
        "time_nav_preempted",
        "time_ackermann_zero",
    ]
    latency_cols = [c for c in df.columns if c.startswith("latency_")]

    n_total = len(df) if not df.empty else 0
    lines = [
        "Latency statistics (reaction time w.r.t. error-inducing message timestamp)",
        "Total runs (rows): {}".format(n_total),
        "",
    ]

    for col in time_cols:
        if col not in df.columns:
            continue
        valid = df[col].notna().sum()
        pct = 100.0 * valid / n_total if n_total else 0.0
        lines.append("--- {} ---".format(col))
        lines.append("  Instances (valid time): {}  (success rate: {:.1f}%)".format(valid, pct))
        lines.append("")

    for lat_col in sorted(latency_cols):
        if lat_col not in df.columns:
            continue
        valid = df[lat_col].notna()
        n_valid = valid.sum()
        pct = 100.0 * n_valid / n_total if n_total else 0.0
        subset = df.loc[valid, lat_col]
        mean_s = subset.mean() if n_valid else np.nan
        std_s = subset.std() if n_valid and n_valid > 1 else np.nan
        p95 = np.nanpercentile(subset, 95) if n_valid else np.nan
        lines.append("--- {} (w.r.t. time_error_injection) ---".format(lat_col))
        lines.append("  Instances (valid): {}  (success rate: {:.1f}%)".format(n_valid, pct))
        lines.append("  Mean reaction time (s): {:.4f}".format(mean_s))
        lines.append("  Std reaction time (s): {:.4f}".format(std_s))
        lines.append("  95% percentile reaction time (s): {:.4f}".format(p95))
        lines.append("")

    with open(output_path, "w") as f:
        f.write("\n".join(lines))
    print("Wrote stats: {}".format(output_path), file=sys.stderr)


def plot_histograms(df, output_dir):
    """Plot histogram of latency for each latency column, per signal and overall."""
    latency_cols = [c for c in df.columns if c.startswith("latency_")]
    if not latency_cols:
        return

    for lat_col in latency_cols:
        # Overall histogram
        data = df[lat_col].dropna()
        if data.empty:
            continue
        fig, ax = plt.subplots()
        ax.hist(data, bins=min(50, max(10, len(data) // 5)), edgecolor="black", alpha=0.7)
        ax.set_xlabel("Latency (s)")
        ax.set_ylabel("Count")
        ax.set_title("Latency: {}".format(lat_col))
        fig.tight_layout()
        out_path = os.path.join(output_dir, "histogram_{}.png".format(lat_col))
        fig.savefig(out_path, dpi=150)
        plt.close(fig)
        print("Wrote {}".format(out_path), file=sys.stderr)

    # Per-signal histograms (one figure per latency column, subplots per signal)
    for lat_col in latency_cols:
        signals_in_df = df["signal"].unique() if "signal" in df.columns else []
        if not list(signals_in_df):
            continue
        n_sigs = len(signals_in_df)
        n_cols = min(3, n_sigs)
        n_rows = (n_sigs + n_cols - 1) // n_cols
        fig, axes = plt.subplots(n_rows, n_cols, figsize=(5 * n_cols, 4 * n_rows))
        if n_sigs == 1:
            axes = np.array([axes])
        axes = axes.flatten()
        for idx, sig in enumerate(signals_in_df):
            data = df.loc[df["signal"] == sig, lat_col].dropna()
            axes[idx].hist(
                data,
                bins=min(30, max(5, len(data) // 3)),
                edgecolor="black",
                alpha=0.7,
            )
            axes[idx].set_xlabel("Latency (s)")
            axes[idx].set_ylabel("Count")
            axes[idx].set_title("{} – {}".format(lat_col, sig))
        for j in range(idx + 1, len(axes)):
            axes[j].set_visible(False)
        fig.suptitle("Latency by signal: {}".format(lat_col), y=1.02)
        fig.tight_layout()
        out_path = os.path.join(output_dir, "histogram_by_signal_{}.png".format(lat_col))
        fig.savefig(out_path, dpi=150)
        plt.close(fig)
        print("Wrote {}".format(out_path), file=sys.stderr)


def main():
    parser = argparse.ArgumentParser(
        description="Analyze profiler rosbags: extract event times, CSV, histograms, stats."
    )
    parser.add_argument(
        "test_dir",
        type=str,
        help="Root test directory containing signal subdirs (e.g. battery_level/, ...) with run_*.bag files.",
    )
    parser.add_argument(
        "--output-dir",
        type=str,
        default=None,
        help="Directory for CSV, histograms, and stats file (default: same as test_dir).",
    )
    args = parser.parse_args()

    test_dir = os.path.abspath(args.test_dir)
    if not os.path.isdir(test_dir):
        print("Not a directory: {}".format(test_dir), file=sys.stderr)
        sys.exit(1)

    output_dir = os.path.abspath(args.output_dir) if args.output_dir else test_dir

    # Avoid ROS init if we only need rosbag (rosbag doesn't require a node)
    if not rospy.core.is_initialized():
        rospy.init_node("analyze_rosbags", anonymous=True)

    df = analyze_test_dir(test_dir, output_dir)
    if df.empty:
        print("No bags found under {}".format(test_dir), file=sys.stderr)
        sys.exit(0)

    csv_path = os.path.join(output_dir, "event_times_and_latencies.csv")
    save_csv(df, csv_path)

    stats_path = os.path.join(output_dir, "latency_statistics.txt")
    compute_and_save_stats(df, stats_path)

    plot_histograms(df, output_dir)

    print("Done. CSV: {}, Stats: {}, Histograms in {}".format(
        csv_path, stats_path, output_dir
    ), file=sys.stderr)


if __name__ == "__main__":
    main()
