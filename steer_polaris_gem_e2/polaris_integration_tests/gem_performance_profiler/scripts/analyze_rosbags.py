#!/usr/bin/env python3
"""
Analyze rosbag folders (one per signal) from the error-timing profiler.
Supports two scenarios: run_to_error (reaction to fault) and error_to_run (recovery).
Extracts event times and latencies, outputs CSV, histograms, and a statistics text file.
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
import matplotlib.ticker as mpticker

from gem_performance_profiler.performance_scenarios import (
    SIGNALS,
    ERROR_TOPIC_AND_PREDICATE,
    GOOD_TOPIC_AND_PREDICATE,
    SCENARIO_RUN_TO_ERROR,
    SCENARIO_ERROR_TO_RUN,
)

# Event constants
SYSTEM_STATE_ERROR = 2   # SystemStateStamped.STATE_ERROR
SYSTEM_STATE_RUNNING = 1
TASK_PAUSED = 1         # TaskPlannerStatus.STATUS_PAUSED_TASK
TASK_RUNNING = 2        # TaskPlannerStatus.STATUS_RUNNING_TASK
GOAL_PREEMPTED = 2      # actionlib_msgs.GoalStatus.PREEMPTED
GOAL_ACTIVE = 1

TOPIC_SYSTEM_STATE = "/system_state"
TOPIC_TASK_PLANNER_STATUS = "/task_planner_status"
TOPIC_NAV_STATUS = "/navigate_waypoints/status"
TOPIC_ACKERMANN = "/gem/ackermann_cmd"
ACKERMANN_SPEED_RUNNING = 1.0

# Histogram: bin edges and x-axis ticks aligned to grid; tick resolution >= 5 ms
MIN_TICK_RESOLUTION_S = 0.005  # 5 ms


def _stamp_to_sec(stamp):
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


# -----------------------------------------------------------------------------
# Run-to-error: extract events (only count downstream events after error in bag order)
# -----------------------------------------------------------------------------


def extract_events_run_to_error(bag_path, signal_name):
    """
    Extract event times for run_to_error scenario.
    Reference: time_error_injection. Downstream events only counted after injection in bag order.
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

    bag_time_error_injection = None

    with rosbag.Bag(bag_path, "r") as bag:
        for topic, msg, t in bag.read_messages():
            t_sec = _stamp_to_sec(t)
            if t_sec is None:
                continue

            if out["time_error_injection"] is None and topic == topic_err and is_error(msg):
                out["time_error_injection"] = get_time(msg, t)
                bag_time_error_injection = t_sec

            after_injection = bag_time_error_injection is not None and t_sec >= bag_time_error_injection
            ref = out["time_error_injection"]
            # Only record downstream times that are at or after the reference (avoids test noise / reordering)
            def _at_or_after(candidate):
                return candidate is not None and (ref is None or candidate >= ref)

            if after_injection and topic == TOPIC_SYSTEM_STATE and getattr(msg, "state", None) == SYSTEM_STATE_ERROR:
                if out["time_system_state_error"] is None:
                    cand = get_time(msg, t)
                    if _at_or_after(cand):
                        out["time_system_state_error"] = cand

            if after_injection and topic == TOPIC_TASK_PLANNER_STATUS and getattr(msg, "status", None) == TASK_PAUSED:
                if out["time_task_planner_paused"] is None:
                    cand = get_time(msg, t)
                    if _at_or_after(cand):
                        out["time_task_planner_paused"] = cand

            if after_injection and topic == TOPIC_NAV_STATUS and out["time_system_state_error"] is not None:
                status_list = getattr(msg, "status_list", []) or []
                if status_list:
                    last_status = getattr(status_list[-1], "status", None)
                    if last_status == GOAL_PREEMPTED and out["time_nav_preempted"] is None:
                        cand = get_time(msg, t)
                        if _at_or_after(cand):
                            out["time_nav_preempted"] = cand

            if after_injection and topic == TOPIC_ACKERMANN and getattr(msg, "speed", 1.0) < 0.01 and out["time_system_state_error"] is not None:
                if out["time_ackermann_zero"] is None and _at_or_after(t_sec):
                    out["time_ackermann_zero"] = t_sec

    return out


# -----------------------------------------------------------------------------
# Error-to-run: extract events (error cleared, then recovery events after in bag order)
# -----------------------------------------------------------------------------


def extract_events_error_to_run(bag_path, signal_name):
    """
    Extract event times for error_to_run (recovery) scenario.
    Reference: time_error_cleared. We need to see error first, then first "good" signal message.
    Recovery events only counted after error_cleared in bag order.
    """
    out = {
        "time_error_cleared": None,
        "time_system_state_running": None,
        "time_task_planner_running": None,
        "time_nav_active": None,
        "time_ackermann_positive": None,
    }
    topic_err, is_error = ERROR_TOPIC_AND_PREDICATE.get(
        signal_name, (None, lambda m: False)
    )
    topic_good, is_good = GOOD_TOPIC_AND_PREDICATE.get(
        signal_name, (None, lambda m: False)
    )
    if not topic_err or not topic_good:
        return out

    bag_time_error_seen = None   # bag time when we first saw error (so we look for good after)
    bag_time_error_cleared = None

    with rosbag.Bag(bag_path, "r") as bag:
        for topic, msg, t in bag.read_messages():
            t_sec = _stamp_to_sec(t)
            if t_sec is None:
                continue

            if topic == topic_err and is_error(msg) and bag_time_error_seen is None:
                bag_time_error_seen = t_sec

            if bag_time_error_seen is not None and topic == topic_good and is_good(msg):
                if out["time_error_cleared"] is None:
                    out["time_error_cleared"] = get_time(msg, t)
                    bag_time_error_cleared = t_sec

            after_cleared = bag_time_error_cleared is not None and t_sec >= bag_time_error_cleared
            ref = out["time_error_cleared"]
            # Only record downstream times that are at or after the reference (avoids test noise / reordering)
            def _at_or_after(candidate):
                return candidate is not None and (ref is None or candidate >= ref)

            if after_cleared and topic == TOPIC_SYSTEM_STATE and getattr(msg, "state", None) == SYSTEM_STATE_RUNNING:
                if out["time_system_state_running"] is None:
                    cand = get_time(msg, t)
                    if _at_or_after(cand):
                        out["time_system_state_running"] = cand

            if after_cleared and topic == TOPIC_TASK_PLANNER_STATUS and getattr(msg, "status", None) == TASK_RUNNING:
                if out["time_task_planner_running"] is None:
                    cand = get_time(msg, t)
                    if _at_or_after(cand):
                        out["time_task_planner_running"] = cand

            if after_cleared and topic == TOPIC_NAV_STATUS and out["time_system_state_running"] is not None:
                status_list = getattr(msg, "status_list", []) or []
                if status_list:
                    last_status = getattr(status_list[-1], "status", None)
                    if last_status == GOAL_ACTIVE and out["time_nav_active"] is None:
                        cand = get_time(msg, t)
                        if _at_or_after(cand):
                            out["time_nav_active"] = cand

            if after_cleared and topic == TOPIC_ACKERMANN and getattr(msg, "speed", 0.0) > ACKERMANN_SPEED_RUNNING and out["time_system_state_running"] is not None:
                if out["time_ackermann_positive"] is None and _at_or_after(t_sec):
                    out["time_ackermann_positive"] = t_sec

    return out


# -----------------------------------------------------------------------------
# Analyze directory and build DataFrame
# -----------------------------------------------------------------------------


def analyze_test_dir(test_dir, scenario, output_dir=None):
    """Scan test_dir for signal subdirs, analyze each bag, return DataFrame."""
    if output_dir is None:
        output_dir = test_dir
    os.makedirs(output_dir, exist_ok=True)

    if scenario == SCENARIO_RUN_TO_ERROR:
        extract_fn = extract_events_run_to_error
        time_ref = "time_error_injection"
        time_cols = [
            "time_error_injection",
            "time_system_state_error",
            "time_task_planner_paused",
            "time_nav_preempted",
            "time_ackermann_zero",
        ]
    else:
        extract_fn = extract_events_error_to_run
        time_ref = "time_error_cleared"
        time_cols = [
            "time_error_cleared",
            "time_system_state_running",
            "time_task_planner_running",
            "time_nav_active",
            "time_ackermann_positive",
        ]

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
            events = extract_fn(bag_path, signal_name)
            events["signal"] = signal_name
            events["bag"] = bag_path
            events["run"] = bag_file
            events["scenario"] = scenario
            rows.append(events)

    df = pd.DataFrame(rows)
    if df.empty:
        return df

    t0 = df[time_ref]
    for col in time_cols:
        if col == time_ref:
            continue
        if col not in df.columns:
            continue
        lat_col = "latency_" + col.replace("time_", "")
        df[lat_col] = np.where(
            t0.notna() & df[col].notna(),
            df[col] - t0,
            np.nan,
        )

    return df


def save_csv(df, output_path):
    df.to_csv(output_path, index=False)
    print("Wrote CSV: {}".format(output_path), file=sys.stderr)


def get_message_periods_ms(bag_path, topic):
    """
    Read messages for the given topic from the bag; return list of inter-message
    periods in milliseconds (using header.stamp when available, else bag time).
    """
    stamps = []
    with rosbag.Bag(bag_path, "r") as bag:
        for _, msg, t in bag.read_messages(topics=[topic]):
            ts = get_time(msg, t)
            if ts is not None:
                stamps.append(ts)
    stamps = sorted(stamps)
    if len(stamps) < 2:
        return []
    periods_s = np.diff(np.asarray(stamps, dtype=float))
    return (periods_s * 1000.0).tolist()  # convert to ms


def compute_period_stats_by_signal(df):
    """
    For each signal, collect message publish periods (ms) for /system_state and
    /task_planner_status from all bags in df. Return nested dict:
    signal -> topic -> {"min_ms", "max_ms", "mean_ms", "std_ms", "n_periods"}.
    """
    topics = [TOPIC_SYSTEM_STATE, TOPIC_TASK_PLANNER_STATUS]
    # (signal, bag) pairs - each row is one bag
    bags_per_signal = df[["signal", "bag"]].drop_duplicates()
    period_by_signal_topic = {}  # signal -> topic -> list of periods (ms)
    for _, row in bags_per_signal.iterrows():
        sig, bag_path = row["signal"], row["bag"]
        if sig not in period_by_signal_topic:
            period_by_signal_topic[sig] = {t: [] for t in topics}
        for topic in topics:
            period_by_signal_topic[sig][topic].extend(get_message_periods_ms(bag_path, topic))

    result = {}
    for sig in period_by_signal_topic:
        result[sig] = {}
        for topic in topics:
            periods = np.asarray(period_by_signal_topic[sig][topic])
            if len(periods) == 0:
                result[sig][topic] = {"min_ms": np.nan, "max_ms": np.nan, "mean_ms": np.nan, "std_ms": np.nan, "n_periods": 0}
            else:
                result[sig][topic] = {
                    "min_ms": float(np.min(periods)),
                    "max_ms": float(np.max(periods)),
                    "mean_ms": float(np.mean(periods)),
                    "std_ms": float(np.std(periods)) if len(periods) > 1 else 0.0,
                    "n_periods": len(periods),
                }
    return result


def compute_and_save_stats(df, output_path, scenario, time_ref_name):
    """Write stats: per-column valid count (success rate %), mean, std, min, max, 95%ile, 99%ile; per-signal min, max, 99%."""
    time_cols = [c for c in df.columns if c.startswith("time_") and c in df.columns]
    latency_cols = [c for c in df.columns if c.startswith("latency_")]

    n_total = len(df) if not df.empty else 0
    lines = [
        "Scenario: {}".format(scenario),
        "Reference time for latencies: {}".format(time_ref_name),
        "Total runs (rows): {}".format(n_total),
        "",
    ]

    for col in time_cols:
        valid = df[col].notna().sum()
        pct = 100.0 * valid / n_total if n_total else 0.0
        lines.append("--- {} ---".format(col))
        lines.append("  Instances (valid time): {}  (success rate: {:.1f}%)".format(valid, pct))
        lines.append("")

    for lat_col in sorted(latency_cols):
        # valid = df[lat_col].notna()
        # n_valid = valid.sum()
        # pct = 100.0 * n_valid / n_total if n_total else 0.0
        # subset = df.loc[valid, lat_col]
        # mean_s = subset.mean() if n_valid else np.nan
        # std_s = subset.std() if n_valid and n_valid > 1 else np.nan
        # min_s = subset.min() if n_valid else np.nan
        # max_s = subset.max() if n_valid else np.nan
        # p95 = np.nanpercentile(subset, 95) if n_valid else np.nan
        # p99 = np.nanpercentile(subset, 99) if n_valid else np.nan
        lines.append("--- {} (w.r.t. {}) ---".format(lat_col, time_ref_name))
        # lines.append("  Instances (valid): {}  (success rate: {:.1f}%)".format(n_valid, pct))
        # lines.append("  Mean reaction time (s): {:.4f}".format(mean_s))
        # lines.append("  Std reaction time (s): {:.4f}".format(std_s))
        # lines.append("  Min reaction time (s): {:.4f}".format(min_s))
        # lines.append("  Max reaction time (s): {:.4f}".format(max_s))
        # lines.append("  95% percentile reaction time (s): {:.4f}".format(p95))
        # lines.append("  99% percentile reaction time (s): {:.4f}".format(p99))
        # Per-signal: min, max, 99% for each signal (bag file that saw min/max in brackets)
        if "signal" in df.columns and "bag" in df.columns:
            lines.append("  Per signal (min, max, 99% latency in s; bag for min/max in brackets):")
            for sig in sorted(df["signal"].unique()):
                sub_df = df.loc[df["signal"] == sig, [lat_col, "bag"]].dropna(subset=[lat_col])
                if len(sub_df):
                    valid = sub_df[lat_col].notna()
                    n_valid = valid.sum()
                    pct = 100.0 * n_valid / len(sub_df) if len(sub_df) else 0.0
                    idx_min = sub_df[lat_col].idxmin()
                    idx_max = sub_df[lat_col].idxmax()
                    bag_min = os.path.basename(sub_df.loc[idx_min, "bag"])
                    bag_max = os.path.basename(sub_df.loc[idx_max, "bag"])
                    lines.append("    {}: success rate: {:.1f}%, min={:.4f} ({}), max={:.4f} ({}), mean={:.4f}, 50%={:.4f}, 95%={:.4f}, 99%={:.4f}, std={:.4f} seconds (n={})".format(
                        sig,
                        pct,
                        sub_df[lat_col].min(),
                        bag_min,
                        sub_df[lat_col].max(),
                        bag_max,
                        sub_df[lat_col].mean(),
                        np.nanpercentile(sub_df[lat_col], 50),
                        np.nanpercentile(sub_df[lat_col], 95),
                        np.nanpercentile(sub_df[lat_col], 99),
                        sub_df[lat_col].std(),
                        len(sub_df),
                    ))
                else:
                    lines.append("    {}: no valid samples".format(sig))
        lines.append("")

    # Message publish periods (ms) per signal for /system_state and /task_planner_status
    lines.append("--- Message publish periods (ms) per signal ---")
    try:
        period_stats = compute_period_stats_by_signal(df)
        for topic in [TOPIC_SYSTEM_STATE, TOPIC_TASK_PLANNER_STATUS]:
            lines.append("  Topic: {}".format(topic))
            for sig in sorted(period_stats.keys()):
                s = period_stats[sig].get(topic, {})
                n = s.get("n_periods", 0)
                if n == 0:
                    lines.append("    {}: no periods (too few messages)".format(sig))
                else:
                    lines.append("    {}: min={:.2f} ms, max={:.2f} ms, mean={:.2f} ms, std={:.2f} ms  (n={})".format(
                        sig, s["min_ms"], s["max_ms"], s["mean_ms"], s["std_ms"], n
                    ))
            lines.append("")
    except Exception as e:
        lines.append("  (error computing periods: {})".format(e))
        lines.append("")

    with open(output_path, "w") as f:
        f.write("\n".join(lines))
    print("Wrote stats: {}".format(output_path), file=sys.stderr)


def _aligned_bin_edges(data, min_tick_s=MIN_TICK_RESOLUTION_S, target_bins=30):
    """
    Return bin edges aligned to a grid that is a multiple of min_tick_s (e.g. 5 ms).
    Bin boundaries and ticks can share the same grid. Uses target_bins as a hint.
    """
    data = np.asarray(data)
    if data.size == 0:
        return np.array([0.0, min_tick_s])
    data_min, data_max = float(np.min(data)), float(np.max(data))
    span = data_max - data_min
    if span <= 0:
        span = min_tick_s
    raw_step = span / max(1, min(target_bins, data.size))
    step = max(min_tick_s, np.ceil(raw_step / min_tick_s) * min_tick_s)
    first_edge = np.floor(data_min / step) * step
    last_edge = np.ceil(data_max / step) * step
    edges = np.arange(first_edge, last_edge + step * 0.5, step)
    return edges


def _set_histogram_axis_ticks(ax, x_min, x_max, min_tick_s=MIN_TICK_RESOLUTION_S, max_ticks=15):
    """Set x-axis limits and major locator: tick spacing >= min_tick_s (5 ms), reasonable number of ticks."""
    span = x_max - x_min
    if span <= 0:
        span = min_tick_s
        x_max = x_min + span
    ax.set_xlim(x_min, x_max)
    raw_spacing = span / max(1, max_ticks - 1)
    spacing = max(min_tick_s, np.ceil(raw_spacing / min_tick_s) * min_tick_s)
    ax.xaxis.set_major_locator(mpticker.MultipleLocator(spacing))
    try:
        base_size = float(plt.rcParams.get("xtick.labelsize", 10))
    except (TypeError, ValueError):
        base_size = 10.0
    ax.tick_params(axis="x", labelsize=base_size * 0.85)


def plot_histograms(df, output_dir, scenario):
    latency_cols = [c for c in df.columns if c.startswith("latency_")]
    if not latency_cols:
        return

    prefix = scenario + "_"

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
            if len(data):
                # Use fewer bins so bars are wider and visible (target_bins capped at 20)
                edges = _aligned_bin_edges(data, target_bins=min(20, max(8, len(data) // 5)))
                axes[idx].hist(data, bins=edges, edgecolor="black", alpha=0.7)
                # Use bin edges so x-range extends slightly beyond data min/max
                x_min_s, x_max_s = float(edges[0]), float(edges[-1])
            else:
                axes[idx].hist([], bins=np.array([0.0, MIN_TICK_RESOLUTION_S]), edgecolor="black", alpha=0.7)
                x_min_s, x_max_s = 0.0, MIN_TICK_RESOLUTION_S
            # Per-subplot x range and ticks (each subplot scales to its own data)
            _set_histogram_axis_ticks(axes[idx], x_min_s, x_max_s)
            axes[idx].set_xlabel("Latency (s)")
            axes[idx].set_ylabel("Count")
            axes[idx].set_title(sig)
        for j in range(idx + 1, len(axes)):
            axes[j].set_visible(False)
        fig.tight_layout(rect=[0, 0, 1, 0.94])
        fig.suptitle("{} – {}".format(scenario, lat_col), y=0.98)
        out_path = os.path.join(output_dir, "{}histogram_by_signal_{}.png".format(prefix, lat_col))
        fig.savefig(out_path, dpi=150)
        plt.close(fig)
        print("Wrote {}".format(out_path), file=sys.stderr)


def main():
    parser = argparse.ArgumentParser(
        description="Analyze profiler rosbags: extract event times, CSV, histograms, stats.",
    )
    parser.add_argument(
        "test_dir",
        type=str,
        help="Root test directory containing signal subdirs with run_*.bag files.",
    )
    parser.add_argument(
        "--scenario",
        type=str,
        choices=[SCENARIO_RUN_TO_ERROR, SCENARIO_ERROR_TO_RUN],
        default=SCENARIO_RUN_TO_ERROR,
        help="Scenario that produced the bags: run_to_error or error_to_run. Default: run_to_error.",
    )
    parser.add_argument(
        "--output-dir",
        type=str,
        default=None,
        help="Directory for CSV, histograms, and stats (default: same as test_dir).",
    )
    args = parser.parse_args()

    test_dir = os.path.abspath(args.test_dir)
    if not os.path.isdir(test_dir):
        print("Not a directory: {}".format(test_dir), file=sys.stderr)
        sys.exit(1)

    output_dir = os.path.abspath(args.output_dir) if args.output_dir else test_dir
    scenario = args.scenario

    if not rospy.core.is_initialized():
        rospy.init_node("analyze_rosbags", anonymous=True)

    df = analyze_test_dir(test_dir, scenario, output_dir)
    if df.empty:
        print("No bags found under {}".format(test_dir), file=sys.stderr)
        sys.exit(0)

    time_ref = "time_error_injection" if scenario == SCENARIO_RUN_TO_ERROR else "time_error_cleared"
    csv_path = os.path.join(output_dir, "event_times_and_latencies_{}.csv".format(scenario))
    save_csv(df, csv_path)

    stats_path = os.path.join(output_dir, "latency_statistics_{}.txt".format(scenario))
    compute_and_save_stats(df, stats_path, scenario, time_ref)

    plot_histograms(df, output_dir, scenario)

    print("Done. CSV: {}, Stats: {}, Histograms in {}".format(
        csv_path, stats_path, output_dir
    ), file=sys.stderr)


if __name__ == "__main__":
    main()
