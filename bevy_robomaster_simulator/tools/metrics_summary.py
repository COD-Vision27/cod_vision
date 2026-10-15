#!/usr/bin/env python3
"""Validate and summarize simulation telemetry JSONL (schema v1).

Physical event rates and visual target metrics are intentionally reported as
separate sections.  Visual metrics require explicit estimate events; shot
outcomes are never used as a proxy for detector precision or recall.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from collections import Counter
from pathlib import Path

SCHEMA = "daedalus.simulation_metrics"
VERSION = 1
ESTIMATE_EVENTS = {"visual_estimate", "vision_estimate", "target_estimate"}
STATE_EVENTS = {"state_estimate"}


def _finite_number(value: object) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def read_events(path: Path) -> list[dict]:
    events: list[dict] = []
    for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        if not line.strip():
            continue
        try:
            event = json.loads(line)
        except json.JSONDecodeError as error:
            raise ValueError(f"line {line_number}: invalid JSON: {error}") from error
        if not isinstance(event, dict):
            raise ValueError(f"line {line_number}: event must be a JSON object")
        if event.get("schema") != SCHEMA or event.get("version") != VERSION or isinstance(event.get("version"), bool):
            raise ValueError(f"line {line_number}: expected {SCHEMA} version {VERSION}")
        if not isinstance(event.get("event"), str) or not event["event"]:
            raise ValueError(f"line {line_number}: event must be a non-empty string")
        for field in ("frame_seq", "launch_frame_seq", "talos_frame_seq"):
            if field in event and (not isinstance(event[field], int) or isinstance(event[field], bool) or event[field] < 0):
                raise ValueError(f"line {line_number}: {field} must be a non-negative integer")
        for field in ("timestamp_ns", "sim_timestamp_ns", "launch_timestamp_ns", "talos_timestamp_ns"):
            if field in event and (not isinstance(event[field], int) or isinstance(event[field], bool) or event[field] < 0):
                raise ValueError(f"line {line_number}: {field} must be a non-negative integer")
        if event["event"] in STATE_EVENTS:
            target_index = event.get("target_index")
            if not isinstance(target_index, int) or isinstance(target_index, bool) or target_index < 0:
                raise ValueError(f"line {line_number}: state_estimate target_index must be non-negative")
            if event.get("target_index_scope") != "frame":
                raise ValueError(f"line {line_number}: state_estimate target_index_scope must be frame")
            tracker_state = event.get("tracker_state")
            if not isinstance(tracker_state, str) or not tracker_state:
                raise ValueError(f"line {line_number}: state_estimate tracker_state must be non-empty")
            if not isinstance(event.get("state_valid"), bool):
                raise ValueError(f"line {line_number}: state_estimate state_valid must be boolean")
            ekf_x = event.get("ekf_x")
            if event["state_valid"] and (
                not isinstance(ekf_x, list)
                or not ekf_x
                or not all(_finite_number(value) for value in ekf_x)
            ):
                raise ValueError(f"line {line_number}: valid state_estimate requires finite ekf_x")
            if ekf_x is not None and (
                not isinstance(ekf_x, list)
                or not all(_finite_number(value) for value in ekf_x)
            ):
                raise ValueError(f"line {line_number}: state_estimate ekf_x must be finite numbers")
            xyza = event.get("armor_xyza_list")
            if xyza is not None and (
                not isinstance(xyza, list)
                or any(
                    not isinstance(item, list)
                    or len(item) != 4
                    or not all(_finite_number(value) for value in item)
                    for item in xyza
                )
            ):
                raise ValueError(f"line {line_number}: state_estimate armor_xyza_list must contain finite 4-vectors")
        events.append(event)
    return events


def _angle(event: dict, name: str) -> float | None:
    """Read an explicitly degree-labelled angle; do not infer units."""
    value = event.get(f"{name}_deg")
    if value is None and isinstance(event.get("angles"), dict):
        value = event["angles"].get(f"{name}_deg")
    return float(value) if _finite_number(value) else None


def _wrap_degrees(value: float) -> float:
    return (value + 180.0) % 360.0 - 180.0


def _target_key(event: dict) -> object:
    return event.get("target_index", event.get("target_id"))


def _hungarian(cost: list[list[float]]) -> list[int]:
    """Return the minimum-cost column for every row (rectangular matrix).

    The implementation follows the O(rows * cols * min(rows, cols)) shortest
    augmenting-path form of the Hungarian algorithm.  ``cost`` must have at
    least as many columns as rows; this lets callers represent an unmatched
    row with an additional dummy column instead of enumerating subsets.
    """
    rows = len(cost)
    cols = len(cost[0]) if rows else 0
    if any(len(row) != cols for row in cost):
        raise ValueError("Hungarian cost matrix must be rectangular")
    if rows > cols:
        raise ValueError("Hungarian cost matrix needs at least as many columns as rows")
    if not rows:
        return []

    # p[j] is the row currently assigned to column j; way[j] stores the
    # predecessor column used by the augmenting path ending at j.
    potential_row = [0.0] * (rows + 1)
    potential_col = [0.0] * (cols + 1)
    assigned_col = [0] * (cols + 1)
    predecessor = [0] * (cols + 1)
    for row in range(1, rows + 1):
        assigned_col[0] = row
        column = 0
        minimum = [math.inf] * (cols + 1)
        used = [False] * (cols + 1)
        while True:
            used[column] = True
            current_row = assigned_col[column]
            delta = math.inf
            next_column = 0
            for candidate in range(1, cols + 1):
                if used[candidate]:
                    continue
                reduced = (
                    cost[current_row - 1][candidate - 1]
                    - potential_row[current_row]
                    - potential_col[candidate]
                )
                if reduced < minimum[candidate]:
                    minimum[candidate] = reduced
                    predecessor[candidate] = column
                if minimum[candidate] < delta:
                    delta = minimum[candidate]
                    next_column = candidate
            for candidate in range(cols + 1):
                if used[candidate]:
                    potential_row[assigned_col[candidate]] += delta
                    potential_col[candidate] -= delta
                else:
                    minimum[candidate] -= delta
            column = next_column
            if assigned_col[column] == 0:
                break
        while True:
            previous = predecessor[column]
            assigned_col[column] = assigned_col[previous]
            column = previous
            if column == 0:
                break

    assignment = [-1] * rows
    for column in range(1, cols + 1):
        if assigned_col[column]:
            assignment[assigned_col[column] - 1] = column - 1
    return assignment


def _global_pairs(
    estimates: list[dict], ground_truth: list[dict]
) -> list[tuple[dict, dict, float, float]]:
    """Find a deterministic minimum-cost one-to-one assignment in O(n^3).

    Every estimate receives either one eligible ground-truth column or a dummy
    unmatched column.  The dummy cost is larger than any possible total real
    edge cost, so cardinality is maximized before angular distance is
    minimized.  A keyed estimate is restricted to ground truth records with
    the same key, preserving the explicit-ID priority used by the old matcher.
    """
    if not estimates or not ground_truth:
        return []

    edge_data: list[dict[int, tuple[float, float, float]]] = []
    max_distance = 0.0
    for estimate in estimates:
        ey, ep = _angle(estimate, "yaw"), _angle(estimate, "pitch")
        keyed_candidates = [
            index for index, target in enumerate(ground_truth)
            if _target_key(target) is not None
            and _target_key(target) == _target_key(estimate)
        ]
        candidates = keyed_candidates or list(range(len(ground_truth)))
        edges: dict[int, tuple[float, float, float]] = {}
        for target_index in candidates:
            gy = _angle(ground_truth[target_index], "yaw")
            gp = _angle(ground_truth[target_index], "pitch")
            dy, dp = _wrap_degrees(ey - gy), ep - gp
            distance = math.hypot(dy, dp)
            edges[target_index] = (dy, dp, distance)
            max_distance = max(max_distance, distance)
        edge_data.append(edges)

    scale = max_distance if max_distance > 0.0 and math.isfinite(max_distance) else 1.0
    # A real edge costs at most 1 after scaling.  Thus one extra real match is
    # always preferable to any possible increase in total real-edge cost.
    unmatched_cost = float(len(estimates) + 1)
    blocked_cost = unmatched_cost * float(len(estimates) + len(ground_truth) + 1)
    cost = []
    for edges in edge_data:
        row = [blocked_cost] * len(ground_truth)
        for target_index, (_, _, distance) in edges.items():
            row[target_index] = distance / scale
        row.extend([unmatched_cost] * len(estimates))
        cost.append(row)

    assignment = _hungarian(cost)
    pairs = []
    for estimate_index, target_index in enumerate(assignment):
        edge = edge_data[estimate_index].get(target_index)
        if edge is None:
            continue
        dy, dp, _ = edge
        pairs.append((estimates[estimate_index], ground_truth[target_index], dy, dp))
    return pairs


def pair_visual_targets(events: list[dict], angle_threshold_deg: float = 2.0) -> dict:
    """Pair explicit visual estimates to GT in each frame and report errors.

    Events without both ``yaw_deg`` and ``pitch_deg`` remain counted as
    available records but cannot contribute angular errors.  Matching uses an
    explicit target id when present, otherwise nearest angular distance within
    the same frame; each record is consumed at most once.
    """
    if not _finite_number(angle_threshold_deg) or angle_threshold_deg <= 0:
        raise ValueError("angle threshold must be a positive finite number")
    gt_target = [e for e in events if e.get("event") == "ground_truth_target"]
    gt_armor = [e for e in events if e.get("event") == "ground_truth_armor"]
    armor_frames = {
        e["frame_seq"] for e in gt_armor
        if isinstance(e.get("frame_seq"), int) and not isinstance(e.get("frame_seq"), bool)
    }
    # Prefer per-armor truth whenever it exists for a frame; root-center truth
    # is retained as a fallback for captures produced by older builds.
    gt = gt_armor + [
        e for e in gt_target
        if e.get("frame_seq") not in armor_frames
    ]
    estimates = [e for e in events if e.get("event") in ESTIMATE_EVENTS]
    gt_by_frame: dict[int, list[dict]] = {}
    est_by_frame: dict[int, list[dict]] = {}
    for item in gt:
        if isinstance(item.get("frame_seq"), int) and not isinstance(item.get("frame_seq"), bool):
            dest = gt_by_frame.setdefault(item["frame_seq"], [])
            # Targets outside the captured image are not recall opportunities.
            in_view = item.get("visible", True)
            if (in_view is not False and _angle(item, "yaw") is not None
                    and _angle(item, "pitch") is not None):
                dest.append(item)
    for item in estimates:
        if isinstance(item.get("frame_seq"), int) and not isinstance(item.get("frame_seq"), bool):
            dest = est_by_frame.setdefault(item["frame_seq"], [])
            if _angle(item, "yaw") is not None and _angle(item, "pitch") is not None:
                dest.append(item)

    pairs: list[tuple[dict, dict, float, float]] = []
    for frame in sorted(set(gt_by_frame) | set(est_by_frame)):
        pairs.extend(
            _global_pairs(est_by_frame.get(frame, []), gt_by_frame.get(frame, []))
        )

    accepted = [(dy, dp) for _, _, dy, dp in pairs if math.hypot(dy, dp) <= angle_threshold_deg]
    errors = [math.hypot(dy, dp) for _, _, dy, dp in pairs]
    tp = len(accepted)
    gt_count = sum(len(v) for v in gt_by_frame.values())
    estimate_count = sum(len(v) for v in est_by_frame.values())
    precision = tp / estimate_count if estimate_count else 0.0
    recall = tp / gt_count if gt_count else 0.0
    return {
        "ground_truth_with_angles": gt_count,
        "estimates_with_angles": estimate_count,
        "matched_pairs": len(pairs),
        "true_positives": tp,
        "false_positives": estimate_count - tp,
        "false_negatives": gt_count - tp,
        "precision": precision,
        "recall": recall,
        "f1": 2 * precision * recall / (precision + recall) if precision + recall else 0.0,
        "angle_threshold_deg": angle_threshold_deg,
        "angular_error_mae_deg": sum(errors) / len(errors) if errors else 0.0,
        "angular_error_p95_deg": sorted(errors)[max(0, math.ceil(0.95 * len(errors)) - 1)] if errors else 0.0,
    }


def summarize(events: list[dict], angle_threshold_deg: float = 2.0) -> dict:
    fired = [event for event in events if event.get("event") == "shot_fired"]
    finished = [event for event in events if event.get("event") == "shot_finished"]
    ground_truth = [
        event for event in events if event.get("event") == "ground_truth_target"
    ]
    outcomes = Counter(event.get("outcome", "unknown") for event in finished)
    kinds = Counter(event.get("kind", "unknown") for event in fired)
    state_events = [event for event in events if event.get("event") in STATE_EVENTS]
    projectile_finished = [
        event for event in finished if event.get("kind") == "projectile"
    ]
    projectile_armor_hits = sum(
        event.get("outcome") == "armor_hit" for event in projectile_finished
    )
    return {
        "events": len(events),
        "ground_truth_targets": len(ground_truth),
        "shots": len(fired),
        "finished": len(finished),
        "pending": len(fired) - len(finished),
        "projectile_shots": kinds.get("projectile", 0),
        "dart_shots": kinds.get("dart", 0),
        "armor_hits": outcomes.get("armor_hit", 0),
        "projectile_armor_hits": projectile_armor_hits,
        "rune_hits": outcomes.get("rune_hit", 0),
        "rune_misses": outcomes.get("rune_miss", 0),
        "misses": outcomes.get("pending", 0),
        "state_estimates": len(state_events),
        "invalid_state_estimates": sum(
            not event.get("state_valid", False) for event in state_events
        ),
        "projectile_armor_accuracy": (
            projectile_armor_hits / kinds.get("projectile", 0)
            if kinds.get("projectile", 0)
            else 0.0
        ),
        "visual": pair_visual_targets(events, angle_threshold_deg),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("metrics", type=Path, help="metrics JSONL path")
    parser.add_argument("--angle-threshold-deg", type=float, default=2.0,
                        help="visual match threshold in degrees (default: 2)")
    args = parser.parse_args()
    try:
        summary = summarize(read_events(args.metrics), args.angle_threshold_deg)
    except (OSError, ValueError) as error:
        print(f"metrics validation failed: {error}", file=sys.stderr)
        return 2
    print(json.dumps(summary, ensure_ascii=True, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
