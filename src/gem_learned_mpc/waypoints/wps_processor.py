#!/usr/bin/env python3
"""Process a headerless 5-column waypoint CSV into an MPC reference profile.
Input columns: x, y, raw_yaw, signed_speed_or_aux, csv_speed.
Output columns: s, x, y, yaw, curvature, csv_speed, curve_speed, reference_speed.
"""
from __future__ import annotations
import argparse
from pathlib import Path
import numpy as np

COLS = ("x", "y", "raw_yaw", "speed_signed", "csv_speed")


def load_waypoints(path: str | Path) -> np.ndarray:
    data = np.genfromtxt(path, delimiter=",", dtype=float)
    if data.ndim == 1:
        data = data.reshape(1, -1)
    if data.shape[1] < 5:
        raise ValueError(f"CSV must have at least 5 columns, got {data.shape[1]}")
    data = data[:, :5]
    data = data[np.isfinite(data).all(axis=1)]
    if len(data) < 3:
        raise ValueError("At least 3 finite waypoint rows are required")
    return data


def remove_close_neighbors(data: np.ndarray, min_distance: float = 0.02) -> np.ndarray:
    """Keep first point and each later point sufficiently far from last kept point.
    The final point is retained when possible so the route endpoint is not lost.
    """
    keep = [0]
    for i in range(1, len(data)):
        if np.hypot(*(data[i, :2] - data[keep[-1], :2])) >= min_distance:
            keep.append(i)
    if keep[-1] != len(data) - 1:
        if np.hypot(*(data[-1, :2] - data[keep[-1], :2])) > 1e-9:
            keep.append(len(data) - 1)
    out = data[np.asarray(keep)]
    if len(out) < 3:
        raise ValueError("Too few points remain after neighbor filtering")
    return out


def cumulative_arc_length(x: np.ndarray, y: np.ndarray) -> np.ndarray:
    ds = np.hypot(np.diff(x), np.diff(y))
    return np.r_[0.0, np.cumsum(ds)]


def resample_by_arc_length(data: np.ndarray, spacing: float = 0.25):
    if spacing <= 0:
        raise ValueError("spacing must be positive")
    s = cumulative_arc_length(data[:, 0], data[:, 1])
    good = np.r_[True, np.diff(s) > 1e-12]
    data, s = data[good], s[good]
    s_new = np.arange(0.0, s[-1], spacing)
    if len(s_new) == 0 or s_new[-1] < s[-1]:
        s_new = np.r_[s_new, s[-1]]
    out = np.column_stack([np.interp(s_new, s, data[:, j]) for j in range(5)])
    return s_new, out


def geometric_yaw(x: np.ndarray, y: np.ndarray, s: np.ndarray) -> np.ndarray:
    edge_order = 2 if len(x) >= 3 else 1
    dx = np.gradient(x, s, edge_order=edge_order)
    dy = np.gradient(y, s, edge_order=edge_order)
    return np.unwrap(np.arctan2(dy, dx))


def curvature_from_xy(x: np.ndarray, y: np.ndarray, s: np.ndarray) -> np.ndarray:
    edge_order = 2 if len(x) >= 3 else 1
    dx = np.gradient(x, s, edge_order=edge_order)
    dy = np.gradient(y, s, edge_order=edge_order)
    ddx = np.gradient(dx, s, edge_order=edge_order)
    ddy = np.gradient(dy, s, edge_order=edge_order)
    denom = np.power(dx * dx + dy * dy, 1.5)
    return (dx * ddy - dy * ddx) / np.maximum(denom, 1e-9)


def limit_acceleration(v: np.ndarray, s: np.ndarray, accel: float, decel: float) -> np.ndarray:
    """Forward/backward pass enforcing v^2 relation over distance."""
    out = np.maximum(v.astype(float).copy(), 0.0)
    for i in range(1, len(out)):
        ds = max(s[i] - s[i-1], 0.0)
        out[i] = min(out[i], np.sqrt(max(out[i-1]**2 + 2.0 * accel * ds, 0.0)))
    for i in range(len(out)-2, -1, -1):
        ds = max(s[i+1] - s[i], 0.0)
        out[i] = min(out[i], np.sqrt(max(out[i+1]**2 + 2.0 * decel * ds, 0.0)))
    return out


def process(path: str | Path, min_distance=0.02, spacing=0.25,
            max_lateral_accel=2.0, speed_limit=5.5556,
            max_accel=1.5, max_decel=2.0, smooth_window=5):
    raw = load_waypoints(path)
    filtered = remove_close_neighbors(raw, min_distance)
    s, sampled = resample_by_arc_length(filtered, spacing)
    x, y = sampled[:, 0], sampled[:, 1]
    yaw = geometric_yaw(x, y, s)
    curvature = curvature_from_xy(x, y, s)
    if smooth_window > 1:
        w = int(smooth_window)
        if w % 2 == 0: w += 1
        kernel = np.ones(w) / w
        curvature = np.convolve(curvature, kernel, mode="same")
    csv_speed = np.maximum(sampled[:, 4], 0.0)
    curve_speed = np.sqrt(max_lateral_accel / (np.abs(curvature) + 1e-3))
    reference_speed = np.minimum.reduce([
        csv_speed, curve_speed, np.full_like(csv_speed, speed_limit)
    ])
    reference_speed = limit_acceleration(reference_speed, s, max_accel, max_decel)
    profile = np.column_stack([s, x, y, yaw, curvature, csv_speed, curve_speed, reference_speed])
    stats = {
        "raw_points": len(raw), "filtered_points": len(filtered),
        "output_points": len(profile), "path_length_m": float(s[-1]),
        "max_csv_speed_mps": float(csv_speed.max()),
        "max_reference_speed_mps": float(reference_speed.max()),
        "max_abs_curvature_1pm": float(np.abs(curvature).max())
    }
    return profile, stats


def save_profile(profile: np.ndarray, path: str | Path):
    header = "s,x,y,yaw,curvature,csv_speed,curve_speed,reference_speed"
    np.savetxt(path, profile, delimiter=",", header=header, comments="", fmt="%.9f")


def main():
    p = argparse.ArgumentParser(description="Process wps.csv for MPC reference path/profile")
    p.add_argument("input_csv")
    p.add_argument("-o", "--output", default="wps_processed.csv")
    p.add_argument("--min-distance", type=float, default=0.02)
    p.add_argument("--spacing", type=float, default=0.25)
    p.add_argument("--max-lateral-accel", type=float, default=2.0)
    p.add_argument("--speed-limit", type=float, default=5.5556)
    p.add_argument("--max-accel", type=float, default=1.5)
    p.add_argument("--max-decel", type=float, default=2.0)
    p.add_argument("--smooth-window", type=int, default=5)
    args = p.parse_args()
    profile, stats = process(args.input_csv, args.min_distance, args.spacing,
                             args.max_lateral_accel, args.speed_limit,
                             args.max_accel, args.max_decel, args.smooth_window)
    save_profile(profile, args.output)
    print("Processing complete")
    for key, value in stats.items(): print(f"  {key}: {value}")
    print(f"  output: {Path(args.output).resolve()}")

if __name__ == "__main__":
    main()
