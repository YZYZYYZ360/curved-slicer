#!/usr/bin/env python3
"""Compare baseline metrics directories.

TODO(Phase 5): add STL/PLY visual comparison.
TODO(Phase 5): add field-value comparison if a future phase reintroduces phi dumps.
"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Any

import numpy as np


def load_metrics(root: Path) -> dict[str, dict[str, Any]]:
    metrics: dict[str, dict[str, Any]] = {}
    if not root.exists():
        raise FileNotFoundError(f"metrics root does not exist: {root}")

    for metrics_path in sorted(root.glob("*/metrics.json")):
        with metrics_path.open("r", encoding="utf-8") as handle:
            data = json.load(handle)
        model = str(data.get("model") or metrics_path.parent.name)
        metrics[model] = data
    return metrics


def face_count(metrics: dict[str, Any]) -> int | None:
    m4 = metrics.get("M4", {})
    value = m4.get("face_count_total", m4.get("face_count"))
    return None if value is None else int(value)


def connected_components(metrics: dict[str, Any]) -> int | None:
    m4 = metrics.get("M4", {})
    value = m4.get("connected_components")
    if value is not None:
        return int(value)
    per_layer = m4.get("connected_components_per_layer")
    if per_layer:
        return int(np.max(np.asarray(per_layer, dtype=np.int64)))
    return None


def max_layer_connected_components(metrics: dict[str, Any]) -> int | None:
    m4 = metrics.get("M4", {})
    value = m4.get("max_connected_components_per_layer")
    if value is not None:
        return int(value)
    per_layer = m4.get("connected_components_per_layer")
    if per_layer:
        return int(np.max(np.asarray(per_layer, dtype=np.int64)))
    return None


def max_abs_mean_curvature(metrics: dict[str, Any]) -> float | None:
    m1 = metrics.get("M1", {})
    value = m1.get("max_abs_mean_curvature")
    return None if value is None else float(value)


def hemisphere_violation_ratio(metrics: dict[str, Any]) -> float | None:
    m2 = metrics.get("M2", {})
    value = m2.get("hemisphere_violation_ratio")
    return None if value is None else float(value)


def solver_ms(metrics: dict[str, Any]) -> float | None:
    m6 = metrics.get("M6", {})
    for key in ("solver_ms", "wavefront_ms", "poisson_ms", "field_ms"):
        value = m6.get(key)
        if value is not None:
            return float(value)
    return None


def format_value(value: int | float | None) -> str:
    if value is None:
        return ""
    if isinstance(value, float):
        return f"{value:.3f}"
    return str(value)


def build_rows(old_metrics: dict[str, dict[str, Any]], new_metrics: dict[str, dict[str, Any]]) -> list[dict[str, str]]:
    rows: list[dict[str, str]] = []
    for model in sorted(set(old_metrics) | set(new_metrics)):
        old = old_metrics.get(model, {})
        new = new_metrics.get(model, {})
        rows.append(
            {
                "model": model,
                "M4_face_count_old": format_value(face_count(old)),
                "M4_face_count_new": format_value(face_count(new)),
                "M4_connected_components_old": format_value(connected_components(old)),
                "M4_connected_components_new": format_value(connected_components(new)),
                "M4_max_layer_cc_old": format_value(max_layer_connected_components(old)),
                "M4_max_layer_cc_new": format_value(max_layer_connected_components(new)),
                "M1_max_abs_mean_curvature_old": format_value(max_abs_mean_curvature(old)),
                "M1_max_abs_mean_curvature_new": format_value(max_abs_mean_curvature(new)),
                "M2_hemisphere_violation_old": format_value(hemisphere_violation_ratio(old)),
                "M2_hemisphere_violation_new": format_value(hemisphere_violation_ratio(new)),
                "M6_solver_ms_old": format_value(solver_ms(old)),
                "M6_solver_ms_new": format_value(solver_ms(new)),
            }
        )
    return rows


def print_table(rows: list[dict[str, str]]) -> None:
    if not rows:
        print("No metrics.json files found in either directory.")
        return

    headers = list(rows[0].keys())
    widths = {header: len(header) for header in headers}
    for row in rows:
        for header in headers:
            widths[header] = max(widths[header], len(row[header]))

    print(" | ".join(header.ljust(widths[header]) for header in headers))
    print("-+-".join("-" * widths[header] for header in headers))
    for row in rows:
        print(" | ".join(row[header].ljust(widths[header]) for header in headers))


def write_csvs(rows: list[dict[str, str]], output_root: Path) -> None:
    output_root.mkdir(parents=True, exist_ok=True)
    if not rows:
        return

    headers = list(rows[0].keys())
    for row in rows:
        output_path = output_root / f"{row['model']}_table.csv"
        with output_path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=headers)
            writer.writeheader()
            writer.writerow(row)


def main() -> int:
    parser = argparse.ArgumentParser(description="Compare old and new baseline metrics.json directories.")
    parser.add_argument("baseline_dir", type=Path)
    parser.add_argument("new_dir", type=Path)
    parser.add_argument("--output-dir", type=Path, default=Path("runs") / "comparison")
    args = parser.parse_args()

    old_metrics = load_metrics(args.baseline_dir)
    new_metrics = load_metrics(args.new_dir)
    rows = build_rows(old_metrics, new_metrics)
    print_table(rows)
    write_csvs(rows, args.output_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
