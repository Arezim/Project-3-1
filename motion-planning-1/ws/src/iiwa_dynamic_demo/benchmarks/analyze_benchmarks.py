#!/usr/bin/env python3
import csv
import statistics
import sys
import os


def analyze_csv(path: str):
    print("=" * 80)
    print(f"File: {path}")
    print("-" * 80)

    if not os.path.exists(path):
        print("  [ERROR] File does not exist.")
        return

    data = {}

    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        # Expect at least columns: run, planner, ok, time_s, ...
        for row in reader:
            pid = row.get("planner", "").strip()
            if not pid:
                continue
            data.setdefault(pid, []).append(row)

    if not data:
        print("  [WARN] No rows with a 'planner' column found.")
        return

    # Header line
    print(f"{'planner':30} {'n':>4} {'success[%]':>11} {'median_time[s]':>15}")
    print("-" * 80)

    # Sorted by planner name for stable output
    for pid in sorted(data.keys()):
        rows = data[pid]
        n = len(rows)

        # Success = ok != 0
        successes = [
            r for r in rows
            if str(r.get("ok", "0")).strip() not in ("0", "", "False", "false")
        ]
        success_rate = 100.0 * len(successes) / n if n > 0 else 0.0

        # Times only for successful runs
        times = []
        for r in successes:
            t_str = r.get("time_s", "").strip()
            if t_str:
                try:
                    times.append(float(t_str))
                except ValueError:
                    pass

        median_time = statistics.median(times) if times else None

        if median_time is not None:
            print(f"{pid:30} {n:4d} {success_rate:11.1f} {median_time:15.4f}")
        else:
            print(f"{pid:30} {n:4d} {success_rate:11.1f} {'N/A':>15}")

    print()  # blank line after each file


def main():
    # If no arguments: analyze all three default files in this folder
    if len(sys.argv) == 1:
        files = [
            "bench_manipulator.csv",
            "bench_static_manipulator.csv",
            "bench_dynamic_manipulator.csv",
        ]
    else:
        files = sys.argv[1:]

    for fname in files:
        path = os.path.abspath(fname)
        analyze_csv(path)


if __name__ == "__main__":
    main()

