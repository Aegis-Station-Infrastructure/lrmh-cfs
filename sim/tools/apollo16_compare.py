#!/usr/bin/env python3
# Copyright 2026 Aegis Station Infrastructure LLC
# SPDX-License-Identifier: Apache-2.0
"""Lay a regolith preset against the Apollo 16 self-recording penetrometer records.

    sim/tools/apollo16_compare.py [--probe build/sim/regolith_probe] [--cache ~/.cache/lrmh/apollo16] [presets...]

Fetches the ten digitized LSRP records from the PDS Geosciences Node (once, into the cache),
takes each record's resistance envelope (the largest load seen at each depth, made monotone, over
the cone's area), and prints min / median / max resistance at a few depths beside the same for
the preset drawn from twelve seeds with the Apollo 0.2 in^2 (1.29 cm^2) cone.

The Apollo values flatten toward 1,650 kPa: the instrument's force limit was 215 N. Fit at 10
and 15 cm; at depth the records are the penetrometer's limit, not the soil's.
"""
import argparse
import os
import re
import statistics
import subprocess
import sys
import urllib.request

PDS = "https://pds-geosciences.wustl.edu/Lunar/urn-nasa-pds-a16lsrp_soil_mechanics/data/a16_smss_72_m016936_"
RECORDS = ["017", "019", "021", "022_rec", "024", "026", "028", "030", "031", "032"]
AREA_CM2 = {"0.2": 1.2903, "0.5": 3.2258}
DEPTHS = [5, 10, 15, 20, 30]


def fetch(cache):
    os.makedirs(cache, exist_ok=True)
    for r in RECORDS:
        for ext in ("tab", "txt"):
            path = os.path.join(cache, f"{r}.{ext}")
            if not os.path.exists(path):
                urllib.request.urlretrieve(f"{PDS}{r}.{ext}", path)


def envelope(cache, r):
    """(cone area cm2, [(depth cm, load N)] monotone) for one record."""
    txt = open(os.path.join(cache, f"{r}.txt"), errors="replace").read()
    m = re.search(r"([\d.]+) in\*\*2 cone", txt)
    area = AREA_CM2[m.group(1)] if m else 3.2258   # 0.5 in^2, the larger cone
    best = {}
    for line in open(os.path.join(cache, f"{r}.tab"), errors="replace"):
        f = line.split()
        if len(f) < 9:
            continue
        try:
            load, depth = float(f[4]), float(f[8])     # deg and min are two tokens; load is col 4, depth col 8
        except ValueError:
            continue
        best[depth] = max(best.get(depth, 0.0), load)
    run, mono = 0.0, []
    for z, F in sorted(best.items()):
        run = max(run, F)
        mono.append((z, run))
    return area, mono


def apollo_columns(cache, area_cm2):
    cols = {z: [] for z in DEPTHS}
    for r in RECORDS:
        area, mono = envelope(cache, r)
        if abs(area - area_cm2) > 0.01:
            continue
        for z in DEPTHS:
            below = [F for zz, F in mono if zz <= z]
            if below:
                cols[z].append(below[-1] / area * 10)          # N/cm2 -> kPa
    return cols


def probe_columns(probe, preset, seed, area_cm2):
    out = subprocess.run([probe, preset, str(seed), str(area_cm2), "35", "0.5"], capture_output=True, text=True, check=True).stdout
    q = {float(l.split()[0]): float(l.split()[1]) for l in out.splitlines() if l and not l.startswith("#")}
    return [q[float(z)] for z in DEPTHS]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--probe", default=os.path.join(os.path.dirname(__file__), "..", "..", "build", "sim", "regolith_probe"))
    ap.add_argument("--cache", default=os.path.expanduser("~/.cache/lrmh/apollo16"))
    ap.add_argument("--seeds", type=int, default=12)
    ap.add_argument("presets", nargs="*", default=["lunar_highland", "lunar_mare", "lunar_loose"])
    a = ap.parse_args()
    fetch(a.cache)
    head = "                 " + " ".join(f"{z:>5} cm" for z in DEPTHS)
    cols = apollo_columns(a.cache, 1.29)
    print("Apollo 16, 1.29 cm2 cone, 9 records (kPa; min / median / max)")
    print(head)
    for lab, f in (("min", min), ("median", statistics.median), ("max", max)):
        print(f"  {lab:14s}" + " ".join(f"{f(cols[z]):8.0f}" for z in DEPTHS))
    for preset in a.presets:
        rows = [probe_columns(a.probe, preset, s, 1.29) for s in range(1, a.seeds + 1)]
        print(f"{preset}, {a.seeds} seeds")
        for lab, f in (("min", min), ("median", statistics.median), ("max", max)):
            print(f"  {lab:14s}" + " ".join(f"{f(c):8.0f}" for c in zip(*rows)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
