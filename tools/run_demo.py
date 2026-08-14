#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
from dataclasses import asdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from swarm_autonomy.simulation import run_mission

parser = argparse.ArgumentParser(description="Multi-UAV autonomy core demonstration")
parser.add_argument("--steps", type=int, default=240)
args = parser.parse_args()
report = run_mission(args.steps)
output = ROOT / "artifacts" / "mission_summary.json"
output.parent.mkdir(exist_ok=True)
output.write_text(json.dumps(asdict(report), ensure_ascii=False, indent=2), encoding="utf-8")
print(json.dumps(asdict(report), ensure_ascii=False, indent=2))
raise SystemExit(0 if report.success else 1)
