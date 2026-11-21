"""Reuse baseline native MoveIt launch with experiment-local robot artifact."""
from pathlib import Path
import importlib.util
ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('cup_planning',ROOT.parents[1]/'launch/planning.launch.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
module.ROOT=ROOT
generate_launch_description=module.generate_launch_description
