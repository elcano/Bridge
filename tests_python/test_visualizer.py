#!/usr/bin/env python3
from __future__ import annotations
import sys
import tempfile
import unittest
from pathlib import Path
import matplotlib
# Use a non-interactive backend so tests work without opening windows.
matplotlib.use("Agg")
REPOSITORY_ROOT=Path(__file__).resolve().parents[1]
TOOLS_DIRECTORY=REPOSITORY_ROOT/"tools"
sys.path.insert(0, str(TOOLS_DIRECTORY))
from test_visualizer import extract_series, generate_report # noqa: E402
class TestVisualizer(unittest.TestCase):
	def setUp(self)->None:
		self.log=[
			(
				0,
				{
					"east_cm":0,
					"north_cm":0,
					"heading_centiDeg":0,
					"actual_angle_tenths":0,
					"router_angle_tenths":0,
					"cmd_speed_cmPs":1500,
					"cmd_angle_tenths":0,
				},
			),
			(
				1000,
				{
					"east_cm":10,
					"north_cm":100,
					"heading_centiDeg":500,
					"actual_angle_tenths":80,
					"router_angle_tenths":100,
					"cmd_speed_cmPs":1500,
					"cmd_angle_tenths":250,
				},
			),
			(
				2000,
				{
					"east_cm":40,
					"north_cm":200,
					"heading_centiDeg":1500,
					"actual_angle_tenths":210,
					"router_angle_tenths":220,
					"cmd_speed_cmPs":1500,
					"cmd_angle_tenths":250,
				},
			),
		]
	def test_extract_series_scales_values(self)->None:
		times, values=extract_series(
			self.log,
			"heading_centiDeg",
			scale=0.01
		)
		self.assertEqual(times, [0.0, 1.0, 2.0])
		self.assertEqual(values, [0.0, 5.0, 15.0])
	def test_extract_series_skips_missing_fields(self)->None:
		incomplete_log=[
			(0, {"cmd_speed_cmPs":100}),
			(1000, {}),
			(2000, {"cmd_speed_cmPs":300}),
		]
		times, values=extract_series(
			incomplete_log,
			"cmd_speed_cmPs",
			scale=0.01,
		)
		self.assertEqual(times, [0.0, 2.0])
		self.assertEqual(values, [1.0, 3.0])
	def test_generate_report_creates_all_plots(self)->None:
		with tempfile.TemporaryDirectory() as temporary_directory:
			output_directory=Path(temporary_directory)
			generated=generate_report(self.log, output_directory)
			generated_names={path.name for path in generated}
			self.assertEqual(
				generated_names,
				{
					"trajectory.png",
					"steering_response.png",
					"heading.png",
					"commanded_speed.png",
				},
			)
			for path in generated:
				self.assertTrue(path.exists())
				self.assertGreater(path.stat().st_size, 0)
	def test_generate_report_handles_empty_log(self)->None:
		with tempfile.TemporaryDirectory() as temporary_directory:
			generated=generate_report([], temporary_directory)
			self.assertEqual(generated, [])
if __name__=="__main__":
	unittest.main()