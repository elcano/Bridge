#!/usr/bin/env python3
"""
Post-test visualization for Elcano closed-loop simulation telemetry.

This module consumes the existing test_runner log structure:

	list[tuple[int, dict[str, int]]]

Each tuple contains PC-side elapsed time in milliseconds and the telemetry
fields parsed from one LOG line. The module does not depend on which board
forwarded the telemetry, so it can support both the current Sensor Hub
gateway and a future Router-based test gateway.
"""
from __future__ import annotations
from pathlib import Path
from typing import TypeAlias
import matplotlib.pyplot as plt
LogSample:TypeAlias=tuple[int, dict[str, int]]
TestLog:TypeAlias=list[LogSample]
def extract_series(
	log:TestLog,
	field:str,
	scale:float=1.0,
)->tuple[list[float], list[float]]:
	"""Extract elapsed seconds and a scaled telemetry field."""
	times:list[float]=[]
	values:list[float]=[]
	for elapsed_ms, fields in log:
		if field not in fields:
			continue
		times.append(elapsed_ms/1000.0)
		values.append(fields[field]*scale)
	return times, values
def generate_trajectory_plot(
	log:TestLog,
	output_directory:Path,
)->Path|None:
	"""Plot east/north vehicle position in metres."""
	east_m:list[float]=[]
	north_m:list[float]=[]
	for _, fields in log:
		if "east_cm" not in fields or "north_cm" not in fields:
			continue
		east_m.append(fields["east_cm"]/100.0)
		north_m.append(fields["north_cm"]/100.0)
	if not east_m:
		return None
	figure, axes=plt.subplots(figsize=(8, 7))
	axes.plot(east_m, north_m, label="Vehicle path")
	axes.scatter(east_m[0], north_m[0], marker="o", label="Start")
	axes.scatter(east_m[-1], north_m[-1], marker="x", label="End")
	axes.set_title("Closed-Loop Vehicle Trajectory")
	axes.set_xlabel("East position (m)")
	axes.set_ylabel("North position (m)")
	axes.axis("equal")
	axes.grid(True)
	axes.legend()
	figure.tight_layout()
	output_path=output_directory/"trajectory.png"
	figure.savefig(output_path, dpi=160)
	plt.close(figure)
	return output_path
def generate_steering_plot(
	log:TestLog,
	output_directory:Path,
)->Path|None:
	"""Compare commanded, DBW-reported, and Router steering angles."""
	command_time, command_angle=extract_series(
		log,
		"cmd_angle_tenths",
		scale=0.1,
	)
	actual_time, actual_angle=extract_series(
		log,
		"actual_angle_tenths",
		scale=0.1,
	)
	router_time, router_angle=extract_series(
		log,
		"router_angle_tenths",
		scale=0.1,
	)
	if not command_time and not actual_time and not router_time:
		return None
	figure, axes=plt.subplots(figsize=(10, 6))
	if command_time:
		axes.plot(command_time, command_angle, label="Commanded")
	if actual_time:
		axes.plot(actual_time, actual_angle, label="DBW actual")
	if router_time:
		axes.plot(router_time, router_angle, label="Router simulated")
	axes.set_title("Closed-Loop Steering Response")
	axes.set_xlabel("Elapsed time (s)")
	axes.set_ylabel("Steering angle (degrees)")
	axes.grid(True)
	axes.legend()
	figure.tight_layout()
	output_path=output_directory/"steering_response.png"
	figure.savefig(output_path, dpi=160)
	plt.close(figure)
	return output_path
def generate_heading_plot(
	log:TestLog,
	output_directory:Path,
)->Path|None:
	"""Plot vehicle heading in degrees over time."""
	times, heading=extract_series(
		log,
		"heading_centiDeg",
		scale=0.01,
	)
	if not times:
		return None
	figure, axes=plt.subplots(figsize=(10, 5))
	axes.plot(times, heading, label="Vehicle heading")
	axes.set_title("Vehicle Heading")
	axes.set_xlabel("Elapsed time (s)")
	axes.set_ylabel("Heading (degrees)")
	axes.grid(True)
	axes.legend()
	figure.tight_layout()
	output_path=output_directory/"heading.png"
	figure.savefig(output_path, dpi=160)
	plt.close(figure)
	return output_path
def generate_speed_plot(
	log:TestLog,
	output_directory:Path,
)->Path|None:
	"""Plot commanded speed in meters per second over time."""
	times, speed=extract_series(
		log,
		"cmd_speed_cmPs",
		scale=0.01,
	)
	if not times:
		return None
	figure, axes=plt.subplots(figsize=(10, 5))
	axes.plot(times, speed, label="Commanded speed")
	axes.set_title("Commanded Speed")
	axes.set_xlabel("Elapsed time(s)")
	axes.set_ylabel("Speed (m/s)")
	axes.grid(True)
	axes.legend()
	figure.tight_layout()
	output_path=output_directory/"commanded_speed.png"
	figure.savefig(output_path, dpi=160)
	plt.close(figure)
	return output_path
def generate_report(
	log:TestLog,
	output_directory:str|Path,
)->list[Path]:
	"""Generate every plot supported by the available telemetry fields."""
	directory=Path(output_directory)
	directory.mkdir(parents=True, exist_ok=True)
	generated_files:list[Path]=[]
	generators=(
		generate_trajectory_plot,
		generate_steering_plot,
		generate_heading_plot,
		generate_speed_plot,
	)
	for generator in generators:
		output_path=generator(log, directory)
		if output_path is not None:
			generated_files.append(output_path)
	return generated_files