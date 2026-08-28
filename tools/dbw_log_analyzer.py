from __future__ import annotations
import csv
import sys
from pathlib import Path
import matplotlib.pyplot as plt
def load_dbw_csv(path:Path)->list[dict[str, str]]:
    with path.open("r", newline="", encoding="utf-8-sig") as f:
        lines=f.readlines()
    header_index=None
    for i, line in enumerate(lines):
        if line.startswith("time_ms,"):
            header_index=i
            break
    if header_index is None:
        raise ValueError("Could not find DBW CSV header")
    return list(csv.DictReader(lines[header_index:]))
def to_float(row:dict[str, str], key:str)->float|None:
    value=row.get(key, "")
    if value in(None, ""):
        return None
    try:
        return float(value)
    except ValueError:
        return None
def extract_series(
    rows:list[dict[str, str]],
    key:str,
    scale:float=1.0,
)->tuple[list[float], list[float]]:
    times:list[float]=[]
    values:list[float]=[]
    for row in rows:
        time_ms=to_float(row, "time_ms")
        value=to_float(row, key)
        if time_ms is None or value is None:
            continue
        times.append(time_ms/1000.0)
        values.append(value*scale)
    return times, values
def analyze(rows:list[dict[str, str]])->list[str]:
    warnings:list[str]=[]
    measured=[
        value
        for row in rows
        if(value:=to_float(row, "measured_speed")) is not None
    ]
    if measured and max(measured)==min(measured):
        warnings.append(
            f"measured_speed is constant for the entire log "
            f"({measured[0]:g})"
        )
    neutral_nonzero=0;
    throttle_while_braking=0
    for row in rows:
        mapped_throttle=to_float(row, "MapThB")
        desired_speed=to_float(row, "desired_speed_cmPs")
        throttle_pwm=to_float(row, "throttle_pwm")
        brake_on=to_float(row, "BrakeOn")
        if(
            mapped_throttle is not None
            and desired_speed is not None
            and abs(mapped_throttle)<=5
            and desired_speed>5
        ):
            neutral_nonzero+=1
        if(
            throttle_pwm is not None
            and brake_on is not None
            and throttle_pwm>0
            and brake_on!=0
        ):
            throttle_while_braking+=1
    if neutral_nonzero:
        warnings.append(
            f"{neutral_nonzero} samples have near-neutral MapThB "
            "but desired_speed_cmPs > 5"
        )
    if throttle_while_braking:
        warnings.append(
            f"{throttle_while_braking} samples have throttle PWM > 0 "
            "while BrakeOn is active"
        )
    return warnings
def plot_series(
    rows:list[dict[str, str]],
    specifications:list[tuple[str, float, str]],
    output:Path,
    title:str,
    ylabel:str,
)->bool:
    plt.figure()
    plotted=False
    for key, scale, label in specifications:
        times, values=extract_series(rows, key, scale)
        if not times:
            continue
        plt.plot(times, values, label=label)
        plotted=True
    if not plotted:
        plt.close()
        return False
    plt.xlabel("Time (s)")
    plt.ylabel(ylabel)
    plt.title(title)
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(output)
    plt.close()
    return True
def max_value(rows:list[dict[str, str]], key:str)->float|None:
    values=[
        value
        for row in rows
        if(value:=to_float(row, key)) is not None
    ]
    return max(values) if values else None
def min_value(rows:list[dict[str, str]], key:str)->float|None:
    values=[
        value
        for row in rows
        if(value:=to_float(row, key)) is not None
    ]
    return min(values) if values else None
def count_transitions(rows:list[dict[str, str]], key:str)->int:
    values=[
        value
        for row in rows
        if(value:=to_float(row, key)) is not None
    ]
    if len(values)<2:
        return 0
    return sum(
        current!=previous
        for previous, current in zip(values, values[1:])
    )
def build_summary(rows:list[dict[str, str]])->str:
    duration=0.0
    times=[
        value
        for row in rows
        if(value:=to_float(row, "time_ms")) is not None
    ]
    if times:
        duration=(max(times)-min(times))/1000.0
    max_desired_speed=max_value(rows, "desired_speed_cmPs")
    max_angle=max_value(rows, "desired_angle_DegX10")
    min_angle=min_value(rows, "desired_angle_DegX10")
    max_current_angle=max_value(rows, "current_angle")
    min_current_angle=min_value(rows, "current_angle")
    estop_values={
        value
        for row in rows
        if(value:=to_float(row, "op_estop")) is not None
    }
    lines=[
        f"Samples: {len(rows)}",
        f"Duration: {duration:.2f} s",
        (
            f"Maximum desired speed: {max_desired_speed/100.0:.2f} m/s"
            if max_desired_speed is not None
            else "Maximum desired speed: unavailable"
        ),
        (
            f"Desired steering range: "
            f"{min_angle/10.0:.1f} to {max_angle/10.0:.1f} degrees"
            if min_angle is not None and max_angle is not None
            else "Desired steering range: unavailable"
        ),
        (
            f"Current steering range: "
            f"{min_current_angle/10.0:.1f} to "
            f"{max_current_angle/10.0:.1f} degrees"
            if min_current_angle is not None and max_current_angle is not None
            else "Current steering range: unavailable"                
        ),
        (
            "Operator E-stop signal values: "
            +", ".join(f"{value:g}" for value in sorted(estop_values))
            if estop_values
            else "Operator E-stop signal values: unavailable"
        ),
        f"Operator E-stop signal transitions: {count_transitions(rows, 'op_estop')}",
        f"Drive mode transitions: {count_transitions(rows, 'driveMode')}",
        f"Operator mode transitions: {count_transitions(rows, 'op_mode')}",
    ]
    return "\n".join(lines)+"\n"
def generate_report(csv_path:Path, output_dir:Path)->list[Path]:
    rows=load_dbw_csv(csv_path)
    output_dir.mkdir(parents=True, exist_ok=True)
    for old_file in output_dir.glob("*.png"):
        old_file.unlink()
    for old_file in output_dir.glob("*.txt"):
        old_file.unlink()
    generated:list[Path]=[]
    plots=[
        (
            [
                ("desired_angle_DegX10", 0.1, "Desired angle"),
                ("current_angle", 0.1, "Current angle"),
            ],
            "steering.png",
            "Steering Response",
            "Steering angle (degrees)",
        ),
        (
            [
                ("MapStr", 1.0, "Mapped steering input"),
                ("MapThB", 1.0, "Mapped throttle/brake input"),
            ],
            "control_inputs.png",
            "Mapped Control Inputs",
            "Mapped command",
        ),
        (
            [
                ("desired_speed_cmPs", 0.01, "Desired speed"),
            ],
            "desired_speed.png",
            "Desired Speed",
            "Speed (m/s)",
        ),
        (
            [
                ("throttle_pwm", 1.0, "Throttle PWM"),
            ],
            "throttle_pwm.png",
            "Throttle Output",
            "PWM value",
        ),
        (
            [
                ("BrakeOn", 1.0, "BrakeOn"),
                ("op_estop", 1.0, "Operator E-stop"),
                ("op_mode", 1.0, "Operator mode"),
                ("driveMode", 1.0, "Drive mode"),
            ],
            "safety_modes.png",
            "Safety and Mode States",
            "State"
        ),
    ]
    for specs, filename, title, ylabel in plots:
        output=output_dir/filename
        if plot_series(rows, specs, output, title, ylabel):
            generated.append(output)
    warnings_path=output_dir/"warnings.txt"
    warnings=analyze(rows)
    warnings_path.write_text(
        "\n".join(warnings)+"\n"
        if warnings
        else "No warnings detected.\n",
        encoding="utf-8"
    )
    generated.append(warnings_path)
    summary_path=output_dir/"summary.txt"
    summary_path.write_text(
        build_summary(rows),
        encoding="utf-8",
    )
    generated.append(summary_path)
    return generated
def main()->int:
    if len(sys.argv)!=2:
        print(f"Usage: {sys.argv[0]} <dbw_log.csv>")
        return 2
    csv_path=Path(sys.argv[1])
    if not csv_path.exists():
        print(f"File not found: {csv_path}")
        return 1
    output_dir=Path("dbw_report")/csv_path.stem
    generated=generate_report(csv_path, output_dir)
    print(f"Generated report in {output_dir}")
    for path in generated:
        print(f"  {path}")
    return 0
if __name__=="__main__":
    raise SystemExit(main())