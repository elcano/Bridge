import tempfile
import unittest
from pathlib import Path
from tools.dbw_log_analyzer import analyze, generate_report, load_dbw_csv
CSV_TEXT="""metadata,,,,test
time_ms,MapStr,MapThB,desired_speed_cmPs,desired_angle_DegX10,current_angle,throttle_pwm,measured_speed,BrakeOn,op_estop,op_mode,driveMode
0,0,0,0,0,0,0,0,1,0,1,1
100,50,0,0,50,20,0,0,1,0,1,1
200,100,2,20,100,40,20,0,0,0,1,1
300,0,0,0,0,20,0,0,1,1,0,0
"""
class TestDbwLogAnalyzer(unittest.TestCase):
    def make_csv(self, directory:Path)->Path:
        path=directory/"sample.csv"
        path.write_text(CSV_TEXT, encoding="utf-8")
        return path
    def test_load_dbw_csv_skips_metadata_line(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=self.make_csv(Path(tmp))
            rows=load_dbw_csv(path)
            self.assertEqual(len(rows), 4)
            self.assertEqual(rows[1]["MapStr"], "50")
    def test_analyze_detects_constant_measured_speed(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=self.make_csv(Path(tmp))
            warnings=analyze(load_dbw_csv(path))
            self.assertTrue(
                any("measured_speed is constant" in warning for warning in warnings)
            )
    def test_analyze_detects_neutral_speed_issue(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=self.make_csv(Path(tmp))
            warnings=analyze(load_dbw_csv(path))
            self.assertTrue(
                any("near-neutral MapThB" in warning for warning in warnings)
            )
    def test_generate_report_creates_output(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp_path=Path(tmp)
            csv_path=self.make_csv(tmp_path)
            output=tmp_path/"report"
            generated=generate_report(csv_path, output)
            names={path.name for path in generated}
            self.assertIn("steering.png", names)
            self.assertIn("control_inputs.png", names)
            self.assertIn("desired_speed.png", names)
            self.assertIn("throttle_pwm.png", names)
            self.assertIn("safety_modes.png", names)
            self.assertIn("warnings.txt", names)
            self.assertIn("summary.txt", names)
if __name__=="__main__":
    unittest.main()