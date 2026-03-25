import math
import os
import unittest

from tools.walk_high_step import (
    DEFAULT_REFERENCE_MOTION,
    HighStepConfig,
    build_step_plan,
    build_target_frame,
    load_reference_zero,
    simulate_cycles,
)
from tools.walk import SERVO_UNITS_PER_DEG


class TestHighStepWalk(unittest.TestCase):
    def test_reference_zero_uses_first_and_last_frame_average(self):
        zero = load_reference_zero(DEFAULT_REFERENCE_MOTION)
        self.assertEqual(zero["left_hip_pitch"], 570)
        self.assertEqual(zero["right_hip_pitch"], 764)
        self.assertEqual(zero["left_ankle_roll"], 635)
        self.assertEqual(zero["right_ankle_roll"], 562)

    def test_state_machine_never_double_airborne_for_100_cycles(self):
        config = HighStepConfig()
        config.validate()
        zero = load_reference_zero(DEFAULT_REFERENCE_MOTION)
        report = simulate_cycles(100, config, zero)
        self.assertEqual(report.double_airborne_errors, 0)

    def test_stability_metrics_stay_within_requested_bounds(self):
        config = HighStepConfig()
        zero = load_reference_zero(DEFAULT_REFERENCE_MOTION)
        report = simulate_cycles(20, config, zero)
        self.assertLessEqual(report.max_com_y_mm, config.max_com_lateral_mm)
        self.assertLessEqual(report.max_com_x_mm, config.max_com_forward_mm)
        self.assertLessEqual(report.max_cycle_com_y_mm, config.max_com_cycle_lateral_mm)
        self.assertLessEqual(report.max_cycle_com_x_mm, config.max_com_cycle_forward_mm)
        self.assertGreaterEqual(report.min_support_ratio, 0.8)
        self.assertLessEqual(report.max_pose_angle_deg, 2.0)
        self.assertLessEqual(report.max_landing_force_ratio, 1.2)
        self.assertLessEqual(report.max_landing_db, 45.0)

    def test_swing_knee_exceeds_90_deg(self):
        config = HighStepConfig()
        zero = load_reference_zero(DEFAULT_REFERENCE_MOTION)
        t = config.transfer_s + config.swing_raise_s * 0.9
        plan = build_step_plan(t, config)
        _, angles = build_target_frame(plan, zero, config)
        swing_knee_deg = math.degrees(angles[plan.swing_leg][2])
        self.assertGreaterEqual(swing_knee_deg, 90.0)

    def test_single_support_leg_always_grounded(self):
        config = HighStepConfig()
        sample_times = [i * config.dt_s for i in range(int(config.period_s / config.dt_s) * 4)]
        for t in sample_times:
            plan = build_step_plan(t, config)
            support_pose = plan.left if plan.support_leg == "left" else plan.right
            self.assertTrue(support_pose.contact)

    def test_servo_delta_never_exceeds_30_deg(self):
        config = HighStepConfig()
        zero = load_reference_zero(DEFAULT_REFERENCE_MOTION)
        report = simulate_cycles(10, config, zero)
        self.assertLessEqual(report.max_servo_delta_deg, config.servo_limit_deg)
        t = config.transfer_s + config.swing_advance_s
        plan = build_step_plan(t, config)
        frame, _ = build_target_frame(plan, zero, config)
        limit_units = config.servo_limit_deg * SERVO_UNITS_PER_DEG
        for name, value in frame.items():
            self.assertLessEqual(abs(value - zero[name]), limit_units + 1)


if __name__ == "__main__":
    unittest.main()
