import math
import unittest

from tools.walk import (
    DEFAULT_ZERO,
    GaitParams,
    SERVO_UNITS_PER_DEG,
    STAND_ANGLES,
    build_forward_frame_plan,
    generate_frame,
    ik_to_servo,
    plan_forward_phase,
)


class TestWalkForwardPipeline(unittest.TestCase):
    def test_phase_plan_alternates_swing_leg(self):
        params = GaitParams()
        p1 = plan_forward_phase(0.1, params)
        p2 = plan_forward_phase(params.period * 0.6, params)
        self.assertEqual(p1.swing_leg, "right")
        self.assertEqual(p2.swing_leg, "left")

    def test_forward_progression_not_reversed(self):
        params = GaitParams()
        t_start = params.pre_shift_ratio * 0.5 * params.period + 0.01
        t_end = 0.5 * params.period - 0.01
        plan_start = build_forward_frame_plan(t_start, params)
        plan_end = build_forward_frame_plan(t_end, params)
        self.assertGreater(plan_end.right.x, plan_start.right.x)
        self.assertLess(plan_end.left.x, plan_start.left.x)

    def test_only_one_leg_lifts_in_half_cycle(self):
        params = GaitParams()
        t = params.period * 0.35
        plan = build_forward_frame_plan(t, params)
        left_h = plan.left.h
        right_h = plan.right.h
        self.assertNotEqual(left_h > right_h, right_h > left_h)

    def test_generate_frame_output_contract(self):
        params = GaitParams()
        frame = generate_frame(0.2, DEFAULT_ZERO.copy(), params)
        expected = set(DEFAULT_ZERO.keys())
        self.assertEqual(set(frame.keys()), expected)
        self.assertTrue(all(isinstance(v, int) for v in frame.values()))

    def test_ankle_pitch_limit_enforced(self):
        zero = DEFAULT_ZERO.copy()
        angles = list(STAND_ANGLES)
        angles[3] = STAND_ANGLES[3] + math.radians(90)
        result = ik_to_servo(tuple(angles), "left", zero, ankle_pitch_limit_deg=45.0)
        delta = abs(result["left_ankle_pitch"] - zero["left_ankle_pitch"])
        self.assertLessEqual(delta, int(math.ceil(45.0 * SERVO_UNITS_PER_DEG)))


if __name__ == "__main__":
    unittest.main()
