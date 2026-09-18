#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""YOLOv8n-pose 后处理:人的低分框保留(供 ByteTrack 式关联)。"""
import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "radar_system"))

try:
    import numpy as np
except ImportError:
    np = None

if np is not None:
    from pose_inference import postprocess, PERSON_LOW_CONFIDENCE


def make_pose_outputs():
    return [np.full((1, 65, n, n), -20, dtype=np.float32) for n in (80, 40, 20)] + [
        np.zeros((1, 17, 3, 8400), dtype=np.float32)
    ]


def add_person(out, branch=0, row=30, col=30, prob=0.9, point=(240., 240., .9)):
    logit = float(np.log(prob / (1 - prob))) if 0 < prob < 1 else (20. if prob >= 1 else -20.)
    h = out[branch]
    h[0, :64, row, col] = -20
    for side in range(4):
        h[0, side * 16 + 2, row, col] = 20
    h[0, 64, row, col] = logit
    index = sum(n * n for n in (80, 40, 20)[:branch]) + row * h.shape[-1] + col
    out[3][0, :, :, index] = point


TRANSFORM = (1.0, 0, 0, 640, 640)


@unittest.skipIf(np is None, "需要 numpy")
class TestPosePersonLowScore(unittest.TestCase):

    def test_low_confidence_floor_is_015(self):
        self.assertEqual(PERSON_LOW_CONFIDENCE, 0.15)

    def test_low_score_person_is_kept(self):
        out = make_pose_outputs()
        add_person(out, branch=0, row=30, col=30, prob=0.20)
        res = postprocess(out, TRANSFORM)
        self.assertEqual(len(res), 1)
        self.assertEqual(res[0]['label'], 'person')
        self.assertAlmostEqual(res[0]['conf'], 0.20, delta=0.03)

    def test_below_person_floor_is_dropped(self):
        out = make_pose_outputs()
        add_person(out, branch=0, row=30, col=30, prob=0.10)
        res = postprocess(out, TRANSFORM)
        self.assertEqual(len(res), 0)

    def test_high_score_person_is_kept(self):
        out = make_pose_outputs()
        add_person(out, branch=0, row=30, col=30, prob=0.90)
        res = postprocess(out, TRANSFORM)
        self.assertEqual(len(res), 1)
        self.assertEqual(res[0]['label'], 'person')
        self.assertGreaterEqual(res[0]['conf'], 0.85)


if __name__ == '__main__':
    unittest.main()
