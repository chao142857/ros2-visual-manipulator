"""Plane discovery peeling must not remove evidence from support scoring."""

from dataclasses import replace

import numpy as np
import pytest

from tabletop_grasp import geometry
from tabletop_grasp.config import PerceptionConfig


def overlapping_planes():
    x, y = np.meshgrid(np.linspace(-.03, .03, 41), np.linspace(-.009, .009, 19))
    top = np.column_stack((x.ravel(), y.ravel(), np.full(x.size, .03)))
    x, y = np.meshgrid(np.linspace(-.03, .03, 41), np.linspace(.010, .018, 5))
    side = np.column_stack((x.ravel(), y.ravel(), .03 - np.tan(np.radians(15)) * y.ravel()))
    return np.vstack((top, side)), len(top)


def tilted_plane(angle_deg, height):
    angle = np.radians(angle_deg)
    return np.array([0., np.sin(angle), np.cos(angle), -height * np.cos(angle)])


def mock_plane_discovery(monkeypatch, hypotheses):
    """Control discovery order; every returned inlier obeys the real threshold."""
    pending = iter(hypotheses)

    class Cloud:
        def __init__(self, points=None):
            self.points = np.empty((0, 3)) if points is None else points

        def segment_plane(self, distance_threshold, ransac_n, num_iterations):
            assert ransac_n == 3 and num_iterations == 700
            plane = next(pending)
            points = np.asarray(self.points)
            indices = np.flatnonzero(np.abs(points @ plane[:3] + plane[3]) <= distance_threshold)
            return plane, indices.tolist()

        def select_by_index(self, indices, invert=False):
            points = np.asarray(self.points)
            if not invert:
                return Cloud(points[indices])
            keep = np.ones(len(points), dtype=bool)
            keep[indices] = False
            return Cloud(points[keep])

    monkeypatch.setattr(geometry.o3d.geometry, 'PointCloud', Cloud)


def test_top_support_recovers_points_removed_by_normal_ineligible_plane(monkeypatch):
    points, top_count = overlapping_planes()
    cfg = replace(PerceptionConfig(), refine_top_surface=True)
    support_plane = np.array([0., 0., 1., 0.])
    raw = geometry.estimate_box(points, support_plane, cfg)
    mock_plane_discovery(monkeypatch, [tilted_plane(15, .03), tilted_plane(0, .03)])
    metrics = {}
    box = geometry.refine_box_from_top(points, support_plane, cfg, raw, metrics)
    assert np.allclose(box.dimensions, [.06, .018, .03])
    assert box.observed_bounds is raw
    assert metrics['top_points'] == top_count
    assert metrics['top_inlier_ratio'] == pytest.approx(top_count / len(points))
    candidate = metrics['plane_candidates'][1]
    assert candidate['discovery_points'] / len(points) < cfg.top_surface_min_ratio
    assert candidate['points'] == top_count
    assert candidate['support_reference'] == 'full_original_cluster'
    assert metrics['collision_geometry_replaced'] is False


def test_best_top_candidate_uses_full_support_not_remaining_support(monkeypatch):
    points, top_count = overlapping_planes()
    cfg = replace(PerceptionConfig(), refine_top_surface=True)
    support_plane = np.array([0., 0., 1., 0.])
    raw = geometry.estimate_box(points, support_plane, cfg)
    mock_plane_discovery(
        monkeypatch, [tilted_plane(15, .03), tilted_plane(7, .0275), tilted_plane(0, .03)])
    metrics = {}
    box = geometry.refine_box_from_top(points, support_plane, cfg, raw, metrics)
    first, second = metrics['plane_candidates'][1:]
    assert first['discovery_points'] >= second['discovery_points']
    assert first['points'] < second['points']
    assert metrics['top_points'] == top_count
    assert metrics['top_normal_error_deg'] == pytest.approx(0.)
    assert np.allclose(box.dimensions, [.06, .018, .03])
