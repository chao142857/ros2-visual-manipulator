"""Replaceable instance segmentation and a separate target-selection policy.

A future semantic plugin implements segment(cloud) -> list[index arrays].
Indices refer to the preprocessed above-table cloud. It must not estimate OBBs
or generate grasps. A synchronized image mask can be projected to these points
in a separate frontend without changing geometry or planning.
"""

import importlib
from typing import Protocol

import numpy as np


class Segmenter(Protocol):
    def segment(self, cloud) -> list:
        ...


class GeometrySegmenter:
    def __init__(self, config):
        self.config = config

    def segment(self, cloud):
        labels = np.asarray(cloud.cluster_dbscan(
            eps=self.config.cluster_eps_m, min_points=self.config.cluster_min_points,
            print_progress=False))
        return [np.flatnonzero(labels == label) for label in np.unique(labels) if label >= 0]


def load_segmenter(path, config):
    if not path:
        return GeometrySegmenter(config)
    module, name = path.rsplit(':', 1)
    result = getattr(importlib.import_module(module), name)(config)
    if not callable(getattr(result, 'segment', None)):
        raise ValueError('segmenter must implement segment(cloud)')
    return result


def select_target(boxes, policy):
    if not boxes:
        raise ValueError('no validated tabletop object')
    if policy == 'isolated' and len(boxes) != 1:
        raise ValueError(f'ambiguous scene: {len(boxes)} objects; isolated policy requires one')
    if policy not in ('isolated', 'largest'):
        raise ValueError('unknown selection policy')
    return int(np.argmax([box.point_count for box in boxes]))
