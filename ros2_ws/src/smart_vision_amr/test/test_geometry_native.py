"""Optional compiled geometry must retain the independent NumPy guard result."""
import math

import numpy as np
import pytest

from smart_vision_amr import braking_envelope as braking

native = pytest.importorskip('smart_vision_amr._geometry_native')


@pytest.mark.parametrize('distance', [-.5, -.015, 0., .015, .5])
@pytest.mark.parametrize('angle', [-7., -math.pi, -.2, -1.e-6, -1.e-7,
                                  0., 1.e-7, 1.e-6, .2, math.pi, 7.])
def test_signed_arcs_and_uncertainty_match_numpy(distance, angle):
    rng = np.random.default_rng(20261105)
    points = np.column_stack((np.arange(360), rng.uniform(-1., 1., (360, 2))))
    for longitudinal, lateral, yaw in ((0., 0., 0.), (.08, .015, .17)):
        args = (points, distance, angle, .22, longitudinal, lateral, yaw)
        result = np.frombuffer(native.clearances(*args), dtype=np.float64)
        expected = braking._clearances_numpy(*args)
        np.testing.assert_allclose(result, expected, rtol=0., atol=2.e-15)
        np.testing.assert_array_equal(result <= 1.e-9, expected <= 1.e-9)


def test_straight_threshold_hits_retain_one_nanometre_rule():
    points = np.array([(i, x, y) for i, (x, y) in enumerate(
        [(0., .22), (.3, .22+1.e-9), (.3, .22+2.e-9),
         (.3, -.22), (-.3, -.22+1.e-9), (.5, 0.)])])
    args = (points, .4, 0., .22, 0., 0., 0.)
    expected = braking._clearances_numpy(*args)
    result = np.frombuffer(native.clearances(*args), dtype=np.float64)
    np.testing.assert_array_equal(result, expected)
    np.testing.assert_array_equal(result <= 1.e-9, expected <= 1.e-9)


def test_empty_and_unaligned_float64_buffers_are_supported():
    empty = np.empty((0, 3), dtype=np.float64)
    assert native.clearances(empty, .2, .3, .22, 0., 0., 0.) == b''
    backing = bytearray(1+6*8)
    points = np.ndarray((2, 3), dtype=np.float64, buffer=backing, offset=1)
    points[:] = [[0., .4, .2], [1., -.3, -.2]]
    args = (points, -.2, -.4, .22, .03, .01, .1)
    result = np.frombuffer(native.clearances(*args), dtype=np.float64)
    np.testing.assert_allclose(result, braking._clearances_numpy(*args), atol=1.e-15)


@pytest.mark.parametrize('points', [
    np.zeros((3, 3), dtype=np.float32),
    np.zeros((3, 2), dtype=np.float64),
    np.zeros((3, 3), dtype=np.float64)[::2],
    np.array([[0., math.nan, 0.]]),
    np.array([[0., 0., math.inf]]),
])
def test_invalid_input_buffers_raise(points):
    with pytest.raises(ValueError):
        native.clearances(points, .2, .3, .22, 0., 0., 0.)


@pytest.mark.parametrize('parameters', [
    (math.inf, 0., .22, 0., 0., 0.),
    (.2, math.nan, .22, 0., 0., 0.),
    (.2, 0., 0., 0., 0., 0.),
    (.2, 0., .22, -.01, 0., 0.),
    (.2, 0., .22, 0., -.01, 0.),
    (.2, 0., .22, 0., 0., -.01),
])
def test_invalid_geometry_parameters_raise(parameters):
    with pytest.raises(ValueError):
        native.clearances(np.zeros((1, 3)), *parameters)


def test_box_union_matches_dense_numpy_distances():
    rng = np.random.default_rng(16)
    points = np.column_stack((np.arange(360), rng.uniform(-2., 2., (360, 2))))
    lows = rng.uniform(-.3, .3, (2, 60))
    highs = lows+rng.uniform(0., .05, (2, 60))
    boxes = np.ascontiguousarray([lows[0], highs[0], lows[1], highs[1]])
    x, y = points[:, 1, None], points[:, 2, None]
    dx = np.maximum(np.maximum(boxes[0]-x, x-boxes[1]), 0.)
    dy = np.maximum(np.maximum(boxes[2]-y, y-boxes[3]), 0.)
    expected = np.min(np.sqrt(dx*dx+dy*dy)-.22, axis=1)
    result = np.frombuffer(native.box_clearances(points, boxes, .22), dtype=np.float64)
    np.testing.assert_array_equal(result, expected)


@pytest.mark.parametrize('boxes', [
    np.zeros((3, 4)), np.empty((4, 0)), np.zeros((4, 3), dtype=np.float32),
    np.array([[1.], [0.], [0.], [1.]]),
    np.array([[0.], [1.], [math.nan], [1.]]),
])
def test_invalid_box_bounds_raise(boxes):
    with pytest.raises(ValueError):
        native.box_clearances(np.zeros((1, 3)), boxes, .22)


def test_dispatcher_fallback_retains_original_numpy_kernel(monkeypatch):
    monkeypatch.setattr(braking, '_native_clearances', None)
    points = np.array([[0., .3, .1], [1., -.1, -.4]])
    args = (points, .2, -.3, .22, .01, .02, .03)
    np.testing.assert_array_equal(braking._clearances(*args), braking._clearances_numpy(*args))
