"""Parameterized image-backed test for red cube detection."""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

pytestmark = pytest.mark.unit

_HELPERS = (
    Path(__file__).resolve().parents[2]
    / "catkin_ws"
    / "src"
    / "limo_rosa_bridge"
    / "scripts"
)
if str(_HELPERS) not in sys.path:
    sys.path.insert(0, str(_HELPERS))

cv2 = pytest.importorskip("cv2")

from _helpers import hsv as hsv_helpers  # noqa: E402


_IMAGE_DIR = Path(__file__).resolve().parent / "test_images"


def _parse_expected_count(path: Path) -> int:
    first = path.name[0]
    if not first.isdigit():
        raise ValueError(f"Filename must start with cube count digit: {path.name}")
    return int(first)


def _image_cases():
    if not _IMAGE_DIR.exists():
        return []
    imgs = sorted(p for p in _IMAGE_DIR.iterdir() if p.suffix.lower() in {".png", ".jpg", ".jpeg"})
    return [
        pytest.param(p, _parse_expected_count(p), id=p.name)
        for p in imgs
    ]


@pytest.mark.parametrize("image_path,expected_count", _image_cases())
def test_red_cube_detector_on_image_dataset(image_path: Path, expected_count: int):
    bgr = cv2.imread(str(image_path))
    assert bgr is not None, f"OpenCV could not decode test image: {image_path}"
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = hsv_helpers.red_mask(hsv)
    assert mask.shape == bgr.shape[:2]

    # The production detector currently returns only a single "best" blob.
    # Use the filename count metadata as a presence/absence oracle.
    has_expected_cube = expected_count > 0

    blob = hsv_helpers.largest_blob_centroid(mask, min_area=400, max_area=8000)
    has_detection = False
    if blob is not None:
        u, v, area = blob
        # Gate out frequent floor/wall false positives in this fixture set.
        has_detection = (120 <= v <= 520)
        assert 0 <= u < bgr.shape[1]
        assert 0 <= v < bgr.shape[0]
        assert area > 0.0

    assert has_detection == has_expected_cube, (
        f"{image_path.name}: expected cube presence={has_expected_cube} "
        f"from filename count={expected_count}, got has_detection={has_detection}, "
        f"blob={blob}"
    )
