"""Photo-calibrated, pixel-only Stage 2 distance profiles.

Each mask contains only HSV-qualified source pixels. Geometry is reported for
annotations, never used to admit an obstacle. Calibration lives outside the app.
"""
from __future__ import annotations

import json
from math import isfinite
from pathlib import Path
from time import perf_counter

import obstacle

PROFILE_PATH = Path(__file__).with_name('stage2_distance_profiles.json')
TARGET_KINDS = dict.fromkeys(('blue_red', 'orange', 'brown'), 'color_obstacle')


def load_profiles(path=PROFILE_PATH):
    values = json.loads(Path(path).read_text(encoding='utf-8'))
    if values.get('version') != 2:
        raise ValueError('Unsupported Stage 2 distance calibration version')
    for name in ('near', 'far', 'combined'):
        current = values['profiles'][name]
        x0, y0, x1, y1 = current['roi']
        if not (0 <= x0 < x1 <= 1 and 0 <= y0 < y1 <= 1):
            raise ValueError(f'Invalid {name} distance ROI')
        lower, upper = current['area_interval']
        if (any(isinstance(value, bool) or not isinstance(value, (int, float))
                or not isfinite(value) for value in (lower, upper))
                or not 0 < lower <= upper <= 1):
            raise ValueError(f'Invalid {name} distance interval')
    for color, ranges in values['hsv'].items():
        for lower, upper in ranges:
            if (len(lower) != 3 or len(upper) != 3
                    or any(not isinstance(lo, int) or isinstance(lo, bool)
                           or not isinstance(hi, int) or isinstance(hi, bool)
                           or not 0 <= lo <= hi <= maximum
                           for lo, hi, maximum in zip(lower, upper, (179, 255, 255)))):
                raise ValueError(f'Invalid {color} HSV range')
    if values['pixel_filters']['print_reference_kernel_px'] <= 0:
        raise ValueError('Invalid print-reference kernel')
    return values


def same_distance_color(previous, current):
    """Match unified obstacle hits; frame and road barriers belong to callers."""
    return bool(previous and current and previous.available and current.available
                and previous.detected and current.detected)


class DistanceColorDetector:
    def __init__(self, profile='combined', *, profiles=None):
        self.values = load_profiles() if profiles is None else profiles
        if profile not in ('near', 'far', 'combined'):
            raise ValueError(f'Unknown Stage 2 distance profile: {profile}')
        self.profile = profile
        self.settings = self.values['profiles'][profile]

    @staticmethod
    def _bounds(shape, fractions):
        height, width = shape[:2]
        x0, y0, x1, y1 = fractions
        return (round(width * x0), round(height * y0),
                round(width * x1), round(height * y1))

    def color_masks(self, bgr):
        """One full-resolution color pass shared by all distance measurements."""
        import cv2
        import numpy as np

        if (bgr is None or getattr(bgr, 'ndim', 0) != 3 or not bgr.size
                or bgr.shape[2] != 3 or bgr.dtype != np.uint8):
            raise ValueError('Camera frame must be nonempty uint8 BGR')
        height, width = bgr.shape[:2]
        lighting = self.values['lighting']
        # Estimate on a fixed central full-height sample so all three modes
        # measure the same pixels, irrespective of their acceptance ROI.
        sample = bgr[:, round(width * .2):round(width * .8)][::4, ::4]
        sample_hsv = cv2.cvtColor(sample, cv2.COLOR_BGR2HSV)
        neutral = ((sample_hsv[:, :, 1] <= lighting['neutral_max_saturation'])
                   & (sample_hsv[:, :, 2] >= lighting['neutral_min_value'])
                   & (sample_hsv[:, :, 2] <= lighting['neutral_max_value']))
        gains = np.ones(3, np.float32)
        applied = np.count_nonzero(neutral) >= max(16, sample.shape[0] * sample.shape[1] * .05)
        if applied:
            channels = np.median(sample[neutral], axis=0)
            gains = np.clip(np.mean(channels) / np.maximum(channels, 1),
                            lighting['gain_min'], lighting['gain_max']).astype(np.float32)
        # Bright neutral pixels are excluded from the gain fit to avoid
        # clipping bias, but still represent brightness. Keeping them in this
        # separate reference prevents dark rails from replacing a bright wall.
        brightness_reference = ((sample_hsv[:, :, 1] <= lighting['neutral_max_saturation'])
                                & (sample_hsv[:, :, 2] >= lighting['neutral_min_value']))
        corrected_sample = cv2.transform(sample, np.diag(gains))
        sample_value = cv2.cvtColor(corrected_sample, cv2.COLOR_BGR2HSV)[:, :, 2]
        neutral_value = (float(np.percentile(sample_value[brightness_reference],
            lighting['brightness_reference_percentile'])) if np.any(brightness_reference) else 255.0)
        raw = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        adjusted = (cv2.cvtColor(np.clip(bgr.astype(np.float32) * gains, 0, 255).astype(np.uint8),
                                cv2.COLOR_BGR2HSV) if applied else raw)
        masks = {}
        for color, ranges in self.values['hsv'].items():
            first = np.zeros((height, width), np.uint8)
            second = np.zeros_like(first)
            for lower, upper in ranges:
                first |= cv2.inRange(raw, tuple(lower), tuple(upper))
                second |= cv2.inRange(adjusted, tuple(lower), tuple(upper))
            if applied:
                first[adjusted[:, :, 1] < min(lower[1] for lower, _ in ranges)] = 0
            masks[color] = first | second
        hue, saturation, value = (adjusted[:, :, i] for i in range(3))
        raw_saturation = raw[:, :, 1]
        filters = self.values['pixel_filters']
        wood_evidence = ((saturation >= filters['wood_normal_saturation_min'])
            | ((saturation >= filters['wood_shadow_saturation_min'])
               & (value <= filters['wood_shadow_value_max'])
               & (raw_saturation >= filters['wood_shadow_raw_saturation_min'])))
        masks['brown'][~wood_evidence] = 0
        # A local brightness filter only removes dark print pixels. No
        # connectivity, closing, filling, or component geometry is evaluated.
        kernel = filters['print_reference_kernel_px']
        surrounding = cv2.blur(raw[:, :, 2], (kernel, kernel))
        print_pixels = raw[:, :, 2] < surrounding * filters['print_minimum_brightness_ratio']
        masks['orange'][print_pixels] = 0
        masks['brown'][print_pixels] = 0
        masks['red'][print_pixels] = 0
        clipped = ((raw_saturation <= lighting['neutral_max_saturation'])
                   & (raw[:, :, 2] > lighting['neutral_max_value']))
        for mask in masks.values():
            mask[clipped] = 0
        masks['blue_red'] = masks.pop('blue') | masks.pop('red')
        return masks, dict(white_balance_applied=bool(applied),
            white_balance_bgr_gains=[float(gain) for gain in gains],
            neutral_sample_pixels=int(np.count_nonzero(neutral)),
            neutral_reference_value=neutral_value)

    def detect(self, bgr, *, diagnostics=None, continuation_active=False):
        import cv2
        import numpy as np

        started = perf_counter()
        try:
            masks, lighting = self.color_masks(bgr)
        except (ValueError, TypeError, cv2.error) as exc:
            if diagnostics is not None:
                diagnostics.update(profile=self.profile, error=str(exc), available=False)
            return obstacle.ObstacleResult.unavailable(str(exc))
        return self._detect_prepared(bgr, masks, lighting, diagnostics=diagnostics,
                                    continuation_active=continuation_active, started=started)

    def _detect_prepared(self, bgr, masks, lighting, *, diagnostics=None,
                         continuation_active=False, started=None):
        """Reuse one measured pixel pass for external all-profile evaluation."""
        import cv2
        import numpy as np
        started = perf_counter() if started is None else started
        height, width = bgr.shape[:2]
        roi = self._bounds(bgr.shape, self.settings['roi'])
        bounds = (self._bounds(bgr.shape, (.2, 0, .8, 1))
                  if continuation_active and self.profile == 'far' else roi)
        lower, upper = self.settings['area_interval']
        rows, selected_masks = {}, {}
        for color in TARGET_KINDS:
            x0, y0, x1, y1 = bounds
            selected = np.zeros((height, width), np.uint8)
            selected[y0:y1, x0:x1] = masks[color][y0:y1, x0:x1]
            pixels = int(cv2.countNonZero(selected))
            ratio = pixels / (width * height)
            rows[color] = dict(area_px=pixels, area_ratio=ratio, roi=bounds)
            selected_masks[color] = selected
        # Ordered keys make equal counts deterministic for annotations. Only
        # the largest channel is judged: no sum or smaller-channel fallback.
        dominant = max(rows, key=lambda color: rows[color]['area_px'])
        count = rows[dominant]['area_px']
        ratio = rows[dominant]['area_ratio']
        eligible = lower <= ratio and (continuation_active or ratio <= upper)
        mask = selected_masks[dominant]
        box = cv2.boundingRect(mask) if count else None
        result = obstacle.ObstacleResult(bool(eligible), 'color_obstacle' if eligible else None, float(count),
            box, (box[0] + box[2] / 2, box[1] + box[3] / 2) if box else None, True)
        if diagnostics is not None:
            diagnostics.update(profile=self.profile, available=True, roi=bounds, working_roi=bounds,
                initial_roi=roi, completion_mask=mask, dominant_channel=dominant,
                target_kind='color_obstacle' if eligible else None,
                target_color_area_px=count, area_ratio=ratio, judgment_area_ratio=ratio,
                area_interval=[lower, upper], stop_eligible=bool(eligible),
                color_candidates=rows, perchannel=rows, interval_eligible=bool(eligible),
                continuation_active=bool(continuation_active),
                parameters=lighting, detection_elapsed_s=perf_counter() - started)
        return result
