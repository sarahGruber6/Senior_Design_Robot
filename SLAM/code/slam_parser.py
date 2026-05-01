from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import List, Optional

import numpy as np

MIN_RANGE_MM = 150
MAX_RANGE_MM = 10000
SCAN_BINS = 360
MAX_CONSECUTIVE_MISSING = 220
HARD_MIN_VALID_BINS = 30


@dataclass
class ParsedScan:
    seq: int
    ts_robot_ms: int
    distances_mm: List[int]
    valid_bins: int


def _fill_gaps(distances_mm: List[int]) -> Optional[List[int]]:
    arr = np.array(distances_mm, dtype=float)
    arr[(arr < MIN_RANGE_MM) | (arr > MAX_RANGE_MM)] = np.nan

    valid = np.isfinite(arr)
    valid_count = int(valid.sum())
    if valid_count < HARD_MIN_VALID_BINS:
        return None

    missing = (~valid).astype(int)
    doubled = np.concatenate([missing, missing])
    best = cur = 0
    for x in doubled:
        cur = (cur + 1) if x else 0
        best = max(best, cur)
    longest_gap = min(best, SCAN_BINS)

    if longest_gap > MAX_CONSECUTIVE_MISSING:
        return None

    idx = np.arange(SCAN_BINS)
    valid_idx = idx[valid]
    valid_vals = arr[valid]
    wrap_idx = np.concatenate([valid_idx - SCAN_BINS, valid_idx, valid_idx + SCAN_BINS])
    wrap_vals = np.tile(valid_vals, 3)

    interp = np.interp(idx, wrap_idx, wrap_vals)
    interp = np.clip(interp, MIN_RANGE_MM, MAX_RANGE_MM)
    return interp.astype(int).tolist()


def _apply_exclusions(distances: list, zones: list) -> list:
    out = list(distances)
    for center, half_width in zones:
        for offset in range(-half_width, half_width + 1):
            out[(center + offset) % SCAN_BINS] = 0
    return out


def parse_scan(payload: bytes, reverse_scan: bool = True, exclusion_zones: list = None) -> Optional[ParsedScan]:
    if len(payload) != 4 + 4 + 4 + 2 + 360 * 2:
        raise ValueError(f"SCN3: expected 734 bytes, got {len(payload)}")

    if payload[:4] != b"SCN3":
        raise ValueError(f"Unknown payload magic: {payload[:4]!r}")

    seq = struct.unpack_from("<I", payload, 4)[0]
    ts_robot_ms = struct.unpack_from("<I", payload, 8)[0]
    reported_valid = struct.unpack_from("<H", payload, 12)[0]
    distances = list(struct.unpack_from("<360H", payload, 14))

    if exclusion_zones:
        distances = _apply_exclusions(distances, exclusion_zones)

    filled = _fill_gaps(distances)
    if filled is None:
        raise ValueError(
            f"SCN3 seq={seq}: rejected (valid={reported_valid}, hard_min={HARD_MIN_VALID_BINS})"
        )

    if reverse_scan:
        filled = filled[::-1]

    valid_bins = sum(1 for d in distances if MIN_RANGE_MM <= d <= MAX_RANGE_MM)
    return ParsedScan(
        seq=seq,
        ts_robot_ms=ts_robot_ms,
        distances_mm=filled,
        valid_bins=valid_bins,
    )


parse_l360 = parse_scan