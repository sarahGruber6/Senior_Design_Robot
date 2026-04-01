# Parse L360 MQTT payloads from the Arduino lidar test sketch
# and return a BreezySLAM-ready scan.

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional

import numpy as np


MIN_RANGE_MM = 150
MAX_RANGE_MM = 10000
SCAN_BINS = 360

# loosen/tighten later?
MIN_VALID_BINS = 90
MAX_CONSECUTIVE_MISSING = 140

HARD_MIN_VALID_BINS = 50
SOFT_MIN_VALID_BINS = 90


@dataclass
class ParsedScan:
    seq: int
    ts_robot_ms: int
    distances_mm: List[int]       # final BreezySLAM-ready distances
    valid_bins: int
    start_deg: Optional[float] = None
    end_deg: Optional[float] = None
    scan_count: Optional[int] = None


def _parse_header_value(raw: str):
    # try int, then float, else keep string
    try:
        return int(raw)
    except ValueError:
        try:
            return float(raw)
        except ValueError:
            return raw


def _largest_missing_run(valid_mask: np.ndarray) -> int:
    # valid_mask True = valid, False = missing
    missing = ~valid_mask
    if missing.size == 0:
        return 0

    # circular longest run
    doubled = np.concatenate([missing, missing])
    best = 0
    cur = 0

    for x in doubled:
        if x:
            cur += 1
            if cur > best:
                best = cur
        else:
            cur = 0

    return min(best, missing.size)


def _fill_small_gaps(distances_mm: List[int]) -> Optional[List[int]]:
    arr = np.array(distances_mm, dtype=float)

    # invalid -> NaN
    arr[(arr < MIN_RANGE_MM) | (arr > MAX_RANGE_MM)] = np.nan

    valid = np.isfinite(arr)
    valid_count = int(valid.sum())

    if valid_count < HARD_MIN_VALID_BINS:
        return None

    longest_gap = _largest_missing_run(valid)
    if longest_gap > MAX_CONSECUTIVE_MISSING:
        return None

    idx = np.arange(SCAN_BINS)
    valid_idx = idx[valid]
    valid_vals = arr[valid]

    # circular interpolation
    wrap_idx = np.concatenate([valid_idx - SCAN_BINS, valid_idx, valid_idx + SCAN_BINS])
    wrap_vals = np.concatenate([valid_vals, valid_vals, valid_vals])

    interp = np.interp(idx, wrap_idx, wrap_vals)
    interp = np.clip(interp, MIN_RANGE_MM, MAX_RANGE_MM)

    return interp.astype(int).tolist()


def parse_l360(payload: str, reverse_scan: bool = True) -> ParsedScan:
    """
    Example payload:
    L360,seq=12,ts=123456,valid=248,scan_count=13,start_deg=4.25,end_deg=359.10;0,0,412,405,...

    Returns a ParsedScan with distances ready for slam.update(...).
    """
    payload = payload.strip()
    if not payload:
        raise ValueError("empty payload")

    if ";" not in payload:
        raise ValueError("missing header/body separator")

    header, body = payload.split(";", 1)

    parts = header.split(",")
    if not parts or parts[0] != "L360":
        raise ValueError(f"unexpected scan type: {parts[0] if parts else 'missing'}")

    meta = {}
    for item in parts[1:]:
        if "=" not in item:
            raise ValueError(f"bad header item: {item}")
        k, v = item.split("=", 1)
        meta[k] = _parse_header_value(v)

    raw_vals = [x.strip() for x in body.split(",") if x.strip() != ""]
    if len(raw_vals) != SCAN_BINS:
        raise ValueError(f"expected {SCAN_BINS} bins, got {len(raw_vals)}")

    distances = [int(x) for x in raw_vals]

    # sanitize obvious garbage
    cleaned = []
    for d in distances:
        if d < MIN_RANGE_MM or d > MAX_RANGE_MM:
            cleaned.append(0)
        else:
            cleaned.append(d)

    filled = _fill_small_gaps(cleaned)
    if filled is None:
        raise ValueError("scan rejected: insufficient valid bins or missing sector too large")

    # Match the direction your live system needs.
    # Keep this switch easy to flip during testing.
    if reverse_scan:
        filled = filled[::-1]

    valid_bins = sum(1 for d in cleaned if d >= MIN_RANGE_MM)

    return ParsedScan(
        seq=int(meta["seq"]),
        ts_robot_ms=int(meta["ts"]),
        distances_mm=filled,
        valid_bins=valid_bins,
        start_deg=float(meta["start_deg"]) if "start_deg" in meta else None,
        end_deg=float(meta["end_deg"]) if "end_deg" in meta else None,
        scan_count=int(meta["scan_count"]) if "scan_count" in meta else None,
    )