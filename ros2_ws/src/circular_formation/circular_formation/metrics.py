"""Formation quality metrics (pure Python), shared by the monitor and tests."""

import math
from dataclasses import dataclass
from typing import Sequence, Tuple

from .geometry import TWO_PI, all_gaps, centroid, polar


@dataclass
class Metrics:
    center: Tuple[float, float]
    mean_radius: float
    mean_radius_err: float   # mean |rho - R|, metres
    max_radius_err: float    # max  |rho - R|, metres
    max_gap_err_deg: float   # max |gap - 2pi/N|, degrees
    min_separation: float    # closest pair, metres


def formation_metrics(points: Sequence[Tuple[float, float]], radius: float,
                      center_mode: str = 'centroid',
                      fixed_center: Tuple[float, float] = (0.0, 0.0)
                      ) -> Metrics:
    n = len(points)
    c = fixed_center if center_mode == 'fixed' else centroid(points)
    polars = [polar(p, c) for p in points]
    errs = [abs(rho - radius) for rho, _ in polars]
    gaps = all_gaps([a for _, a in polars], list(range(n)))
    ideal = TWO_PI / n
    gap_err = max(abs(g - ideal) for g in gaps) if n > 1 else 0.0
    sep = min((math.hypot(points[i][0] - points[j][0],
                          points[i][1] - points[j][1])
               for i in range(n) for j in range(i + 1, n)),
              default=float('inf'))
    return Metrics(center=c,
                   mean_radius=sum(rho for rho, _ in polars) / n,
                   mean_radius_err=sum(errs) / n,
                   max_radius_err=max(errs),
                   max_gap_err_deg=math.degrees(gap_err),
                   min_separation=sep)
