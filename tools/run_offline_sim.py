#!/usr/bin/env python3
"""Run the formation controller offline (no ROS, no Gazebo) and plot it.

    python3 tools/run_offline_sim.py --robots 6 --radius 2.5 --omega 0.05 \
        --out docs/offline_convergence.png

Needs numpy-free pure Python for the simulation itself; matplotlib only for
the plot (pip install matplotlib).
"""

import argparse
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'ros2_ws', 'src',
                                'circular_formation'))

from circular_formation.control_law import FormationParams  # noqa: E402
from circular_formation.simulation import simulate  # noqa: E402
from circular_formation.spawn_layout import spawn_poses  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--robots', type=int, default=5)
    ap.add_argument('--radius', type=float, default=2.0)
    ap.add_argument('--omega', type=float, default=0.05)
    ap.add_argument('--center-mode', choices=['centroid', 'fixed'],
                    default='centroid')
    ap.add_argument('--layout', choices=['spiral', 'ring'], default='spiral')
    ap.add_argument('--spawn-scale', type=float, default=0.8)
    ap.add_argument('--duration', type=float, default=240.0)
    ap.add_argument('--out', default='', help='PNG path (omit: print only)')
    a = ap.parse_args()

    prm = FormationParams(radius=a.radius, omega=a.omega,
                          center_mode=a.center_mode, num_robots=a.robots)
    res = simulate(spawn_poses(a.robots, a.layout, a.spawn_scale), prm,
                   duration=a.duration)
    m = res.metrics[-1]
    print(f'final: mean R = {m.mean_radius:.3f} m (target {a.radius}), '
          f'max |R-err| = {m.max_radius_err:.4f} m, '
          f'max gap err = {m.max_gap_err_deg:.2f} deg, '
          f'closest approach over run = {res.min_separation:.2f} m')

    if a.out:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt

        fig, (ax, bx) = plt.subplots(1, 2, figsize=(11, 4.6))
        for i, path in enumerate(res.paths):
            xs, ys = zip(*path)
            ax.plot(xs, ys, lw=1.2, label=f'robot {i}')
            ax.plot(xs[0], ys[0], 'o', ms=5, color=ax.lines[-1].get_color(),
                    mfc='white')
            ax.plot(xs[-1], ys[-1], 'o', ms=7, color=ax.lines[-1].get_color())
        t = [k / 100 for k in range(361)]
        cx, cy = m.center
        ax.plot([cx + a.radius * math.cos(2 * math.pi * k / 360)
                 for k in range(361)],
                [cy + a.radius * math.sin(2 * math.pi * k / 360)
                 for k in range(361)], 'k--', lw=0.8, label='target circle')
        ax.set_aspect('equal')
        ax.set_title(f'{a.robots} robots, R={a.radius} m, '
                     f'centre={a.center_mode} (o start, filled end)')
        ax.legend(fontsize=7, loc='upper right')
        ax.grid(alpha=0.3)

        bx.plot(res.t, [x.max_radius_err for x in res.metrics],
                label='max |radius error| [m]')
        bx.plot(res.t, [x.max_gap_err_deg / 10 for x in res.metrics],
                label='max gap error [deg / 10]')
        bx.set_yscale('log')
        bx.set_xlabel('time [s]')
        bx.set_title('convergence')
        bx.legend(fontsize=8)
        bx.grid(alpha=0.3, which='both')
        fig.tight_layout()
        os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
        fig.savefig(a.out, dpi=130)
        print('wrote', a.out)


if __name__ == '__main__':
    main()
