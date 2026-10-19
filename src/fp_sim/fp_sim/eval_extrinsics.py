import argparse

import numpy as np

from fp_common.transforms import load_calibration, project_points


def angle_diff_deg(R1, R2):
    tr = np.clip((np.trace(R1.T @ R2) - 1.0) / 2.0, -1, 1)
    return np.rad2deg(np.arccos(tr))


def main():
    ap = argparse.ArgumentParser(description='compare two calibration yamls (est vs truth)')
    ap.add_argument('est')
    ap.add_argument('truth')
    ap.add_argument('--depths', default='2,5,10,20', help='meters, comma list')
    a = ap.parse_args()
    e, t = load_calibration(a.est), load_calibration(a.truth)
    dR = angle_diff_deg(e['R'], t['R'])
    dtv = (e['t'] - t['t']) * 1000
    dt = np.linalg.norm(dtv)
    print(f'rotation error: {dR:.3f} deg')
    print(f'translation error: {dt:.1f} mm  (dx={dtv[0]:+.0f} dy={dtv[1]:+.0f} dz={dtv[2]:+.0f} mm, cam frame)')
    rng = np.random.default_rng(7)
    for d0 in [float(x) for x in a.depths.split(',')]:
        # lidar frame: x forward, y left, z up
        P = np.column_stack([rng.uniform(0.7, 1.3, 3000) * d0,
                             rng.normal(0, 0.25, 3000) * d0,
                             rng.normal(0, 0.2, 3000) * d0])
        uv1, dep1, _ = project_points(P, e['K'], e['R'], e['t'], e['D'])
        uv2, dep2, _ = project_points(P, t['K'], t['R'], t['t'], t['D'])
        err = np.linalg.norm(uv1 - uv2, axis=1)
        m = (dep1 > 0.5) & (dep2 > 0.5)
        print(f'{d0:>5.1f} m: mean reproj diff = {err[m].mean():6.2f} px, '
              f'90th = {np.percentile(err[m], 90):6.2f} px')


if __name__ == '__main__':
    main()
