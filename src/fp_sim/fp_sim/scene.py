import numpy as np
import cv2

# ADE20K label ids used by the scene
FLOOR, WALL, GRASS, TABLE, BOX, CAR, CABINET, POLE = 3, 0, 9, 15, 41, 20, 10, 93

BG = (60, 60, 60)
COLORS = {
    FLOOR: (120, 120, 120),
    WALL: (140, 150, 160),
    GRASS: (60, 140, 60),
    TABLE: (255, 255, 255),
    BOX: (40, 40, 160),
    CAR: (30, 30, 180),
    CABINET: (140, 90, 60),
    POLE: (200, 200, 40),
}

SQUARE_M = 0.035
BOARD_CORNERS = (11, 8)  # inner corners


class SimScene:
    """Static indoor scene in camera frame (x right, y down, z forward)."""

    def __init__(self, R_l2c, t_l2c):
        self.R = np.asarray(R_l2c, float)
        self.t = np.asarray(t_l2c, float)
        self.boxes = [
            (np.array([2.0, -0.20, 2.6]), np.array([3.4, 0.45, 3.4]), CABINET),
            (np.array([-2.6, -0.25, 3.0]), np.array([-1.6, 0.45, 3.4]), BOX),
            (np.array([3.6, -0.75, 2.0]), np.array([5.2, 0.45, 3.9]), CAR),
            (np.array([-2.0, -0.45, 1.9]), np.array([-1.86, 0.45, 2.1]), POLE),
        ]
        self.floor_y = 0.45
        self.wall_z = 4.0
        bw = BOARD_CORNERS[0] + 1
        bh = BOARD_CORNERS[1] + 1
        self.board_hw = bw * SQUARE_M / 2.0
        self.board_hh = bh * SQUARE_M / 2.0
        self.board_base = np.array([0.0, -0.10, 1.40])

    # ---------------- board pose ----------------
    def board_pose(self, ts):
        c = self.board_base + np.array([
            0.40 * np.sin(0.50 * ts),
            -0.10 + 0.16 * np.sin(0.37 * ts + 1.0),
            0.22 * np.sin(0.21 * ts + 0.5)])
        a = np.array([
            np.deg2rad(10 * np.sin(0.53 * ts + 0.4)),
            np.deg2rad(20 * np.sin(0.31 * ts)),
            np.deg2rad(8 * np.sin(0.44 * ts + 2.0))])
        Rb = cv2.Rodrigues(a)[0]
        return c, Rb

    # ---------------- camera rendering (painter) ----------------
    def _proj(self, pts, K, W, H):
        r = cv2.Rodrigues(np.eye(3))[0]
        uv, _ = cv2.projectPoints(np.asarray(pts, float), r, np.zeros((3, 1)), K, None)
        return uv.reshape(-1, 2)

    def _poly(self, img, p3, K, color):
        p3 = np.asarray(p3, float)
        if (p3[:, 2] < 0.08).any():
            return
        uv = self._proj(p3, K, *img.shape[1::-1])
        cv2.fillPoly(img, [uv.astype(np.int32)], color)

    def render(self, K, W, H, ts):
        img = np.full((H, W, 3), BG, np.uint8)
        polys = []
        fy, wz = self.floor_y, self.wall_z
        for x0 in np.arange(-6, 6, 1.0):
            for z0 in np.arange(0.5, 3.95, 1.0):
                shade = 120 if int(x0 + 6) % 2 == int((z0 - 0.5)) % 2 else 145
                polys.append((z0 + 0.5, [(x0, fy, z0), (x0 + 1, fy, z0), (x0 + 1, fy, z0 + 1),
                                          (x0, fy, z0 + 1)], ('tile', shade)))
        polys.append((wz, [(-8, -1.2, wz), (8, -1.2, wz), (8, fy, wz), (-8, fy, wz)], WALL))
        polys.append((2.8, [(-4.5, fy - 0.001, 2.2), (-2.5, fy - 0.001, 2.2),
                            (-2.5, fy - 0.001, 3.4), (-4.5, fy - 0.001, 3.4)], GRASS))
        for bmin, bmax, lab in self.boxes:
            d = (bmin + bmax) / 2
            for i in range(3):
                for sgn in (0, 1):
                    q = np.array(list(np.ndindex(2, 2)))
                    ax = [j for j in range(3) if j != i]
                    corners = []
                    for a, b in q:
                        pt = d.copy()
                        pt[i] = bmax[i] if sgn else bmin[i]
                        pt[ax[0]] = bmax[ax[0]] if a else bmin[ax[0]]
                        pt[ax[1]] = bmax[ax[1]] if b else bmin[ax[1]]
                        corners.append(pt)
                    if np.mean(corners, axis=0)[2] > 0.1:
                        order = [0, 1, 3, 2]
                        polys.append((np.mean(corners, axis=0)[2], [corners[k] for k in order], lab))
        c, Rb = self.board_pose(ts)
        u, v, n = Rb[:, 0], Rb[:, 1], Rb[:, 2]
        hw, hh = self.board_hw, self.board_hh
        polys.append((np.linalg.norm(c), [c - u * hw - v * hh, c + u * hw - v * hh,
                                         c + u * hw + v * hh, c - u * hw + v * hh], TABLE))
        polys.sort(key=lambda p: -p[0])
        for depth, quad, lab in polys:
            color = COLORS[lab] if isinstance(lab, int) else (lab[1], lab[1], lab[1])
            self._poly(img, quad, K, color)
        bw, bh = BOARD_CORNERS[0] + 1, BOARD_CORNERS[1] + 1
        sq = SQUARE_M
        for r in range(bh):
            for col in range(bw):
                if (r + col) % 2 == 0:
                    continue
                p0 = c - u * hw - v * hh
                a = p0 + u * (col * sq) + v * (r * sq)
                self._poly(img, [a, a + u * sq, a + u * sq + v * sq, a + v * sq], K, (10, 10, 10))
        return img

    # ---------------- lidar raycast (rays in lidar frame) ----------------
    def scan(self, ts, n_az=1800, n_el=16, max_range=20.0, fov=30.0, rng=None):
        els = np.deg2rad(np.linspace(-fov / 2, fov / 2, n_el))
        azs = np.linspace(0, 2 * np.pi, n_az, endpoint=False)
        E, A = np.meshgrid(els, azs, indexing='ij')
        d_l = np.stack([np.cos(E) * np.cos(A), np.cos(E) * np.sin(A), np.sin(E)], -1).reshape(-1, 3)
        d_c = (self.R @ d_l.T).T
        o_c = self.t
        best_t = np.full(len(d_l), np.inf)
        best_lab = np.full(len(d_l), 255, np.uint8)

        def hit(tt, lab, cond):
            nonlocal best_t, best_lab
            ok = cond & (tt > 0.05) & (tt < max_range) & (tt <= best_t)
            best_t = np.where(ok, tt, best_t)
            best_lab = np.where(ok, lab, best_lab)

        # floor / grass (y = fy plane)
        with np.errstate(divide='ignore', invalid='ignore'):
            tf = (self.floor_y - o_c[1]) / d_c[:, 1]
            px = o_c[0] + tf * d_c[:, 0]
            pz = o_c[2] + tf * d_c[:, 2]
        cond = (d_c[:, 1] > 1e-6) & (px > -6) & (px < 6) & (pz > 0.4) & (pz < 3.99)
        hit(tf, FLOOR, cond)
        g = cond & (px > -4.5) & (px < -2.5) & (pz > 2.2) & (pz < 3.4)
        hit(tf, GRASS, g)
        # wall
        with np.errstate(divide='ignore', invalid='ignore'):
            tw = (self.wall_z - o_c[2]) / d_c[:, 2]
            wx = o_c[0] + tw * d_c[:, 0]
            wy = o_c[1] + tw * d_c[:, 1]
        hit(tw, WALL, (d_c[:, 2] > 1e-6) & (wx > -8) & (wx < 8) & (wy > -1.2) & (wy < self.floor_y))
        # boxes (AABB slab test)
        for bmin, bmax, lab in self.boxes:
            t0 = np.full(len(d_c), -np.inf)
            t1 = np.full(len(d_c), np.inf)
            for i in range(3):
                with np.errstate(divide='ignore', invalid='ignore'):
                    ta = (bmin[i] - o_c[i]) / d_c[:, i]
                    tb = (bmax[i] - o_c[i]) / d_c[:, i]
                lo, hi = np.minimum(ta, tb), np.maximum(ta, tb)
                par = np.abs(d_c[:, i]) < 1e-9
                lo = np.where(par & ((o_c[i] < bmin[i]) | (o_c[i] > bmax[i])), np.inf, lo)
                hi = np.where(par, -np.inf, hi)
                t0 = np.maximum(t0, lo)
                t1 = np.minimum(t1, hi)
            inside = (o_c > bmin) & (o_c < bmax)
            tt = np.where(inside.all(), 0.01, t0)
            hit(tt, lab, (t1 >= t0) & (t0 > 0))
        # board
        c, Rb = self.board_pose(ts)
        u, v, n = Rb[:, 0], Rb[:, 1], Rb[:, 2]
        with np.errstate(divide='ignore', invalid='ignore'):
            tb = ((c - o_c) @ n) / (d_c @ n)
        P = o_c[None, :] + tb[:, None] * d_c
        rel = P - c
        au, av = rel @ u, rel @ v
        hit(tb, TABLE, (tb > 0.05) & (np.abs(au) < self.board_hw) & (np.abs(av) < self.board_hh) & (tb < max_range))

        alive = np.isfinite(best_t)
        P_l = d_l[alive] * best_t[alive][:, None]
        if rng is None:
            rng = np.random.default_rng(0)
        P_l += rng.normal(scale=0.006, size=P_l.shape)
        return P_l.astype(np.float32), best_lab[alive]

