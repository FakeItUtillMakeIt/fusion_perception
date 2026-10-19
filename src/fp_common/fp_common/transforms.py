import numpy as np
import yaml
import cv2


def load_calibration(path):
    with open(path) as f:
        d = yaml.safe_load(f)
    K = np.asarray(d['K'], dtype=np.float64).reshape(3, 3)
    D = np.asarray(d.get('D') or [], dtype=np.float64).ravel()
    R = np.asarray(d['R_lidar_to_cam'], dtype=np.float64).reshape(3, 3)
    t = np.asarray(d['t_lidar_to_cam'], dtype=np.float64).ravel()
    size = (int(d['image_width']), int(d['image_height']))
    return {'K': K, 'D': D, 'R': R, 't': t, 'size': size}


def save_calibration(path, K, D, R, t, size, note=''):
    d = {
        'image_width': int(size[0]),
        'image_height': int(size[1]),
        'K': [float(x) for x in np.asarray(K).ravel()],
        'D': [float(x) for x in np.asarray(D).ravel()],
        'R_lidar_to_cam': [float(x) for x in np.asarray(R).ravel()],
        't_lidar_to_cam': [float(x) for x in np.asarray(t).ravel()],
    }
    if note:
        d['note'] = note
    with open(path, 'w') as f:
        yaml.safe_dump(d, f, sort_keys=False)


def project_points(P_lidar, K, R, t, D=None):
    P_lidar = np.ascontiguousarray(P_lidar, dtype=np.float64)
    rvec = cv2.Rodrigues(np.ascontiguousarray(R))[0]
    tvec = np.ascontiguousarray(t, dtype=np.float64).reshape(3, 1)
    dist = np.ascontiguousarray(D, dtype=np.float64) if D is not None and len(D) else None
    uv, _ = cv2.projectPoints(P_lidar, rvec, tvec, np.ascontiguousarray(K), dist)
    uv = uv.reshape(-1, 2)
    P_cam = (R @ P_lidar.T).T + t
    depth = P_cam[:, 2]
    return uv, depth, P_cam


def in_image_mask(uv, depth, width, height):
    valid = depth > 0.05
    valid &= uv[:, 0] >= 0
    valid &= uv[:, 0] < width
    valid &= uv[:, 1] >= 0
    valid &= uv[:, 1] < height
    return valid


def euler_to_R(rx_deg, ry_deg, rz_deg):
    r = np.deg2rad([rx_deg, ry_deg, rz_deg])
    return cv2.Rodrigues(r)[0]
