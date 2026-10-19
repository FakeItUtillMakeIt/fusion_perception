import numpy as np
import yaml

from .ade20k import ID2LABEL as ADE20K

UNKNOWN = 255

_DEFAULT_COLORS = {
    'passable': (0, 200, 0),
    'soft': (0, 220, 220),
    'hard': (0, 0, 230),
    'transparent': (255, 0, 255),
    'unknown': (128, 128, 128),
}


class Policy:
    def __init__(self, path, id2label=None):
        with open(path) as f:
            cfg = yaml.safe_load(f)
        if id2label is None:
            id2label = ADE20K
        self.id2label = {int(k): str(v).lower() for k, v in id2label.items()}
        self.group_order = list(cfg['groups'].keys())
        self.colors = {}
        self.drop_groups = set(cfg.get('costmap_drop_groups') or [])
        self.unknown_color = tuple(cfg.get('unknown', {}).get('color', _DEFAULT_COLORS['unknown']))

        name2group = {}
        self.group_lut = np.full(256, -1, dtype=np.int8)
        self.group_index = {g: i for i, g in enumerate(self.group_order)}
        for g in self.group_order:
            spec = cfg['groups'][g] or {}
            self.colors[g] = tuple(spec.get('color', _DEFAULT_COLORS.get(g, (200, 200, 200))))
            for cls in spec.get('classes') or []:
                if isinstance(cls, int):
                    self.group_lut[cls] = self.group_index[g]
                else:
                    name2group[str(cls).strip().lower()] = g

        for lid, cname in self.id2label.items():
            g = name2group.get(cname)
            if g is not None and 0 <= lid < 256:
                self.group_lut[lid] = self.group_index[g]

    def group_ids(self):
        return self.group_order

    def classes_of_group(self, group):
        idx = self.group_index[group]
        return sorted(lid for lid in range(256) if self.group_lut[lid] == idx)

    def drop_mask(self, labels):
        labels = np.asarray(labels).ravel()
        gi = self.group_lut[labels]
        drop_idx = {self.group_index[g] for g in self.drop_groups if g in self.group_index}
        mask = np.zeros(labels.shape, dtype=bool)
        for i in drop_idx:
            mask |= (gi == i)
        return mask

    def bgr_of_labels(self, labels):
        labels = np.asarray(labels)
        flat = labels.ravel().astype(np.int32)
        gi = self.group_lut[flat]
        out = np.empty((flat.size, 3), dtype=np.uint8)
        out[:] = self.unknown_color
        for g, i in self.group_index.items():
            m = gi == i
            out[m] = self.colors[g]
        return out.reshape(labels.shape + (3,))
