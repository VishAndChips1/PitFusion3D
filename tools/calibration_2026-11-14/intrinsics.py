"""RGB intrinsics from the 5 checkerboard bags (4x6 inner corners, 58 mm squares), with stability checks."""
import numpy as np, cv2, json, itertools
D = '/home/vish/checkerboard_analysis/data'; W, H = 1280, 720; SQ = 0.058
NAMES = ['close', 'far', 'left', 'right', 'centre']
obj = np.zeros((24, 3), np.float32)
obj[:, 0] = np.tile(np.arange(4), 6) * SQ; obj[:, 1] = np.repeat(np.arange(6), 4) * SQ
C = {n: np.load(f'{D}/{n}_corners.npy').astype(np.float32) for n in NAMES}


def views(names, step=10):
    """Every `step`-th frame of each bag (the board is static, so more frames add noise averaging, not new geometry)."""
    return [(n, C[n][i]) for n in names for i in range(0, len(C[n]), step)]


def calibrate(names, flags, step=10, k_init=1065.0, dist_n=5):
    v = views(names, step)
    K0 = np.array([[k_init, 0, W / 2], [0, k_init, H / 2], [0, 0, 1.0]])
    rms, K, d, rv, tv = cv2.calibrateCamera([obj] * len(v), [c.reshape(-1, 1, 2) for _, c in v], (W, H), K0.copy(),
                                            np.zeros(dist_n), flags=flags)
    return rms, K, d.ravel(), rv, tv, v


def per_view_error(K, d, rv, tv, v):
    out = []
    for (n, c), r, t in zip(v, rv, tv):
        p, _ = cv2.projectPoints(obj, r, t, K, d)
        out.append(float(np.sqrt(((p.reshape(-1, 2) - c) ** 2).sum(1).mean())))
    return out


F = cv2.CALIB_USE_INTRINSIC_GUESS
CONFIGS = {
    'A: f,cx,cy free + k1,k2 (aspect fixed, no tangential, k3 off)': F | cv2.CALIB_FIX_ASPECT_RATIO | cv2.CALIB_ZERO_TANGENT_DIST | cv2.CALIB_FIX_K3,
    'B: f free, centre fixed, k1,k2': F | cv2.CALIB_FIX_ASPECT_RATIO | cv2.CALIB_ZERO_TANGENT_DIST | cv2.CALIB_FIX_K3 | cv2.CALIB_FIX_PRINCIPAL_POINT,
    'C: f free, centre fixed, k1 only': F | cv2.CALIB_FIX_ASPECT_RATIO | cv2.CALIB_ZERO_TANGENT_DIST | cv2.CALIB_FIX_K3 | cv2.CALIB_FIX_K2 | cv2.CALIB_FIX_PRINCIPAL_POINT,
    'D: nothing fixed but k3 (fx,fy,cx,cy,k1,k2,p1,p2)': F | cv2.CALIB_FIX_K3,
}
if __name__ == '__main__':
    res = {}
    for label, fl in CONFIGS.items():
        rms, K, d, rv, tv, v = calibrate(NAMES, fl)
        pv = per_view_error(K, d, rv, tv, v)
        loo = []
        for leave in NAMES:
            r2, K2, d2, *_ = calibrate([n for n in NAMES if n != leave], fl)
            loo.append((leave, round(K2[0, 0], 1), round(K2[1, 2], 1)))
        print(f'\n{label}\n  rms {rms:.3f}px  fx {K[0,0]:.1f} fy {K[1,1]:.1f} cx {K[0,2]:.1f} cy {K[1,2]:.1f}  dist {np.round(d,4)}'
              f'\n  worst view {max(pv):.2f}px  leave-one-bag-out fx: {[l[1] for l in loo]}')
        res[label] = dict(rms=rms, K=K.tolist(), dist=d.tolist(), loo_fx=[l[1] for l in loo])
    json.dump(res, open(f'{D}/intrinsics_variants.json', 'w'), indent=1)
