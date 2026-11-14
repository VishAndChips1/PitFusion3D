"""Final checkerboard detector: tight per-bag region of interest, CLAHE contrast boost, bright border, sub-pixel refinement.

The regions were read off a frame of each bag (the board is held still for the 5 s).
"""
import numpy as np, cv2
from detect import refine, orient, D, PAT
ROIS = {'close': [(676, 130, 886, 416), (686, 115, 878, 433)], 'far': [(638, 236, 756, 397), (641, 228, 751, 404)],
        'left': [(30, 171, 208, 440)], 'right': [(1052, 75, 1280, 415), (1030, 60, 1280, 440)],
        'centre': [(500, 209, 645, 410)]}


def detect_in(g, roi):
    x0, y0, x1, y1 = roi
    for clip in (2.5, 2.0, 3.0):
        c = cv2.createCLAHE(clip, (2, 2)).apply(g[y0:y1, x0:x1])
        for pad in (10, 20):
            cp = cv2.copyMakeBorder(c, pad, pad, pad, pad, cv2.BORDER_CONSTANT, value=255)
            ok, cor = cv2.findChessboardCornersSB(cp, PAT, cv2.CALIB_CB_EXHAUSTIVE + cv2.CALIB_CB_NORMALIZE_IMAGE)
            if not ok:
                ok, cor = cv2.findChessboardCorners(cp, PAT, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)
            if ok:
                return refine(g, cor.reshape(-1, 2) + np.array([x0 - pad, y0 - pad], np.float32))
    return None


def detect_frame(bgr, name):
    g = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    for roi in ROIS[name]:
        c = detect_in(g, roi)
        if c is not None:
            return orient(c)
    return None


if __name__ == '__main__':
    for name in ROIS:
        imgs = np.load(f'{D}/{name}.npz', allow_pickle=True)['imgs']
        out = [detect_frame(b, name) for b in imgs]
        print(name, f'{sum(o is not None for o in out)}/{len(imgs)}', flush=True)
        np.save(f'{D}/{name}_corners.npy', np.array([o if o is not None else np.full((24, 2), np.nan) for o in out]))
        i = next(k for k, o in enumerate(out) if o is not None)
        v = imgs[i].copy(); cv2.drawChessboardCorners(v, PAT, out[i].reshape(-1, 1, 2).astype(np.float32), True)
        cv2.imwrite(f'{D}/{name}/detected.png', v)
