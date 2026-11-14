"""Detect the 4x6 inner corners of the checkerboard in every RGB frame.

The board's light squares are dim in the camera image, so each frame is contrast-boosted (CLAHE) first.
Strategies are tried in order: whole frame, then a crop around the dark-square blob with a bright border.
"""
import numpy as np, cv2, os
D = os.path.expanduser('~/checkerboard_analysis/data'); PAT = (4, 6)
SB = cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_ACCURACY


def refine(g, c):
    return cv2.cornerSubPix(g, c.reshape(-1, 1, 2).astype(np.float32), (5, 5), (-1, -1),
                            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 1e-4)).reshape(-1, 2)


def detect(bgr, roi=None):
    g = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    ge = cv2.createCLAHE(3.0, (8, 8)).apply(g)
    ok, c = cv2.findChessboardCornersSB(ge, PAT, SB)
    if ok:
        return c.reshape(-1, 2), 'full-SB'
    # crop around the largest dark compact region found by thresholding
    for x0, y0, x1, y1 in ([roi] if roi else candidate_boxes(g)):
        crop = cv2.createCLAHE(3.0, (4, 4)).apply(g[y0:y1, x0:x1])
        for pad in (20, 15, 25):
            cp = cv2.copyMakeBorder(crop, pad, pad, pad, pad, cv2.BORDER_CONSTANT, value=int(np.percentile(crop, 98)))
            ok, c = cv2.findChessboardCornersSB(cp, PAT, SB)
            if not ok:
                ok, c = cv2.findChessboardCorners(cp, PAT, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
                                                  + cv2.CALIB_CB_FILTER_QUADS)
            if ok:
                c = c.reshape(-1, 2) + np.array([x0 - pad, y0 - pad], np.float32)
                return refine(g, c), ('crop', (x0, y0, x1, y1))
    return None, None


def candidate_boxes(g):
    """Sliding windows of plausible board sizes (board is 5x7 squares, so about 0.71 wide:high), coarse to fine."""
    boxes = []
    h_img, w_img = g.shape
    for h in (180, 230, 290, 350, 420):
        w = int(h * 0.74)
        for y in range(0, h_img - h + 1, max(20, h // 6)):
            for x in range(0, w_img - w + 1, max(20, w // 6)):
                boxes.append((x, y, x + w, y + h))
    return boxes


def orient(c):
    """Order corners row-major from the top-left of the image, so index 0 is always top-left."""
    c = c.reshape(6, 4, 2) if abs(c[1, 0] - c[0, 0]) > abs(c[1, 1] - c[0, 1]) else c.reshape(4, 6, 2).transpose(1, 0, 2)
    if c[0, 0, 1] > c[-1, 0, 1]:
        c = c[::-1]
    if c[0, 0, 0] > c[0, -1, 0]:
        c = c[:, ::-1]
    return c.reshape(-1, 2)


def grow(c, shape, f=0.3):
    x0, y0 = c.min(axis=0); x1, y1 = c.max(axis=0); w, h = x1 - x0, y1 - y0
    # inner corners span 3/5 x 5/7 of the board; add a full square margin each side
    return (max(0, int(x0 - w * f)), max(0, int(y0 - h * f)), min(shape[1], int(x1 + w * f)), min(shape[0], int(y1 + h * f)))


if __name__ == '__main__':
    for name in ['close', 'far', 'left', 'right', 'centre']:
        z = np.load(f'{D}/{name}.npz', allow_pickle=True); imgs = z['imgs']
        out, how, roi = [], [], None
        for bgr in imgs:
            c, h = detect(bgr, roi)
            if c is None and roi is not None:
                c, h = detect(bgr)
            if c is not None and roi is None:
                roi = grow(c, bgr.shape)
            out.append(orient(c) if c is not None else np.full((24, 2), np.nan)); how.append(h[0] if h else None)
        n = sum(h is not None for h in how)
        print(f'{name}: {n}/{len(imgs)} detected', {k: how.count(k) for k in set(how)}, 'roi', roi, flush=True)
        np.save(f'{D}/{name}_corners.npy', np.array(out))
        i = next((k for k, h in enumerate(how) if h), None)
        if i is not None:
            v = imgs[i].copy(); cv2.drawChessboardCorners(v, PAT, out[i].reshape(-1, 1, 2).astype(np.float32), True)
            cv2.imwrite(f'{D}/{name}/detected.png', v)
