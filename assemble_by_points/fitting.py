"""Axis-constrained fitting of two (scanned) meshes.

The target (active object) stays fixed. The user faces the parts the way
they join and marks three or more *matching pairs*: a point on each part
that touch when assembled. The pairs define the frame of the fit:

* the centre of each part's matching points is its reference point (they
  coincide once assembled),
* the *axis of assembly* is the joining direction: the rotation axis of the
  turn that maps the moving part's points onto the target's (when it is a
  real turn), otherwise the normal of the plane through the target's points.

The moving object may spin freely about the axis (within the spin range),
tilt only slightly away from it, and slide a little sideways.

Both surfaces are turned into height fields over the plane perpendicular to
the axis by casting rays along it: the target's surface that faces the
moving object and the moving object's surface that faces the target. For a
candidate pose the gap (positive) or overlap (negative) is then known at
every grid cell, and moving along the axis just shifts all gaps equally. The
axial position (and small tilt, which to first order adds a plane to the
heights) is chosen to minimise the average absolute gap/overlap, and that
minimum average is the score used to compare poses. Scan meshes never match
exactly, so no vertex or face correspondence is assumed anywhere.

Optional *matching pairs* (a point on each object that the user says touch
when assembled) add a second term to the score: how far apart each pair
ends up, beyond a small click tolerance. They steer the coarse search, which
matters for hollow parts whose thin rims barely register on a coarse grid,
while the surfaces still decide the final seating.

The search runs coarse to fine: every spin and sideways shift on a coarse
grid, then the best few candidates on successively finer grids, and finally
the winner is polished by re-casting the moving object's rays at its exact
tilted orientation.
"""

from dataclasses import dataclass, replace
import math

import numpy as np
from mathutils.bvhtree import BVHTree

LEVELS = (24, 48, 96, 192)       # grid cells across the moving footprint, per level
LATERAL_RANGE = 0.3              # sideways search radius, fraction of footprint radius
TOUCH_CELLS = 4.0                # gaps wider than this many cells count as not touching
OVERLAP_CAP = 10.0               # overlap keeps costing up to this many times as much:
                                 # solids can't pass through each other (hollow parts!)
PLANE_TILT = math.radians(5)     # tilt one height field can absorb as a fitted plane
RING_STEP = math.radians(10)     # spacing of explicitly cast tilts beyond that
KEEP = 6                         # candidates carried from one level to the next
CHUNK = 2_000_000                # max candidate x cell elements evaluated at once
POLISH_ROUNDS = 30
PAIR_WEIGHT = 1.0                # pair distance vs surface gap in the score
PAIR_TOLERANCE = 0.02            # pair slack (click accuracy), fraction of footprint radius


# --------------------------------------------------------------------------
# Geometry helpers
# --------------------------------------------------------------------------

def rodrigues(w):
    theta = float(np.linalg.norm(w))
    if theta < 1e-15:
        return np.eye(3)
    k = w / theta
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(theta) * K + (1 - math.cos(theta)) * (K @ K)


def rotation_between(p, q):
    """Smallest rotation taking unit vector p onto unit vector q."""
    axis = np.cross(p, q)
    s, c = float(np.linalg.norm(axis)), float(np.dot(p, q))
    if s < 1e-12:
        return np.eye(3)            # only used for small tilts, never ~180 degrees
    return rodrigues(axis / s * math.atan2(s, c))


def tilt_angle(R, a):
    return math.acos(max(-1.0, min(1.0, float((R @ a) @ a))))


def clamp_tilt(R, a, limit):
    """Limit how far R tips the axis ``a``, keeping its spin component."""
    t = tilt_angle(R, a)
    if t <= limit:
        return R
    tip = rotation_between(a, R @ a)
    axis = np.cross(a, R @ a)
    axis /= np.linalg.norm(axis)
    return rodrigues(axis * limit) @ tip.T @ R


def plane_to_rotation(slope_u, slope_v, u, v):
    """Rotation that adds ``slope_u*x + slope_v*y`` to heights along a = u x v."""
    w = slope_v * u - slope_u * v
    s = math.hypot(slope_u, slope_v)
    if s < 1e-15:
        return np.eye(3)
    return rodrigues(w / s * math.atan(s))


# --------------------------------------------------------------------------
# Height fields
# --------------------------------------------------------------------------

class Caster:
    """Ray casts against one object's evaluated mesh (modifiers included)."""

    def __init__(self, obj, depsgraph):
        self.name = obj.name
        self.bvh = BVHTree.FromObject(obj, depsgraph)
        self.mw = np.array(obj.matrix_world, dtype=np.float64)
        self.inv = np.linalg.inv(self.mw)
        eval_obj = obj.evaluated_get(depsgraph)
        mesh = eval_obj.to_mesh()
        try:
            co = np.empty(len(mesh.vertices) * 3)
            mesh.vertices.foreach_get("co", co)
            has_faces = len(mesh.polygons) > 0
        finally:
            eval_obj.to_mesh_clear()
        if not has_faces:
            raise ValueError(f"'{obj.name}' has no faces")
        self.verts = co.reshape(-1, 3) @ self.mw[:3, :3].T + self.mw[:3, 3]

    def heights(self, center, basis, half, n, toward):
        """Height field on an n x n grid of cells (size 2*half/n) centred on
        ``center`` in the plane spanned by basis[0], basis[1].

        Rays travel along ``toward * basis[2]`` (-1: down the axis, seeing the
        surface that faces +axis). Heights are measured along basis[2] from
        ``center``; cells whose ray misses are NaN.
        """
        u, v, a = (np.asarray(b, dtype=np.float64) for b in basis)
        h = 2 * half / n
        c = -half + (np.arange(n) + 0.5) * h
        gx, gy = np.meshgrid(c, c, indexing="ij")
        far = float(np.linalg.norm(self.verts - center, axis=1).max()) * 1.01 + h
        start = (center + gx[..., None] * u + gy[..., None] * v - toward * far * a).reshape(-1, 3)

        local = start @ self.inv[:3, :3].T + self.inv[:3, 3]
        d = self.inv[:3, :3] @ (toward * a)
        d = (d / np.linalg.norm(d)).tolist()
        hits = np.full((len(local), 3), np.nan)
        cast = self.bvh.ray_cast
        for i, o in enumerate(local.tolist()):
            loc = cast(o, d)[0]
            if loc is not None:
                hits[i] = loc
        world = hits @ self.mw[:3, :3].T + self.mw[:3, 3]
        return Field(((world - center) @ a).reshape(n, n), half, h)


@dataclass
class Field:
    H: np.ndarray        # (n, n) heights, H[i, j] at x_i, y_j; NaN = no surface
    half: float
    h: float

    @property
    def count(self):
        return int(np.isfinite(self.H).sum())

    def cells(self):
        """(points (P, 2), heights (P,)) of the cells that hit a surface."""
        n = self.H.shape[0]
        c = -self.half + (np.arange(n) + 0.5) * self.h
        gx, gy = np.meshgrid(c, c, indexing="ij")
        ok = np.isfinite(self.H)
        return np.stack([gx[ok], gy[ok]], -1), self.H[ok]

    def sample(self, qx, qy):
        """Bilinear lookup; NaN outside the grid or next to a missing cell."""
        n = self.H.shape[0]
        fx = (qx + self.half) / self.h - 0.5
        fy = (qy + self.half) / self.h - 0.5
        ix, iy = np.floor(fx), np.floor(fy)
        tx, ty = fx - ix, fy - iy
        inside = (ix >= 0) & (ix < n - 1) & (iy >= 0) & (iy < n - 1)
        ix = np.clip(ix, 0, n - 2).astype(np.intp)
        iy = np.clip(iy, 0, n - 2).astype(np.intp)
        H = self.H
        val = (H[ix, iy] * (1 - tx) * (1 - ty) + H[ix + 1, iy] * tx * (1 - ty)
               + H[ix, iy + 1] * (1 - tx) * ty + H[ix + 1, iy + 1] * tx * ty)
        return np.where(inside, val, np.nan)


# --------------------------------------------------------------------------
# Scoring
# --------------------------------------------------------------------------

def _cost(r, touch):
    """Per-cell cost of residual r (> 0 = overlap, < 0 = gap).

    Gaps are capped at ``touch``: a wide gap is legitimate, just "not
    touching". Overlap is capped much higher, because real solids can't
    interpenetrate; otherwise e.g. the inside of a hollow part could "touch"
    the inside of the other while their rims pass straight through each other.
    """
    return np.where(r > 0, np.minimum(r, OVERLAP_CAP * touch), np.minimum(-r, touch))


def _fit_planes(y, qx, qy, mask, touch, slope_max, iters=6):
    """Per row, fit y ~ c + a*qx + b*qy minimising the mean truncated |residual|
    (see ``_cost``) by IRLS, with the slope magnitude clamped to ``slope_max``.

    Starts from first contact (bringing the part in until the surfaces just
    meet, ignoring the highest 2% as scan junk) so it approaches from outside.
    """
    eps = 0.0125 * touch
    count = mask.sum(1)
    ranked = np.sort(np.where(mask, y, -np.inf), axis=1)       # missing cells first
    pick = np.clip(y.shape[1] - 1 - (0.02 * count).astype(np.intp), 0, y.shape[1] - 1)
    c = np.take_along_axis(ranked, pick[:, None], 1)[:, 0]
    c = np.where(count > 0, c, 0.0)
    a = np.zeros_like(c)
    b = np.zeros_like(c)
    y = np.where(mask, y, 0.0)
    qx = np.where(mask, qx, 0.0)
    qy = np.where(mask, qy, 0.0)
    m = mask.astype(np.float64)
    for _ in range(iters):
        r = y - c[:, None] - a[:, None] * qx - b[:, None] * qy
        ar = np.abs(r)
        active = np.where(r > 0, ar < OVERLAP_CAP * touch, ar < touch)
        w = m * np.where(active, 1.0 / np.maximum(ar, eps), 1e-6 / touch)
        sw, sx, sy = w.sum(1), (w * qx).sum(1), (w * qy).sum(1)
        sxx, sxy, syy = (w * qx * qx).sum(1), (w * qx * qy).sum(1), (w * qy * qy).sum(1)
        ridge = 1e-9 * (sxx + syy) + 1e-30
        A = np.stack([np.stack([sw + 1e-30, sx, sy], -1),
                      np.stack([sx, sxx + ridge, sxy], -1),
                      np.stack([sy, sxy, syy + ridge], -1)], 1)
        rhs = np.stack([(w * y).sum(1), (w * y * qx).sum(1), (w * y * qy).sum(1)], -1)
        c, a, b = np.linalg.solve(A, rhs[..., None])[..., 0].T
        s = np.hypot(a, b)
        f = np.minimum(1.0, slope_max / np.maximum(s, 1e-30))
        a, b = a * f, b * f
        c = (w * (y - a[:, None] * qx - b[:, None] * qy)).sum(1) / np.maximum(sw, 1e-30)
    r = y - c[:, None] - a[:, None] * qx - b[:, None] * qy
    return c, a, b, r


@dataclass
class Candidate:
    tilt: int            # index into the explicit tilt list
    theta: float         # spin about the axis (radians)
    d: tuple             # sideways shift (u, v)
    score: float = math.inf
    c: float = 0.0       # axial position of the moving origin relative to the target's
    a: float = 0.0       # residual tilt, as height slopes along u and v
    b: float = 0.0


@dataclass
class PairTerms:
    """Matching pairs expressed in the frames used by one moving height field."""
    q: np.ndarray        # (n, 2) moving points across the axis, before spin
    h: np.ndarray        # (n,) moving points along the axis
    p: np.ndarray        # (n, 2) target points across the axis
    H: np.ndarray        # (n,) target points along the axis
    tol: float

    @classmethod
    def build(cls, pairs, R0, o_m, o_t, basis, tol):
        if not pairs:
            return None
        u, v, a = basis
        w = (np.array([m for m, _t in pairs]) - o_m) @ R0.T
        rp = np.array([t for _m, t in pairs]) - o_t
        return cls(np.stack([w @ u, w @ v], -1), w @ a,
                   np.stack([rp @ u, rp @ v], -1), rp @ a, tol)

    def cost(self, cs, sn, d, c, a, b):
        """Mean pair distance beyond the tolerance, per candidate."""
        qx, qy = self.q[None, :, 0], self.q[None, :, 1]
        lx = cs * qx - sn * qy + d[:, 0, None] - self.p[None, :, 0]
        ly = sn * qx + cs * qy + d[:, 1, None] - self.p[None, :, 1]
        lz = c[:, None] + a[:, None] * qx + b[:, None] * qy + self.h[None] - self.H[None]
        dist = np.sqrt(lx * lx + ly * ly + lz * lz)
        return np.maximum(dist - self.tol, 0.0).mean(1)


def _evaluate(field, tgt_pts, tgt_h, cands, slope_max, touch, nref, pairs=None,
              stats=False):
    """Score candidates that share one moving height field (same tilt)."""
    th = np.array([cd.theta for cd in cands])
    d = np.array([cd.d for cd in cands], dtype=np.float64)
    step = max(1, CHUNK // max(len(tgt_h), 1))
    out = []
    for s0 in range(0, len(cands), step):
        cs, sn = np.cos(th[s0:s0 + step])[:, None], np.sin(th[s0:s0 + step])[:, None]
        rx = tgt_pts[None, :, 0] - d[s0:s0 + step, 0, None]
        ry = tgt_pts[None, :, 1] - d[s0:s0 + step, 1, None]
        qx, qy = cs * rx + sn * ry, -sn * rx + cs * ry        # rotate by -theta
        hm = field.sample(qx, qy)
        mask = np.isfinite(hm)
        c, a, b, r = _fit_planes(tgt_h[None, :] - hm, qx, qy, mask, touch, slope_max)
        count = mask.sum(1)
        cost = np.where(mask, _cost(r, touch), 0.0).sum(1)
        cost += np.clip(nref - count, 0, None) * touch       # footprint not covered
        score = cost / nref
        if pairs is not None:
            score = score + PAIR_WEIGHT * pairs.cost(cs, sn, d[s0:s0 + step], c, a, b)
        for k, cd in enumerate(cands[s0:s0 + step]):
            out.append(replace(cd, score=float(score[k]), c=float(c[k]),
                               a=float(a[k]), b=float(b[k])))
        if stats:
            gap = -r[0][mask[0]]                              # + gap, - overlap
            return out, gap, touch
    return out


# --------------------------------------------------------------------------
# Search
# --------------------------------------------------------------------------

@dataclass
class FitResult:
    matrix: np.ndarray       # 4x4 world-space delta to apply to the moving object
    mean_gap: float          # average |gap or overlap| where the surfaces touch
    contact: float           # fraction of the smaller footprint that touches
    overlap_share: float     # fraction of the touching area that overlaps
    spin: float              # radians turned about the axis
    tilt: float              # radians tipped away from the axis
    shift: float             # sideways slide
    travel: float            # distance moved along the axis
    pair_distances: list     # final distance between each matching pair
    axis: np.ndarray         # axis of assembly used (world, unit, target -> moving)


def _tilt_candidates(u, v, limit):
    """Explicit orientations to cast; each one covers +-PLANE_TILT around itself."""
    rings = []
    if limit > PLANE_TILT + 1e-9:
        rings = list(np.arange(2 * PLANE_TILT, limit + 1e-9, RING_STEP))
        if limit - (rings[-1] if rings else 0.0) > PLANE_TILT:
            rings.append(limit)
    out = [(0.0, np.eye(3))]
    for t in rings:
        m = max(6, math.ceil(2 * math.pi * t / RING_STEP))
        for k in range(m):
            phi = 2 * math.pi * k / m
            out.append((t, rodrigues((math.cos(phi) * u + math.sin(phi) * v) * t)))
    return out


def _wrap(theta, spin_range):
    if spin_range >= 2 * math.pi - 1e-6:
        return (theta + math.pi) % (2 * math.pi) - math.pi
    half = spin_range / 2
    return min(max(theta, -half), half)


def _distinct(cands, keep, dtheta, h):
    out = []
    for cd in sorted(cands, key=lambda c: c.score):
        if all(not (o.tilt == cd.tilt
                    and abs((o.theta - cd.theta + math.pi) % (2 * math.pi) - math.pi) < 1.5 * dtheta
                    and math.dist(o.d, cd.d) < 1.5 * h) for o in out):
            out.append(cd)
            if len(out) == keep:
                break
    return out


MIN_PAIRS = 3
SPIN_AXIS_ANGLE = math.radians(20)   # turns larger than this define the axis themselves


def _bbox_center(obj):
    mw = np.array(obj.matrix_world, dtype=np.float64)
    corners = np.array([c[:] for c in obj.bound_box], dtype=np.float64)
    return (corners @ mw[:3, :3].T + mw[:3, 3]).mean(0)


def assembly_frame(pairs, target, moving):
    """(target reference point, moving reference point, axis) from matching pairs.

    ``pairs`` is a list of (point on moving, point on target), world space.
    The axis points from the target toward the moving part.
    """
    if len(pairs) < MIN_PAIRS:
        raise ValueError(f"Create at least {MIN_PAIRS} matching points between the parts "
                         f"({len(pairs)} so far)")
    pm = np.array([m for m, _t in pairs], dtype=np.float64)
    pt = np.array([t for _m, t in pairs], dtype=np.float64)
    o_m, o_t = pm.mean(0), pt.mean(0)

    # Plane through the target's points: its normal is the fallback axis.
    _u, sv, vt = np.linalg.svd(pt - o_t)
    if sv[1] < 0.1 * sv[0]:
        raise ValueError(f"The matching points on '{target.name}' lie almost in a line; "
                         "spread them out around the joint")
    normal = vt[2]

    # The turn taking the moving part's points onto the target's (Kabsch).
    # If it's a real turn, its axis is the joining direction itself, so the
    # fit spins about exactly the right line and needs no extra tilt.
    H = (pm - o_m).T @ (pt - o_t)
    U, _s, Wt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Wt.T @ U.T)) or 1.0
    R = Wt.T @ np.diag([1.0, 1.0, d]) @ U.T
    angle = math.acos(max(-1.0, min(1.0, (np.trace(R) - 1) / 2)))
    a = normal
    if angle > SPIN_AXIS_ANGLE:
        axis = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
        if np.linalg.norm(axis) > 1e-9:
            a = axis / np.linalg.norm(axis)

    # Point it from the target toward the moving part: away from the target's
    # bulk, toward the moving part's (as they're laid out now).
    side = (_bbox_center(moving) - o_t) @ a - (_bbox_center(target) - o_t) @ a
    if side < 0:
        a = -a
    return o_t, o_m, a


def fit(target, moving, depsgraph, tilt_limit, spin_range, pairs=(), progress=None):
    """Find the pose of ``moving`` that best mates with ``target``.

    ``pairs`` is a list of (point on moving, point on target) in world space;
    at least three are needed.
    """
    o_t, o_m, a = assembly_frame(pairs, target, moving)
    dist = float(np.linalg.norm(o_m - o_t))
    u = np.cross(a, [1.0, 0, 0] if abs(a[0]) < 0.9 else [0, 1.0, 0])
    u /= np.linalg.norm(u)
    v = np.cross(a, u)                          # (u, v, a) is right-handed

    tgt_caster = Caster(target, depsgraph)
    mov_caster = Caster(moving, depsgraph)
    rel = mov_caster.verts - o_m
    radius = float(np.linalg.norm(rel - np.outer(rel @ a, a), axis=1).max())
    if radius <= 0:
        raise ValueError(f"'{moving.name}' has no extent across the axis")
    lateral = LATERAL_RANGE * radius
    tilts = _tilt_candidates(u, v, tilt_limit)

    def slope_max(t0):
        return math.tan(min(max(tilt_limit - t0, 0.0), PLANE_TILT))

    pairs = [(np.asarray(m, dtype=np.float64), np.asarray(t, dtype=np.float64))
             for m, t in pairs]

    def pair_terms(R0):
        return PairTerms.build(pairs, R0, o_m, o_t, (u, v, a), PAIR_TOLERANCE * radius)

    def moving_field(R0, n):
        basis = (R0.T @ u, R0.T @ v, R0.T @ a)
        return mov_caster.heights(o_m, basis, radius, n, +1)

    total_steps = len(LEVELS) + 3
    cands = []
    for li, n in enumerate(LEVELS):
        h = 2 * radius / n
        k = math.ceil(lateral / h)
        tgt_field = tgt_caster.heights(o_t, (u, v, a), radius + k * h, n + 2 * k, -1)
        tgt_pts, tgt_h = tgt_field.cells()
        if len(tgt_h) == 0:
            raise ValueError(f"'{target.name}' has no surface facing '{moving.name}' "
                             "along the axis of assembly")
        touch = TOUCH_CELLS * h
        dtheta = h / radius                         # one cell of travel at the rim

        if li == 0:
            if spin_range >= 2 * math.pi - 1e-6:
                m = math.ceil(2 * math.pi / dtheta)
                thetas = -math.pi + 2 * math.pi * np.arange(m) / m
            else:
                m = math.ceil(spin_range / dtheta) + 1
                thetas = np.linspace(-spin_range / 2, spin_range / 2, max(m, 1))
            r = int(lateral // h)
            shifts = [(i * h, j * h) for i in range(-r, r + 1) for j in range(-r, r + 1)
                      if (i * i + j * j) * h * h <= lateral * lateral + 1e-12]
            todo = [Candidate(ti, float(t), s) for ti in range(len(tilts))
                    for t in thetas for s in shifts]
        else:
            seen, todo = set(), []
            for cd in cands:
                for dt in (-dtheta, 0.0, dtheta):
                    for i in (-1, 0, 1):
                        for j in (-1, 0, 1):
                            d = (cd.d[0] + i * h, cd.d[1] + j * h)
                            if math.hypot(*d) > lateral + h:
                                continue
                            t = _wrap(cd.theta + dt, spin_range)
                            key = (cd.tilt, round(t / dtheta * 4), round(d[0] / h * 4),
                                   round(d[1] / h * 4))
                            if key not in seen:
                                seen.add(key)
                                todo.append(Candidate(cd.tilt, t, d))

        scored = []
        for ti in sorted({cd.tilt for cd in todo}):
            field = moving_field(tilts[ti][1], n)
            nref = max(min(field.count, len(tgt_h)), 1)
            group = [cd for cd in todo if cd.tilt == ti]
            scored += _evaluate(field, tgt_pts, tgt_h, group, slope_max(tilts[ti][0]),
                                touch, nref, pair_terms(tilts[ti][1]))
        cands = _distinct(scored, KEEP if li < len(LEVELS) - 1 else 1, dtheta, h)
        if progress:
            progress((li + 1) / total_steps)

    # Polish: re-cast the moving object at its exact tilt so the plane
    # approximation no longer matters, then pattern-search spin and shift:
    # keep stepping while it improves, halve the step when it doesn't.
    best = cands[0]
    R0 = tilts[best.tilt][1]
    step_t, step_d = dtheta, h
    for rnd in range(POLISH_ROUNDS):
        if rnd == 0 or math.atan(math.hypot(best.a, best.b)) > math.radians(0.02):
            R0 = clamp_tilt(plane_to_rotation(best.a, best.b, u, v) @ R0, a, tilt_limit)
            field = moving_field(R0, LEVELS[-1])
            nref = max(min(field.count, len(tgt_h)), 1)
            sm = slope_max(tilt_angle(R0, a))
            pt = pair_terms(R0)
            best = _evaluate(field, tgt_pts, tgt_h, [replace(best, tilt=0)], sm, touch, nref,
                             pt)[0]
        todo = [Candidate(0, _wrap(best.theta + i * step_t, spin_range),
                          (best.d[0] + j * step_d, best.d[1] + k * step_d))
                for i in (-1, 0, 1) for j in (-1, 0, 1) for k in (-1, 0, 1)
                if i or j or k]
        found = min(_evaluate(field, tgt_pts, tgt_h, todo, sm, touch, nref, pt),
                    key=lambda c: c.score)
        if found.score < best.score:
            best = found
        else:
            step_t, step_d = step_t / 2, step_d / 2
            if step_d < h / 8:
                break
        if progress:
            progress(min(len(LEVELS) + 1 + rnd / 4, total_steps - 1) / total_steps)

    (best,), gap, touch = _evaluate(field, tgt_pts, tgt_h, [best], sm, touch, nref, pt,
                                    stats=True)
    R0 = clamp_tilt(plane_to_rotation(best.a, best.b, u, v) @ R0, a, tilt_limit)
    R = rodrigues(a * best.theta) @ R0
    new_origin = o_t + best.d[0] * u + best.d[1] * v + best.c * a

    M = np.eye(4)
    M[:3, :3] = R
    M[:3, 3] = new_origin - R @ o_m
    touching = np.abs(gap) < touch
    pair_distances = [float(np.linalg.norm(R @ (m - o_m) + new_origin - t)) for m, t in pairs]
    return FitResult(
        matrix=M,
        mean_gap=float(np.abs(gap[touching]).mean()) if touching.any() else math.inf,
        contact=float(touching.sum()) / nref,
        overlap_share=float((gap[touching] < 0).mean()) if touching.any() else 0.0,
        spin=best.theta,
        tilt=tilt_angle(R0, a),
        shift=math.hypot(*best.d),
        travel=dist - best.c,
        pair_distances=pair_distances,
        axis=a,
    )
