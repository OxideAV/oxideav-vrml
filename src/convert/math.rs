//! Small vector / rotation helpers shared by the converters.

pub(crate) type V3 = [f32; 3];

pub(crate) fn add(a: V3, b: V3) -> V3 {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

pub(crate) fn sub(a: V3, b: V3) -> V3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

pub(crate) fn scale(a: V3, s: f32) -> V3 {
    [a[0] * s, a[1] * s, a[2] * s]
}

pub(crate) fn dot(a: V3, b: V3) -> f32 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

pub(crate) fn cross(a: V3, b: V3) -> V3 {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

pub(crate) fn len(a: V3) -> f32 {
    dot(a, a).sqrt()
}

/// Normalise; returns `None` for (near) zero or non-finite vectors.
pub(crate) fn normalize(a: V3) -> Option<V3> {
    let l = len(a);
    if l.is_finite() && l > 1e-20 {
        Some(scale(a, 1.0 / l))
    } else {
        None
    }
}

/// VRML axis–angle (`x y z angle`) → unit quaternion (xyzw). A zero
/// axis yields the identity.
pub(crate) fn axis_angle_to_quat(r: [f32; 4]) -> [f32; 4] {
    let Some(axis) = normalize([r[0], r[1], r[2]]) else {
        return [0.0, 0.0, 0.0, 1.0];
    };
    if !r[3].is_finite() {
        return [0.0, 0.0, 0.0, 1.0];
    }
    let (s, c) = (r[3] * 0.5).sin_cos();
    [axis[0] * s, axis[1] * s, axis[2] * s, c]
}

/// Unit quaternion (xyzw) → VRML axis–angle with angle in `[0, π]`.
pub(crate) fn quat_to_axis_angle(q: [f32; 4]) -> [f32; 4] {
    let n = (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]).sqrt();
    if !n.is_finite() || n < 1e-20 {
        return [0.0, 0.0, 1.0, 0.0];
    }
    let mut q = [q[0] / n, q[1] / n, q[2] / n, q[3] / n];
    if q[3] < 0.0 {
        q = [-q[0], -q[1], -q[2], -q[3]];
    }
    let s = (1.0 - q[3] * q[3]).max(0.0).sqrt();
    if s < 1e-7 {
        return [0.0, 0.0, 1.0, 0.0];
    }
    let angle = 2.0 * q[3].clamp(-1.0, 1.0).acos();
    let axis = normalize([q[0] / s, q[1] / s, q[2] / s]).unwrap_or([0.0, 0.0, 1.0]);
    // Snap axis components that are within rounding of ±1 / 0 so
    // principal-axis rotations print exactly.
    let axis = axis.map(|c| {
        if (c.abs() - 1.0).abs() < 1e-6 {
            c.signum()
        } else if c.abs() < 1e-7 {
            0.0
        } else {
            c
        }
    });
    [axis[0], axis[1], axis[2], angle]
}

/// Rotate `v` by unit quaternion `q`.
pub(crate) fn quat_rotate(q: [f32; 4], v: V3) -> V3 {
    let u = [q[0], q[1], q[2]];
    let w = q[3];
    let t = scale(cross(u, v), 2.0);
    add(add(v, scale(t, w)), cross(u, t))
}

/// Shortest-arc quaternion rotating unit vector `from` onto `to`.
pub(crate) fn quat_between(from: V3, to: V3) -> [f32; 4] {
    let (Some(f), Some(t)) = (normalize(from), normalize(to)) else {
        return [0.0, 0.0, 0.0, 1.0];
    };
    let d = dot(f, t);
    if d > 1.0 - 1e-7 {
        return [0.0, 0.0, 0.0, 1.0];
    }
    if d < -1.0 + 1e-7 {
        // 180°: any axis perpendicular to `from`.
        let alt = if f[0].abs() < 0.9 {
            [1.0, 0.0, 0.0]
        } else {
            [0.0, 1.0, 0.0]
        };
        let axis = normalize(cross(f, alt)).unwrap_or([0.0, 0.0, 1.0]);
        return [axis[0], axis[1], axis[2], 0.0];
    }
    let c = cross(f, t);
    let w = 1.0 + d;
    let n = (c[0] * c[0] + c[1] * c[1] + c[2] * c[2] + w * w).sqrt();
    [c[0] / n, c[1] / n, c[2] / n, w / n]
}

/// 4×4 row-major column-vector matrix (glTF layout used by mesh3d).
pub(crate) type M4 = [[f32; 4]; 4];

pub(crate) fn m4_identity() -> M4 {
    [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]
}

pub(crate) fn m4_mul(a: &M4, b: &M4) -> M4 {
    let mut out = [[0.0f32; 4]; 4];
    for (r, row) in out.iter_mut().enumerate() {
        for (c, cell) in row.iter_mut().enumerate() {
            *cell = (0..4).map(|k| a[r][k] * b[k][c]).sum();
        }
    }
    out
}

pub(crate) fn m4_translation(t: V3) -> M4 {
    let mut m = m4_identity();
    m[0][3] = t[0];
    m[1][3] = t[1];
    m[2][3] = t[2];
    m
}

pub(crate) fn m4_scale(s: V3) -> M4 {
    let mut m = m4_identity();
    m[0][0] = s[0];
    m[1][1] = s[1];
    m[2][2] = s[2];
    m
}

/// Rotation matrix of a VRML axis–angle (ISO/IEC 14772-1 §5.8).
pub(crate) fn m4_rotation(r: [f32; 4]) -> M4 {
    let q = axis_angle_to_quat(r);
    let (x, y, z, w) = (q[0], q[1], q[2], q[3]);
    let mut m = m4_identity();
    m[0][0] = 1.0 - 2.0 * (y * y + z * z);
    m[0][1] = 2.0 * (x * y - z * w);
    m[0][2] = 2.0 * (x * z + y * w);
    m[1][0] = 2.0 * (x * y + z * w);
    m[1][1] = 1.0 - 2.0 * (x * x + z * z);
    m[1][2] = 2.0 * (y * z - x * w);
    m[2][0] = 2.0 * (x * z - y * w);
    m[2][1] = 2.0 * (y * z + x * w);
    m[2][2] = 1.0 - 2.0 * (x * x + y * y);
    m
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: &[f32], b: &[f32]) -> bool {
        a.iter().zip(b).all(|(x, y)| (x - y).abs() < 1e-5)
    }

    #[test]
    fn axis_angle_round_trip() {
        let r = [0.0, 1.0, 0.0, 1.2];
        let q = axis_angle_to_quat(r);
        assert!(close(&quat_to_axis_angle(q), &r));
        let v = quat_rotate(q, [0.0, 0.0, -1.0]);
        let m = m4_rotation(r);
        let mv = [-m[0][2], -m[1][2], -m[2][2]];
        assert!(close(&v, &mv));
    }

    #[test]
    fn between() {
        let q = quat_between([0.0, 0.0, -1.0], [1.0, 0.0, 0.0]);
        assert!(close(&quat_rotate(q, [0.0, 0.0, -1.0]), &[1.0, 0.0, 0.0]));
        let q = quat_between([0.0, 0.0, -1.0], [0.0, 0.0, 1.0]);
        assert!(close(&quat_rotate(q, [0.0, 0.0, -1.0]), &[0.0, 0.0, 1.0]));
    }
}
