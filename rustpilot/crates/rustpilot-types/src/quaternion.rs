//! Quaternions, ported from pypilot/quaternion.py. Stored as `[w, x, y, z]`,
//! pypilot's order.

use crate::vector::Vec3;

/// A rotation quaternion `w + xi + yj + zk`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Quaternion {
    /// Scalar part.
    pub w: f32,
    /// i component.
    pub x: f32,
    /// j component.
    pub y: f32,
    /// k component.
    pub z: f32,
}

impl Default for Quaternion {
    fn default() -> Self {
        Self::IDENTITY
    }
}

impl Quaternion {
    /// No rotation.
    pub const IDENTITY: Quaternion = Quaternion::new(1.0, 0.0, 0.0, 0.0);

    /// Build from components.
    pub const fn new(w: f32, x: f32, y: f32, z: f32) -> Self {
        Self { w, x, y, z }
    }

    /// Build from pypilot's `[w, x, y, z]` list.
    pub const fn from_array(q: [f32; 4]) -> Self {
        Self::new(q[0], q[1], q[2], q[3])
    }

    /// Components in pypilot's order.
    pub const fn to_array(self) -> [f32; 4] {
        [self.w, self.x, self.y, self.z]
    }

    /// Rotation of `angle` radians about axis `v` (`angvec2quat`).
    pub fn from_angle_axis(angle: f32, v: Vec3) -> Self {
        let n = v.norm();
        let fac = if n == 0.0 {
            0.0
        } else {
            libm::sinf(angle / 2.0) / n
        };
        Self::new(libm::cosf(angle / 2.0), v.x * fac, v.y * fac, v.z * fac)
    }

    /// Shortest rotation taking direction `a` to direction `b` (`vec2vec2quat`).
    pub fn from_two_vectors(a: Vec3, b: Vec3) -> Self {
        let n = a.cross(b);
        let fac = (a.dot(b) / a.norm() / b.norm()).clamp(-1.0, 1.0);
        Self::from_angle_axis(libm::acosf(fac), n)
    }

    /// Rotation angle in radians (`angle`).
    pub fn angle(self) -> f32 {
        2.0 * libm::acosf(self.w.clamp(-1.0, 1.0))
    }

    /// Hamilton product `self * q2` (`multiply`).
    pub fn multiply(self, q2: Quaternion) -> Quaternion {
        let q1 = self;
        Quaternion::new(
            q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z,
            q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y,
            q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x,
            q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w,
        )
    }

    /// Conjugate (inverse for a unit quaternion).
    pub fn conjugate(self) -> Quaternion {
        Quaternion::new(self.w, -self.x, -self.y, -self.z)
    }

    /// Scale to unit length.
    pub fn normalize(self) -> Quaternion {
        let d = libm::sqrtf(self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z);
        Quaternion::new(self.w / d, self.x / d, self.y / d, self.z / d)
    }

    /// Rotate a vector by this quaternion (`rotvecquat`).
    pub fn rotate(self, v: Vec3) -> Vec3 {
        let w = Quaternion::new(0.0, v.x, v.y, v.z);
        let r = self.multiply(w).multiply(self.conjugate());
        Vec3::new(r.x, r.y, r.z)
    }

    /// `(roll, pitch, heading)` in radians (`toeuler`).
    pub fn to_euler(self) -> (f32, f32, f32) {
        let q = self;
        let roll = libm::atan2f(
            2.0 * (q.y * q.z + q.w * q.x),
            1.0 - 2.0 * (q.x * q.x + q.y * q.y),
        );
        let pitch = libm::asinf((2.0 * (q.w * q.y - q.x * q.z)).clamp(-1.0, 1.0));
        let heading = libm::atan2f(
            2.0 * (q.x * q.y + q.w * q.z),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        );
        (roll, pitch, heading)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::f32::consts::FRAC_PI_2;

    fn close(a: f32, b: f32) -> bool {
        (a - b).abs() < 1e-5
    }

    #[test]
    fn rotate_about_z() {
        let q = Quaternion::from_angle_axis(FRAC_PI_2, Vec3::new(0.0, 0.0, 1.0));
        let v = q.rotate(Vec3::new(1.0, 0.0, 0.0));
        assert!(close(v.x, 0.0) && close(v.y, 1.0) && close(v.z, 0.0));
        let (roll, pitch, heading) = q.to_euler();
        assert!(close(roll, 0.0) && close(pitch, 0.0) && close(heading, FRAC_PI_2));
        assert!(close(q.angle(), FRAC_PI_2));
    }

    #[test]
    fn two_vectors() {
        let a = Vec3::new(0.0, 0.0, 1.0);
        let b = Vec3::new(1.0, 0.0, 0.0);
        let q = Quaternion::from_two_vectors(a, b);
        let r = q.rotate(a);
        assert!(close(r.x, 1.0) && close(r.y, 0.0) && close(r.z, 0.0));
    }
}
