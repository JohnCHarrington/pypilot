//! Three-element vectors, ported from pypilot/vector.py.

use core::ops::{Add, Mul, Neg, Sub};

/// A 3-vector of `f32`.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Vec3 {
    /// X component.
    pub x: f32,
    /// Y component.
    pub y: f32,
    /// Z component.
    pub z: f32,
}

impl Vec3 {
    /// The zero vector.
    pub const ZERO: Vec3 = Vec3::new(0.0, 0.0, 0.0);

    /// Build a vector from components.
    pub const fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }

    /// Build from an array, in pypilot's `[x, y, z]` order.
    pub const fn from_array(a: [f32; 3]) -> Self {
        Self::new(a[0], a[1], a[2])
    }

    /// The components as an array.
    pub const fn to_array(self) -> [f32; 3] {
        [self.x, self.y, self.z]
    }

    /// Dot product.
    pub fn dot(self, b: Vec3) -> f32 {
        self.x * b.x + self.y * b.y + self.z * b.z
    }

    /// Cross product.
    pub fn cross(self, b: Vec3) -> Vec3 {
        Vec3::new(
            self.y * b.z - self.z * b.y,
            self.z * b.x - self.x * b.z,
            self.x * b.y - self.y * b.x,
        )
    }

    /// Euclidean length.
    pub fn norm(self) -> f32 {
        libm::sqrtf(self.dot(self))
    }

    /// Unit vector in the same direction; the zero vector stays zero, as in
    /// pypilot.
    pub fn normalize(self) -> Vec3 {
        let n = self.norm();
        if n == 0.0 { self } else { self * (1.0 / n) }
    }

    /// Projection of `self` onto `b`.
    pub fn project(self, b: Vec3) -> Vec3 {
        b * (self.dot(b) / b.dot(b))
    }

    /// Squared distance to `b`.
    pub fn dist2(self, b: Vec3) -> f32 {
        let d = self - b;
        d.dot(d)
    }

    /// Distance to `b`.
    pub fn dist(self, b: Vec3) -> f32 {
        (self - b).norm()
    }
}

impl Add for Vec3 {
    type Output = Vec3;
    fn add(self, b: Vec3) -> Vec3 {
        Vec3::new(self.x + b.x, self.y + b.y, self.z + b.z)
    }
}

impl Sub for Vec3 {
    type Output = Vec3;
    fn sub(self, b: Vec3) -> Vec3 {
        Vec3::new(self.x - b.x, self.y - b.y, self.z - b.z)
    }
}

impl Mul<f32> for Vec3 {
    type Output = Vec3;
    fn mul(self, m: f32) -> Vec3 {
        Vec3::new(self.x * m, self.y * m, self.z * m)
    }
}

impl Neg for Vec3 {
    type Output = Vec3;
    fn neg(self) -> Vec3 {
        Vec3::new(-self.x, -self.y, -self.z)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn basics() {
        let a = Vec3::new(1.0, 0.0, 0.0);
        let b = Vec3::new(0.0, 1.0, 0.0);
        assert_eq!(a.cross(b), Vec3::new(0.0, 0.0, 1.0));
        assert_eq!(a.dot(b), 0.0);
        assert_eq!(Vec3::new(3.0, 4.0, 0.0).norm(), 5.0);
        assert_eq!(Vec3::ZERO.normalize(), Vec3::ZERO);
        assert_eq!(
            Vec3::new(2.0, 2.0, 0.0).project(a),
            Vec3::new(2.0, 0.0, 0.0)
        );
    }
}
