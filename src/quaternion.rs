/// A simple quaternion implementation for vector rotations
///
/// Author: Steven Michael (ssmichael@gmail.com)
/// Date: 2024-11-16
///

#[derive(Clone, Copy, Debug)]
pub struct Quaternion {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub w: f64,
}

use super::Matrix3;
use super::Vector3;

impl Quaternion {
    /// Create a new quaternion
    ///
    /// # Arguments
    /// * `x` - The x component
    /// * `y` - The y component
    /// * `z` - The z component
    /// * `w` - The scalar
    ///
    /// # Returns
    /// A new quaternion
    ///
    /// # Examples
    /// ```
    /// use satctrl::Quaternion;
    /// let q = Quaternion::new(0.0, 0.0, 0.0, 1.0);
    /// ```
    ///
    pub fn new(x: f64, y: f64, z: f64, w: f64) -> Self {
        Self { x, y, z, w }
    }

    /// Create a new quaternion from a direction cosine matrix
    ///
    /// # Arguments
    /// * `dcm` - The direction cosine matrix
    ///
    /// # Returns
    /// A new quaternion representing the same rotation as the DCM
    ///
    /// # Examples
    /// ```
    /// use satctrl::Quaternion;
    /// use satctrl::Matrix3;
    /// let dcm = Matrix3::identity();
    /// let q = Quaternion::from_dcm(&dcm);
    /// ```
    ///
    pub fn from_dcm(dcm: &Matrix3) -> Self {
        // Algortihm is from
        // https://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/index.htm
        let trace = dcm.trace();
        let mut q = Self::new(0.0, 0.0, 0.0, 0.0);
        if trace > 0.0 {
            let s = (trace + 1.0).sqrt() * 2.0;
            q.w = 0.25 * s;
            q.x = (dcm[(2, 1)] - dcm[(1, 2)]) / s;
            q.y = (dcm[(0, 2)] - dcm[(2, 0)]) / s;
            q.z = (dcm[(1, 0)] - dcm[(0, 1)]) / s;
        } else if dcm[(0, 0)] > dcm[(1, 1)] && dcm[(0, 0)] > dcm[(2, 2)] {
            let s = (1.0 + dcm[(0, 0)] - dcm[(1, 1)] - dcm[(2, 2)]).sqrt() * 2.0;
            q.w = (dcm[(2, 1)] - dcm[(1, 2)]) / s;
            q.x = 0.25 * s;
            q.y = (dcm[(0, 1)] + dcm[(1, 0)]) / s;
            q.z = (dcm[(0, 2)] + dcm[(2, 0)]) / s;
        } else if dcm[(1, 1)] > dcm[(2, 2)] {
            let s = (1.0 + dcm[(1, 1)] - dcm[(0, 0)] - dcm[(2, 2)]).sqrt() * 2.0;
            q.w = (dcm[(0, 2)] - dcm[(2, 0)]) / s;
            q.x = (dcm[(0, 1)] + dcm[(1, 0)]) / s;
            q.y = 0.25 * s;
            q.z = (dcm[(1, 2)] + dcm[(2, 1)]) / s;
        } else {
            let s = (1.0 + dcm[(2, 2)] - dcm[(0, 0)] - dcm[(1, 1)]).sqrt() * 2.0;
            q.w = (dcm[(1, 0)] - dcm[(0, 1)]) / s;
            q.x = (dcm[(0, 2)] + dcm[(2, 0)]) / s;
            q.y = (dcm[(1, 2)] + dcm[(2, 1)]) / s;
            q.z = 0.25 * s;
        }
        q
    }

    /// Convert the quaternion to a direction cosine matrix
    ///
    /// # Returns
    /// The direction cosine matrix
    ///
    /// # Examples
    ///
    /// ```
    /// use satctrl::Quaternion;
    /// use satctrl::Matrix3;
    /// let q = Quaternion::rotz(std::f64::consts::PI / 2.0);
    /// let dcm = q.as_dcm();
    /// ```
    ///
    pub fn as_dcm(&self) -> Matrix3 {
        let mut dcm = Matrix3::identity();
        let xx = self.x * self.x;
        let xy = self.x * self.y;
        let xz = self.x * self.z;
        let xw = self.x * self.w;
        let yy = self.y * self.y;
        let yz = self.y * self.z;
        let yw = self.y * self.w;
        let zz = self.z * self.z;
        let zw = self.z * self.w;
        dcm[(0, 0)] = 1.0 - 2.0 * (yy + zz);
        dcm[(0, 1)] = 2.0 * (xy - zw);
        dcm[(0, 2)] = 2.0 * (xz + yw);
        dcm[(1, 0)] = 2.0 * (xy + zw);
        dcm[(1, 1)] = 1.0 - 2.0 * (xx + zz);
        dcm[(1, 2)] = 2.0 * (yz - xw);
        dcm[(2, 0)] = 2.0 * (xz - yw);
        dcm[(2, 1)] = 2.0 * (yz + xw);
        dcm[(2, 2)] = 1.0 - 2.0 * (xx + yy);
        dcm
    }

    /// Create a new quaternion from an axis and angle
    ///
    /// # Arguments
    /// * `axis` - The axis of rotation
    /// * `angle` - The angle of rotation in radians
    ///
    /// # Returns
    /// A new quaternion representing the rotation
    ///
    /// # Examples
    /// ```
    /// use satctrl::Quaternion;
    /// use satctrl::Vector3;
    /// let q = Quaternion::from_axis_angle(&Vector3::zhat(), std::f64::consts::PI / 2.0);
    /// ```
    pub fn from_axis_angle(axis: &Vector3, angle: f64) -> Self {
        let half_angle = angle / 2.0;
        let s = half_angle.sin();
        Self::new(axis[0] * s, axis[1] * s, axis[2] * s, half_angle.cos())
    }

    /// Create a new quaternion representing no rotation
    ///
    /// # Returns
    /// A new quaternion representing no rotation
    ///
    pub fn identity() -> Self {
        Self::new(0.0, 0.0, 0.0, 1.0)
    }

    /// Normalize the quaternion
    ///
    /// # Examples
    /// ```
    /// use satctrl::Quaternion;
    /// let mut q = Quaternion::new(1.0, 2.0, 3.0, 4.0);
    /// q.normalize_inplace();
    /// assert!(q.norm() - 1.0 < f64::EPSILON);
    /// ```
    ///
    /// # Notes
    /// The quaternion is normalized in place
    ///
    pub fn normalize_inplace(&mut self) {
        let norm = (self.x * self.x + self.y * self.y + self.z * self.z + self.w * self.w).sqrt();
        if norm < f64::EPSILON {
            return;
        }
        self.x /= norm;
        self.y /= norm;
        self.z /= norm;
        self.w /= norm;
    }

    /// Quaternion to roll, pitch, yaw
    ///
    /// # Returns
    /// A tuple of roll, pitch, yaw in radians
    pub fn to_rpy(&self) -> (f64, f64, f64) {
        let r = self.x.atan2(self.w * self.z + self.x * self.y);
        let p = -self.y.asin();
        let y = self.x.atan2(self.w * self.y + self.z * self.x);
        (r, p, y)
    }

    /// Create a new quaternion from roll, pitch, yaw
    ///
    /// # Arguments
    /// * `roll` - The roll angle in radians
    /// * `pitch` - The pitch angle in radians
    /// * `yaw` - The yaw angle in radians
    ///
    /// # Returns
    /// A new quaternion representing the rotation
    ///
    /// # Notes
    /// * The order of rotation is roll, pitch, yaw
    /// * For a reference, see:
    ///   <https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles>
    pub fn from_rpy(roll: f64, pitch: f64, yaw: f64) -> Self {
        let half_roll = roll / 2.0;
        let half_pitch = pitch / 2.0;
        let half_yaw = yaw / 2.0;
        let cr = half_roll.cos();
        let sr = half_roll.sin();
        let cp = half_pitch.cos();
        let sp = half_pitch.sin();
        let cy = half_yaw.cos();
        let sy = half_yaw.sin();
        Self::new(
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        )
    }

    /// Quaternion norm
    ///
    /// # Returns
    /// The norm of the quaternion
    ///
    /// # Examples
    /// ```
    /// use satctrl::Quaternion;
    /// let q = Quaternion::new(1.0, 2.0, 3.0, 4.0);
    /// let norm = q.norm();
    /// ```
    ///
    pub fn norm(&self) -> f64 {
        (self.x * self.x + self.y * self.y + self.z * self.z + self.w * self.w).sqrt()
    }

    /// Sphereical linear interpolation between two quaternions
    ///
    /// # Arguments
    /// * `other` - The other quaternion
    /// * `t` - The interpolation distance from self to other, in range [0, 1]
    ///
    /// # Returns
    /// The interpolated quaternion
    ///
    /// # Examples
    ///
    /// ```
    /// use satctrl::Quaternion;
    /// let q1 = Quaternion::rotx(std::f64::consts::PI / 2.0);
    /// let q2 = Quaternion::roty(std::f64::consts::PI / 2.0);
    /// let q = q1.slerp(&q2, 0.5);
    /// ```
    ///
    pub fn slerp(&self, other: &Self, t: f64) -> Self {
        // See: https://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/slerp/index.htm
        let coshalftheta = self.dot(other);

        // Case where quaternions are the same
        if (1.0 - coshalftheta * coshalftheta).abs() < f64::EPSILON {
            return *self;
        }

        let sinhalftheta = (1.0 - coshalftheta * coshalftheta).sqrt();

        // Case where quaternions are 180 degrees apart
        if sinhalftheta.abs() < f64::EPSILON {
            return Self::new(
                (self.x + other.x) / 2.0,
                (self.y + other.y) / 2.0,
                (self.z + other.z) / 2.0,
                (self.w + other.w) / 2.0,
            );
        }
        let halftheta = coshalftheta.acos();
        let ratioa = (halftheta * (1.0 - t)).sin() / sinhalftheta;
        let ratiob = (halftheta * t).sin() / sinhalftheta;
        Self::new(
            self.x * ratioa + other.x * ratiob,
            self.y * ratioa + other.y * ratiob,
            self.z * ratioa + other.z * ratiob,
            self.w * ratioa + other.w * ratiob,
        )
    }

    /// Quaternion dot product
    ///
    /// # Arguments
    /// * `other` - The other quaternion
    ///
    /// # Returns
    /// The dot product of the two quaternions
    ///
    /// # Examples
    ///
    /// ```
    /// use satctrl::Quaternion;
    /// let q1 = Quaternion::new(1.0, 2.0, 3.0, 4.0);
    /// let q2 = Quaternion::new(1.0, 2.0, 3.0, 4.0);
    /// let dot = q1.dot(&q2);
    /// ```
    pub fn dot(&self, other: &Self) -> f64 {
        self.x * other.x + self.y * other.y + self.z * other.z + self.w * other.w
    }

    /// Create a new quaternion representing a rotation around the x axis
    ///
    /// # Arguments
    /// * `angle` - The angle of rotation in radians
    ///
    /// # Returns
    /// A new quaternion representing a rotation around the x axis
    ///
    /// # Examples
    /// ```
    /// use satctrl::Quaternion;
    /// let q = Quaternion::rotx(std::f64::consts::PI / 2.0);
    /// ```
    ///
    pub fn rotx(angle: f64) -> Self {
        let half_angle = angle / 2.0;
        Self::new(half_angle.sin(), 0.0, 0.0, half_angle.cos())
    }

    /// Create a new quaternion representing a rotation around the y axis
    ///
    /// # Arguments
    /// * `angle` - The angle of rotation in radians
    ///
    /// # Returns
    /// A new quaternion representing a rotation around the y axis
    ///
    /// # Examples
    /// ```
    /// use satctrl::Quaternion;
    /// let q = Quaternion::roty(std::f64::consts::PI / 2.0);
    /// ```
    ///
    pub fn roty(angle: f64) -> Self {
        let half_angle = angle / 2.0;
        Self::new(0.0, half_angle.sin(), 0.0, half_angle.cos())
    }

    /// Create a new quaternion representing a rotation around the z axis
    ///
    /// # Arguments
    /// * `angle` - The angle of rotation in radians
    ///
    /// # Returns
    /// A new quaternion representing a rotation around the z axis
    ///
    /// # Examples
    /// ```
    /// use satctrl::Quaternion;
    /// let q = Quaternion::rotz(std::f64::consts::PI / 2.0);
    /// ```
    ///
    pub fn rotz(angle: f64) -> Self {
        let half_angle = angle / 2.0;
        Self::new(0.0, 0.0, half_angle.sin(), half_angle.cos())
    }

    /// Quaternion conjugate
    ///
    /// # Returns
    /// The conjugate of the quaternion
    ///
    pub fn conjugate(&self) -> Self {
        Self::new(-self.x, -self.y, -self.z, self.w)
    }

    /// Axis of rotation
    ///
    /// # Returns
    /// The axis of rotation
    ///
    /// # Examples
    /// ```
    /// use satctrl::Quaternion;
    /// use satctrl::Vector3;
    /// let q = Quaternion::rotz(std::f64::consts::PI / 2.0);
    /// let axis = q.axis();
    /// assert_eq!(axis, Vector3::zhat());
    /// ```
    ///
    pub fn axis(&self) -> Vector3 {
        let s = (1.0 - self.w * self.w).sqrt();
        if s < 1.0e-6 {
            Vector3::from_vec([self.x, self.y, self.z])
        } else {
            Vector3::from_vec([self.x / s, self.y / s, self.z / s])
        }
    }

    /// Angle of rotation
    ///
    /// # Returns
    /// The angle of rotation in radians
    ///
    /// # Examples
    /// ```
    /// use satctrl::Quaternion;
    /// let q = Quaternion::rotz(std::f64::consts::PI / 3.0);
    /// let angle = q.angle();
    /// assert!(f64::abs(angle - std::f64::consts::PI / 3.0) < f64::EPSILON);
    /// ```
    ///
    pub fn angle(&self) -> f64 {
        2.0 * self.w.acos()
    }

    /// Quaternion derivative
    ///
    /// # Arguments
    /// * `omega` - The angular velocity vector, in rad/s
    ///
    /// # Returns
    /// The derivative of the quaternion
    ///
    /// # Examples
    ///
    /// ```
    /// use satctrl::Quaternion;
    /// use satctrl::Vector3;
    /// let q = Quaternion::rotz(std::f64::consts::PI / 2.0);
    /// let omega = Vector3::zhat();
    /// let q_dot = q.derivative(&omega);
    /// ```
    ///
    pub fn derivative(&self, omega: &Vector3) -> Self {
        let q = Self::new(omega[0], omega[1], omega[2], 0.0);
        *self * q * 0.5 + self * (1.0 - self.norm())
    }

    /// Integrate the quaternion using the Runge-Kutta 4th order method
    ///
    /// # Arguments
    /// * `omega` - The angular velocity vector, in rad/s
    /// * `dt` - The time step
    ///
    /// # Examples
    ///
    /// ```
    /// use satctrl::Quaternion;
    /// use satctrl::Vector3;
    /// let mut q = Quaternion::rotz(std::f64::consts::PI / 2.0);
    /// let omega = Vector3::zhat();
    /// q.integrate_inplace_rk4(&omega, 0.1);
    /// ```
    ///
    pub fn integrate_inplace_rk4(&mut self, omega: &Vector3, dt: f64) {
        let f = |_, q: &Self| q.derivative(omega);
        crate::ode::rk4_integrate_inplace(f, 0.0, self, dt);
        self.normalize_inplace();
    }

    /// Integrate the quaternion using the Euler method
    ///
    /// # Arguments
    /// * `omega` - The angular velocity vector, in rad/s
    /// * `dt` - The time step
    ///
    /// # Examples
    ///
    /// ```
    /// use satctrl::Quaternion;
    /// use satctrl::Vector3;
    /// let mut q = Quaternion::rotz(std::f64::consts::PI / 2.0);
    /// let omega = Vector3::zhat();
    /// q.integrate_inplace(&omega, 0.1);
    /// ```
    ///
    pub fn integrate_inplace(&mut self, omega: &Vector3, dt: f64) {
        let q_dot = self.derivative(omega);

        *self += q_dot * dt;
        self.normalize_inplace();
    }

    /// Angular distance between two quaternions
    ///
    /// # Arguments
    /// * `other` - The other quaternion
    ///
    /// # Returns
    /// The angular distance between the two quaternions
    ///
    /// # Examples
    ///
    /// ```
    /// use satctrl::Quaternion;
    /// let q1 = Quaternion::rotx(std::f64::consts::PI / 2.0);
    /// let q2 = Quaternion::roty(std::f64::consts::PI / 2.0);
    /// let angle = q1.angular_distance(&q2);
    /// ```
    ///
    pub fn angular_distance(&self, other: &Self) -> f64 {
        let qc = self.conjugate() * other;
        qc.angle()
    }
}

/// Quaternion multiplication by a scalar
///
/// # Arguments
/// * `rhs` - The scalar to multiply by
///
/// # Returns
/// The quaternion multiplied by the scalar
///
/// # Examples
///
/// ```
/// use satctrl::Quaternion;
/// let q = Quaternion::new(1.0, 2.0, 3.0, 4.0);
/// let q_scaled = q * 2.0;
/// assert_eq!(q_scaled, Quaternion::new(2.0, 4.0, 6.0, 8.0));
/// ```
impl std::ops::Mul<f64> for Quaternion {
    type Output = Self;

    fn mul(self, rhs: f64) -> Self {
        Self {
            x: self.x * rhs,
            y: self.y * rhs,
            z: self.z * rhs,
            w: self.w * rhs,
        }
    }
}

/// Quaternion reference multiplication by a scalar
///
/// # Arguments
/// * `rhs` - The scalar to multiply by
///
/// # Returns
/// The quaternion multiplied by the scalar
///
/// # Examples
///
/// ```
/// use satctrl::Quaternion;
/// let q = Quaternion::new(1.0, 2.0, 3.0, 4.0);
/// let q_scaled = &q * 2.0;
/// assert_eq!(q_scaled, Quaternion::new(2.0, 4.0, 6.0, 8.0));
/// ```
///
impl std::ops::Mul<f64> for &Quaternion {
    type Output = Quaternion;

    fn mul(self, rhs: f64) -> Quaternion {
        Quaternion {
            x: self.x * rhs,
            y: self.y * rhs,
            z: self.z * rhs,
            w: self.w * rhs,
        }
    }
}

/// Quaternion division by a scalar
///
/// # Arguments
/// * `rhs` - The scalar to divide by
///
/// # Returns
/// The quaternion divided by the scalar
///
/// # Examples
///
/// ```
/// use satctrl::Quaternion;
/// let q = Quaternion::new(2.0, 4.0, 6.0, 8.0);
/// let q_scaled = q / 2.0;
/// assert_eq!(q_scaled, Quaternion::new(1.0, 2.0, 3.0, 4.0));
/// ```
///
impl std::ops::Div<f64> for Quaternion {
    type Output = Self;

    fn div(self, rhs: f64) -> Self {
        Self {
            x: self.x / rhs,
            y: self.y / rhs,
            z: self.z / rhs,
            w: self.w / rhs,
        }
    }
}

/// Quaternion multiplication by another quaternion
///
/// # Arguments
/// * `rhs` - The quaternion to multiply by
///
/// # Returns
/// The quaternion product
///
/// # Examples
///
/// ```
/// use satctrl::Quaternion;
/// let q1 = Quaternion::rotx(std::f64::consts::PI / 2.0);
/// let q2 = Quaternion::roty(std::f64::consts::PI / 2.0);
/// let q = q1 * q2;
/// ```
///
impl std::ops::Mul<Self> for Quaternion {
    type Output = Self;

    fn mul(self, rhs: Self) -> Self {
        Self {
            x: self.w * rhs.x + self.x * rhs.w + self.y * rhs.z - self.z * rhs.y,
            y: self.w * rhs.y - self.x * rhs.z + self.y * rhs.w + self.z * rhs.x,
            z: self.w * rhs.z + self.x * rhs.y - self.y * rhs.x + self.z * rhs.w,
            w: self.w * rhs.w - self.x * rhs.x - self.y * rhs.y - self.z * rhs.z,
        }
    }
}

/// Quaternion multiplication by another quaternion
impl std::ops::Mul<&Self> for Quaternion {
    type Output = Self;

    fn mul(self, rhs: &Self) -> Self {
        Self {
            x: self.w * rhs.x + self.x * rhs.w + self.y * rhs.z - self.z * rhs.y,
            y: self.w * rhs.y - self.x * rhs.z + self.y * rhs.w + self.z * rhs.x,
            z: self.w * rhs.z + self.x * rhs.y - self.y * rhs.x + self.z * rhs.w,
            w: self.w * rhs.w - self.x * rhs.x - self.y * rhs.y - self.z * rhs.z,
        }
    }
}

/// Quaternion addition
impl std::ops::Add<Self> for Quaternion {
    type Output = Self;

    fn add(self, rhs: Self) -> Self {
        Self {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
            z: self.z + rhs.z,
            w: self.w + rhs.w,
        }
    }
}

/// Quaternion subtraction
impl std::ops::Sub<Self> for Quaternion {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self {
        Self {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
            z: self.z - rhs.z,
            w: self.w - rhs.w,
        }
    }
}

/// Quaternion subtraction of reference quaternion
impl std::ops::Sub<&Self> for Quaternion {
    type Output = Self;

    fn sub(self, rhs: &Self) -> Self {
        Self {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
            z: self.z - rhs.z,
            w: self.w - rhs.w,
        }
    }
}

/// Quaternion multiplication by vector representing a rotation
///
/// # Arguments
/// * `rhs` - The vector to rotate
///
/// # Returns
/// The rotated vector
///
/// # Examples
///
/// ```
/// use satctrl::Quaternion;
/// use satctrl::Vector3;
/// let q = Quaternion::rotz(std::f64::consts::PI / 2.0);
/// let v = Vector3::xhat();
/// let v_rot = q * v;
/// assert_eq!(v_rot, Vector3::yhat());
/// ```
///
impl std::ops::Mul<Vector3> for Quaternion {
    type Output = Vector3;

    fn mul(self, rhs: Vector3) -> Vector3 {
        let q = Self::new(rhs[0], rhs[1], rhs[2], 0.0);
        let q_conj = self.conjugate();
        let q_rot = self * q * q_conj;
        Vector3::from_vec([q_rot.x, q_rot.y, q_rot.z])
    }
}

impl std::ops::MulAssign<f64> for Quaternion {
    fn mul_assign(&mut self, rhs: f64) {
        *self = *self * rhs;
    }
}

/// Quaternion addition in-place
///
/// # Arguments
/// * `rhs` - The quaternion to add
///
/// # Examples
///
/// ```
/// use satctrl::Quaternion;
/// let mut q = Quaternion::new(1.0, 2.0, 3.0, 4.0);
/// q += Quaternion::new(1.0, 2.0, 3.0, 4.0);
/// ```
///
impl std::ops::AddAssign<Self> for Quaternion {
    fn add_assign(&mut self, rhs: Self) {
        self.x += rhs.x;
        self.y += rhs.y;
        self.z += rhs.z;
        self.w += rhs.w;
    }
}

/// Quaternion multiplication of reference vector to represent
/// rotation of vector by quaternion
impl std::ops::Mul<&Vector3> for Quaternion {
    type Output = Vector3;

    fn mul(self, rhs: &Vector3) -> Vector3 {
        let q = Self::new(rhs[0], rhs[1], rhs[2], 0.0);
        let q_conj = self.conjugate();
        let q_rot = self * q * q_conj;
        Vector3::from_vec([q_rot.x, q_rot.y, q_rot.z])
    }
}

/// Quaternion multiplication of vector to represent rotation of
/// vector by reference quaternion
impl std::ops::Mul<Vector3> for &Quaternion {
    type Output = Vector3;

    fn mul(self, rhs: Vector3) -> Vector3 {
        let q = Quaternion::new(rhs[0], rhs[1], rhs[2], 0.0);
        let q_conj = self.conjugate();
        let q_rot = *self * q * q_conj;
        Vector3::from_vec([q_rot.x, q_rot.y, q_rot.z])
    }
}

/// Quaternion multiplication of reference vector to represent
/// rotation of vector by reference quaternion
impl std::ops::Mul<&Vector3> for &Quaternion {
    type Output = Vector3;

    fn mul(self, rhs: &Vector3) -> Vector3 {
        let q = Quaternion::new(rhs[0], rhs[1], rhs[2], 0.0);
        let q_conj = self.conjugate();
        let q_rot = *self * q * q_conj;
        Vector3::from_vec([q_rot.x, q_rot.y, q_rot.z])
    }
}

/// Quaternion multiplication in place of other quaternion
///
/// # Arguments
/// * `rhs` - The quaternion to multiply by
///
/// # Examples
///
/// ```
/// use satctrl::Quaternion;
/// let mut q1 = Quaternion::rotx(std::f64::consts::PI / 2.0);
/// let q2 = Quaternion::roty(std::f64::consts::PI / 2.0);
/// q1 *= q2;
/// ```
///
impl std::ops::MulAssign<Self> for Quaternion {
    fn mul_assign(&mut self, rhs: Self) {
        *self = *self * rhs;
    }
}

/// Quaternion multiplication in place of reference quaternion
///
/// # Arguments
/// * `rhs` - The quaternion to multiply by
///
/// # Examples
///
/// ```
/// use satctrl::Quaternion;
/// let mut q1 = Quaternion::rotx(std::f64::consts::PI / 2.0);
/// let q2 = Quaternion::roty(std::f64::consts::PI / 2.0);
/// q1 *= &q2;
/// ```
impl std::ops::MulAssign<&Self> for Quaternion {
    fn mul_assign(&mut self, rhs: &Self) {
        *self = *self * rhs;
    }
}

/// Test that quaternion represents identical rotations
/// Allowing for sign ambiguities (i.e. q and -q represent the same rotation)
impl PartialEq for Quaternion {
    fn eq(&self, other: &Self) -> bool {
        const TOL: f64 = f64::EPSILON * 5.0;
        if other.w * self.w < 0.0 {
            let other = other * -1.0;
            (self.x - other.x).abs() < TOL
                && (self.y - other.y).abs() < TOL
                && (self.z - other.z).abs() < TOL
                && (self.w - other.w).abs() < TOL
        } else {
            (self.x - other.x).abs() < f64::EPSILON * 5.0
                && (self.y - other.y).abs() < TOL
                && (self.z - other.z).abs() < TOL
                && (self.w - other.w).abs() < TOL
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rand::distributions::Distribution;

    #[test]
    fn test_new() {
        let q = Quaternion::new(1.0, 2.0, 3.0, 4.0);
        assert_eq!(q.x, 1.0);
        assert_eq!(q.y, 2.0);
        assert_eq!(q.z, 3.0);
        assert_eq!(q.w, 4.0);
    }

    #[test]
    /// Quaternion is as expected
    fn test_id() {
        let q = Quaternion::identity();
        assert_eq!(q.x, 0.0);
        assert_eq!(q.y, 0.0);
        assert_eq!(q.z, 0.0);
        assert_eq!(q.w, 1.0);
    }

    /// One test of quaternion correctness:
    /// the quaternion multiplication is associative
    #[test]
    fn test_associative() {
        let q1 = Quaternion::rotx(std::f64::consts::PI / 2.0);
        let q2 = Quaternion::roty(std::f64::consts::PI / 2.0);
        let q3 = Quaternion::rotz(std::f64::consts::PI / 2.0);
        let q = q1 * q2 * q3;
        let q12 = q1 * q2;
        let q123 = q12 * q3;
        assert_eq!(q, q123);
    }

    /// Test that conversion to and from direction-cosine matrices (dcm)
    /// from quaternions is able to produce an identical rotation
    /// of a 3-vector
    #[test]
    fn test_dcm() {
        let q = Quaternion::rotz(std::f64::consts::PI / 2.0);
        let dcm = q.as_dcm();
        let dcm_expected =
            Matrix3::from_row_major_array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]);
        assert_eq!(dcm, dcm_expected);

        let interval = rand::distributions::Uniform::from(-1.0..1.0);
        let mut rng = rand::thread_rng();
        for _ in 0..100 {
            // Create a random quaternion
            let mut q = Quaternion::new(
                interval.sample(&mut rng),
                interval.sample(&mut rng),
                interval.sample(&mut rng),
                interval.sample(&mut rng),
            );
            q.normalize_inplace();

            // Represent the quaternion as a dcm
            let dcm = q.as_dcm();

            // Create a random vector
            let v = Vector3::from_slice(&[
                interval.sample(&mut rng),
                interval.sample(&mut rng),
                interval.sample(&mut rng),
            ]);

            // Re-create quaternion from the created DCM
            let q2 = Quaternion::from_dcm(&dcm);

            // Rotate the vectors
            let v_rot = dcm * v;
            let v_rot_q = q * v;
            let v_rot_q2 = q2 * v;

            // Check for equality
            assert!(v_rot == v_rot_q);
            assert!(v_rot == v_rot_q2);
        }
    }

    #[test]
    fn test_conjugate() {
        let q = Quaternion::new(1.0, 2.0, 3.0, 4.0);
        let q_conj = q.conjugate();
        assert_eq!(q_conj.x, -1.0);
        assert_eq!(q_conj.y, -2.0);
        assert_eq!(q_conj.z, -3.0);
        assert_eq!(q_conj.w, 4.0);
    }

    #[test]
    fn test_slerp() {
        let q1 = Quaternion::rotz(std::f64::consts::PI / 2.0);
        let q2 = Quaternion::rotz(std::f64::consts::PI / 4.0);
        let q = q1.slerp(&q2, 0.5);
        let q_expected = Quaternion::rotz(std::f64::consts::PI * 3.0 / 8.0);
        assert_eq!(q, q_expected);
    }

    /// Test that quatenrion rotations about axes are all right-handed
    #[test]
    fn test_righthanded_rotation() {
        let q = Quaternion::rotx(std::f64::consts::PI / 2.0);
        let zhat = q * Vector3::yhat();
        assert_eq!(zhat, Vector3::zhat());

        let q = Quaternion::rotz(std::f64::consts::PI / 2.0);
        let yhat = q * Vector3::xhat();
        assert_eq!(yhat, Vector3::yhat());

        let q = Quaternion::roty(std::f64::consts::PI / 2.0);
        let xhat = q * Vector3::zhat();
        assert_eq!(xhat, Vector3::xhat());
    }
}
