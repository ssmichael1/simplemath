mod matherr;
mod matrix;
mod quaternion;

pub mod filters;
pub mod ode;

#[cfg(test)]
mod tests;

// The SCResult type is a Result type that returns a Box<dyn std::error::Error> on error
pub type Result<T> = std::result::Result<T, matherr::MathError>;

pub use matherr::MathError;

pub use matrix::matrixutils;
pub use matrix::Matrix;
pub use matrix::Vector;

pub use quaternion::Quaternion;

/// Some common vector types
pub type Vector6 = Vector<6>;
pub type Vector5 = Vector<5>;
pub type Vector4 = Vector<4>;
pub type Vector3 = Vector<3>;
pub type Vector2 = Vector<2>;
pub type Vector1 = Vector<1>;

/// Some common matrix types
pub type Matrix2 = Matrix<2, 2>;
pub type Matrix3 = Matrix<3, 3>;
pub type Matrix4 = Matrix<4, 4>;
pub type Matrix5 = Matrix<5, 5>;
pub type Matrix6 = Matrix<6, 6>;
