use crate::Result;
use thiserror::Error;

#[derive(Error, Debug)]
pub enum MathError {
    #[error("Matrix is singular")]
    SingularMatrix,
    #[error("Matrix is not square")]
    NotSquareMatrix,
    #[error("Matrix is not invertible")]
    NotInvertibleMatrix,
    #[error("Matrix is not positive definite")]
    NotPositiveDefiniteMatrix,
    #[error("Matrix is not positive semi-definite")]
    NotPositiveSemiDefiniteMatrix,
    #[error("Invalid Index: {0}, {1}")]
    InvalidIndex(usize, usize),
}

impl<T> From<MathError> for Result<T> {
    fn from(err: MathError) -> Self {
        Err(err)
    }
}
