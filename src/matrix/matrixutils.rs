use super::matrixbase::Matrix;
use crate::matherr::MathError;

use crate::Result;

/// Cholesky decomposition
/// Takes the input positive-definite square matrix `a` and returns the lower triangular matrix `l`
/// representing the Cholesky decomposition of `a`.
///
/// # Arguments
/// * `a` - The input square matrix
///
/// # Returns
/// The lower triangular matrix `l`
///
/// # Example
/// ```
/// use satctrl::matrixutils::cholesky_decomp;
/// use satctrl::Matrix3;
/// let a = Matrix3::from_row_major_array([[25.0, 15.0, -5.0], [15.0, 18.0, 0.0], [-5.0, 0.0, 11.0]]);
/// let l = cholesky_decomp(&a);
/// ```
///
pub fn cholesky_decomp<const N: usize>(a: &Matrix<N, N>) -> Result<Matrix<N, N>> {
    let mut l = Matrix::<N, N>::zeros();
    for i in 0..N {
        for j in 0..=i {
            let mut sum = a[(i, j)];
            for k in 0..j {
                sum -= l[(i, k)] * l[(j, k)];
            }
            if i == j {
                if sum <= 0.0 {
                    return MathError::NotPositiveDefiniteMatrix.into();
                }
                l[(i, j)] = sum.sqrt();
            } else {
                l[(i, j)] = sum / l[(j, j)];
            }
        }
    }
    Ok(l)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Matrix3;

    #[test]
    fn test_cholesky_decomp() {
        let a = Matrix3::from_row_major_array([
            [25.0, 15.0, -5.0],
            [15.0, 18.0, 0.0],
            [-5.0, 0.0, 11.0],
        ]);
        let l = match cholesky_decomp(&a) {
            Ok(l) => l,
            Err(_) => panic!("Cholesky decomposition failed"),
        };
        let l_expected =
            Matrix3::from_row_major_array([[5.0, 0.0, 0.0], [3.0, 3.0, 0.0], [-1.0, 1.0, 3.0]]);
        assert_eq!(l, l_expected);
        assert_eq!(a, l * l.transpose());
    }
}
