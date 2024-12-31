use super::Matrix;
use std::cmp::PartialEq;

/// Implementations for equality comparison
///
/// # Example
/// ```
/// use satctrl::Matrix;
/// let m1 = Matrix::<3, 3>::identity();
/// let m2 = Matrix::<3, 3>::identity();
/// assert_eq!(m1, m2);
/// ```
///
/// # Returns
/// True if the two matrices are equal, false otherwise
impl<const M: usize, const N: usize> PartialEq for Matrix<M, N> {
    fn eq(&self, other: &Self) -> bool {
        for i in 0..M {
            for j in 0..N {
                // Give a little cushion for floating point comparison
                // to account for compounding numeric errors
                if (self.data[j][i] - other.data[j][i]).abs() > f64::EPSILON * 5.0 {
                    return false;
                }
            }
        }
        true
    }
}

/// Multiply matrix by a scalar
impl<const M: usize, const N: usize> std::ops::Mul<f64> for Matrix<M, N> {
    type Output = Self;

    fn mul(self, rhs: f64) -> Self::Output {
        Self {
            data: {
                let mut data = [[0.0; M]; N];
                for (i, row) in data.iter_mut().enumerate() {
                    for (j, value) in row.iter_mut().enumerate() {
                        *value = self.data[i][j] * rhs;
                    }
                }
                data
            },
        }
    }
}

/// Multiply reference matrix by a scalar
impl<const M: usize, const N: usize> std::ops::Mul<f64> for &Matrix<M, N> {
    type Output = Matrix<M, N>;

    fn mul(self, rhs: f64) -> Self::Output {
        Matrix {
            data: {
                let mut data = [[0.0; M]; N];
                for (i, row) in data.iter_mut().enumerate() {
                    for (j, value) in row.iter_mut().enumerate() {
                        *value = self.data[i][j] * rhs;
                    }
                }
                data
            },
        }
    }
}

/// Left-multiply scalar by a matrix
impl<const M: usize, const N: usize> std::ops::Mul<Matrix<M, N>> for f64 {
    type Output = Matrix<M, N>;

    fn mul(self, rhs: Matrix<M, N>) -> Self::Output {
        Matrix {
            data: {
                let mut data = [[0.0; M]; N];
                for (i, row) in data.iter_mut().enumerate() {
                    for (j, value) in row.iter_mut().enumerate() {
                        *value = rhs.data[i][j] * self;
                    }
                }
                data
            },
        }
    }
}

/// Left-multiply scalar by reference matrix
impl<const M: usize, const N: usize> std::ops::Mul<&Matrix<M, N>> for f64 {
    type Output = Matrix<M, N>;

    fn mul(self, rhs: &Matrix<M, N>) -> Self::Output {
        Matrix {
            data: {
                let mut data = [[0.0; M]; N];
                for (i, row) in data.iter_mut().enumerate() {
                    for (j, value) in row.iter_mut().enumerate() {
                        *value = rhs.data[i][j] * self;
                    }
                }
                data
            },
        }
    }
}

/// Divide matrix by a scalar
impl<const M: usize, const N: usize> std::ops::Div<f64> for Matrix<M, N> {
    type Output = Self;

    fn div(self, rhs: f64) -> Self::Output {
        Self {
            data: {
                let mut data = [[0.0; M]; N];
                for (i, row) in data.iter_mut().enumerate() {
                    for (j, value) in row.iter_mut().enumerate() {
                        *value = self.data[i][j] / rhs;
                    }
                }
                data
            },
        }
    }
}

/// Add a scalar to a matrix
impl<const M: usize, const N: usize> std::ops::Add<f64> for Matrix<M, N> {
    type Output = Self;

    fn add(self, rhs: f64) -> Self::Output {
        Self {
            data: {
                let mut data = [[0.0; M]; N];
                for (i, row) in data.iter_mut().enumerate() {
                    for (j, value) in row.iter_mut().enumerate() {
                        *value = self.data[i][j] + rhs;
                    }
                }
                data
            },
        }
    }
}

impl<const M: usize, const N: usize> std::ops::AddAssign<Self> for Matrix<M, N> {
    fn add_assign(&mut self, rhs: Self) {
        for i in 0..N {
            for j in 0..M {
                self.data[i][j] += rhs.data[i][j];
            }
        }
    }
}

impl<const M: usize, const N: usize> std::ops::SubAssign<Self> for Matrix<M, N> {
    fn sub_assign(&mut self, rhs: Self) {
        for i in 0..N {
            for j in 0..M {
                self.data[i][j] -= rhs.data[i][j];
            }
        }
    }
}

/// Add two matrices
impl<const M: usize, const N: usize> std::ops::Add<Self> for Matrix<M, N> {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Self {
            data: {
                let mut data = [[0.0; M]; N];
                for (i, row) in data.iter_mut().enumerate() {
                    for (j, value) in row.iter_mut().enumerate() {
                        *value = self.data[i][j] + rhs.data[i][j];
                    }
                }
                data
            },
        }
    }
}

/// Add reference matrix to matrix
impl<const M: usize, const N: usize> std::ops::Add<&Self> for Matrix<M, N> {
    type Output = Self;

    fn add(self, rhs: &Self) -> Self::Output {
        Self {
            data: {
                let mut data = [[0.0; M]; N];
                for (i, row) in data.iter_mut().enumerate() {
                    for (j, value) in row.iter_mut().enumerate() {
                        // data is column major
                        *value = self.data[i][j] + rhs.data[i][j];
                    }
                }
                data
            },
        }
    }
}

/// add matrix to reference matrix
impl<const M: usize, const N: usize> std::ops::Add<Matrix<M, N>> for &Matrix<M, N> {
    type Output = Matrix<M, N>;

    fn add(self, rhs: Matrix<M, N>) -> Self::Output {
        Matrix {
            data: {
                let mut data = [[0.0; M]; N];
                for (i, row) in data.iter_mut().enumerate() {
                    for (j, value) in row.iter_mut().enumerate() {
                        // data is column major
                        *value = self.data[i][j] + rhs.data[i][j];
                    }
                }
                data
            },
        }
    }
}

/// Subtract two matrices
impl<const M: usize, const N: usize> std::ops::Sub<Self> for Matrix<M, N> {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        Self {
            data: {
                let mut data = [[0.0; M]; N];
                for (i, row) in data.iter_mut().enumerate() {
                    for (j, value) in row.iter_mut().enumerate() {
                        // data is column major
                        *value = self.data[i][j] - rhs.data[i][j];
                    }
                }
                data
            },
        }
    }
}

impl<const M: usize, const N: usize> std::ops::Sub<Matrix<M, N>> for &Matrix<M, N> {
    type Output = Matrix<M, N>;

    fn sub(self, rhs: Matrix<M, N>) -> Self::Output {
        Matrix {
            data: {
                let mut data = [[0.0; M]; N];
                for (i, row) in data.iter_mut().enumerate() {
                    for (j, value) in row.iter_mut().enumerate() {
                        // data is column major
                        *value = self.data[i][j] - rhs.data[i][j];
                    }
                }
                data
            },
        }
    }
}

/// Implementations for matrix multiplication
/// # Example
/// ```
/// use satctrl::Matrix;
/// let m1 = Matrix::<3, 2>::from_row_major_array([[1.0, 2.0], [3.0, 4.0], [5.0, 6.0]]);
/// let m2 = Matrix::<2, 3>::from_row_major_array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]);
/// let m3 = m1 * m2;
/// ```
/// # Returns
/// A new matrix that is the result of the matrix multiplication
/// of the two input matrices
///
impl<const M: usize, const N: usize, const P: usize> std::ops::Mul<Matrix<N, P>> for Matrix<M, N> {
    type Output = Matrix<M, P>;

    fn mul(self, rhs: Matrix<N, P>) -> Self::Output {
        let mut data = [[0.0; M]; P];
        for (i, row) in data.iter_mut().enumerate() {
            for (j, value) in row.iter_mut().enumerate() {
                for k in 0..N {
                    *value += self.data[k][j] * rhs.data[i][k];
                }
            }
        }
        Matrix::<M, P> { data }
    }
}
