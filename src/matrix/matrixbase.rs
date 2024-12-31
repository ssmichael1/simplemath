/// A simple fixed-size matrix library
///
/// Author: Steven Michael (ssmichael@gmail.com)
/// Date: 2024-11-16
///
/// Description:
///
/// This module provides a simple fixed-size matrix library with no
/// external dependencies. The library is designed to be used in
/// embedded systems where dynamic memory allocation is not desired.

/// Fixed-size matrix type
///
/// Note: data storage is natively column major
/// Note: data type is set as f64
///
#[derive(Clone, Copy)]

pub struct Matrix<const M: usize, const N: usize> {
    // This would ideally be a 1D array, but Rust does not support const generics for this yet
    pub(crate) data: [[f64; M]; N], // M is rows, N is columns; store as column major
}

/// Fixed-size vector type (a 1-D column matrix)
pub type Vector<const M: usize> = Matrix<M, 1>;

impl<const M: usize, const N: usize> Matrix<M, N> {
    /// Create a new matrix from a 2D col-major array representation
    ///
    /// # Arguments
    ///   * `data` - A 2D array of f64 values, column major!
    ///
    /// # Example
    /// ```
    /// use satctrl::Matrix;
    /// let m = Matrix::<3, 2>::from_col_major_array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]);
    /// ```
    ///
    pub fn from_col_major_array(data: [[f64; M]; N]) -> Self {
        Self { data }
    }

    /// Create a new matrix from a 2D row-major array representation
    ///
    /// # Arguments
    ///
    /// * `data` - A 2D array of f64 values, row major!
    ///
    /// # Example
    ///
    /// ```
    /// use satctrl::Matrix;
    /// let m = Matrix::<3, 2>::from_row_major_array([[1.0, 2.0], [3.0, 4.0], [5.0, 6.0]]);
    /// ```
    ///
    pub fn from_row_major_array(data: [[f64; N]; M]) -> Self {
        let mut m = Self::zeros();
        for (i, row) in data.iter().enumerate() {
            for (j, &value) in row.iter().enumerate() {
                m.data[j][i] = value;
            }
        }
        m
    }

    pub const fn num_elements(&self) -> usize {
        M * N
    }

    /// Create a new matrix from a 1D slice in column major order
    ///
    /// # Arguments
    /// * `data` - A 1D slice of f64 values representing a 2D matrix in column-major order
    ///
    /// # Example
    /// ```
    /// use satctrl::Matrix;
    /// let m = Matrix::<3, 2>::from_col_major_slice(&[1.0, 2.0, 3.0, 4.0, 5.0, 6.0]);
    /// ```
    ///
    /// # Returns
    /// A new matrix created from the input slice
    ///
    pub fn from_col_major_slice(data: &[f64]) -> Self {
        let mut m = Self::zeros();
        for i in 0..N {
            for j in 0..M {
                m.data[i][j] = data[i * M + j];
            }
        }
        m
    }

    /// Create a new matrix from a 1D slice in row major order
    ///
    /// # Arguments
    /// * `data` - A 1D slice of f64 values representing a 2D matrix in row-major order
    ///
    /// # Example
    /// ```
    /// use satctrl::Matrix;
    /// let m = Matrix::<3, 2>::from_row_major_slice(&[1.0, 2.0, 3.0, 4.0, 5.0, 6.0]);
    /// ```
    ///
    /// # Returns
    /// A new matrix created from the input slice
    ///
    pub fn from_row_major_slice(data: &[f64]) -> Self {
        let mut m = Self::zeros();
        for i in 0..M {
            for j in 0..N {
                m.data[j][i] = data[i * N + j];
            }
        }
        m
    }

    /// Create a new matrix with all elements set to zero
    /// # Example
    /// ```
    /// use satctrl::Matrix;
    /// let m = Matrix::<3, 3>::zeros();
    /// ```
    /// # Returns
    /// A new matrix with all elements set to zero
    ///
    pub fn zeros() -> Self {
        Self {
            data: [[0.0; M]; N],
        }
    }

    /// Create a new matrix with all elements set to one
    ///
    /// # Example
    /// ```
    /// use satctrl::Matrix;
    /// let m = Matrix::<3, 3>::ones();
    /// ```
    ///
    /// # Returns
    /// A new matrix with all elements set to one
    ///
    pub fn ones() -> Self {
        Self {
            data: [[1.0; M]; N],
        }
    }

    /// Get the number of rows in the matrix
    ///
    /// # Example
    /// ```
    /// use satctrl::Matrix;
    /// let m = Matrix::<3, 3>::identity();
    /// assert_eq!(m.rows(), 3);
    /// ```
    ///
    /// # Returns
    ///
    /// The number of rows in the matrix
    ///
    pub fn rows(&self) -> usize {
        M
    }

    /// Get the number of columns in the matrix
    pub fn cols(&self) -> usize {
        N
    }

    /// Get the element at the given row and column
    pub fn get(&self, row: usize, col: usize) -> f64 {
        self.data[row][col]
    }

    /// Set the element at the given row and column
    pub fn set(&mut self, row: usize, col: usize, value: f64) {
        self.data[row][col] = value;
    }

    /// Transpose the matrix
    ///
    /// # Example
    /// ```
    /// use satctrl::Matrix;
    /// let m = Matrix::<3, 2>::from_row_major_array([[1.0, 2.0], [3.0, 4.0], [5.0, 6.0]]);
    /// let t = m.transpose();
    /// ```
    /// # Returns
    /// A new matrix that is the transpose of the original matrix
    ///
    pub fn transpose(&self) -> Matrix<N, M> {
        let mut data = [[0.0; N]; M];
        for (i, row) in self.data.iter().enumerate() {
            for (j, value) in row.iter().enumerate() {
                data[j][i] = *value;
            }
        }
        Matrix::<N, M> { data }
    }

    /// Return the column at the given index
    ///
    /// # Arguments
    /// * `col` - The index of the column
    ///
    /// # Returns
    /// A new vector that is the column at the given index
    ///
    /// # Example
    ///
    /// ```
    /// use satctrl::Matrix;
    /// let m = Matrix::<3, 3>::identity();
    /// let c = m.column(0);
    /// ```
    ///
    pub fn column(&self, col: usize) -> Vector<M> {
        Vector::<M> {
            data: [self.data[col]],
        }
    }

    /// Return the row at the given index
    ///
    /// # Arguments
    /// * `row` - The index of the row
    ///
    /// # Returns
    /// A new vector that is the row at the given index
    ///
    /// # Example
    ///
    /// ```
    /// use satctrl::Matrix;
    /// let m = Matrix::<3, 3>::identity();
    /// let r = m.row(0);
    /// ```    
    ///
    pub fn row(&self, row: usize) -> Vector<N> {
        let mut data = [0.0; N];
        for (i, value) in data.iter_mut().enumerate() {
            *value = self.data[i][row];
        }
        Vector::<N> { data: [data] }
    }
}

/// Display in in debug format the matrix
///
/// # Example
///
/// ```
/// use satctrl::Matrix;
/// let m = Matrix::<3, 3>::identity();
/// println!("{:?}", m);
/// ```
///
/// # Returns
/// A string representation of the matrix in debug format
impl<const M: usize, const N: usize> std::fmt::Debug for Matrix<M, N> {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        writeln!(f, "Matrix {}x{}", M, N)?;
        for row in 0..M {
            write!(f, "[")?;
            for col in 0..N {
                write!(f, "{:8.3}", self[(row, col)])?;
                if col < N - 1 {
                    write!(f, ", ")?;
                }
            }
            write!(f, "]")?;
            writeln!(f)?;
        }
        Ok(())
    }
}

/// Immutable index for vector
///
/// # Example
/// ```
/// use satctrl::Vector;
/// let v = Vector::<3>::from_vec([1.0, 2.0, 3.0]);
/// assert_eq!(v[0], 1.0);
/// ```
///
/// # Returns
/// A reference to the element at the given index
impl<const N: usize> std::ops::Index<usize> for Vector<N> {
    type Output = f64;

    fn index(&self, index: usize) -> &Self::Output {
        &self.data[0][index]
    }
}

/// Mutable index for vector
///
/// # Example
///
/// ```
/// use satctrl::Vector;
/// let mut v = Vector::<3>::from_vec([1.0, 2.0, 3.0]);
/// v[0] = 4.0;
/// ```
///
/// # Returns
/// A mutable reference to the element at the given index
///
impl<const N: usize> std::ops::IndexMut<usize> for Vector<N> {
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        &mut self.data[0][index]
    }
}

/// Immutable index for matrix
///
/// # Arguments
/// * `index` - A tuple of the row and column of the element
///
/// # Example
///
/// ```
/// use satctrl::Matrix;
/// let m = Matrix::<3, 3>::identity();
/// assert_eq!(m[(0, 0)], 1.0);
/// ```
/// # Returns
/// A reference to the element at the given row and column
///
impl<const M: usize, const N: usize> std::ops::Index<(usize, usize)> for Matrix<M, N> {
    type Output = f64;

    fn index(&self, index: (usize, usize)) -> &Self::Output {
        // data is column major (column is 2nd index)
        &self.data[index.1][index.0]
    }
}

/// Mutable index for matrix
///
/// # Arguments
/// * `index` - A tuple of the row and column of the element
///
/// # Example
/// ```
/// use satctrl::Matrix;
/// let mut m = Matrix::<3, 3>::zeros();
/// m[(0, 0)] = 1.0;
/// ```
/// # Returns
/// A mutable reference to the element at the given row and column
///
impl<const M: usize, const N: usize> std::ops::IndexMut<(usize, usize)> for Matrix<M, N> {
    fn index_mut(&mut self, index: (usize, usize)) -> &mut Self::Output {
        // data is column major (column is 2nd index)
        &mut self.data[index.1][index.0]
    }
}

/// Implementations for square matrices
impl<const M: usize> Matrix<M, M> {
    /// Create a new diagonal square matrix given input diagonal elements (trace)
    ///
    /// # Arguments
    ///    * `d` - A vector of diagonal elements
    /// # Example
    /// ```
    /// use satctrl::Matrix;
    /// use satctrl::Vector;
    /// let d = Vector::<3>::from_slice(&[1.0, 2.0, 3.0]);
    /// let m = Matrix::<3, 3>::diag_from_vector(&d);
    /// ```
    ///
    /// # Returns
    /// A new diagonal matrix
    ///
    pub fn diag_from_vector(d: &Vector<M>) -> Self {
        let mut data = [[0.0; M]; M];
        for (i, row) in data.iter_mut().enumerate() {
            row[i] = d[i];
        }
        Self { data }
    }

    /// Create a new identity matrix
    ///
    /// # Example
    /// ```
    /// use satctrl::Matrix;
    /// let m = Matrix::<3, 3>::identity();
    /// ```
    ///
    pub fn identity() -> Self {
        let mut data = [[0.0; M]; M];
        for (i, row) in data.iter_mut().enumerate() {
            row[i] = 1.0;
        }
        Self { data }
    }

    /// Return trace of the matrix
    ///
    /// # Example
    /// ```
    /// use satctrl::Matrix;
    /// let m = Matrix::<3, 3>::identity();
    /// assert_eq!(m.trace(), 3.0);
    /// ```
    /// # Returns
    /// The trace of the matrix (sum of diagonal elements)
    ///
    pub fn trace(&self) -> f64 {
        let mut sum = 0.0;
        for (i, row) in self.data.iter().enumerate() {
            sum += row[i];
        }
        sum
    }

    /// Return the determinant of the matrix
    /// # Example
    /// ```
    /// use satctrl::Matrix;
    /// let m = Matrix::<3, 3>::identity();
    /// assert_eq!(m.determinant(), 1.0);
    /// ```
    /// # Returns
    /// The determinant of the matrix
    ///
    pub fn determinant(&self) -> f64 {
        let mut data = self.data;
        let mut det = 1.0;
        for i in 0..M {
            if data[i][i] == 0.0 {
                for j in i + 1..M {
                    if data[j][i] != 0.0 {
                        data.swap(i, j);
                        det = -det;
                        break;
                    }
                }
            }
            if data[i][i] == 0.0 {
                return 0.0;
            }
            det *= data[i][i];
            for j in i + 1..M {
                let factor = data[j][i] / data[i][i];
                for k in i + 1..M {
                    data[j][k] -= factor * data[i][k];
                }
            }
        }
        det
    }

    /// Return the inverse of the matrix if matrix is non-singular
    ///
    /// # Returns
    /// The inverse of the matrix if it exists, None otherwise
    ///
    /// # Example
    ///
    /// ```
    /// use satctrl::Matrix;
    /// let m = Matrix::<3, 3>::identity();
    /// let inv = m.inverse().unwrap();
    /// ```
    ///
    pub fn inverse(&self) -> Option<Self> {
        let n = M;
        let mut lu = *self;
        let mut p = (0..n).collect::<Vec<_>>();

        // LU decomposition with partial pivoting
        for i in 0..n {
            let mut max = i;
            for j in i + 1..n {
                if lu.data[j][i].abs() > lu.data[max][i].abs() {
                    max = j;
                }
            }
            if lu.data[max][i] == 0.0 {
                return None;
            }
            lu.data.swap(i, max);
            p.swap(i, max);

            for j in i + 1..n {
                lu.data[j][i] /= lu.data[i][i];
                for k in i + 1..n {
                    lu.data[j][k] -= lu.data[j][i] * lu.data[i][k];
                }
            }
        }

        // Inverse calculation
        let mut inv = Self::identity();
        for (i, &pi) in p.iter().enumerate() {
            for j in 0..n {
                inv.data[i][j] = if pi == j { 1.0 } else { 0.0 };
                for k in 0..i {
                    inv.data[i][j] -= lu.data[i][k] * inv.data[k][j];
                }
            }
        }
        for i in (0..n).rev() {
            for j in 0..n {
                for k in i + 1..n {
                    inv.data[i][j] -= lu.data[i][k] * inv.data[k][j];
                }
                inv.data[i][j] /= lu.data[i][i];
            }
        }

        Some(inv)
    }
}

impl<const N: usize> Vector<N> {
    /// Create a new vector from a 1D array
    /// # Arguments
    ///  * `data` - A 1D array of f64 values
    ///
    /// # Example
    /// ```
    /// use satctrl::Vector;
    /// let v = Vector::<3>::from_vec([1.0, 2.0, 3.0]);
    /// ```
    ///
    /// # Returns
    /// A new vector
    ///
    pub fn from_vec(data: [f64; N]) -> Self {
        Self { data: [data] }
    }

    pub fn iter(&self) -> impl Iterator<Item = &'_ f64> {
        self.data[0].iter()
    }

    /// Create a new vector from a slice
    ///
    /// # Arguments
    /// * `data` - A slice of f64 values
    ///
    /// # Example
    /// ```
    /// use satctrl::Vector;
    /// let v = Vector::<3>::from_slice(&[1.0, 2.0, 3.0]);
    /// ```
    ///
    pub fn from_slice(data: &[f64]) -> Self {
        let mut v = Self::zeros();
        v.data[0].copy_from_slice(data);
        v
    }

    /// return vector as a slice
    ///
    /// # Example
    /// ```
    /// use satctrl::Vector;
    /// let v = Vector::<3>::from_vec([1.0, 2.0, 3.0]);
    /// let s = v.as_slice();
    /// ```
    ///
    /// # Returns
    /// Vector represented as a slice
    ///
    pub fn as_slice(&self) -> &[f64] {
        &self.data[0]
    }

    /// Return vector as a mutable slice
    ///
    /// # Example
    /// ```
    /// use satctrl::Vector;
    /// let mut v = Vector::<3>::from_vec([1.0, 2.0, 3.0]);
    /// let s = v.as_mut_slice();
    /// ```
    ///
    /// # Returns
    /// Vector represented as a mutable slice
    pub fn as_mut_slice(&mut self) -> &mut [f64] {
        &mut self.data[0]
    }

    /// Return the dot product of two vectors
    ///
    /// # Example
    ///
    /// ```
    /// use satctrl::Vector;
    /// let v1 = Vector::<3>::from_vec([1.0, 2.0, 3.0]);
    /// let v2 = Vector::<3>::from_vec([4.0, 5.0, 6.0]);
    /// let d = v1.dot(&v2);
    /// assert_eq!(d, 32.0);
    /// ```
    ///
    /// # Returns
    /// The dot product of the two vectors
    ///
    pub fn dot(&self, other: &Self) -> f64 {
        let mut sum = 0.0;
        for i in 0..N {
            sum += self.data[0][i] * other.data[0][i];
        }
        sum
    }

    /// Return the norm of the vector
    ///
    /// # Example
    /// ```
    /// use satctrl::Vector;
    /// let v = Vector::<3>::from_vec([1.0, 2.0, 3.0]);
    /// let n = v.norm();
    /// assert_eq!(n, 14.0_f64.sqrt());
    /// ```
    ///
    /// # Returns
    /// The norm of the vector
    pub fn norm(&self) -> f64 {
        self.dot(self).sqrt()
    }

    /// Return the square of the norm of the vector
    ///
    /// # Example
    /// ```
    /// use satctrl::Vector;
    /// let v = Vector::<3>::from_vec([1.0, 2.0, 3.0]);
    /// let n = v.normsq();
    /// assert_eq!(n, 14.0);
    /// ```
    ///
    /// # Returns
    /// The square of the norm of the vector
    ///
    pub fn normsq(&self) -> f64 {
        self.dot(self)
    }
}

impl Vector<3> {
    /// Return the cross product of two vectors
    ///
    /// # Example
    /// ```
    /// use satctrl::Vector;
    /// let v1 = Vector::<3>::from_vec([1.0, 0.0, 0.0]);
    /// let v2 = Vector::<3>::from_vec([0.0, 1.0, 0.0]);
    /// let v3 = v1.cross(&v2);
    /// assert_eq!(v3, Vector::<3>::from_vec([0.0, 0.0, 1.0]));
    /// ```
    ///
    /// # Returns
    /// The cross product of the two vectors
    ///
    pub fn cross(&self, other: &Self) -> Self {
        Self::from_vec([
            self.data[0][1] * other.data[0][2] - self.data[0][2] * other.data[0][1],
            self.data[0][2] * other.data[0][0] - self.data[0][0] * other.data[0][2],
            self.data[0][0] * other.data[0][1] - self.data[0][1] * other.data[0][0],
        ])
    }

    /// Return xhat unit vector
    ///
    /// # Example
    /// ```
    /// use satctrl::Vector3;
    /// let xhat = Vector3::xhat();
    /// ```
    ///
    /// # Returns
    /// The xhat unit vector
    ///
    pub fn xhat() -> Self {
        Self::from_vec([1.0, 0.0, 0.0])
    }

    /// Return yhat unit vector
    ///
    /// # Example
    /// ```
    /// use satctrl::Vector3;
    /// let yhat = Vector3::yhat();
    /// ```
    ///
    /// # Returns
    /// The yhat unit vector
    ///
    pub fn yhat() -> Self {
        Self::from_vec([0.0, 1.0, 0.0])
    }

    /// Return zhat unit vector
    ///
    /// # Example
    /// ```
    /// use satctrl::Vector3;
    /// let zhat = Vector3::zhat();
    /// ```
    ///
    /// # Returns
    /// The zhat unit vector
    ///
    pub fn zhat() -> Self {
        Self::from_vec([0.0, 0.0, 1.0])
    }

    /// Return the angle between two vectors
    ///
    /// # Returns
    /// The angle between the two vectors in radians
    ///
    /// # Example
    ///
    /// ```
    /// use satctrl::Vector;
    /// let v1 = Vector::<3>::from_vec([1.0, 0.0, 0.0]);
    /// let v2 = Vector::<3>::from_vec([0.0, 1.0, 0.0]);
    /// let angle = v1.angle_between(&v2);
    /// assert!(angle - std::f64::consts::FRAC_PI_2 < 1e-10);
    /// ```
    ///
    pub fn angle_between(&self, other: &Self) -> f64 {
        let dot = self.dot(other);
        let norm = self.norm() * other.norm();
        (dot / norm).acos()
    }
}

impl<const N: usize> From<[f64; N]> for Vector<N> {
    fn from(data: [f64; N]) -> Self {
        Self::from_slice(&data)
    }
}

impl<const M: usize, const N: usize> From<[[f64; M]; N]> for Matrix<M, N> {
    fn from(data: [[f64; M]; N]) -> Self {
        Self::from_col_major_array(data)
    }
}

impl<const M: usize> FromIterator<f64> for Vector<M> {
    fn from_iter<I: IntoIterator<Item = f64>>(iter: I) -> Self {
        let mut v = Self::zeros();
        for (i, value) in iter.into_iter().enumerate() {
            v.data[0][i] = value;
        }
        v
    }
}
