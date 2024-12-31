use crate::matrixutils;
use crate::Result;
use crate::{Matrix, Vector};

/// Unscented Kalman filter with "N" states
///
pub struct UKF<const N: usize> {
    pub alpha: f64,
    pub beta: f64,
    pub kappa: f64,
    weight_m: Vec<f64>,
    weight_c: Vec<f64>,
    pub x: Vector<N>,
    pub p: Matrix<N, N>,
}

impl<const N: usize> UKF<N> {
    fn weight_m(alpha: f64, kappa: f64) -> Vec<f64> {
        let mut weight_m = Vec::<f64>::with_capacity(2 * N + 1);
        let den = alpha.powi(2) * (N as f64 + kappa);
        weight_m.push(1.0 - N as f64 / den);
        for _i in 1..2 * N + 1 {
            weight_m.push(1.0 / (2.0 * den));
        }
        weight_m
    }

    fn weight_c(alpha: f64, beta: f64, kappa: f64) -> Vec<f64> {
        let mut weight_c = Vec::<f64>::with_capacity(2 * N + 1);
        let den: f64 = alpha.powi(2) * (N as f64 + kappa);
        weight_c.push(2.0 - alpha.powi(2) + beta - N as f64 / den);
        for _i in 1..2 * N + 1 {
            weight_c.push(1.0 / (2.0 * den));
        }
        weight_c
    }

    /// Construct new Unscented Kalman filter with default values
    ///
    /// For nice refernce implementation, see:
    /// <https://www.mathworks.com/help/control/ug/extended-and-unscented-kalman-filter-algorithms-for-online-state-estimation.html>
    ///
    /// Default values are:
    /// * alpha = 0.001
    /// * beta = 2.0
    /// * kappa = 0.0
    ///
    /// # Returns
    /// A new Unscented Kalman filter with state and covariance initialized to identity
    ///
    /// # Example
    ///
    /// ```
    /// use satctrl::filters::UKF;
    /// let mut ukf = UKF::<2>::new_default();
    /// ```
    ///
    pub fn new_default() -> Self {
        Self {
            alpha: 0.001,
            beta: 2.0,
            kappa: 0.0,
            weight_m: Self::weight_m(0.001, 0.0),
            weight_c: Self::weight_c(0.001, 2.0, 0.0),
            x: Vector::<N>::zeros(),
            p: Matrix::<N, N>::identity(),
        }
    }

    /// Construct a new Unscented Kalman filter with user-settable parameters
    ///
    /// For nice reference implementation, see:
    /// <https://www.mathworks.com/help/control/ug/extended-and-unscented-kalman-filter-algorithms-for-online-state-estimation.html>
    ///
    /// # Arguments
    /// * `alpha` - Scaling parameter for sigma points
    /// * `beta` - Incorporates prior knowledge of the distribution of the state
    /// * `kappa` - Secondary scaling parameter
    ///
    /// # Returns
    /// A new Unscented Kalman filter with state and covariance initialized to identity
    ///
    /// # Example
    ///
    /// ```
    /// use satctrl::filters::UKF;
    /// let mut ukf = UKF::<2>::new(0.001, 2.0, 0.0);
    /// ```
    ///
    pub fn new(alpha: f64, beta: f64, kappa: f64) -> Self {
        Self {
            alpha,
            beta,
            kappa,
            weight_m: Self::weight_m(alpha, kappa),
            weight_c: Self::weight_c(alpha, beta, kappa),
            x: Vector::<N>::zeros(),
            p: Matrix::<N, N>::identity(),
        }
    }

    /// Update step: Update with a measurement of size "M" and a measurement covariacne
    ///
    /// # Arguments
    /// * `y` - Measurement vector
    /// * `y_cov` - Measurement covariance
    /// * `f` - Function that maps state to measurement
    ///
    /// # Returns
    /// A result indicating success or failure
    ///
    /// # Example
    ///
    /// ```
    /// use satctrl::filters::UKF;
    /// use satctrl::Matrix;
    /// use satctrl::Vector;
    /// let mut ukf = UKF::<2>::new_default();
    /// let y = Vector::<2>::from_slice(&[3.0, 4.0]);
    /// let y_cov = Matrix::<2, 2>::from_row_major_slice(&[1.0, 0.0, 0.0, 1.0]);
    /// let f = |x: Vector<2>| x;
    /// ukf.update(&y, &y_cov, f);
    /// ```
    ///
    pub fn update<const M: usize>(
        &mut self,
        y: &Vector<M>,
        y_cov: &Matrix<M, M>,
        f: impl Fn(Vector<N>) -> Result<Vector<M>>,
    ) -> Result<()> {
        let c = self.alpha.powi(2) * (N as f64 + self.kappa);

        let cp = c.sqrt() * matrixutils::cholesky_decomp(&self.p)?;

        let mut x_sigma_points = Vec::<Vector<N>>::with_capacity(2 * N + 1);

        // Create prior weights
        x_sigma_points.push(self.x);
        for i in 0..N {
            x_sigma_points.push(self.x + cp.column(i));
        }
        for i in 0..N {
            x_sigma_points.push(self.x - cp.column(i));
        }

        // Compute predict with sigma values
        let yhat_i = x_sigma_points
            .iter()
            .map(|x| f(*x))
            .collect::<Result<Vec<Vector<M>>>>()?;

        let yhat = yhat_i
            .iter()
            .enumerate()
            .fold(Vector::<M>::zeros(), |acc, (i, y)| {
                acc + self.weight_m[i] * y
            });

        // Compute predicted covariance
        let p_yy = yhat_i.iter().enumerate().fold(*y_cov, |acc, (i, y)| {
            acc + self.weight_c[i] * (y - yhat) * (y - yhat).transpose()
        });

        // Compute cross covariance
        let p_xy = x_sigma_points
            .iter()
            .zip(yhat_i.iter())
            .enumerate()
            .fold(Matrix::<N, M>::zeros(), |acc, (i, (x, y))| {
                acc + self.weight_c[i] * (x - self.x) * (y - yhat).transpose()
            });

        let kalman_gain = p_xy
            * p_yy
                .inverse()
                .ok_or(crate::MathError::NotInvertibleMatrix)?;
        self.x += kalman_gain * (y - yhat);
        self.p -= kalman_gain * p_yy * kalman_gain.transpose();
        Ok(())
    }

    /// Predict step
    /// Note: add your own process noise after this function is called
    ///
    /// # Arguments
    /// * `f` - Function that maps previous state to next state
    ///
    /// # Returns
    /// A result indicating success or failure
    ///
    /// # Example
    ///
    /// ```
    /// use satctrl::filters::UKF;
    /// use satctrl::Vector;
    /// let mut ukf = UKF::<2>::new_default();
    /// let f = |x: &Vector<2>| Ok(x.clone());
    /// ukf.predict(f);
    /// ```
    pub fn predict(&mut self, f: impl Fn(&Vector<N>) -> Result<Vector<N>>) -> Result<()> {
        let c = self.alpha.powi(2) * (N as f64 + self.kappa);

        let cp = c.sqrt() * matrixutils::cholesky_decomp(&self.p)?;

        let mut x_sigma_points = Vec::<Vector<N>>::with_capacity(2 * N + 1);

        // Create prior weights
        x_sigma_points.push(self.x);
        for i in 0..N {
            x_sigma_points.push(self.x + cp.column(i));
        }
        for i in 0..N {
            x_sigma_points.push(self.x - cp.column(i));
        }

        // Compute predict with sigma values
        let x_post = x_sigma_points.iter().map(f).collect::<Result<Vec<_>>>()?;

        // Update state
        self.x = x_post
            .iter()
            .enumerate()
            .fold(Vector::<N>::zeros(), |acc, (i, x)| {
                acc + self.weight_m[i] * x
            });

        // Update covariance
        self.p = x_post
            .iter()
            .enumerate()
            .fold(Matrix::<N, N>::zeros(), |acc, (i, x)| {
                acc + self.weight_c[i] * (x - self.x) * (x - self.x).transpose()
            });

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use rand_distr::{Distribution, Normal};

    #[test]
    fn test_ukf() {
        let mut ukf = UKF::<2>::new_default();

        let normal = Normal::new(0.0, 1.0).unwrap();

        let ytruth = Vector::<2>::from_slice(&[3.0, 4.0]);
        let y_cov = Matrix::<2, 2>::from_row_major_slice(&[1.0, 0.0, 0.0, 1.0]);
        let v = normal.sample(&mut rand::thread_rng());
        let w = normal.sample(&mut rand::thread_rng());
        let ysample = ytruth + Vector::<2>::from_slice(&[v, w]);
        let offset = Vector::<2>::from_slice(&[5.0, 8.0]);
        let observe = |x: Vector<2>| Ok(x + offset);

        // Process noise
        let q = Matrix::<2, 2>::from_row_major_slice(&[1.0e-12, 0.0, 0.0, 1.0e-12]);
        ukf.x = ysample;
        ukf.p = y_cov;
        for _ix in 0..500 {
            let v = normal.sample(&mut rand::thread_rng());
            let w = normal.sample(&mut rand::thread_rng());
            let ysample = observe(ytruth + Vector::<2>::from_vec([v, w])).unwrap();
            match ukf.update(&ysample, &y_cov, observe) {
                Ok(_) => (),
                Err(_) => panic!("UKF update failed"),
            }
            ukf.p += q;
        }
    }
}
