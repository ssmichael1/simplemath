/// Runga-Kutta 4th order method
///
/// Integrate a function using the Runge-Kutta 4th order method.
///
/// # Arguments
/// * `f` - The function to integrate (dy/dx)
/// * `x` - The current x value
/// * `y` - The current y (state) value)
/// * `h` - The interval to integrate over
///
/// # Returns
///
/// The new y value after integrating over the interval h.
///
/// # Example
///
/// ```
/// use satctrl::rk4_integrate;
/// use satctrl::Vector2;
/// // 1D harmonic oscillator. 1st state is position, 2nd is velocity.
/// let f = |x: f64, y: &Vector2| Vector2::from_vec([y[1], -y[0]]);
/// let x = 0.0;
/// let y = Vector2::from_vec([1.0, 0.0]);
/// let h = 0.1;
/// let y_new = rk4_integrate(f, x, y, h);
/// ```
///
pub fn rk4_integrate<F, S>(f: F, x: f64, y: S, h: f64) -> S
where
    F: Fn(f64, &S) -> S,
    S: std::ops::Div<f64, Output = S>
        + std::ops::Mul<f64, Output = S>
        + std::ops::Add<S, Output = S>
        + Clone,
{
    let k1 = f(x, &y) * h;
    let k2 = f(x + h / 2.0, &(y.clone() + k1.clone() / 2.0)) * h;
    let k3 = f(x + h / 2.0, &(y.clone() + k2.clone() / 2.0)) * h;
    let k4 = f(x + h, &(y.clone() + k3.clone())) * h;
    y + (k1 + k2 * 2.0 + k3 * 2.0 + k4) / 6.0
}

/// Runga-Kutta 4th order method (in-place)
///
/// Integrate a function using the Runge-Kutta 4th order method.
///
/// # Arguments
/// * `f` - The function to integrate (dy/dx)
/// * `x` - The current x value
/// * `y` - The current y (state) value)
/// * `h` - The interval to integrate over
///
/// # Example
///
/// ```
/// use satctrl::rk4_integrate_inplace;
/// use satctrl::Vector2;
/// // 1D harmonic oscillator. 1st state is position, 2nd is velocity.
/// let f = |x: f64, y: &Vector2| Vector2::from_vec([y[1], -y[0]]);
/// let mut y = Vector2::from_vec([1.0, 0.0]);
/// let x = 0.0;
/// let h = 0.1;
/// rk4_integrate_inplace(f, x, &mut y, h);
/// ```
///
pub fn rk4_integrate_inplace<F, S>(f: F, x: f64, y: &mut S, h: f64)
where
    F: Fn(f64, &S) -> S,
    S: std::ops::Div<f64, Output = S>
        + std::ops::Mul<f64, Output = S>
        + std::ops::Add<S, Output = S>
        + Clone,
{
    let k1 = f(x, y) * h;
    let k2 = f(x + h / 2.0, &(y.clone() + k1.clone() / 2.0)) * h;
    let k3 = f(x + h / 2.0, &(y.clone() + k2.clone() / 2.0)) * h;
    let k4 = f(x + h, &(y.clone() + k3.clone())) * h;
    *y = y.clone() + (k1 + k2 * 2.0 + k3 * 2.0 + k4) / 6.0;
}
