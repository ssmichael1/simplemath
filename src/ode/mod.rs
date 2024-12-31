//! Ordinary Differential Equation (ODE) solvers
//!
//! This module contains Runga-Kutta ODE solvers of varying order and accuracy.
//!
//!
//! Solvers use Runga-Kutta coefficients from Jim Verner's 2006 paper "High-Order Runge-Kutta
//! Methods for Ordinary Differential Equations" and the 2010 paper "High-Order Embedded Runge-Kutta
//! Formulae" by Tsitouras.
//!
//!
//!
//! Author: Steven Michael (ssmichael@gmail.com)
//! Date: 12/8/24
//! License: MIT
//!
//!

pub mod rk_adaptive;
pub mod rk_adaptive_settings;
pub mod rk_explicit;
mod types;
//mod rkf45;
mod rk4;
mod rkts54;
mod rkv65;
mod rkv65_table;
mod rkv87;
mod rkv87_table;
mod rkv98;
mod rkv98_nointerp;
mod rkv98_nointerp_table;
mod rkv98_table;

#[allow(unused)]
pub use rk_adaptive::RKAdaptive;
#[allow(unused)]
pub use rk_adaptive_settings::RKAdaptiveSettings;

pub use rk4::rk4_integrate;
pub use rk4::rk4_integrate_inplace;

pub mod adaptive {
    #[allow(unused)]
    pub use super::rk_explicit::Midpoint;
    #[allow(unused)]
    pub use super::rk_explicit::RK4;
    #[allow(unused)]
    pub use super::rkts54::RKTS54;
    #[allow(unused)]
    pub use super::rkv65::RKV65;
    #[allow(unused)]
    pub use super::rkv87::RKV87;
    #[allow(unused)]
    pub use super::rkv98::RKV98;
    #[allow(unused)]
    pub use super::rkv98_nointerp::RKV98NoInterp;
}

pub use types::*;

mod odestate_matrix;
