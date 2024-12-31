# Simple Math Library

## About

The Simple Math library is -- as the name implies -- a simple library for mathematical functions, with a focus on matrix manipulation, 3D rotations, and solutions to ordinary differential equations.

The libary has minimmal external dependencies, and is intended to be suitable for use in high-reliability embedded applications.

Note that the crate still uses the standard libary; it *is not* suitable for bare-metal applications.

It includes fixed-size matrices and associated functions for manipulation.  It **does not** include dynamic matrices.  External dependencies are ekept to a bare minimum

Manipulation of 3D vectors is a particular focus. The library includes a quaternion structure to enable
fast rotations of 3D vectors. 

The library also includse a module for the integration of ordinary differential equations via implicit 
adaptive Runga-Kutta methods.  

This library is written with a focus on applications for satellite attitude dynamics and orbital dynamics, with features added on an as-needed basis to support such applications.

## Why another math library?

Rust includes multiple mathematics libraries, including the well-written [nalgebra](https://docs.rs/nalgebra/latest/nalgebra/) library.  However, I find some of the nomenclature in nalgebra tedious and non-intuitive.  This is intended to be a focused, lightweight, and simpler alternative.

## Additional Features

Additional features will be added as needed; this is still very much a work in progress

## Author

Steven Michael (ssmichael@gmail.com)

## License

MIT License