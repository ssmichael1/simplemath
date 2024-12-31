#[cfg(test)]
use super::Matrix;
use super::Vector;

#[test]
fn test_multiply() {
    let m1 = Matrix::<3, 2>::from_row_major_array([[1.0, 2.0], [3.0, 4.0], [5.0, 6.0]]);
    let m2 = Matrix::<2, 3>::from_row_major_array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]);
    let m3 = m1 * m2;
    assert_eq!(
        m3,
        Matrix::<3, 3>::from_row_major_array([
            [9.0, 12.0, 15.0],
            [19.0, 26.0, 33.0],
            [29.0, 40.0, 51.0]
        ])
    );
}

#[test]
fn test_vec() {
    let m1 =
        Matrix::<3, 3>::from_row_major_array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 9.0]]);
    let v1 = Vector::<3>::from_slice(&[1.0, 2.0, 3.0]);
    let vout = m1 * v1;
    assert_eq!(vout, Vector::<3>::from_slice(&[14.0, 32.0, 50.0]));
}

#[test]
fn test_cross_product() {
    // Test cross product follows right-handed convention
    type Vector3 = Vector<3>;
    assert!(Vector3::xhat().cross(&Vector3::yhat()) == Vector3::zhat());
    assert!(Vector3::yhat().cross(&Vector3::zhat()) == Vector3::xhat());
    assert!(Vector3::zhat().cross(&Vector3::xhat()) == Vector3::yhat());
}
