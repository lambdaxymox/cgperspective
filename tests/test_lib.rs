extern crate cgperspective;
extern crate cglinalg;


use cglinalg::{
    Matrix4x4,
    Angle,
    Degrees,
    Radians,
};
use cgperspective::{
    PerspectiveSpec,
    OrthographicSpec,
    PerspectiveFovSpec,
    OrthographicFovSpec,
    PerspectiveProjection,
    PerspectiveFovProjection,
    OrthographicProjection,
    OrthographicFovProjection,
    CameraModel,
};


#[test]
fn test_perspective_projection_transformation() {
    let left = -4.0;
    let right = 4.0;
    let bottom = -2.0;
    let top = 3.0;
    let near = 1.0;
    let far = 100.0;
    let spec = PerspectiveSpec::new(left, right, bottom, top, near, far);
    let expected = Matrix4x4::new(
        1.0 / 4.0,  0.0,        0.0,           0.0,
        0.0,        2.0 / 5.0,  0.0,           0.0,
        0.0,        1.0 / 5.0, -101.0 / 99.0, -1.0,
        0.0,        0.0,       -200.0 / 99.0,  0.0
    );
    let result = PerspectiveProjection::from_spec(&spec);

    assert_eq!(result.to_matrix(), &expected);
}


#[test]
fn test_perspective_projection_fov_transformation() {
    let fovy = Degrees(72.0);
    let aspect = 800 as f32 / 600 as f32;
    let near = 0.1;
    let far = 100.0;
    let spec = PerspectiveFovSpec::new(fovy, aspect, near, far);
    let expected = Matrix4x4::new(
        1.0322863, 0.0,        0.0,       0.0, 
        0.0,       1.3763818,  0.0,       0.0, 
        0.0,       0.0,       -1.002002, -1.0, 
        0.0,       0.0,       -0.2002002, 0.0
    );
    let result = PerspectiveFovProjection::from_spec(&spec);

    assert_eq!(result.to_matrix(), &expected);
}


#[test]
fn test_orthographic_projection_transformation() {
    let left = -4.0;
    let right = 4.0;
    let bottom = -2.0;
    let top = 2.0;
    let near = 1.0;
    let far = 100.0;
    let spec = OrthographicSpec::new(left, right, bottom, top, near, far);
    let expected = Matrix4x4::new(
        1.0 / 4.0,  0.0,        0.0,          0.0,
        0.0,        1.0 / 2.0,  0.0,          0.0,
        0.0,        0.0,       -2.0 / 99.0,   0.0,
        0.0,        0.0,       -101.0 / 99.0, 1.0
    );
    let result = OrthographicProjection::from_spec(&spec);

    assert_eq!(result.to_matrix(), &expected);
}


#[test]
fn test_orthographic_fov_projecton_transformation() {
    let aspect = 2.0;
    // 9.1478425198 Degrees.
    let fovy = Degrees::from(Radians::atan2(8.0, 100.0) * 2.0);
    let near = 1.0;
    let far = 100.0;
    let spec = OrthographicFovSpec::new(fovy, aspect, near, far);
    let expected = Matrix4x4::new(
        1.0 / 4.0,  0.0,        0.0,          0.0,
        0.0,        1.0 / 2.0,  0.0,          0.0,
        0.0,        0.0,       -2.0 / 99.0,   0.0,
        0.0,        0.0,       -101.0 / 99.0, 1.0
    );
    let result = OrthographicFovProjection::from_spec(&spec);

    assert_eq!(result.to_matrix(), &expected);
}
