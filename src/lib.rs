use cglinalg::{
    Degrees,
    Vector3,
    Vector4,
    Matrix4x4, 
    Quaternion,
    ScalarFloat,
    InvertibleSquareMatrix,
    Unit,
};
use core::fmt;
use core::ops;


const MOVE_LEFT: u16             = 1 << 0;
const MOVE_RIGHT: u16            = 1 << 1;
const MOVE_UP: u16               = 1 << 2;
const MOVE_DOWN: u16             = 1 << 3;
const MOVE_FORWARD: u16          = 1 << 4;
const MOVE_BACKWARD: u16         = 1 << 5;
const PITCH_UP: u16              = 1 << 6;
const PITCH_DOWN: u16            = 1 << 7;
const YAW_LEFT: u16              = 1 << 8;
const YAW_RIGHT: u16             = 1 << 9;
const ROLL_CLOCKWISE: u16        = 1 << 10;
const ROLL_COUNTERCLOCKWISE: u16 = 1 << 11;
const NO_MOVEMENT: u16           = 0;


/// A simple camera movement is a basic movement from which all of the 
/// general movements of a camera movement are composed of.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub enum SimpleCameraMovement {
    MoveLeft,
    MoveRight,
    MoveUp,
    MoveDown,
    MoveForward,
    MoveBackward,
    PitchUp,
    PitchDown,
    YawLeft,
    YawRight,
    RollClockwise,
    RollCounterClockwise,
    NoMovement,
}

impl fmt::Display for SimpleCameraMovement {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        fmt::Debug::fmt(&self, f)
    }
}

/// A camera movement that is a sequence of simple movements use to compute
/// a camera trajectory on each game state update.
#[derive(Copy, Clone, PartialEq, Eq, Hash)]
pub struct CameraMovement {
    total: u16,
}

impl CameraMovement {
    pub const fn new() -> CameraMovement {
        CameraMovement {
            total: 0
        }
    }

    /// Include a camera movement in the compound camera movement.
    /// If the camera movement is already present in the compound camera movement,
    /// nothing changes.
    #[inline]
    fn add_movement(self, movement: SimpleCameraMovement) -> CameraMovement {
        use SimpleCameraMovement::*;
        let move_to = match movement {
            MoveLeft => MOVE_LEFT,
            MoveRight => MOVE_RIGHT,
            MoveUp => MOVE_UP,
            MoveDown => MOVE_DOWN,
            MoveForward => MOVE_FORWARD,
            MoveBackward => MOVE_BACKWARD,
            PitchUp => PITCH_UP,
            PitchDown => PITCH_DOWN,
            YawLeft => YAW_LEFT,
            YawRight => YAW_RIGHT,
            RollClockwise => ROLL_CLOCKWISE,
            RollCounterClockwise => ROLL_COUNTERCLOCKWISE,
            NoMovement => NO_MOVEMENT,
        };

        CameraMovement {
            total: self.total | move_to
        }
    }

    /// Remove camera movement if it is present in the compound camera movement.
    /// If the camera movement is not present in the compound camera movement, 
    /// nothing changes.
    #[inline]
    fn subtract_movement(self, movement: SimpleCameraMovement) -> CameraMovement {
        use SimpleCameraMovement::*;
        let move_to = match movement {
            MoveLeft => MOVE_LEFT,
            MoveRight => MOVE_RIGHT,
            MoveUp => MOVE_UP,
            MoveDown => MOVE_DOWN,
            MoveForward => MOVE_FORWARD,
            MoveBackward => MOVE_BACKWARD,
            PitchUp => PITCH_UP,
            PitchDown => PITCH_DOWN,
            YawLeft => YAW_LEFT,
            YawRight => YAW_RIGHT,
            RollClockwise => ROLL_CLOCKWISE,
            RollCounterClockwise => ROLL_COUNTERCLOCKWISE,
            NoMovement => NO_MOVEMENT,
        };

        CameraMovement {
            total: self.total ^ move_to
        } 
    }
}

impl ops::Add<SimpleCameraMovement> for CameraMovement {
    type Output = CameraMovement;

    #[inline]
    fn add(self, other: SimpleCameraMovement) -> Self::Output {
        self.add_movement(other)
    }
}

impl ops::Sub<SimpleCameraMovement> for CameraMovement {
    type Output = CameraMovement;

    #[inline]
    fn sub(self, other: SimpleCameraMovement) -> Self::Output {
        self.subtract_movement(other)
    }
}

impl ops::AddAssign<SimpleCameraMovement> for CameraMovement {
    #[inline]
    fn add_assign(&mut self, other: SimpleCameraMovement) {
        *self = self.add_movement(other)
    }
}

impl ops::SubAssign<SimpleCameraMovement> for CameraMovement {
    #[inline]
    fn sub_assign(&mut self, other: SimpleCameraMovement) {
        *self = self.subtract_movement(other)
    }
}

#[repr(C)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct DeltaAttitude<S> {
    x: S,
    y: S,
    z: S,
    roll: S,
    yaw: S,
    pitch: S,
}

impl<S> DeltaAttitude<S> where S: ScalarFloat {
    fn zero() -> DeltaAttitude<S> {
        DeltaAttitude {
            x: S::zero(),
            y: S::zero(),
            z: S::zero(),
            roll: S::zero(),
            yaw: S::zero(),
            pitch: S::zero(),
        }
    }
}

pub trait CameraModel {
    type Spec;
    type Projection;

    fn from_spec(spec: &Self::Spec) -> Self;

    fn projection(&self) -> &Self::Projection;

    fn update(&mut self, width: usize, height: usize);
}

pub trait CameraKinematics<S> {
    type Spec;

    fn from_spec(spec: &Self::Spec) -> Self;
    
    fn update(&self, movement: CameraMovement, elapsed: S) -> DeltaAttitude<S>;
}


/// A perspective projection based on the `near` plane, the `far` plane and 
/// the vertical field of view angle `fovy` and the horizontal/vertical aspect 
/// ratio `aspect`.
///
/// We assume the following constraints to make a useful perspective projection 
/// transformation.
/// ```text
/// 0 radians < fovy < pi radians
/// aspect > 0
/// near < far (along the negative z-axis)
/// ```
/// This perspective projection model imposes some constraints on the more 
/// general perspective specification based on the arbitrary planes. The `fovy` 
/// parameter combined with the aspect ratio `aspect` ensures that the top and 
/// bottom planes are the same distance from the eye position along the vertical 
/// axis on opposite side. They ensure that the `left` and `right` planes are 
/// equidistant from the eye on opposite sides along the horizontal axis. 
#[repr(C)]
#[derive(Clone, Debug)]
pub struct PerspectiveFovSpec<S> {
    /// The vertical field of view angle of the perspective transformation
    /// viewport.
    fovy: Degrees<S>,
    /// The ratio of the horizontal width to the vertical height.
    aspect: S,
    /// The position of the near plane along the **negative z-axis**.
    near: S,
    /// The position of the far plane along the **negative z-axis**.
    far: S,
}

impl<S> PerspectiveFovSpec<S> {
    /// Construct a new perspective projection operation specification
    /// based on the vertical field of view angle `fovy`, the `near` plane, the 
    /// `far` plane, and aspect ratio `aspect`.
    pub fn new(fovy: Degrees<S>, aspect: S, near: S, far: S) -> PerspectiveFovSpec<S> {
        PerspectiveFovSpec {
            fovy: fovy,
            aspect: aspect,
            near: near,
            far: far,
        }
    }
}

impl<S> fmt::Display for PerspectiveFovSpec<S> where S: fmt::Display {
    fn fmt(&self, formatter: &mut fmt::Formatter) -> fmt::Result {
       write!(
           formatter,
           "PerspectiveFovSpec [fovy={}, aspect={}, near={}, far={}]",
           self.fovy, self.aspect, self.near, self.far
       )
    }
}

/// A perspective projection transformation for converting from camera space to
/// normalized device coordinates based on the perspective field of view model.
///
/// Orthographic projections differ from perspective projections because
/// orthographic projections keeps parallel lines parallel, whereas perspective 
/// projections preserve the perception of distance. Perspective 
/// projections preserve the spatial ordering of points in the distance they 
/// are located from the viewing plane. This property of perspective projection 
/// transformations is important for operations such as z-buffering and 
/// occlusion detection.
#[repr(C)]
#[derive(Clone, Debug)]
pub struct PerspectiveFovProjection<S> {
    /// The vertical field of view angle of the perspective transformation
    /// viewport.
    fovy: Degrees<S>,
    /// The ratio of the horizontal width to the vertical height.
    aspect: S,
    /// The position of the near plane along the **negative z-axis**.
    near: S,
    /// The position of the far plane along the **negative z-axis**.
    far: S,
    /// The underlying perspective projection transformation.
    matrix: Matrix4x4<S>,
}

impl<S> PerspectiveFovProjection<S> {
    /// Returns a reference to the underlying perspective projection matrix.
    #[inline]
    pub fn to_matrix(&self) -> &Matrix4x4<S> {
        &self.matrix
    }
}

impl<S> CameraModel for PerspectiveFovProjection<S> where S: ScalarFloat {
    type Spec = PerspectiveFovSpec<S>;
    type Projection = Matrix4x4<S>;

    fn from_spec(spec: &Self::Spec) -> Self {
        let matrix = Matrix4x4::from_perspective_fov(
            spec.fovy, 
            spec.aspect, 
            spec.near, 
            spec.far
        );

        PerspectiveFovProjection {
            fovy: spec.fovy,
            aspect: spec.aspect,
            near: spec.near,
            far: spec.far,
            matrix: matrix,
        }
    }

    #[inline]
    fn projection(&self) -> &Self::Projection {
        &self.matrix
    }

    fn update(&mut self, width: usize, height: usize) {
        let width_float = cglinalg::num_traits::cast::<usize, S>(width).unwrap();
        let height_float = cglinalg::num_traits::cast::<usize, S>(height).unwrap();
        self.aspect = width_float / height_float;
        self.matrix = Matrix4x4::from_perspective_fov(
            self.fovy, 
            self.aspect, 
            self.near, 
            self.far
        );
    }
}


/// A perspective projection based on arbitrary `left`, `right`, `bottom`,
/// `top`, `near`, and `far` planes.
///
/// We assume the following constraints to construct a useful perspective 
/// projection
/// ```text
/// left   < right
/// bottom < top
/// near   < far   (along the negative z-axis)
/// ```
/// Each parameter in the specification is a description of the position along
/// an axis of a plane that the axis is perpendicular to.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub struct PerspectiveSpec<S> {
    /// The horizontal position of the left-hand plane in camera space.
    /// The left-hand plane is a plane parallel to the **yz-plane** at
    /// the origin.
    left: S,
    /// The horizontal position of the right-hand plane in camera space.
    /// The right-hand plane is a plane parallel to the **yz-plane** at
    /// the origin.
    right: S,
    /// The vertical position of the bottom plane in camera space.
    /// The bottom plane is a plane parallel to the **xz-plane** at the origin.
    bottom: S,
    /// The vertical position of the top plane in camera space.
    /// the top plane is a plane parallel to the **xz-plane** at the origin.
    top: S,
    /// The distance along the **negative z-axis** of the near plane from the eye.
    /// The near plane is a plane parallel to the **xy-plane** at the origin.
    near: S,
    /// the distance along the **negative z-axis** of the far plane from the eye.
    /// The far plane is a plane parallel to the **xy-plane** at the origin.
    far: S,
}

impl<S> PerspectiveSpec<S> {
    /// Construct a new perspective specification.
    #[inline]
    pub const fn new(left: S, right: S, bottom: S, top: S, near: S, far: S) -> PerspectiveSpec<S> {
        PerspectiveSpec {
            left: left,
            right: right,
            bottom: bottom,
            top: top,
            near: near,
            far: far,
        }
    }
}

impl<S> fmt::Display for PerspectiveSpec<S> where S: fmt::Display {
    fn fmt(&self, formatter: &mut fmt::Formatter) -> fmt::Result {
        write!(
            formatter,
            "PerspectiveSpec [left={}, right={}, bottom={}, top={}, near={}, far={}]",
            self.left, self.right, self.bottom, self.top, self.near, self.far
        )
    }
}

/// A perspective projection transformation for converting from camera space to
/// normalized device coordinates.
///
/// Orthographic projections differ from perspective projections because
/// orthographic projections keeps parallel lines parallel, whereas perspective 
/// projections preserve the perception of distance. Perspective 
/// projections preserve the spatial ordering of points in the distance they 
/// are located from the viewing plane. This property of perspective projection 
/// transformations is important for operations such as z-buffering and 
/// occlusion detection.
#[repr(C)]
#[derive(Clone, Debug)]
pub struct PerspectiveProjection<S> {
    /// The horizontal position of the left-hand plane in camera space.
    /// The left-hand plane is a plane parallel to the **yz-plane** at
    /// the origin.
    left: S,
    /// The horizontal position of the right-hand plane in camera space.
    /// The right-hand plane is a plane parallel to the **yz-plane** at
    /// the origin.
    right: S,
    /// The vertical position of the bottom plane in camera space.
    /// The bottom plane is a plane parallel to the **xz-plane** at the origin.
    bottom: S,
    /// The vertical position of the top plane in camera space.
    /// the top plane is a plane parallel to the **xz-plane** at the origin.
    top: S,
    /// The distance along the **negative z-axis** of the near plane from the eye.
    /// The near plane is a plane parallel to the **xy-plane** at the origin.
    near: S,
    /// the distance along the **negative z-axis** of the far plane from the eye.
    /// The far plane is a plane parallel to the **xy-plane** at the origin.
    far: S,
    /// The underlying perspective projection matrix.
    matrix: Matrix4x4<S>
}

impl<S> PerspectiveProjection<S> {
    /// Returns a reference to the underlying perspective projection matrix.
    #[inline]
    pub fn to_matrix(&self) -> &Matrix4x4<S> {
        &self.matrix
    }
}

impl<S> fmt::Display for PerspectiveProjection<S> where S: fmt::Display {
    fn fmt(&self, formatter: &mut fmt::Formatter) -> fmt::Result {
        write!(
            formatter,
            "PerspectiveProjection [{}]",
            self.matrix
        )
    }
}

impl<S> CameraModel for PerspectiveProjection<S> where S: ScalarFloat {
    type Spec = PerspectiveSpec<S>;
    type Projection = Matrix4x4<S>;

    fn from_spec(spec: &Self::Spec) -> Self {
        let matrix = Matrix4x4::from_perspective(
            spec.left, 
            spec.right, 
            spec.bottom, 
            spec.top,
            spec.near,
            spec.far
        );

        PerspectiveProjection {
            left: spec.left,
            right: spec.right,
            bottom: spec.bottom,
            top: spec.top,
            near: spec.near,
            far: spec.far,
            matrix: matrix,
        }
    }

    #[inline]
    fn projection(&self) -> &Self::Projection {
        &self.matrix
    }

    fn update(&mut self, width: usize, height: usize) {
        let width_float = cglinalg::num_traits::cast::<usize, S>(width).unwrap();
        let height_float = cglinalg::num_traits::cast::<usize, S>(height).unwrap();
        unimplemented!();
    }
}


/// A description of an orthographic projection with arbitrary `left`, `right`, 
/// `top`, `bottom`, `near`, and `far` planes.
///
/// We assume the following constraints to construct a useful orthographic 
/// projection
/// ```text
/// left   < right
/// bottom < top
/// near   < far   (along the negative z-axis).
/// ```
/// Each parameter in the specification is a description of the position along 
/// an axis of a plane that the axis is perpendicular to.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub struct OrthographicSpec<S> {
    /// The horizontal position of the left-hand plane in camera space.
    /// The left-hand plane is a plane parallel to the **yz-plane** at
    /// the origin.
    left: S,
    /// The horizontal position of the right-hand plane in camera space.
    /// The right-hand plane is a plane parallel to the **yz-plane** at
    /// the origin.
    right: S,
    /// The vertical position of the **bottom plane** in camera space.
    /// The bottom plane is a plane parallel to the **xz-plane** at the origin.
    bottom: S,
    /// The vertical position of the **top plane** in camera space.
    /// the top plane is a plane parallel to the **xz-plane** at the origin.
    top: S,
    /// The distance along the **negative z-axis** of the **near plane** from the eye.
    /// The near plane is a plane parallel to the **xy-plane** at the origin.
    near: S,
    /// the distance along the **negative z-axis** of the **far plane** from the eye.
    /// The far plane is a plane parallel to the **xy-plane** at the origin.
    far: S,
}

impl<S> OrthographicSpec<S> {
    /// Construct a new orthographic specification.
    #[inline]
    pub const fn new(left: S, right: S, bottom: S, top: S, near: S, far: S) -> OrthographicSpec<S> {
        OrthographicSpec {
            left: left,
            right: right,
            bottom: bottom,
            top: top,
            near: near,
            far: far,
        }
    }
}

impl<S> fmt::Display for OrthographicSpec<S> where S: fmt::Display {
    fn fmt(&self, formatter: &mut fmt::Formatter) -> fmt::Result {
        write!(
            formatter,
            "OrthographicSpec [left={}, right={}, bottom={}, top={}, near={}, far={}]",
            self.left, self.right, self.bottom, self.top, self.near, self.far
        )
    }
}

/// An orthographic projection transformation for converting from camera space to
/// normalized device coordinates. 
///
/// Orthographic projections differ from perspective projections in that 
/// orthographic projections keeps parallel lines parallel, whereas perspective 
/// projections preserve the perception of distance. Perspective 
/// projections preserve the spatial ordering in the distance that points are 
/// located from the viewing plane.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct OrthographicProjection<S> {
    /// The horizontal position of the left-hand plane in camera space.
    /// The left-hand plane is a plane parallel to the **yz-plane** at
    /// the origin.
    left: S,
    /// The horizontal position of the right-hand plane in camera space.
    /// The right-hand plane is a plane parallel to the **yz-plane** at
    /// the origin.
    right: S,
    /// The vertical position of the **bottom plane** in camera space.
    /// The bottom plane is a plane parallel to the **xz-plane** at the origin.
    bottom: S,
    /// The vertical position of the **top plane** in camera space.
    /// the top plane is a plane parallel to the **xz-plane** at the origin.
    top: S,
    /// The distance along the **negative z-axis** of the **near plane** from the eye.
    /// The near plane is a plane parallel to the **xy-plane** at the origin.
    near: S,
    /// the distance along the **negative z-axis** of the **far plane** from the eye.
    /// The far plane is a plane parallel to the **xy-plane** at the origin.
    far: S,
    /// The underlying matrix that implements the orthographic projection.
    matrix: Matrix4x4<S>,
}

impl<S> OrthographicProjection<S> where S: ScalarFloat {
    /// Get the underlying matrix implementing the orthographic transformation.
    #[inline]
    pub fn to_matrix(&self) -> &Matrix4x4<S> {
        &self.matrix
    }
}

impl<S> CameraModel for OrthographicProjection<S> where S: ScalarFloat {
    type Spec = OrthographicSpec<S>;
    type Projection = Matrix4x4<S>;

    fn from_spec(spec: &Self::Spec) -> Self {
        let matrix = Matrix4x4::from_orthographic(
            spec.left, 
            spec.right, 
            spec.bottom, 
            spec.top,
            spec.near,
            spec.far
        );

        OrthographicProjection {
            left: spec.left,
            right: spec.right,
            bottom: spec.bottom,
            top: spec.top,
            near: spec.near,
            far: spec.far,
            matrix: matrix,
        }
    }

    #[inline]
    fn projection(&self) -> &Self::Projection {
        &self.matrix
    }

    fn update(&mut self, width: usize, height: usize) {
        let width_float = cglinalg::num_traits::cast::<usize, S>(width).unwrap();
        let height_float = cglinalg::num_traits::cast::<usize, S>(height).unwrap();
        unimplemented!();
    }
}


#[derive(Clone, Debug)]
pub struct CameraAttitudeSpec<S> {
    position: Vector3<S>,
    forward: Vector3<S>,
    right: Vector3<S>,
    up: Vector3<S>,
    axis: Vector3<S>,
}

impl<S> CameraAttitudeSpec<S> where S: ScalarFloat {
    #[inline]
    pub fn new(
        position: Vector3<S>,
        forward: Vector3<S>,
        right: Vector3<S>,
        up: Vector3<S>,
        axis: Vector3<S>) -> Self {

        CameraAttitudeSpec {
            position: position,
            forward: forward,
            right: right,
            up: up,
            axis: axis,
        }
    }
}

#[repr(C)]
#[derive(Clone, Debug)]
struct CameraAttitude<S> {
    position: Vector3<S>,
    forward: Vector4<S>,
    right: Vector4<S>,
    up: Vector4<S>,
    axis: Quaternion<S>,
    translation_matrix: Matrix4x4<S>,
    rotation_matrix: Matrix4x4<S>,
    view_matrix: Matrix4x4<S>,
}

impl<S> CameraAttitude<S> where S: ScalarFloat {
    fn from_spec(spec: &CameraAttitudeSpec<S>) -> CameraAttitude<S> {
        let axis = Quaternion::from_parts(S::zero(), spec.axis);
        let translation_matrix = Matrix4x4::from_affine_translation(
            &(-spec.position)
        );
        let rotation_matrix = Matrix4x4::from(&axis);
        let view_matrix = rotation_matrix * translation_matrix;

        CameraAttitude {
            position: spec.position,
            forward: spec.forward.expand(S::zero()),
            right: spec.right.expand(S::zero()),
            up: spec.up.expand(S::zero()),
            axis: axis,
            translation_matrix: translation_matrix,
            rotation_matrix: rotation_matrix,
            view_matrix: view_matrix,
        }

    }

    /// Get the camera's up direction in camera space.
    #[inline]
    fn up_axis_eye(&self) -> Vector3<S> {
        let zero = S::zero();
        let one = S::one();

        Vector3::new(zero, one, zero)
    }
        
    /// Get the camera's right axis in camera space.
    #[inline]
    fn right_axis_eye(&self) -> Vector3<S> {
        let zero = S::zero();
        let one = S::one();
        
        Vector3::new(one, zero ,zero)
    }
        
    /// Get the camera's forward axis in camera space.
    #[inline]
    fn forward_axis_eye(&self) -> Vector3<S> {
        let zero = S::zero();
        let one = S::one();
        
        Vector3::new(zero, zero, -one)
    }

    #[inline]
    fn update_movement<K: CameraKinematics<S>>(
        &mut self, kinematics: &K, movement: CameraMovement, elapsed: S) {
        
        let delta_attitude = kinematics.update(movement, elapsed);
        self.update(&delta_attitude);
    }

    // Update the camera position.
    #[inline]
    fn update_position(&mut self, delta_attitude: &DeltaAttitude<S>) {
        self.position += self.forward.contract() * -delta_attitude.z;
        self.position += self.up.contract()      *  delta_attitude.y;
        self.position += self.right.contract()   *  delta_attitude.x;

        let translation_inv = Matrix4x4::from_affine_translation(
            &self.position
        );
        self.translation_matrix = translation_inv.inverse().unwrap();
    }

    // Update the camera axes so we can rotate the camera about the new rotation axes.
    #[inline]
    fn update_orientation(&mut self, delta_attitude: &DeltaAttitude<S>) {
        let axis_yaw = Unit::from_value(self.up.contract());
        let q_yaw = Quaternion::from_axis_angle(
            &axis_yaw, Degrees(delta_attitude.yaw)
        );
        self.axis = q_yaw * self.axis;

        let axis_pitch = Unit::from_value(self.right.contract());
        let q_pitch = Quaternion::from_axis_angle(
            &axis_pitch, Degrees(delta_attitude.pitch)
        );
        self.axis = q_pitch * self.axis;

        let axis_roll = Unit::from_value(self.forward.contract());
        let q_roll = Quaternion::from_axis_angle(
            &axis_roll, Degrees(delta_attitude.roll), 
        );
        self.axis = q_roll * self.axis;

        let rotation_matrix_inv = Matrix4x4::from(&self.axis);
        self.forward = rotation_matrix_inv * self.forward_axis_eye().expand(S::zero());
        self.right   = rotation_matrix_inv * self.right_axis_eye().expand(S::zero());
        self.up      = rotation_matrix_inv * self.up_axis_eye().expand(S::zero());
        self.rotation_matrix = rotation_matrix_inv.inverse().unwrap();
    }

    #[inline]
    fn update(&mut self, delta_attitude: &DeltaAttitude<S>) {
        self.update_orientation(delta_attitude);
        self.update_position(delta_attitude);
        self.view_matrix = self.rotation_matrix * self.translation_matrix;
    }

    #[inline]
    fn view_matrix(&self) -> &Matrix4x4<S> {
        &self.view_matrix
    }
}

#[derive(Copy, Clone, Debug)]
pub struct FreeKinematicsSpec<S> {
    movement_speed: S,
    rotation_speed: S,
}

impl<S> FreeKinematicsSpec<S> where S: ScalarFloat {
    #[inline]
    pub fn new(movement_speed: S, rotation_speed: S) -> FreeKinematicsSpec<S> {
        FreeKinematicsSpec {
            movement_speed: movement_speed,
            rotation_speed: rotation_speed,
        }
    }
}

#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct FreeKinematics<S> {
    movement_speed: S,
    rotation_speed: S,
}

impl<S> CameraKinematics<S> for FreeKinematics<S> where S: ScalarFloat {
    type Spec = FreeKinematicsSpec<S>;

    #[inline]
    fn from_spec(spec: &Self::Spec) -> Self {
        FreeKinematics {
            movement_speed: spec.movement_speed,
            rotation_speed: spec.rotation_speed,
        }
    }

    #[inline]
    fn update(&self, movement: CameraMovement, elapsed: S) -> DeltaAttitude<S> {
        let mut delta_attitude = DeltaAttitude::zero();

        if movement.total & MOVE_LEFT != 0 {
            delta_attitude.x -= self.movement_speed * elapsed;
        }
        if movement.total & MOVE_RIGHT != 0 {
            delta_attitude.x += self.movement_speed * elapsed;
        }
        if movement.total & MOVE_UP != 0 {
            delta_attitude.y -= self.movement_speed * elapsed;
        }
        if movement.total & MOVE_DOWN != 0 {
            delta_attitude.y += self.movement_speed * elapsed;
        }
        if movement.total & MOVE_FORWARD != 0 {
            // We subtract along z-axis to move forward because the
            // forward axis is the (-z) direction in camera space.
            delta_attitude.z -= self.movement_speed * elapsed;
        }
        if movement.total & MOVE_BACKWARD != 0 {
            // We add along the z-axis to move backward because the
            // forward axis is the (-z) direction in camera space.
            delta_attitude.z += self.movement_speed * elapsed;
        }

        if movement.total & PITCH_UP != 0 {
            delta_attitude.pitch += self.rotation_speed * elapsed;
        }
        if movement.total & PITCH_DOWN != 0 {
            delta_attitude.pitch -= self.rotation_speed * elapsed;
        }
        if movement.total & YAW_LEFT != 0 {
            delta_attitude.yaw += self.rotation_speed * elapsed;
        }
        if movement.total & YAW_RIGHT != 0 {
            delta_attitude.yaw -= self.rotation_speed * elapsed;
        }
        if movement.total & ROLL_CLOCKWISE != 0 {
            delta_attitude.roll += self.rotation_speed * elapsed;
        }
        if movement.total & ROLL_COUNTERCLOCKWISE != 0 {
            delta_attitude.roll -= self.rotation_speed * elapsed;
        }

        delta_attitude
    }
}

#[repr(C)]
#[derive(Clone, Debug)]
pub struct Camera<S, M, K> {
    model: M,
    attitude: CameraAttitude<S>,
    kinematics: K,
}

impl<S, M, K> Camera<S, M, K> 
    where S: ScalarFloat,
          M: CameraModel,
          K: CameraKinematics<S>,
{
    pub fn new(
        model_spec: &M::Spec, 
        attitude_spec: &CameraAttitudeSpec<S>, 
        kinematics_spec: &K::Spec) -> Self {

        Camera {
            model: <M as CameraModel>::from_spec(model_spec),
            attitude: CameraAttitude::from_spec(attitude_spec),
            kinematics: <K as CameraKinematics<S>>::from_spec(kinematics_spec)
        }
    }

    pub fn update_movement(&mut self, movement: CameraMovement, elapsed_seconds: S) {
        self.attitude.update_movement(&self.kinematics, movement, elapsed_seconds);
    }

    pub fn update_viewport(&mut self, width: usize, height: usize) {
        self.model.update(width, height);
    }

    pub fn update_attitude(&mut self, delta_attitude: &DeltaAttitude<S>) {
        self.attitude.update(delta_attitude);
    }

    /// Get the camera's eye position in world space.
    #[inline]
    pub fn position(&self) -> Vector3<S> { 
        self.attitude.position
    }
    
    /// Get the camera's up direction in world space.
    #[inline]
    pub fn up_axis(&self) -> Vector3<S> {
        self.attitude.up.contract()
    }
    
    /// Get the camera's right axis in world space.
    #[inline]
    pub fn right_axis(&self) -> Vector3<S> {
        self.attitude.right.contract()
    }
    
    /// Get the camera's forward axis in world space.
    #[inline]
    pub fn forward_axis(&self) -> Vector3<S> {
        self.attitude.forward.contract()
    }
    
    /// Get the camera's up direction in camera space.
    #[inline]
    pub fn up_axis_eye(&self) -> Vector3<S> {
        self.attitude.up_axis_eye()
    }
        
    /// Get the camera's right axis in camera space.
    #[inline]
    pub fn right_axis_eye(&self) -> Vector3<S> {
        self.attitude.right_axis_eye()
    }
        
    /// Get the camera's forward axis in camera space.
    #[inline]
    pub fn forward_axis_eye(&self) -> Vector3<S> {
        self.attitude.forward_axis_eye()
    }
    
    /// Get the camera's axis of rotation.
    #[inline]
    pub fn axis(&self) -> Vector3<S> {
        self.attitude.axis.v
    }

    #[inline]
    pub fn view_matrix(&self) -> &Matrix4x4<S> {
        self.attitude.view_matrix()
    }

    #[inline]
    pub fn projection(&self) -> &M::Projection {
        self.model.projection()
    }
}

