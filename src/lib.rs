use cglinalg::{
    Degrees,
    Zero,
    Vector3,
    Vector4,
    Matrix4, 
    Quaternion,
    ScalarFloat,
    InvertibleSquareMatrix,
    Unit,
};
use std::fmt;
use std::ops;


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
    fn add_assign(&mut self, other: SimpleCameraMovement) {
        *self = self.add_movement(other)
    }
}

impl ops::SubAssign<SimpleCameraMovement> for CameraMovement {
    fn sub_assign(&mut self, other: SimpleCameraMovement) {
        *self = self.subtract_movement(other)
    }
}


#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct CameraSpecification<S> {
    near: S,
    far: S,
    fovy: Degrees<S>,
    aspect: S,
}

impl<S> CameraSpecification<S> where S: ScalarFloat {
    pub fn new(near: S, far: S, fovy: Degrees<S>, aspect: S) -> CameraSpecification<S> {
        CameraSpecification {
            near: near,
            far: far,
            fovy: fovy,
            aspect: aspect,
        }
    }
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub struct CameraKinematics<S> {
    speed: S,
    yaw_speed: S,
    position: Vector3<S>,
    forward: Vector4<S>,
    right: Vector4<S>,
    up: Vector4<S>,
    axis: Quaternion<S>,
}

impl<S> CameraKinematics<S> where S: ScalarFloat {
    pub fn new(
        speed: S, 
        yaw_speed: S, 
        position: Vector3<S>, 
        forward: Vector4<S>, 
        right: Vector4<S>, 
        up: Vector4<S>, 
        axis: Quaternion<S>) -> CameraKinematics<S> {

        CameraKinematics {
            speed: speed,
            yaw_speed: yaw_speed,
            position: position,
            forward: forward,
            right: right,
            up: up,
            axis: axis,
        }

    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct CameraAttitude<S> {
    pub roll: S,
    pub pitch: S,
    pub yaw: S,
}

impl<S> CameraAttitude<S> where S: ScalarFloat {
    #[inline]
    pub fn new(roll: S, pitch: S, yaw: S) -> CameraAttitude<S> {
        CameraAttitude {
            roll: roll,
            pitch: pitch,
            yaw: yaw,
        }
    }
}


#[derive(Clone, Debug)]
pub struct Camera<S> {
    // Camera specification parameters.
    pub near: S,
    pub far: S,
    pub fovy: Degrees<S>,
    pub aspect: S,

    // Camera kinematics.
    pub movement_speed: S,
    pub rotation_speed: S,
    position: Vector3<S>,
    forward: Vector4<S>,
    right: Vector4<S>,
    up: Vector4<S>,
    axis: Quaternion<S>,

    // Camera matrices.
    pub proj_mat: Matrix4<S>,
    pub trans_mat: Matrix4<S>,
    pub rot_mat: Matrix4<S>,
    pub view_mat: Matrix4<S>,
}

impl<S> fmt::Display for Camera<S> where S: ScalarFloat + fmt::Display {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        let mut output = String::from("Camera model:\n");
        output.push_str(&format!("near: {}\n", self.near));
        output.push_str(&format!("far: {}\n", self.far));
        output.push_str(&format!("aspect: {}\n", self.aspect));
        output.push_str(&format!("speed: {}\n", self.movement_speed));
        output.push_str(&format!("yaw_speed: {}\n", self.rotation_speed));
        output.push_str(&format!("position: {}\n", self.position));
        output.push_str(&format!("forward: {}\n", self.forward));
        output.push_str(&format!("right: {}\n", self.right));
        output.push_str(&format!("up: {}\n", self.up));
        output.push_str(&format!("axis: {}\n", self.axis));
        output.push_str(&format!("proj_mat: {}\n", self.proj_mat));
        output.push_str(&format!("trans_mat: {}\n", self.trans_mat));
        output.push_str(&format!("rot_mat: {}\n", self.rot_mat));
        output.push_str(&format!("view_mat: {}\n", self.view_mat));
        writeln!(f, "{}", output)
    }
}

impl<S> Camera<S> where S: ScalarFloat {
    /// Construct a new camera.
    pub fn new(spec: CameraSpecification<S>, kinematics: CameraKinematics<S>) -> Camera<S> {
        let proj_mat = Matrix4::from_perspective_fov(spec.fovy, spec.aspect, spec.near, spec.far);
        let trans_mat = Matrix4::from_affine_translation(&(-kinematics.position));
        let rot_mat = Matrix4::from(kinematics.axis);
        let view_mat = rot_mat * trans_mat;

        Camera {
            near: spec.near,
            far: spec.far,
            fovy: spec.fovy,
            aspect: spec.aspect,

            movement_speed: kinematics.speed,
            rotation_speed: kinematics.yaw_speed,
            position: kinematics.position,
            forward: kinematics.forward,
            right: kinematics.right,
            up: kinematics.up,
            axis: kinematics.axis,

            proj_mat: proj_mat,
            trans_mat: trans_mat,
            rot_mat: rot_mat,
            view_mat: view_mat,
        }
    }

    /// Get the camera's eye position in world space.
    #[inline]
    pub fn position(&self) -> Vector3<S> { 
        self.position
    }

    /// Get the camera's up direction in world space.
    #[inline]
    pub fn up_axis(&self) -> Vector3<S> {
        Vector3::new(self.up.x, self.up.y, self.up.z)
    }

    /// Get the camera's right axis in world space.
    #[inline]
    pub fn right_axis(&self) -> Vector3<S> {
        Vector3::new(self.right.x, self.right.y, self.right.z)
    }

    /// Get the camera's forward axis in world space.
    #[inline]
    pub fn forward_axis(&self) -> Vector3<S> {
        Vector3::new(self.forward.x, self.forward.y, self.forward.z)
    }

    /// Get the camera's up direction in camera space.
    #[inline]
    pub fn up_axis_eye(&self) -> Vector3<S> {
        let zero = S::zero();
        let one = S::one();
        Vector3::new(zero, one, zero)
    }
    
    /// Get the camera's right axis in camera space.
    #[inline]
    pub fn right_axis_eye(&self) -> Vector3<S> {
        let zero = S::zero();
        let one = S::one();
        Vector3::new(one, zero ,zero)
    }
    
    /// Get the camera's forward axis in camera space.
    #[inline]
    pub fn forward_axis_eye(&self) -> Vector3<S> {
        let zero = S::zero();
        let one = S::one();
        Vector3::new(zero, zero, -one)
    }

    /// Get the camera's axis or rotation.
    #[inline]
    pub fn axis(&self) -> Quaternion<S> {
        self.axis
    }

    /// Update the camera position and attitude based on the input camera movements.
    /// All movements are updated in the camera's coordinate system.
    #[inline]
    pub fn update_movement(&mut self, movement: CameraMovement, elapsed_seconds: S) {
        let mut delta_position = Vector3::zero();
        if movement.total & MOVE_LEFT != 0 {
            delta_position.x -= self.movement_speed * elapsed_seconds;
        }
        if movement.total & MOVE_RIGHT != 0 {
            delta_position.x += self.movement_speed * elapsed_seconds;
        }
        if movement.total & MOVE_UP != 0 {
            delta_position.y -= self.movement_speed * elapsed_seconds;
        }
        if movement.total & MOVE_DOWN != 0 {
            delta_position.y += self.movement_speed * elapsed_seconds;
        }
        if movement.total & MOVE_FORWARD != 0 {
            // We subtract along z-axis to move forward because the
            // forward axis is the (-z) direction in camera space.
            delta_position.z -= self.movement_speed * elapsed_seconds;
        }
        if movement.total & MOVE_BACKWARD != 0 {
            // We add along the z-axis to move backward because the
            // forward axis is the (-z) direction in camera space.
            delta_position.z += self.movement_speed * elapsed_seconds;
        }

        let mut delta_attitude = CameraAttitude::new(S::zero(), S::zero(), S::zero());
        if movement.total & PITCH_UP != 0 {
            delta_attitude.pitch += self.rotation_speed * elapsed_seconds;
        }
        if movement.total & PITCH_DOWN != 0 {
            delta_attitude.pitch -= self.rotation_speed * elapsed_seconds;
        }
        if movement.total & YAW_LEFT != 0 {
            delta_attitude.yaw += self.rotation_speed * elapsed_seconds;
        }
        if movement.total & YAW_RIGHT != 0 {
            delta_attitude.yaw -= self.rotation_speed * elapsed_seconds;
        }
        if movement.total & ROLL_CLOCKWISE != 0 {
            delta_attitude.roll += self.rotation_speed * elapsed_seconds;
        }
        if movement.total & ROLL_COUNTERCLOCKWISE != 0 {
            delta_attitude.roll -= self.rotation_speed * elapsed_seconds;
        }

        self.update(delta_position, delta_attitude);
    }

    /// Update the camera viewport dimensions in case the viewport dimensions have changed.
    #[inline]
    pub fn update_viewport(&mut self, width: u32, height: u32) {
        let width_float = cglinalg::num_traits::cast::<u32, S>(width).unwrap();
        let height_float = cglinalg::num_traits::cast::<u32, S>(height).unwrap();
        self.aspect = width_float / height_float;
        self.proj_mat = Matrix4::from_perspective_fov(self.fovy, self.aspect, self.near, self.far);
    }

    /// Update the camera position and attitude in world space.
    #[inline]
    pub fn update(&mut self, delta_position: Vector3<S>, delta_attitude: CameraAttitude<S>) {
        // Update the axis of rotation of the camera.
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

        // Update the camera axes so we can rotate the camera about the new rotation axes.
        let zero = S::zero();
        let rot_mat_inv = Matrix4::from(self.axis);
        self.forward = rot_mat_inv * self.forward_axis_eye().expand(zero);
        self.right   = rot_mat_inv * self.right_axis_eye().expand(zero);
        self.up      = rot_mat_inv * self.up_axis_eye().expand(zero);

        // Update the camera position.
        self.position += self.forward.contract() * -delta_position.z;
        self.position += self.up.contract()      *  delta_position.y;
        self.position += self.right.contract()   *  delta_position.x;

        // Update the camera matrices.
        let trans_mat_inv = Matrix4::from_affine_translation(&self.position);
        self.rot_mat = rot_mat_inv.inverse().unwrap();
        self.trans_mat = trans_mat_inv.inverse().unwrap();
        self.view_mat = self.rot_mat * self.trans_mat;
    }
}
