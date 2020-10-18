# cgperspective

## Introduction
**cgperspective** is a camera model library written in Rust for real-time 
computer graphics applications. It provides a flexible camera model with the
following characteristics:

* A generic kinematics property for describing how to move the camera in world 
  space in response to control inputs.
* A generic camera model property for using different camera models to render
  different kinds of effects. The most common use case is either a perspective 
  projection or orthographic project pinhole camera model but it can accommodate
  more sophisticated camera models as well.

## Using **cgperspective**
Make certain you have the latest version of the 
[rust compiler](https://www.rust-lang.org) and the official package manager 
[cargo](https://github.com/rust-lang/cargo).

After that, add the following line to your `Cargo.toml` file:
```ignore
[dependencies]
cgperspective = "0.1.14"
```

## Features
**cgperspective** is designed to be a building block for real-time 
computer graphics programs built on top of a graphics API such as OpenGL,
Vulkan, or Direct3D. Common use cases would be small games, tech demos,
or as a building block for some kind of runtime environment. It is not meant
to be used with an existing three-dimensional development framework, as those
typically provide their own camera models. 

The library provides the following features:

* The `CameraKinematics` trait allows the user of the library to supply
  a custom method of moving the camera around a scene.
* The `CameraModel` trait describes how the camera transforms incoming light
  from the scene into the eye into vectors in normalized device coordinates.
  This allows the use of custom lighting effects such as depth of field and 
  occlusion culling.
* Several default camera models for traditional pinhole cameras.
* A `CameraMovement` type that describes how the camera moves between each frame.
  This allows the camera model to be windowing system agnostic because camera 
  attitude updates are not coupled to the windowing system input processing system.
* A generic `Camera` type that enables generic programming of the camera type.
  The `Camera` type will operate over any floating point type compatible with the 
  `cglinalg` library, typically floating point or double precision floating point. 
  The camera is also generic over the choice of camera kinematics and camera model;
  the library user can use a combination of the models provided by the library, or
  supply their own by implementing the `CameraKinematics` and `CameraModel` traits 
  for their respective custom to types.

