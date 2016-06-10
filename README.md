# Dynamics 2.0

This is a draft showing an example API implementing the model of a three links
pendulum. While link1 and link2 are rigid, the third link (link3) is elastic and
 modeled using modal decomposition using the first three modes.
The main concepts introduced are:
 - `MBDWorld`: the Dynamics 2.0 manager of all bodies, joints, forces, and
 collision elements in the world. It makes sure to "globally" connect
 (or not if not needed) all of these objects and assign them with proper global
 indexes to position their states in the global composite state vector.
 - `CollisionDispatcher`. The `MBDWorld` owns it. It is responsible to implement
 the collision dispatch of the model. In general it wont be necessary to expose
  it to the user. In this example we do so to show this is owned by the
  `MBDWorld` and to show how options could be passed to it.
 - `MBDSystem`: It allows to create complex MBD models by composition
 (it inherits from `Diagram`). It also allows to encapsulate a very complex
 model by inheriting from this class.
 Take Atlas as an example. We would put all of its code in an AtlasSystem
 inheriting from `MBDSystem`. This `AtlasSystem` can then be instantiated as
 many times as needed.
 - `MultiBodyTree`: It is the general purpose `RigidBodyTree` that supports a
 mixture of rigid as well as flexible (soft) bodies.
 - `RigidBody`: Inherits from `Body`.
 - `ElasticBody`: Inherits from `Body`. Implements deformable bodies using modal
 decomposition.
 - Each object is instantiated through a factory. This avoids the user making
 explicit `new` calls and hides the ownership handling from the user. The user
 does not need to worry about cleanup.
 - `MBDWorldSystem`: The `ContinuousSystem` implementing multibody dynamics through
 `SystemInterface<T>` (System 2.0 concept). This `MBDWorldSystem` will allow to
 compose MBD systems out of simpler MBD systems by composition. Therefore this
 will provide support for multiple, very complex, robots composed of hundreds of
  components including bodies, forces, actuators, sensors, controllers, etc.
 - The example shows how "global" connectivities between elements would be
 computed at an `Initialize` stage (L. 123).
 - Once the system is created a `Context` is created (System 2.0 concept).
 - The context is then used on every query by the user and will also be used by
 the system's integrators (current PR #2513 for spring-mass system).
 - The proposed draft shows every object templated on the scalar type (`<T>`).
 This however needs to be revisited.
 - `Frame`. It provides a nice representation for coordinate frames. Right now
 we use `Eigen::Isometry3d`. A `Frame` class could wrap an `Isometry3d` and also
 provide a number of convenient functionalities for coordinate transformations.
 Should we provide frame checking?
