# Dynamics 2.0

This is a draft showing an example API implementing the model of a three links pendulum. 
While link1 and link2 are rigid, the third link (link3) is elastic and modeled used modal decomposition using the first three modes.
The main concepts introduced are:
 - MBDWorld: the Dynamics 2.0 manager of all bodies, joints, forces, and collision elements in the world. 
 - CollisionDispatcher. The MBDWorld owns it. It is responsible to implement the collision dispatch of the model. In general it wont be necessary to expose it to the user. In this example we do so to show this is owned by the MBDWorld and how options could be passed.
 - MultiBodyTree: It is the general purpose RigidBodyTree that supports a mixture of rigid as well as flexible (soft) bodies.
 - Each object is instantiated through a factory. This avoids the user making explicit "new" calls and hides the ownership handling from the user. The user does not need to worry about cleanup. 
 - MBDWorldSystem: The ContinuousSystem implementing multibody dynamics through SystemInterface<T> (System 2.0 concept). This MBDWorldSystem will allow to compose MBD systems out of simpler MBD systems by composition. Therefore this will provide support for multiple, very complex, robots composed of hundreds of components including bodies, forces, actuators, sensors, controllers, etc.
 - The example shows how "global" connecitivities between elements wold be computed at an "Initialize" stage (L. 123). 
 - Once the system is created a "Context" is created (System 2.0 concept).
 - The context is then used on every query by the user and will also be used by the system's integrators (current PR #2513 for spring-mass system).
 - The proposed draft shows every object templated on the scalar type (<T>). This however needs to be revisited. 