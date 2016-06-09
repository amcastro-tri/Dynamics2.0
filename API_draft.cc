#include <iostream>
#include <memory>

// We could lump these into a common header file.
#include "drake/physics/multibody_dyanmics/mbd.h"
#include "drake/math/time_steppers/rk4.h"
#include "drake/core/logging/logger.h"

typedef double Real;

// My particular problem size, if auto-differentiation is needed.
// For efficiency, it is best to use a fixed size derivatives vector with the
// proper dimension for your problem.
typedef 87 MY_FIXED_SIZE;
typedef AutoDiffScalar<Eigen::Vector<Real,MY_FIXED_SIZE>> ADScalar;

// The actual type of this problem setup
typedef Real T;
// We could've used autodiff scalars instead:
// typedef ADScalar T;

// Some convenient definitions
typedef Eigen::Vector<T,3,1> Vector3t;

int main()
{
  try {
    // The Drake-like system.
    // Drake objects are created using factories (static methods) to provide the 
    // user an "operator new"-free API while allowing the user to hold onto a
    // pointer to the allocated type.
    // With that pointer the user can perform queries on the object.
    // MBDWorld inherits from systems::ContinuousSystem<T>
    MBDWorldSystem<T>* mbd_world_system = MBDWorldSystem<T>::Create();
    
    // The multibody dynamics world. MBDSystem owns it.
    MBDWorld* world = mbd_world_system->world();

    CollisionDispatcher::Options dispatch_options;
    // ... set collision dispatching options here ...
    // ... Or not, just use default options ...

    // Just showing here that collision dispatcher belongs to the MBD world,
    // not to the MBDSystem.
    world->collision_dispatcher()->set_options(dispatch_options); 

    // Gravity and other force fields are global in scope and in principle
    // affect all bodies. It belongs to the world.
    // UniformForceField inherits from ForceField (could be electrostatic,
    // gravity, Lennard-Jones, etc)
    UniformForceField* gravity =
        UniformForceField::CreateAndAddToMBDWorld(world);
    gravity->set(Vector3t(0.0, 9.81, 0.0));
    
    MultibodyTree<T>* triple_pendulum =
        MultibodyTree<T>::CreateAndAddToMBDWorld(world);

    RigidBody<T>* link1 =
        RigidBody<T>::CreateAndAddToMultibodyTree(triple_pendulum);
    link1->set_mass(1.0);
    link1->set_gyradii(0.2,1.0,1.0);

    // We could make link2 identical to link1
    // RigidBody<T>* link2 =
    //     RigidBody<T>::CloneAndAddToMultibodyTree(triple_pendulum);
    // Or use parameters for CreateAndAdd
    RigidBody<T>* link2 =
        RigidBody<T>::CreateAndAddToMultibodyTree(
            triple_pendulum, 1.0, Vector3t(0.2, 1.0, 1.0));

    // The third link is elastic (link3 has state!!).
    // ElasticBody and RigidBody inherit from Body
    // ElasticBody will use modal decomposition to accurately represent elastic
    // deformations and their dynamics by just using a few extra DOF's.
    // Notice MultibodyTree supports having a mixture of RigidBody and
    // ElasticBody.
    ElasticBody<T>* link3 =
        ElasticBody<T>::CreateAndAddToMultibodyTree(triple_pendulum);

    // Modal decomposition can be pre-computed even with the most
    // complex/expensive FEM method as a pre-processing step!
    link3->set_mass(1.0);
    link3->set_gyradii(0.2,1.0,1.0);
    //only load the first three modes. (only three DOF's!)
    link3->load_modal_decomposition(
        "extremely_complex_body_modes_and_frequencies.modes", 3);

    // pin1 connects world (converts to Body*) to link1
    RevoluteJoint<T>* pin1 =
        RevoluteJoint<T>::CreateAndAddToMultibodyTree(
            triple_pendulum,
            world /* parent Body */,
            Frame::Identity()                 /* Frame on parent body */,
            link1 /* child Body  */,
            Frame::Translation(0.0, 1.0, 0.0) /* Frame on child body  */);

    // pin2 connects link1 to link2
    RevoluteJoint<T>* pin2 =
        RevoluteJoint<T>::CreateAndAddToMultibodyTree(
            triple_pendulum,
            world /* parent Body */,
            Frame::Identity()                 /* Frame on parent body */,
            link1 /* child Body  */,
            Frame::Translation(0.0, 1.0, 0.0) /* Frame on child body  */);

    // pin3 connects link2 to link3
    RevoluteJoint<T>* pin3 =
        RevoluteJoint<T>::CreateAndAddToMultibodyTree(
            triple_pendulum,
            link2 /* parent Body */,
            Frame::Identity()                 /* Frame on parent body */,
            link3 /* child Body  */,
            Frame::Translation(0.0, 1.0, 0.0) /* Frame on child body  */);

    // Computes/initializes:
    // - global connectivities.
    // - Dynamic memory allocation: ideally, this would be the last place where
    //   dynamic allocation happens.
    //   It should not happen at runtime. We could enforce this by disabling
    //   dynamic memory allocation on Debug builds (like Eigen does) so that a
    //   "new" throws an exception.
    //   On Release builds (already debugged) these checks are disabled.
    // After this call the user cannot modify the system anymore and attempts
    // to do so will throw.
    mbd_world_system->Initialize();    

    // This throws an exception after MBDWorldSystem::Initialize():
    // RigidBody<T>* link3 =
    //     RigidBody<T>::CreateAndAddToMultibodyTree(
    //         triple_pendulum, 1.0, Vector3t(0.2, 1.0, 1.0));

    std::unique_ptr<MBDWorldSystem<T>::Context> context =
        mbd_world_system->CreateDefaultContext();

    // Initial conditions.
    pin1->set_angle(context, 0.0);
    pin1->set_angular_velocity(context, 1.0);

    pin2->set_angle(context, drake::math::pi/2.0);
    pin2->set_angular_velocity(context, 0.0);

    pin3->set_angle(context, 0.0);
    pin3->set_angular_velocity(context, 0.0);

    
    // What follows is the solving stage.
    // ... Integrators/solvers setup here ...
    // ... Time stepping loop here ... 
    

  } catch (const std::exception& e) {
    std::cout << "Abnormal termination: " << e.what() << std::endl;
    return 1;
  }
  return 0;
}




