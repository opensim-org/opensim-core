/* -------------------------------------------------------------------------- *
 * OpenSim Muscollo: testConstraints.cpp                                      *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Nicholas Bianco                                                 *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0          *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

#include <Muscollo/osimMuscollo.h>
#include <OpenSim\Simulation\SimbodyEngine\UniversalJoint.h>
#include <OpenSim\Simulation\SimbodyEngine\BallJoint.h>
#include <OpenSim\Simulation\SimbodyEngine\GimbalJoint.h>
#include <OpenSim\Simulation\SimbodyEngine\WeldConstraint.h>
#include <OpenSim\Simulation\SimbodyEngine\PointConstraint.h>
#include <OpenSim\Simulation\SimbodyEngine\PointOnLineConstraint.h>
#include <OpenSim\Simulation\SimbodyEngine\ConstantDistanceConstraint.h>
#include <OpenSim\Common\LinearFunction.h>
#include <simbody\internal\Constraint.h>
#include <simbody\internal\Constraint_Ball.h>

using namespace OpenSim;
using SimTK::Vec3;
using SimTK::UnitVec3;
using SimTK::Vector;
using SimTK::State;

const int NUM_BODIES = 10;
const double BOND_LENGTH = 0.5;

/// Keep constraints satisfied to this tolerance during testing.
static const double ConstraintTol = 1e-10;

/// Compare two quantities that should have been calculated to machine tolerance
/// given the problem size, which we'll characterize by the number of mobilities
/// (borrowed from Simbody's 'testConstraints.cpp').
#define MACHINE_TEST(a,b) SimTK_TEST_EQ_SIZE(a,b, 10*state.getNU())

/// Create a model consisting of a chain of bodies. This model is nearly 
/// identical to the model implemented in Simbody's 'testConstraints.cpp'.
Model createModel() {
    Model model;
    const SimTK::Real mass = 1.23;
    Body* body = new Body("body0", mass, SimTK::Vec3(0.1, 0.2, -0.03),
              mass*SimTK::UnitInertia(1.1, 1.2, 1.3, 0.01, -0.02, 0.07));
    model.addBody(body);

    GimbalJoint* joint = new GimbalJoint("joint0",
        model.getGround(), Vec3(-0.1, 0.3, 0.2), Vec3(0.3, -0.2, 0.1),
        *body, Vec3(BOND_LENGTH, 0, 0), Vec3(-0.2, 0.1, -0.3));
    model.addJoint(joint);

    for (int i = 1; i < NUM_BODIES; ++i) {
        Body& parent = model.getBodySet().get(model.getNumBodies() - 1);

        std::string bodyName = "body" + std::to_string(i + 1);
        Body* body = new Body(bodyName, mass, SimTK::Vec3(0.1, 0.2, -0.03),
            mass*SimTK::UnitInertia(1.1, 1.2, 1.3, 0.01, -0.02, 0.07));
        model.addBody(body);

        std::string jointName = "joint" + std::to_string(i+1);
        if (i == NUM_BODIES-5) {
            UniversalJoint* joint = new UniversalJoint(jointName,
                parent, Vec3(-0.1, 0.3, 0.2), Vec3(0.3, -0.2, 0.1), 
                *body, Vec3(BOND_LENGTH, 0, 0), Vec3(-0.2, 0.1, -0.3));
            model.addJoint(joint);
        } else if (i == NUM_BODIES-3) {
            BallJoint* joint = new BallJoint(jointName,
                parent, Vec3(-0.1, 0.3, 0.2), Vec3(0.3, -0.2, 0.1),
                *body, Vec3(BOND_LENGTH, 0, 0), Vec3(-0.2, 0.1, -0.3));
            model.addJoint(joint);
        } else {
            GimbalJoint* joint = new GimbalJoint(jointName,
                parent, Vec3(-0.1, 0.3, 0.2), Vec3(0.3, -0.2, 0.1),
                *body, Vec3(BOND_LENGTH, 0, 0), Vec3(-0.2, 0.1, -0.3));
            model.addJoint(joint);
        }
    }
    
    return model;
}

/// Create a random state for the model. This implementation mimics the random
/// state creation in Simbody's 'testConstraints.cpp'.
void createState(Model& model, State& state, const Vector& qOverride=Vector()) {
    state = model.initSystem();
    SimTK::Random::Uniform random; 
    for (int i = 0; i < state.getNY(); ++i)
        state.updY()[i] = random.getValue();
    if (qOverride.size())
        state.updQ() = qOverride;
    model.realizeVelocity(state);

    model.updMultibodySystem().project(state, ConstraintTol);
    model.realizeAcceleration(state);
}

/// Get model accelerations given the constraint multipliers. This calculation
/// is necessary for computing constraint defects associated with the system
/// dynamics, represented by the equations 
///
///     M udot + G^T lambda + f_inertial(q,u) = f_applied
///
/// If using an explicit representation of the system dynamics, the derivatives
/// of the generalized speeds for the system need to be compuated in order to 
/// construct the defects. Rearranging the equations above (and noting that 
/// Simbody does not actually invert the mass matrix, but rather uses an order-N
/// approach), we obtain
///
///     udot = M_inv (f_applied - f_inertial(q,u) - G^T lambda) 
///          = f(q, u, lambda)
/// 
/// where,
///              q | generalized coordinates
///              u | generalized speeds
///         lambda | Lagrange multipliers
///
/// Since the three quantities required to compute the system accelerations 
/// will eventually becomes NLP variables in a direct collocation problem, 
/// it is not sufficient to use the internally calculated Lagrange 
/// multipliers in Simbody. An intermediate calculation must be made:
///
///     f_constraint(lambda) = G^T lambda
///
/// Therefore, this method computes the generalized speed derivatives via the
/// equation
///
///     udot = M_inv (f_applied - f_inertial(q,u) - f_constraint(lambda))
///
/// Finally, note that in order for f_constraint to be used like an applied 
/// force (i.e. appear on the RHS), the multipliers are negated in the call to
/// obtain Simbody constraint forces.
void calcAccelerationsFromMultipliers(const Model& model, const State& state, 
        const Vector& multipliers, Vector& udot) {

    const SimTK::MultibodySystem& multibody = model.getMultibodySystem();
    const SimTK::Vector_<SimTK::SpatialVec>& appliedBodyForces = 
        multibody.getRigidBodyForces(state, SimTK::Stage::Dynamics);
    const Vector& appliedMobilityForces = multibody.getMobilityForces(state,
        SimTK::Stage::Dynamics);

    const SimTK::SimbodyMatterSubsystem& matter = model.getMatterSubsystem();
    SimTK::Vector_<SimTK::SpatialVec> constraintBodyForces;
    Vector constraintMobilityForces;
    // Multipliers are negated so constraint forces can be used like applied 
    // forces.
    matter.calcConstraintForcesFromMultipliers(state, -multipliers, 
        constraintBodyForces, constraintMobilityForces);

    SimTK::Vector_<SimTK::SpatialVec> A_GB;
    matter.calcAccelerationIgnoringConstraints(state, 
        appliedMobilityForces + constraintMobilityForces,
        appliedBodyForces + constraintBodyForces, udot, A_GB);
}

/// The following tests add a constraint to a model and check that the method
/// calcAccelerationsFromMultipliers() is implemented correctly. Each test 
/// follows a similar structure:
///     1) Create a model and add a constraint between two bodies
///     2) Create a random state and realize the model to Stage::Acceleration
///     3) Check that state contains at least one Lagrange multiplier
///     4) Compute the model accelerations from Simbody
///     5) Retrieve the Lagrange multiplier values for the current state
///     6) Compute the accelerations from calcAccelerationsFromMultipliers()
///     7) Ensure that the accelerations from step 4 and 6 match

void testWeldConstraint() {

    State state;
    Model model = createModel();
    const std::string& firstBodyName = model.getBodySet().get(0).getName();
    const std::string& lastBodyName = 
        model.getBodySet().get(NUM_BODIES-1).getName();
    WeldConstraint* constraint = new WeldConstraint("weld", firstBodyName, 
        lastBodyName);
    model.addConstraint(constraint);
    createState(model, state);
    // Check that constraint was added successfully.
    SimTK_TEST(state.getNMultipliers() > 0);

    const Vector& udotSimbody = model.getMatterSubsystem().getUDot(state);
    const Vector& multipliers = 
        model.getMatterSubsystem().getConstraintMultipliers(state);
    Vector udotMultipliers;
    calcAccelerationsFromMultipliers(model, state, multipliers, udotMultipliers);
    // Check that accelerations calculated from Lagrange multipliers match
    // Simbody's accelerations.
    MACHINE_TEST(udotSimbody, udotMultipliers);

}

void testPointConstraint() {

    State state;
    Model model = createModel();
    const Body& firstBody = model.getBodySet().get(0);
    const Body& lastBody =
        model.getBodySet().get(NUM_BODIES - 1);
    PointConstraint* constraint = new PointConstraint(firstBody, Vec3(0), 
        lastBody, Vec3(0));
    model.addConstraint(constraint);
    createState(model, state);
    // Check that constraint was added successfully.
    SimTK_TEST(state.getNMultipliers() > 0);
    
    const Vector& udotSimbody = model.getMatterSubsystem().getUDot(state);
    const Vector& multipliers =
        model.getMatterSubsystem().getConstraintMultipliers(state);
    Vector udotMultipliers;
    calcAccelerationsFromMultipliers(model, state, multipliers, udotMultipliers);
    // Check that accelerations calculated from Lagrange multipliers match
    // Simbody's accelerations.
    MACHINE_TEST(udotSimbody, udotMultipliers);

}

void testPointOnLineConstraint() {

    State state;
    Model model = createModel();
    const Body& firstBody = model.getBodySet().get(0);
    const Body& lastBody =
        model.getBodySet().get(NUM_BODIES - 1);
    PointOnLineConstraint* constraint = new PointOnLineConstraint(firstBody,
        Vec3(1,0,0), Vec3(0), lastBody, Vec3(0));
    model.addConstraint(constraint);
    createState(model, state);
    // Check that constraint was added successfully.
    SimTK_TEST(state.getNMultipliers() > 0);

    const Vector& udotSimbody = model.getMatterSubsystem().getUDot(state);
    const Vector& multipliers =
        model.getMatterSubsystem().getConstraintMultipliers(state);
    Vector udotMultipliers;
    calcAccelerationsFromMultipliers(model, state, multipliers, udotMultipliers);
    // Check that accelerations calculated from Lagrange multipliers match
    // Simbody's accelerations.
    MACHINE_TEST(udotSimbody, udotMultipliers);

}

void testConstantDistanceConstraint() {

    State state;
    Model model = createModel();
    const Body& firstBody = model.getBodySet().get(0);
    const Body& lastBody =
        model.getBodySet().get(NUM_BODIES - 1);
    ConstantDistanceConstraint* constraint = new ConstantDistanceConstraint(
        firstBody, Vec3(0), lastBody, Vec3(0), 4.56);
    model.addConstraint(constraint);
    createState(model, state);
    // Check that constraint was added successfully.
    SimTK_TEST(state.getNMultipliers() > 0);

    const Vector& udotSimbody = model.getMatterSubsystem().getUDot(state);
    const Vector& multipliers =
        model.getMatterSubsystem().getConstraintMultipliers(state);
    Vector udotMultipliers;
    calcAccelerationsFromMultipliers(model, state, multipliers, udotMultipliers);
    // Check that accelerations calculated from Lagrange multipliers match
    // Simbody's accelerations.
    MACHINE_TEST(udotSimbody, udotMultipliers);

}

void testLockedCoordinate() {

    State state;
    Model model = createModel();
    CoordinateSet& coordSet = model.updCoordinateSet();
    coordSet.getLast()->set_locked(true);
    createState(model, state);
    // Check that constraint was added successfully.
    SimTK_TEST(state.getNMultipliers() > 0);

    const Vector& udotSimbody = model.getMatterSubsystem().getUDot(state);
    const Vector& multipliers =
        model.getMatterSubsystem().getConstraintMultipliers(state);
    Vector udotMultipliers;
    calcAccelerationsFromMultipliers(model, state, multipliers, udotMultipliers);
    // Check that accelerations calculated from Lagrange multipliers match
    // Simbody's accelerations.
    MACHINE_TEST(udotSimbody, udotMultipliers);

}

void testPrescribedMotion() {

    State state;
    Model model = createModel();
    CoordinateSet& coordSet = model.updCoordinateSet();
    LinearFunction func(1.0, 0.0);
    coordSet.getLast()->setPrescribedFunction(func);
    coordSet.getLast()->setDefaultIsPrescribed(true);
    createState(model, state);
    // Check that constraint was added successfully.
    SimTK_TEST(state.getNMultipliers() > 0);

    const Vector& udotSimbody = model.getMatterSubsystem().getUDot(state);
    const Vector& multipliers =
        model.getMatterSubsystem().getConstraintMultipliers(state);
    Vector udotMultipliers;
    calcAccelerationsFromMultipliers(model, state, multipliers, udotMultipliers);
    // Check that accelerations calculated from Lagrange multipliers match
    // Simbody's accelerations.
    MACHINE_TEST(udotSimbody, udotMultipliers);

}

int main() {
    SimTK_START_TEST("testConstraints");
        SimTK_SUBTEST(testWeldConstraint);
        SimTK_SUBTEST(testPointConstraint);
        SimTK_SUBTEST(testPointOnLineConstraint);
        SimTK_SUBTEST(testConstantDistanceConstraint);
        SimTK_SUBTEST(testLockedCoordinate);
        SimTK_SUBTEST(testPrescribedMotion);
    SimTK_END_TEST();
}