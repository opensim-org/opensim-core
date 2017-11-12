/* -------------------------------------------------------------------------- *
 * OpenSim Muscollo: sandboxContact.cpp                                       *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Christopher Dembia                                              *
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

#include <OpenSim/Simulation/SimbodyEngine/SliderJoint.h>
#include <OpenSim/Actuators/CoordinateActuator.h>
#include <Muscollo/osimMuscollo.h>
#include <OpenSim/Simulation/Manager/Manager.h>
#include <OpenSim/Common/STOFileAdapter.h>

// TODO achieve sliding friction (apply constant tangential force,
// maybe from gravity?).

using namespace OpenSim;
using SimTK::Vec3;

class CustomContactForce : public Force {
    OpenSim_DECLARE_CONCRETE_OBJECT(CustomContactForce, Force);
public:
    OpenSim_DECLARE_SOCKET(station, Station, "TODO");
    void computeForce(const SimTK::State& s,
            SimTK::Vector_<SimTK::SpatialVec>& bodyForces,
            SimTK::Vector& /*generalizedForces*/) const override {
        const auto& pt = getConnectee<Station>("station");
        const auto& pos = pt.getLocationInGround(s);
        const auto& vel = pt.getVelocityInGround(s);
        const SimTK::Real y = pos[1];
        const SimTK::Real vy = vel[1];
        SimTK::Vec3 force(0);
        const SimTK::Real depth = 0 - y;
        const SimTK::Real depthRate = 0 - vy;
        const SimTK::Real a = 5e7; // N/m^3
        const SimTK::Real b = 1.0; // s/m
        if (depth > 0) {
            //force = 1000 * depth;
            force[1] = fmax(0, a * pow(depth, 3) * (1 + b * depthRate));
        }
        const SimTK::Real voidStiffness = 1.0; // N/m
        force[1] += voidStiffness * depth;
        //applyGeneralizedForce(s, getModel().getCoordinateSet().get(0),
        //        force, generalizedForces);
        applyForceToPoint(s, pt.getParentFrame(), pt.get_location(), force,
                bodyForces);
        // TODO equal and opposite force on ground.
    }
};

class CustomContactForceFriction : public Force {
OpenSim_DECLARE_CONCRETE_OBJECT(CustomContactForceFriction, Force);
public:
    OpenSim_DECLARE_SOCKET(station, Station, "TODO");
    void computeForce(const SimTK::State& s,
            SimTK::Vector_<SimTK::SpatialVec>& bodyForces,
            SimTK::Vector& /*generalizedForces*/) const override {
        const auto& pt = getConnectee<Station>("station");
        const auto& pos = pt.getLocationInGround(s);
        const auto& vel = pt.getVelocityInGround(s);
        const SimTK::Real y = pos[1];
        const SimTK::Real velNormal = vel[1];
        // TODO should project vel into ground.
        const SimTK::Real velSliding = vel[0];
        SimTK::Vec3 force(0);
        const SimTK::Real depth = 0 - y;
        const SimTK::Real depthRate = 0 - velNormal;
        const SimTK::Real a = 5e7; // N/m^3
        const SimTK::Real b = 1.0; // s/m
        if (depth > 0) {
            force[1] = fmax(0, a * pow(depth, 3) * (1 + b * depthRate));
        }
        const SimTK::Real voidStiffness = 1.0; // N/m
        force[1] += voidStiffness * depth;

        const SimTK::Real velSlidingScaling = 0.05;
        const SimTK::Real z0 = exp(-velSliding / velSlidingScaling);
        const SimTK::Real coeff_friction = 1.0; // TODO
        // TODO decide direction!!!

        // TODO tropter thinks there are 402 seeds req for Jacobian.
        const SimTK::Real frictionForce =
                -(1 - z0) / (1 + z0) * coeff_friction * force[1];

        //std::cout << "DEBUG " << s.getTime() << " " << force[1]
        //        << " " << frictionForce <<
        //        std::endl;
        force[0] = frictionForce;

        if (force.isNaN()) {
        std::cout << "DEBUGcomputeForce"
                << " t:" << s.getTime()
                << " depth:" << depth
                << " f0:" << force[0]
                << " f1:" << force[1]
                << " vs: " << velSliding
                << " z0: " << z0
                << std::endl;
        }

        const auto& frame = pt.getParentFrame();
        applyForceToPoint(s, frame, pt.get_location(), force, bodyForces);
        applyForceToPoint(s, getModel().getGround(), pos, -force, bodyForces);
    }
};

Model createModel() {
    Model model;
    model.setName("point_mass");
    auto* body = new Body("body", 50.0, Vec3(0), SimTK::Inertia(0));
    model.addComponent(body);

    // Allows translation along x.
    auto* joint = new SliderJoint("slider",
            model.getGround(), Vec3(0), SimTK::Vec3(0, 0, 0.5 * SimTK::Pi),
            *body, Vec3(0), Vec3(0));
    auto& coord = joint->updCoordinate(SliderJoint::Coord::TranslationX);
    coord.setName("y");
    model.addComponent(joint);

    auto* station = new Station();
    station->setName("contact_point");
    station->connectSocket_parent_frame(*body);
    model.addComponent(station);

    auto* force = new CustomContactForce();
    force->setName("contact");
    model.addComponent(force);
    force->connectSocket_station(*station);

    return model;
}

Model createModel2D() {
    Model model;
    model.setName("point_mass");
    auto* intermed = new Body("intermed", 0, Vec3(0), SimTK::Inertia(0));
    model.addComponent(intermed);
    // TODO inertia...
    auto* body = new Body("body", 50.0, Vec3(0), SimTK::Inertia(1));
    model.addComponent(body);

    // Allows translation along x.
    auto* jointX = new SliderJoint();
    jointX->setName("tx");
    jointX->connectSocket_parent_frame(model.getGround());
    jointX->connectSocket_child_frame(*intermed);
    auto& coordX = jointX->updCoordinate(SliderJoint::Coord::TranslationX);
    coordX.setName("tx");
    model.addComponent(jointX);

    // The joint's x axis must point in the global "+y" direction.
    auto* jointY = new SliderJoint("ty",
            *intermed, Vec3(0), Vec3(0, 0, 0.5 * SimTK::Pi),
            *body, Vec3(0), Vec3(0, 0, 0.5 * SimTK::Pi));
    auto& coordY = jointY->updCoordinate(SliderJoint::Coord::TranslationX);
    coordY.setName("ty");
    model.addComponent(jointY);

    auto* station = new Station();
    station->setName("contact_point");
    station->connectSocket_parent_frame(*body);
    model.addComponent(station);

    //auto* force = new CustomContactForce();
    auto* force = new CustomContactForceFriction();
    force->setName("contact");
    model.addComponent(force);
    force->connectSocket_station(*station);

    return model;
}

int main() {

    const SimTK::Real y0 = 0.5;
    const SimTK::Real vx0 = 0.7;
    const SimTK::Real finalTime = 1.0;

    if (false) {
        Model model = createModel();
        auto state = model.initSystem();
        model.setStateVariableValue(state, "/slider/y/value", y0);
        Manager manager(model);
        manager.integrate(state, finalTime);
        visualize(model, manager.getStateStorage());
    }



    Model model = createModel2D();
    auto state = model.initSystem();
    model.setStateVariableValue(state, "ty/ty/value", y0);
    model.setStateVariableValue(state, "tx/tx/speed", vx0);
    SimTK::RungeKuttaMersonIntegrator integrator(model.getMultibodySystem());
    // Without the next line: Simbody takes steps that are too large and NaNs
    // are generated.
    integrator.setMaximumStepSize(0.05);
    Manager manager(model, integrator);
    manager.integrate(state, finalTime);
    const auto& statesTimeStepping = manager.getStateStorage();
    visualize(model, statesTimeStepping);
    const auto statesTimeSteppingTable =
            statesTimeStepping.getAsTimeSeriesTable();
    STOFileAdapter::write(statesTimeSteppingTable, "ball2d_timestepping.sto");

    // TODO use the simulation as an initial guess!!!

    // TODO solve with Muscollo and forward simulation and ensure we get the
    // same answer.


    if (true) {
        MucoTool muco;
        muco.setName("ball2d");

        // Define the optimal control problem.
        // ===================================
        MucoProblem& mp = muco.updProblem();

        // Model (dynamics).
        // -----------------
        mp.setModel(model);

        // Bounds.
        // -------
        // Initial time must be 0, final time can be within [0, 5].
        mp.setTimeBounds(0, finalTime);

        mp.setStateInfo("tx/tx/value", {-5, 5}, 0);
        mp.setStateInfo("ty/ty/value", {-0.5, 1}, y0);
        mp.setStateInfo("tx/tx/speed", {-10, 10}, vx0);
        mp.setStateInfo("ty/ty/speed", {-10, 10}, 0);

        // Configure the solver.
        // =====================
        MucoTropterSolver& ms = muco.initSolver();
        ms.set_num_mesh_points(100);

        MucoIterate guess = ms.createGuess();

        // Setting this guess reduces the number of iterations from 90 to 6.
        guess.setStatesTrajectory(statesTimeSteppingTable);
        ms.setGuess(guess);

        // TODO interface for setting these options:
        // TODO ms.setOption("optim.hessian-approximation", "limited-memory");
        // TODO ms.set_optimizer_algorithm("ipopt");


        // Now that we've finished setting up the tool, print it to a file.
        //muco.print("contact.omuco");

        // Solve the problem.
        // ==================
        MucoSolution solution = muco.solve();

        solution.write("ball2d_solution.sto");

        // TODO copmare the forward and direct collocation integration.

        solution.isNumericallyEqual(guess);

        // Visualize.
        // ==========
        muco.visualize(solution);

    }

    return EXIT_SUCCESS;
}
