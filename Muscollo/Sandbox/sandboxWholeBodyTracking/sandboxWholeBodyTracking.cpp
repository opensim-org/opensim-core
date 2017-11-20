/* -------------------------------------------------------------------------- *
 * OpenSim Muscollo: exampleWholeBodyTracking.cpp                             *
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
 
#include <OpenSim/Common/osimCommon.h>
#include <OpenSim/Simulation/osimSimulation.h>
#include <OpenSim/Actuators/osimActuators.h>
//#include <tropter/tropter.h>
#include <Muscollo/osimMuscollo.h>

using namespace OpenSim;

Model createHipModel() {
    Model model;
    model.setName("right_hip");

    using SimTK::Vec3;
    using SimTK::Inertia;

    // Borrowed from the Dynamic Walker example.
    double pelvisWidth = 0.20;
    double thighLength = 0.40;
    double shankLength = 0.435;
    
    // Create bodies
    auto* pelvis = new OpenSim::Body("pelvis", 1, Vec3(0), Inertia(1));
    auto* pelvis_geom = new OpenSim::Ellipsoid;
    pelvis_geom->set_radii(Vec3(0.03, 0.03, pelvisWidth / 2.0));
    pelvis->attachGeometry(pelvis_geom);
    model.addBody(pelvis);

    auto* thigh = new OpenSim::Body("thigh", 1, Vec3(0), Inertia(1));
    auto* thigh_geom = new OpenSim::Ellipsoid; 
    thigh_geom->set_radii(Vec3(0.03, thighLength / 2.0, 0.03));
    thigh->attachGeometry(thigh_geom);
    model.addBody(thigh);

    // Create free joint between ground and pelvis
    //auto* gp = new FreeJoint("gp", 
    //        model.getGround(), Vec3(0, 1.0, 0), Vec3(0), 
    //        *pelvis, Vec3(0), Vec3(0));
    //auto& p_tx = gp->updCoordinate(FreeJoint::Coord::TranslationX);
    //p_tx.setName("p_tx");
    //auto& p_ty = gp->updCoordinate(FreeJoint::Coord::TranslationY);
    //p_ty.setName("p_ty");
    //auto& p_tz = gp->updCoordinate(FreeJoint::Coord::TranslationZ);
    //p_tz.setName("p_tz");
    //auto& p_rx = gp->updCoordinate(FreeJoint::Coord::Rotation1X);
    //p_rx.setName("p_rx");
    //auto& p_ry = gp->updCoordinate(FreeJoint::Coord::Rotation2Y);
    //p_ry.setName("p_ry");
    //auto& p_rz = gp->updCoordinate(FreeJoint::Coord::Rotation3Z);
    //p_rz.setName("p_rz");
    //model.addJoint(gp);

    auto* gp = new PlanarJoint("gp",
        model.getGround(), Vec3(0, 1.0, 0), Vec3(0),
        *pelvis, Vec3(0), Vec3(0));
    auto& p_tx = gp->updCoordinate(PlanarJoint::Coord::TranslationX);
    p_tx.setName("p_tx");
    auto& p_ty = gp->updCoordinate(PlanarJoint::Coord::TranslationY);
    p_ty.setName("p_ty");
    auto& p_rz = gp->updCoordinate(PlanarJoint::Coord::RotationZ);
    p_rz.setName("p_rz");
    model.addJoint(gp);

    // Create hip joint
    auto* hip = new PinJoint("hip",
            *pelvis, Vec3(0, 0, pelvisWidth/2.0), Vec3(0),
            *thigh, Vec3(0, thighLength/2.0, 0), Vec3(0, 0, 0));
    //auto& hip_rx = hip->updCoordinate(BallJoint::Coord::Rotation1X);
    //hip_rx.setName("hip_rx");
    //auto& hip_ry = hip->updCoordinate(BallJoint::Coord::Rotation2Y);
    //hip_ry.setName("hip_ry");
    auto& hip_rz = hip->updCoordinate();
    //hip_rz.setRangeMax(SimTK::Pi/2);
    //hip_rz.setRangeMin(-SimTK::Pi/2);
  
    hip_rz.setName("hip_rz");
    model.addJoint(hip);

    // Add markers
    // TODO

    // Add actuators
    // ground-pelvis
    auto* tau_p_tx = new CoordinateActuator();
    tau_p_tx->setCoordinate(&p_tx);
    tau_p_tx->setName("tau_p_tx");
    tau_p_tx->setOptimalForce(1);
    model.addComponent(tau_p_tx);

    auto* tau_p_ty = new CoordinateActuator();
    tau_p_ty->setCoordinate(&p_ty);
    tau_p_ty->setName("tau_p_ty");
    tau_p_ty->setOptimalForce(1);
    model.addComponent(tau_p_ty);

    //auto* tau_p_tz = new CoordinateActuator();
    //tau_p_tz->setCoordinate(&p_tz);
    //tau_p_tz->setName("tau_p_tz");
    //tau_p_tz->setOptimalForce(1);
    //model.addComponent(tau_p_tz);

    //auto* tau_p_rx = new CoordinateActuator();
    //tau_p_rx->setCoordinate(&p_rx);
    //tau_p_rx->setName("tau_p_rx");
    //tau_p_rx->setOptimalForce(1);
    //model.addComponent(tau_p_rx);

    //auto* tau_p_ry = new CoordinateActuator();
    //tau_p_ry->setCoordinate(&p_ry);
    //tau_p_ry->setName("tau_p_ry");
    //tau_p_ry->setOptimalForce(1);
    //model.addComponent(tau_p_ry);

    auto* tau_p_rz = new CoordinateActuator();
    tau_p_rz->setCoordinate(&p_rz);
    tau_p_rz->setName("tau_p_rz");
    tau_p_rz->setOptimalForce(1);
    model.addComponent(tau_p_rz);

    // hip
    //auto* tau_hip_rx = new CoordinateActuator();
    //tau_hip_rx->setCoordinate(&hip_rx);
    //tau_hip_rx->setName("tau_hip_rx");
    //tau_hip_rx->setOptimalForce(1);
    //model.addComponent(tau_hip_rx);

    //auto* tau_hip_ry = new CoordinateActuator();
    //tau_hip_ry->setCoordinate(&hip_ry);
    //tau_hip_ry->setName("tau_hip_ry");
    //tau_hip_ry->setOptimalForce(1);
    //model.addComponent(tau_hip_ry);

    auto* tau_hip_rz = new CoordinateActuator();
    tau_hip_rz->setCoordinate(&hip_rz);
    tau_hip_rz->setName("tau_hip_rz");
    tau_hip_rz->setOptimalForce(1);
    model.addComponent(tau_hip_rz);

    model.print("hip_model.osim");

    return model;
}

int main() {

    MucoTool muco;
    muco.setName("whole_body_tracking");

    // Define the optimal control problem.
    // ===================================
    MucoProblem& mp = muco.updProblem();

    // Model (dynamics).
    // -----------------
    mp.setModel(createHipModel());

    // Bounds.
    // -------
    mp.setTimeBounds(0.4, 1.6);
    // ground-pelvis
    mp.setStateInfo("gp/p_tx/value", { -10, 10 });
    mp.setStateInfo("gp/p_tx/speed", { -50, 50 });
    mp.setStateInfo("gp/p_ty/value", { -10, 10 });
    mp.setStateInfo("gp/p_ty/speed", { -50, 50 });
    //mp.setStateInfo("gp/p_tz/value", { -10, 10 });
    //mp.setStateInfo("gp/p_tz/speed", { -50, 50 });
    //mp.setStateInfo("gp/p_rx/value", { -10, 10 });
    //mp.setStateInfo("gp/p_rx/speed", { -50, 50 });
    //mp.setStateInfo("gp/p_ry/value", { -10, 10 });
    //mp.setStateInfo("gp/p_ry/speed", { -50, 50 });
    mp.setStateInfo("gp/p_rz/value", { -10, 10 });
    mp.setStateInfo("gp/p_rz/speed", { -50, 50 });
    // hip
    //mp.setStateInfo("hip/hip_rx/value", { -10, 10 });
    //mp.setStateInfo("hip/hip_rx/speed", { -50, 50 });
    //mp.setStateInfo("hip/hip_ry/value", { -10, 10 });
    //mp.setStateInfo("hip/hip_ry/speed", { -50, 50 });
    mp.setStateInfo("hip/hip_rz/value", { -10, 10 });
    mp.setStateInfo("hip/hip_rz/speed", { -50, 50 });
    // torques
    mp.setControlInfo("tau_p_tx", { -100, 100 });
    mp.setControlInfo("tau_p_ty", { -100, 100 });
    //mp.setControlInfo("tau_p_tz", { -100, 100 }); 
    //mp.setControlInfo("tau_p_rx", { -100, 100 });
    //mp.setControlInfo("tau_p_ry", { -100, 100 }); 
    mp.setControlInfo("tau_p_rz", { -100, 100 });
    //mp.setControlInfo("tau_hip_rx", { -100, 100 }); 
    //mp.setControlInfo("tau_hip_ry", { -100, 100 });
    mp.setControlInfo("tau_hip_rz", { -100, 100 });


    MucoStateTrackingCost trackingCost;
    auto ref = STOFileAdapter::read("state_reference.mot");
    auto refFilt = filterLowpass(ref, 6.0, true);

    auto model = mp.getPhase().getModel();
    model.initSystem();
    auto refFilt2 = refFilt;
    model.getSimbodyEngine().convertDegreesToRadians(refFilt2);
    STOFileAdapter::write(refFilt2, "state_reference_radians.sto");

    trackingCost.setReference(refFilt);
    trackingCost.allowUnusedReferences(true);
    trackingCost.setWeight("gp/p_rz/value", 100.0);
    trackingCost.setWeight("gp/p_tx/value", 25.0);
    //trackingCost.setWeight("gp/p_ty/value", 10.0);
    trackingCost.setWeight("hip/hip_rz/value", 2.0);
    mp.addCost(trackingCost);

    // Configure the solver.
    // =====================
    MucoTropterSolver& ms = muco.initSolver();
    ms.set_num_mesh_points(50);
    ms.set_verbosity(2);
    ms.set_optim_solver("ipopt");
    ms.set_optim_hessian_approximation("exact");

    MucoIterate guess = ms.createGuess();
    guess.setStatesTrajectory(refFilt2, true, true);
    guess.write("states_guess.sto");
    ms.setGuess(guess);

    // Solve the problem.
    // ==================
    MucoSolution solution = muco.solve().unseal();
    solution.write("sandboxWholeBodyTracking_solution.sto");

    muco.visualize(solution);

    return EXIT_SUCCESS;


}