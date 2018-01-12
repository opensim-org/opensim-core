/* -------------------------------------------------------------------------- *
 * OpenSim Muscollo: sandboxCalibrateContact.cpp                              *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Nicholas Bianco, Chris Dembia                                   *
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
#include <OpenSim/Tools/InverseKinematicsTool.h>
#include <OpenSim/Common/TimeSeriesTable.h>
#include <Muscollo/osimMuscollo.h>

#include <MuscolloSandboxShared.h>

#include <tropter/tropter.h>

using namespace OpenSim;
using Eigen::VectorXd;

template <typename T, typename TStiffness>
T contact_force(const TStiffness& stiffness, const T& y) {
    const double stiffness_fictitious = 1.0; // N/m
    const T ground_height = 0;
    // Positive if penetrated.
    const T depth = ground_height - y;
    const T depth_pos = fmax(0, depth);
    const T contact_normal_force = stiffness * depth_pos +
            stiffness_fictitious * depth;
    return contact_normal_force;
}

template<typename T>
class BouncingBallLinear : public tropter::OptimalControlProblem<T> {
public:
    static const double mass; // kg
    static const double stiffness; // N/m
    static const double g; // m/s^2
    BouncingBallLinear() {
        this->set_time(0, 1.25);
        this->add_state("y", {-1, 1}, 1);
        this->add_state("vy", {-10, 10}, 0);
    }
    void calc_differential_algebraic_equations(
            const tropter::DAEInput<T>& in,
            tropter::DAEOutput<T> out) const override {
        const T& y = in.states[0];
        const T& vy = in.states[1];
        out.dynamics[0] = vy;
        const auto contact_normal_force = contact_force(this->stiffness, y);
        out.dynamics[1] = -g + (contact_normal_force) / mass;
    }
    static tropter::OptimalControlSolution run() {
        auto ocp = std::make_shared<BouncingBallLinear<T>>();
        const int N = 1000;
        tropter::DirectCollocationSolver<T> dircol(ocp, "trapezoidal", "ipopt",
                N);
        tropter::OptimalControlSolution solution = dircol.solve();
        //std::cout << "States: " << solution.states << std::endl;
        //solution.write("sandboxCalibrateContact_bouncing_ball_solution.csv");
        return solution;
    }
};
template <typename T>
const double BouncingBallLinear<T>::mass = 50.0; // kg
template <typename T>
const double BouncingBallLinear<T>::stiffness = 3180.0; // N/m
template <typename T>
const double BouncingBallLinear<T>::g = 9.81; // m/s^2

class BallCalibration : public tropter::OptimizationProblem<double> {
public:
    BallCalibration(Eigen::VectorXd yTraj, Eigen::VectorXd contactForceTraj) :
            tropter::OptimizationProblem<double>(1, 0),
            m_yTraj(yTraj), m_contactForceTraj(contactForceTraj) {
        Eigen::VectorXd lower(1); lower[0] = 0;
        Eigen::VectorXd upper(1); upper[0] = 10000;
        set_variable_bounds(lower, upper);
    }
    void calc_objective(const VectorXd& x, double& obj_value) const override {

        const auto& stiffness = x[0];
        obj_value = 0;
        for (int it = 0; it < m_yTraj.size(); ++it) {
            const auto& y = m_yTraj[it];
            const auto simContactForce = contact_force(stiffness, y);
            const auto& expContactForce = m_contactForceTraj[it];
            obj_value += pow(simContactForce - expContactForce, 2);
        }
    }
private:
    Eigen::VectorXd m_yTraj;
    Eigen::VectorXd m_contactForceTraj;
};

void calibrateBall() {
    const auto exp = BouncingBallLinear<adouble>::run();
    Eigen::VectorXd Fy(exp.time.size());
    for (int it = 0; it < exp.time.size(); ++it) {
        Fy[it] = contact_force(BouncingBallLinear<double>::stiffness,
                exp.states(0, it));
    }
    BallCalibration problem(exp.states.row(0).transpose(), Fy);
    tropter::IPOPTSolver solver(problem);
    solver.set_verbosity(1);
    auto solution = solver.optimize();
    std::cout << solution.variables << std::endl;
}

/// Optimize the stiffness of contact points to match the ground reaction
/// force.
class ContactCalibration : public tropter::OptimizationProblem<double> {
public:
    double forceScalingFactor = 1e9;
    ContactCalibration(Model model, StatesTrajectory statesTraj) :
            tropter::OptimizationProblem<double>(3, 0),
            m_model(std::move(model)), m_statesTraj(std::move(statesTraj)) {
        set_variable_bounds(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 1, 1));

        m_model.initSystem();


        // Foot–ground contact data.
        // -------------------------
        auto data = STOFileAdapter::read("walk_gait1018_subject01_grf.mot");
        auto time = data.getIndependentColumn();
        SimTK::Vector Fx = data.getDependentColumn("ground_force_vx");
        SimTK::Vector Fy = data.getDependentColumn("ground_force_vy");
        // TODO GCVSpline FxSpline(5, (int)time.size(), time.data(), &Fx[0]);
        m_FySpline = GCVSpline(5, (int)time.size(), time.data(), &Fy[0]);
    }

    void applyParametersToModel(const VectorXd& x, Model& model) const {
        int icontact = 0;
        for (auto& contact :
                model.updComponentList<AckermannVanDenBogert2010Force>()) {
            contact.set_stiffness(forceScalingFactor * x[icontact]);
            ++icontact;
        }
    }

    void calc_objective(const VectorXd& x, double& obj_value) const override {
        obj_value = 0;

        // Apply parameters.
        // -----------------
        applyParametersToModel(x, m_model);
        // TODO is this expensive?
        m_model.initSystem();

        // Compute contact force error.
        // ----------------------------

        // TODO add fore-aft force.

        auto contacts =
                m_model.updComponentList<AckermannVanDenBogert2010Force>();
        for (const auto& state : m_statesTraj) {
            m_model.realizeVelocity(state);
            SimTK::Real simFy = 0;

            for (auto& contact : contacts) {
                simFy += contact.calcContactForce(state)[1];
            }

            SimTK::Vector timeVec(1, state.getTime());
            SimTK::Real expFy = m_FySpline.calcValue(timeVec);
            obj_value += pow(simFy - expFy, 2);
        }
    }
private:

    mutable Model m_model;
    StatesTrajectory m_statesTraj;
    GCVSpline m_FySpline;

};

/// Convenience function to apply a CoordinateActuator to the model.
static void addCoordinateActuator(Model& model, std::string coordName,
        double optimalForce) {

    auto& coordSet = model.updCoordinateSet();

    CoordinateActuator* actu = nullptr;
    //auto* actActu = new ActivationCoordinateActuator();
    //actActu->set_default_activation(0.1);
    //actu = actActu;
    actu = new CoordinateActuator();
    actu->setName("tau_" + coordName);
    actu->setCoordinate(&coordSet.get(coordName));
    actu->setOptimalForce(optimalForce);
    model.addComponent(actu);
}

void addContact(Model& model, std::string markerName, double stiffness = 5e7) {
    //const double stiffness = 5e7;
    const double friction_coefficient = 0.95;
    const double velocity_scaling = 0.3;
    auto* contact = new AckermannVanDenBogert2010Force();
    contact->setName(markerName + "_contact");
    contact->set_stiffness(stiffness);
    contact->set_friction_coefficient(friction_coefficient);
    contact->set_tangent_velocity_scaling_factor(velocity_scaling);
    model.addComponent(contact);
    contact->updSocket("station").setConnecteeName(markerName);
}

void calibrateContact() {

    // Model.
    // ------
    Model model("gait1018_subject01_onefoot_v30516.osim");
    model.initSystem();

    // TODO increase mass of foot!!!!

    addCoordinateActuator(model, "rz", 250);
    addCoordinateActuator(model, "tx", 5000);
    addCoordinateActuator(model, "ty", 5000);

    const auto& calcn = dynamic_cast<Body&>(model.updComponent("calcn_r"));
    model.addMarker(new Marker("R.Heel.Distal", calcn,
            SimTK::Vec3(0.01548, -0.0272884, -0.00503735)));
    model.addMarker(new Marker("R.Ball.Lat", calcn,
            SimTK::Vec3(0.16769, -0.0272884, 0.066)));
    model.addMarker(new Marker("R.Ball.Med", calcn,
            SimTK::Vec3(0.1898, -0.0272884, -0.03237)));
    addContact(model, "R.Heel.Distal", 5e7);
    addContact(model, "R.Ball.Lat", 7.5e7);
    addContact(model, "R.Ball.Med", 7.5e7);

    // Kinematics data.
    // ----------------
    StatesTrajectory statesTraj;
    MucoSolution mucoSol;
    {
        // TODO: use marker data, do IK, yada yada.
        const std::string trcFile = "sandboxCalibrateContact_markers.trc";
        const std::string motFile = "sandboxCalibrateContact.mot";
        auto ref = TRCFileAdapter::read("walk_marker_trajectories.trc");
        // Convert from millimeters to meters.
        ref.updMatrix() /= 1000;
        const auto& reftime = ref.getIndependentColumn();
        const double walkingSpeed = 1.05; // m/s
        for (int i = 0; i < (int)ref.getNumColumns(); ++i) {
            SimTK::VectorView_<SimTK::Vec3> col =
                    ref.updDependentColumnAtIndex(i);
            for (int j = 0; j < col.size(); ++j) {
                col[j][0] += walkingSpeed * reftime[j]; // x
                col[j][1] -= 0.03; // y TODO
            }
        }
        TimeSeriesTable refFilt = filterLowpass(ref.flatten({ "_x", "_y", "_z" }),
                6.0, true);
        // TODO TRCFileAdapter::write(ref, trcFile);
        auto refPacked = refFilt.pack<double>();
        TimeSeriesTableVec3 refToUse(refPacked);

        Set<MarkerWeight> markerWeights;
        markerWeights.cloneAndAppend({ "R.Heel", 2 });
        markerWeights.cloneAndAppend({ "R.Toe.Tip", 2 });
        MarkersReference markersRef(refToUse, &markerWeights);

        MucoTool muco;
        MucoProblem& mp = muco.updProblem();
        mp.setModel(model);
        MucoBounds defaultSpeedBounds(-25, 25);
        mp.setTimeBounds(0.48, 1.8); // TODO [.58, 1.8] for gait cycle of right leg.
        mp.setStateInfo("ground_toes/rz/value", { -10, 10 });
        mp.setStateInfo("ground_toes/rz/speed", defaultSpeedBounds);
        mp.setStateInfo("ground_toes/tx/value", { -10, 10 });
        mp.setStateInfo("ground_toes/tx/speed", defaultSpeedBounds);
        mp.setStateInfo("ground_toes/ty/value", { -10, 10 });
        mp.setStateInfo("ground_toes/ty/speed", defaultSpeedBounds);
        mp.setControlInfo("tau_rz", { -1, 1 });
        mp.setControlInfo("tau_tx", { -1, 1 });
        mp.setControlInfo("tau_ty", { -1, 1 });

        MucoMarkerTrackingCost tracking;
        tracking.setMarkersReference(markersRef);
        tracking.setAllowUnusedReferences(true);
        tracking.setTrackedMarkerComponents("xy");
        mp.addCost(tracking);

        auto& ms = muco.initSolver();
        ms.set_num_mesh_points(30);
        ms.set_optim_convergence_tolerance(1e-2);
        ms.set_optim_constraint_tolerance(1e-2);

        mucoSol = muco.solve();
        statesTraj = mucoSol.exportToStatesTrajectory(mp);

        visualize(model, mucoSol.exportToStatesStorage());

        /*
        Storage ref(motFile);
        // TODO ref.pad(ref.getSize() / 2);
        ref.lowpassIIR(6.0);
        m_model.getSimbodyEngine().convertDegreesToRadians(ref);
        m_statesTraj = StatesTrajectory::createFromStatesStorage(m_model,
                ref, true);
                */
        // TODO visualize!

    }

    ContactCalibration problem(model, statesTraj);
    tropter::IPOPTSolver solver(problem);
    solver.set_verbosity(1);
    auto solution = solver.optimize();
    problem.applyParametersToModel(solution.variables, model);
    visualize(model, mucoSol.exportToStatesStorage());
    std::cout << solution.variables << std::endl;
}


int main() {

    // calibrateBall();

    calibrateContact();

    return EXIT_SUCCESS;
}

