/* -------------------------------------------------------------------------- *
 * OpenSim Moco: sandboxSandbox.cpp                                           *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2019 Stanford University and the Authors                     *
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

// This file provides a way to easily prototype or test temporary snippets of
// code during development.

#include <OpenSim/Moco/osimMoco.h>
#include <casadi/casadi.hpp>
#include <OpenSim/Common/STOFileAdapter.h>
#include <OpenSim/Simulation/VisualizerUtilities.h>

using namespace OpenSim;
using namespace casadi;

using ContactForceRefs = 
            std::vector<SimTK::ReferencePtr<const SmoothSphereHalfSpaceForce>>;

void calcContactForces(const Model& model, SimTK::State& state, 
        const SimTK::Vector& q, const SimTK::Vector& u,
        const ContactForceRefs& contactForcesLeft,
        const ContactForceRefs& contactForcesRight,
        SimTK::Vec3& forcesLeft, SimTK::Vec3& forcesRight) {

    state.updQ() = q;
    state.updU() = u;
    model.realizeVelocity(state);
    for (const auto& contactForce : contactForcesLeft) {
        forcesLeft += contactForce->getSphereForce(state)[1];
    }
    for (const auto& contactForce : contactForcesRight) {
        forcesRight += contactForce->getSphereForce(state)[1];
    }
}

class ContactTrackingObjective : public Callback {
public:
    // Constructor
    ContactTrackingObjective(Model model,
            TimeSeriesTable grfs, TimeSeriesTable states,
            const std::vector<std::string>& contactForcesLeft,
            const std::vector<std::string>& contactForcesRight) {

        // Model.
        m_model = std::move(model);
        m_state = m_model.initSystem();
        
        // Contact forces.
        for (const auto& contactForce : contactForcesLeft) {
            m_contactForcesLeft.emplace_back(
                    &m_model.getComponent<SmoothSphereHalfSpaceForce>(
                        contactForce));
        }
        for (const auto& contactForce : contactForcesRight) {
            m_contactForcesRight.emplace_back(
                    &m_model.getComponent<SmoothSphereHalfSpaceForce>(
                        contactForce));
        }

        // Ground reaction forces.
        m_grfs = GCVSplineSet(grfs);

        // States.
        m_states = GCVSplineSet(states);

        casadi::Dict opts;
        opts["enable_fd"] = true;
        this->construct("objective", opts);
    }

    void setTime(double time) {
        m_time = SimTK::Vector(1, time);
    }

    void setForceWeight(double weight) {
        m_forceWeight = weight;
    }

    void setCoordinateWeight(double weight) {
        m_coordinateWeight = weight;
    }

    void setSpeedWeight(double weight) {
        m_speedWeight = weight;
    }

    const ContactForceRefs& getContactForcesLeft() const {
        return m_contactForcesLeft;
    }

    const ContactForceRefs& getContactForcesRight() const {
        return m_contactForcesRight;
    }

    // Number of inputs and outputs
    casadi_int get_n_in() override { return 2;}
    casadi_int get_n_out() override { return 1;}

    Sparsity get_sparsity_in(casadi_int i) override {
        if (i == 0) {
            return Sparsity::dense(m_state.getNQ(), 1);
        } else if (i == 1) {
            return Sparsity::dense(m_state.getNU(), 1);
        } else {
            return Sparsity::dense(0, 0);
        }
    }

    Sparsity get_sparsity_out(casadi_int i) override {
        if (i == 0) {
            return Sparsity::dense(1, 1);
        } else {
            return Sparsity::dense(0, 0);
        }
    }

    // Evaluate numerically
    std::vector<DM> eval(const std::vector<DM>& arg) const override {        
        SimTK::Vector q(m_state.getNQ(), arg.at(0)->data(), true);
        SimTK::Vector u(m_state.getNU(), arg.at(1)->data(), true);

        SimTK::Vec3 errorsLeft(0);
        SimTK::Vec3 errorsRight(0);
        m_state.updQ() = q;
        m_state.updU() = u;
        m_model.realizeVelocity(m_state);
        for (const auto& contactForce : m_contactForcesLeft) {
            errorsLeft += contactForce->getSphereForce(m_state)[1];
        }
        for (const auto& contactForce : m_contactForcesRight) {
            errorsRight += contactForce->getSphereForce(m_state)[1];
        }

        // Force errors.
        errorsLeft[0] -= m_grfs.get(0).calcValue(m_time);
        errorsLeft[1] -= m_grfs.get(1).calcValue(m_time);
        errorsLeft[2] -= m_grfs.get(2).calcValue(m_time);
        errorsRight[0] -= m_grfs.get(3).calcValue(m_time);
        errorsRight[1] -= m_grfs.get(4).calcValue(m_time);
        errorsRight[2] -= m_grfs.get(5).calcValue(m_time);

        // Coordinate errors.
        double coordinateError = 0.0;
        int istate = 0;
        for (int iq = 0; iq < m_state.getNQ(); ++iq) {
            double error = q[iq] - m_states.get(istate).calcValue(m_time);
            coordinateError += m_coordinateWeight * error * error;
            istate++;
        }
        for (int iu = 0; iu < m_state.getNU(); ++iu) {
            double error = u[iu] - m_states.get(istate).calcValue(m_time);
            coordinateError += m_speedWeight * error * error;
            istate++;
        }
           
        DM f = m_forceWeight * (errorsLeft.normSqr() + errorsRight.normSqr()) + 
               (coordinateError / (m_state.getNQ() + m_state.getNU()));
        return {f};
    }

private:
    Model m_model;
    mutable SimTK::State m_state;
    SimTK::Vector m_time;

    GCVSplineSet m_grfs;
    GCVSplineSet m_states;

    ContactForceRefs m_contactForcesLeft;
    ContactForceRefs m_contactForcesRight;

    double m_forceWeight = 1.0;
    double m_coordinateWeight = 1.0;
    double m_speedWeight = 1e-3;
};

SimTK::RowVector solveContactTrackingProblem(ContactTrackingObjective& objfun, 
        double time, const TimeSeriesTable& states) {

    // The number of states. For each foot, we have a 6 DOF joint between the 
    // ground and the calcaneus, and a 1 DOF joint between the calcaneus and
    // the toes.
    int nq = 14;      // number of generalized coordinates
    int nu = nq;      // number of generalized speeds
    int ns = nq + nu; // number of states

    // Define the optimization variables.
    MX q = MX::sym("q", nq, 1); // coordinates
    MX u = MX::sym("u", nu, 1); // speeds
    MX x = vertcat(q, u);       // states

    // Objective function
    objfun.setTime(time);
    objfun.setForceWeight(1e-4);
    objfun.setCoordinateWeight(0.01);
    objfun.setSpeedWeight(0.01);

    MXVector costOut;
    objfun.call({q, u}, costOut);
    MX f = MX::sum1(costOut.at(0));

    // Create the NLP.
    MXDict nlp;
    nlp.emplace(std::make_pair("x", x));
    nlp.emplace(std::make_pair("f", f));
    casadi::Dict opts;
    casadi::Dict optsIpopt;
    double tol = 1e-3;
    optsIpopt["tol"] = tol;
    optsIpopt["dual_inf_tol"] = tol;
    optsIpopt["compl_inf_tol"] = tol;
    optsIpopt["acceptable_tol"] = tol;
    optsIpopt["acceptable_dual_inf_tol"] = tol;
    optsIpopt["acceptable_compl_inf_tol"] = tol;
    opts["ipopt"] = optsIpopt;
    casadi::Function solver = nlpsol("solver", "ipopt", nlp, opts);

    // Set the bounds and initial guess.
    SimTK::RowVector row = states.getNearestRow(time);
    std::vector<double> x0;
    std::vector<double> lbx;
    std::vector<double> ubx;
    for (int i = 0; i < row.size(); ++i) {
        x0.push_back(row[i]);
        lbx.push_back(-std::numeric_limits<double>::infinity());
        ubx.push_back(std::numeric_limits<double>::infinity());
        // lbx.push_back(row[i] - 0.25*std::abs(row[i]));
        // ubx.push_back(row[i] + 0.25*std::abs(row[i]));
    }

    // Solve the problem
    DMDict input = {{"lbx", lbx},
                    {"ubx", ubx},
                    {"x0",  x0}};
    DMDict output = solver(input);

    // Print the optimal solution
    std::vector<double> xopt(output.at("x"));
    SimTK::RowVector solution(nq+nu, xopt.data());

    return solution;
}

void contactTracking() {

    // Load the input data.
    Model model("feet.osim");
    model.initSystem();
    TimeSeriesTable grfs("grf_walk.mot");
    TableProcessor tableProcessor = 
            TableProcessor("feet_coordinate_reference.sto") |
            TabOpAppendCoordinateValueDerivativesAsSpeeds();
    TimeSeriesTable coordinates = tableProcessor.process(&model);

    // Define the contact forces.
    std::vector<std::string> contactForcesRight = {"contactHeel_r", 
            "contactLateralRearfoot_r", "contactLateralMidfoot_r", 
            "contactMedialMidfoot_r", "contactLateralToe_r", 
            "contactMedialToe_r"};
    std::vector<std::string> contactForcesLeft = {"contactHeel_l", 
            "contactLateralRearfoot_l", "contactLateralMidfoot_l", 
            "contactMedialMidfoot_l", "contactLateralToe_l", 
            "contactMedialToe_l"};

    // Sort the ground reaction forces.
    std::vector<std::string> grfsLabelsInOrder = {"ground_force_l_vx", 
            "ground_force_l_vy", "ground_force_l_vz", "ground_force_r_vx",
            "ground_force_r_vy", "ground_force_r_vz"};

    TimeSeriesTable grfsInOrder(grfs.getIndependentColumn());
    for (const auto& label : grfsLabelsInOrder) {
        grfsInOrder.appendColumn(label, grfs.getDependentColumn(label));
    }

    std::vector<std::string> stateLabelsInOrder;
    auto coordinateRefs = model.getCoordinatesInMultibodyTreeOrder();
    for (int i = 0; i < (int)coordinateRefs.size(); ++i) {
        std::string valuePath = fmt::format("{}/value", 
                coordinateRefs[i]->getAbsolutePathString());
        stateLabelsInOrder.push_back(valuePath);
    }
    for (int i = 0; i < (int)coordinateRefs.size(); ++i) {
        std::string speedPath = fmt::format("{}/speed", 
                coordinateRefs[i]->getAbsolutePathString());
        stateLabelsInOrder.push_back(speedPath);
    }

    TimeSeriesTable statesInOrder(coordinates.getIndependentColumn());
    for (const auto& label : stateLabelsInOrder) {
        statesInOrder.appendColumn(label, 
                coordinates.getDependentColumn(label));
    }

    ContactTrackingObjective objfun(model, grfsInOrder, statesInOrder, 
            contactForcesLeft, contactForcesRight);


    const auto& times = coordinates.getIndependentColumn();


    
    TimeSeriesTable statesSolutionTable;
    TimeSeriesTable forcesSolutionTable;

    SimTK::State state = model.initSystem();
    for (int itime = 50; itime < 100; ++itime) {
        double time = times[itime];
        SimTK::RowVector row = solveContactTrackingProblem(objfun, time, 
                statesInOrder);
        statesSolutionTable.appendRow(time, row);

        SimTK::Vector q(state.getNQ(), row.getContiguousScalarData(), true);
        SimTK::Vector u(state.getNU(), 
                row.getContiguousScalarData() + state.getNQ(), true);

        SimTK::Vec3 forcesLeft(0);
        SimTK::Vec3 forcesRight(0);
        calcContactForces(model, state, q, u, 
                objfun.getContactForcesLeft(), objfun.getContactForcesRight(),
                forcesLeft, forcesRight);

        SimTK::RowVector forcesRow(6);
        forcesRow[0] = forcesLeft[0];
        forcesRow[1] = forcesLeft[1];
        forcesRow[2] = forcesLeft[2];
        forcesRow[3] = forcesRight[0];
        forcesRow[4] = forcesRight[1];
        forcesRow[5] = forcesRight[2];

        forcesSolutionTable.appendRow(time, forcesRow);
    }
    statesSolutionTable.setColumnLabels(stateLabelsInOrder);
    statesSolutionTable.addTableMetaData<std::string>("inDegrees", "no");
    STOFileAdapter::write(statesSolutionTable, "contact_initializer_solution_states.sto");

    forcesSolutionTable.setColumnLabels(grfsLabelsInOrder);
    STOFileAdapter::write(forcesSolutionTable, "contact_initializer_solution_forces.sto");

    VisualizerUtilities::showMotion(model, 
        TimeSeriesTable("contact_initializer_solution_states.sto"));

}

int main() {
    // rocket();
    contactTracking();

    return EXIT_SUCCESS;
}


