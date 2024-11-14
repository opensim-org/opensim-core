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

using namespace OpenSim;
using namespace casadi;

class ContactTrackingObjective : public Callback {
public:
    // Constructor
    ContactTrackingObjective(Model model,
            TimeSeriesTable grfs,
            TimeSeriesTable coordinates,
            const std::vector<std::string>& contactForcesLeft,
            const std::vector<std::string>& contactForcesRight,
            const std::vector<std::string>& grfsLeft,
            const std::vector<std::string>& grfsRight) {

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
        for (const auto& grf : grfsLeft) {
            m_grfIndexesLeft.push_back(
                static_cast<int>(grfs.getColumnIndex(grf)));
        }
        for (const auto& grf : grfsRight) {
            m_grfIndexesRight.push_back(
                static_cast<int>(grfs.getColumnIndex(grf)));
        }

        // Coordinates.
        m_coordinates = GCVSplineSet(coordinates);
        auto coordinateRefs = m_model.getCoordinatesInMultibodyTreeOrder();
        for (int i = 0; i < (int)coordinateRefs.size(); ++i) {
            std::string valuePath = fmt::format("{}/value", 
                    coordinateRefs[i]->getAbsolutePathString());
            m_coordinateMap[i] = (int)coordinates.getColumnIndex(valuePath);
        }
        
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

        m_state.updQ() = q;
        m_state.updU() = u;
        m_model.realizeVelocity(m_state);


        // Force errors.
        SimTK::Vec3 errorsLeft(0.0);
        SimTK::Vec3 errorsRight(0.0);
        for (const auto& contactForce : m_contactForcesLeft) {
            errorsLeft += contactForce->getSphereForce(m_state)[1];
        }
        for (const auto& contactForce : m_contactForcesRight) {
            errorsRight += contactForce->getSphereForce(m_state)[1];
        }

        errorsLeft[0] -= m_grfs.get(m_grfIndexesLeft[0]).calcValue(m_time);
        errorsLeft[1] -= m_grfs.get(m_grfIndexesLeft[1]).calcValue(m_time);
        errorsLeft[2] -= m_grfs.get(m_grfIndexesLeft[2]).calcValue(m_time);
        errorsRight[0] -= m_grfs.get(m_grfIndexesRight[0]).calcValue(m_time);
        errorsRight[1] -= m_grfs.get(m_grfIndexesRight[1]).calcValue(m_time);
        errorsRight[2] -= m_grfs.get(m_grfIndexesRight[2]).calcValue(m_time);

        // Coordinate errors.
        double coordinateError = 0.0;
        for (const auto& kv : m_coordinateMap) {
            double error = q[kv.first] - 
                           m_coordinates.get(kv.second).calcValue(m_time);
            coordinateError += error * error;
        }
           
        DM f = m_forceWeight * (errorsLeft.normSqr() * errorsRight.normSqr()) + 
               m_coordinateWeight * coordinateError +
               m_speedWeight * u.normSqr();
        return {f};
    }

private:
    Model m_model;
    mutable SimTK::State m_state;
    SimTK::Vector m_time;

    GCVSplineSet m_grfs;
    std::vector<int> m_grfIndexesLeft;
    std::vector<int> m_grfIndexesRight;

    using ContactForceRefs = 
            std::vector<SimTK::ReferencePtr<const SmoothSphereHalfSpaceForce>>;
    ContactForceRefs m_contactForcesLeft;
    ContactForceRefs m_contactForcesRight;

    GCVSplineSet m_coordinates;
    std::unordered_map<int, int> m_coordinateMap;

    double m_forceWeight = 1.0;
    double m_coordinateWeight = 1.0;
    double m_speedWeight = 1e-3;
};

SimTK::RowVector solveContactTrackingProblem(ContactTrackingObjective& objfun, 
        double time) {

    // The number of states. For each foot, we have a 6 DOF joint between the 
    // ground and the calcaneus, and a 1 DOF joint between the calcaneus and
    // the toes.
    int nq = 14;      // number of generalized coordinates
    int nu = nq;      // number of generalized speeds
    int ns = nq + nu; // number of states

    // Define the optimization variables.
    MX q = MX::sym("q", nq, 1); // coordinates
    MX u = MX::sym("u", nu, 1); // speeds
    MX x = vertcat(q, u);    // states

    // Objective function
    objfun.setTime(time);
    objfun.setForceWeight(1.0);
    objfun.setCoordinateWeight(10.0);
    objfun.setSpeedWeight(1e-3);

    MXVector costOut;
    objfun.call({q, u}, costOut);
    MX f = MX::sum1(costOut.at(0));

    // Create the NLP
    MXDict nlp;
    nlp.emplace(std::make_pair("x", x));
    nlp.emplace(std::make_pair("f", f));

    // Allocate an NLP solver and buffers
    casadi::Function solver = nlpsol("solver", "ipopt", nlp);

    std::vector<double> lbx(nq+nu, -std::numeric_limits<double>::infinity());
    std::vector<double> ubx(nq+nu, std::numeric_limits<double>::infinity());
    std::vector<double> x0(nq+nu, 0.0);

    // Solve the problem
    DMDict input = {{"lbx", lbx},
                    {"ubx", ubx},
                    {"x0",  x0}};
    DMDict output = solver(input);

    // Print the optimal cost
    double cost(output.at("f"));
    std::cout << "optimal cost: " << cost << std::endl;

    // Print the optimal solution
    std::vector<double> xopt(output.at("x"));
    std::cout << "optimal configuration: " << xopt << std::endl;
    SimTK::RowVector solution(nq+nu, xopt.data());

    return solution;
}

void contactTracking() {

    // Load the "feet" model.
    Model model("feet.osim");
    TimeSeriesTable grfs("grf_walk.mot");
    TimeSeriesTable coordinates("feet_coordinate_reference.sto");

    std::vector<std::string> contactForcesLeft = {"contactHeel_r", 
            "contactLateralRearfoot_r", "contactLateralMidfoot_r", 
            "contactMedialMidfoot_r", "contactLateralToe_r", 
            "contactMedialToe_r"};
    std::vector<std::string> contactForcesRight = {"contactHeel_l", 
            "contactLateralRearfoot_l", "contactLateralMidfoot_l", 
            "contactMedialMidfoot_l", "contactLateralToe_l", 
            "contactMedialToe_l"};
    std::vector<std::string> grfsLeft = {"ground_force_l_vx", 
            "ground_force_l_vy", "ground_force_l_vz"};
    std::vector<std::string> grfsRight = {"ground_force_r_vx",
            "ground_force_r_vy", "ground_force_r_vz"};

    ContactTrackingObjective objfun(model, grfs, coordinates, 
            contactForcesLeft, contactForcesRight, grfsLeft, grfsRight);

    
    TimeSeriesTable solutionTable;

    std::vector<std::string> columnLabels;
    auto coordinateRefs = model.getCoordinatesInMultibodyTreeOrder();
    for (int i = 0; i < (int)coordinateRefs.size(); ++i) {
        std::string valuePath = fmt::format("{}/value", 
                coordinateRefs[i]->getAbsolutePathString());
        columnLabels.push_back(valuePath);
    }
    for (int i = 0; i < (int)coordinateRefs.size(); ++i) {
        std::string speedPath = fmt::format("{}/speed", 
                coordinateRefs[i]->getAbsolutePathString());
        columnLabels.push_back(speedPath);
    }

    SimTK::RowVector row = solveContactTrackingProblem(objfun, 0.5);


}

int main() {
    // rocket();
    contactTracking();

    return EXIT_SUCCESS;
}


