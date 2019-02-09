/* -------------------------------------------------------------------------- *
 * OpenSim Muscollo: CasADiTrapezoidal.cpp                                    *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2018 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s):                                                                 *
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
#include "CasADiTrapezoidal.h"

#include <OpenSim/Simulation/InverseDynamicsSolver.h>

void CasADiTrapezoidal::addConstraintsImpl() {

    m_dynamicsFunction =
            make_unique<DynamicsFunction>("dynamics", *this, m_probRep);

    // Compute qdot symbolically.
    OPENSIM_THROW_IF(m_state.getNQ() != m_state.getNU(), Exception);
    const int NQ = m_state.getNQ();

    // Bounds for multibody constraint errors.
    // ---------------------------------------
    // Add any scalar constraints associated with multibody constraints in
    // the model as path constraints in the problem.
    std::vector<std::string> mcNames =
            m_probRep.createKinematicConstraintNames();
    DM mcLowerBounds(m_numMultibodyConstraintEqs, 1);
    DM mcUpperBounds(m_numMultibodyConstraintEqs, 1);
    // TODO make sure the mcNames are in the same order as the State::getQErr().
    int multIndex = 0;
    for (const auto& mcName : mcNames) {
        const auto& mc = m_probRep.getKinematicConstraint(mcName);
        const auto& bounds = mc.getConstraintInfo().getBounds();
        const auto& kinLevels = mc.getKinematicLevels();
        for (int ibound = 0; ibound < (int)bounds.size(); ++ibound) {
            if (kinLevels[ibound] == KinematicLevel::Position ||
                    kinLevels[ibound] == KinematicLevel::Velocity ||
                    kinLevels[ibound] == KinematicLevel::Acceleration) {
                // Only grab the constraints for the position-level constraints.
                mcLowerBounds(multIndex) = bounds[ibound].getLower();
                mcUpperBounds(multIndex) = bounds[ibound].getUpper();
                ++multIndex;
            }
        }
    }

    // Dynamics defects and multibody constraints.
    // -------------------------------------------
    const auto& states = m_vars[Var::states];
    const auto calcDAE = [&](casadi_int itime, MX& xdot, MX& qerr) {
        MX qdot = states(Slice(NQ, 2 * NQ), itime);
        auto dynamicsOutput = m_dynamicsFunction->operator()(
                {m_vars[Var::initial_time],
                 m_vars[Var::states](Slice(), itime),
                 m_vars[Var::controls](Slice(), itime),
                 m_vars[Var::multipliers](Slice(), itime),
                 m_vars[Var::parameters]
                });
        MX udotzdot = dynamicsOutput.at(0);
        xdot = casadi::MX::vertcat({qdot, udotzdot});
        qerr = dynamicsOutput.at(1);
    };
    MX qerr;
    MX xdot_i;
    MX xdot_im1;
    calcDAE(0, xdot_im1, qerr);
    if (m_numMultibodyConstraintEqs) {
        m_opti.subject_to(mcLowerBounds <= qerr <= mcUpperBounds);
    }
    for (int itime = 1; itime < m_numTimes; ++itime) {
        auto h = m_times(itime) - m_times(itime - 1);
        auto x_i = states(Slice(), itime);
        auto x_im1 = states(Slice(), itime - 1);
        calcDAE(itime, xdot_i, qerr);
        m_opti.subject_to(x_i == (x_im1 + 0.5 * h * (xdot_i + xdot_im1)));
        if (m_numMultibodyConstraintEqs) {
            m_opti.subject_to(mcLowerBounds <= qerr <= mcUpperBounds);
        }
        xdot_im1 = xdot_i;
    }
}

int CasADiTrapezoidal::DynamicsFunction::eval(
        const double** inputs, double** outputs,
        casadi_int*, double*, void*) const {
    const double* time = inputs[0];
    const double* states = inputs[1];
    const double* controls = inputs[2];
    const double* multipliers = inputs[3];
    const double* parameters = inputs[4];
    double* out_derivatives = outputs[0];
    double* out_multibody_constraint_errors = outputs[1];
    m_transcrip.applyParametersToModel(
            SimTK::Vector(p.getNumParameters(), parameters, true));
    const auto& model = p.getModel();
    auto& simtkState = m_transcrip.m_state;
    convertToSimTKState(time, states, controls, p.getModel(), simtkState);
    // If enabled constraints exist in the model, compute accelerations
    // based on Lagrange multipliers.
    if (m_transcrip.getNumMultibodyConstraintEquations()) {
        // TODO Antoine and Gil said realizing Dynamics is a lot costlier than
        // realizing to Velocity and computing forces manually.
        model.realizeDynamics(simtkState);

        const SimTK::MultibodySystem& multibody =
                model.getMultibodySystem();
        const SimTK::Vector_<SimTK::SpatialVec>& appliedBodyForces =
                multibody.getRigidBodyForces(simtkState,
                        SimTK::Stage::Dynamics);
        const SimTK::Vector& appliedMobilityForces =
                multibody.getMobilityForces(simtkState, SimTK::Stage::Dynamics);

        const SimTK::SimbodyMatterSubsystem& matter =
                model.getMatterSubsystem();

        // TODO: Use working memory.
        SimTK::Vector_<SimTK::SpatialVec> constraintBodyForces;
        SimTK::Vector constraintMobilityForces;
        // If enabled constraints exist in the model, compute accelerations
        // based on Lagrange multipliers.

        // Multipliers are negated so constraint forces can be used like
        // applied forces.
        SimTK::Vector simtkMultipliers(m_transcrip.getNumMultipliers(),
                multipliers, true);
        matter.calcConstraintForcesFromMultipliers(simtkState,
                -simtkMultipliers,
                constraintBodyForces, constraintMobilityForces);

        SimTK::Vector udot;
        matter.calcAccelerationIgnoringConstraints(simtkState,
                appliedMobilityForces + constraintMobilityForces,
                appliedBodyForces + constraintBodyForces, udot, A_GB);

        // Constraint errors.
        // TODO double-check that disabled constraints don't show up in
        // state
        std::copy_n(simtkState.getQErr().getContiguousScalarData(),
                simtkState.getNQErr(), out_multibody_constraint_errors);

        // Copy state derivative values to output struct. We cannot simply
        // use getYDot() because that requires realizing to Acceleration.
        const int nu = udot.size();
        const int nz = simtkState.getNZ();
        std::copy_n(udot.getContiguousScalarData(), udot.size(),
                out_derivatives);
        // TODO: Does computing ZDot require realizing to Acceleration?
        std::copy_n(simtkState.getZDot().getContiguousScalarData(), nz,
                out_derivatives + nu);

    } else {
        p.getModel().realizeAcceleration(simtkState);

        // TODO create member variable for numRowsOut.
        int numRowsOut = simtkState.getNU() + simtkState.getNZ();
        std::copy_n(simtkState.getYDot().getContiguousScalarData() +
                        simtkState.getNQ(),
                numRowsOut, out_derivatives);
    }
    return 0;
}

void CasADiTrapezoidalImplicit::addConstraintsImpl() {
    m_dynamicsFunction =
            make_unique<DynamicsFunction>("dynamics", *this, m_probRep);
    m_residualFunction =
            make_unique<ResidualFunction>("residual", *this, m_probRep);

    const int NQ = m_state.getNQ();
    const auto& states = m_vars[Var::states];
    MX qdot = states(Slice(NQ, 2 * NQ), 0);
    MX udot = m_vars[Var::derivatives](Slice(), 0);
    MX zdot = m_dynamicsFunction->operator()(
            {m_vars[Var::initial_time],
             states(Slice(), 0),
             m_vars[Var::controls](Slice(), 0),
             m_vars[Var::parameters]
            }).at(0);
    MX xdot_im1 = casadi::MX::vertcat({qdot, udot, zdot});

    MX residual = m_residualFunction->operator()(
            {m_vars[Var::initial_time],
             m_vars[Var::states](Slice(), 0),
             m_vars[Var::controls](Slice(), 0),
             m_vars[Var::derivatives](Slice(), 0),
             m_vars[Var::parameters]
            }).at(0);
    m_opti.subject_to(residual == 0);
    for (int itime = 1; itime < m_numTimes; ++itime) {
        auto h = m_times(itime) - m_times(itime - 1);
        auto x_i = states(Slice(), itime);
        auto x_im1 = states(Slice(), itime - 1);
        qdot = states(Slice(NQ, 2 * NQ), itime);
        udot = m_vars[Var::derivatives](Slice(), itime);
        zdot = m_dynamicsFunction->operator()(
                {m_times(itime),
                 m_vars[Var::states](Slice(), itime),
                 m_vars[Var::controls](Slice(), itime),
                 m_vars[Var::parameters]}).at(0);
        MX xdot_i = MX::vertcat({qdot, udot, zdot});
        m_opti.subject_to(x_i == (x_im1 + 0.5 * h * (xdot_i + xdot_im1)));
        xdot_im1 = xdot_i;

        residual = m_residualFunction->operator()(
                {m_times(itime),
                 m_vars[Var::states](Slice(), itime),
                 m_vars[Var::controls](Slice(), itime),
                 m_vars[Var::derivatives](Slice(), itime),
                 m_vars[Var::parameters]
                }).at(0);
        m_opti.subject_to(residual == 0);
    }
}

casadi::Sparsity
CasADiTrapezoidalImplicit::DynamicsFunction::get_sparsity_out(casadi_int i) {
    if (i == 0) {
        return casadi::Sparsity::dense(m_transcrip.m_state.getNZ(), 1);
    }
    else return casadi::Sparsity(0, 0);
}
int CasADiTrapezoidalImplicit::DynamicsFunction::eval(
        const double** inputs, double** outputs,
        casadi_int*, double*, void*) const {
    m_transcrip.applyParametersToModel(
            SimTK::Vector(p.getNumParameters(), inputs[3], true));
    auto& state = m_transcrip.m_state;
    convertToSimTKState(inputs[0], inputs[1], inputs[2], p.getModel(), state);
    // TODO: Avoid calculating accelerations. Only calculate auxiliary dynamics.
    p.getModel().realizeAcceleration(state);

    std::copy_n(state.getZDot().getContiguousScalarData(), state.getNZ(),
            outputs[0]);
    return 0;
}

casadi::Sparsity
CasADiTrapezoidalImplicit::ResidualFunction::get_sparsity_out(casadi_int i) {
    if (i == 0) {
        int numRows = m_transcrip.m_state.getNU();
        return casadi::Sparsity::dense(numRows, 1);
    }
    else return casadi::Sparsity(0, 0);
}
int CasADiTrapezoidalImplicit::ResidualFunction::eval(
        const double** inputs, double** outputs,
        casadi_int*, double*, void*) const {
    m_transcrip.applyParametersToModel(
            SimTK::Vector(p.getNumParameters(), inputs[4], true));
    auto& state = m_transcrip.m_state;
    convertToSimTKState(inputs[0], inputs[1], inputs[2], p.getModel(), state);

    InverseDynamicsSolver id(m_transcrip.m_model);
    SimTK::Vector udot(state.getNU(), inputs[3], true);
    SimTK::Vector residual = id.solve(state, udot);

    std::copy_n(residual.getContiguousScalarData(), residual.size(),
            outputs[0]);
    return 0;
}
