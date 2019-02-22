/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoCasADiBridge.cpp                                         *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2018 Stanford University and the Authors                     *
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

#include "MocoCasADiBridge.h"

using namespace OpenSim;

thread_local SimTK::Vector MocoCasADiPathConstraint::m_errors;

template <bool T>
thread_local SimTK::Vector_<SimTK::SpatialVec>
        MocoCasADiMultibodySystem<T>::m_constraintBodyForces;
template <bool T>
thread_local SimTK::Vector
        MocoCasADiMultibodySystem<T>::m_constraintMobilityForces;
template <bool T>
thread_local SimTK::Vector MocoCasADiMultibodySystem<T>::m_udot;
template <bool T>
thread_local SimTK::Vector_<SimTK::SpatialVec>
        MocoCasADiMultibodySystem<T>::m_A_GB;
template <bool T>
thread_local SimTK::Vector MocoCasADiMultibodySystem<T>::m_pvaerr;

thread_local SimTK::Vector MocoCasADiVelocityCorrection::m_qdotCorr;

template <bool T>
thread_local SimTK::Vector_<SimTK::SpatialVec>
        MocoCasADiMultibodySystemImplicit<T>::m_constraintBodyForces;
template <bool T>
thread_local SimTK::Vector
        MocoCasADiMultibodySystemImplicit<T>::m_constraintMobilityForces;
template <bool T>
thread_local SimTK::Vector MocoCasADiMultibodySystemImplicit<T>::m_pvaerr;


template <bool CalcKinConErrors>
VectorDM MocoCasADiMultibodySystem<CalcKinConErrors>::eval(
        const VectorDM& args) const {
    const double& time = args.at(0).scalar();
    const casadi::DM& states = args.at(1);
    const casadi::DM& controls = args.at(2);
    const casadi::DM& multipliers = args.at(3);
    const casadi::DM& parameters = args.at(4);
    VectorDM out;

    auto mocoProblemRep = m_jar.take();
    // TODO: deal with constness better.
    auto& model = const_cast<Model&>(mocoProblemRep->getModel());
    auto& simtkState = model.updWorkingState();
    applyParametersToModel(SimTK::Vector(this->m_casProblem->getNumParameters(),
                                   parameters.ptr(), true),
            *mocoProblemRep);
    convertToSimTKState(time, states, controls, model, m_yIndexMap, simtkState);

    // If enabled constraints exist in the model, compute accelerations
    // based on Lagrange multipliers.
    // The total number of scalar holonomic, non-holonomic, and acceleration
    // constraint equations enabled in the model. This does not count
    // equations for derivatives of holonomic and non-holonomic constraints.
    const int total_mp =
            this->m_casProblem->getNumHolonomicConstraintEquations();
    const int total_mv =
            this->m_casProblem->getNumNonHolonomicConstraintEquations();
    const int total_ma =
            this->m_casProblem->getNumAccelerationConstraintEquations();
    // This is the sum of m_total_m(p|v|a).
    const int numMultipliers = this->m_casProblem->getNumMultipliers();
    if (numMultipliers) {
        const auto& enforceConstraintDerivatives =
                m_mocoCasADiSolver.get_enforce_constraint_derivatives();

        model.realizeDynamics(simtkState);

        const SimTK::MultibodySystem& multibody = model.getMultibodySystem();
        const SimTK::Vector_<SimTK::SpatialVec>& appliedBodyForces =
                multibody.getRigidBodyForces(
                        simtkState, SimTK::Stage::Dynamics);
        const SimTK::Vector& appliedMobilityForces =
                multibody.getMobilityForces(simtkState, SimTK::Stage::Dynamics);

        const SimTK::SimbodyMatterSubsystem& matter =
                model.getMatterSubsystem();

        // Multipliers are negated so constraint forces can be used like
        // applied forces.
        SimTK::Vector simtkMultipliers(numMultipliers, multipliers.ptr(), true);
        matter.calcConstraintForcesFromMultipliers(simtkState,
                -simtkMultipliers, constraintBodyForces,
                constraintMobilityForces);

        matter.calcAccelerationIgnoringConstraints(simtkState,
                appliedMobilityForces + constraintMobilityForces,
                appliedBodyForces + constraintBodyForces, udot, A_GB);

        // Constraint errors.
        // TODO double-check that disabled constraints don't show up in
        // state
        out.resize(2);
        if (CalcKinConErrors) {
            // Position-level errors.
            casadi::DM out_kinematic_constraint_errors =
                    convertToCasADiDM(simtkState.getQErr());

            if (enforceConstraintDerivatives || total_ma) {
                // Calculuate udoterr. We cannot use State::getUDotErr()
                // because that uses Simbody's multiplilers and UDot,
                // whereas we have our own multipliers and UDot.
                matter.calcConstraintAccelerationErrors(
                        simtkState, udot, m_pvaerr);
            } else {
                m_pvaerr = SimTK::NaN;
            }

            casadi::DM uerr;
            casadi::DM udoterr;
            if (enforceConstraintDerivatives) {
                // Velocity-level errors.
                uerr = convertToCasADiDM(simtkState.getUErr());
                // Acceleration-level errors.
                udoterr = convertToCasADiDM(m_pvaerr);
            } else {
                // Velocity-level errors. Skip derivatives of position-level
                // constraint equations.
                uerr = convertToCasADiDM(SimTK::Vector(total_mv,
                        simtkState.getUErr().getContiguousScalarData() +
                                total_mp,
                        true));
                // Acceleration-level errors. Skip derivatives of velocity-
                // and position-level constraint equations.
                udoterr = convertToCasADiDM(SimTK::Vector(total_ma,
                        m_pvaerr.getContiguousScalarData() + total_mp +
                                total_mv,
                        true));
            }
            out_kinematic_constraint_errors = casadi::DM::vertcat(
                    {out_kinematic_constraint_errors, uerr, udoterr});

            // Copy state derivative values to output. We cannot simply
            // use getYDot() because that requires realizing to
            // Acceleration.
            out.push_back(out_kinematic_constraint_errors);
        }
        out[0] = convertToCasADiDM(udot);
        // TODO: zdot probably depends on realizing to Acceleration.
        out[1] = convertToCasADiDM(simtkState.getZDot());

    } else {
        // If no constraints exist in the model, simply compute
        // accelerations directly from Simbody.
        model.realizeAcceleration(simtkState);

        out = {convertToCasADiDM(simtkState.getUDot()),
                convertToCasADiDM(simtkState.getZDot())};
        if (CalcKinConErrors) {
            // Add an empty kinematic constraint error vector.
            out.emplace_back(0, 1);
        }
    }
    m_jar.leave(std::move(mocoProblemRep));
    return out;
}

template class OpenSim::MocoCasADiMultibodySystem<false>;
template class OpenSim::MocoCasADiMultibodySystem<true>;
