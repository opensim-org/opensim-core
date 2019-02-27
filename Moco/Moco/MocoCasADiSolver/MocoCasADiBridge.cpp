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
template <bool T>
thread_local SimTK::Vector MocoCasADiMultibodySystemImplicit<T>::m_residual;


template class OpenSim::MocoCasADiMultibodySystem<false>;
template class OpenSim::MocoCasADiMultibodySystem<true>;

template class OpenSim::MocoCasADiMultibodySystemImplicit<false>;
template class OpenSim::MocoCasADiMultibodySystemImplicit<true>;
