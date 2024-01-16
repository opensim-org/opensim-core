/* -------------------------------------------------------------------------- *
 *                     OpenSim: TaskSpaceUtilities.h                          *
 * -------------------------------------------------------------------------- *
 * Developed by CFD Research Corporation for a project sponsored by the US    *
 * Army Medical Research and Materiel Command under Contract No.              *
 * W81XWH-22-C-0020. Any views, opinions and/or findings expressed in this    *
 * material are those of the author(s) and should not be construed as an      *
 * official Department of the Army position, policy or decision unless so     *
 * designated by other documentation.                                         *
 *                                                                            *
 * Please refer to the following publication for mathematical details, and    *
 * cite this paper if you use this code in your own research:                 *
 *                                                                            *
 * Pickle and Sundararajan. "Predictive simulation of human movement in       *
 * OpenSim using floating-base task space control".                           *
 *                                                                            *
 * Copyright (c) 2023 CFD Research Corporation and the Authors                *
 *                                                                            *
 * Author(s): Aravind Sundararajan, Nathan Pickle, Garrett Tuer               *
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
#ifndef OPENSIM_TASK_SPACE_UTILITIES_H_
#define OPENSIM_TASK_SPACE_UTILITIES_H_

#include "simbody/internal/MobilizedBody.h"
#include "OpenSim/Simulation/Model/Model.h"

/**
 * \brief Struct that holds matrices relevant to support consistent modeling
 * such that each map value is keyed with the MobilizedBodyIndex of the
 * supporting link.
 *
 */
namespace OpenSim {

//================================================================================
//                              CONVENIENCE FUNCTIONS
//================================================================================

/**
 * @brief  wraps SimbodyMatterSubsystems's calcM()
 *
 * @param s
 * @param model
 * @return Matrix
 */
SimTK::Matrix calcM(
        const SimTK::State& s, const OpenSim::Model& model);

/**
 * @brief return SVD inverse of M.
 *
 * @param s
 * @param model
 * @return Matrix
 */
SimTK::Matrix calcMInv(
        const SimTK::State& s, const OpenSim::Model& model);

/**
 * @brief wraps SimbodyMatterSubsystem's calcFrameJacobian(). Used to compute
 * components of Support Consistent Jacobian. For use with Support Model.
 *
 * @param s
 * @param model
 * @param b
 * @param loc_in_B
 * @return Matrix
 */

/**
 * @brief wraps SimbodyMatterSubsystem's calcFrameJacobian(). Used to compute
 * components of Support Consistent Jacobian. For use with Support Model.
 *
 * @param s
 * @param model
 * @param b
 * @param loc_in_B
 * @return Matrix
 */
SimTK::Matrix calcFrameJacobian(const SimTK::State& s,
        const OpenSim::Model& model, const SimTK::MobilizedBodyIndex& b,
        const SimTK::Vec3& loc_in_B);

/**
 * Calculates the total force that act on the model (\f$ f \f$). This requires
 * that the model is realized to Stage::Dynamics. A working model is used as
 * this method may be called by a controller during numerical integration and
 * all controllers of the working model are removed to avoid infinite loops.
 * The actuators of the working model are disabled since they are the actuation
 * (i.e. muscles \f$ \tau = R f_m \f$ and not \f$ f \f$). Call this method only
 * from objects that are derived from OpenSim::Controller and never from
 * objects that are derived from OpenSim::Force.
 */
SimTK::Vector calcTotalGeneralizedForces(const SimTK::State& s,
                                         const OpenSim::Model& model);


//================================================================================
//                              MATH UTILITIES
//================================================================================

/**
 * SVD Pseudoinverse wrapped from Simbody FactorSVD.inverse() 
 *
 * @param A
 * @return Matrix
 */
SimTK::Matrix FactorSVDPseudoinverse(const SimTK::Matrix& A);

/**
 * @brief Selectively damped least-squares inverse with SVD based on the
 * specified threshold tolerance.
 *
 * @param A
 * @param tolerance
 * @return Matrix
 */
SimTK::Matrix DampedSVDPseudoinverse(
    const SimTK::Matrix& A,
    const double& tolerance=std::numeric_limits<float>::epsilon());

/**
 * \brief Convenience method for turning a 3x2 spatial vec into a 6x1 Matrix
 */
SimTK::Matrix FlattenSpatialVec(const SimTK::SpatialVec& S);

/**
 * @brief Concatenate 2 Matrices along the specified dimension dim.
 *
 * @param A
 * @param B
 * @param dim
 * @return Matrix
 */
SimTK::Matrix concatenate(
        const SimTK::Matrix& A, const SimTK::Matrix& B, const int dim);

/**
 * @brief Construct a diagonal matrix from a vector
 *
 * @param A
 * @return Matrix
 */
SimTK::Matrix diagonalize(const SimTK::Vector& A);

/**
 * @brief Create a block diagonal matrix.
 *
 * @param A
 * @param B
 * @return Matrix
 */
SimTK::Matrix diagonalize(
        const SimTK::Matrix& A, const SimTK::Matrix& B);



} // namespace OpenSim
#endif
