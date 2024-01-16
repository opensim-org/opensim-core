/* -------------------------------------------------------------------------- *
 *                   OpenSim: TaskSpaceUtilities.cpp                          *
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

#include "TaskSpaceUtilities.h"

using namespace std;
using namespace SimTK;

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
Matrix calcM(const State& s, const Model& model) {
    Matrix M;
    model.getMatterSubsystem().calcM(s, M);
    return M;
}

/**
 * @brief return SVD inverse of M.
 *
 * @param s
 * @param model
 * @return Matrix
 */
Matrix calcMInv(const State& s, const Model& model) {
    Matrix MInv, M;
    model.getMatterSubsystem().calcM(s, M);
    MInv = FactorSVDPseudoinverse(M);
    return MInv;
}

/**
 * @brief wraps SimbodyMatterSubsystem's calcFrameJacobian()
 *
 * @param s
 * @param model
 * @param b
 * @param loc_in_B
 * @return Matrix
 */
Matrix calcFrameJacobian(const State& s, const Model& model,
        const SimTK::MobilizedBodyIndex& b, const Vec3& loc_in_B) {
    SimTK::Matrix J;
    model.getMultibodySystem().getMatterSubsystem().calcFrameJacobian(
            s, b, loc_in_B, J);
    return J;
}

/**
 * This function calculates the total applied forces. In order to calculate them
 * the model must be realized to Stage::Dynamics. This method is typically
 * called in mixed dynamics scheme thus while the model is numerically
 * integrated (forward dynamics) we may be interested in the acting forces so
 * that a controller (e.g. inverse dynamics torque controller) can calculate the
 * controls correctly. If however the original model is realized to
 * Stage::Dynamics the controller's computeControls will be called again and
 * this will cause an infinite loop. To avoid this problem upon first entrance
 * to this function a working copy of the model is created without the
 * controller and this working model is realized to Stage::Dynamics. Each time
 * the working state is updated from the current state of the simulating model
 * and the variables of the working model's working state.
 */
Vector calcTotalForces(const State& s, const Model& model) {
    // create a working instance of the model, upon first entrance to this
    // function
    static Model* workingModel = NULL;
    if (workingModel == NULL) {
        workingModel = model.clone();
        workingModel->setUseVisualizer(false);
        // remove controllers to avoid infinite loop
        workingModel->updControllerSet().setSize(0);
        // disable any actuators when computing the total force
        auto& fs = workingModel->updForceSet();
        for (int i = 0; i < fs.getSize(); i++) {
            auto pathActuator = dynamic_cast<PathActuator*>(&fs[i]);
            if (pathActuator) {
                pathActuator->set_appliesForce(false);
            }
        }
        workingModel->finalizeFromProperties();
        workingModel->initSystem();
    }
    // initialize working state from s
    auto workingState = workingModel->updWorkingState();
    workingState.setTime(s.getTime());
    workingState.setQ(s.getQ());
    workingState.setU(s.getU());
    workingState.setZ(s.getZ());
    workingState.setY(s.getY());
    // generalized forces and torques should be accessed at a Dynamics stage
    workingModel->realizeDynamics(workingState);
    // get acting torques and body forces
    auto bodyForces = workingModel->getMultibodySystem()
        .getRigidBodyForces(workingState, Stage::Dynamics);
    auto generalizedForces = workingModel->getMultibodySystem()
        .getMobilityForces(workingState, Stage::Dynamics);
    // map body forces to joint forces
    Vector jointForces;
    workingModel->getMatterSubsystem()
        .multiplyBySystemJacobianTranspose(workingState, bodyForces, jointForces);
    // in our convention we subtract their contribution
    return -1.0 * generalizedForces - jointForces;
}

/**
 * Calculates the Coriolis contribution \f$ \tau_c \f$. We assume that
 * \f$ \tau_c \f$ is on the same side with the joint space accelerations
 * (i.e. \f$ M \ddot{q} + \tau_c = \tau \f$).
 */
Vector calcCoriolis(const State& s, const Model& model) {
    Vector c;
    model.getMatterSubsystem().calcResidualForceIgnoringConstraints(
        s, Vector(0), Vector_<SpatialVec>(0), Vector(0), c);
    return c;
}

Vector calcTotalGeneralizedForces(const State& s, const Model& model) {
    // compute all acting forces add Coriolis since they are not accounted
    return  calcCoriolis(s, model) + calcTotalForces(s, model);
    // compute only Coriolis and gravity (works always)
    // return calcCoriolis(s, model) + calcGravity(s, model);
}

//================================================================================
//                               MATH UTILITIES
//================================================================================

Matrix concatenate(const Matrix &A, const Matrix &B, const int dim) {
    if (A.nelt() == 0) { return B; }
    if (B.nelt() == 0) { return A; }
    
    Matrix C;
    if (dim == 0) {
        // Concatenate vertically
        if (A.ncol() != B.ncol()) {
            OPENSIM_THROW(
                OpenSim::Exception,
                "Matrices being concatenated must have same number of columns.")
        }
        C.resize(A.nrow() + B.nrow(), A.ncol());
        C.updBlock(0, 0, A.nrow(), A.ncol()) = A;
        C.updBlock(A.nrow(), 0, B.nrow(), B.ncol()) = B;
    } else if (dim == 1) {
        // Concatenate horizontally
        if (A.nrow() != B.nrow()) {
            OPENSIM_THROW(
                OpenSim::Exception,
                "Matrices being concatenated must have same number of rows.")
        }
        C.resize(A.nrow(), A.ncol() + B.ncol());
        C.updBlock(0, 0, A.nrow(), A.ncol()) = A;
        C.updBlock(0, A.ncol(), B.nrow(), B.ncol()) = B;
    } else {
        OPENSIM_THROW(OpenSim::Exception, "Dimension must be 0 or 1.")
    }
    return C;
}

/**
 * \brief Convenience method for turning a spatial vec into a matrix
 */
Matrix FlattenSpatialVec(const SpatialVec& S) {
    // turn a spatialvec into a 6x1 matrix.
    Matrix spatialVecFlat(6, 1, 0.0);
    spatialVecFlat[0] = S[0][0];
    spatialVecFlat[1] = S[0][1];
    spatialVecFlat[2] = S[0][2];
    spatialVecFlat[3] = S[1][0];
    spatialVecFlat[4] = S[1][1];
    spatialVecFlat[5] = S[1][2];
    return spatialVecFlat;
}

/**
 * @brief SVD Pseudoinverse wrapped from Simbody FactorSVD.inverse()
 *
 * @param A
 * @return Matrix
 */
Matrix FactorSVDPseudoinverse(const Matrix& A) {
    Matrix Ainv;
    FactorSVD AInvSVD(A);
    try {
        AInvSVD.inverse(Ainv);
    } catch (...) {
        OPENSIM_THROW(OpenSim::Exception,
                "SVD inverse failed because the matrix is rank deficient or "
                "ill-conditioned, try with a different technique or check the "
                "joint configuration if you are near a singular configuration "
                "of the system.");
    }
    return Ainv;
}

/**
 * @brief Selectively damped least-squares inverse with SVD based on the
 * threshold tolerance defined in InverseDynamicsModel.cpp
 *
 * @param A
 * @return Matrix
 */
Matrix DampedSVDPseudoinverse(const Matrix& A, const double& tolerance) {
    Matrix Ainv;
    Matrix U = Matrix();
    Matrix VT = Matrix();
    Vector S = Vector();
    FactorSVD AInvSVD(A);
    try {
        AInvSVD.getSingularValuesAndVectors(S, U, VT);

        auto dInv = S;
        for (int k = 0; k < dInv.nrow(); ++k) {
            if (S(k) < tolerance) {
                dInv(k) = std::pow(tolerance, 2.0); // 0.0;
            } else {
                dInv(k) = 1 / S(k);
            }
        }
        Matrix SInv = diagonalize(dInv);
        Ainv = VT.transpose() * SInv * U.transpose();
    } catch (...) {
        OPENSIM_THROW(OpenSim::Exception,
                "SVD inverse failed because the matrix is rank deficient or "
                "ill-conditioned,",
                " try with a different technique or check the joint "
                "configuration if you are near a singular configuration of the "
                "system.");
    }
    return Ainv;
}

/**
 * @brief Construct a diagonal matrix from a vector
 *
 * @param A
 * @return Matrix
 */
Matrix diagonalize(const Vector& A) {
    Matrix B = Matrix(A.nrow(), A.nrow(), 0.0);
    for (int i = 0; i < A.nelt(); ++i) { B(i, i) = A(i); }
    return B;
}

/**
 * @brief Create a block diagonal matrix.
 *
 * @param A
 * @param B
 * @return Matrix
 */
Matrix diagonalize(const Matrix& A, const Matrix& B) {
    Matrix C(A.nrow() + B.nrow(), A.ncol() + B.ncol(), 0.0);
    C.updBlock(0, 0, A.nrow(), A.ncol()) = A;
    C.updBlock(A.nrow(), A.ncol(), B.nrow(), B.ncol()) = B;

    return C;
}
} // namespace OpenSim