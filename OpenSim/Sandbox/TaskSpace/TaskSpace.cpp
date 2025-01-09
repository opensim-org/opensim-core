/* ------------------------------------------------------------------------- *
*                               Simbody(tm)                                  *
* -------------------------------------------------------------------------- *
* This is part of the SimTK biosimulation toolkit originating from           *
* Simbios, the NIH National Center for Physics-Based Simulation of           *
* Biological Structures at Stanford, funded under the NIH Roadmap for        *
* Medical Research, grant U54 GM072970. See https://simtk.org/home/simbody.  *
*                                                                            *
* Portions Copyright (c) 2005-2017 Stanford University and the Authors.      *
* Authors: Chris Dembia                                                      *
* Contributors: Michael Sherman, Dimitar Stanev                              *
*                                                                            *
* Licensed under the Apache License, Version 2.0 (the "License"); you may    *
* not use this file except in compliance with the License. You may obtain a  *
* copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
*                                                                            *
* Unless required by applicable law or agreed to in writing, software        *
* distributed under the License is distributed on an "AS IS" BASIS,          *
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
* See the License for the specific language governing permissions and        *
* limitations under the License.                                             *
* -------------------------------------------------------------------------- */

#include <Simbody.h>
#include "TaskSpace.h"

using namespace SimTK;
using namespace OpenSim;

//==============================================================================
// Jacobian
//==============================================================================
void TaskSpace::Jacobian::updateCache(const State& s, Matrix& cache) const
{
    m_tspace->getMatterSubsystem().calcStationJacobian(
            s,
            m_tspace->getMobilizedBodyIndices(),
            m_tspace->getStations(),
            cache);
}

const TaskSpace::JacobianTranspose& TaskSpace::Jacobian::transpose() const
{
    return m_tspace->getJacobianTranspose();
}

Vector TaskSpace::Jacobian::multiplyByJ(const State& s, const Vector& u) const
{
    Vector_<Vec3> Ju;
    m_tspace->getMatterSubsystem().multiplyByStationJacobian(s,
            m_tspace->getMobilizedBodyIndices(), m_tspace->getStations(),
            u, Ju);

    // Convert to a Vector.
    Vector out(3 * Ju.size());
    for (int i = 0; i < Ju.size(); ++i) {
        out[3 * i] = Ju[i][0];
        out[3 * i + 1] = Ju[i][1];
        out[3 * i + 2] = Ju[i][2];
    }

    return out;
}


//==============================================================================
// JacobianTranspose
//==============================================================================
void TaskSpace::JacobianTranspose::updateCache(const State& s, Matrix& cache) const
{
    cache = transpose().getValue(s).transpose();
}

const TaskSpace::Jacobian& TaskSpace::JacobianTranspose::transpose() const
{
    return m_tspace->getJacobian();
}

Vector TaskSpace::JacobianTranspose::multiplyByJT(const State& s,
    const Vector_<Vec3>& f_GP) const
{
    Vector f;
    m_tspace->getMatterSubsystem().multiplyByStationJacobianTranspose(
            s,
            m_tspace->getMobilizedBodyIndices(),
            m_tspace->getStations(),
            f_GP,
            f);
    return f;
}

Vector TaskSpace::JacobianTranspose::multiplyByJT(const State& s,
    const Vector& f_GP) const
{
    unsigned int nIn = f_GP.size();
    SimTK_APIARGCHECK1_ALWAYS(nIn % 3 == 0,
            "TaskSpace::JacobianTranspose", "operator*",
            "Length of f_GP, which is %i, is not divisible by 3.", nIn);

    unsigned int nOut = nIn / 3;

    // Create the Vector_<Vec3>.
    // TODO debug, or look for methods that already do this.
    Vector_<Vec3> my_f_GP(nOut);
    for (unsigned int i = 0; i < nOut; ++i)
    {
        // getAs is just a recast; doesn't copy.
        my_f_GP[i] = Vec3::getAs(&f_GP[3 * i]);
    }

    // Perform the multiplication.
    return multiplyByJT(s, my_f_GP);
}

Vector TaskSpace::JacobianTranspose::multiplyByJT(const State& s,
    const Vec3& f_GP) const
{
   return multiplyByJT(s, Vector_<Vec3>(1, f_GP));
}

Matrix TaskSpace::JacobianTranspose::multiplyByJT(const State& s,
    const Matrix& f_GP) const
{
    unsigned int nrow = s.getNU();
    unsigned int ncol = f_GP.ncol();

    Matrix out(nrow, ncol);
    for (unsigned int j = 0; j < ncol; ++j)
    {
        // TODO is this cast inefficient? Is it copying?
        out(j) = multiplyByJT(s, Vector(f_GP(j)));
    }

    return out;
}

Matrix TaskSpace::JacobianTranspose::multiplyByJT(
    const State& s, const TaskSpace::Inertia& Lambda) const
{
    // TOOD could be more efficient.
    return multiplyByJT(s, Lambda.getValue(s));
}

Matrix TaskSpace::JacobianTranspose::multiplyByJT(const State& s,
    const TaskSpace::DynamicallyConsistentJacobianInverseTranspose& JBarT) const
{
    return multiplyByJT(s, JBarT.getValue(s));
}

//==============================================================================
// Inertia
//==============================================================================
void TaskSpace::Inertia::updateCache(const State& s, Matrix& cache) const
{
    FactorLU inertiaInverse(m_tspace->getInertiaInverse().getValue(s));
    inertiaInverse.inverse(cache);
}

const TaskSpace::InertiaInverse& TaskSpace::Inertia::inverse() const
{
    return m_tspace->getInertiaInverse();
}

Vector TaskSpace::Inertia::multiplyByLambda(const State& s, const Vector& a) const
{
    return getValue(s) * a;
}

Vector TaskSpace::Inertia::multiplyByLambda(const State& s, const Vec3& a) const
{
    return multiplyByLambda(s, Vector(a));
}

//==============================================================================
// InertiaInverse
//==============================================================================
void TaskSpace::InertiaInverse::updateCache(const State& s, Matrix& cache) const
{
    const SimbodyMatterSubsystem& matter = m_tspace->getMatterSubsystem();

    const JacobianTranspose& JT = m_tspace->JT();
    // TODO const Matrix& JT = m_tspace.JT().value();
    const Matrix& J = m_tspace->J().getValue(s);
    /* TODO
    // TODO cache the result.

    unsigned int nst = m_tspace.getNumScalarTasks();
    unsigned int nu = m_tspace.getState().getNU();

    Matrix J = m_tspace.getJacobian().value();

    Matrix MInvJt(nu, nst);

    for (unsigned int j = 0; j < nst; ++j)
    {
        matter.multiplyByMInv(m_tspace.getState(), J.transpose()(j), MInvJt(j));
    }

    updCacheValue() = J * MInvJt;
    */

    unsigned int nt = m_tspace->getNumTasks();
    unsigned int nst = m_tspace->getNumScalarTasks();
    unsigned int nu = s.getNU();

    Matrix& inertiaInverse = cache;
    inertiaInverse.resize(nst, nst);

    // Create temporary variables.
    Vector Jtcol(nu);
    Vector MInvJtcol(nu);
    Vector_<Vec3> JMInvJt_j(nt);

    // f_GP is used to pluck out one column at a time of Jt. Exactly one
    // element at a time of f_GP will be 1, the rest are 0.
    Vector f_GP(nst, Real(0));

    for (unsigned int j = 0; j < nst; ++j)
    {
        f_GP[j] = 1;
        Jtcol = JT.multiplyByJT(s, f_GP);
        f_GP[j] = 0;

        matter.multiplyByMInv(s, Jtcol, MInvJtcol);

        // TODO replace with operator.
        inertiaInverse(j) = J * MInvJtcol;
        /* TODO
        matter.multiplyByStationJacobian(m_tspace.getState(),
                m_tspace.getMobilizedBodyIndices(), m_tspace.getStations(),
                MInvJtcol, JMInvJt_j);

        inertiaInverse(j) = JMInvJt_j;
        */
    }
}

const TaskSpace::Inertia& TaskSpace::InertiaInverse::inverse() const
{
    return m_tspace->getInertia();
}


//==============================================================================
// DynamicallyConsistentJacobianInverse
//==============================================================================
void TaskSpace::DynamicallyConsistentJacobianInverse::updateCache(const State& s,
    Matrix& cache)
    const
{
    const JacobianTranspose& JT = m_tspace->getJacobianTranspose();
    const Inertia& Lambda = m_tspace->getInertia();

    // TODO inefficient?
    Matrix JtLambda = JT.multiplyByJT(s, Lambda);

    unsigned int nst = m_tspace->getNumScalarTasks();
    unsigned int nu = s.getNU();

    Matrix& Jbar = cache;
    Jbar.resize(nu, nst);

    for (unsigned int j = 0; j < nst; ++j)
    {
        m_tspace->getMatterSubsystem().multiplyByMInv(s,
                JtLambda(j), Jbar(j));
    }
}

const TaskSpace::DynamicallyConsistentJacobianInverseTranspose&
TaskSpace::DynamicallyConsistentJacobianInverse::transpose() const
{
    return m_tspace->getDynamicallyConsistentJacobianInverseTranspose();
}

Vector TaskSpace::DynamicallyConsistentJacobianInverse::multiplyByJBar(
    const State& s, const Vector& vec) const
{
    const JacobianTranspose& JT = m_tspace->getJacobianTranspose();
    const Inertia& Lambda = m_tspace->getInertia();

    // TODO where is this even used? TODO test this.

    Vector JBarvec;
    m_tspace->getMatterSubsystem().multiplyByMInv(s,
            JT.multiplyByJT(s, Lambda.multiplyByLambda(s, vec)), JBarvec);
    return  JBarvec;
}

Matrix TaskSpace::DynamicallyConsistentJacobianInverse::multiplyByJBar(
    const State& s, const Matrix& mat) const
{
    unsigned int nrow = s.getNU();
    unsigned int ncol = mat.ncol();

    Matrix out(nrow, ncol);
    for (unsigned int j = 0; j < ncol; ++j)
    {
        out(j) = multiplyByJBar(s, mat(j).getAsVector());
    }

    return out;
}


//==============================================================================
// DynamicallyConsistentJacobianInverseTranspose
//==============================================================================
void TaskSpace::DynamicallyConsistentJacobianInverseTranspose::updateCache(
    const State& s, Matrix& cache) const
{
    cache = transpose().getValue(s).transpose();
}

const TaskSpace::DynamicallyConsistentJacobianInverse&
TaskSpace::DynamicallyConsistentJacobianInverseTranspose::transpose() const
{
    return m_tspace->getDynamicallyConsistentJacobianInverse();
}

Vector TaskSpace::DynamicallyConsistentJacobianInverseTranspose::multiplyByJBarT(
    const State& s, const Vector& g) const
{
    // TODO inefficient. can we have an MInvT operator??
    return getValue(s) * g;
}


//==============================================================================
// InertialForces
//==============================================================================
void TaskSpace::InertialForces::updateCache(const State& s, Vector& cache) const
{
    Vector jointSpaceInertialForces;
    m_tspace->getMatterSubsystem().calcResidualForceIgnoringConstraints(
            s, Vector(0), Vector_<SpatialVec>(0), Vector(0),
            jointSpaceInertialForces);

    Vector JDotu;
    m_tspace->getMatterSubsystem().calcBiasForStationJacobian(
            s,
            m_tspace->getMobilizedBodyIndices(), m_tspace->getStations(),
            JDotu);

    const DynamicallyConsistentJacobianInverseTranspose& JBarT =
        m_tspace->JBarT();
    const Vector& b = jointSpaceInertialForces;
    const Inertia& Lambda = m_tspace->Lambda();

    cache = JBarT.multiplyByJBarT(s, b) - Lambda.multiplyByLambda(s, JDotu);
}


//==============================================================================
// Gravity
//==============================================================================
void TaskSpace::Gravity::updateCache(const State& s, Vector& cache) const
{
    cache = m_tspace->JBarT().multiplyByJBarT(s, systemGravity(s));
}


Vector TaskSpace::Gravity::systemGravity(const State& s) const
{
    // TODO where does this go? Make a separate class?
    Vector g;
    m_tspace->getMatterSubsystem().multiplyBySystemJacobianTranspose(
        s,
        m_tspace->getGravityForce().getBodyForces(s),
        g);
    // Negate, since we want the 'g' that appears on the same side of the
    // equations of motion as does the mass matrix. That is, M udot + C + g = F
    return -g;
}


//==============================================================================
// NullspaceProjection
//==============================================================================
void TaskSpace::NullspaceProjection::updateCache(const State& s, Matrix& cache)
const
{
    cache = transpose().getValue(s).transpose();
}

const TaskSpace::NullspaceProjectionTranspose&
TaskSpace::NullspaceProjection::transpose() const
{
    return m_tspace->getNullspaceProjectionTranspose();
}

Vector TaskSpace::NullspaceProjection::multiplyByN(const State& s,
    const Vector& vec) const
{
    return vec - m_tspace->JBar().multiplyByJBar(s,
        m_tspace->J().multiplyByJ(s, vec));
}


//==============================================================================
// NullspaceProjectionTranspose
//==============================================================================
void TaskSpace::NullspaceProjectionTranspose::updateCache(const State& s,
    Matrix& cache) const
{
    cache = 1 - m_tspace->JT().multiplyByJT(s, m_tspace->JBarT());
}

const TaskSpace::NullspaceProjection&
TaskSpace::NullspaceProjectionTranspose::transpose() const
{
    return m_tspace->getNullspaceProjection();
}

Vector TaskSpace::NullspaceProjectionTranspose::multiplyByNT(const State& s,
    const Vector& vec) const
{
    return vec - m_tspace->JT().multiplyByJT(s,
        m_tspace->JBarT().multiplyByJBarT(s, vec));
}


//==============================================================================
// TaskSpace
//==============================================================================
// TODO account for applied forces? velocities?
Vector TaskSpace::calcInverseDynamics(const State& s,
    const Vector& taskAccelerations) const
{
    Vector ftasl = JT().multiplyByJT(s,
        Lambda().multiplyByLambda(s, taskAccelerations) +
        mu().getValue(s) +
        p().getValue(s));

    return ftasl;
}

Vector TaskSpace::calcGravityCompensation(const State& s) const
{
    return NT().multiplyByNT(s, g(s));
}
