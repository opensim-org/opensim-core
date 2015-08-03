/* -------------------------------------------------------------------------- *
 *                     OpenSim:  CoupledBushingForce.cpp                      *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Ajay Seth                                                       *
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

//=============================================================================
// INCLUDES
//=============================================================================
#include <OpenSim/Simulation/Model/BodySet.h>
#include <OpenSim/Simulation/Model/Model.h>

#include "CoupledBushingForce.h"

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace SimTK;
using namespace OpenSim;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
CoupledBushingForce::~CoupledBushingForce()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
CoupledBushingForce::CoupledBushingForce() : Force()
{
    setNull();
}

/* Convenience constructor */
CoupledBushingForce::CoupledBushingForce( const std::string& frame1Name,
                                          const std::string& frame2Name,
                                          SimTK::Mat66 stiffnessMat,
                                          SimTK::Mat66 dampingMat) : Force()
{
    setNull();
    constructInfrastructure();

    updConnector<PhysicalFrame>("frame1").set_connected_to_name(frame1Name);
    updConnector<PhysicalFrame>("frame2").set_connected_to_name(frame2Name);

    _stiffnessMatrix = stiffnessMat;
    _dampingMatrix = dampingMat;
    updatePropertiesFromMatrices();
}


//_____________________________________________________________________________
/**
 * Set the data members of this CoupledBushingForce to their null values.
 */
void CoupledBushingForce::setNull()
{
    setAuthors("Ajay Seth");
    constructProperties();
}

//_____________________________________________________________________________
/*
 * Create properties 
 */
void CoupledBushingForce::constructProperties()
{
    // default bushing material properties
    // 6x6 stiffness matrix
    constructProperty_stiffness_row1(Vec6(0));
    constructProperty_stiffness_row2(Vec6(0));
    constructProperty_stiffness_row3(Vec6(0));
    constructProperty_stiffness_row4(Vec6(0));
    constructProperty_stiffness_row5(Vec6(0));
    constructProperty_stiffness_row6(Vec6(0));
    // 6x6 damping matrix
    constructProperty_damping_row1(Vec6(0));
    constructProperty_damping_row2(Vec6(0));
    constructProperty_damping_row3(Vec6(0));
    constructProperty_damping_row4(Vec6(0));
    constructProperty_damping_row5(Vec6(0));
    constructProperty_damping_row6(Vec6(0));

    //Default frames list is empty
    constructProperty_frames();
}

//_____________________________________________________________________________
/*
* Construct Structural Connectors
*/
void CoupledBushingForce::constructConnectors() {
    constructConnector<PhysicalFrame>("frame1");
    constructConnector<PhysicalFrame>("frame2");
}

void CoupledBushingForce::extendFinalizeFromProperties()
{
    Super::extendFinalizeFromProperties();

    //mark frames in property list as subcomponents
    for (int i = 0; i < updProperty_frames().size(); ++i){
        addComponent(&upd_frames(i));
    }

    finalizeMatricesFromProperties();
}


//=============================================================================
// COMPUTATION
//=============================================================================
/** Compute the deflection (spatial separation) of the two frames connected
    by the bushing force. Angular displacement expressed in Euler angles.
    The force and potential energy are determined by the deflection.  */
SimTK::Vec6 CoupledBushingForce::computeDeflection(const SimTK::State& s) const
{
    // get connected frames
    const PhysicalFrame& frame1 = getConnectee<PhysicalFrame>("frame1");
    const PhysicalFrame& frame2 = getConnectee<PhysicalFrame>("frame2");

    // Define the frame on body 1 is the "fixed" frame, F
    Transform X_GF = frame1.getGroundTransform(s);
    // Define the frame on body 2 as the "moving" frame, M
    Transform X_GM = frame2.getGroundTransform(s);
    // Express M in F
    Transform X_FM = ~X_GF * X_GM;    

    // Calculate stiffness generalized forces of bushing by first computing
    // the deviation of the two frames measured by dq
    Vec6 dq(0);
    // First 3 are rotational deviations
    dq.updSubVec<3>(0) = X_FM.R().convertRotationToBodyFixedXYZ();
    // Second 3 are translational
    dq.updSubVec<3>(3) = X_FM.p();

    return dq;
}

/**
 * Compute the force contribution to the system and add in to appropriate
 * bodyForce and/or system generalizedForce.
 * CoupledBushingForce implementation based SimTK::Force::LinearBushing
 * developed and implemented by Michael Sherman.
 */
void CoupledBushingForce::computeForce(const SimTK::State& s, 
                              SimTK::Vector_<SimTK::SpatialVec>& bodyForces, 
                              SimTK::Vector& generalizedForces) const
{
    // get connected frames
    const PhysicalFrame& frame1 = getConnectee<PhysicalFrame>("frame1");
    const PhysicalFrame& frame2 = getConnectee<PhysicalFrame>("frame2");

    const Transform& X_GB1 = frame1.getMobilizedBody().getBodyTransform(s);
    const Transform& X_GB2 = frame2.getMobilizedBody().getBodyTransform(s);

    Transform X_GF = frame1.getGroundTransform(s);
    Transform X_GM = frame2.getGroundTransform(s);
    Transform X_FM = ~X_GF * X_GM;
    const Rotation& R_GF = X_GF.R();
    const Rotation& R_GM = X_GM.R();
    const Rotation& R_FM = X_FM.R();

    // Calculate stiffness generalized forces of bushing by first computing
    // the deviation of the two frames measured by dq
    Vec6 dq = computeDeflection(s);

    // Evaluate the force in the bushing frame affixed to body1 (F) 
    Vec6 fk = _stiffnessMatrix*dq;

    // Now evaluate velocities.
    const SpatialVec& V_GB1 = frame1.getMobilizedBody().getBodyVelocity(s);
    const SpatialVec& V_GB2 = frame2.getMobilizedBody().getBodyVelocity(s);

    // Re-express local vectors in the Ground frame.
    Vec3 p_B1F_G = X_GB1.R() * frame1.findTransformInBaseFrame().p();
    Vec3 p_B2M_G = X_GB2.R() * frame2.findTransformInBaseFrame().p();
    Vec3 p_FM_G  =  X_GF.R()  * X_FM.p();    // 15 flops

    SpatialVec V_GF(V_GB1[0], V_GB1[1] + V_GB1[0] % p_B1F_G);
    SpatialVec V_GM(V_GB2[0], V_GB2[1] + V_GB2[0] % p_B2M_G);

    // This is the velocity of M in F, but with the time derivative
    // taken in the Ground frame.
    const SpatialVec V_FM_G = V_GM - V_GF;

    // To get derivative in F, we must remove the part due to the
    // angular velocity w_GF of F in G.
    SpatialVec V_FM = ~R_GF * SpatialVec(V_FM_G[0], 
                                 V_FM_G[1] - V_GF[0] % p_FM_G);

    // Need angular velocity in M frame for conversion to qdot.
    const Vec3  w_FM_M = ~R_FM * V_FM[0];
    const Mat33 N_FM   = Rotation::calcNForBodyXYZInBodyFrame(dq.getSubVec<3>(0));
    Vec6 dqdot(0);
    dqdot.updSubVec<3>(0) = N_FM * w_FM_M;
    dqdot.updSubVec<3>(3) = V_FM[1];

    // velocity dependent force according to the speed of frame2 on 
    // body2 relative to frame1 
    Vec6 fv = _dampingMatrix * dqdot;

    Vec6 f = -(fk+fv); // generalized forces on body 2
    const Vec3& fB2_q = f.getSubVec<3>(0); // in q basis
    const Vec3& fM_F  = f.getSubVec<3>(3); // acts at M, but exp. in F frame

    // Calculate the matrix relating q-space generalized forces to a real-space
    // moment vector. We know qforce = ~H * moment (where H is the
    // the hinge matrix for a mobilizer using qdots as generalized speeds).
    // In that case H would be N^-1, qforce = ~(N^-1)*moment so
    // moment = ~N*qforce. Caution: our N wants the moment in the outboard
    // body frame, in this case M.
    const Vec3  mB2_M = ~N_FM * fB2_q; // moment acting on body 2, exp. in M
    const Vec3  mB2_G =  R_GM * mB2_M; // moment on body 2, now exp. in G

    // Transform force from F frame to ground. This is the force to 
    // apply to body 2 at point OM; -f goes on body 1 at the same
    // spatial location. Here we actually apply it at OF so we have to
    // account for the moment produced by the shift from OM.
    const Vec3 fM_G = R_GF*fM_F;

    SpatialVec F_GM(  mB2_G,                       fM_G);
    SpatialVec F_GF(-(mB2_G + p_FM_G % fM_G) , -fM_G);

    // Shift forces to body origins.
    SpatialVec F_GB2(F_GM[0] + p_B2M_G % F_GM[1], F_GM[1]);
    SpatialVec F_GB1(F_GF[0] + p_B1F_G % F_GF[1], F_GF[1]);

    // Apply (add-in) the body forces to the system set of body forces
    bodyForces[frame2.getMobilizedBodyIndex()] += F_GB2;
    bodyForces[frame1.getMobilizedBodyIndex()] += F_GB1;
}

/** Potential energy stored in the bushing is purely a function of the deflection
    from the rest position and the stiffness */
double CoupledBushingForce::computePotentialEnergy(const SimTK::State& s) const
{
    Vec6 dq = computeDeflection(s);
    // Energy stored in the bushing
    double U = 0.5*(~dq*_stiffnessMatrix*dq);
    return U;
}


//=============================================================================
// Reporting
//=============================================================================
/*
 * Provide names of the quantities (column labels) of the force value(s) to be
 * reported
 */
OpenSim::Array<std::string> CoupledBushingForce::getRecordLabels() const 
{
    OpenSim::Array<std::string> labels("");

    // get connected frames
    const PhysicalFrame& frame1 = getConnectee<PhysicalFrame>("frame1");
    const PhysicalFrame& frame2 = getConnectee<PhysicalFrame>("frame2");

    // Forces applied to underlying MobilizedBody which is a base PhysicalFrame
    std::string base1Name = frame1.findBaseFrame().getName();
    std::string base2Name = frame2.findBaseFrame().getName();

    labels.append(getName() + "." + base1Name + ".force.X");
    labels.append(getName() + "." + base1Name + ".force.Y");
    labels.append(getName() + "." + base1Name + ".force.Z");
    labels.append(getName() + "." + base1Name + ".torque.X");
    labels.append(getName() + "." + base1Name + ".torque.Y");
    labels.append(getName() + "." + base1Name + ".torque.Z");
    labels.append(getName() + "." + base2Name + ".force.X");
    labels.append(getName() + "." + base2Name + ".force.Y");
    labels.append(getName() + "." + base2Name + ".force.Z");
    labels.append(getName() + "." + base2Name + ".torque.X");
    labels.append(getName() + "." + base2Name + ".torque.Y");
    labels.append(getName() + "." + base2Name + ".torque.Z");

    return labels;
}
/**
 * Provide the value(s) to be reported that correspond to the labels
 */
OpenSim::Array<double> CoupledBushingForce::
getRecordValues(const SimTK::State& state) const 
{
    OpenSim::Array<double> values(1);

        // get connected frames
    const PhysicalFrame& frame1 = getConnectee<PhysicalFrame>("frame1");
    const PhysicalFrame& frame2 = getConnectee<PhysicalFrame>("frame2");

    const SimTK::Force::LinearBushing &simtkSpring =
        (SimTK::Force::LinearBushing &)(_model->getForceSubsystem().getForce(_index));

    SimTK::Vector_<SimTK::SpatialVec> bodyForces(0);
    SimTK::Vector_<SimTK::Vec3> particleForces(0);
    SimTK::Vector mobilityForces(0);

    //get the net force added to the system contributed by the bushing
    simtkSpring.calcForceContribution(state, bodyForces, particleForces, mobilityForces);
    SimTK::Vec3 forces = bodyForces[frame1.getMobilizedBodyIndex()][1];
    SimTK::Vec3 torques = bodyForces[frame1.getMobilizedBodyIndex()][0];
    values.append(3, &forces[0]);
    values.append(3, &torques[0]);

    forces = bodyForces[frame2.getMobilizedBodyIndex()][1];
    torques = bodyForces[frame2.getMobilizedBodyIndex()][0];

    values.append(3, &forces[0]);
    values.append(3, &torques[0]);

    return values;
}

/* UTILITIES */
void CoupledBushingForce::finalizeMatricesFromProperties()
{
    _stiffnessMatrix = Mat66( ~get_stiffness_row1(),
                              ~get_stiffness_row2(),
                              ~get_stiffness_row3(),
                              ~get_stiffness_row4(),
                              ~get_stiffness_row5(),
                              ~get_stiffness_row6() );

    _dampingMatrix = Mat66( ~get_damping_row1(),
                            ~get_damping_row2(),
                            ~get_damping_row3(),
                            ~get_damping_row4(),
                            ~get_damping_row5(),
                            ~get_damping_row6() );
}

void CoupledBushingForce::updatePropertiesFromMatrices()
{
    upd_stiffness_row1() = _stiffnessMatrix(0);
    upd_stiffness_row2() = _stiffnessMatrix(1);
    upd_stiffness_row3() = _stiffnessMatrix(2);
    upd_stiffness_row4() = _stiffnessMatrix(3);
    upd_stiffness_row5() = _stiffnessMatrix(4);
    upd_stiffness_row6() = _stiffnessMatrix(5);

    upd_damping_row1() = _dampingMatrix(0);
    upd_damping_row2() = _dampingMatrix(1);
    upd_damping_row3() = _dampingMatrix(2);
    upd_damping_row4() = _dampingMatrix(3);
    upd_damping_row5() = _dampingMatrix(4);
    upd_damping_row6() = _dampingMatrix(5);
}
