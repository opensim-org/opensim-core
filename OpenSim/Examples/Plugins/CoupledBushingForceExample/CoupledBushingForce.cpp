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
CoupledBushingForce::CoupledBushingForce() : Force(),
	_body1Name(_body1NameProp.getValueStr()),
	_body2Name(_body2NameProp.getValueStr()),
	_locationInBody1(_locationInBody1Prop.getValueDblVec()),
	_orientationInBody1(_orientationInBody1Prop.getValueDblVec()),
	_locationInBody2(_locationInBody2Prop.getValueDblVec()),
	_orientationInBody2(_orientationInBody2Prop.getValueDblVec())
{
	constructMatricesFromProperties();
	setNull();
}

/* Convenience constructor */
CoupledBushingForce::CoupledBushingForce( std::string body1Name, SimTK::Vec3 point1, SimTK::Vec3 orientation1,
		          std::string body2Name, SimTK::Vec3 point2, SimTK::Vec3 orientation2,
				  SimTK::Mat66 stiffnessMat, SimTK::Mat66 dampingMat):	Force(),
	_body1Name(_body1NameProp.getValueStr()),
	_body2Name(_body2NameProp.getValueStr()),
	_locationInBody1(_locationInBody1Prop.getValueDblVec()),
	_orientationInBody1(_orientationInBody1Prop.getValueDblVec()),
	_locationInBody2(_locationInBody2Prop.getValueDblVec()),
	_orientationInBody2(_orientationInBody2Prop.getValueDblVec())
{
	setNull();
	constructMatricesFromProperties();
	_body1Name = body1Name;
	_body2Name = body2Name;
	_locationInBody1 = point1;
	_orientationInBody1 = orientation1;
	_locationInBody2 = point2;
	_orientationInBody2 = orientation2;
	_stiffnessMatrix = stiffnessMat;
	_dampingMatrix = dampingMat;
	updatePropertiesFromMatrices();
}


//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aForce CoupledBushingForce to be copied.
 */
CoupledBushingForce::CoupledBushingForce(const CoupledBushingForce &aForce) :
   Force(aForce),
	_body1Name(_body1NameProp.getValueStr()),
	_body2Name(_body2NameProp.getValueStr()),
	_locationInBody1(_locationInBody1Prop.getValueDblVec()),
	_orientationInBody1(_orientationInBody1Prop.getValueDblVec()),
	_locationInBody2(_locationInBody2Prop.getValueDblVec()),
	_orientationInBody2(_orientationInBody2Prop.getValueDblVec())
{
	setNull();
	constructMatricesFromProperties();
	copyData(aForce);
}

//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Copy data members from one CoupledBushingForce to another.
 *
 * @param aForce CoupledBushingForce to be copied.
 */
void CoupledBushingForce::copyData(const CoupledBushingForce &aForce)
{
	_body1Name = aForce._body1Name;
	_body2Name = aForce._body2Name;
	_locationInBody1 = aForce._locationInBody1;
	_orientationInBody1 = aForce._orientationInBody1;
	_locationInBody2 = aForce._locationInBody2;
	_orientationInBody2 = aForce._orientationInBody2;
	_stiffnessMatrix = aForce._stiffnessMatrix;
	_dampingMatrix = aForce._dampingMatrix;
	updatePropertiesFromMatrices();
}

//_____________________________________________________________________________
/**
 * Set the data members of this CoupledBushingForce to their null values.
 */
void CoupledBushingForce::setNull()
{
	setAuthors("Ajay Seth");
	_b1 = NULL;
	_b2 = NULL;
	setupProperties();
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void CoupledBushingForce::setupProperties()
{
	// Body 1 name
	_body1NameProp.setName("body_1");
	_propertySet.append(&_body1NameProp);

	// Body 2 name
	_body2NameProp.setName("body_2");
	_propertySet.append(&_body2NameProp);

	//Default location and orientation (rotation sequence)
	SimTK::Vec3 origin(0.0, 0.0, 0.0);

	// Location in Body 1 
	_locationInBody1Prop.setName("location_body_1");
	_locationInBody1Prop.setValue(origin);
	_propertySet.append(&_locationInBody1Prop);

	// Orientation in Body 1 
	_orientationInBody1Prop.setName("orientation_body_1");
	_orientationInBody1Prop.setValue(origin);
	_propertySet.append(&_orientationInBody1Prop);

	// Location in Body 2 
	_locationInBody2Prop.setName("location_body_2");
	_locationInBody2Prop.setValue(origin);
	_propertySet.append(&_locationInBody2Prop);

	// Orientation in Body 2 
	_orientationInBody2Prop.setName("orientation_body_2");
	_orientationInBody2Prop.setValue(origin);
	_propertySet.append(&_orientationInBody2Prop);

	_stiffnessMatrixRow1Prop.setName("stiffness_row1");
	_propertySet.append(&_stiffnessMatrixRow1Prop );

	_stiffnessMatrixRow2Prop.setName("stiffness_row2");
	_propertySet.append(&_stiffnessMatrixRow2Prop );

	_stiffnessMatrixRow3Prop.setName("stiffness_row3");
	_propertySet.append(&_stiffnessMatrixRow3Prop );

	_stiffnessMatrixRow4Prop.setName("stiffness_row4");
	_propertySet.append(&_stiffnessMatrixRow4Prop );

	_stiffnessMatrixRow5Prop.setName("stiffness_row5");
	_propertySet.append(&_stiffnessMatrixRow5Prop );

	_stiffnessMatrixRow6Prop.setName("stiffness_row6");
	_propertySet.append(&_stiffnessMatrixRow6Prop );

	_dampingMatrixRow1Prop.setName("damping_row1");
	_propertySet.append(&_dampingMatrixRow1Prop );

	_dampingMatrixRow2Prop.setName("damping_row2");
	_propertySet.append(&_dampingMatrixRow2Prop );

	_dampingMatrixRow3Prop.setName("damping_row3");
	_propertySet.append(&_dampingMatrixRow3Prop );

	_dampingMatrixRow4Prop.setName("damping_row4");
	_propertySet.append(&_dampingMatrixRow4Prop );

	_dampingMatrixRow5Prop.setName("damping_row5");
	_propertySet.append(&_dampingMatrixRow5Prop );

	_dampingMatrixRow6Prop.setName("damping_row6");
	_propertySet.append(&_dampingMatrixRow6Prop );
}

//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aModel OpenSim model containing this CoupledBushingForce.
 */
void CoupledBushingForce::connectToModel(Model& aModel)
{
    Super::connectToModel(aModel);

	string errorMessage;
	// Look up the two bodies being connected by bushing by name in the
	// model and might as well keep a pointer to them
	if (!aModel.updBodySet().contains(_body1Name)) {
		errorMessage = "Invalid bushing body1 (" + _body1Name + ") specified in Force " + getName();
		throw (Exception(errorMessage.c_str()));
	}
	if (!aModel.updBodySet().contains(_body2Name)) {
		errorMessage = "Invalid bushing body2 (" + _body2Name + ") specified in Force " + getName();
		throw (Exception(errorMessage.c_str()));
	}
}

void CoupledBushingForce::extendAddToSystem(SimTK::MultibodySystem& system) const
{
    Super::extendAddToSystem(system);

	Body &body1 = _model->updBodySet().get(_body1Name);
	Body &body2 = _model->updBodySet().get(_body2Name);

	// Beyond the const Component get access to underlying SimTK elements
	CoupledBushingForce* mutableThis = const_cast<CoupledBushingForce *>(this);

	// Get underlying mobilized bodies
	mutableThis->_b1 = &body1.getMobilizedBody();
	mutableThis->_b2 = &body2.getMobilizedBody();
	// Define the transforms for the bushing frames affixed to the specified bodies
	SimTK::Rotation r1; r1.setRotationToBodyFixedXYZ(_orientationInBody1);
	SimTK::Rotation r2; r2.setRotationToBodyFixedXYZ(_orientationInBody2);
	// Hang on to the transforms for the bushing frames
	mutableThis->_inb1 = SimTK::Transform(r1, _locationInBody1);
	mutableThis->_inb2 = SimTK::Transform(r2, _locationInBody2);
}

//=============================================================================
// OPERATORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Assignment operator.
 *
 * @return Reference to this object.
 */
CoupledBushingForce& CoupledBushingForce::operator=(const CoupledBushingForce &aForce)
{
	Force::operator=(aForce);
	copyData(aForce);
	return(*this);
}

//=============================================================================
// SET
//=============================================================================
//_____________________________________________________________________________
/**
 * Following methods set attributes of the weld Force */
void CoupledBushingForce::setBody1ByName(std::string aBodyName)
{
	_body1Name = aBodyName;
}

void CoupledBushingForce::setBody2ByName(std::string aBodyName)
{
	_body2Name = aBodyName;
}

/** Set the location and orientation (optional) for weld on body 1*/
void CoupledBushingForce::setBody1BushingLocation(Vec3 location, Vec3 orientation)
{
	_locationInBody1 = location;
	_orientationInBody1 = orientation;
}

/** Set the location and orientation (optional) for weld on body 2*/
void CoupledBushingForce::setBody2BushingLocation(Vec3 location, Vec3 orientation)
{
	_locationInBody2 = location;
	_orientationInBody2 = orientation;
}

//=============================================================================
// COMPUTATION
//=============================================================================
/** Compute the deflection (spatial separation) of the two frames connected
	by the bushing force. Angualar displacement expressed in Euler angles.
	The force and potential energy are determined by the deflection.  */
SimTK::Vec6 CoupledBushingForce::computeDeflection(const SimTK::State& s) const
{
	const Transform& X_GB1 = _b1->getBodyTransform(s);

    const Transform& X_GB2 = _b2->getBodyTransform(s);

	// Define the frame on body 1 is the "fixed" frame, F
	Transform X_GF = X_GB1 * _inb1;
	// Define the frame on body 2 as the "moving" frame, M
    Transform X_GM = X_GB2 * _inb2;
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
    const Transform& X_GB1 = _b1->getBodyTransform(s);
    const Transform& X_GB2 = _b2->getBodyTransform(s);

	Transform X_GF = X_GB1 * _inb1;   
    Transform X_GM = X_GB2 * _inb2;   
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
    const SpatialVec& V_GB1 = _b1->getBodyVelocity(s);
    const SpatialVec& V_GB2 = _b2->getBodyVelocity(s);

	// Re-express local vectors in the Ground frame.
    Vec3 p_B1F_G =  X_GB1.R() * _inb1.p();   // 15 flops
    Vec3 p_B2M_G =  X_GB2.R() * _inb2.p();   // 15 flops
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
	bodyForces[_b2->getMobilizedBodyIndex()] += F_GB2;
    bodyForces[_b1->getMobilizedBodyIndex()] += F_GB1;
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
/** 
 * Provide names of the quantities (column labels) of the force value(s) reported
 * 
 */
OpenSim::Array<std::string> CoupledBushingForce::getRecordLabels() const 
{
	OpenSim::Array<std::string> labels("");
	labels.append(getName()+"."+_body1Name+".force.X");
	labels.append(getName()+"."+_body1Name+".force.Y");
	labels.append(getName()+"."+_body1Name+".force.Z");
	labels.append(getName()+"."+_body1Name+".torque.X");
	labels.append(getName()+"."+_body1Name+".torque.Y");
	labels.append(getName()+"."+_body1Name+".torque.Z");
	labels.append(getName()+"."+_body2Name+".force.X");
	labels.append(getName()+"."+_body2Name+".force.Y");
	labels.append(getName()+"."+_body2Name+".force.Z");
	labels.append(getName()+"."+_body2Name+".torque.X");
	labels.append(getName()+"."+_body2Name+".torque.Y");
	labels.append(getName()+"."+_body2Name+".torque.Z");

	return labels;
}
/**
 * Provide the value(s) to be reported that correspond to the labels
 */
OpenSim::Array<double> CoupledBushingForce::getRecordValues(const SimTK::State& state) const 
{
	OpenSim::Array<double> values(1);

	const SimTK::Force::LinearBushing &simtkSpring = (SimTK::Force::LinearBushing &)(_model->getForceSubsystem().getForce(_index));

	SimTK::Vector_<SimTK::SpatialVec> bodyForces(0);
	SimTK::Vector_<SimTK::Vec3> particleForces(0);
	SimTK::Vector mobilityForces(0);

	//get the net force added to the system contributed by the bushing
	simtkSpring.calcForceContribution(state, bodyForces, particleForces, mobilityForces);
	SimTK::Vec3 forces = bodyForces(_model->getBodySet().get(_body1Name).getMobilizedBodyIndex())[1];
	SimTK::Vec3 torques = bodyForces(_model->getBodySet().get(_body1Name).getMobilizedBodyIndex())[0];
	values.append(3, &forces[0]);
	values.append(3, &torques[0]);

	forces = bodyForces(_model->getBodySet().get(_body2Name).getMobilizedBodyIndex())[1];
	torques = bodyForces(_model->getBodySet().get(_body2Name).getMobilizedBodyIndex())[0];

	values.append(3, &forces[0]);
	values.append(3, &torques[0]);

	return values;
}

/* UTILITIES */
void CoupledBushingForce::constructMatricesFromProperties()
{
	_stiffnessMatrix = Mat66(~_stiffnessMatrixRow1Prop.getValueDblVec(),
							 ~_stiffnessMatrixRow2Prop.getValueDblVec(),
							 ~_stiffnessMatrixRow3Prop.getValueDblVec(),
							 ~_stiffnessMatrixRow4Prop.getValueDblVec(),
							 ~_stiffnessMatrixRow5Prop.getValueDblVec(),
							 ~_stiffnessMatrixRow6Prop.getValueDblVec());

	_dampingMatrix = Mat66(~_dampingMatrixRow1Prop.getValueDblVec(),
						   ~_dampingMatrixRow2Prop.getValueDblVec(),
						   ~_dampingMatrixRow3Prop.getValueDblVec(),
						   ~_dampingMatrixRow4Prop.getValueDblVec(),
						   ~_dampingMatrixRow5Prop.getValueDblVec(),
						   ~_dampingMatrixRow6Prop.getValueDblVec());
}

void CoupledBushingForce::updatePropertiesFromMatrices()
{
	_stiffnessMatrixRow1Prop.setValue(_stiffnessMatrix(0));
	_stiffnessMatrixRow2Prop.setValue(_stiffnessMatrix(1));
	_stiffnessMatrixRow3Prop.setValue(_stiffnessMatrix(2));
	_stiffnessMatrixRow4Prop.setValue(_stiffnessMatrix(3));
	_stiffnessMatrixRow5Prop.setValue(_stiffnessMatrix(4));
	_stiffnessMatrixRow6Prop.setValue(_stiffnessMatrix(5));

	_dampingMatrixRow1Prop.setValue(_dampingMatrix(0));
	_dampingMatrixRow2Prop.setValue(_dampingMatrix(1));
	_dampingMatrixRow3Prop.setValue(_dampingMatrix(2));
	_dampingMatrixRow4Prop.setValue(_dampingMatrix(3));
	_dampingMatrixRow5Prop.setValue(_dampingMatrix(4));
	_dampingMatrixRow6Prop.setValue(_dampingMatrix(5));
}
