/* -------------------------------------------------------------------------- *
 *                  OpenSim:  ExpressionBasedBushingForce.cpp                 *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Matt S. DeMers                                                  *
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

#include "ExpressionBasedBushingForce.h"

using namespace std;
using namespace SimTK;
using namespace OpenSim;


// string formatting helper utility

template <typename T>
std::string to_string(T const& value) {
    stringstream sstr;
    sstr << value;
    return sstr.str();
}

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
// Uses default (compiler-generated) destructor, copy constructor, and copy
// assignment operator.

//_____________________________________________________________________________
// Default constructor.
ExpressionBasedBushingForce::ExpressionBasedBushingForce()
{
	setNull();
    constructProperties();
}

// Convenience constructor for zero value force functions.
ExpressionBasedBushingForce::ExpressionBasedBushingForce(const string&    body1Name, 
                           const Vec3&      point1, 
                           const Vec3&      orientation1,
		                   const string&    body2Name, 
                           const Vec3&      point2, 
                           const Vec3&      orientation2)
{
	setNull();
    constructProperties();

	set_body_1(body1Name);
	set_body_2(body2Name);
	set_location_body_1(point1);
	set_orientation_body_1(orientation1);
	set_location_body_2(point2);
	set_orientation_body_2(orientation2);
    
}

// Convenience constructor for linear functions.
ExpressionBasedBushingForce::ExpressionBasedBushingForce(const string&    body1Name, 
                           const Vec3&      point1, 
                           const Vec3&      orientation1,
		                   const string&    body2Name, 
                           const Vec3&      point2, 
                           const Vec3&      orientation2,
				           const Vec3&      transStiffness, 
                           const Vec3&      rotStiffness, 
                           const Vec3&      transDamping, 
                           const Vec3&      rotDamping)
{
	setNull();
    constructProperties();

	set_body_1(body1Name);
	set_body_2(body2Name);
	set_location_body_1(point1);
	set_orientation_body_1(orientation1);
	set_location_body_2(point2);
	set_orientation_body_2(orientation2);
    // populate moments and forces as linear (ramp) expressions based on stiffness
	
    setMxExpression( to_string(rotStiffness[0]) + string("*theta_x") );
    setMyExpression( to_string(rotStiffness[1]) + string("*theta_y") );
    setMzExpression( to_string(rotStiffness[2]) + string("*theta_z") );
	setFxExpression( to_string(transStiffness[0]) + string("*delta_x") );
	setFyExpression( to_string(transStiffness[1]) + string("*delta_y") );
	setFzExpression( to_string(transStiffness[2]) + string("*delta_z") );
	set_rotational_damping(rotDamping);
	set_translational_damping(transDamping);
	
}

//_____________________________________________________________________________
// Set the data members of this ExpressionBasedBushingForce to their null values.
void ExpressionBasedBushingForce::setNull()
{
	setAuthors("Matt DeMers");
    _b1 = NULL;
	_b2 = NULL;
    // no data members

}

//_____________________________________________________________________________
// Allocate and initialize properties.
void ExpressionBasedBushingForce::constructProperties()
{
	constructProperty_body_1();
	constructProperty_body_2();

	//Default location and orientation (rotation sequence)
	constructProperty_location_body_1(Vec3(0)); // body origin
	constructProperty_orientation_body_1(Vec3(0)); // no rotation
	constructProperty_location_body_2(Vec3(0));
	constructProperty_orientation_body_2(Vec3(0));

    //set all deflection functions to zero
	std::string zero = "0.0";
    constructProperty_Mx_expression( zero );
    constructProperty_My_expression( zero );
    constructProperty_Mz_expression( zero );
    constructProperty_Fx_expression( zero );
    constructProperty_Fy_expression( zero );
    constructProperty_Fz_expression( zero );
	setMxExpression( "0.0" );
    setMyExpression( "0.0" );
    setMzExpression( "0.0" );
	setFxExpression( "0.0" );
	setFyExpression( "0.0" );
	setFzExpression( "0.0" );
    
    constructProperty_rotational_damping(Vec3(0));
	constructProperty_translational_damping(Vec3(0));
	
    constructProperty_moment_visual_scale( 1.0 );
    constructProperty_force_visual_scale( 1.0 );
    constructProperty_visual_aspect_ratio(1.0);
}

//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aModel OpenSim model containing this ExpressionBasedBushingForce.
 */
void ExpressionBasedBushingForce::extendConnectToModel(Model& aModel)
{
    Super::extendConnectToModel(aModel); // base class first

	// must initialize the 6 force functions using the user provided expressions
	setMxExpression(get_Mx_expression());
	setMyExpression(get_My_expression());
	setMzExpression(get_Mz_expression());
	setFxExpression(get_Fx_expression());
	setFyExpression(get_Fy_expression());
	setFzExpression(get_Fz_expression());

	string errorMessage;
	const string& body1Name = get_body_1(); // error if unspecified
	const string& body2Name = get_body_2();


	// Look up the two bodies being connected by bushing by name in the
	// model. TODO: keep a pointer to them?
	if (aModel.updBodySet().contains(body1Name)) {
        // ?Keep a pointer to body1?
    }
    else {
		errorMessage = "Invalid bushing body1 (" + body1Name 
                        + ") specified in Force " + getName();
		throw OpenSim::Exception(errorMessage);
	}

	if (aModel.updBodySet().contains(body2Name)) {
        // ?Keep a pointer to body2?
    }
    else {
		errorMessage = "Invalid bushing body2 (" + body2Name 
                        + ") specified in Force " + getName();
		throw OpenSim::Exception(errorMessage);
	}
}

void ExpressionBasedBushingForce::extendAddToSystem(SimTK::MultibodySystem& system) const
{
    Super::extendAddToSystem(system);

	const string&      body1Name            = get_body_1();
	const string&      body2Name            = get_body_2();
	const SimTK::Vec3& locationInBody1      = get_location_body_1();
	const SimTK::Vec3& orientationInBody1   = get_orientation_body_1();
	const SimTK::Vec3& locationInBody2      = get_location_body_2();
	const SimTK::Vec3& orientationInBody2   = get_orientation_body_2();

	Body& body1 = _model->updBodySet().get(body1Name);
	Body& body2 = _model->updBodySet().get(body2Name);

    // Beyond the const Component get access to underlying SimTK elements
	ExpressionBasedBushingForce* mutableThis = const_cast<ExpressionBasedBushingForce *>(this);

    // Get underlying mobilized bodies
    mutableThis->_b1 = &body1.getMobilizedBody();
    mutableThis->_b2 = &body2.getMobilizedBody();
	// Define the transforms for the bushing frames affixed to the specified bodies
	SimTK::Rotation r1; r1.setRotationToBodyFixedXYZ(orientationInBody1);
	SimTK::Rotation r2; r2.setRotationToBodyFixedXYZ(orientationInBody2);
	// Hang on to the transforms for the bushing frames
	mutableThis->_inb1 = SimTK::Transform(r1, locationInBody1);
	mutableThis->_inb2 = SimTK::Transform(r2, locationInBody2);
}

//=============================================================================
// SET
//=============================================================================
//_____________________________________________________________________________
// The following methods set properties of the bushing Force.
void ExpressionBasedBushingForce::setBody1ByName(const std::string& aBodyName)
{
	set_body_1(aBodyName);
}

void ExpressionBasedBushingForce::setBody2ByName(const std::string& aBodyName)
{
	set_body_2(aBodyName);
}

/** Set the location and orientation (optional) for weld on body 1*/
void ExpressionBasedBushingForce::setBody1BushingLocation(const Vec3& location, 
                                           const Vec3& orientation)
{
	set_location_body_1(location);
	set_orientation_body_1(orientation);
}

/** Set the location and orientation (optional) for weld on body 2*/
void ExpressionBasedBushingForce::setBody2BushingLocation(const Vec3& location, 
                                           const Vec3& orientation)
{
	set_location_body_2(location);
	set_orientation_body_2(orientation);
}

/** Set the expression for the Mx function and create it's lepton program */
void ExpressionBasedBushingForce::setMxExpression(std::string expression) 
{
	expression.erase( remove_if(expression.begin(), expression.end(), ::isspace), 
						expression.end() );
	set_Mx_expression(expression);
	MxProg = Lepton::Parser::parse(expression).optimize().createProgram();
}

/** Set the expression for the My function and create it's lepton program */
void ExpressionBasedBushingForce::setMyExpression(std::string expression) 
{
	
	expression.erase( remove_if(expression.begin(), expression.end(), ::isspace), 
						expression.end() );
	set_My_expression(expression);
	MyProg = Lepton::Parser::parse(expression).optimize().createProgram();
}

/** Set the expression for the Mz function and create it's lepton program */
void ExpressionBasedBushingForce::setMzExpression(std::string expression) 
{
	expression.erase( remove_if(expression.begin(), expression.end(), ::isspace), 
						expression.end() );
	set_Mz_expression(expression);
	MzProg = Lepton::Parser::parse(expression).optimize().createProgram();
}

/** Set the expression for the Fx function and create it's lepton program */
void ExpressionBasedBushingForce::setFxExpression(std::string expression) 
{
	expression.erase( remove_if(expression.begin(), expression.end(), ::isspace), 
						expression.end() );
	set_Fx_expression(expression);
	FxProg = Lepton::Parser::parse(expression).optimize().createProgram();
}

/** Set the expression for the Fy function and create it's lepton program */
void ExpressionBasedBushingForce::setFyExpression(std::string expression) 
{
	expression.erase( remove_if(expression.begin(), expression.end(), ::isspace), 
						expression.end() );
	set_Fy_expression(expression);
	FyProg = Lepton::Parser::parse(expression).optimize().createProgram();
}

/** Set the expression for the Fz function and create it's lepton program */
void ExpressionBasedBushingForce::setFzExpression(std::string expression) 
{
	expression.erase( remove_if(expression.begin(), expression.end(), ::isspace), 
						expression.end() );
	set_Fz_expression(expression);
	FzProg = Lepton::Parser::parse(expression).optimize().createProgram();
}
//=============================================================================
// COMPUTATION
//=============================================================================
/** Compute the deflection (spatial separation) of the two frames connected
	by the bushing force. Angualar displacement expressed in Euler angles.
	The force and potential energy are determined by the deflection.  */
SimTK::Vec6 ExpressionBasedBushingForce::computeDeflection(const SimTK::State& s) const
{
	const Transform& X_GB1 = _b1->getBodyTransform(s);

    const Transform& X_GB2 = _b2->getBodyTransform(s);

	// Define the frame on body 1 is the "fixed" frame, F
	Transform X_GF = X_GB1 * _inb1;
	// Define the frame on body 2 as the "moving" frame, M
    Transform X_GM = X_GB2 * _inb2;
	// Express M in F
    Transform X_FM = ~X_GF * X_GM;    

	// the deviation of the two frames measured by dq
    Vec6 dq(0);
	// First 3 are rotational deviations
	dq.updSubVec<3>(0) = X_FM.R().convertRotationToBodyFixedXYZ();
	// Second 3 are translational
	dq.updSubVec<3>(3) = X_FM.p();

	return dq;
}

/** compute the bushing force at the bushing location
*/
void ExpressionBasedBushingForce::ComputeForcesAtBushing(const SimTK::State& state, 
    SpatialVec& forces_on_M_in_ground, SpatialVec& forces_on_F_in_ground) const
{
    const Transform& X_GB1 = _b1->getBodyTransform(state);
    const Transform& X_GB2 = _b2->getBodyTransform(state);

	Transform X_GF = X_GB1 * _inb1;   
    Transform X_GM = X_GB2 * _inb2;   
    Transform X_FM = ~X_GF * X_GM;    
	const Rotation& R_GF = X_GF.R();
	const Rotation& R_GM = X_GM.R();
	const Rotation& R_FM = X_FM.R();

    // Calculate stiffness generalized forces of bushing by first computing
	// the deviation of the two frames measured by dq
    Vec6 dq = computeDeflection(state);

    // TO DO: compute the deflection forces from functions
    //------------------------------------------
    Vec6 fk = Vec6(0.0);

	std::map<std::string, double> deflectionVars;
	deflectionVars["theta_x"] = dq[0];
	deflectionVars["theta_y"] = dq[1];
	deflectionVars["theta_z"] = dq[2];
	deflectionVars["delta_x"] = dq[3];
	deflectionVars["delta_y"] = dq[4];
	deflectionVars["delta_z"] = dq[5];

	
	fk[0] = MxProg.evaluate(deflectionVars);
	fk[1] = MyProg.evaluate(deflectionVars);
	fk[2] = MzProg.evaluate(deflectionVars);
	fk[3] = FxProg.evaluate(deflectionVars);
	fk[4] = FyProg.evaluate(deflectionVars);
	fk[5] = FzProg.evaluate(deflectionVars);

    // Now evaluate velocities.
    const SpatialVec& V_GB1 = _b1->getBodyVelocity(state);
    const SpatialVec& V_GB2 = _b2->getBodyVelocity(state);

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


    // QUESTION:  Should be damp omega or qdot?  The process that follows
    //            here damps qdot.

    // Need angular velocity in M frame for conversion to qdot.
    const Vec3  w_FM_M = ~R_FM * V_FM[0];
    const Mat33 N_FM   = Rotation::calcNForBodyXYZInBodyFrame(dq.getSubVec<3>(0));
	Vec6 dqdot(0);
    dqdot.updSubVec<3>(0) = N_FM * w_FM_M;
    dqdot.updSubVec<3>(3) = V_FM[1];

    // TO DO: compute damping matrix earlier?
    //------------------------------------------
    SimTK::Mat66 _dampingMatrix(0.0);

    // fill damping matrix with damping from  vector member
    for(int i=0; i<3;i++){
        _dampingMatrix[i][i] = get_rotational_damping(0)[i];
        _dampingMatrix[i+3][i+3] = get_translational_damping(0)[i];
    }
	
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

    forces_on_M_in_ground[0] = mB2_G;
    forces_on_M_in_ground[1] = fM_G;

    forces_on_F_in_ground[0] = -(mB2_G + p_FM_G % fM_G);
    forces_on_F_in_ground[1] = -fM_G;
    //SpatialVec F_GM(  mB2_G,                       fM_G);
    //SpatialVec F_GF(-(mB2_G + p_FM_G % fM_G) , -fM_G);
}
/**
 * Compute the force contribution to the system and add in to appropriate
 * bodyForce and/or system generalizedForce.
 * ExpressionBasedBushingForce implementation based SimTK::Force::LinearBushing
 * developed and implemented by Michael Sherman.
 */
void ExpressionBasedBushingForce::computeForce(const SimTK::State& s, 
							  SimTK::Vector_<SimTK::SpatialVec>& bodyForces, 
							  SimTK::Vector& generalizedForces) const
{
    
    
    SpatialVec F_GM( Vec3(0.0),Vec3(0.0) );
    SpatialVec F_GF( Vec3(0.0),Vec3(0.0) );
    
    
    ComputeForcesAtBushing(s, F_GM, F_GF);
    

    const Transform& X_GB1 = _b1->getBodyTransform(s);
    const Transform& X_GB2 = _b2->getBodyTransform(s);

	Transform X_GF = X_GB1 * _inb1;   
    Transform X_GM = X_GB2 * _inb2;   
    Transform X_FM = ~X_GF * X_GM;    
	const Rotation& R_GF = X_GF.R();
	const Rotation& R_GM = X_GM.R();
	const Rotation& R_FM = X_FM.R();

    // Re-express local vectors in the Ground frame.
    Vec3 p_B1F_G =  X_GB1.R() * _inb1.p();   // 15 flops
    Vec3 p_B2M_G =  X_GB2.R() * _inb2.p();   // 15 flops
    Vec3 p_FM_G  =  X_GF.R()  * X_FM.p();    // 15 flops
    
    // Shift forces to body origins.
    SpatialVec F_GB2(F_GM[0] + p_B2M_G % F_GM[1], F_GM[1]);
    SpatialVec F_GB1(F_GF[0] + p_B1F_G % F_GF[1], F_GF[1]);

	// Apply (add-in) the body forces to the system set of body forces
	bodyForces[_b2->getMobilizedBodyIndex()] += F_GB2;
    bodyForces[_b1->getMobilizedBodyIndex()] += F_GB1;
}

/** Potential energy stored in the bushing is purely a function of the deflection
    from the rest position and the stiffness */
/**
double CoupledBushingForce::computePotentialEnergy(const SimTK::State& s) const
{
	Vec6 dq = computeDeflection(s);
	// Energy stored in the bushing
	double U = 0.5*(~dq*_stiffnessMatrix*dq);
	return U;
}
*/

//=============================================================================
// Reporting
//=============================================================================
/** 
 * Provide names of the quantities (column labels) of the force value(s) reported
 * 
 */
OpenSim::Array<std::string> ExpressionBasedBushingForce::getRecordLabels() const 
{
	const string& body1Name = get_body_1();
	const string& body2Name = get_body_2();

	OpenSim::Array<std::string> labels("");
	labels.append(getName()+"."+body1Name+".force.X");
	labels.append(getName()+"."+body1Name+".force.Y");
	labels.append(getName()+"."+body1Name+".force.Z");
	labels.append(getName()+"."+body1Name+".torque.X");
	labels.append(getName()+"."+body1Name+".torque.Y");
	labels.append(getName()+"."+body1Name+".torque.Z");
	labels.append(getName()+"."+body2Name+".force.X");
	labels.append(getName()+"."+body2Name+".force.Y");
	labels.append(getName()+"."+body2Name+".force.Z");
	labels.append(getName()+"."+body2Name+".torque.X");
	labels.append(getName()+"."+body2Name+".torque.Y");
	labels.append(getName()+"."+body2Name+".torque.Z");

	return labels;
}
/**
 * Provide the value(s) to be reported that correspond to the labels
 */
OpenSim::Array<double> ExpressionBasedBushingForce::
getRecordValues(const SimTK::State& state) const 
{
    SpatialVec F_GM( Vec3(0.0),Vec3(0.0) );
    SpatialVec F_GF( Vec3(0.0),Vec3(0.0) );
    
    ComputeForcesAtBushing(state, F_GM, F_GF);

    OpenSim::Array<double> values(1);

    values.append(3, &F_GF[1][0]);
	values.append(3, &F_GF[0][0]);
    values.append(3, &F_GM[1][0]);
	values.append(3, &F_GM[0][0]);


    /*  Old stuff reporting body forces at body origin
    -----------------------------------------------------
	const string& body1Name = get_body_1();
	const string& body2Name = get_body_2();

	OpenSim::Array<double> values(1);

	const SimTK::Force::LinearBushing &simtkSpring = 
        (SimTK::Force::LinearBushing &)(_model->getForceSubsystem().getForce(_index));

	SimTK::Vector_<SimTK::SpatialVec> bodyForces(0);
	SimTK::Vector_<SimTK::Vec3> particleForces(0);
	SimTK::Vector mobilityForces(0);

	//get the net force added to the system contributed by the bushing
	simtkSpring.calcForceContribution(state, bodyForces, particleForces, mobilityForces);
	SimTK::Vec3 forces = bodyForces(_model->getBodySet().get(body1Name).getIndex())[1];
	SimTK::Vec3 torques = bodyForces(_model->getBodySet().get(body1Name).getIndex())[0];
	values.append(3, &forces[0]);
	values.append(3, &torques[0]);

	forces = bodyForces(_model->getBodySet().get(body2Name).getIndex())[1];
	torques = bodyForces(_model->getBodySet().get(body2Name).getIndex())[0];

	values.append(3, &forces[0]);
	values.append(3, &torques[0]);

    */

	return values;
}

//_____________________________________________________________________________
/**
 * Get the visible object used to represent the spring.
 */

void ExpressionBasedBushingForce::updateDisplayer(const SimTK::State& s)
{
	SimTK::Vec3 globalLocation1, globalLocation2;
	const OpenSim::Body& body1 = _model->getBodySet().get(get_body_1());
	const OpenSim::Body& body2 = _model->getBodySet().get(get_body_2());
	_model->getSimbodyEngine().transformPosition(s, body1, get_location_body_1(), 
                                                 globalLocation1);
	_model->getSimbodyEngine().transformPosition(s, body2, get_location_body_2(),
                                                 globalLocation2);
    /*
	if (_displayer.countGeometry()==0){
		Geometry *g = new LineGeometry();
		g->setFixed(false);
		_displayer.addGeometry(g);
	}
	((LineGeometry *)_displayer.getGeometry(0))->
        setPoints(globalLocation1, globalLocation2);
        */
}

void ExpressionBasedBushingForce::updateGeometry(const SimTK::State& s)
{
    /*
	if (_displayer.countGeometry()==0){
		Geometry *g = new LineGeometry();
		g->setFixed(false);
		_displayer.addGeometry(g);
	} */
	updateDisplayer(s);
}

void ExpressionBasedBushingForce::generateDecorations
       (bool                                        fixed, 
        const ModelDisplayHints&                    hints,
        const SimTK::State&                         state,
        SimTK::Array_<SimTK::DecorativeGeometry>&   geometryArray) const
    {
        // invoke parent class method
        Super::generateDecorations(fixed,hints,state,geometryArray); 
        // the frame on body 1 will be red
        SimTK::Vec3 frame1color(1.0,0.0,0.0);
        // the frame on body 2 will be blue
        SimTK::Vec3 frame2color(0.0,0.5,1.0);
        // the moment on body 2 will be yellow
        SimTK::Vec3 moment2color(1.0,1.0,0.0);
        // the force on body 2 will be green
        SimTK::Vec3 force2color(0.0,1.0,0.0);

        // create frames to be fixed on body 1 and body 2
        SimTK::DecorativeFrame body1Frame(0.2);
        SimTK::DecorativeFrame body2Frame(0.2);

        // attach frame 1 to body 1, translate and rotate it to the location of the bushing
        body1Frame.setBodyId(_model->getBodySet().get(get_body_1()).getMobilizedBodyIndex());
        body1Frame.setTransform(_inb1);
        body1Frame.setColor(frame1color);

        // attach frame 2 to body 2, translate and rotate it to the location of the bushing
        body2Frame.setBodyId(_model->getBodySet().get(get_body_2()).getMobilizedBodyIndex());
        body2Frame.setTransform(_inb2);
        body2Frame.setColor(frame2color);

        geometryArray.push_back(body1Frame);
        geometryArray.push_back(body2Frame);


        // if the model is moving, calculate and draw the bushing forces.
        if(!fixed){

            SpatialVec F_GM( Vec3(0.0),Vec3(0.0) );
            SpatialVec F_GF( Vec3(0.0),Vec3(0.0) );
        
            _model->getMultibodySystem().realize(state, Stage::Velocity );
            // calculate forces and moments the bushing applies to each body
            ComputeForcesAtBushing(state, F_GM, F_GF);

            // location of the bushing on body 2
            SimTK::Vec3 p_b2M_b2 = _inb2.p();
            SimTK::Vec3 p_GM_G(0.0);
            
            // fing the body2 location of the bushing in ground
            _model->getSimbodyEngine().transformPosition(state, 
                _model->getBodySet().get(get_body_2()), p_b2M_b2, p_GM_G);

            // Add moment on body2 as line vector starting at bushing location
            SimTK::Vec3 scaled_M_GM(get_moment_visual_scale()*F_GM[0]);
            SimTK::Real m_length(scaled_M_GM.norm());
            SimTK::Real m_radius(m_length/get_visual_aspect_ratio()/2.0);
            SimTK::Transform   X_m2cylinder( SimTK::Rotation( SimTK::UnitVec3(scaled_M_GM), SimTK::YAxis), p_GM_G + scaled_M_GM/2.0);
            SimTK::DecorativeCylinder body2Moment(m_radius, m_length/2.0);
            body2Moment.setTransform(X_m2cylinder);
            body2Moment.setColor(moment2color);
            geometryArray.push_back(body2Moment);

            // Add force on body2 as line vector starting at bushing location
            SimTK::Vec3 scaled_F_GM(get_force_visual_scale()*F_GM[1]);
            SimTK::Real f_length(scaled_F_GM.norm());
            SimTK::Real f_radius(f_length/get_visual_aspect_ratio()/2.0);
            SimTK::Transform   X_f2cylinder( SimTK::Rotation( SimTK::UnitVec3(scaled_F_GM), SimTK::YAxis), p_GM_G + scaled_F_GM/2.0);
            SimTK::DecorativeCylinder body2Force(f_radius, f_length/2.0);
            body2Force.setTransform(X_f2cylinder);
            body2Force.setColor(force2color);
            
            geometryArray.push_back(body2Force);

        }

    }
