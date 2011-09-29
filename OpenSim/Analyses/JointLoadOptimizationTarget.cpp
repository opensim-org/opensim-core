// JointLoadOptimizationTarget.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Matt DeMers
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c)  2005, Stanford University. All rights reserved. 
* Use of the OpenSim software in source form is permitted provided that the following
* conditions are met:
* 	1. The software is used only for non-commercial research and education. It may not
*     be used in relation to any commercial activity.
* 	2. The software is not distributed or redistributed.  Software distribution is allowed 
*     only through https://simtk.org/home/opensim.
* 	3. Use of the OpenSim software or derivatives must be acknowledged in all publications,
*      presentations, or documents describing work in which OpenSim or derivatives are used.
* 	4. Credits to developers may not be removed from executables
*     created from modifications of the source.
* 	5. Modifications of source code must retain the above copyright notice, this list of
*     conditions and the following disclaimer. 
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
*  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
*  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
*  SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
*  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
*  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR BUSINESS INTERRUPTION) OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
*  WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/* 
 *
 *
 */


//=============================================================================
// INCLUDES
//=============================================================================
#include <stdlib.h>
#include <stdio.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/ActivationFiberLengthMuscle.h>
#include <OpenSim/Simulation/Model/ForceSet.h>
#include <OpenSim/Simulation/SimbodyEngine/Coordinate.h>
#include <OpenSim/Simulation/SimbodyEngine/SimbodyEngine.h>
#include "JointLoadOptimizationTarget.h"
#include <iostream>

using namespace OpenSim;
using namespace std;
using SimTK::Vector;
using SimTK::Matrix;
using SimTK::Real;

#define USE_LINEAR_CONSTRAINT_MATRIX

const double JointLoadOptimizationTarget::SMALLDX = 1.0e-14;
//const double JointLoadOptimizationTarget::_activationExponent = 2.0;
 
//==============================================================================
// CONSTRUCTOR
//==============================================================================
//______________________________________________________________________________
/**
 * Construct an optimization target.
 *
 * @param aNP The number of parameters.
 * @param aNC The number of constraints.
 * @param aT Current time in the integration.
 * @param aX Current control values.
 * @param aY Current states.
 * @param aDYDT Current state derivatives.
 */
JointLoadOptimizationTarget::
JointLoadOptimizationTarget(const SimTK::State& s, Model *aModel,int aNP,int aNC, Set<JointReactionReference>& aRefSet, bool useMusclePhysiology):
	_jointReferenceSet(aRefSet)
{

	// ALLOCATE STATE ARRAYS
	_recipAreaSquared.setSize(aNP);
	_recipOptForceSquared.setSize(aNP);
	_optimalForce.setSize(aNP);
	_useMusclePhysiology=useMusclePhysiology;
	

	setModel(*aModel);
	setNumParams(aNP);
	setNumConstraints(aNC);
	setActivationExponent(2.0);
	computeActuatorAreas(s);

	// Gather indices into speed set corresponding to the unconstrained degrees of freedom (for which we will set acceleration constraints)
	_accelerationIndices.setSize(0);
	const CoordinateSet& coordSet = _model->getCoordinateSet();
	for(int i=0; i<coordSet.getSize(); i++) {
		const Coordinate& coord = coordSet.get(i);
		if(!coord.isConstrained(s)) {
			_accelerationIndices.append(i);
		}
	}

}


//==============================================================================
// CONSTRUCTION
//==============================================================================
bool JointLoadOptimizationTarget::
prepareToOptimize(SimTK::State& s, double *x)
{

	// for each joint reference, determine the child body index, the index 
	// of the body receiving the joint load (parent or child of joint) and the 
	// index of the body whoes reference frame will be used to express the joint load
	// vectors.  Compute and cache these in a vector now so that the optimizer
	// doesn't wast time doing it for each objective function evaluation
	buildLoadDescriptionIndices();


	// COMPUTE MAX ISOMETRIC FORCE
	const ForceSet& fSet = _model->getForceSet();
    
	for(int i=0, j=0;i<fSet.getSize();i++) {
 		 Actuator* act = dynamic_cast<Actuator*>(&fSet.get(i));
         if( act ) {
             double fOpt;
             ActivationFiberLengthMuscle *mus = dynamic_cast<ActivationFiberLengthMuscle*>(&fSet.get(i));
             if( mus ) {
    		 	  if(_useMusclePhysiology) {
					_model->setAllControllersEnabled(true);
    				fOpt = mus->computeIsokineticForceAssumingInfinitelyStiffTendon(const_cast<SimTK::State&>(s), 1.0);
					_model->setAllControllersEnabled(false);
    			  } else {
    				fOpt = mus->getMaxIsometricForce();
                  }
             } else {
                  fOpt = act->getOptimalForce();
             }
		    _optimalForce[j++] = fOpt;
		 }
	}

#ifdef USE_LINEAR_CONSTRAINT_MATRIX
	//cout<<"Computing linear constraint matrix..."<<endl;
	int np = getNumParameters();
	int nc = getNumConstraints();

	_constraintMatrix.resize(nc,np);
	_constraintVector.resize(nc);

	Vector pVector(np), cVector(nc);

	// Build linear constraint matrix and constant constraint vector
	pVector = 0;
	computeConstraintVector(s, pVector,_constraintVector);

	for(int p=0; p<np; p++) {
		pVector[p] = 1;
		computeConstraintVector(s, pVector, cVector);
		for(int c=0; c<nc; c++) _constraintMatrix(c,p) = (cVector[c] - _constraintVector[c]);
		pVector[p] = 0;
	}
#endif

	// return false to indicate that we still need to proceed with optimization
	return false;
}
//==============================================================================
// SET AND GET
//==============================================================================
//------------------------------------------------------------------------------
// MODEL
//------------------------------------------------------------------------------
///______________________________________________________________________________
/**
 * Set the model.
 *
 * @param aModel Model.
 */
void JointLoadOptimizationTarget::
setModel(Model& aModel)
{
	_model = &aModel;
}
//------------------------------------------------------------------------------
// STATES STORAGE
//------------------------------------------------------------------------------
///______________________________________________________________________________
/**
 * Set the states storage.
 *
 * @param aStatesStore States storage.
 */
void JointLoadOptimizationTarget::
setStatesStore(const Storage *aStatesStore)
{
	_statesStore = aStatesStore;
}
//------------------------------------------------------------------------------
// STATES SPLINE SET
//------------------------------------------------------------------------------
///______________________________________________________________________________
/**
 * Set the states spline set.
 *
 * @param aStatesSplineSet States spline set.
 */
void JointLoadOptimizationTarget::
setStatesSplineSet(GCVSplineSet aStatesSplineSet)
{
	_statesSplineSet = aStatesSplineSet;
}

//------------------------------------------------------------------------------
// CONTROLS
//------------------------------------------------------------------------------
///______________________________________________________________________________
/**
 * Set the number of paramters.
 *
 * The number of parameters can be set at any time.  However, the perturbation
 * sizes for the parameters (i.e., _dx) is destroyed.  Therefore, the
 * perturbation sizes must be reset.
 *
 * @param aNP Number of parameters.
 * @see setDX()
 */
void JointLoadOptimizationTarget::
setNumParams(const int aNP)
{
	setNumParameters(aNP);
	_dx.setSize(getNumParameters());
}

//------------------------------------------------------------------------------
// CONSTRAINTS
//------------------------------------------------------------------------------
///______________________________________________________________________________
/**
 * Set the number of constraints.
 *
 * @param aNC Number of constraints.
 */
void JointLoadOptimizationTarget::
setNumConstraints(const int aNC)
{
	// There are only linear equality constraints.
	setNumEqualityConstraints(aNC);
	setNumLinearEqualityConstraints(aNC);
}	

//------------------------------------------------------------------------------
// DERIVATIVE PERTURBATION SIZES
//------------------------------------------------------------------------------
//______________________________________________________________________________
/**
 * Set the derivative perturbation size.
 */
void JointLoadOptimizationTarget::
setDX(int aIndex,double aValue)
{
	// VALIDATE VALUE
	validatePerturbationSize(aValue);

	// SET VALUE (use get to do bounds checking)
	_dx.get(aIndex) = aValue;
}
//______________________________________________________________________________
/**
 * Set the derivative perturbation size for all controls.
 */
void JointLoadOptimizationTarget::
setDX(double aValue)
{
	// VALIDATE VALUE
	validatePerturbationSize(aValue);

	// SET VALUE
	for(int i=0;i<getNumParameters();i++) _dx.get(i) = aValue;
}
//______________________________________________________________________________
/**
 * Get the derivative perturbation size.
 */
double JointLoadOptimizationTarget::
getDX(int aIndex)
{
	return _dx.get(aIndex);
}
//______________________________________________________________________________
/**
 * Get a pointer to the vector of derivative perturbation sizes.
 */
double* JointLoadOptimizationTarget::
getDXArray()
{
	return &_dx[0];
}

//______________________________________________________________________________
/**
 * Get an optimal force.
 */
void JointLoadOptimizationTarget::
getActuation(SimTK::State& s, const SimTK::Vector &parameters, SimTK::Vector &forces)
{
	//return(_optimalForce[aIndex]);
	const ForceSet& fs = _model->getForceSet();
	SimTK::Vector tempAccel(getNumConstraints());
	computeAcceleration(s, parameters, tempAccel);
	for(int i=0,j=0;i<fs.getSize();i++) {
        Actuator* act = dynamic_cast<Actuator*>(&fs.get(i));
		if( act )forces(j++) = act->getForce(s);
	}
}
//==============================================================================
// UTILITY
//==============================================================================
//______________________________________________________________________________
/**
 * Ensure that a derivative perturbation is a valid size
 */
void JointLoadOptimizationTarget::
validatePerturbationSize(double &aSize)
{
	if(aSize<SMALLDX) {
		printf("JointLoadOptimizationTarget.validatePerturbationSize: WARNING- ");
		printf("dx size too small (%le).\n",aSize);
		printf("\tResetting dx=%le.\n",SMALLDX);
		aSize = SMALLDX;
	}
}
//______________________________________________________________________________
/**
 */
void JointLoadOptimizationTarget::
printPerformance(SimTK::State& s, double *parameters)
{
	double p;
	setCurrentState( &s );
	objectiveFunc(SimTK::Vector(getNumParameters(),parameters,true),true,p);
	SimTK::Vector constraints(getNumConstraints());
	constraintFunc(SimTK::Vector(getNumParameters(),parameters,true),true,constraints);
	cout << endl;
	cout << "time = " << s.getTime() <<" Performance =" << p << 
	" Constraint violation = " << sqrt(~constraints*constraints) << endl;
}

//______________________________________________________________________________
/**
 */
void JointLoadOptimizationTarget::
computeActuatorAreas(const SimTK::State& s )
{
	// COMPUTE ACTUATOR AREAS
	ForceSet& forceSet = _model->updForceSet();
	for(int i=0, j=0;i<forceSet.getSize();i++) {
        Actuator *act = dynamic_cast<Actuator*>(&forceSet.get(i));
        if( act ) {
 		     act->setForce(s, 1.0);
    		 _recipAreaSquared[j] = act->getStress(s);
    		 _recipAreaSquared[j] *= _recipAreaSquared[j];
             j++;
        }
	}
}

//=============================================================================
// STATIC DERIVATIVES
//=============================================================================
//_____________________________________________________________________________
/**
 * Compute derivatives of a constraint with respect to the
 * controls by central differences.
 *
 * @param dx An array of control perturbation values.
 * @param x Values of the controls at time t.
 * @param ic Index of the constraint.
 * @param dcdx The derivatives of the constraints.
 *
 * @return -1 if an error is encountered, 0 otherwize.
 */
int JointLoadOptimizationTarget::
CentralDifferencesConstraint(const JointLoadOptimizationTarget *aTarget,
	double *dx,const Vector &x,Matrix &jacobian)
{
	if(aTarget==NULL) return(-1);

	// INITIALIZE CONTROLS
	int nx = aTarget->getNumParameters(); if(nx<=0) return(-1);
	int nc = aTarget->getNumConstraints(); if(nc<=0) return(-1);
	Vector xp=x;
	Vector cf(nc),cb(nc);

	// INITIALIZE STATUS
	int status = -1;

	// LOOP OVER CONTROLS
	for(int i=0;i<nx;i++) {

		// PERTURB FORWARD
		xp[i] = x[i] + dx[i];
		status = aTarget->constraintFunc(xp,true,cf);
		if(status<0) return(status);

		// PERTURB BACKWARD
		xp[i] = x[i] - dx[i];
		status = aTarget->constraintFunc(xp,true,cb);
		if(status<0) return(status);

		// DERIVATIVES OF CONSTRAINTS
		double rdx = 0.5 / dx[i];
		for(int j=0;j<nc;j++) jacobian(j,i) = rdx*(cf[j]-cb[j]);

		// RESTORE CONTROLS
		xp[i] = x[i];
	}

	return(status);
}
//_____________________________________________________________________________
/**
 * Compute derivatives of performance with respect to the
 * controls by central differences.  Note that the gradient array should
 * be allocated as dpdx[nx].
 *
 * @param dx An array of control perturbation values.
 * @param x Values of the controls at time t.
 * @param dpdx The derivatives of the performance criterion.
 *
 * @return -1 if an error is encountered, 0 otherwize.
 */
int JointLoadOptimizationTarget::
CentralDifferences(const JointLoadOptimizationTarget *aTarget,
	double *dx,const Vector &x,Vector &dpdx)
{
	if(aTarget==NULL) return(-1);

	// CONTROLS
	int nx = aTarget->getNumParameters();  if(nx<=0) return(-1);
	Vector xp=x;

	// PERFORMANCE
	double pf,pb;

	// INITIALIZE STATUS
	int status = -1;

	// LOOP OVER CONTROLS
	for(int i=0;i<nx;i++) {

		// PERTURB FORWARD
		xp[i] = x[i] + dx[i];
		status = aTarget->objectiveFunc(xp,true,pf);
		if(status<0) return(status);

		// PERTURB BACKWARD
		xp[i] = x[i] - dx[i];
		status = aTarget->objectiveFunc(xp,true,pb);
		if(status<0) return(status);

		// DERIVATIVES OF PERFORMANCE
		double rdx = 0.5 / dx[i];
		dpdx[i] = rdx*(pf-pb);

		// RESTORE CONTROLS
		xp[i] = x[i];
	}

	return(status);
}

//==============================================================================
// PERFORMANCE AND CONSTRAINTS
//==============================================================================
//------------------------------------------------------------------------------
// PERFORMANCE
//------------------------------------------------------------------------------
//______________________________________________________________________________
/**
 * Compute performance given parameters.
 *
 * @param parameters Vector of optimization parameters.
 * @param performance Value of the performance criterion.
 * @return Status (normal termination = 0, error < 0).
 */
int JointLoadOptimizationTarget::
objectiveFunc(const Vector &parameters, const bool new_parameters, Real &performance) const
{
	// For timing
	/*
	LARGE_INTEGER start;
	LARGE_INTEGER stop;
	LARGE_INTEGER frequency;

	QueryPerformanceFrequency(&frequency);
	QueryPerformanceCounter(&start);
	*/

	int na = _model->getActuators().getSize();
	double p = 0.0;
	for(int i=0;i<na;i++) {
		p +=  pow(fabs(parameters[i]),_activationExponent);
	}

	//const SimTK::State* localState = getCurrentState();
	/*
	SimTK::Vector_<SimTK::Vec3> load1(3);
	SimTK::Vector_<SimTK::Vec3> load2(3);
	*/
	double jointTerm = 0;

	computeJointLoadsCost(*_currentState, parameters, jointTerm);

	/*
	int numBodies = _model->getNumBodies();
	SimTK::Vector_<SimTK::Vec3> forces(numBodies);
	SimTK::Vector_<SimTK::Vec3> moments(numBodies);

	computeJointLoads(*_currentState, parameters, forces, moments);
	//Start Debuging lines
	
	SimTK::Vec3 zero(0.0,0.0,0.0);
	//load1 = zero;
	//load2 = zero;
	//std::cout << "Load 1: " << load1 << endl;
	//std::cout << "Load 2: " << load2 << endl;
	
	//End Debuging lines
	
	int numReferenceJoints = _jointReferenceSet->getSize();
	for(int i=0; i<numReferenceJoints; i++) {
		//for each joint load we're interested in
		for(int j=0; j<3; j++) {
			// add the weighted square of the force and moment component
			// to the jointTerm of the cost function
			jointTerm += pow(forces[i][j], 2);
			jointTerm += pow(moments[i][j], 2);
		}

	}
	*/
	

	performance = (p + jointTerm * _jointTermScaleFactor) * _objectiveScaleFactor;

	//parameters.dump();
	//std::cout<<"p = " <<p << ", j = " << jointTerm <<std::endl;

	// for timing
	/*
	QueryPerformanceCounter(&stop);
	double duration = (double)(stop.QuadPart-start.QuadPart)/(double)frequency.QuadPart;
	std::cout << "objectiveFunc time = " << (duration*1.0e3) << " milliseconds" << std::endl;
	*/
	

	return(0);
}
//______________________________________________________________________________
/**
 * Compute the gradient of performance given parameters.
 *
 * @param parameters Vector of optimization parameters.
 * @param gradient Derivatives of performance with respect to the parameters.
 * @return Status (normal termination = 0, error < 0).
 */
int JointLoadOptimizationTarget::
gradientFunc(const Vector &parameters, const bool new_parameters, Vector &gradient) const
{
	//LARGE_INTEGER start;
	//LARGE_INTEGER stop;
	//LARGE_INTEGER frequency;

	//QueryPerformanceFrequency(&frequency);
	//QueryPerformanceCounter(&start);

	int na = _model->getActuators().getSize();
	for(int i=0;i<na;i++) {
		if(parameters[i] < 0) {
			gradient[i] =  -1.0 * _activationExponent * pow(fabs(parameters[i]),_activationExponent-1.0);
		} else {
			gradient[i] =  _activationExponent * pow(fabs(parameters[i]),_activationExponent-1.0);
	}
	}

	//QueryPerformanceCounter(&stop);
	//double duration = (double)(stop.QuadPart-start.QuadPart)/(double)frequency.QuadPart;
	//std::cout << "gradientFunc time = " << (duration*1.0e3) << " milliseconds" << std::endl;

	// 0.02 ms

	return(0);
}

//------------------------------------------------------------------------------
// CONSTRAINT
//------------------------------------------------------------------------------
//______________________________________________________________________________
/**
 * Compute acceleration constraints given parameters.
 *
 * @param parameters Vector of optimization parameters.
 * @param constraints Vector of optimization constraints.
 * @return Status (normal termination = 0, error < 0).
 */
int JointLoadOptimizationTarget::
constraintFunc(const SimTK::Vector &parameters, const bool new_parameters, SimTK::Vector &constraints) const
{
	//LARGE_INTEGER start;
	//LARGE_INTEGER stop;
	//LARGE_INTEGER frequency;

	//QueryPerformanceFrequency(&frequency);
	//QueryPerformanceCounter(&start);

#ifndef USE_LINEAR_CONSTRAINT_MATRIX

	// Evaluate constraint function for all constraints and pick the appropriate component
	computeConstraintVector(parameters,constraints);

#else

	// Use precomputed constraint matrix
	//cout<<"Computing constraints assuming linear dependence..."<<endl;
	constraints = _constraintMatrix * parameters + _constraintVector;

#endif

	//QueryPerformanceCounter(&stop);
	//double duration = (double)(stop.QuadPart-start.QuadPart)/(double)frequency.QuadPart;
	//std::cout << "constraintFunc time = " << (duration*1.0e3) << " milliseconds" << std::endl;

	// 0.11 ms

	return(0);
}

//______________________________________________________________________________
/**
 * Compute all constraints given parameters.
 */
void JointLoadOptimizationTarget::
computeConstraintVector(SimTK::State& s, const Vector &parameters,Vector &constraints) const
{
	//LARGE_INTEGER start;
	//LARGE_INTEGER stop;
	//LARGE_INTEGER frequency;

	//QueryPerformanceFrequency(&frequency);
	//QueryPerformanceCounter(&start);

	// Compute actual accelerations
	Vector actualAcceleration(getNumConstraints());
	computeAcceleration(s, parameters, actualAcceleration);

	// CONSTRAINTS
	for(int i=0; i<getNumConstraints(); i++) {
		Coordinate& coord = _model->getCoordinateSet().get(_accelerationIndices[i]);
		Function& presribedFunc = _statesSplineSet.get(_statesStore->getStateIndex(coord.getName(),0));
		std::vector<int> derivComponents(2);
		derivComponents[0]=0;
		derivComponents[1]=0;
		double targetAcceleration = presribedFunc.calcDerivative(derivComponents,SimTK::Vector(1,s.getTime()));
		//std::cout << "computeConstraintVector:" << targetAcceleration << " - " <<  actualAcceleration[i] << endl;
		constraints[i] = targetAcceleration - actualAcceleration[i];
	}

	//QueryPerformanceCounter(&stop);
	//double duration = (double)(stop.QuadPart-start.QuadPart)/(double)frequency.QuadPart;
	//std::cout << "computeConstraintVector time = " << (duration*1.0e3) << " milliseconds" << std::endl;

	// 1.5 ms
}
//______________________________________________________________________________
/**
 * Compute the gradient of constraint given parameters.
 *
 * @param parameters Vector of parameters.
 * @param jac Derivative of constraint with respect to the parameters.
 * @return Status (normal termination = 0, error < 0).
 */
int JointLoadOptimizationTarget::
constraintJacobian(const SimTK::Vector &parameters, const bool new_parameters, SimTK::Matrix &jac) const
{
	//LARGE_INTEGER start;
	//LARGE_INTEGER stop;
	//LARGE_INTEGER frequency;

	//QueryPerformanceFrequency(&frequency);
	//QueryPerformanceCounter(&start);

#ifndef USE_LINEAR_CONSTRAINT_MATRIX

	// Compute gradient 
	JointLoadOptimizationTarget::CentralDifferencesConstraint(this,&_dx[0],parameters,jac);

#else

	// Use precomputed constraint matrix (works if constraint is linear)
	//cout<<"Computing constraint gradient assuming linear dependence..."<<endl;
	jac = _constraintMatrix;

#endif

	//QueryPerformanceCounter(&stop);
	//double duration = (double)(stop.QuadPart-start.QuadPart)/(double)frequency.QuadPart;
	//std::cout << "constraintJacobian time = " << (duration*1.0e3) << " milliseconds" << std::endl;

	// 0.01 ms

	return 0;
}
//=============================================================================
// ACCELERATION
//=============================================================================
//
void JointLoadOptimizationTarget::
computeAcceleration(SimTK::State& s, const SimTK::Vector &parameters,SimTK::Vector &rAccel) const
{
	//LARGE_INTEGER start;
	//LARGE_INTEGER stop;
	//LARGE_INTEGER frequency;

	//QueryPerformanceFrequency(&frequency);
	//QueryPerformanceCounter(&start);

	// SimTK requires that time be >= 0 when setting Discreate variables (overrideForce)
	// JACKM: Need to talk to sherm if this restriction can be removed
	double time = s.getTime();
	

	const ForceSet& fs = _model->getForceSet();
	for(int i=0,j=0;i<fs.getSize();i++)  {
         Actuator *act = dynamic_cast<Actuator*>(&fs.get(i));
		 if( act ) {
             //act->overrideForce(s,true);
             act->setOverrideForce(s,parameters[j]*_optimalForce[j]);
		 }
         j++;
    }

	_model->getMultibodySystem().realize(s,SimTK::Stage::Acceleration);

	SimTK::Vector udot = _model->getMatterSubsystem().getUDot(s);

	for(int i=0; i<_accelerationIndices.getSize(); i++) 
		rAccel[i] = udot[_accelerationIndices[i]];

	//QueryPerformanceCounter(&stop);
	//double duration = (double)(stop.QuadPart-start.QuadPart)/(double)frequency.QuadPart;
	//std::cout << "computeAcceleration time = " << (duration*1.0e3) << " milliseconds" << std::endl;

	// 1.45 ms
}

//=============================================================================
// JOINT LOADS
//=============================================================================
//


/* Compute the cummulative contribution of joint load terms to the objective function
* using the weights and load descriptions specified by the JointReactionReferenceSet.
*/
void JointLoadOptimizationTarget::
computeJointLoadsCost(SimTK::State& s, const SimTK::Vector &parameters, double &aCost) const
{
	int numBodies = _model->getNumBodies();
	SimTK::Vector_<SimTK::Vec3> allForces(numBodies);
	SimTK::Vector_<SimTK::Vec3> allMoments(numBodies);

	computeJointLoads(*_currentState, parameters, allForces, allMoments);
	//Start Debuging lines
	
	//SimTK::Vec3 zero(0.0,0.0,0.0);
	//load1 = zero;
	//load2 = zero;
	//std::cout << "Load 1: " << load1 << endl;
	//std::cout << "Load 2: " << load2 << endl;
	
	//End Debuging lines

	int numReferenceJoints = _jointReferenceSet.getSize();
	SimTK::Vec3 force;
	SimTK::Vec3 moment;
	SimTK::Vec3 position;
	SimTK::Vec3 forceWeights;
	SimTK::Vec3 momentWeights;

	for(int i=0; i<numReferenceJoints; i++) {
		//for each joint load we're interested in
		if(_jointReferenceSet[i].getIsOn()) {
			//get the correctly transformed force, moment, and position
			getRequestedLoad(_jointReferenceSet[i], allForces, allMoments, force, moment, position);
			// get the weights to multiply to the force and moment term
		
			_jointReferenceSet.get(i).getWeights(*_currentState, momentWeights, forceWeights);

			for(int j=0; j<3; j++) {
				// add the weighted square of the force and moment component
				// to the jointTerm of the cost function
				aCost += pow(force[j], 2)*forceWeights[j];
				aCost += pow(moment[j], 2)*momentWeights[j];
			}
		}

	}
}
/* 
* Compute the joint forces and moments for each joint requested by the user inputs.
* The parameters are used as activations to set the muscle/actuator forces.  The
* resulting joint forces and moments are calculated.

*/
void JointLoadOptimizationTarget::
computeJointLoads(SimTK::State& s, const SimTK::Vector &parameters, SimTK::Vector_<SimTK::Vec3> &forces, SimTK::Vector_<SimTK::Vec3> &moments) const
{
	/*
	LARGE_INTEGER startA;
	LARGE_INTEGER startB;
	LARGE_INTEGER stopA;
	LARGE_INTEGER stopB;
	LARGE_INTEGER frequency;

	QueryPerformanceFrequency(&frequency);
	QueryPerformanceCounter(&startA);
	*/

	double time = s.getTime();
	

	const ForceSet& fs = _model->getForceSet();
	for(int i=0,j=0;i<fs.getSize();i++)  {
         Actuator *act = dynamic_cast<Actuator*>(&fs.get(i));
		 if( act ) {
             //act->overrideForce(s,true);
             act->setOverrideForce(s,parameters[j]*_optimalForce[j]);
		 }
         j++;
    }


	/** define 2 variable length vectors of Vec3 vectors to contain calculated  
	*   forces and moments for all the bodies in the model */
	//int numBodies = _model->getNumBodies();
	//SimTK::Vector_<SimTK::Vec3> allForcesVec(numBodies);
	//SimTK::Vector_<SimTK::Vec3> allMomentsVec(numBodies);


	//_model->getMultibodySystem().realize(s,SimTK::Stage::Acceleration);

	// Timing
	//QueryPerformanceCounter(&startB);
	// End timing

	/* Calculate All joint reaction forces and moments.
	*  Applied to child bodies, expressed in ground frame.  
	*  computeReactions realizes to the acceleration stage internally
	*  so you don't have to call realize in this analysis.*/ 
	_model->getSimbodyEngine().computeReactions(s, forces, moments);

	//Timing
	/*
	QueryPerformanceCounter(&stopB);
	double durationB = (double)(stopB.QuadPart-startB.QuadPart)/(double)frequency.QuadPart;
	std::cout << "computeReactions time = " << (durationB*1.0e3) << " milliseconds" << std::endl;
	*/
	// end Timing

	

	// for timing
	/*
	QueryPerformanceCounter(&stopA);
	double durationA = (double)(stopA.QuadPart-startA.QuadPart)/(double)frequency.QuadPart;
	std::cout << "computeJointLoads time = " << (durationA*1.0e3) << " milliseconds" << std::endl;
	*/
	
}



void JointLoadOptimizationTarget::
getJointLoadsToPrint(SimTK::State& s, const SimTK::Vector &parameters, SimTK::Vector &jointLoads){

	int numBodies = _model->getNumBodies();
	

	SimTK::Vector_<SimTK::Vec3> allForces(numBodies);
	SimTK::Vector_<SimTK::Vec3> allMoments(numBodies);
	computeJointLoads(s, parameters, allForces, allMoments);

	int numReferenceJoints = _jointReferenceSet.getSize();
	// now need to compute the position of each applied joint load.  For now, get in global
	SimTK::Vec3 force;
	SimTK::Vec3 moment;
	SimTK::Vec3 position;

	const JointSet& jointSet = _model->getJointSet();

	for(int i=0; i<numReferenceJoints; i++){
		if(jointSet.contains(_jointReferenceSet[i].getName()) )
		{
			getRequestedLoad(_jointReferenceSet[i], allForces, allMoments, force, moment, position);
			for(int j=0; j<3 ; j++){

				jointLoads[9*i + j ] = position[j]; //not calculating position yet.  fill with zero for now
				jointLoads[9*i + j + 3] = force[j];
				jointLoads[9*i + j + 6] = moment[j];
			}
		}

	}

}

void JointLoadOptimizationTarget::buildLoadDescriptionIndices() {

	// Each joinReactionReference gets an associated set of 3 indices.
	// The first index the is the SimTK bodyset index of the joint's child body needed 
	// to access the loads from the computeReactions results.  The second index identifies
	// which joint load to consider (on child or on parent) and holds that body's index.  
	// The third index identifies the body whoes frame will be used to express the minimized
	// joint loads. This frame matters because each vector component can be weighted independently.
	const JointSet& jointSet = _model->getJointSet();
	const BodySet& bodySet = _model->getBodySet();
	int numReferences = _jointReferenceSet.getSize();
	_loadDescriptionIndices.resize(numReferences);

	for(int i=0; i<numReferences; i++){
		int simbodyIndex = -1;
		int onBodyIndex = -1;
		int frameIndex = -1;
		

		if(jointSet.contains(_jointReferenceSet.get(i).getName()) ) {
			//simbodyIndex = bodySet.getIndex(jointSet.get(jointName).getBody().getName());
			// get the joint and find it's child body.  This body's index in the bodyset is the correct
			// index for extracting joint loads from computeReactions.
			const Joint& joint = jointSet.get(_jointReferenceSet.get(i).getName());
			simbodyIndex = joint.getBody().getIndex();

			// Check if the load should be reported on the child or parent
			if(_jointReferenceSet.get(i).getReceivingBody() == "child"){ onBodyIndex = simbodyIndex; }
			else { onBodyIndex = joint.getParentBody().getIndex(); }

			// Check which body frame the load should be expressed in (child, parent, ground)
			if(_jointReferenceSet.get(i).getReferenceBodyFrame() == "child") { frameIndex = simbodyIndex; }
			else if(_jointReferenceSet.get(i).getReferenceBodyFrame() == "parent") { frameIndex = joint.getParentBody().getIndex(); }
			else { frameIndex = _model->getGroundBody().getIndex(); }

		}
		else {
			std::cout << "Couldn't find a joint called " << _jointReferenceSet.get(i).getName() << " in the model." << std::endl;
		}

		_loadDescriptionIndices[i][0] = simbodyIndex;
		_loadDescriptionIndices[i][1] = onBodyIndex;
		_loadDescriptionIndices[i][2] = frameIndex;
	}

}

void JointLoadOptimizationTarget::
getRequestedLoad(const JointReactionReference &aRef, const SimTK::Vector_<SimTK::Vec3> &allForces, const SimTK::Vector_<SimTK::Vec3> &allMoments, 
	SimTK::Vec3 &force, SimTK::Vec3 &moment, SimTK::Vec3 &position) const 
{

	const JointSet& jointSet = _model->getJointSet();

	
	int simbodyIndex = -1;
	int onBodyIndex = -1;
	int frameIndex = -1;
		

	if(jointSet.contains(aRef.getName()) ) {
		//simbodyIndex = bodySet.getIndex(jointSet.get(jointName).getBody().getName());
		// get the joint and find it's child body.  This body's index in the bodyset is the correct
		// index for extracting joint loads from computeReactions.
		const Joint& joint = jointSet.get(aRef.getName());
		simbodyIndex = joint.getBody().getIndex();
		force = allForces[simbodyIndex];
		moment = allMoments[simbodyIndex]; // raw force and moment are expressed in ground frame
		// get position on child in child frame
		SimTK::Vec3 childLocation(0,0,0);
		joint.getLocation(childLocation);
		// and find it's current location in the ground reference frame
		SimTK::Vec3 childLocationInGlobal(0,0,0);
		_model->getSimbodyEngine().getPosition(*_currentState, joint.getBody(), childLocation,childLocationInGlobal);


		// Check if the load should be reported on the child or parent
		if(aRef.getReceivingBody() == "parent"){ 
			// if it should be applied to parent, find equivalent load on parent
			// at the location in parent
			/*Take reaction load from child and apply on parent*/
			force = -force;
			moment = -moment;
			SimTK::Vec3 parentLocation(0,0,0);
			
			joint.getLocationInParent(parentLocation);
			SimTK::Vec3 parentLocationInGlobal(0,0,0);
			//_model->getSimbodyEngine().getPosition(s_analysis, joint.getBody(), childLocation,childLocationInGlobal);
			_model->getSimbodyEngine().getPosition(*_currentState, joint.getParentBody(), parentLocation, parentLocationInGlobal);

			// define vector from the mobilizer location on the child to the location on the parent
			SimTK::Vec3 translation = parentLocationInGlobal - childLocationInGlobal;
			// find equivalent moment if the load is shifted to the parent loaction
			moment -= translation % force;

			// reset the point of application to the joint location in the parent expressed in ground
			position = parentLocationInGlobal;
		}
		else{
			// set the point of application to the joint laction in the child expressed in ground
			position = childLocationInGlobal;
		}
			

		// Check which body frame the load should be expressed in (child, parent, ground)
		Body* expressedInBody=NULL;
		if(aRef.getReferenceBodyFrame() == "child") 
		{ 
			expressedInBody = &joint.getBody(); 
		}
		else if(aRef.getReferenceBodyFrame() == "parent") 
		{ 
			expressedInBody = &joint.getParentBody(); 
		}
		else 
		{
			expressedInBody = &_model->getGroundBody();
		}
		
		// if the user hasn't selected the loads and position to be expressed in ground, rotate all forces and transform position
		if(expressedInBody->getName() != _model->getGroundBody().getName()) {
			Body& ground = _model->getGroundBody();
			_model->getSimbodyEngine().transform(*_currentState,ground,force,*expressedInBody,force);
			_model->getSimbodyEngine().transform(*_currentState,ground,moment,*expressedInBody,moment);
			_model->getSimbodyEngine().transformPosition(*_currentState,ground,position,*expressedInBody,position);
		}
		// done transforming forces, moments, and position

	}
	else {
		std::cout << "Couldn't find a joint called " <<aRef.getName() << " in the model." << std::endl;
		force.setToZero();
		moment.setToZero();
		position.setToZero();
	}

}