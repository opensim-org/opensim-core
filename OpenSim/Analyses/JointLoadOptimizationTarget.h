#ifndef _JointLoadOptimizationTarget_h_
#define _JointLoadOptimizationTarget_h_
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

//=============================================================================
// INCLUDES
//=============================================================================
#include "osimAnalysesDLL.h"
#include "OpenSim/Common/Array.h"
#include <OpenSim/Common/GCVSplineSet.h>
#include <OpenSim/Simulation/JointReactionReference.h>
#include <OpenSim/Simulation/SimbodyEngine/SimbodyEngine.h>
#include "SimTKsimbody.h"
#include <simmath/Optimizer.h>

//=============================================================================
//=============================================================================
namespace OpenSim { 

/**
 * This class provides an interface specification for static optimization Objective Function.
 * This objective function includes terms for minimizing or maximizing joint joint force.
 *
 * @author Jeff Reinbolt
 */
class OSIMANALYSES_API JointLoadOptimizationTarget : public SimTK::OptimizerSystem
{
//=============================================================================
// DATA
//=============================================================================
public:
	/** Smallest allowable perturbation size for computing derivatives. */
	static const double SMALLDX;
	/** . */

	
private:

	/** Work model. */
	Model *_model;
	/** Current state */
	SimTK::State* _currentState;
	/** Reciprocal of actuator area squared. */
	Array<double> _recipAreaSquared;
	/** Reciprocal of optimal force squared accounting for force-length curve if actuator is a muscle. */
	Array<double> _recipOptForceSquared;
	/** Optimal force accounting for force-length curve if desired and if actuator is a muscle. */
	Array<double> _optimalForce;
	
	SimTK::Matrix _constraintMatrix;
	SimTK::Vector _constraintVector;

	const Storage *_statesStore;
	GCVSplineSet _statesSplineSet;

	//SimTK::Vector_<SimTK::Vec3> jointLoad1; // point, force, then moment
	//SimTK::Vector_<SimTK::Vec3> jointLoad2;

protected:
	double _activationExponent;
	bool   _useMusclePhysiology;
	/** Perturbation size for computing numerical derivatives. */
	Array<double> _dx;
	Array<int> _accelerationIndices;

	// Added by Matt DeMers to optimize joint forces and moments
	//---------------------------------------------------------
	Set<JointReactionReference> &_jointReferenceSet;
	double _objectiveScaleFactor;
	double _jointTermScaleFactor;
	//Array<int> _jointChildIndices;
	SimTK::Vector_<SimTK::Vec3> _loadDescriptionIndices; //SimTK joint set index, on body index, body frame index

//=============================================================================
// METHODS
//=============================================================================
public:
	JointLoadOptimizationTarget(const SimTK::State& s, Model *aModel,int aNX,int aNC, Set<JointReactionReference>& aRefSet, const bool useMusclePhysiology=true);

	// SET AND GET
	void setModel(Model& aModel);
	void setStatesStore(const Storage *aStatesStore);
	void setStatesSplineSet(GCVSplineSet aStatesSplineSet);
	void setNumParams(const int aNP);
	void setNumConstraints(const int aNC);
	void setDX(double aVal);
	void setDX(int aIndex,double aVal);
	double getDX(int aIndex);
	double* getDXArray();
	void getActuation(SimTK::State& s, const SimTK::Vector &parameters, SimTK::Vector &forces);
	void setActivationExponent(double aActivationExponent) { _activationExponent=aActivationExponent; }
	double getActivationExponent() const { return _activationExponent; }
	void setCurrentState(SimTK::State* state) { _currentState = state; }
	const SimTK::State* getCurrentState() const { return _currentState; }
	void setJointReferences(Set<JointReactionReference>& aSet) { _jointReferenceSet = aSet; }
	void getJointLoadsToPrint(SimTK::State& s, const SimTK::Vector &parameters, SimTK::Vector &jointLoads);
	void setObjectiveScaleFactor(double aFactor) { _objectiveScaleFactor = aFactor; }
	void setJointTermScaleFactor(double aFactor) { _jointTermScaleFactor = aFactor; }

	// UTILITY
	void validatePerturbationSize(double &aSize);

	virtual void printPerformance(SimTK::State& s, double *x);

	void computeActuatorAreas(const SimTK::State& s);

	static int
		CentralDifferencesConstraint(const JointLoadOptimizationTarget *aTarget,
		double *dx,const SimTK::Vector &x,SimTK::Matrix &jacobian);
	static int
		CentralDifferences(const JointLoadOptimizationTarget *aTarget,
		double *dx,const SimTK::Vector &x,SimTK::Vector &dpdx);

	bool prepareToOptimize(SimTK::State& s, double *x);

	//--------------------------------------------------------------------------
	// REQUIRED OPTIMIZATION TARGET METHODS
	//--------------------------------------------------------------------------
	int objectiveFunc(const SimTK::Vector &x, const bool new_coefficients, SimTK::Real& rP) const;
	int gradientFunc(const SimTK::Vector &x, const bool new_coefficients, SimTK::Vector &gradient) const;
	int constraintFunc(const SimTK::Vector &x, const bool new_coefficients, SimTK::Vector &constraints) const;
	int constraintJacobian(const SimTK::Vector &x, const bool new_coefficients, SimTK::Matrix &jac) const;

private:
	void computeConstraintVector(SimTK::State& s, const SimTK::Vector &x, SimTK::Vector &c) const;
	void computeAcceleration(SimTK::State& s, const SimTK::Vector &aF,SimTK::Vector &rAccel) const;
	void computeJointLoads(SimTK::State& s, const SimTK::Vector &parameters, SimTK::Vector_<SimTK::Vec3> &forces, SimTK::Vector_<SimTK::Vec3> &moments) const;
	void cumulativeTime(double &aTime, double aIncrement);
	void computeJointLoadsCost(SimTK::State& s, const SimTK::Vector &parameters, double &aCost) const;
	void buildLoadDescriptionIndices();
	void getRequestedLoad(const JointReactionReference &aRef, const SimTK::Vector_<SimTK::Vec3> &allForces, const SimTK::Vector_<SimTK::Vec3> &allMoments, SimTK::Vec3 &force, SimTK::Vec3 &moment, SimTK::Vec3 &position) const;
};

}; //namespace

#endif // _JointLoadOptimizationTarget_h_
