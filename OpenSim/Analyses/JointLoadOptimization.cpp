// JointLoadOptimization.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Matt DeMers
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/* Copyright (c)  2006 Stanford University
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
#include <iostream>
#include <string>
#include <OpenSim/Common/IO.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/SimbodyEngine/SimbodyEngine.h>
#include <OpenSim/Simulation/Model/BodySet.h>
#include <OpenSim/Simulation/Model/CoordinateSet.h>
#include <OpenSim/Simulation/Model/ForceSet.h>
#include <OpenSim/Simulation/Model/Muscle.h>
#include <OpenSim/Common/GCVSplineSet.h>
#include <OpenSim/Actuators/CoordinateActuator.h>
#include <OpenSim/Simulation/Control/ControlSet.h>
#include <SimTKmath.h>
#include <SimTKlapack.h>
#include "JointLoadOptimization.h"
#include "JointLoadOptimizationTarget.h"


using namespace OpenSim;
using namespace std;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
JointLoadOptimization::~JointLoadOptimization()
{
	deleteStorage();
	delete _modelWorkingCopy;
	if(_ownsForceSet) delete _forceSet;
}
//_____________________________________________________________________________
/**
 */
JointLoadOptimization::JointLoadOptimization(Model *aModel) :
	Analysis(aModel),
	_useModelForceSet(_useModelForceSetProp.getValueBool()),
	_activationExponent(_activationExponentProp.getValueDbl()),
	_useMusclePhysiology(_useMusclePhysiologyProp.getValueBool()),
	_optimizerAlgorithm(_optimizerAlgorithmProp.getValueStr()),
	_convergenceCriterion(_convergenceCriterionProp.getValueDbl()),
	_maxIterations(_maxIterationsProp.getValueInt()),
	_optimizerDX(_optimizerDXProp.getValueDbl()),
	_printLevel(_printLevelProp.getValueInt()),
	_objectiveScaleFactor(_objectiveScaleFactorProp.getValueDbl()),
	_jointTermScale(_jointTermScaleProp.getValueDbl()),
	_jointTaskSetProp(PropertyObj("", Set<JointReactionReference>())),
	_jointTaskSet((Set<JointReactionReference>&)_jointTaskSetProp.getValueObj()),
	_modelWorkingCopy(NULL),
	_numCoordinateActuators(0)
{
	setupProperties();
	setNull();

	if(aModel) setModel(*aModel);
	else allocateStorage();
}
// Copy constrctor and virtual copy 
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 */
JointLoadOptimization::JointLoadOptimization(const JointLoadOptimization &aJointLoadOptimization):
	Analysis(aJointLoadOptimization),
	_useModelForceSet(_useModelForceSetProp.getValueBool()),
	_activationExponent(_activationExponentProp.getValueDbl()),
	_useMusclePhysiology(_useMusclePhysiologyProp.getValueBool()),
	_optimizerAlgorithm(_optimizerAlgorithmProp.getValueStr()),
	_convergenceCriterion(_convergenceCriterionProp.getValueDbl()),
	_maxIterations(_maxIterationsProp.getValueInt()),
	_optimizerDX(_optimizerDXProp.getValueDbl()),
	_jointTermScale(_jointTermScaleProp.getValueDbl()),
	_printLevel(_printLevelProp.getValueInt()),
	_objectiveScaleFactor(_objectiveScaleFactorProp.getValueDbl()),
	_jointTaskSetProp(PropertyObj("", Set<JointReactionReference>())),
	_jointTaskSet((Set<JointReactionReference>&)_jointTaskSetProp.getValueObj()),
	_modelWorkingCopy(NULL),
	_numCoordinateActuators(aJointLoadOptimization._numCoordinateActuators)
{
	setupProperties();
	setNull();
	// COPY TYPE AND NAME
	*this = aJointLoadOptimization;
}
//_____________________________________________________________________________
/**
 * Clone
 *
 */
Object* JointLoadOptimization::copy() const
{
	JointLoadOptimization *object = new JointLoadOptimization(*this);
	return(object);

}
//=============================================================================
// OPERATORS
//=============================================================================
//-----------------------------------------------------------------------------
// ASSIGNMENT
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Assign this object to the values of another.
 *
 * @return Reference to this object.
 */
JointLoadOptimization& JointLoadOptimization::
operator=(const JointLoadOptimization &aJointLoadOptimization)
{
	// BASE CLASS
	Analysis::operator=(aJointLoadOptimization);

	_modelWorkingCopy = aJointLoadOptimization._modelWorkingCopy;
	_numCoordinateActuators = aJointLoadOptimization._numCoordinateActuators;
	_useModelForceSet = aJointLoadOptimization._useModelForceSet;
	_useMusclePhysiology=aJointLoadOptimization._useMusclePhysiology;
	_activationExponent=aJointLoadOptimization._activationExponent;
	_optimizerAlgorithm=aJointLoadOptimization._optimizerAlgorithm;
	_convergenceCriterion=aJointLoadOptimization._convergenceCriterion;
	_maxIterations=aJointLoadOptimization._maxIterations;
	_optimizerDX=aJointLoadOptimization._optimizerDX;
	_printLevel = aJointLoadOptimization._printLevel;
	_objectiveScaleFactor = aJointLoadOptimization._objectiveScaleFactor;
	_jointTermScale = aJointLoadOptimization._jointTermScale;
	_jointTaskSet = aJointLoadOptimization._jointTaskSet;

	return(*this);
}

//_____________________________________________________________________________
/**
 * SetNull().
 */
void JointLoadOptimization::
setNull()
{
	

	// OTHER VARIABLES
	_useModelForceSet = true;
	_activationStorage = NULL;
	_forceStorage = NULL;
	_jointForceStorage = NULL;
	_ownsForceSet = false;
	_forceSet = NULL;
	_activationExponent=2;
	_optimizerAlgorithm="ipopt";
	_convergenceCriterion = 10e-004;
	_optimizerDX = 0.0001;
	_maxIterations = 2000;
	_printLevel=0;
	_objectiveScaleFactor = 1.0;
	_jointTermScale = 1.0;
	_useMusclePhysiology=true;
	_numCoordinateActuators = 0;
	

	//Array<double> zeros = {0,0,0};
	setType("JointLoadOptimization");
	setName("JointLoadOptimization");
}
//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void JointLoadOptimization::
setupProperties()
{
	_useModelForceSetProp.setComment("If true, the model's own force set will be used in the static optimization computation.  "
													"Otherwise, inverse dynamics for coordinate actuators will be computed for all unconstrained degrees of freedom.");
	_useModelForceSetProp.setName("use_model_force_set");
	_propertySet.append(&_useModelForceSetProp);

	_activationExponentProp.setComment(
		"A double indicating the exponent to raise activations to when solving static optimization.  ");
	_activationExponentProp.setName("activation_exponent");
	_propertySet.append(&_activationExponentProp);

	_optimizerAlgorithmProp.setComment(
		"A string identifying which optimizer algorithm to use.  Default is ipopt (interior point optimization)."
		"List of options: bestavailable, ipopt, lbfgs, lbfgsb, cfsqp.");
	_optimizerAlgorithmProp.setName("optimizer_algorithm");
	_propertySet.append(&_optimizerAlgorithmProp);

	_convergenceCriterionProp.setComment(
		"A double for setting the convergence criterion to use in the optimizer.  ");
	_convergenceCriterionProp.setName("optimizer_convergence_criterion");
	_propertySet.append(&_convergenceCriterionProp);

	_maxIterationsProp.setComment(
		"An integer for setting the maximum number of iterations the optimizer can use at each time.  ");
	_maxIterationsProp.setName("optimizer_max_iterations");
	_propertySet.append(&_maxIterationsProp);

	_optimizerDXProp.setComment(
		"A double indicating the corseness of the optimizer's parameter control.  In this case, a smaller "
		"number allows the optimizer finer control of the muscle/actuator activations when minimizing the cost funtion.");
	_optimizerDXProp.setName("optimizer_dx_resolution");
	_propertySet.append(&_optimizerDXProp);

	_objectiveScaleFactorProp.setComment(
		"Scalar that multiplies the entire objective function.  Used to scale the size of the cost function."
		"Default is 1.0 ");
	_objectiveScaleFactorProp.setName("objective_scale_factor");
	_propertySet.append(&_objectiveScaleFactorProp);

	_jointTermScaleProp.setComment(
		"Scalar that multiplies all joint load contributions to the objective function.  It does not multiply"
		" the contribution from muscle activations.  Used to scale the relative importance minimizing joint "
		"loads vs activations.  Default is 1.0 ");
	_jointTermScaleProp.setName("scale_joint_term");
	_propertySet.append(&_jointTermScaleProp);

	_printLevelProp.setComment(
		"An integer (0 through 4) used for debuging the optimizer.  0 prints nothing, 3 prints optimization results summary, 4 prints all iterations");
	_printLevelProp.setName("optimizer_print_level");
	_propertySet.append(&_printLevelProp);

	
	_useMusclePhysiologyProp.setComment(
		"If true muscle force-length curve is observed while running optimization.");
	_useMusclePhysiologyProp.setName("use_muscle_physiology");
	_propertySet.append(&_useMusclePhysiologyProp);

	_jointTaskSetProp.setComment(
		"A set of joint reaction references used to define which joint loads to minimize and"
		"their relative weights in the objective function.");
	_jointTaskSet.setType("JointTaskSet");
	_jointTaskSetProp.setName("JointTaskSet");
	_propertySet.append(&_jointTaskSetProp);






}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Find the joint set indicies for all joints we plan to optimize.
 */
Array<int> JointLoadOptimization::
findJointsToOptimize()
{
	int numTerms = _jointTaskSet.getSize();
	Array<int> _jointIndices;
	_jointIndices.setSize(numTerms);
	/*
	const JointSet& jointSet = _model->getJointSet();
	const BodySet& bodySet = _model->getBodySet();
	int numJoints = jointSet.getSize();
	for(int i=0; i<numTerms; i++){
		std::string jointName = _jointNames[i];
		for(int j=0; j<numJoints; j++){
			Joint& joint = jointSet.get(j);
			if(jointName == joint.getName()){
				_jointIndices[i]= bodySet.getIndex(joint.getBody().getName());
			}
		}

	}
	*/
	for(int i=0; i<numTerms; i++){
		std::string jointName = _jointTaskSet.get(i).getName();
		_jointIndices[i] = findBodyIndexForJoint(jointName);
	}

	return _jointIndices;
}
//_____________________________________________________________________________
/**
 * Find the SimTK body index associated with each joint load.  In simtk land,
 * joints and joint loads are associated with the child body that the joint
 * connects to the existing tree structure.  We'll use this when collecting
 * joint loads from computeReactions or calcMobilizerReactionForces.  These
 * functions conform to SimTK bodyset indexing, not OpenSim's JointSet indexing.
 */
int JointLoadOptimization::
findBodyIndexForJoint(std::string &jointName)
{
	const JointSet& jointSet = _model->getJointSet();
	const BodySet& bodySet = _model->getBodySet();
	int simbodyIndex = -1;

	if(jointSet.contains(jointName) ) {
		//simbodyIndex = bodySet.getIndex(jointSet.get(jointName).getBody().getName());
		// get the joint and find it's child body.  This body's index in the bodyset is the correct
		// index for extracting joint loads from computeReactions.
		simbodyIndex = jointSet.get(jointName).getBody().getIndex();
	}
	else {
		std::cout << "Couldn't find a joint called " << jointName << " in the model." << std::endl;
	}

	return simbodyIndex;
}

//_____________________________________________________________________________
/**
 * Construct a description for the static optimization files.
 */
void JointLoadOptimization::
constructDescription()
{
	string descrip = "This file contains results from a Joint Load Optimization.\n\n";
	setDescription(descrip);
}

//_____________________________________________________________________________
/**
 * Construct column labels for the static optimization files.
 */
void JointLoadOptimization::
constructColumnLabels()
{
	Array<string> labels;
	labels.append("time");
	if(_model) 
		for (int i=0; i < _forceSet->getSize(); i++) labels.append(_forceSet->get(i).getName());
	setColumnLabels(labels);
}

//_____________________________________________________________________________
/**
 * Construct column labels for the file containing joint forces.
 */
void JointLoadOptimization::constructJointForceColumnLabels()
{
	Array<string> labels;
	labels.append("time");
	if(_model) 
	{
		const JointSet& jointSet = _model->getJointSet();
		for (int i=0; i < _jointTaskSet.getSize(); i++) {
			if (jointSet.contains(_jointTaskSet[i].getName() ) )
			{
				std::string jointName = _jointTaskSet.get(i).getName();
				std::string onBodyName = "";
				if(_jointTaskSet[i].getReceivingBody() == "child")
				{
					onBodyName = jointSet.get(jointName).getBody().getName();
				}
				else if (_jointTaskSet[i].getReceivingBody() == "parent")
				{
					onBodyName = jointSet.get(jointName).getParentBody().getName();
				}
				
				std::string inFrameName = "";

				if(_jointTaskSet[i].getReferenceBodyFrame() == "child")
				{
					inFrameName = jointSet.get(jointName).getBody().getName();
				}
				else if (_jointTaskSet[i].getReferenceBodyFrame() == "parent")
				{
					inFrameName = jointSet.get(jointName).getParentBody().getName();
				}
				else 
				{
					inFrameName = _model->getGroundBody().getName();
				}

				std::string labelRoot = jointName + "_on_" + onBodyName + "_in_" + inFrameName;
				labels.append(labelRoot + "_px");
				labels.append(labelRoot + "_py");
				labels.append(labelRoot + "_pz");
				labels.append(labelRoot + "_fx");
				labels.append(labelRoot + "_fy");
				labels.append(labelRoot + "_fz");
				labels.append(labelRoot + "_tx");
				labels.append(labelRoot + "_ty");
				labels.append(labelRoot + "_tz");
			}
			
		}
	}
	_jointForceLabels = labels;
}
//_____________________________________________________________________________
/**
 * Allocate storage for the static optimization.
 */
void JointLoadOptimization::
allocateStorage()
{
	_activationStorage = new Storage(1000,"activations");
	_activationStorage->setDescription(getDescription());
	_activationStorage->setColumnLabels(getColumnLabels());

	_forceStorage = new Storage(1000,"actuator forces");
	_forceStorage->setDescription(getDescription());
	_forceStorage->setColumnLabels(getColumnLabels());

	_jointForceStorage = new Storage(1000, "joint loads");
	_jointForceStorage->setDescription(getDescription());
	_jointForceStorage->setColumnLabels(_jointForceLabels);

}


//=============================================================================
// DESTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Delete storage objects.
 */
void JointLoadOptimization::
deleteStorage()
{
	delete _activationStorage; _activationStorage = NULL;
	delete _forceStorage; _forceStorage = NULL;
	delete _jointForceStorage; _jointForceStorage = NULL;
}

//=============================================================================
// GET AND SET
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the model for which the static optimization is to be computed.
 *
 * @param aModel Model pointer
 */
void JointLoadOptimization::
setModel(Model& aModel)
{
	Analysis::setModel(aModel);
	//SimTK::State& s = aModel->getMultibodySystem()->updDefaultState();
}

//-----------------------------------------------------------------------------
// STORAGE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the activation storage.
 *
 * @return Activation storage.
 */
Storage* JointLoadOptimization::
getActivationStorage()
{
	return(_activationStorage);
}
//_____________________________________________________________________________
/**
 * Get the force storage.
 *
 * @return Force storage.
 */
Storage* JointLoadOptimization::
getForceStorage()
{
	return(_forceStorage);
}

//_____________________________________________________________________________
/**
 * Get the joint force storage.
 *
 * @return joint Force storage.
 */
Storage* JointLoadOptimization::
getJointForceStorage()
{
	return(_jointForceStorage);
}

//-----------------------------------------------------------------------------
// STORAGE CAPACITY
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the capacity increments of all storage instances.
 *
 * @param aIncrement Increment by which storage capacities will be increased
 * when storage capacities run out.
 */
void JointLoadOptimization::
setStorageCapacityIncrements(int aIncrement)
{
	_activationStorage->setCapacityIncrement(aIncrement);
	_forceStorage->setCapacityIncrement(aIncrement);
	_jointForceStorage->setCapacityIncrement(aIncrement);
}

//=============================================================================
// ANALYSIS
//=============================================================================
//_____________________________________________________________________________
/**
 * Record the results.
 */
int JointLoadOptimization::
record(const SimTK::State& s)
{
	if(!_modelWorkingCopy) return -1;

	// Set model Q's and U's
	SimTK::State& sWorkingCopy = _modelWorkingCopy->updMultibodySystem().updDefaultState();

	sWorkingCopy.setTime(s.getTime());
	sWorkingCopy.setQ(s.getQ());
	sWorkingCopy.setU(s.getU());
	_modelWorkingCopy->computeEquilibriumForAuxiliaryStates(sWorkingCopy);

    const Set<Actuator>& fs = _modelWorkingCopy->getActuators();

	int na = fs.getSize();
	int nacc = _accelerationIndices.getSize();

	// IPOPT
	//_optimizerDX = 0.0001;
	//_optimizerAlgorithm = "ipopt";
	//_printLevel = 2;
	//_convergenceCriterion = 1e-004;
	//_maxIterations = 2000;

	// Optimization target
	_modelWorkingCopy->setAllControllersEnabled(false);
	JointLoadOptimizationTarget target(sWorkingCopy,_modelWorkingCopy,na,nacc, _jointTaskSet, _useMusclePhysiology);
	target.setStatesStore(_statesStore);
	target.setStatesSplineSet(_statesSplineSet);
	target.setActivationExponent(_activationExponent);
	target.setDX(_optimizerDX);
	target.setObjectiveScaleFactor(_objectiveScaleFactor);
	target.setJointTermScaleFactor(_jointTermScale);
	

	// Pick optimizer algorithm (default is ipopt)
	// List of options: bestavailable, ipopt, lbfgs, lbfgsb, cfsqp
	SimTK::OptimizerAlgorithm algorithm = SimTK::InteriorPoint;
	//SimTK::OptimizerAlgorithm algorithm = SimTK::CFSQP;
	std::transform(_optimizerAlgorithm.begin(),_optimizerAlgorithm.end(),_optimizerAlgorithm.begin(), ::tolower);

	if(_optimizerAlgorithm == "bestavailable") { algorithm = SimTK::BestAvailiable; }
	else if (_optimizerAlgorithm == "ipopt" || _optimizerAlgorithm == "interiorpoint")
	{ algorithm = SimTK::InteriorPoint; }
	else if (_optimizerAlgorithm == "lbfgs") { algorithm = SimTK::LBFGS; }
	else if (_optimizerAlgorithm == "lbfgsb") { algorithm = SimTK::LBFGSB; }
	else if (_optimizerAlgorithm == "cfsqp") { algorithm = SimTK::CFSQP; }


	// Optimizer
	SimTK::Optimizer *optimizer = new SimTK::Optimizer(target, algorithm);

	// Optimizer options
	//cout<<"\nSetting optimizer print level to "<<_printLevel<<".\n";
	optimizer->setDiagnosticsLevel(_printLevel);
	//cout<<"Setting optimizer convergence criterion to "<<_convergenceCriterion<<".\n";
	optimizer->setConvergenceTolerance(_convergenceCriterion);
	//cout<<"Setting optimizer maximum iterations to "<<_maxIterations<<".\n";
	optimizer->setMaxIterations(_maxIterations);
	optimizer->useNumericalGradient(true); // false for static opt, but must be use numerical gradient if minimizing joint forces
	optimizer->useNumericalJacobian(false);
	if(algorithm == SimTK::InteriorPoint) {
		// Some IPOPT-specific settings
		optimizer->setLimitedMemoryHistory(500); // works well for our small systems
		optimizer->setAdvancedBoolOption("warm_start",true);
		optimizer->setAdvancedRealOption("obj_scaling_factor",1);
		optimizer->setAdvancedRealOption("nlp_scaling_max_gradient",1);
	}

	// Parameter bounds
	SimTK::Vector lowerBounds(na), upperBounds(na);
	for(int i=0,j=0;i<fs.getSize();i++) {
		Actuator& act = fs.get(i);
		lowerBounds(j) = act.getMinControl();
	    upperBounds(j) = act.getMaxControl();
        j++;
	}
	
	target.setParameterLimits(lowerBounds, upperBounds);

	_parameters = 0; // Set initial guess to zeros

	// Static optimization
	_modelWorkingCopy->getMultibodySystem().realize(sWorkingCopy,SimTK::Stage::Velocity);
	target.prepareToOptimize(sWorkingCopy, &_parameters[0]);

	//LARGE_INTEGER start;
	//LARGE_INTEGER stop;
	//LARGE_INTEGER frequency;

	//QueryPerformanceFrequency(&frequency);
	//QueryPerformanceCounter(&start);

	try {
		target.setCurrentState( &sWorkingCopy );
		optimizer->optimize(_parameters);
	}
	catch (const SimTK::Exception::Base &ex) {
		cout << ex.getMessage() << endl;
		cout << "OPTIMIZATION FAILED..." << endl;
		cout << endl;
		cout << "JointLoadOptimization.record:  WARN- The optimizer could not find a solution at time = " << s.getTime() << endl;
		cout << endl;

		double tolBounds = 1e-1;
		bool weakModel = false;
		string msgWeak = "The model appears too weak for static optimization.\nTry increasing the strength and/or range of the following force(s):\n";
		for(int a=0;a<na;a++) {
			Actuator* act = dynamic_cast<Actuator*>(&_forceSet->get(a));
            if( act ) {
			    Muscle*  mus = dynamic_cast<Muscle*>(&_forceSet->get(a));
 			    if(mus==NULL) {
			    	if(_parameters(a) < (lowerBounds(a)+tolBounds)) {
			    		msgWeak += "   ";
			    		msgWeak += act->getName();
			    		msgWeak += " approaching lower bound of ";
			    		ostringstream oLower;
			    		oLower << lowerBounds(a);
			    		msgWeak += oLower.str();
			    		msgWeak += "\n";
			    		weakModel = true;
			    	} else if(_parameters(a) > (upperBounds(a)-tolBounds)) {
			    		msgWeak += "   ";
			    		msgWeak += act->getName();
			    		msgWeak += " approaching upper bound of ";
			    		ostringstream oUpper;
			    		oUpper << upperBounds(a);
			    		msgWeak += oUpper.str();
			    		msgWeak += "\n";
			    		weakModel = true;
			    	} 
			    } else {
			    	if(_parameters(a) > (upperBounds(a)-tolBounds)) {
			    		msgWeak += "   ";
			    		msgWeak += mus->getName();
			    		msgWeak += " approaching upper bound of ";
			    		ostringstream o;
			    		o << upperBounds(a);
			    		msgWeak += o.str();
			    		msgWeak += "\n";
			    		weakModel = true;
			    	}
			    }
            }
		}
		if(weakModel) cout << msgWeak << endl;

		if(!weakModel) {
			double tolConstraints = 1e-6;
			bool incompleteModel = false;
			string msgIncomplete = "The model appears unsuitable for static optimization.\nTry appending the model with additional force(s) or locking joint(s) to reduce the following acceleration constraint violation(s):\n";
			SimTK::Vector constraints;
			target.constraintFunc(_parameters,true,constraints);
			const CoordinateSet& coordSet = _modelWorkingCopy->getCoordinateSet();
			for(int acc=0;acc<nacc;acc++) {
				if(fabs(constraints(acc)) > tolConstraints) {
					const Coordinate& coord = coordSet.get(_accelerationIndices[acc]);
					msgIncomplete += "   ";
					msgIncomplete += coord.getName();
					msgIncomplete += ": constraint violation = ";
					ostringstream o;
					o << constraints(acc);
					msgIncomplete += o.str();
					msgIncomplete += "\n";
					incompleteModel = true;
				}
			}
			if(incompleteModel) cout << msgIncomplete << endl;
		}
	}

	//QueryPerformanceCounter(&stop);
	//double duration = (double)(stop.QuadPart-start.QuadPart)/(double)frequency.QuadPart;
	//cout << "optimizer time = " << (duration*1.0e3) << " milliseconds" << endl;

	target.printPerformance(sWorkingCopy, &_parameters[0]);

	// save the ACTIVATION results to storage
	_activationStorage->append(sWorkingCopy.getTime(),na,&_parameters[0]);

	// save the ACTUATOR FORCES results to storage
	SimTK::Vector actuatorForces(na);
	target.getActuation(const_cast<SimTK::State&>(sWorkingCopy), _parameters, actuatorForces);

	_forceStorage->append(sWorkingCopy.getTime(),na, &actuatorForces[0]);

	// save the Joint Load results to storage
	SimTK::Vector jointLoads(_optimizedJointsIndices.getSize()*9);
	target.getJointLoadsToPrint(sWorkingCopy, _parameters, jointLoads);


	_jointForceStorage->append(sWorkingCopy.getTime(), _optimizedJointsIndices.getSize()*9, &jointLoads[0]);

	return 0;
}
//_____________________________________________________________________________
/**
 * This method is called at the beginning of an analysis so that any
 * necessary initializations may be performed.
 *
 * This method is meant to be called at the begining of an integration 
 *
 * @param s Current state .
 *
 * @return -1 on error, 0 otherwise.
 */
int JointLoadOptimization::
begin(SimTK::State& s )
{
	/*
	for(int i=0; i<_jointTaskSet.getSize(); i++){
		std::cout << "JointReactionReference " << i << ": " << _jointTaskSet.get(i).getName() << std::endl;
	}
	if(!proceed()) return(0);

	std::cout << "_testReference :" << _testReference.getName() << std::endl;
	*/

	// Make a working copy of the model
	delete _modelWorkingCopy;
	_modelWorkingCopy =  dynamic_cast<Model*>(_model->copy());
	//_modelWorkingCopy = _model->clone();
	_modelWorkingCopy->initSystem();

	// Replace model force set with only generalized forces
	if(_model) {
		SimTK::State& sWorkingCopyTemp = _modelWorkingCopy->updMultibodySystem().updDefaultState();
		// Update the _forceSet we'll be computing inverse dynamics for
		if(_ownsForceSet) delete _forceSet;
		if(_useModelForceSet) {
			// Set pointer to model's internal force set
			_forceSet = &_modelWorkingCopy->updForceSet();
			_ownsForceSet = false;
		} else {
			ForceSet& as = _modelWorkingCopy->updForceSet();
			// Keep a copy of forces that are not muscles to restore them back.
			ForceSet* saveForces = (ForceSet*)as.copy();
			// Generate an force set consisting of a coordinate actuator for every unconstrained degree of freedom
			_forceSet = CoordinateActuator::CreateForceSetOfCoordinateActuatorsForModel(sWorkingCopyTemp,*_modelWorkingCopy,1,false);
			_ownsForceSet = false;
			_modelWorkingCopy->setAllControllersEnabled(false);
			_numCoordinateActuators = _forceSet->getSize();
			// Copy whatever forces that are not muscles back into the model
			
			for(int i=0; i<saveForces->getSize(); i++){
				const Force& f=saveForces->get(i);
				if ((dynamic_cast<const Muscle*>(&saveForces->get(i)))==NULL)
					as.append((Force*)saveForces->get(i).copy());
			}
		}

		SimTK::State& sWorkingCopy = _modelWorkingCopy->initSystem();

		// Set modeling options for Actuators to be overriden
		for(int i=0,j=0; i<_forceSet->getSize(); i++) {
			Actuator* act = dynamic_cast<Actuator*>(&_forceSet->get(i));
			if( act ) {
				act->overrideForce(sWorkingCopy,true);
			}
		}

		sWorkingCopy.setQ(s.getQ());
		sWorkingCopy.setU(s.getU());
		sWorkingCopy.setZ(s.getZ());
		_modelWorkingCopy->getMultibodySystem().realize(s,SimTK::Stage::Velocity);
		_modelWorkingCopy->computeEquilibriumForAuxiliaryStates(sWorkingCopy);
		// Gather indices into speed set corresponding to the unconstrained degrees of freedom (for which we will set acceleration constraints)
		_accelerationIndices.setSize(0);
		const CoordinateSet& coordSet = _model->getCoordinateSet();
		for(int i=0; i<coordSet.getSize(); i++) {
			const Coordinate& coord = coordSet.get(i);
			if(!coord.isConstrained(sWorkingCopy)) {
				_accelerationIndices.append(i);
			}
		}

		int na = _forceSet->getSize();
		int nacc = _accelerationIndices.getSize();

		if(na < nacc) 
			throw(Exception("StaticOptimization: ERROR- overconstrained system -- need at least as many forces as there are degrees of freedom.\n"));

		_parameters.resize(na);
		_parameters = 0;

		// cache the indices of joints loads we want to optimize
		_optimizedJointsIndices = findJointsToOptimize();
	}

	

	_statesSplineSet=GCVSplineSet(5,_statesStore);

	// DESCRIPTION AND LABELS
	constructDescription();
	constructColumnLabels();
	constructJointForceColumnLabels();

	deleteStorage();
	allocateStorage();

	// RESET STORAGE
	_activationStorage->reset(s.getTime());
	_forceStorage->reset(s.getTime());
	_jointForceStorage->reset(s.getTime());

	// RECORD
	int status = 0;
	if(_activationStorage->getSize()<=0 && _forceStorage->getSize()<=0 && _jointForceStorage->getSize()<=0) {
		status = record(s);
		if(_printLevel > 0 ){

			const Set<Actuator>& fs = _modelWorkingCopy->getActuators();
			for(int k=0;k<fs.getSize();k++) {
				Actuator& act = fs.get(k);
				cout << "Bounds for " << act.getName() << ": " << act.getMinControl()<< " to "<< act.getMaxControl() << endl;
			}
		}
	}

	return(status);
}
//_____________________________________________________________________________
/**
 * This method is called to perform the analysis.  It can be called during
 * the execution of a forward integrations or after the integration by
 * feeding it the necessary data.
 *
 * This method should be overriden in derived classes.  It is
 * included here so that the derived class will not have to implement it if
 * it is not necessary.
 *
 * @param s Current state .
 *
 * @return -1 on error, 0 otherwise.
 */
int JointLoadOptimization::
step(const SimTK::State& s, int stepNumber )
{
	if(!proceed(stepNumber)) return(0);

	record(s);

	return(0);
}
//_____________________________________________________________________________
/**
 * This method is called at the end of an analysis so that any
 * necessary finalizations may be performed.
 *
 * @param s Current state 
 *
 * @return -1 on error, 0 otherwise.
 */
int JointLoadOptimization::
end( SimTK::State& s )
{
	if(!proceed()) return(0);

	record(s);

	return(0);
}


//=============================================================================
// IO
//=============================================================================
//_____________________________________________________________________________
/**
 * Print results.
 * 
 * The file names are constructed as
 * aDir + "/" + aBaseName + "_" + ComponentName + aExtension
 *
 * @param aDir Directory in which the results reside.
 * @param aBaseName Base file name.
 * @param aDT Desired time interval between adjacent storage vectors.  Linear
 * interpolation is used to print the data out at the desired interval.
 * @param aExtension File extension.
 *
 * @return 0 on success, -1 on error.
 */
int JointLoadOptimization::
printResults(const string &aBaseName,const string &aDir,double aDT,
				 const string &aExtension)
{
	// ACTIVATIONS
	Storage::printResult(_activationStorage,aBaseName+"_"+getName()+"_activation",aDir,aDT,aExtension);

	// MUSCLE FORCES
	Storage::printResult(_forceStorage,aBaseName+"_"+getName()+"_force",aDir,aDT,aExtension);

	// JOINT FORCES
	//_jointForceStorage->scaleTime(_model->getTimeNormConstant());
	Storage::printResult(_jointForceStorage,aBaseName+"_"+getName()+"_joint_force",aDir,aDT,aExtension);


	// Make a ControlSet out of activations for use in forward dynamics
	ControlSet cs(*_activationStorage);
	std::string path = (aDir=="") ? "." : aDir;
	std::string name = path + "/" + aBaseName+"_"+getName()+"_controls.xml";
	cs.print(name);
	return(0);
}
