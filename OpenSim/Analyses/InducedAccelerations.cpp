// InducedAccelerations.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Ajay Seth
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/* Copyright (c)  2009 Stanford University
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
#include <OpenSim/Common/rdMath.h>
#include <OpenSim/Simulation/Model/DerivCallbackSet.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/AbstractDynamicsEngine.h>
#include <OpenSim/Simulation/Model/BodySet.h>
#include <OpenSim/DynamicsEngines/SimbodyEngine/SimbodyEngine.h>
#include <OpenSim/Actuators/ForceApplier.h>
#include <OpenSim/DynamicsEngines/SimbodyEngine/RollingOnSurfaceConstraint.h>
#include "InducedAccelerations.h"

using namespace OpenSim;
using namespace std;

//=============================================================================
// CONSTANTS
//=============================================================================
#define CENTER_OF_MASS_NAME string("CENTER_OF_MASS")

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 * Delete any variables allocated using the "new" operator.  You will not
 * necessarily have any of these.
 */
InducedAccelerations::~InducedAccelerations()
{

}
//_____________________________________________________________________________
/*
 * Construct an InducedAccelerations instance.
 *
 * @param aModel Model for which the analysis is to be run.
 */
InducedAccelerations::InducedAccelerations(Model *aModel) :
	Analysis(aModel),
	_coordNames(_coordNamesProp.getValueStrArray()),
	_bodyNames(_bodyNamesProp.getValueStrArray()),
	_constraintType(_constraintTypeProp.getValueStr()),
	_constraintSetProp(PropertyObj("", ConstraintSet())),
	_constraintSet((ConstraintSet&)_constraintSetProp.getValueObj()),
	_forceThreshold(_forceThresholdProp.getValueDbl()),
	_computePotentialsOnly(_computePotentialsOnlyProp.getValueBool())
{
	// make sure members point to NULL if not valid. 
	setNull();
	if(_model==NULL) return;

	// DESCRIPTION
	constructDescription();
}
//_____________________________________________________________________________
/*
 * Construct an object from file.
 *
 * The object is constructed from the root element of the XML document.
 * The type of object is the tag name of the XML root element.
 *
 * @param aFileName File name of the document.
 */
InducedAccelerations::InducedAccelerations(const std::string &aFileName):
	Analysis(aFileName, false),
	_coordNames(_coordNamesProp.getValueStrArray()),
	_bodyNames(_bodyNamesProp.getValueStrArray()),
	_constraintType(_constraintTypeProp.getValueStr()),
	_constraintSetProp(PropertyObj("", ConstraintSet())),
	_constraintSet((ConstraintSet&)_constraintSetProp.getValueObj()),
	_forceThreshold(_forceThresholdProp.getValueDbl()),
	_computePotentialsOnly(_computePotentialsOnlyProp.getValueBool())
{
	setNull();

	// Read properties from XML
	updateFromXMLNode();
}

// Copy constrctor and virtual copy 
//_____________________________________________________________________________
/*
 * Copy constructor.
 *
 */
InducedAccelerations::InducedAccelerations(const InducedAccelerations &aInducedAccelerations):
	Analysis(aInducedAccelerations),
	_coordNames(_coordNamesProp.getValueStrArray()),
	_bodyNames(_bodyNamesProp.getValueStrArray()),
	_constraintType(_constraintTypeProp.getValueStr()),
	_constraintSetProp(PropertyObj("", ConstraintSet())),
	_constraintSet((ConstraintSet&)_constraintSetProp.getValueObj()),
	_forceThreshold(_forceThresholdProp.getValueDbl()),
	_computePotentialsOnly(_computePotentialsOnlyProp.getValueBool())
{
	setNull();
	// COPY TYPE AND NAME
	*this = aInducedAccelerations;
}
//_____________________________________________________________________________
/**
 * Clone
 *
 */
Object* InducedAccelerations::copy() const
{
	InducedAccelerations *object = new InducedAccelerations(*this);
	return(object);

}
//=============================================================================
// OPERATORS
//=============================================================================
//-----------------------------------------------------------------------------
// ASSIGNMENT
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/*
 * Assign this object to the values of another.
 *
 * @return Reference to this object.
 */
InducedAccelerations& InducedAccelerations::
operator=(const InducedAccelerations &aInducedAccelerations)
{
	// Base Class
	Analysis::operator=(aInducedAccelerations);

	// Member Variables
	_coordNames = aInducedAccelerations._coordNames;
	_bodyNames = aInducedAccelerations._bodyNames;
	_constraintType = aInducedAccelerations._constraintType;
	_constraintSet = aInducedAccelerations._constraintSet;
	_forceThreshold = aInducedAccelerations._forceThreshold;
	_computePotentialsOnly = aInducedAccelerations._computePotentialsOnly;
	_includeCOM = aInducedAccelerations._includeCOM;
	return(*this);
}

//_____________________________________________________________________________
/**
 * SetNull().
 */
void InducedAccelerations::
setNull()
{
	setType("InducedAccelerations");
	setupProperties();

	_forceThreshold = 6.00;
	_constraintType = "weld";
	_coordNames.setSize(0);
	_bodyNames.setSize(1);
	_bodyNames[0] = CENTER_OF_MASS_NAME;
	_computePotentialsOnly = false;
	// Analysis does not own these sets
	_coordSet.setMemoryOwner(false);
	_bodySet.setMemoryOwner(false);
}
//_____________________________________________________________________________
/*
 * Set up the properties for your analysis.
 *
 * You should give each property a meaningful name and an informative comment.
 * The name you give each property is the tag that will be used in the XML
 * file.  The comment will appear before the property in the XML file.
 * In addition, the comments are used for tool tips in the OpenSim GUI.
 *
 * All properties are added to the property set.  Once added, they can be
 * read in and written to file.
 */
void InducedAccelerations::
setupProperties()
{
	_coordNamesProp.setName("coordinate_names");
	_coordNamesProp.setComment("Names of the coordinates for which to compute induced accelerations."
		"The key word 'All' indicates that the analysis should be performed for all coordinates.");
	_propertySet.append(&_coordNamesProp);

	_bodyNamesProp.setName("body_names");
	_bodyNamesProp.setComment("Names of the bodies for which to compute induced accelerations."
		"The key word 'All' indicates that the analysis should be performed for all bodies."
		"Use 'center_of_mass' to indicate the induced accelerations of the system center of mass.");
	_propertySet.append(&_bodyNamesProp);

	_constraintTypeProp.setName("constraint_type");
	_constraintTypeProp.setComment("Specify the constraint type used to replace ground contact."
									"Currently, 'weld', 'ball' and 'roll' constraints are supported. ");
	_propertySet.append(&_constraintTypeProp);

	_constraintSetProp.setName("ConstraintSet");
	_constraintSetProp.setComment("Specify the constraints used to replace ground contact."
								  "Currently, RollingOnSurfaceConstraints are supported ");
	_propertySet.append(&_constraintSetProp);

	_forceThresholdProp.setName("force_threshold");
	_forceThresholdProp.setComment("The minimum amount of contact force (N) that is sufficient to be replaced with a constraint.");
	_propertySet.append(&_forceThresholdProp);

	_computePotentialsOnlyProp.setName("compute_potentials_only");
	_computePotentialsOnlyProp.setComment("Only compute the potential (accelertion/force) of a muscle to accelerate the model.");
	_propertySet.append(&_computePotentialsOnlyProp);
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Construct a description for the body kinematics files.
 */
void InducedAccelerations::
constructDescription()
{
	string descrip;

	descrip = "\nThis file contains accelerations of coordinates or bodies.\n";
	descrip += "\nUnits are S.I. units (seconds, meters, Newtons, ...)";
	if(getInDegrees()) {
		descrip += "\nAngles are in degrees.";
	} else {
		descrip += "\nAngles are in radians.";
	}
	descrip += "\n\n";

	setDescription(descrip);
	assembleContributors();
}

//_____________________________________________________________________________
/**
 * Assemble the list of contributors for induced acceleration analysis
 */
void InducedAccelerations:: assembleContributors()
{
	Array<string> contribs;
	if (!_computePotentialsOnly)
		contribs.append("total");

	ActuatorSet *actuatorSet = _model->getActuatorSet();

	//Do the analysis on the bodies that are in the indices list
	for(int i=0; i< actuatorSet->getSize(); i++) {
		AbstractActuator *actuator = actuatorSet->get(i);
		if(actuator != NULL){ //If not a non-existant body
			contribs.append(actuator->getName()) ;
		}
	}
 
	contribs.append("gravity");
	contribs.append("velocity");

	_contributors = contribs;
}

/**
 * Construct column labels for the output results.
 *
 * For analyses that run during a simulation, the first column is 
 * always time.  For the purpose of example, the code below adds labels
 * for each contributor
 * This method needs to be called as necessary to update the column labels.
 */
Array<string> InducedAccelerations:: constructColumnLabelsForCoordinate()
{
	Array<string> labels;
	labels.append("time");
	labels.append(_contributors);

	return labels;
}

/**
 * Construct column labels for the body acceleration results.
 *
 * For analyses that run during a simulation, the first column is 
 * always time.  For the purpose of example, the code below adds labels
 * appropriate for recording the translation and orientation of the 
 * desired body.
 *
 * This method needs to be called as necessary to update the column labels.
 */
Array<string> InducedAccelerations:: constructColumnLabelsForBody()
{
	// Get the main headings for all the contributors
	Array<string> contributors = constructColumnLabelsForCoordinate();
	Array<string> labels;

	// first label is time not a contributor
	labels.append(contributors[0]);
	for(int i=1; i<contributors.getSize(); i++) {
		labels.append(contributors[i] + "_X");
		labels.append(contributors[i] + "_Y");
		labels.append(contributors[i] + "_Z");
		labels.append(contributors[i] + "_Ox");
		labels.append(contributors[i] + "_Oy");
		labels.append(contributors[i] + "_Oz");
	}
	return labels;
}

Array<string> InducedAccelerations:: constructColumnLabelsForCOM()
{
	// Get the main headings for all the contributors
	Array<string> contributors = constructColumnLabelsForCoordinate();
	Array<string> labels;

	// first label is time not a contributor
	labels.append(contributors[0]);
	for(int i=1; i<contributors.getSize(); i++) {
		// ADD CONTRIBUTORS TO THE ACCELERATION OF THE WHOLE BODY
		labels.append(contributors[i] + "_X");
		labels.append(contributors[i] + "_Y");
		labels.append(contributors[i] + "_Z");
	}
	return labels;
}


//_____________________________________________________________________________
/**
 * Set up storage objects.
 *
 * In general, the storage objects in your analysis are used to record
 * the results of your analysis and write them to file.  You will often
 * have a number of storage objects, each for recording a different
 * kind of result.
 */
void InducedAccelerations::setupStorage()
{
	CoordinateSet* modelCoordSet = _model->getDynamicsEngine().getCoordinateSet();
	int nc = _coordNames.getSize();

	// Get the indices of the coordinates we are interested in
	_storeInducedAccelerations.setSize(0);
	_coordSet.setSize(0);
	if(nc && (IO::Uppercase(_coordNames.get(0)) == "ALL")) {
		nc = modelCoordSet->getSize();
		for(int i=0; i<nc; i++)
			_coordSet.append(modelCoordSet->get(i));
	}
	else{
		for(int i=0; i<nc; i++){
			AbstractCoordinate *coord = modelCoordSet->get(_coordNames[i]);
			if(coord)
				_coordSet.append(coord);
		}
	}

	// Setup storage object or each coordinate
	nc = _coordSet.getSize();
	_coordIndAccs.setSize(0);
	Array<string> coordAccLabels = constructColumnLabelsForCoordinate();
	for(int i=0; i<nc; i++){
		_storeInducedAccelerations.append(new Storage(1000));
		_storeInducedAccelerations[i]->setName(_coordSet.get(i)->getName());
		_storeInducedAccelerations[i]->setDescription(getDescription());
		_storeInducedAccelerations[i]->setColumnLabels(coordAccLabels);
		_coordIndAccs.append(new Array<double>(0, coordAccLabels.getSize()));
	}

	// Now get the bodies that we are interested, including system center of mass
	BodySet* modelBodySet = _model->getDynamicsEngine().getBodySet();
	int nb = _bodyNames.getSize();

	_bodySet.setSize(0);

	if(nb && (IO::Uppercase(_bodyNames.get(0)) == "ALL")) {
		nb = modelBodySet->getSize();
		for(int i=0; i<nb; i++)
			_bodySet.append(modelBodySet->get(i));
	}
	else{
		for(int i=0; i<nb; i++){
			AbstractBody *body = modelBodySet->get(_bodyNames[i]);
			if(body)
				_bodySet.append(body);
			else if(IO::Uppercase(_bodyNames.get(i)) == CENTER_OF_MASS_NAME)
				_includeCOM = true;
		}
	}

	// Setup storage object for bodies
	nb = _bodySet.getSize();
	_bodyIndAccs.setSize(0);
	Array<string> bodyAccLabels = constructColumnLabelsForBody();
	for(int i=0; i<nb; i++){
		_storeInducedAccelerations.append(new Storage(1000));
		cout << "Body " << i << " in _bodySet: " << _bodySet.get(i)->getName() << endl;
		_storeInducedAccelerations[nc+i]->setName(_bodySet.get(i)->getName());
		_storeInducedAccelerations[nc+i]->setDescription(getDescription());
		_storeInducedAccelerations[nc+i]->setColumnLabels(bodyAccLabels);
		_bodyIndAccs.append(new Array<double>(0, bodyAccLabels.getSize()));
	}

	if(_includeCOM){
		Array<string> comAccLabels = constructColumnLabelsForCOM();
		_comIndAccs.setSize(0);
		_storeInducedAccelerations.append(new Storage(1000, CENTER_OF_MASS_NAME));
		_storeInducedAccelerations[nc+nb]->setDescription(getDescription());
		_storeInducedAccelerations[nc+nb]->setColumnLabels(comAccLabels);
	}

	_coordSet.setMemoryOwner(false);
	_bodySet.setMemoryOwner(false);
}


//=============================================================================
// GET AND SET
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the model for which this analysis is to be run.
 *
 * Sometimes the model on which an analysis should be run is not available
 * at the time an analysis is created.  Or, you might want to change the
 * model.  This method is used to set the model on which the analysis is
 * to be run.
 *
 * @param aModel Model pointer
 */
void InducedAccelerations::
setModel(Model *aModel)
{
	// SET THE MODEL IN THE BASE CLASS
	Analysis::setModel(aModel);

	// UPDATE VARIABLES IN THIS CLASS
	constructDescription();
	setupStorage();
}


//=============================================================================
// ANALYSIS
//=============================================================================
//_____________________________________________________________________________
/**
 * Compute and record the results.
 *
 * This method, for the purpose of example, records the position and
 * orientation of each body in the model.  You will need to customize it
 * to perform your analysis.
 *
 * @param aT Current time in the simulation.
 * @param aX Current values of the controls.
 * @param aY Current values of the states: includes generalized coords and speeds
 */
int InducedAccelerations::record(double aT,double *aX,double *aY)
{
	// Get sizes 
	int nq = _model->getNumCoordinates();
	int nu = _model->getNumSpeeds();
	int ny = _model->getNumStates();

	double *aY_zeroVel = (double *)malloc(ny*sizeof(double));
	memcpy(aY_zeroVel, aY, ny*sizeof(double));
	for(int i = nq; i< nq+nu; i++)
		aY_zeroVel[i] = 0.0;

	Array<double> u(0, nu);
	Array<double> udot(0, nu);

	cout << "time = " << aT << endl;

	// Reset Accelerations for coordinates at this time step
	for(int i=0;i<_coordSet.getSize();i++) {
		_coordIndAccs[i]->setSize(0);
	}

	// Reset Accelerations for bodies at this time step
	for(int i=0;i<_bodySet.getSize();i++) {
		_bodyIndAccs[i]->setSize(0);
	}

	// Reset Accelerations for system center of mass at this time step
	_comIndAccs.setSize(0);

	DerivCallbackSet *derivCallbacks = _model->getDerivCallbackSet();

	// Make sure that the model states and those of the external forces are uptodate
	_model->set(aT,aX,aY);
	derivCallbacks->set(aT,aX,aY);
	
	// Get the Simbody dynamics engine for setting up the force contributors and constraints
	SimbodyEngine *engine = (SimbodyEngine *)(&_model->getDynamicsEngine());

	SimTK::Vector Q = engine->getMultibodySystem()->getMatterSubsystem().getQ(*engine->getSimbodyState());

	//Q.dump("The joint coordinates:");

	// Check the external forces and apply contact constraints
	Array<bool> constraintOn = applyContactConstraintAccordingToExternalForces(aT,aX,aY);

	engine->setInvalid();

	// Cycle through the force contributors to the system acceleration
	for(int c=0; c< _contributors.getSize(); c++){
		if(_contributors[c] == "total"){
			// Set the configuration (gen. coords and speeds) of the model.
			engine->setGravity(_gravity);
			_model->set(aT,aX,aY);
			derivCallbacks->set(aT,aX,aY);
			// Compute and apply all actuator forces.
			_model->getActuatorSet()->computeActuation();
			_model->getActuatorSet()->apply();

			// Realize to acceleration here first to compute constraint forces used to decide if constraints
			// are on or off below, for individual contributors we do not change their status but use cached values
			engine->getMultibodySystem()->realize(*engine->getSimbodyState(), SimTK::Stage::Acceleration);
			if(IO::Uppercase(_constraintType) == "ROLL"){
				for(int i=0; i<constraintOn.getSize(); i++) {
					((RollingOnSurfaceConstraint *)_constraintSet.get(i))->unilateralConditionsSatisfied();
				}

			}

		}
		else if(_contributors[c] == "gravity"){
			engine->setGravity(_gravity);
			// Set the configuration (gen. coords and speeds) of the model.
			_model->set(aT,aX,aY_zeroVel);
			derivCallbacks->set(aT,aX,aY_zeroVel);
		}
		else if(_contributors[c] == "velocity"){
			// Set gravity to zero
			engine->setGravity(SimTK::Vec3(0));
			// Set the configuration (gen. coords and speeds) of the model.
			_model->set(aT,aX,aY);
			derivCallbacks->set(aT,aX,aY);
			// Do not apply any actuation.
		}
		else{ //The rest are actuators
			// Set gravity to zero
			engine->setGravity(SimTK::Vec3(0));
			// Set the configuration (gen. coords and speeds) with speeds zeroed.
			_model->set(aT,aX,aY_zeroVel);
		
			AbstractActuator* actuator = _model->getActuatorSet()->get(_contributors[c]);
			if(actuator){
				if(_computePotentialsOnly)
					actuator->setForce(1.0);
				else{
					actuator->computeActuation();
				}
				actuator->apply();
			}

			derivCallbacks->set(aT,aX,aY_zeroVel);
			//derivCallbacks->computeActuation(aT,aX,aY_zeroVel);
			//derivCallbacks->applyActuation(aT,aX,aY_zeroVel);
		}

		// Make sure that constraints are enabled/disabled correctly
		for(int i=0; i<constraintOn.getSize(); i++) {
			if(IO::Uppercase(_constraintType) == "ROLL"){
				if(constraintOn[i])
					((RollingOnSurfaceConstraint *)_constraintSet.get(i))->setIsDisabledWithCachedUnilateralConditions(false, *engine->getSimbodyState());
				else
					((RollingOnSurfaceConstraint *)_constraintSet.get(i))->setIsDisabledWithCachedUnilateralConditions(true, *engine->getSimbodyState());
			}
			else{
				if(constraintOn[i])
					_constraints[i]->enable(*engine->getSimbodyState());
				else
					_constraints[i]->disable(*engine->getSimbodyState());
			}
		}

		// After setting the state of the model and applying forces
		// Compute the derivative of the multibody system (speeds and accelerations)
		// COMPUTE THE ACCELERATION
		engine->computeDerivatives(&u[0],&udot[0]);

		// Sanity check that constraints hasn't totally changed the configuration of the model
		double error = (Q-engine->getMultibodySystem()->getMatterSubsystem().getQ(*engine->getSimbodyState())).norm();

		// VARIABLES
		SimTK::Vec3 vec,angVec;

		// Get Accelerations for kinematics of bodies
		for(int i=0;i<_coordSet.getSize();i++) {
			AbstractSpeed *speed = engine->getSpeedSet()->get(AbstractSpeed::getSpeedName(_coordSet.get(i)->getName()));
			double acc = speed->getAcceleration();

			if(getInDegrees()) 
				acc *= SimTK_RADIAN_TO_DEGREE;	
			_coordIndAccs[i]->append(1, &acc);
		}

		//cout << "Input Body Names: "<< _bodyNames << endl;

		// Get Accelerations for kinematics of bodies
		for(int i=0;i<_bodySet.getSize();i++) {

			AbstractBody *body = _bodySet.get(i);
			//cout << "Body Name: "<< body->getName() << endl;
			SimTK::Vec3 com(0);
			// Get the center of mass location for this body
			body->getMassCenter(com);
			
			// Get the body acceleration
			engine->getAcceleration(*body, com, vec);
			engine->getAngularAcceleration(*body, angVec);	

			// CONVERT TO DEGREES?
			if(getInDegrees()) 
				angVec *= SimTK_RADIAN_TO_DEGREE;	

			// FILL KINEMATICS ARRAY
			_bodyIndAccs[i]->append(3, &vec[0]);
			_bodyIndAccs[i]->append(3, &angVec[0]);
		}

		// Get Accelerations for kinematics of COM
		if(_includeCOM){
			// Get the body acceleration in ground
			vec = engine->getSystemCenterOfMassAcceleration();

			// FILL KINEMATICS ARRAY
			_comIndAccs.append(3, &vec[0]);
		}
	} // End cycling through contributors at this time step

	// Set the accelerations of coordinates into their storages
	int nc = _coordSet.getSize();
	for(int i=0; i<nc; i++) {
		_storeInducedAccelerations[i]->append(aT, _coordIndAccs[i]->getSize(),&(_coordIndAccs[i]->get(0)));
	}

	// Set the accelerations of bodies into their storages
	int nb = _bodySet.getSize();
	for(int i=0; i<nb; i++) {
		_storeInducedAccelerations[nc+i]->append(aT, _bodyIndAccs[i]->getSize(),&(_bodyIndAccs[i]->get(0)));
	}

	// Set the accelerations of system center of mass into a storage
	_storeInducedAccelerations[nc+nb]->append(aT, _comIndAccs.getSize(), &_comIndAccs[0]);

	return(0);
}
//_____________________________________________________________________________
/**
 * This method is called at the beginning of an analysis so that any
 * necessary initializations may be performed.
 *
 * This method is meant to be called at the begining of an integration in
 * Model::integBeginCallback() and has the same argument list.
 *
 * @param aStep Step number of the integration.
 * @param aDT Size of the time step that will be attempted.
 * @param aT Current time in the integration.
 * @param aX Current control values.
 * @param aY Current states.
 * @param aYP Current pseudo states.
 * @param aDYDT Current state derivatives.
 * @param aClientData General use pointer for sending in client data.
 *
 * @return -1 on error, 0 otherwise.
 */
int InducedAccelerations::
begin(int aStep,double aDT,double aT,double *aX,double *aY,double *aYP,double *aDYDT,
		void *aClientData)
{
	if(!proceed()) return(0);

	// Get the model gravity so we can restore it
	_model->getGravity(_gravity);

	_externalForces.setSize(0);
	_constraints.setSize(0);

	DerivCallbackSet *derivCallbacks = _model->getDerivCallbackSet();

	SimbodyEngine *engine = (SimbodyEngine *)(&_model->getDynamicsEngine());
	for(int i=0; i<_constraintSet.getSize(); i++){
		engine->getConstraintSet()->append(_constraintSet.get(i));
	}
	engine->setup(_model);

	// Create a list of ball or weld constraints used to model contact with the ground
	// based on external forces (ForceAppliers) applied to the model
	for(int i=0; i<derivCallbacks->getSize(); i++){
		ForceApplier *exf = dynamic_cast<ForceApplier *>(derivCallbacks->get(i));
		if(exf){
			addContactConstraintFromExternalForce(exf);
		}
	}
	engine->setInvalid();

	// RESET STORAGES
	for(int i = 0; i<_storeInducedAccelerations.getSize(); i++){
		_storeInducedAccelerations[i]->reset(aT);
	}

	// RECORD
	int status = 0;
	status = record(aT,aX,aY);

	cout << "Performing Induced Accelerations Analysis" << endl;

	return(status);
}
//_____________________________________________________________________________
/**
 * This method is called to perform the analysis.  It can be called during
 * the execution of a forward integrations or after the integration by
 * feeding it the necessary data.
 *
 * When called during an integration, this method is meant to be called in
 * Model::integStepCallback(), which has the same argument list.
 *
 * @param aXPrev Controls at the beginining of the current time step.
 * @param aYPrev States at the beginning of the current time step.
 * @param aYPPrev Pseudo states at the beginning of the current time step.
 * @param aStep Step number of the integration.
 * @param aDT Size of the time step that was just taken.
 * @param aT Current time in the integration.
 * @param aX Current control values.
 * @param aY Current states.
 * @param aYP Current pseudo states.
 * @param aDYDT Current state derivatives.
 * @param aClientData General use pointer for sending in client data.
 *
 * @return -1 on error, 0 otherwise.
 */
int InducedAccelerations::
step(double *aXPrev,double *aYPrev,double *aYPPrev,
	int aStep,double aDT,double aT,double *aX,double *aY,double *aYP,double *aDYDT,
	void *aClientData)
{
	if(!proceed(aStep)) return(0);

	record(aT,aX,aY);

	return(0);
}
//_____________________________________________________________________________
/**
 * This method is called at the end of an analysis so that any
 * necessary finalizations may be performed.
 *
 * This method is meant to be called at the end of an integration in
 * Model::integEndCallback() and has the same argument list.
 *
 * @param aStep Step number of the integration.
 * @param aDT Size of the time step that was just completed.
 * @param aT Current time in the integration.
 * @param aX Current control values.
 * @param aY Current states.
 * @param aYP Current pseudo states.
 * @param aDYDT Current state derivatives.
 * @param aClientData General use pointer for sending in client data.
 *
 * @return -1 on error, 0 otherwise.
 */
int InducedAccelerations::
end(int aStep,double aDT,double aT,double *aX,double *aY,double *aYP,double *aDYDT,
		void *aClientData)
{
	if(!proceed()) return(0);

	record(aT,aX,aY);

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
int InducedAccelerations::
printResults(const string &aBaseName,const string &aDir,double aDT,
				 const string &aExtension)
{
	// Write out induced accelerations for all kinematic variables
	for(int i = 0; i < _storeInducedAccelerations.getSize(); i++){
		_storeInducedAccelerations[i]->scaleTime(_model->getTimeNormConstant());
		Storage::printResult(_storeInducedAccelerations[i],aBaseName+"_"
			                   +getName()+"_"+_storeInducedAccelerations[i]->getName(),aDir,aDT,aExtension);
	}
	
	return(0);
}


void InducedAccelerations::addContactConstraintFromExternalForce(ForceApplier *externalForce)
{
	if(IO::Uppercase(_constraintType) == "BALL"){
		addBallConstraintFromExternalForce(externalForce);
		cout << "Ball constraint(s) successfully added to model." << endl;
	}
	else if(IO::Uppercase(_constraintType) == "ROLL"){
		addRollingConstraintFromExternalForce(externalForce);
		cout << "Roll constraint(s) successfully added to model." << endl;
	}
	else{
		addWeldConstraintFromExternalForce(externalForce);
		cout << "Weld constraint(s) successfully added to model." << endl;
	}
}

Array<bool> InducedAccelerations::applyContactConstraintAccordingToExternalForces(double aT,double *aX,double *aY)
{
	if(IO::Uppercase(_constraintType) == "BALL")
		return applyBallConstraintAccordingToExternalForces( aT, aX, aY);
	else if(IO::Uppercase(_constraintType) == "ROLL")
		return applyRollingConstraintAccordingToExternalForces( aT, aX, aY);
	else
		return applyWeldConstraintAccordingToExternalForces( aT, aX, aY);
}

void InducedAccelerations::addBallConstraintFromExternalForce(ForceApplier *externalForce)
{
	SimbodyEngine *engine = (SimbodyEngine *)(&_model->getDynamicsEngine());

	//Get OpenSim::Bodies
	Body* body1 = dynamic_cast<OpenSim::Body*>(externalForce->getBody());
	Body* body2 = dynamic_cast<OpenSim::Body*>(&engine->getGroundBody());
	// Get underlying mobilized bodies
	SimTK::MobilizedBody b1 = engine->getMultibodySystem()->updMatterSubsystem().getMobilizedBody(body1->getIndex());
	SimTK::MobilizedBody b2 = engine->getMultibodySystem()->updMatterSubsystem().getMobilizedBody(body2->getIndex());
	SimTK::Constraint::Ball* contactPoint = new SimTK::Constraint::Ball(b1, b2);
	//SimTK::Constraint::Weld* contactPoint = new SimTK::Constraint::Weld(b1, b2);
	// Add contact constraint to the list
	_constraints.append(contactPoint);
	// Keep track of which external force we are replacing
	_externalForces.append(externalForce);
}

void InducedAccelerations::addWeldConstraintFromExternalForce(ForceApplier *externalForce)
{
	SimbodyEngine *engine = (SimbodyEngine *)(&_model->getDynamicsEngine());

	//Get OpenSim::Bodies
	Body* body1 = dynamic_cast<OpenSim::Body*>(externalForce->getBody());
	Body* body2 = dynamic_cast<OpenSim::Body*>(&engine->getGroundBody());
	// Get underlying mobilized bodies
	SimTK::MobilizedBody b1 = engine->getMultibodySystem()->updMatterSubsystem().getMobilizedBody(body1->getIndex());
	SimTK::MobilizedBody b2 = engine->getMultibodySystem()->updMatterSubsystem().getMobilizedBody(body2->getIndex());
	SimTK::Constraint::Weld* contactPoint = new SimTK::Constraint::Weld(b1, b2);
	// Add contact constraint to the list
	_constraints.append(contactPoint);
	// Keep track of which external force we are replacing
	_externalForces.append(externalForce);
}

Array<bool> InducedAccelerations::applyBallConstraintAccordingToExternalForces(double aT,double *aX,double *aY)
{
	// Get the matter subsystem
	SimbodyEngine* engine = (SimbodyEngine *)(&_model->getDynamicsEngine());
	SimTK::MultibodySystem* system = engine->getMultibodySystem();

	Array<bool> constraintOn(false, _constraints.getSize());

	for(int i=0; i<_externalForces.getSize(); i++){
		ForceApplier *exf = _externalForces[i];
		SimTK::Vec3 point, force, gpoint;
		exf->getForceFunction()->evaluate(&aT,&force[0]);

		// If the applied force is "significant" replace it with a constraint
		if (force.norm() > _forceThreshold){
			exf->getPointFunction()->evaluate(&aT,&point[0]);
			SimTK::Constraint::Ball *constraint = (SimTK::Constraint::Ball *)_constraints[i];
			constraint->setDefaultPointOnBody1(point);

			// For external forces we assume w.r.t. ground
			engine->getPosition(*exf->getBody(), point, gpoint);
			constraint->setDefaultPointOnBody2(gpoint);
			engine->setInvalid();

			//Re-validate model
			_model->set(aT,aX,aY);

			constraintOn[i] = true;
		}
	}

	return constraintOn;
}

Array<bool> InducedAccelerations::applyWeldConstraintAccordingToExternalForces(double aT,double *aX,double *aY)
{
	// Get the matter subsystem
	SimbodyEngine* engine = (SimbodyEngine *)(&_model->getDynamicsEngine());
	SimTK::MultibodySystem* system = engine->getMultibodySystem();

	Array<bool> constraintOn(false, _constraints.getSize());

	for(int i=0; i<_externalForces.getSize(); i++){
		ForceApplier *exf = _externalForces[i];
		SimTK::Vec3 point, force, gpoint;
		exf->getForceFunction()->evaluate(&aT,&force[0]);
		SimTK::Transform onB1(SimTK::Rotation(), point);

		// If the applied force is "significant" replace it with a constraint
		if (force.norm() > _forceThreshold){
			exf->getPointFunction()->evaluate(&aT,&point[0]);
			SimTK::Constraint::Weld *constraint = (SimTK::Constraint::Weld *)_constraints[i];
			constraint->setDefaultFrameOnBody1(onB1);

			// For external forces we assume w.r.t. ground
			engine->getPosition(*exf->getBody(), point, gpoint);
			OpenSim::Transform temp = engine->getTransform(*(exf->getBody()));
			SimTK::Transform onB2(SimTK::Rotation(SimTK::Mat33(temp.getMatrix())));
			onB2.setT(gpoint);
			constraint->setDefaultFrameOnBody2(onB2);
			engine->setInvalid();
			//Re-validate model
			_model->set(aT,aX,aY);
			constraintOn[i] = true;
		}
	}

	return constraintOn;
}

void InducedAccelerations::addRollingConstraintFromExternalForce(ForceApplier *externalForce)
{
	// By Selecting "roll" as the constraint type, we expect the InducedAccelerations setup
	// to contain a constraintSet with the desired constraints

	if (_constraintSet.getSize() < 1)
		throw(Exception("ERROR Using 'roll' constraint type: no constraints specified",__FILE__,__LINE__));

	// Keep track of which external force we are replacing
	_externalForces.append(externalForce);
}

Array<bool> InducedAccelerations::applyRollingConstraintAccordingToExternalForces(double aT,double *aX,double *aY)
{
	// Get the matter subsystem
	SimbodyEngine* engine = (SimbodyEngine *)(&_model->getDynamicsEngine());
	SimTK::MultibodySystem* system = engine->getMultibodySystem();

	Array<bool> constraintOn(false, _constraintSet.getSize());
	//int nc = 0;

	for(int i=0; i<_externalForces.getSize(); i++){
		ForceApplier *exf = _externalForces[i];
		SimTK::Vec3 point, force, gpoint;
		exf->getForceFunction()->evaluate(&aT,&force[0]);

		// If the applied force is "significant" replace it with a constraint
		if (force.norm() > _forceThreshold){
			exf->getPointFunction()->evaluate(&aT,&point[0]);

			((RollingOnSurfaceConstraint *)_constraintSet.get(i))->setContactPointOnRollingBody(point);
			_constraintSet.get(i)->setIsDisabled(false);

			constraintOn[i] = true;
			engine->setInvalid();
			//Re-validate model
			_model->set(aT,aX,aY);
		}
	}

	return constraintOn;
}