// SimpleFeedbackController.cpp
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
//
// This software, originally developed by Realistic Dynamics, Inc., was
// transferred to Stanford University on November 1, 2006.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//=============================================================================
// INCLUDES
//=============================================================================
#include "osimToolsDLL.h"
#include <iostream>
#include <string>
#include <OpenSim/Common/rdMath.h>
#include <OpenSim/Common/Exception.h>
#include <OpenSim/Common/Array.h>
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Common/RootSolver.h>
#include <OpenSim/Simulation/Model/IntegCallbackSet.h>
#include <OpenSim/Simulation/Model/AnalysisSet.h>
#include <OpenSim/Simulation/Model/ModelIntegrandForActuators.h>
#include <OpenSim/Simulation/Control/ControlConstant.h>
#include "SimpleFeedbackController.h"

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
SimpleFeedbackController::~SimpleFeedbackController()
{
}
//_____________________________________________________________________________
/**
 * Default constructor
 */
SimpleFeedbackController::SimpleFeedbackController() :
	Controller(), _kp(_kpProp.getValueDbl()), _kv(_kvProp.getValueDbl())
{
	// NULL
	setNull();
	setupProperties();
}
//_____________________________________________________________________________
/**
 * Constructor
 *
 * @param aModel Model that is to be controlled.
 * @param aYDesStore Storage object containing desired states.
 */
SimpleFeedbackController::SimpleFeedbackController(Model *aModel, ControlSet *aControlSet, Storage *aYDesStore) :
	Controller(aModel), _kp(_kpProp.getValueDbl()), _kv(_kvProp.getValueDbl())
{
	// NULL
	setNull();
	setupProperties();

	// INPUT CONTROLS
	_controlSet = aControlSet;
	// DESIRED STATES
	_yDesStore = aYDesStore;
}
//_____________________________________________________________________________
/**
 * Constructor from an XML Document
 */
SimpleFeedbackController::
SimpleFeedbackController(const string &aFileName) :
	Controller(aFileName,false), _kp(_kpProp.getValueDbl()), _kv(_kvProp.getValueDbl())
{
	setNull();
	setupProperties();
	updateFromXMLNode();
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aController Controller to copy.
 */
SimpleFeedbackController::
SimpleFeedbackController(const SimpleFeedbackController &aController) :
	Controller(aController), _kp(_kpProp.getValueDbl()), _kv(_kvProp.getValueDbl())
{
	setNull();
	setupProperties();
	copyData(aController);
}
//_____________________________________________________________________________
/**
 * Copy this SimpleFeedbackController and return a pointer to the copy.
 * The copy constructor for this class is used.  This method is called
 * when a description of this controller is read in from an XML file.
 *
 * @return Pointer to a copy of this SimpleFeedbackController.
 */
Object* SimpleFeedbackController::copy() const
{
	SimpleFeedbackController *object = new SimpleFeedbackController(*this);
	return object;
}


//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set NULL values for all member variables.
 */
void SimpleFeedbackController::
setNull()
{
	setType("SimpleFeedbackController");
	
	_kp = 100;
	_kv = 20;
	_controlSet = NULL;
	_yDesStore = NULL;
}
//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void SimpleFeedbackController::
setupProperties()
{
	_kpProp.setComment("Gain for position errors");
	_kpProp.setName("kp");
	_propertySet.append( &_kpProp );

	_kvProp.setComment("Gain for velocity errors");
	_kvProp.setName("kv");
	_propertySet.append( &_kvProp );
}
//_____________________________________________________________________________
/**
 * Copy the member variables of the specified controller.
 */
void SimpleFeedbackController::
copyData(const SimpleFeedbackController &aController)
{
	// Copy parent class's members first.
	Controller::copyData(aController);

	// Copy this class's members.
	_kpProp = aController._kpProp;
	_kp = aController._kp;
	_kvProp = aController._kvProp;
	_kv = aController._kv;
	_controlSet = aController._controlSet;
	_yDesStore = aController._yDesStore;
}


//=============================================================================
// GET AND SET
//=============================================================================
//-----------------------------------------------------------------------------
// KP AND KV
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the gain for position errors.
 *
 * @return Current position error gain value.
 */
double SimpleFeedbackController::
getKp() const
{
	return(_kp);
}
//_____________________________________________________________________________
/**
 * Set the gain for position errors.
 *
 * @param aKp Position error gain for controller will be set to aKp.
 */
void SimpleFeedbackController::
setKp(double aKp)
{
	_kp = aKp;
}
//_____________________________________________________________________________
/**
 * Get the gain for velocity errors.
 *
 * @return Current velocity error gain value.
 */
double SimpleFeedbackController::
getKv() const
{
	return(_kv);
}
//_____________________________________________________________________________
/**
 * Set the gain for velocity errors.
 *
 * @param aKv Velocity error gain for controller will be set to aKv.
 */
void SimpleFeedbackController::
setKv(double aKv)
{
	_kv = aKv;
}

//-----------------------------------------------------------------------------
// INPUT CONTROLS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the object containing the input controls of the simulation.
 *
 * @return Input controls object.
 */
ControlSet* SimpleFeedbackController::
getControlSet() const
{
	return(_controlSet);
}
/**
 * Set the object containing the input controls of the simulation.
 * 
 * @param aControlSet Input controls object.
 */
void SimpleFeedbackController::
setControlSet(const ControlSet &aControlSet)
{
	_controlSet = (ControlSet*)aControlSet.copy();
}

//-----------------------------------------------------------------------------
// DESIRED STATES STORAGE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the storage object containing the desired states of the simulation.
 *
 * @return Desired states storage object.
 */
Storage* SimpleFeedbackController::
getDesiredStatesStorage() const
{
	return(_yDesStore);
}
/**
 * Set the storage object containing the desired states of the simulation.
 * 
 * @param aYDesStore Desired states storage object.
 */
void SimpleFeedbackController::
setDesiredStatesStorage(Storage *aYDesStore)
{
	_yDesStore = aYDesStore;
}

//=============================================================================
// COMPUTE
//=============================================================================
//_____________________________________________________________________________
/**
 * Compute the controls for a simulation.
 *
 * The caller should send in an initial guess.
 */
void SimpleFeedbackController::
computeControls(double &rDT,double aT,const double *aY,
	ControlSet &rControlSet)
{
	// NUMBER OF MODEL COORDINATES, SPEEDS, AND ACTUATORS
	// (ALL ARE PROBABLY EQUAL)
	int nq = _model->getNumCoordinates();
	int nu = _model->getNumSpeeds();
	int N = _model->getNumActuators();

	// NEXT TIME STEP
	double tf = aT + rDT;

	// TURN ANALYSES OFF
	_model->getAnalysisSet()->setOn(false);
	_model->getIntegCallbackSet()->setOn(false);

	// TIME STUFF
	_model->setTime(tf);
	double normConstant = _model->getTimeNormConstant();
	double tiReal = aT * normConstant;
	double tfReal = tf * normConstant;

	// SET MODEL STATES
	_model->setStates(aY);
	_model->setInitialStates(aY);

	// GET CURRENT DESIRED COORDINATES AND SPEEDS
	// Note: yDesired[0..nq-1] will contain the generalized coordinates
	// and yDesired[nq..nq+nu-1] will contain the generalized speeds.
	Array<double> yDesired(0.0,nq+nu);
	_yDesStore->getDataAtTime(tiReal,nq+nu,yDesired);

	// GET INPUT CONTROL VALUES
	Array<double> currentControlValues(0.0,N);
	_controlSet->getControlValues(tf,currentControlValues);

	// COMPUTE EXCITATIONS
	Array<double> newControls(0.0,N);
	for(int i=0;i<N;i++) {
		AbstractActuator *actuator = _model->getActuatorSet()->get(i);
		double currentControlValue = currentControlValues[i];
		double oneOverFmax = 1.0 / actuator->getOptimalForce();
		double pErr = aY[i] - yDesired[i];
		double vErr = aY[i+nq] - yDesired[i+nq];
		double pErrTerm = _kp*oneOverFmax*pErr;
		double vErrTerm = _kv*oneOverFmax*vErr;
		newControls[i] = currentControlValue - vErrTerm - pErrTerm;
	}

	// SET EXCITATIONS
	rControlSet.setControlValues(tf,&newControls[0]);

	// TURN ANALYSES BACK ON
	_model->getAnalysisSet()->setOn(true);
	_model->getIntegCallbackSet()->setOn(true);
}

