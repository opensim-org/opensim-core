// ActuatedModelIntegCallback.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Frank C. Anderson
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//=============================================================================
// INCLUDES
//=============================================================================
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <OpenSim/Tools/rdMath.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/ContactForceSet.h>
#include <OpenSim/Actuators/SetPoint.h>
#include "ActuatedModel_SDFast.h"
#include "ActuatedModelIntegCallback.h"


//=============================================================================
// CONSTANTS
//=============================================================================


//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________


using namespace OpenSim;
/**
 * Destructor.
 */
ActuatedModelIntegCallback::~ActuatedModelIntegCallback()
{
}
//_____________________________________________________________________________
/**
 * Construct a derivative callback instance for perturbing actuator forces
 * during an integration.
 *
 * @param aModel Model for which actuator forces are to be perturbed.
 */
ActuatedModelIntegCallback::
ActuatedModelIntegCallback(ActuatedModel_SDFast *aModel) :
	IntegCallback(aModel)
{
	setNull();
}


//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Set member variables to approprate NULL values.
 */
void ActuatedModelIntegCallback::
setNull()
{
	setType("ActuatedModelIntegCallback");
}


//=============================================================================
// GET AND SET
//=============================================================================


//=============================================================================
// UTILITY
//=============================================================================


//=============================================================================
// CALLBACKS
//=============================================================================
//_____________________________________________________________________________
/**
 * Perform any necessary updates after each integration step (e.g., updating
 * pseudostates).
 *
 * @param aXPrev Control values at the previous time step.
 * @param aYPrev State values at the previous time step.
 * @param aStep Number of integrations steps that have been completed.
 * @param aDT Size of the time step that WAS just completed.
 * @param aT Current time in the integration.
 * @param aX Current control values.
 * @param aY Current states.
 * @param aClientData General use pointer for sending in client data.
 */
int ActuatedModelIntegCallback::
step(double *aXPrev,double *aYPrev,int aStep,double aDT,double aT,
	double *aX,double *aY,void *aClientData)
{
	// UPDATE THE PSEUDOSTATES
	ActuatedModel_SDFast *model = (ActuatedModel_SDFast*)_model;
	ContactForceSet *contactSet = model->getContactForceSet();
	model->set(aT,aX,aY);
	contactSet->updatePseudoStates();

	return(0);
}





