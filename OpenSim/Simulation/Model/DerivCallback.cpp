// DerivCallback.cpp
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

/* Note: This code was originally developed by Realistic Dynamics Inc. 
 * Author: Frank C. Anderson 
 */


//=============================================================================
// INCLUDES
//=============================================================================
#include "DerivCallback.h"
#include "Model.h"


//=============================================================================
// STATICS
//=============================================================================


//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________


using namespace OpenSim;
/**
 * Destructor.
 */
DerivCallback::~DerivCallback()
{
}

//_____________________________________________________________________________
/**
 * Default constructor.
 *
 * @param aModel Model to which the callback mthods apply.
 */
DerivCallback::DerivCallback(Model *aModel) :
	Callback(aModel)
{
	setNull();
}

//_____________________________________________________________________________
/**
 * Copy Constructor.
 *
 */
DerivCallback::DerivCallback(const DerivCallback &aDerivCallback):
Callback(aDerivCallback)
{
	setNull();
	*this = aDerivCallback;
}

//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set NULL values for member variables.
 */
void DerivCallback::
setNull()
{
	setType("DerivCallback");
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
DerivCallback& DerivCallback::
operator=(const DerivCallback &aObject)
{
	// BASE CLASS
	Callback::operator=(aObject);

	return(*this);
}

//=============================================================================
// GET AND SET
//=============================================================================


//=============================================================================
// CALLBACKS
//=============================================================================
//_____________________________________________________________________________
/**
 * This method is intended to be called from Model::deriv() after the
 * states of a model have been set (e.g., after Model::set() or
 * Model::setStates()).
 *
 * Override this method in derived classes.
 *
 * @param aT Current time in the simulation.
 * @param aX Current controls of the model.
 * @param aY Current states of the model
 */
void DerivCallback::
set(double aT,double *aX,double *aY)
{
	//printf("DerivCallback.set: %s.\n",getName());
}
//_____________________________________________________________________________
/**
 * This method is intended to be called from Model::deriv() after 
 * contact quantities have been computed (e.g., after
 * Model::computeContact()).
 *
 * Override this method in derived classes.
 *
 * @param aT Current time in the simulation.
 * @param aX Current controls of the model.
 * @param aY Current states of the model
 */
void DerivCallback::
computeContact(double aT,double *aX,double *aY)
{
	//printf("DerivCallback.computeContact: %s.\n",getName());
}
//_____________________________________________________________________________
/**
 * This method is intended to be called from Model::deriv() after 
 * contact forces have been applied (e.g., after
 * Model::applyContactForces()).
 *
 * Override this method in derived classes.
 *
 * @param aT Current time in the simulation.
 * @param aX Current controls of the model.
 * @param aY Current states of the model
 */
void DerivCallback::
applyContact(double aT,double *aX,double *aY)
{
	//printf("DerivCallback.applyContact: %s.\n",getName());
}
//_____________________________________________________________________________
/**
 * This method is intended to be called from Model::deriv() after 
 * actuation quantities have been computed (e.g., after
 * Model::computeActuation()).
 *
 * Override this method in derived classes.
 *
 * @param aT Current time in the simulation.
 * @param aX Current controls of the model.
 * @param aY Current states of the model
 */
void DerivCallback::
computeActuation(double aT,double *aX,double *aY)
{
	//printf("DerivCallback.computeActuation: %s.\n",getName());
}
//_____________________________________________________________________________
/**
 * This method is intended to be called from Model::deriv() after 
 * actuator forces have been applied (e.g., after
 * Model::applyActuatorForces()).
 *
 * Override this method in derived classes.
 *
 * @param aT Current time in the simulation.
 * @param aX Current controls of the model.
 * @param aY Current states of the model
 */
void DerivCallback::
applyActuation(double aT,double *aX,double *aY)
{
	//printf("DerivCallback.applyActuation: %s.\n",getName());
}
//_____________________________________________________________________________
/**
 * This method is intended to be called from Model::deriv() after 
 * derivatives of the model states have been computed (e.g., after
 * Model::computeAccelerations(), ...).
 *
 * Override this method in derived classes.
 *
 * @param aT Current time in the simulation.
 * @param aX Current controls of the model.
 * @param aY Current states of the model
 */
void DerivCallback::
computeDerivatives(double aT,double *aX,double *aY,double *aDY)
{
	//printf("DerivCallback.computeDerivatives: %s.\n",getName());
}

int DerivCallback::
step(double *aXPrev,double *aYPrev,double *aYPPrev,int aStep,double aDT,double aT,
	double *aX,double *aY,double *aYP,double *aDYDT,void *aClientData)
{
	//printf("IntegCallback.step: %s.\n",getName());
	return (0);
}
