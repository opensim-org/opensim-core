// ActuatorPerturbationIndependent.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Frank C. Anderson, Saryn Goldberg, May Liu
// 
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//=============================================================================
// INCLUDES
//=============================================================================
#include <iostream>
#include <string>
#include <OpenSim/Common/rdMath.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/AbstractActuator.h>
#include <OpenSim/Common/Object.h>
#include "ActuatorPerturbationIndependent.h"


using namespace OpenSim;
using namespace std;


//=============================================================================
// CONSTANTS
//=============================================================================


//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
ActuatorPerturbationIndependent::~ActuatorPerturbationIndependent()
{
	deleteStorage();
}
//_____________________________________________________________________________
/**
 * Construct a derivative callback instance for perturbing actuator forces
 * during an integration while forcing all other muscles to exert their nominal
 * force.
 *
 * @param aModel Model for which actuator forces are to be perturbed.
 */
ActuatorPerturbationIndependent::
ActuatorPerturbationIndependent(Model *aModel) :
	ActuatorPerturbation(aModel)
{
	setNull();
}


//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Set member data to null values.
 */
void ActuatorPerturbationIndependent::
setNull()
{
	setType("ActuatorPerturbationIndependent");
	_step = 0;
	_unperturbedForceSplines = NULL;
	_unperturbedForceStorage = NULL;
	_perturbedForceStorage = new Storage();
	_perturbedForceStorage->setName("PerturbedForces");
	constructDescription();
	constructColumnLabels();
	int na = _model->getNumActuators();
	printf("ActuatorPerturbationIndependent.setNull: na = %d\n",na);
	_forces.setSize(na);
	_allowNegForce = true;
	_unperturbedForce = 0;
}
//-----------------------------------------------------------------------------
// DESCRIPTION
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Construct a description.
 */
void ActuatorPerturbationIndependent::
constructDescription()
{
	char descrip[1024];
	strcpy(descrip,"\nUnits are S.I. units (second, meters, Newtons, ...)");
	strcat(descrip,"\n\n");
	_perturbedForceStorage->setDescription(descrip);
}
//_____________________________________________________________________________
/**
 * Construct the column labels.
 */
void ActuatorPerturbationIndependent::
constructColumnLabels()
{
	Array<string> labels;
	labels.append("time");
	ActuatorSet *as = _model->getActuatorSet();
	for(int i=0; i<as->getSize(); i++) labels.append(as->get(i)->getName());
	_perturbedForceStorage->setColumnLabels(labels);
}


//=============================================================================
// DESTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Delete storage objects.
 */
void ActuatorPerturbationIndependent::
deleteStorage()
{
	delete _perturbedForceStorage; _perturbedForceStorage=NULL;
}



//=============================================================================
// GET AND SET
//=============================================================================
//-----------------------------------------------------------------------------
// FORCE STORAGE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the Storage object containing the unperturbed forces. This storage
 * object is not deleted; the caller is responsible for cleaning up the
 * resources associated with the storage object.
 *
 * A cubic spline set is used to fit the forces contained in the object, and
 * the splines are used to apply actuator forces throughout an integration.
 *
 * @param aStore Unperturbed force storage that is used to construct
 * the spline set.
 */
void ActuatorPerturbationIndependent::
setUnperturbedForceSplineSet(Storage *aStore)
{
	if(aStore==NULL) return;
	_unperturbedForceStorage = aStore;
	delete _unperturbedForceSplines;
	_unperturbedForceSplines = new GCVSplineSet(3,_unperturbedForceStorage);
}
//_____________________________________________________________________________
/**
 * Get a pointer to the spline set used to fit the unperturbed actuator
 * forces.
 *
 * @return Spline set containing the unperturbed actuator
 * force curves.
 */
GCVSplineSet* ActuatorPerturbationIndependent::
getUnperturbedForceSplineSet()
{
	return(_unperturbedForceSplines);
}
//_____________________________________________________________________________
/**
 * Get a pointer to the Storage containing the perturbed forces.
 *
 * @return Pointer to the perturbed force storage.
 */
Storage* ActuatorPerturbationIndependent::
getPerturbedForceStorage()
{
	_perturbedForceStorage->setName("PerturbedForce_"+getActuator()->getName());
	return(_perturbedForceStorage);
}

//-----------------------------------------------------------------------------
// RESET
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the step counter to zero and reset the perturbed force storage
 *
 */
void ActuatorPerturbationIndependent::
reset()
{
	_step = 0;
	_perturbedForceStorage->reset();
}

//-----------------------------------------------------------------------------
// GET STEP
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the current value of the step counter.
 *
 * @return The current integrator step value.
 */
int ActuatorPerturbationIndependent::
getStep()
{
	return(_step);
}

//=============================================================================
// CALLBACKS
//=============================================================================
//_____________________________________________________________________________
/**
 * Callback called right after actuation has been computed by the model.
 *
 * The nominal actuator force is recorded so that it can be restored, and the
 * actuator force is replaced by its perturbed value.
 *
 * @param aT Real time.
 * @param aX Controls.
 * @param aY States.
 * @todo Fix output of perturbed forces.
 * @todo Reset perturbed forces storage object for use in multiple
 * integrations.  For now, appending to this storage is disabled.
 */
void ActuatorPerturbationIndependent::
computeActuation(double aT,double *aX,double *aY)
{
	//printf("ActuatorPerturbationIndependent.computeActuation: aT=%.16lf  aY[0]=%.16lf\n",aT,aY[0]);

	if(_model==NULL) {
		printf("ActuatorPerturbation.computeActuation: WARN- no model.\n");
		return;
	}
	if(!getOn()) return;

	// SET UNPERTURBED FORCES
	int actuatorIndex = -1;
	ActuatorSet *as = _model->getActuatorSet();
	_unperturbedForceSplines->evaluate(_forces,0,aT);
	for (int i=0; i<as->getSize(); i++) {
		AbstractActuator *act=as->get(i);
		if(act==_actuator) actuatorIndex = i;
		act->setForce(_forces[i]);
	}

	// COMPUTE PERTURBED FORCE
	if(!_actuator) return;
	_unperturbedForce = _actuator->getForce();
	double force = _unperturbedForce;
	if((aT>=getStartTime()) && (aT<=getEndTime())){
		if(_perturbationType==SCALE) {
			force = force + _perturbation*force;
		}
		else if(_perturbationType==DELTA) {
			force = force + _perturbation;
		}
		else if(_perturbationType==CONSTANT) {
			force = _perturbation;
		}
		else {
			printf("ActuatorPerturbationIndependent.computeActuation:WARN - unrecognized perturbationType.\n");
		}
	}

	// MAKE CORRECTION IF PERTRUBED FORCE IS NEGATIVE AND NEG FORCE FLAG IS SET
	if((_allowNegForce==false) && (force < 0.0))	{
		force = 0.0;
	}

	// SET PERTURBED FORCE
	//printf("\n%d: perturbed=%.16lf",_step+1,force);
	_actuator->setForce(force);

	// Update _forces array and add it to the perturbed force storage
	if(actuatorIndex>=0) _forces[actuatorIndex] = force;
	_perturbedForceStorage->append(aT*_model->getTimeNormConstant(),_model->getNumActuators(),&_forces[0]);
}
//_____________________________________________________________________________
/**
 * Callback called right after actuation has been applied by the model.
 *
 * The perturbed actuator force is recorded and then the unperturbed actuator 
 * force is restored.
 */
void ActuatorPerturbationIndependent::
applyActuation(double aT,double *aX,double *aY)
{
 //  printf("\t\tapplyActuation: aT=%.16lf  aY[0]=%.16lf\n",aT,aY[0]);
	if(_model==NULL) {
		printf("ActuatorPerturbationIndependent.applyActuation: WARN- no model.\n");
		return;
	}
	if(!getOn()) return;

	// RESTORE UNPERTURBED FORCE
	// NOTE: We used to try to get unperturbed force from _unperturbedForceStorage, but instead I
	// just record it in a special variable in computeActuation (to avoid having to figure out
	// the actuator index corresponding to _actuator)
	if(_actuator) {
		_actuator->setForce(_unperturbedForce);
		//printf("\t\t%d: unperturbed=%.16lf\n",_step+1,force);
	}

	// INCREMENT THE STEP
	_step++;
}





