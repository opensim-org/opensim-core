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
	_unperturbedForceStorage = new Storage();
	_unperturbedForceStorage->setName("UnperturbedForces");
	_perturbedForceStorage = new Storage();
	_perturbedForceStorage->setName("PerturbedForces");
	constructDescription();
	constructColumnLabels();
	int na = _model->getNumActuators();
	printf("ActuatorPerturbationIndependent.setNull: na = %d\n",na);
	_forces = new double[na];
	_recordUnperturbedForces = true;
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

	_unperturbedForceStorage->setDescription(descrip);
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
	_unperturbedForceStorage->setColumnLabels(labels);
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
	delete[] _forces; _forces=NULL;
	delete _unperturbedForceStorage; _unperturbedForceStorage=NULL;
	delete _perturbedForceStorage; _perturbedForceStorage=NULL;
}



//=============================================================================
// GET AND SET
//=============================================================================
//-----------------------------------------------------------------------------
//	RECORD UNPERTURBED FORCES OR APPLY PERTURBATION
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set whether to record unperturbed forces or to apply a force perturbation.
 * It is the user's responsibility to ensure that, during the first run of 
 * the simulation, this flag is set to "true" to record the forces that can 
 * be used during the second run when the this flag is set to "false" and the 
 * forces are perturbed.
 *
 * @param aTrueFalse Flag to indicate whether forces should be recorded (true) 
 * or perturbed (false). 
 */
void ActuatorPerturbationIndependent::
setRecordUnperturbedForces(bool aTrueFalse)
{
	_recordUnperturbedForces = aTrueFalse;
}
//_____________________________________________________________________________
/**
 * Get whether to record unperturbed forces or to apply a force perturbation.
 * It is the user's responsibility to ensure that, during the first run of 
 * the simulation, this flag is set to "true" to record the forces that can 
 * be used during the second run when the this flag is set to "false" and the 
 * forces are perturbed.
 *
 * @return True if set to record unperturbed forces; false otherwise. 
 */
bool ActuatorPerturbationIndependent::
getRecordUnperturbedForces()
{
	return(_recordUnperturbedForces);
}

//-----------------------------------------------------------------------------
// FORCE STORAGE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get a pointer to the Storage containing the unperturbed forces.
 *
 * @return Pointer to the unperturbed force storage. 
 */
Storage* ActuatorPerturbationIndependent::
getUnperturbedForceStorage()
{
	return(_unperturbedForceStorage);
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

	// RECORD OR SET UNPERTURBED FORCES
	ActuatorSet *as = _model->getActuatorSet();

	int actuatorIndex = -1;

	// Set unperturbed forces
	if(!_recordUnperturbedForces){
		_unperturbedForceStorage->getData(_step,_model->getNumActuators(),_forces);
		// sanity check
		double checkTime;
		if(!_unperturbedForceStorage->getTime(_step,checkTime) || checkTime!=aT*_model->getTimeNormConstant())
			throw Exception("ActuatorPerturbationIndependent: ERR- time match error",__FILE__,__LINE__);

		for (int i=0; i<as->getSize(); i++)
		{
			AbstractActuator *act=as->get(i);
			if(act==_actuator) actuatorIndex = i;
			act->setForce(_forces[i]);
		}

	// Record
	} else {
		for (int i=0; i<as->getSize(); i++)
		{
			AbstractActuator *act=as->get(i);
			_forces[i] = act->getForce();
		}
		// Important to not check for duplicate time stamps in append (hence the "false" flag),
		// to ensure that every state vector gets appended (which is assumed by this class)
		_unperturbedForceStorage->append(
			aT*_model->getTimeNormConstant(),_model->getNumActuators(),_forces,false);
	}	

	// COMPUTE PERTURBED FORCE
	if(!_recordUnperturbedForces && _actuator) {
		_unperturbedForce = _actuator->getForce();
		double force = _unperturbedForce;
		if((aT>=getStartTime()) && (aT<=getEndTime())){
			if( _perturbationType == SCALE) {
				force = force + _perturbation*force;
			}
			else if( _perturbationType == DELTA) {
				force = force + _perturbation;
			}
			else if( _perturbationType == CONSTANT) {
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
		_perturbedForceStorage->append(aT*_model->getTimeNormConstant(),_model->getNumActuators(),_forces);
	}
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
	if(!_recordUnperturbedForces && _actuator) {
		_actuator->setForce(_unperturbedForce);
		//printf("\t\t%d: unperturbed=%.16lf\n",_step+1,force);
	}

	// INCREMENT THE STEP
	_step++;
}





