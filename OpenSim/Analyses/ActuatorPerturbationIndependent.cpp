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
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/Actuator.h>
#include <OpenSim/Simulation/Model/ForceSet.h>
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
 * Construct an instance for perturbing actuator forces
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
	int na = _model->getActuators().getSize();
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
	ForceSet& fs = _model->updForceSet();
	for(int i=0; i<fs.getSize(); i++){
        Actuator* act = dynamic_cast<Actuator*>(&fs.get(i));
        if( act ) labels.append(act->getName());
    }
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
reset(const SimTK::State& s)
{
	_step = 0;
	_perturbedForceStorage->reset();
	const Set<Actuator>& fs = _model->getActuators();

	for (int i=0; i<fs.getSize(); i++)  fs.get(i).setIsControlled(true);
    _forceStore->reset();

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

//_____________________________________________________________________________
/**
 * called right after actuation has been computed by the model.
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
setForces(const SimTK::State& s)
{
	//printf("ActuatorPerturbationIndependent.computeActuation: aT=%.16lf  aY[0]=%.16lf\n",aT,aY[0]);

	if(_model==NULL) {
		printf("ActuatorPerturbation.setPerturbation: WARN- no model.\n");
		return;
	}
	if(!getOn()) return;

	// SET UNPERTURBED FORCES
	int actuatorIndex = -1;
	const Set<Actuator>& fs = _model->getActuators();

	_unperturbedForceSplines->evaluate(_forces,0,s.getTime());
	for (int i=0; i<fs.getSize(); i++) {
        Actuator& act = fs.get(i);
	    if(&act==_actuator) actuatorIndex = i;
        act.setForce(s,_forces[i]);
        act.setIsControlled(false);
	}

	// COMPUTE PERTURBED FORCE
	if(!_actuator) return;
	_unperturbedForce = _actuator->getForce(s);
	double force = _unperturbedForce;
	if((s.getTime()>=getStartTime()) && (s.getTime()<=getEndTime())){
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
			printf("ActuatorPerturbationIndependent.setPerturbation :WARN - unrecognized perturbationType.\n");
		}
        _actuator->setForce(s, force );
	}

	// MAKE CORRECTION IF PERTRUBED FORCE IS NEGATIVE AND NEG FORCE FLAG IS SET
	if((_allowNegForce==false) && (_actuator->getForce(s) < 0.0))	{
        _actuator->setForce(s,0.0);
	}


	// Update _forces array and add it to the perturbed force storage
	if(actuatorIndex>=0) _forces[actuatorIndex] = force;
	_perturbedForceStorage->append(s.getTime()*_model->getTimeNormConstant(),_model->getActuators().getSize(),&_forces[0]);
}





