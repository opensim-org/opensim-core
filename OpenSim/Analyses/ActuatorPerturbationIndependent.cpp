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
#include "ForcePerturbationFunction.h"


using namespace OpenSim;
using namespace std;
 
/**
 ** Set which actuator has its force perturbed 
 **
 ** @param aAct Pointer to the actuator.
 **/
void ActuatorPerturbationIndependent::
setActuator(Actuator *act)
{
    ForcePerturbationFunction* pertFunc;
 
    _actuator = act;
    Set<Actuator>& actuators = _model->updActuators();
 
    //
    // only one actuator is perturbed at a time, 
    // so first turn off perturbation for all actuators
    // 
    for( int i=0;i<actuators.getSize();i++) {
        pertFunc = static_cast<ForcePerturbationFunction*>(actuators.get(i).updOverrideForceFunction());
        pertFunc->setIsPerturbed( false );
    }
    //
    // perturb the actuator
    // 
    if( act != 0 ) { 
        pertFunc = static_cast<ForcePerturbationFunction*>(act->updOverrideForceFunction());
        pertFunc->setIsPerturbed( true );
     }

}





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
void ActuatorPerturbationIndependent::
initalizeOverrideForces() {

  Set<Actuator>& as = _model->updActuators();

   // create override functions for muscles to use during perturb
   for( int i=0;i<as.getSize(); i++ ) {
       ForcePerturbationFunction* perturbFunc = new ForcePerturbationFunction(&as.get(i), this );
       perturbFunc->setUnperturbedForce( _unperturbedForceSplines->getGCVSpline(i));
       as.get(i).setOverrideForceFunction( perturbFunc );
       _overrideFunctions.append( perturbFunc );
   }
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
	_perturbedForceStorage->reset();
	const Set<Actuator>& fs = _model->getActuators();

	for (int i=0; i<fs.getSize(); i++)  fs.get(i).setIsControlled(true);
    _forceStore->reset();

}


//_____________________________________________________________________________
/**
 * The nominal actuator force is recorded so that it can be restored
 *
 */
void ActuatorPerturbationIndependent::
record(const SimTK::State& s) {

	if(_model==NULL) {
		printf("ActuatorPerturbationIndepent.record: WARN- no model.\n");
		return;
	}
	if(!getOn()) return;

    const Set<Actuator>& as = _model->getActuators();
	for (int i=0; i<as.getSize(); i++) {
        Actuator& act = as.get(i);
        _forces[i] = act.getForce(s);
	}

	_perturbedForceStorage->append(s.getTime(),_model->getActuators().getSize(),&_forces[0]);
}




