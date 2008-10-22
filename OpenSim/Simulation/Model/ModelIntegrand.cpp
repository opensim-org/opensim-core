// ModelIntegrand.cpp
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

/*  
 * Author: Frank C. Anderson 
 */

#include <string>
#include "IntegCallbackSet.h"
#include "AnalysisSet.h"
#include "DerivCallbackSet.h"
#include "ModelIntegrand.h"
#include <OpenSim/Simulation/Control/ControlLinear.h>
#include <OpenSim/Simulation/Control/ControlLinearNode.h>
#include "Model.h"
#include "AbstractDynamicsEngine.h"



using namespace OpenSim;
using namespace std;



//=============================================================================
// CONSTRUCTORS & DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Constructs an integrad for an Model.
 *
 * @param aModel An instance of an Model.
 */
ModelIntegrand::ModelIntegrand(Model *aModel):
	_x(0.0),_xPrev(0.0),_yPrev(0.0),_yp(0.0)
{
	setNull();

	// MEMBER VARIABLES
	_model = aModel;
	_controlSet = constructControlSet();
	_ownsControlSet = true;

	// WORK VARIABLES
	_x.setSize(_model->getNumControls());
	_xPrev.setSize(_model->getNumControls());
	_yPrev.setSize(_model->getNumStates());
	_yp.setSize(_model->getNumPseudoStates());
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
ModelIntegrand::~ModelIntegrand()
{
	if (_ownsControlSet) delete _controlSet;
}


//=============================================================================
// CONSTRUCTION AND DESTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Set member variables to their NULL values.
 */
void ModelIntegrand::
setNull()
{
	setType("ModelIntegrand");
	_ti = 0.0;
	_tf = 0.0;
	_tPrev = 0.0;
	_dtPrev = 0.0;
	_model = NULL;
	_controller = NULL;
	_controlStore = NULL;
	_stateStore = NULL;
	_pseudoStore = NULL;
}



//=============================================================================
// GET & SET
//=============================================================================
//-----------------------------------------------------------------------------
// SIZE
//-----------------------------------------------------------------------------
//____________________________________________________________________________
/**
 * Get the size of the state vector (y) and time derivative of the state
 * vector (dydt).  The size is the number of integrated variables.
 *
 * @return Size of the state vector y.
 */
int ModelIntegrand::
getSize() const
{
	return(_model->getNumStates());
}

//-----------------------------------------------------------------------------
// MODEL
//-----------------------------------------------------------------------------
//____________________________________________________________________________
/**
 * Get the model of this integrand.
 *
 * @return Model.
 */
Model* ModelIntegrand::
getModel()
{
	return(_model);
}

//-----------------------------------------------------------------------------
// CONTROL STORAGE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the storage buffer for the integration controls.
 */
void ModelIntegrand::
setControlStorage(Storage *aStorage)
{
	_controlStore = aStorage;
}
//_____________________________________________________________________________
/**
 * Get the storage buffer for the integration controls.
 */
Storage* ModelIntegrand::
getControlStorage()
{
	return(_controlStore);
}

//-----------------------------------------------------------------------------
// STATE STORAGE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the storage buffer for the integration states.
 */
void ModelIntegrand::
setStateStorage(Storage *aStorage)
{
	_stateStore = aStorage;
}
//_____________________________________________________________________________
/**
 * Get the storage buffer for the integration states.
 */
Storage* ModelIntegrand::
getStateStorage()
{
	return(_stateStore);
}

//-----------------------------------------------------------------------------
// PSEUDOSTATE STORAGE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the storage for the pseudostates.
 */
void ModelIntegrand::
setPseudoStateStorage(Storage *aStorage)
{
	_pseudoStore = aStorage;
}
//_____________________________________________________________________________
/**
 * Get the storage for the pseudostates.
 */
Storage* ModelIntegrand::
getPseudoStateStorage()
{
	return(_pseudoStore);
}

//-----------------------------------------------------------------------------
// CONTROL SET
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the control set for this integrand.
 *
 * @param aControlSet Set of controls for the integrand.
 */
void ModelIntegrand::
setControlSet(const ControlSet &aControlSet)
{
	if(aControlSet.getSize() != _model->getNumControls()) {
		cout<<"ModelIntegrand.setControlSet: incorrect number of controls.\n";
		return;
	}
	if(_ownsControlSet) delete _controlSet;
	_controlSet = (ControlSet*)aControlSet.copy();
	_ownsControlSet = true;
}
//_____________________________________________________________________________
/**
 * Set the control set for this integrand to be a reference to an existing conrtol set.
 *
 * @param aControlSet Set of controls for the integrand.
 */
void ModelIntegrand::
setControlSetReference(ControlSet &aControlSet)
{
	if(aControlSet.getSize() != _model->getNumControls()) {
		cout<<"ModelIntegrand.setControlSetReference: incorrect number of controls.\n";
		return;
	}
	if(_ownsControlSet) delete _controlSet;
	_controlSet = &aControlSet;
	_ownsControlSet = false;
}
//_____________________________________________________________________________
/**
 * Set the control set for this integrand.
 *
 * @return aControlSet Set of controls.
 */
ControlSet* ModelIntegrand::
getControlSet()
{
	return(_controlSet);
}


//-----------------------------------------------------------------------------
// CONTROLLER
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set a controller for this integration.
 * If one is set, the controller will be called each integration step to
 * determine a set of controls for controlling the current model.
 *
 * @param aController Controller.  To remove the controller, set the
 * controller to NULL.
 */
void ModelIntegrand::
setController(Controller *aController)
{
	_controller = aController;
}
//_____________________________________________________________________________
/**
 * Get the dynamic filter.
 *
 * @return Controller set for this model.  If no controller is set NULL is
 * returned.
 */
Controller* ModelIntegrand::
getController()
{
	return(_controller);
}


//=============================================================================
// UTILITY
//=============================================================================
//_____________________________________________________________________________
/**
 * Construct a control set for the model.  All the controls in the set
 * are constructed as ControlLinear controls.
 */
ControlSet* ModelIntegrand::
constructControlSet() const
{
	int nx = _model->getNumControls();
	ControlSet *controlSet = new ControlSet();
	controlSet->setName(_model->getName());

	ArrayPtrs<ControlLinearNode> array;

	for(int i=0;i<nx;i++) {
		ControlLinear *control = new ControlLinear();
		control->setName(_model->getControlName(i));
		controlSet->append(control);
	}

	return(controlSet);
}
//____________________________________________________________________________
/**
 * Convert an array of integrated states to an array of model states.
 * The function sets for the coordinate and speed trajectories are used to
 * fill in the values for the generalized coordinates and speeds.
 *
 * @param t Time (real time not normalized time).
 * @param y Integrated states (size = getSize()).
 * @param yModel Model states (size = getSize()+Model::getNumCoordinates()+Model::getNumSpeeds())
 */
void ModelIntegrand::
convertStatesIntegrandToModel(double t,const double y[],double yModel[])
{
	int i;
	int ny = _model->getNumStates();
	for(i=0;i<ny;i++) yModel[i] = y[i];
}
//____________________________________________________________________________
/**
 * Convert an array of model states to an array of integrated states.
 * The function sets for the coordinate and speed trajectories are used to
 * fill in the values for the generalized coordinates and speeds.
 *
 * @param t Time (real time not normalized time).
 * @param y Integrated states (size = getSize()).
 * @param yModel Model states (size = getSize()+Model::getNumCoordinates()+Model::getNumSpeeds())
 */
void ModelIntegrand::
convertStatesModelToIntegrand(const double yModel[],double y[]) const
{
	int i;
	int ny = _model->getNumStates();
	for(i=0;i<ny;i++) y[i] = yModel[i];
}


//=============================================================================
// INITIAL STATES
//=============================================================================
//__________________________________________________________________________
/**
  * Set the initial states for the integration.
  *
  * @param ti Time at which the initial states occur.
  * @param yi Array of initial states (size = getSize()).
  */
void ModelIntegrand::
setInitialStates(double ti,const double yi[])
 {
	_model->setInitialStates(yi);
 }
 //__________________________________________________________________________
 /**
  * Get the initial states for the integration.
  *
  * @param yi Array of initial states (size = getSize()).
  */
void ModelIntegrand::
getInitialStates(double yi[]) const
{
	_model->getInitialStates(yi);
}


//=============================================================================
// COMPUTATION
//=============================================================================
//____________________________________________________________________________
/**
 * Compute the state derivatives for an Model (dydt).
 *
 * @param t Current time.
 * @param y State vector (size = getSize()).
 * @param dydt ModelIntegrand-- the time derivative of the
 * state vector (size = getSize()).
 */
void ModelIntegrand::
compute(double t,double y[],double dydt[])
{
	// GET CONTROLS
	_controlSet->getControlValues(t,_x);
	
	// TIME, CONTROLS, STATES
	_model->set(t,&_x[0],y);
	_model->getDerivCallbackSet()->set(t,&_x[0],y);

	// ACTUATION
	_model->getActuatorSet()->computeActuation();
	_model->getDerivCallbackSet()->computeActuation(t,&_x[0],y);
	_model->getActuatorSet()->apply();
	_model->getDerivCallbackSet()->applyActuation(t,&_x[0],y);

	// CONTACT
	_model->getContactSet()->computeContact();
	_model->getDerivCallbackSet()->computeContact(t,&_x[0],y);
	_model->getContactSet()->apply();
	_model->getDerivCallbackSet()->applyContact(t,&_x[0],y);

	// DERIVATIVES
	_model->computeDerivatives(dydt);
	_model->getDerivCallbackSet()->computeDerivatives(t,&_x[0],y,dydt);

	// NORMALIZE
	double tnorm = _model->getTimeNormConstant();
	int ny = _model->getNumStates();
	for(int i=0;i<ny;i++) dydt[i] *= tnorm;
}

//____________________________________________________________________________
/**
 * Compute the state derivatives for an Model (dydt).
 *
 * @param t Current time.
 * @param y State vector (size = getSize()).
 * @param dydt ModelIntegrand-- the time derivative of the
 * state vector (size = getSize()).
 */
void ModelIntegrand::
computeDirectControl(double x[],double t,double y[],double dydt[])
{
	// GET CONTROLS
	int nx = _model->getNumControls();
	for (int i=0;i<nx;i++) _x[i]=x[i];
	
	// TIME, CONTROLS, STATES
	_model->set(t,&_x[0],y);
	_model->getDerivCallbackSet()->set(t,&_x[0],y);

	// ACTUATION
	_model->getActuatorSet()->computeActuation();
	_model->getDerivCallbackSet()->computeActuation(t,&_x[0],y);
	_model->getActuatorSet()->apply();
	_model->getDerivCallbackSet()->applyActuation(t,&_x[0],y);

	// CONTACT
	_model->getContactSet()->computeContact();
	_model->getDerivCallbackSet()->computeContact(t,&_x[0],y);
	_model->getContactSet()->apply();
	_model->getDerivCallbackSet()->applyContact(t,&_x[0],y);

	// DERIVATIVES
	_model->computeDerivatives(dydt);
	_model->getDerivCallbackSet()->computeDerivatives(t,&_x[0],y,dydt);

	// NORMALIZE
	double tnorm = _model->getTimeNormConstant();
	int ny = _model->getNumStates();
	for(int i=0;i<ny;i++) dydt[i] *= tnorm;
}

//=============================================================================
// HOOKS
//=============================================================================
//____________________________________________________________________________
/**
 * Initialize the ModelIntegrand at the beginning of an integration.
 *
 * @param step Integration step.
 * @param dt First step size; it can be changed.
 * @param ti Initial time of the integration.
 * @param tf Final time of the simulation.
 * @param y Initial values of the states.
 */
void ModelIntegrand::
initialize(int step,double &dt,double ti,double tf,double y[])
{
	// VARIABLES
	_ti = ti;
	_tf = tf;
	_tPrev = _ti;
	double tReal = _ti * _model->getTimeNormConstant();
	int nx = _model->getNumControls();
	int ny = _model->getNumStates();
	int nyp = _model->getNumPseudoStates();

	// SET
	_model->setTime(_ti);
	_model->setStates(y);
	
	// INITIAL CONTROLS
	if(_controller!=NULL) {
		_controller->computeControls(dt,_ti,y,*_controlSet);
	}
	_controlSet->getControlValues(_ti,_x);
	_model->setControls(&_x[0]);

	// INITIALIZE PREVIOUS CONTROL AND STATE VALUES
	memcpy(&_xPrev[0],&_x[0],nx*sizeof(double));
	memcpy(&_yPrev[0],y,ny*sizeof(double));

	// STORE STARTING CONTROLS
	if(_controlStore!=NULL) {
		// ONLY IF NO CONTROLS WERE PREVIOUSLY STORED
		if(_controlStore->getSize()==0) {
			_controlStore->store(step,tReal,nx,&_x[0]);
		}
	}

	// STORE STARTING STATES
	if(_stateStore!=NULL) {
		// ONLY IF NO STATES WERE PREVIOUSLY STORED
		if(_stateStore->getSize()==0) {
			_stateStore->store(step,tReal,ny,y);
		}
	}

	// STORE STARTING PSEUDOSTATES
	if(_pseudoStore!=NULL) {
		// ONLY IF NO STATES WERE PREVIOUSLY STORED
		if(_pseudoStore->getSize()==0) {
			_model->getPseudoStates(&_yp[0]);
			_pseudoStore->store(step,tReal,nyp,&_yp[0]);
		}
	}

	// ANALYSES & INTEGRATION CALLBACKS
	IntegCallbackSet *callbackSet = _model->getIntegCallbackSet();
	AnalysisSet *analysisSet = _model->getAnalysisSet();
	if(callbackSet!=NULL) callbackSet->begin(step,dt,_ti,&_x[0],y);
	if(analysisSet!=NULL) analysisSet->begin(step,dt,_ti,&_x[0],y);
}
//____________________________________________________________________________
/**
 * Perform any desired operations after the last successful integration step.
 *
 * @param step Step number.
 * @param dt Time delta for the next integration step; it can be changed.
 * @param t Current time of the integration.
 * @param y Current values of the states.
 */
void ModelIntegrand::
processAfterStep(int step,double &dt,double t,double y[])
{
	_dtPrev = t - _tPrev;
	double tReal = t * _model->getTimeNormConstant();

	// GET THE CONTROLS AT THE NEW TIME t
	if(_controller!=NULL) {
		_controller->computeControls(dt,t,y,*_controlSet);
	}
	_controlSet->getControlValues(t,_x);

	// STORE CONTROLS
	int nx = _x.getSize();
	if(_controlStore!=NULL) {
		if(t<_tf) {
			_controlStore->store(step,tReal,nx,&_x[0]);
		} else {
			_controlStore->append(tReal,nx,&_x[0]);
		}
	}

	// STORE STATES
	int ny = _model->getNumStates();
	if(_stateStore!=NULL) {
		if(t<_tf) {
			_stateStore->store(step,tReal,ny,y);
		} else {
			_stateStore->append(tReal,ny,y);
		}
	}

	// This used to be in ActuatedModelIntegCallback::step
	// TODO: possibly move it to its own callback rather than always doing this.
	_model->set(t,&_x[0],y);
	if(_model->getContactSet())
		_model->getContactSet()->updatePseudoStates();

	// INTEGRATION CALLBACKS
	IntegCallbackSet *callbackSet = _model->getIntegCallbackSet();
	if(callbackSet!=NULL)
		callbackSet->step(&_xPrev[0],&_yPrev[0],NULL,step,_dtPrev,t,&_x[0],y);

	// ANALYSES
	AnalysisSet *analysisSet = _model->getAnalysisSet();
	if(analysisSet!=NULL)
		analysisSet->step(&_xPrev[0],&_yPrev[0],NULL,step,_dtPrev,t,&_x[0],y);

	// STORE PSEUDOSTATES
	int nyp = _model->getNumPseudoStates();
	if(_pseudoStore!=NULL) {
		_model->getPseudoStates(&_yp[0]);
		if(t<_tf) {
			_pseudoStore->store(step,tReal,nyp,&_yp[0]);
		} else {
			_pseudoStore->append(tReal,nyp,&_yp[0]);
		}
	}

	// UPDATE PREVIOUS CONTROL AND STATE VALUES
	memcpy(&_xPrev[0],&_x[0],nx*sizeof(double));
	memcpy(&_yPrev[0],&y[0],ny*sizeof(double));

	// UPDATE PREVIOUS TIME
	_tPrev = t;
}
//____________________________________________________________________________
/**
 * Finalize the ModelIntegrand after an integration has completed (e.g., clean up).
 *
 * @param step Step number.
 * @param t Time at which the integration completed.
 * @param y Current values of the states.
 */
void ModelIntegrand::
finalize(int step,double t,double y[])
{
	IntegCallbackSet *callbackSet = _model->getIntegCallbackSet();
	if(callbackSet!=NULL) callbackSet->end(step,_dtPrev,t,&_x[0],y);

	AnalysisSet *analysisSet = _model->getAnalysisSet();
	if(analysisSet!=NULL) analysisSet->end(step,_dtPrev,t,&_x[0],y);
}
