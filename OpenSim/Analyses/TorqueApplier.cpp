// TorqueApplier.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Frank C. Anderson and May Liu
// 
//
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//=============================================================================
// INCLUDES
//=============================================================================
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <OpenSim/Tools/rdMath.h>
#include <OpenSim/Tools/Mtx.h>
#include <OpenSim/Tools/rdTools.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Tools/VectorFunction.h>
#include "TorqueApplier.h"

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________


using namespace OpenSim;
/**
 * Destructor.
 */
TorqueApplier::~TorqueApplier()
{
	deleteStorage();
}
//_____________________________________________________________________________
/**
 * Construct a derivative callback instance for applying external torques
 * during an integration.
 *
 * @param aModel Model for which external torques are to be applied.
 * @param aBody Body to which external torques are to be applied.
 */
TorqueApplier::
TorqueApplier(Model *aModel,int aBody) :
	DerivCallback(aModel)
{
	setNull();

	// MEMBER VARIABLES
	setBody(aBody);

	// STORAGE
	allocateStorage();

	// DESCRIPTION AND LABELS
	constructDescription();
	constructColumnLabels();
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Set member variables to approprate NULL values.
 */
void TorqueApplier::
setNull()
{
	setType("TorqueApplier");
	_body = 0;	
	_torque[0] = _torque[1] = _torque[2] = 0.0;
	_torqueFunction = NULL;
	_inputTorquesInGlobalFrame = true;
	_appliedTorqueStore = NULL;
}
//_____________________________________________________________________________
/**
 * Construct a description for the body kinematics files.
 */
void TorqueApplier::
constructDescription()
{
	char descrip[1024];

	strcpy(descrip,"\nThis file contains the torques ");
	strcat(descrip,"that were applied to the body segment,\n");
	strcat(descrip,"as a function of time.\n");
	strcat(descrip,"\nUnits are S.I. units (seconds, meters, Newtons, ...)");
	strcat(descrip,"\n\n");

	setDescription(descrip);
}

//_____________________________________________________________________________
/**
 * Construct column labels for the body kinematics files.
 */
void TorqueApplier::
constructColumnLabels()
{
	char labels[2048];

	strcpy(labels,"time\tTorque_x\tTorque_y\tTorque_z\n");

	_appliedTorqueStore->setColumnLabels(labels);
}

//_____________________________________________________________________________
/**
 * Allocate storage for the kinematics.
 */
void TorqueApplier::
allocateStorage()
{
	char bodyName[2048];
	char title[2048];
	sprintf(title,"Forces applied to ");
	sprintf(bodyName,"body_%d",_body);
	strcat(title, bodyName);
	_appliedTorqueStore = new Storage(1000,title);
	_appliedTorqueStore->setDescription(getDescription());
	_appliedTorqueStore->setColumnLabels(_appliedTorqueStore->getColumnLabels());

//
}


//=============================================================================
// DESTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Delete storage objects.
 */
void TorqueApplier::
deleteStorage()
{
	if(_appliedTorqueStore!=NULL) { delete _appliedTorqueStore;  _appliedTorqueStore=NULL; }
}


//=============================================================================
// GET AND SET
//=============================================================================
//-----------------------------------------------------------------------------
// BODY
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set to which body an external torque should be applied.
 *
 * @param aIndex Index of the body to which an external torque should be applied.
 */
void TorqueApplier::
setBody(int aBody)
{
	_body = aBody;
}
//_____________________________________________________________________________
/**
 * Get to which body an external torque should be applied.
 *
 * @return aIndex Index of the body to which an external torque should be applied.
 */
int TorqueApplier::
getBody() const
{
	return(_body);
}


//-----------------------------------------------------------------------------
// TORQUE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the external torque to be applied, in the inertial reference frame.
 * 
 * @param aTorque External torque to be applied.
 */
void TorqueApplier::
setTorque(double aTorque[3])
{
	_torque[0] = aTorque[0];
	_torque[1] = aTorque[1];
	_torque[2] = aTorque[2];
}
//_____________________________________________________________________________
/**
 * Get the external torque to be applied, in the inertial reference frame.
 *
 * @return aTorque
 */
void TorqueApplier::
getTorque(double rTorque[3]) const
{
	rTorque[0] = _torque[0];
	rTorque[1] = _torque[1];
	rTorque[2] = _torque[2];
}
//-----------------------------------------------------------------------------
// TORQUE FUNCTION
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the vector function containing the (t,x,y,z) of the torque
 * to applied.
 *
 * @param aTorqueFunction Vector function describing the torque to applied
 * in global coordinates.
 */
void TorqueApplier::
setTorqueFunction(VectorFunction* aTorqueFunction)
{
	_torqueFunction = aTorqueFunction;
}
//_____________________________________________________________________________
/**
 * Get the vector function containing the (t,x,y,z) of the torque
 * to applied.
 *
 * @return rTorqueFunction.
 */
VectorFunction* TorqueApplier::
getTorqueFunction() const
{
	return(_torqueFunction);
}

//-----------------------------------------------------------------------------
// APPLIED TORQUE STORAGE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the applied torque storage.
 *
 * @return Applied torque storage.
 */
Storage* TorqueApplier::
getAppliedTorqueStorage()
{
	return(_appliedTorqueStore);
}
//-----------------------------------------------------------------------------
// STORAGE CAPACITY
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the capacity increments of applied torque storage.
 *
 * @param aIncrement Increment by which storage capacities will be increased
 * when storage capcities run out.
 */
void TorqueApplier::
setStorageCapacityIncrements(int aIncrement)
{
	_appliedTorqueStore->setCapacityIncrement(aIncrement);
}

//-----------------------------------------------------------------------------
// COORDINATE FRAME OF INPUT TORQUES
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set whether the input torques are expressed in the global coordinate 
 * frame.
 *
 * @param inputTorquesInGlobalFrame flag indicates whether whether input torques
 * are expressed in the global coordinate frame
 */
void TorqueApplier::
setInputTorquesInGlobalFrame(bool aTrueFalse)
{
	_inputTorquesInGlobalFrame = aTrueFalse;
}
//_____________________________________________________________________________
/**
 * Get whether the input torques are expressed in the global coordinate 
 * frame.
 *
 * @return inputTorquesInGlobalFrame Flag
 * @see setInputTorquesInGlobalFrame()
 */
bool TorqueApplier::
getInputTorquesInGlobalFrame() const
{
	return(_inputTorquesInGlobalFrame);
}

//=============================================================================
// CALLBACKS
//=============================================================================
//_____________________________________________________________________________
/**
 * Callback called right after actuation has been applied by the model.
 *
 * *
 * @param aT Real time.
 * @param aX Controls.
 * @param aY States.
 */
void TorqueApplier::
applyActuation(double aT,double *aX,double *aY)
{
	double torque[3] = {0,0,0};
	const int ground = _model->getGroundID();
	double treal = aT*_model->getTimeNormConstant();

	if(_model==NULL) {
		printf("TorqueApplier.applyActuation: WARN- no model.\n");
		return;
	}
	if(!getOn()) return;

	if((aT>=getStartTime()) && (aT<getEndTime())){

		if(_torqueFunction!=NULL) {
			_torqueFunction->evaluate(&treal,torque);
			setTorque(torque);
		}
	
		if(_inputTorquesInGlobalFrame == false){
			_model->applyTorqueBodyLocal(_body,_torque);
		} else {
			_model->applyTorque(_body,_torque);
		}

		_appliedTorqueStore->append(aT,3,_torque);

	}	
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
int TorqueApplier::
printResults(const char *aBaseName,const char *aDir,double aDT,
				 const char *aExtension)
{
	if(aBaseName==NULL) return(-1);

	// CONSTRUCT PATH
	char path[2048],name[2048],bodyName[2048];
	if(aDir==NULL) {
		strcpy(path,".");
	} else {
		strcpy(path,aDir);
	}

	sprintf(bodyName,"body_%d",_body);


	// ACCELERATIONS
	_appliedTorqueStore->scaleTime(_model->getTimeNormConstant());
	if(aExtension==NULL) {
		sprintf(name,"%s/%s_%s_appTorque",path,aBaseName,bodyName);
	} else {
		sprintf(name,"%s/%s_%s_appTorque%s",path,aBaseName,bodyName,aExtension);
	}
	if(aDT<=0.0) {
		if(_appliedTorqueStore!=NULL)  _appliedTorqueStore->print(name);
	} else {
		if(_appliedTorqueStore!=NULL)  _appliedTorqueStore->print(name,aDT);
	}

	return(0);
}




