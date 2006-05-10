// BodyKinematics.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Frank C. Anderson
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//=============================================================================
// INCLUDES
//=============================================================================
#include <iostream>
#include <string>
#include <OpenSim/Tools/rdMath.h>
#include <OpenSim/Tools/Mtx.h>
#include <OpenSim/Tools/rdTools.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/DerivCallbackSet.h>
#include "BodyKinematics.h"



using namespace OpenSim;
using namespace std;

//=============================================================================
// CONSTANTS
//=============================================================================
#define MAXLEN 2048


//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
BodyKinematics::~BodyKinematics()
{
	if(_dy!=NULL) { delete[] _dy;  _dy=NULL; }
	if(_kin!=NULL) { delete[] _kin; _kin=NULL; }
	deleteStorage();
}
//_____________________________________________________________________________
/**
 * Construct an BodyKinematics instance for recording the kinematics of
 * the bodies of a model during a simulation.
 *
 * @param aModel Model for which the analyses are to be recorded.
 */
BodyKinematics::BodyKinematics(Model *aModel, bool aInDegrees) :
	Analysis(aModel),
	_angVelInLocalFrame(_angVelInLocalFrameProp.getValueBool())
{
	 setNull();

	// STORAGE
	allocateStorage();

	if (_model ==0)
		return;

	// ALLOCATE STATE VECTOR
	_dy = new double[_model->getNY()];
	_kin = new double[6*_model->getNB() + 3];

	// DESCRIPTION AND LABELS
	constructDescription();
	constructColumnLabels();

}
//_____________________________________________________________________________
/**
 * Construct an object from file.
 *
 * The object is constructed from the root element of the XML document.
 * The type of object is the tag name of the XML root element.
 *
 * @param aFileName File name of the document.
 */
BodyKinematics::BodyKinematics(const std::string &aFileName):
Analysis(aFileName),
_angVelInLocalFrame(_angVelInLocalFrameProp.getValueBool())
{
	setNull();

	// Serialize from XML
	updateFromXMLNode();

	/* The rest will be done by setModel().
	// CONSTRUCT DESCRIPTION AND LABELS
	constructDescription();
	constructColumnLabels();

	// STORAGE
	allocateStorage();
	*/
}
//_____________________________________________________________________________
/**
 * Construct an object from an DOMElement.
 */
BodyKinematics::BodyKinematics(DOMElement *aElement):
Analysis(aElement),
_angVelInLocalFrame(_angVelInLocalFrameProp.getValueBool())
{
	setNull();

	// Serialize from XML
	updateFromXMLNode();

	/* The rest will be done by setModel().
	// CONSTRUCT DESCRIPTION AND LABELS
	constructDescription();
	constructColumnLabels();

	// STORAGE
	allocateStorage();
	*/
}

// Copy constrctor and virtual copy 
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 */
BodyKinematics::BodyKinematics(const BodyKinematics &aBodyKinematics):
Analysis(aBodyKinematics),
_angVelInLocalFrame(_angVelInLocalFrameProp.getValueBool())

{
	setNull();
	// COPY TYPE AND NAME
	*this = aBodyKinematics;
}
//_____________________________________________________________________________
/**
 * Clone
 *
 */
Object* BodyKinematics::copy() const
{
	BodyKinematics *object = new BodyKinematics(*this);
	return(object);

}
//_____________________________________________________________________________
/**
 * Instantiate from DOMElement
 *
 */
Object* BodyKinematics::copy(DOMElement *aElement) const
{
	BodyKinematics *object = new BodyKinematics(aElement);
	*object = *this;
	object->updateFromXMLNode();
	return(object);
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
BodyKinematics& BodyKinematics::
operator=(const BodyKinematics &aBodyKinematics)
{
	// BASE CLASS
	Analysis::operator=(aBodyKinematics);
	_angVelInLocalFrame = aBodyKinematics._angVelInLocalFrame;
	return(*this);
}

//_____________________________________________________________________________
/**
 * SetNull().
 */
void BodyKinematics::
setNull()
{

	// POINTERS
	_dy = 0;
	_kin = 0;
	_pStore = NULL;
	_vStore = NULL;
	_aStore = NULL;

	// OTHER VARIABLES

	setType("BodyKinematics");
	//?_body
	setName("BodyKinematics");

	_angVelInLocalFrameProp.setName("angular_velocity_local");
	_angVelInLocalFrameProp.setValue(true);
	_propertySet.append(&_angVelInLocalFrameProp);
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Construct a description for the body kinematics files.
 */
void BodyKinematics::
constructDescription()
{
	char descrip[1024];
	char tmp[MAXLEN];

	strcpy(descrip,"\nThis file contains the kinematics ");
	strcat(descrip,"(positions and orientations,\n");
	strcat(descrip,"velocities and angular velocities, or");
	strcat(descrip," accelerations and angular accelerations)\n");
	strcat(descrip,"of the centers of mass");
	sprintf(tmp," of the body segments in model %s.\n",
		_model->getName().c_str());
	strcat(descrip,tmp);
	strcat(descrip,"\nBody segment orientations are described using");
	strcat(descrip," body-fixed X-Y-Z Euler angles.\n");
	strcat(descrip,"\nAngular velocities and accelerations are given about");
	strcat(descrip," the body-local axes.\n");
	strcat(descrip,"\nUnits are S.I. units (seconds, meters, Newtons, ...)");
	if(getInDegrees()) {
		strcat(descrip,"\nAngles are in degrees.");
	} else {
		strcat(descrip,"\nAngles are in radians.");
	}
	strcat(descrip,"\n\n");

	setDescription(descrip);
}

//_____________________________________________________________________________
/**
 * Construct column labels for the body kinematics files.
 */
void BodyKinematics::
constructColumnLabels()
{
	char labels[MAXLEN];

	// GET STATE NAMES
	int i;
	char name[MAXLEN];
	strcpy(labels,"time");
	for(i=0;i<_model->getNB();i++) {
		sprintf(name,"\t%s_X",_model->getBodyName(i).c_str());
		strcat(labels,name);
		sprintf(name,"\t%s_Y",_model->getBodyName(i).c_str());
		strcat(labels,name);
		sprintf(name,"\t%s_Z",_model->getBodyName(i).c_str());
		strcat(labels,name);
		sprintf(name,"\t%s_Ox",_model->getBodyName(i).c_str());
		strcat(labels,name);
		sprintf(name,"\t%s_Oy",_model->getBodyName(i).c_str());
		strcat(labels,name);
		sprintf(name,"\t%s_Oz",_model->getBodyName(i).c_str());
		strcat(labels,name);
	}

	// ADD NAMES FOR POSITION, VELOCITY, AND ACCELERATION OF WHOLE BODY
	strcat(labels,"\tWholeBody_X\tWholeBody_Y\tWholeBody_Z");

	strcat(labels,"\n");

	setColumnLabels(labels);
}

//_____________________________________________________________________________
/**
 * Allocate storage for the kinematics.
 */
void BodyKinematics::
allocateStorage()
{
	// ACCELERATIONS
	_aStore = new Storage(1000,"Accelerations");
	_aStore->setDescription(getDescription());
	_aStore->setColumnLabels(getColumnLabels());

	// VELOCITIES
	_vStore = new Storage(1000,"Velocities");
	_vStore->setDescription(getDescription());
	_vStore->setColumnLabels(getColumnLabels());

	// POSITIONS
	_pStore = new Storage(1000,"Positions");
	_pStore->setDescription(getDescription());
	_pStore->setColumnLabels(getColumnLabels());
}


//=============================================================================
// DESTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Delete storage objects.
 */
void BodyKinematics::
deleteStorage()
{
	if(_aStore!=NULL) { delete _aStore;  _aStore=NULL; }
	if(_vStore!=NULL) { delete _vStore;  _vStore=NULL; }
	if(_pStore!=NULL) { delete _pStore;  _pStore=NULL; }
}


//=============================================================================
// GET AND SET
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the model for which the body kinematics are to be computed.
 *
 * @param aModel Model pointer
 */
void BodyKinematics::
setModel(Model *aModel)
{
	Analysis::setModel(aModel);

	// ALLOCATIONS
	if (_dy != 0)
		delete[] _dy;
	if (_kin != 0)
		delete[] _kin;

	_dy = new double[_model->getNY()];
	_kin = new double[6*_model->getNB() + 3];

	// DESCRIPTION AND LABELS
	constructDescription();
	constructColumnLabels();

	deleteStorage();
	allocateStorage();
}

//-----------------------------------------------------------------------------
// STORAGE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the acceleration storage.
 *
 * @return Acceleration storage.
 */
Storage* BodyKinematics::
getAccelerationStorage()
{
	return(_aStore);
}
//_____________________________________________________________________________
/**
 * Get the velocity storage.
 *
 * @return Velocity storage.
 */
Storage* BodyKinematics::
getVelocityStorage()
{
	return(_vStore);
}
//_____________________________________________________________________________
/**
 * Get the position storage.
 *
 * @return Position storage.
 */
Storage* BodyKinematics::
getPositionStorage()
{
	return(_pStore);
}

//-----------------------------------------------------------------------------
// STORAGE CAPACITY
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the capacity increments of all storage instances.
 *
 * @param aIncrement Increment by which storage capacities will be increased
 * when storage capcities run out.
 */
void BodyKinematics::
setStorageCapacityIncrements(int aIncrement)
{
	_aStore->setCapacityIncrement(aIncrement);
	_vStore->setCapacityIncrement(aIncrement);
	_pStore->setCapacityIncrement(aIncrement);
}

//-----------------------------------------------------------------------------
// ANGULAR VELOCITY IN LOCAL FRAME FLAG
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set a flag indicating that the angular velocities should be output in the 
 * the body local reference frame.
 *
 * @param aTrueFalse  False will result in the angular velocities being output
 *		in the global reference frame
 */
void BodyKinematics::
setAngVelInLocalFrame(bool aTrueFalse)
{
	_angVelInLocalFrame = aTrueFalse;
}

/**
 * Get a flag indicating whether the angular velocities should be output in the 
 * the body local reference frame.
 *
 * @param rTrueFalse  False indicates the angular velocities being output
 *		in the global reference frame
 */
bool BodyKinematics::
getAngVelInLocalFrame()
{
	return(_angVelInLocalFrame);
}



//=============================================================================
// ANALYSIS
//=============================================================================
//_____________________________________________________________________________
/**
 * Record the kinematics.
 */
int BodyKinematics::
record(double aT,double *aX,double *aY)
{

	// ----------------------------------
	// SET
	_model->set(aT,aX,aY);
	_model->getDerivCallbackSet()->set(aT,aX,aY);

	// ACTUATION
	_model->computeActuation();
	_model->getDerivCallbackSet()->computeActuation(aT,aX,aY);
	_model->applyActuatorForces();
	_model->getDerivCallbackSet()->applyActuation(aT,aX,aY);

	// CONTACT
	_model->computeContact();
	_model->getDerivCallbackSet()->computeContact(aT,aX,aY);
	_model->applyContactForces();
	_model->getDerivCallbackSet()->applyContact(aT,aX,aY);

	// ACCELERATIONS
	int i;
	int nq = _model->getNQ();
	_model->computeAccelerations(&_dy[0],&_dy[nq]);
	// ----------------------------------


	// VARIABLES
	double origin[] = { 0.0, 0.0, 0.0 };
	double dirCos[3][3];
	double vec[3],angVec[3];
	double Mass = 0.0;
	int I;
	int nk = 6*_model->getNB() + 3;

	// POSITION
	double rP[3] = { 0.0, 0.0, 0.0 };
	for(i=0;i<_model->getNB();i++) {

		// GET POSITIONS AND EULER ANGLES
		_model->getPosition(i,origin,vec);
		_model->getDirectionCosines(i,dirCos);
		_model->convertDirectionCosinesToAngles(dirCos,
			&angVec[0],&angVec[1],&angVec[2]);

		// ADD TO WHOLE BODY MASS
		Mass += _model->getMass(i);
		rP[0] += _model->getMass(i) * vec[0];
		rP[1] += _model->getMass(i) * vec[1];
		rP[2] += _model->getMass(i) * vec[2];

		// CONVERT TO DEGREES?
		if(getInDegrees()) {
			angVec[0] *= rdMath::RTD;
			angVec[1] *= rdMath::RTD;
			angVec[2] *= rdMath::RTD;
		}			

		// FILL KINEMATICS ARRAY
		I = Mtx::ComputeIndex(i,6,0);
		memcpy(&_kin[I],vec,3*sizeof(double));
		memcpy(&_kin[I+3],angVec,3*sizeof(double));
	}

	//COMPUTE COM OF WHOLE BODY AND ADD TO ARRAY
	rP[0] /= Mass;
	rP[1] /= Mass;
	rP[2] /= Mass;
	I = Mtx::ComputeIndex(_model->getNB(),6,0);
	memcpy(&_kin[I],rP,3*sizeof(double));
	
	_pStore->append(aT,nk,_kin);

	// VELOCITY
	double rV[3] = { 0.0, 0.0, 0.0 };
	for(i=0;i<_model->getNB();i++) {

		// GET VELOCITIES AND ANGULAR VELOCITIES
		_model->getVelocity(i,origin,vec);
		if(_angVelInLocalFrame){
			_model->getAngularVelocityBodyLocal(i,angVec);
		} else {
			_model->getAngularVelocity(i,angVec);
		}
		rV[0] += _model->getMass(i) * vec[0];
		rV[1] += _model->getMass(i) * vec[1];
		rV[2] += _model->getMass(i) * vec[2];

		// CONVERT TO DEGREES?
		if(getInDegrees()) {
			angVec[0] *= rdMath::RTD;
			angVec[1] *= rdMath::RTD;
			angVec[2] *= rdMath::RTD;
		}			

		// FILL KINEMATICS ARRAY
		I = Mtx::ComputeIndex(i,6,0);
		memcpy(&_kin[I],vec,3*sizeof(double));
		memcpy(&_kin[I+3],angVec,3*sizeof(double));
	}

	//COMPUTE VELOCITY OF COM OF WHOLE BODY AND ADD TO ARRAY
	rV[0] /= Mass;
	rV[1] /= Mass;
	rV[2] /= Mass;
	I = Mtx::ComputeIndex(_model->getNB(),6,0);
	memcpy(&_kin[I],rV,3*sizeof(double));

	_vStore->append(aT,nk,_kin);

	// ACCELERATIONS
	double rA[3] = { 0.0, 0.0, 0.0 };	
	for(i=0;i<_model->getNB();i++) {

		// GET ACCELERATIONS AND ANGULAR ACCELERATIONS
		_model->getAcceleration(i,origin,vec);
		_model->getAngularAccelerationBodyLocal(i,angVec);
		rA[0] += _model->getMass(i) * vec[0];
		rA[1] += _model->getMass(i) * vec[1];
		rA[2] += _model->getMass(i) * vec[2];

		// CONVERT TO DEGREES?
		if(getInDegrees()) {
			angVec[0] *= rdMath::RTD;
			angVec[1] *= rdMath::RTD;
			angVec[2] *= rdMath::RTD;
		}			

		// FILL KINEMATICS ARRAY
		I = Mtx::ComputeIndex(i,6,0);
		memcpy(&_kin[I],vec,3*sizeof(double));
		memcpy(&_kin[I+3],angVec,3*sizeof(double));
	}

	//COMPUTE ACCELERATION OF COM OF WHOLE BODY AND ADD TO ARRAY
	rA[0] /= Mass;
	rA[1] /= Mass;
	rA[2] /= Mass;
	I = Mtx::ComputeIndex(_model->getNB(),6,0);
	memcpy(&_kin[I],rA,3*sizeof(double));

	_aStore->append(aT,nk,_kin);

	//printf("BodyKinematics:\taT:\t%.16f\trA[1]:\t%.16f\n",aT,rA[1]);

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
 * This method should be overriden in the child class.  It is
 * included here so that the child class will not have to implement it if it
 * is not necessary.
 *
 * @param aStep Step number of the integration.
 * @param aDT Size of the time step that will be attempted.
 * @param aT Current time in the integration.
 * @param aX Current control values.
 * @param aY Current states.
 * @param aClientData General use pointer for sending in client data.
 *
 * @return -1 on error, 0 otherwise.
 */
int BodyKinematics::
begin(int aStep,double aDT,double aT,double *aX,double *aY,
		void *aClientData)
{
	if(!proceed()) return(0);

	// RESET STORAGE
	cout<<"BodyKinematics.begin: reseting storage at time "<<aT<<"."<<endl;
	_pStore->reset(aT);
	_vStore->reset(aT);
	_aStore->reset(aT);

	// RECORD
	int status = 0;
	if(_pStore->getSize()<=0) {
		status = record(aT,aX,aY);
	}

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
 * This method should be overriden in derived classes.  It is
 * included here so that the derived class will not have to implement it if
 * it is not necessary.
 *
 * @param aXPrev Controls at the beginining of the current time step.
 * @param aYPrev States at the beginning of the current time step.
 * @param aStep Step number of the integration.
 * @param aDT Size of the time step that was just taken.
 * @param aT Current time in the integration.
 * @param aX Current control values.
 * @param aY Current states.
 * @param aClientData General use pointer for sending in client data.
 *
 * @return -1 on error, 0 otherwise.
 */
int BodyKinematics::
step(double *aXPrev,double *aYPrev,
	int aStep,double aDT,double aT,double *aX,double *aY,
	void *aClientData)
{
	if(!proceed(aStep)) return(0);

	int status = record(aT,aX,aY);

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
 * This method should be overriden in the child class.  It is
 * included here so that the child class will not have to implement it if it
 * is not necessary.
 *
 * @param aStep Step number of the integration.
 * @param aDT Size of the time step that was just completed.
 * @param aT Current time in the integration.
 * @param aX Current control values.
 * @param aY Current states.
 * @param aClientData General use pointer for sending in client data.
 *
 * @return -1 on error, 0 otherwise.
 */
int BodyKinematics::
end(int aStep,double aDT,double aT,double *aX,double *aY,
		void *aClientData)
{
	if(!proceed(aStep)) return(0);
	printf("rdKinematics.end: Finalizing analysis %s.\n",getName().c_str());
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
int BodyKinematics::
printResults(const char *aBaseName,const char *aDir,double aDT,
				 const char *aExtension)
{
	if(aBaseName==NULL) return(-1);

	// CONSTRUCT PATH
	char path[2048],name[2048];
	if(aDir==NULL) {
		strcpy(path,".");
	} else {
		strcpy(path,aDir);
	}

	// ACCELERATIONS
	_aStore->scaleTime(_model->getTimeNormConstant());
	if(aExtension==NULL) {
		sprintf(name,"%s/%s_%s_acc",path,aBaseName,getName().c_str());
	} else {
		sprintf(name,"%s/%s_%s_acc%s",path,aBaseName,getName().c_str(),aExtension);
	}
	if(aDT<=0.0) {
		if(_aStore!=NULL)  _aStore->print(name);
	} else {
		if(_aStore!=NULL)  _aStore->print(name,aDT);
	}

	// VELOCITIES
	_vStore->scaleTime(_model->getTimeNormConstant());
	if(aExtension==NULL) {
		sprintf(name,"%s/%s_%s_vel",path,aBaseName,getName().c_str());
	} else {
		sprintf(name,"%s/%s_%s_vel%s",path,aBaseName,getName().c_str(),aExtension);
	}
	if(aDT<=0.0) {
		if(_vStore!=NULL)  _vStore->print(name);
	} else {
		if(_vStore!=NULL)  _vStore->print(name,aDT);
	}

	// POSITIONS
	_pStore->scaleTime(_model->getTimeNormConstant());
	if(aExtension==NULL) {
		sprintf(name,"%s/%s_%s_pos",path,aBaseName,getName().c_str());
	} else {
		sprintf(name,"%s/%s_%s_pos%s",path,aBaseName,getName().c_str(),aExtension);
	}
	if(aDT<=0.0) {
		if(_pStore!=NULL)  _pStore->print(name);
	} else {
		if(_pStore!=NULL)  _pStore->print(name,aDT);
	}

	return(0);
}


