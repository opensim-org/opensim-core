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
#include <OpenSim/Simulation/Model/DerivCallbackSet.h>
#include <OpenSim/Simulation/SIMM/AbstractModel.h>
#include <OpenSim/Simulation/SIMM/AbstractDynamicsEngine.h>
#include <OpenSim/Simulation/SIMM/BodySet.h>
#include "BodyKinematics.h"



using namespace OpenSim;
using namespace std;

//=============================================================================
// CONSTANTS
//=============================================================================
#define MAXLEN 10000


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
 * @param aModel AbstractModel for which the analyses are to be recorded.
 */
BodyKinematics::BodyKinematics(AbstractModel *aModel, bool aInDegrees) :
	Analysis(aModel),
	_angVelInLocalFrame(_angVelInLocalFrameProp.getValueBool())
{
	 setNull();

	// STORAGE
	allocateStorage();

	if (_model ==0)
		return;

	// ALLOCATE STATE VECTOR
	_dy = new double[_model->getNumStates()];
	_kin = new double[6*_model->getNumBodies() + 3];

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
	string labels = "time";
	BodySet *bs = _model->getDynamicsEngine().getBodySet();
	int i=0;

	for (i=0; i< bs->getSize(); i++)
	{
		AbstractBody *body = bs->get(i);
		labels += "\t" + body->getName() + "_X";
		labels += "\t" + body->getName() + "_Y";
		labels += "\t" + body->getName() + "_Z";
		labels += "\t" + body->getName() + "_Ox";
		labels += "\t" + body->getName() + "_Oy";
		labels += "\t" + body->getName() + "_Oz";
	}

	// ADD NAMES FOR POSITION, VELOCITY, AND ACCELERATION OF WHOLE BODY
	labels += "\tWholeBody_X\tWholeBody_Y\tWholeBody_Z\n";

	setColumnLabels(labels.c_str());
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
 * @param aModel AbstractModel pointer
 */
void BodyKinematics::
setModel(AbstractModel *aModel)
{
	Analysis::setModel(aModel);

	// ALLOCATIONS
	if (_dy != 0)
		delete[] _dy;
	if (_kin != 0)
		delete[] _kin;

	_dy = new double[_model->getNumStates()];
	_kin = new double[6*_model->getNumBodies() + 3];

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
	_model->getActuatorSet()->computeActuation();
	_model->getDerivCallbackSet()->computeActuation(aT,aX,aY);
	_model->getActuatorSet()->apply();
	_model->getDerivCallbackSet()->applyActuation(aT,aX,aY);

	// CONTACT
	_model->getContactSet()->computeContact();
	_model->getDerivCallbackSet()->computeContact(aT,aX,aY);
	_model->getContactSet()->apply();
	_model->getDerivCallbackSet()->applyContact(aT,aX,aY);

	// ACCELERATIONS
	int nq = _model->getNumCoordinates();
	_model->getDynamicsEngine().computeDerivatives(&_dy[0],&_dy[nq]);
	// ----------------------------------


	// VARIABLES
	double dirCos[3][3];
	double vec[3],angVec[3];
	double Mass = 0.0;
	int I;
	int nk = 6*_model->getNumBodies() + 3;

	// POSITION
	double rP[3] = { 0.0, 0.0, 0.0 };
	int bodyIndex = 0;
	BodySet *bs = _model->getDynamicsEngine().getBodySet();
	int i=0;

	for (i=0; i< bs->getSize(); i++)
	{
		AbstractBody *body = bs->get(i);
		double com[3];
		body->getMassCenter(com);
		// GET POSITIONS AND EULER ANGLES
		_model->getDynamicsEngine().getPosition(*body,com,vec);
		_model->getDynamicsEngine().getDirectionCosines(*body,dirCos);
		_model->getDynamicsEngine().convertDirectionCosinesToAngles(dirCos,
			&angVec[0],&angVec[1],&angVec[2]);

		// ADD TO WHOLE BODY MASS
		Mass += body->getMass();
		rP[0] += body->getMass() * vec[0];
		rP[1] += body->getMass() * vec[1];
		rP[2] += body->getMass() * vec[2];

		// CONVERT TO DEGREES?
		if(getInDegrees()) {
			angVec[0] *= rdMath::RTD;
			angVec[1] *= rdMath::RTD;
			angVec[2] *= rdMath::RTD;
		}			

		// FILL KINEMATICS ARRAY
		I = Mtx::ComputeIndex(bodyIndex++,6,0);
		memcpy(&_kin[I],vec,3*sizeof(double));
		memcpy(&_kin[I+3],angVec,3*sizeof(double));
	}

	//COMPUTE COM OF WHOLE BODY AND ADD TO ARRAY
	rP[0] /= Mass;
	rP[1] /= Mass;
	rP[2] /= Mass;
	I = Mtx::ComputeIndex(_model->getNumBodies(),6,0);
	memcpy(&_kin[I],rP,3*sizeof(double));
	
	_pStore->append(aT,nk,_kin);

	// VELOCITY
	double rV[3] = { 0.0, 0.0, 0.0 };
	bodyIndex = 0;
	for (i=0; i< bs->getSize(); i++)
	{
		AbstractBody *body = bs->get(i);
		double com[3];
		body->getMassCenter(com);
		// GET VELOCITIES AND ANGULAR VELOCITIES
		_model->getDynamicsEngine().getVelocity(*body,com,vec);
		if(_angVelInLocalFrame){
			_model->getDynamicsEngine().getAngularVelocityBodyLocal(*body,angVec);
		} else {
			_model->getDynamicsEngine().getAngularVelocity(*body,angVec);
		}
		rV[0] += body->getMass() * vec[0];
		rV[1] += body->getMass() * vec[1];
		rV[2] += body->getMass() * vec[2];

		// CONVERT TO DEGREES?
		if(getInDegrees()) {
			angVec[0] *= rdMath::RTD;
			angVec[1] *= rdMath::RTD;
			angVec[2] *= rdMath::RTD;
		}			

		// FILL KINEMATICS ARRAY
		I = Mtx::ComputeIndex(bodyIndex++,6,0);
		memcpy(&_kin[I],vec,3*sizeof(double));
		memcpy(&_kin[I+3],angVec,3*sizeof(double));
	}

	//COMPUTE VELOCITY OF COM OF WHOLE BODY AND ADD TO ARRAY
	rV[0] /= Mass;
	rV[1] /= Mass;
	rV[2] /= Mass;
	I = Mtx::ComputeIndex(_model->getNumBodies(),6,0);
	memcpy(&_kin[I],rV,3*sizeof(double));

	_vStore->append(aT,nk,_kin);

	// ACCELERATIONS
	double rA[3] = { 0.0, 0.0, 0.0 };
	bodyIndex = 0;
	for (i=0; i< bs->getSize(); i++)
	{
		AbstractBody *body = bs->get(i);
		double com[3];
		body->getMassCenter(com);
		// GET ACCELERATIONS AND ANGULAR ACCELERATIONS
		_model->getDynamicsEngine().getAcceleration(*body,com,vec);
		_model->getDynamicsEngine().getAngularAccelerationBodyLocal(*body,angVec);
		rA[0] += body->getMass() * vec[0];
		rA[1] += body->getMass() * vec[1];
		rA[2] += body->getMass() * vec[2];

		// CONVERT TO DEGREES?
		if(getInDegrees()) {
			angVec[0] *= rdMath::RTD;
			angVec[1] *= rdMath::RTD;
			angVec[2] *= rdMath::RTD;
		}			

		// FILL KINEMATICS ARRAY
		I = Mtx::ComputeIndex(bodyIndex++,6,0);
		memcpy(&_kin[I],vec,3*sizeof(double));
		memcpy(&_kin[I+3],angVec,3*sizeof(double));
	}

	//COMPUTE ACCELERATION OF COM OF WHOLE BODY AND ADD TO ARRAY
	rA[0] /= Mass;
	rA[1] /= Mass;
	rA[2] /= Mass;
	I = Mtx::ComputeIndex(_model->getNumBodies(),6,0);
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
 * AbstractModel::integBeginCallback() and has the same argument list.
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
 * AbstractModel::integStepCallback(), which has the same argument list.
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

	record(aT,aX,aY);

	return(0);
}
//_____________________________________________________________________________
/**
 * This method is called at the end of an analysis so that any
 * necessary finalizations may be performed.
 *
 * This method is meant to be called at the end of an integration in
 * AbstractModel::integEndCallback() and has the same argument list.
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
	if(!proceed()) return(0);

	record(aT,aX,aY);

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
printResults(const string &aBaseName,const string &aDir,double aDT,
				 const string &aExtension)
{
	// ACCELERATIONS
	_aStore->scaleTime(_model->getTimeNormConstant());
	Storage::printResult(_aStore,aBaseName+"_"+getName()+"_acc",aDir,aDT,aExtension);

	// VELOCITIES
	_vStore->scaleTime(_model->getTimeNormConstant());
	Storage::printResult(_vStore,aBaseName+"_"+getName()+"_vel",aDir,aDT,aExtension);

	// POSITIONS
	_pStore->scaleTime(_model->getTimeNormConstant());
	Storage::printResult(_pStore,aBaseName+"_"+getName()+"_pos",aDir,aDT,aExtension);

	return(0);
}


