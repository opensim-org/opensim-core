// PointKinematics.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Frank C. Anderson
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//=============================================================================
// INCLUDES
//=============================================================================
#include <string>
#include <OpenSim/Common/rdMath.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/BodySet.h>
#include <OpenSim/Simulation/Model/AbstractDynamicsEngine.h>
#include <OpenSim/Simulation/Model/DerivCallbackSet.h>
#include "PointKinematics.h"




using namespace OpenSim;
using namespace std;


//=============================================================================
// CONSTANTS
//=============================================================================
const int PointKinematics::NAME_LENGTH = PointKinematicsNAME_LENGTH;
const int PointKinematics::BUFFER_LENGTH = PointKinematicsBUFFER_LENGTH;


//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
PointKinematics::~PointKinematics()
{
	if(_dy!=NULL) { delete[] _dy;  _dy=NULL; }
	deleteStorage();
}
//_____________________________________________________________________________
/**
 * Construct an PointKinematics instance for recording the kinematics of
 * the bodies of a model during a simulation. Also serves as a default constructor
 *
 * @param aModel Model for which the analyses are to be recorded.
 */
PointKinematics::PointKinematics(Model *aModel) :
Analysis(aModel),
_body(NULL),
_bodyName(_bodyNameProp.getValueStr()),
_point(_pointProp.getValueDblArray()),
_pointName(_pointNameProp.getValueStr())
{
	// NULL
	setNull();

	// STORAGE
	allocateStorage();

	if (aModel==0)
		return;

	// Map name to index
	_body = aModel->getDynamicsEngine().getBodySet()->get(_bodyName);

	// ALLOCATIONS
	if (_dy != 0)
		delete[] _dy;

	_dy = new double[_model->getNumStates()];

	// DESCRIPTION AND LABELS
	constructDescription();
	constructColumnLabels();


}


//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//=============================================================================
// Object Overrides
//=============================================================================
//_____________________________________________________________________________
/**
 * Construct an object from file.
 *
 * The object is constructed from the root element of the XML document.
 * The type of object is the tag name of the XML root element.
 *
 * @param aFileName File name of the document.
 */
PointKinematics::PointKinematics(const std::string &aFileName):
Analysis(aFileName, false),
_body(NULL),
_bodyName(_bodyNameProp.getValueStr()),
_point(_pointProp.getValueDblArray()),
_pointName(_pointNameProp.getValueStr())
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
PointKinematics::PointKinematics(const PointKinematics &aPointKinematics):
Analysis(aPointKinematics),
_body(aPointKinematics._body),
_bodyName(_bodyNameProp.getValueStr()),
_point(_pointProp.getValueDblArray()),
_pointName(_pointNameProp.getValueStr())
{
	setNull();
	// COPY TYPE AND NAME
	*this = aPointKinematics;
}
//_____________________________________________________________________________
/**
 * Clone
 *
 */
Object* PointKinematics::copy() const
{
	PointKinematics *object = new PointKinematics(*this);
	return(object);

}
//_____________________________________________________________________________
/**
 * SetNull().
 */
void PointKinematics::
setNull()
{

	// POINTERS
	_dy = NULL;
	_kin = NULL;
	_pStore = NULL;
	_vStore = NULL;
	_aStore = NULL;

	// OTHER VARIABLES

	// TYPE
	setType("PointKinematics");
	//?_body
	setName("PointKinematics");

	// POINT INFORMATION
	_point.setSize(3);
	Array<double> zero3(0.0, 3);	

	_bodyNameProp.setName("body_name");
	_bodyNameProp.setValue("ground");
	_propertySet.append( &_bodyNameProp );

	_pointNameProp.setName("point_name");
	_pointNameProp.setValue("NONAME");
	_propertySet.append( &_pointNameProp );

	_pointProp.setName("point");
	_pointProp.setValue(zero3);
	_propertySet.append( &_pointProp );
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
PointKinematics& PointKinematics::
operator=(const PointKinematics &aPointKinematics)
{
	// BASE CLASS
	Analysis::operator=(aPointKinematics);
	_body = aPointKinematics._body;
	_point = aPointKinematics._point;
	_pointName = aPointKinematics._pointName;
	_bodyName = aPointKinematics._bodyName;

	// STORAGE
	deleteStorage();
	allocateStorage();

	return(*this);
}
//_____________________________________________________________________________
/**
 * Construct a description for the body kinematics files. (needs a model)
 */
void PointKinematics::
constructDescription()
{
	char descrip[BUFFER_LENGTH];
	char tmp[BUFFER_LENGTH];

	strcpy(descrip,"\nThis file contains the kinematics ");
	strcat(descrip,"(position, velocity, or acceleration) of\n");
	sprintf(tmp,"point (%lf, %lf, %lf) on the %s of model %s.\n",
		_point[0],_point[1],_point[2],_body->getName().c_str(),
		_model->getName().c_str());
	strcat(descrip,tmp);
	strcat(descrip,"\nUnits are S.I. units (seconds, meters, Newtons,...)\n\n");

	setDescription(descrip);
}

//_____________________________________________________________________________
/**
 * Construct column labels for the body kinematics files.
 */
void PointKinematics::
constructColumnLabels()
{
	Array<string> labels;
	labels.append("time");
	labels.append(getPointName() + "_X");
	labels.append(getPointName() + "_Y");
	labels.append(getPointName() + "_Z");
	setColumnLabels(labels);
}

//_____________________________________________________________________________
/**
 * Allocate storage for the kinematics.
 */
void PointKinematics::
allocateStorage()
{
	// ACCELERATIONS
	_aStore = new Storage(1000,"PointAcceleration");
	_aStore->setDescription(getDescription());
	_aStore->setColumnLabels(getColumnLabels());

	// VELOCITIES
	_vStore = new Storage(1000,"PointVelocity");
	_vStore->setDescription(getDescription());
	_vStore->setColumnLabels(getColumnLabels());

	// POSITIONS
	_pStore = new Storage(1000,"PointPosition");
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
void PointKinematics::
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
 * Set the model for which the point kinematics are to be computed.
 *
 * @param aModel Model pointer
 */
void PointKinematics::
setModel(Model *aModel)
{
	Analysis::setModel(aModel);

	// Map name to index
	_body = aModel->getDynamicsEngine().getBodySet()->get(_bodyName);

	// ALLOCATIONS
	if (_dy != 0)
		delete[] _dy;

	_dy = new double[_model->getNumStates()];

	// DESCRIPTION AND LABELS
	constructDescription();
	constructColumnLabels();

}
//-----------------------------------------------------------------------------
// BODY
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the body for which the point kinematics are to be computed and the point on the body
 * represented in local frame. Both params are required to avoid the limbo state where either body
 * or point are undefined..
 *
 * @param aBody Body name
 * @double[3] aPoint point coordinates
 */
void PointKinematics::
setBodyPoint(const std::string& aBody, double aPoint[3])
{
	if (_model == 0)
		return;
	setBody(_model->getDynamicsEngine().getBodySet()->get(aBody));
	setPoint(aPoint);

}
//_____________________________________________________________________________
/**
 * Set the body for which the induced accelerations are to be computed.
 *
 * @param aBody Body ID
 */
void PointKinematics::
setBody(AbstractBody* aBody)
{
	// CHECK
	if (aBody==NULL) {
		printf("PointKinematics.setBody:  WARN- invalid body pointer %p.\n", aBody);
		_body = NULL;
		return;
	}

	// SET
	_body = aBody;
	_bodyName = _body->getName();
	cout<<"PointKinematics.setBody: set body to "<<_bodyName<<endl;

	// RESET STORAGE
	if(_aStore!=NULL) {
		constructDescription();
		_aStore->reset();
		_vStore->reset();
		_pStore->reset();
		_aStore->setDescription(getDescription());
		_vStore->setDescription(getDescription());
		_pStore->setDescription(getDescription());
	}
}
//_____________________________________________________________________________
/**
 * Get the body for which the induced accelerations are to be computed.
 *
 * @return Body ID
 */
AbstractBody* PointKinematics::
getBody()
{
	return(_body);
}

//-----------------------------------------------------------------------------
// POINT
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the point for which the induced accelerations are to be computed.
 *
 * @param aPoint X-Y-Z Point
 */
void PointKinematics::
setPoint(double aPoint[3])
{
	_point[0] = aPoint[0];
	_point[1] = aPoint[1];
	_point[2] = aPoint[2];

	// RESET STORAGE
	if(_aStore!=NULL) {
		constructDescription();
		_aStore->reset();
		_vStore->reset();
		_pStore->reset();
		_aStore->setDescription(getDescription());
		_vStore->setDescription(getDescription());
		_pStore->setDescription(getDescription());
	}
}
//_____________________________________________________________________________
/**
 * Get the point for which the induced accelerations are to be computed.
 *
 * @param rPoint X-Y-Z Point
 */
void PointKinematics::
getPoint(double rPoint[3])
{
	rPoint[0] = _point[0];
	rPoint[1] = _point[1];
	rPoint[2] = _point[2];
}

//-----------------------------------------------------------------------------
// POINT NAME
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set a name for the point.
 *
 * @param aName Name for the point.
 */
void PointKinematics::
setPointName(const char *aName)
{
	_pointName = string(aName);
	constructColumnLabels();
	if(_aStore!=NULL) _aStore->setColumnLabels(getColumnLabels());
	if(_vStore!=NULL) _vStore->setColumnLabels(getColumnLabels());
	if(_pStore!=NULL) _pStore->setColumnLabels(getColumnLabels());
}
//_____________________________________________________________________________
/**
 * Get the point name.
 *
 * @param aName Name for the point.
 */
const std::string &PointKinematics::
getPointName()
{
	return(_pointName);
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
Storage* PointKinematics::
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
Storage* PointKinematics::
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
Storage* PointKinematics::
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
void PointKinematics::
setStorageCapacityIncrements(int aIncrement)
{
	_aStore->setCapacityIncrement(aIncrement);
	_vStore->setCapacityIncrement(aIncrement);
	_pStore->setCapacityIncrement(aIncrement);
}



//=============================================================================
// ANALYSIS
//=============================================================================
//_____________________________________________________________________________
/**
 * Record the kinematics.
 */
int PointKinematics::
record(double aT,double *aX,double *aY)
{
	AbstractDynamicsEngine& de = _model->getDynamicsEngine();

	// COMPUTE ACCELERATIONS
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
	de.computeDerivatives(_dy,&_dy[_model->getNumCoordinates()]);
	// ----------------------------------

	// VARIABLES
	double vec[3];

	// POSITION
	de.getPosition(*_body,_point.get(),vec);
	_pStore->append(aT,3,vec);

	// VELOCITY
	de.getVelocity(*_body,_point.get(),vec);
	_vStore->append(aT,3,vec);

	// ACCELERATIONS
	de.getAcceleration(*_body,_point.get(),vec);
	_aStore->append(aT,3,vec);

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
 * @param aYP Current pseudo states.
 * @param aDYDT Current state derivatives.
 * @param aClientData General use pointer for sending in client data.
 *
 * @return -1 on error, 0 otherwise.
 */
int PointKinematics::
begin(int aStep,double aDT,double aT,double *aX,double *aY,double *aYP,double *aDYDT,
		void *aClientData)
{
	if(!proceed()) return(0);

	// RESET STORAGE
	_pStore->reset(aT);
	_vStore->reset(aT);
	_aStore->reset(aT);

	int status = record(aT,aX,aY);

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
 * @param aYPPrev Pseudo states at the beginning of the current time step.
 * @param aStep Step number of the integration.
 * @param aDT Size of the time step that was just taken.
 * @param aT Current time in the integration.
 * @param aX Current control values.
 * @param aY Current states.
 * @param aYP Current pseudo states.
 * @param aDYDT Current state derivatives.
 * @param aClientData General use pointer for sending in client data.
 *
 * @return -1 on error, 0 otherwise.
 */
int PointKinematics::
step(double *aXPrev,double *aYPrev,double *aYPPrev,
	int aStep,double aDT,double aT,double *aX,double *aY,double *aYP,double *aDYDT,
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
 * @param aYP Current pseudo states.
 * @param aDYDT Current state derivatives.
 * @param aClientData General use pointer for sending in client data.
 *
 * @return -1 on error, 0 otherwise.
 */
int PointKinematics::
end(int aStep,double aDT,double aT,double *aX,double *aY,double *aYP,double *aDYDT,
		void *aClientData)
{
	if(!proceed()) return(0);
	record(aT,aX,aY);
	cout<<"PointKinematics.end: Finalizing analysis "<<getName()<<".\n";
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
int PointKinematics::
printResults(const string &aBaseName,const string &aDir,double aDT,
				 const string &aExtension)
{
	// ACCELERATIONS
	_aStore->scaleTime(_model->getTimeNormConstant());
	Storage::printResult(_aStore,aBaseName+"_"+getName()+"_"+getPointName()+"_acc",aDir,aDT,aExtension);

	// VELOCITIES
	_vStore->scaleTime(_model->getTimeNormConstant());
	Storage::printResult(_vStore,aBaseName+"_"+getName()+"_"+getPointName()+"_vel",aDir,aDT,aExtension);

	// POSITIONS
	_pStore->scaleTime(_model->getTimeNormConstant());
	Storage::printResult(_pStore,aBaseName+"_"+getName()+"_"+getPointName()+"_pos",aDir,aDT,aExtension);

	return(0);
}


