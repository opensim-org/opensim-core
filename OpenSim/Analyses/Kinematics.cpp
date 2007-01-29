// Kinematics.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Frank C. Anderson
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//=============================================================================
// INCLUDES
//=============================================================================
#include <iostream>
#include <string>
#include <OpenSim/Tools/rdMath.h>
#include <OpenSim/Tools/rdTools.h>
#include <OpenSim/Simulation/SIMM/AbstractModel.h>
#include <OpenSim/Simulation/SIMM/AbstractDynamicsEngine.h>
#include <OpenSim/Simulation/Model/DerivCallbackSet.h>
#include <OpenSim/Simulation/SIMM/SpeedSet.h>
#include "Kinematics.h"



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
Kinematics::~Kinematics()
{
	if(_y!=NULL) { delete[] _y;  _y=NULL; }
	if(_dy!=NULL) { delete[] _dy;  _dy=NULL; }
	deleteStorage();
}
//_____________________________________________________________________________
/**
 * Construct an Kinematics object for recording the kinematics of
 * a model's generalized coodinates during a simulation.
 *
 * @param aModel AbstractModel for which the kinematics are to be recorded.
 */
Kinematics::Kinematics(AbstractModel *aModel) :
	Analysis(aModel),
	_coordinates(_coordinatesProp.getValueStrArray())
{
	// NULL
	setNull();

	// DESCRIPTION
	constructDescription();

	// STORAGE
	allocateStorage();

	// CHECK MODEL
	if(_model==NULL) return;

	// ALLOCATE ARRAYS
	_y = new double[_model->getNumStates()];
	_dy = new double[_model->getNumStates()];

	// LABELS
	updateCoordinatesToRecord();
	constructColumnLabels();
}
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
Kinematics::Kinematics(const std::string &aFileName):
	Analysis(aFileName),
	_coordinates(_coordinatesProp.getValueStrArray())
{
	setNull();

	// Serialize from XML
	updateFromXMLNode();

	// CONSTRUCT DESCRIPTION AND LABELS
	constructDescription();

	// STORAGE
	allocateStorage();
}
//_____________________________________________________________________________
/**
 * Construct an object from an DOMElement.
 */
Kinematics::Kinematics(DOMElement *aElement):
	Analysis(aElement),
	_coordinates(_coordinatesProp.getValueStrArray())
{
	setNull();

	// Serialize from XML
	updateFromXMLNode();

	// CONSTRUCT DESCRIPTION AND LABELS
	constructDescription();
	updateCoordinatesToRecord();
	constructColumnLabels();

	// STORAGE
	allocateStorage();
}

// Copy constrctor and virtual copy 
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 */
Kinematics::Kinematics(const Kinematics &aKinematics):
	Analysis(aKinematics),
	_coordinates(_coordinatesProp.getValueStrArray())
{
	setNull();
	// COPY TYPE AND NAME
	*this = aKinematics;
}
//_____________________________________________________________________________
/**
 * Clone
 *
 */
Object* Kinematics::copy() const
{
	Kinematics *object = new Kinematics(*this);
	return(object);

}
//_____________________________________________________________________________
/**
 * Instantiate from DOMElement
 *
 */
Object* Kinematics::copy(DOMElement *aElement) const
{
	Kinematics *object = new Kinematics(aElement);
	*object = *this;
	object->updateFromXMLNode();
	return(object);
}
//_____________________________________________________________________________
/**
 * Set all member variables to their null or default values.
 */
void Kinematics::
setNull()
{
	setupProperties();

	setType("Kinematics");
	setName("Kinematics");
	_y=0;
	_dy=0;
	_pStore=_vStore=_aStore=0;
	_coordinates.setSize(1);
	_coordinates[0] = "all";
}
//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void Kinematics::
setupProperties()
{
	_coordinatesProp.setComment("Names of generalized coordinates to record kinematics for.  Use 'all' to record all coordinates.");
	_coordinatesProp.setName("coordinates");
	_propertySet.append( &_coordinatesProp );
}

//--------------------------------------------------------------------------
// OPERATORS
//--------------------------------------------------------------------------
Kinematics& Kinematics::operator=(const Kinematics &aKinematics)
{
	// BASE CLASS
	Analysis::operator=(aKinematics);

	// Deallocate _y & _dy if already allocated
	if(_y!=NULL) { delete[] _y;  _y=NULL; }
	if(_dy!=NULL) { delete[] _dy;  _dy=NULL; }

	// STORAGE
	deleteStorage();
	allocateStorage();

	// CHECK MODEL
	if(_model!=NULL) {
		_y = new double[_model->getNumStates()];
		_dy = new double[_model->getNumStates()];
		updateCoordinatesToRecord();
		constructColumnLabels();
	}

	return(*this);
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Allocate storage for the kinematics.
 */
void Kinematics::
allocateStorage()
{
	// ACCELERATIONS
	_aStore = new Storage(1000,"Accelerations");
	_aStore->setDescription(getDescription());
	_storageList.append(_aStore);

	// VELOCITIES
	_vStore = new Storage(1000,"Speeds");
	_vStore->setDescription(getDescription());
	_storageList.append(_vStore);

	// POSITIONS
	_pStore = new Storage(1000,"Coordinates");
	_pStore->setDescription(getDescription());
	_storageList.append(_pStore);
}


//=============================================================================
// DESTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Delete storage objects.
 */
void Kinematics::
deleteStorage()
{
	if(_aStore!=NULL) { delete _aStore;  _aStore=NULL; }
	if(_vStore!=NULL) { delete _vStore;  _vStore=NULL; }
	if(_pStore!=NULL) { delete _pStore;  _pStore=NULL; }
}


//_____________________________________________________________________________
/**
 * Update coordinates to record
 */
void Kinematics::
updateCoordinatesToRecord()
{
	if(!_model) {
		_coordinateIndices.setSize(0);
		_values.setSize(0);
		return;
	}

	SpeedSet *ss = _model->getDynamicsEngine().getSpeedSet();
	_coordinateIndices.setSize(_coordinates.getSize());
	for(int i=0; i<_coordinates.getSize(); i++) {
		if(_coordinates[i] == "all") {
			_coordinateIndices.setSize(ss->getSize());
			for(int j=0;j<ss->getSize();j++) _coordinateIndices[j]=j;
			break;
		}
		string speedName = AbstractSpeed::getSpeedName(_coordinates[i]);
		int index = ss->getIndex(speedName);
		if(index<0) 
			throw Exception("Kinematics: ERR- Cound not find coordinate named '"+_coordinates[i]+"'",__FILE__,__LINE__);
		_coordinateIndices[i] = index;
	}
	_values.setSize(_coordinateIndices.getSize());

	if(_values.getSize()==0) cout << "WARNING: Kinematics analysis has no coordinates to record values for" << endl;
}

//=============================================================================
// GET AND SET
//=============================================================================
//-----------------------------------------------------------------------------
// DESCRIPTION
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Construct the description for the kinematics files.
 */
void Kinematics::
constructDescription()
{
	char descrip[1024];

	strcpy(descrip,"\nUnits are S.I. units (second, meters, Newtons, ...)");
	if(getInDegrees()) {
		strcat(descrip,"\nAngles are in degrees.");
	} else {
		strcat(descrip,"\nAngles are in radians.");
	}
	strcat(descrip,"\n\n");

	setDescription(descrip);
}

//-----------------------------------------------------------------------------
// COLUMN LABELS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Construct the column labels for the kinematics files.
 */
void Kinematics::
constructColumnLabels()
{
	// CHECK FOR NULL
	if (!_model || _model->getDynamicsEngine().getNumSpeeds() == 0)
	{
		setColumnLabels(NULL);
		return;
	}

	string labels = "time";
	SpeedSet *ss = _model->getDynamicsEngine().getSpeedSet();

	for(int i=0; i<_coordinateIndices.getSize(); i++) {
		AbstractSpeed *speed= ss->get(_coordinateIndices[i]);
		labels += "\t" + AbstractSpeed::getCoordinateName(speed->getName());
	}

	labels += "\n";

	setColumnLabels(labels.c_str());
	if (_pStore)
		_pStore->setColumnLabels(getColumnLabels());
	if (_vStore)
		_vStore->setColumnLabels(getColumnLabels());
	if (_aStore)
		_aStore->setColumnLabels(getColumnLabels());
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
Storage* Kinematics::
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
Storage* Kinematics::
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
Storage* Kinematics::
getPositionStorage()
{
	return(_pStore);
}
//_____________________________________________________________________________
/**
 * Set the model pointer for analyzing kinematics.
 */
void Kinematics::setModel(AbstractModel *aModel)
{
	// BASE CLASS
	Analysis::setModel(aModel);

	// DATA MEMBERS
	if(_y!=NULL) { delete[] _y;  _y=NULL; }
	if(_dy!=NULL) { delete[] _dy;  _dy=NULL; }

	if (_model){
		// ALLOCATE STATE VECTOR
		_y = new double[_model->getNumStates()];
		_dy = new double[_model->getNumStates()];
	}

	// UPDATE LABELS
	updateCoordinatesToRecord();
	constructColumnLabels();
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
void Kinematics::
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
 *
 * @return 0 of success, -1 on error.
 */
int Kinematics::
record(double aT,double *aX,double *aY)
{
	// NUMBERS
	int nq = _model->getNumCoordinates();
	int nu = _model->getNumSpeeds();
	int ny = _model->getNumStates();

	// COMPUTE DERIVATIVES
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
	_model->getDynamicsEngine().computeDerivatives(_dy,&_dy[nq]);
	// ----------------------------------


	// CONVERT RESULTS TO ANGLES
	memcpy(_y,aY,ny*sizeof(double));
	_model->getDynamicsEngine().convertQuaternionsToAngles(_y,_y);

	// CONVERT TO DEGREES
	if(getInDegrees()) {
		_model->getDynamicsEngine().convertRadiansToDegrees(_y,_y);
		_model->getDynamicsEngine().convertRadiansToDegrees(&_y[nq],&_y[nq]);
		_model->getDynamicsEngine().convertRadiansToDegrees(&_dy[nq],&_dy[nq]);
	}

	// RECORD RESULTS
	int nvalues = _coordinateIndices.getSize();
	for(int i=0;i<nvalues;i++) _values[i] = _y[_coordinateIndices[i]];
	_pStore->append(aT,nvalues,&_values[0]);
	for(int i=0;i<nvalues;i++) _values[i] = _y[nq+_coordinateIndices[i]];
	_vStore->append(aT,nvalues,&_values[0]);
	for(int i=0;i<nvalues;i++) _values[i] = _dy[nq+_coordinateIndices[i]];
	_aStore->append(aT,nvalues,&_values[0]);

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
int Kinematics::
begin(int aStep,double aDT,double aT,double *aX,double *aY,
		void *aClientData)
{
	if(!proceed()) return(0);

	// RESET STORAGE
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
int Kinematics::
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
int Kinematics::
end(int aStep,double aDT,double aT,double *aX,double *aY,
		void *aClientData)
{
	if (!proceed()) return 0;

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
int Kinematics::
printResults(const string &aBaseName,const string &aDir,double aDT,
				 const string &aExtension)
{
	if(!getOn()) {
		printf("Kinematics.printResults: Off- not printing.\n");
		return(0);
	}

	// ACCELERATIONS
	_aStore->scaleTime(_model->getTimeNormConstant());
	Storage::printResult(_aStore,aBaseName+"_"+getName()+"_dudt",aDir,aDT,aExtension);

	// VELOCITIES
	_vStore->scaleTime(_model->getTimeNormConstant());
	Storage::printResult(_vStore,aBaseName+"_"+getName()+"_u",aDir,aDT,aExtension);

	// POSITIONS
	_pStore->scaleTime(_model->getTimeNormConstant());
	Storage::printResult(_pStore,aBaseName+"_"+getName()+"_q",aDir,aDT,aExtension);

	// POSITIONS (.mot file)
	// For now (until we resolve the .sto vs .mot issue) also write
	// an .mot file so the motion can be viewed in SIMM -- Eran.
	bool writeSIMMHeader=_pStore->getWriteSIMMHeader();
	_pStore->setWriteSIMMHeader(true);
	Storage::printResult(_pStore,aBaseName+"_"+getName()+"_q",aDir,aDT,".mot");
	_pStore->setWriteSIMMHeader(writeSIMMHeader);

	return(0);
}


ArrayPtrs<Storage>& Kinematics::getStorageList()
{
	return _storageList;
}

