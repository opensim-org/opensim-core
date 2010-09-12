// Kinematics.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Frank C. Anderson
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/* Copyright (c)  2006 Stanford University
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


//=============================================================================
// INCLUDES
//=============================================================================
#include <iostream>
#include <string>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/SimbodyEngine/SimbodyEngine.h>
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
	if(_q!=NULL) { delete[] _q;  _q=NULL; }
	if(_u!=NULL) { delete[] _u;  _u=NULL; }
	if(_udot!=NULL) { delete[] _udot;  _udot=NULL; }
	deleteStorage();
}
//_____________________________________________________________________________
/**
 * Construct an Kinematics object for recording the kinematics of
 * a model's generalized coodinates during a simulation.
 *
 * @param aModel Model for which the kinematics are to be recorded.
 */
Kinematics::Kinematics(Model *aModel) :
	Analysis(aModel),
	_coordinates(_coordinatesProp.getValueStrArray())
{
	// NULL
	setNull();

	// DESCRIPTION
	constructDescription();

	// CHECK MODEL
	if(aModel==NULL) return;
	setModel(*aModel);

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
	Analysis(aFileName, false),
	_coordinates(_coordinatesProp.getValueStrArray())
{
	setNull();

	// Serialize from XML
	updateFromXMLNode();

	// CONSTRUCT DESCRIPTION AND LABELS
	constructDescription();

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
 * Set all member variables to their null or default values.
 */
void Kinematics::
setNull()
{
	setupProperties();

	setType("Kinematics");
	setName("Kinematics");
	_q=0;
	_u=0;
	_udot=0;
	_pStore=_vStore=_aStore=0;
	_coordinates.setSize(1);
	_coordinates[0] = "all";

	_recordAccelerations = true;
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

	_recordAccelerations = aKinematics._recordAccelerations;
	_coordinates = aKinematics._coordinates;

	// Deallocate if already allocated
	if(_q!=NULL) {    delete[] _q;     _q=NULL; }
	if(_u!=NULL) {    delete[] _u;     _u=NULL; }
	if(_udot!=NULL) { delete[] _udot;  _udot=NULL; }

	// STORAGE
	deleteStorage();
	allocateStorage();

	// CHECK MODEL
	if(_model!=NULL) {
		_q = new double[_model->getNumCoordinates()];
		_u = new double[_model->getNumSpeeds()];
		_udot = new double[_model->getNumSpeeds()];
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
	if(_recordAccelerations) {
		_aStore = new Storage(1000,"Accelerations");
		_aStore->setDescription(getDescription());
		_storageList.append(_aStore);
	}

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

	const CoordinateSet& coordSet = _model->getCoordinateSet();
	_coordinateIndices.setSize(_coordinates.getSize());
	for(int i=0; i<_coordinates.getSize(); i++) {
		if(_coordinates[i] == "all") {
			_coordinateIndices.setSize(coordSet.getSize());
			for(int j=0;j<coordSet.getSize();j++) _coordinateIndices[j]=j;
			break;
		}
		
		int index = coordSet.getIndex(_coordinates[i]);
		if(index<0) 
			throw Exception("Kinematics: ERR- Cound not find coordinate named '"+_coordinates[i]+"'",__FILE__,__LINE__);
		_coordinateIndices[i] = index;
	}
	_values.setSize(_coordinateIndices.getSize());

	if(_values.getSize()==0) {
         cout << "WARNING: Kinematics analysis has no coordinates to record values for" << endl;
    }
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
	if (!_model || _model->getNumSpeeds() == 0)
	{
		setColumnLabels(Array<string>());
		return;
	}

	Array<string> labels;
	labels.append("time");
	const CoordinateSet& cs = _model->getCoordinateSet();
	for(int i=0; i<_coordinateIndices.getSize(); i++) {
		labels.append(cs.get(_coordinateIndices[i]).getName());
	}

	setColumnLabels(labels);
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
void Kinematics::setModel(Model& aModel)
{
	// BASE CLASS
	Analysis::setModel(aModel);

	allocateStorage();

	// DATA MEMBERS
	if(_q!=NULL) {    delete[] _q;     _q=NULL; }
	if(_u!=NULL) {    delete[] _u;     _u=NULL; }
	if(_udot!=NULL) { delete[] _udot;  _udot=NULL; }

	if (_model){
		// ALLOCATE STATE VECTOR
		_q    = new double[_model->getNumCoordinates()];
		_u    = new double[_model->getNumSpeeds()];
		_udot = new double[_model->getNumSpeeds()];
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
record(const SimTK::State& s)
{
    int i;
	// NUMBERS
	//
	int nq = _model->getNumCoordinates();
	int nu = _model->getNumSpeeds();

/*
printf("Kinematics::record t=%14.10f y=",s.getTime());
for(i=0;i<nq;i++) printf(" %f",s.getQ()[i]);
for(i=0;i<nu;i++) printf(" %f",s.getU()[i]);
printf("\n");
*/

    for(i=0;i<nq;i++) _q[i] = s.getQ()[i]; 

	if(_recordAccelerations){
		_model->getMultibodySystem().realize(s, SimTK::Stage::Acceleration);
			for(i=0;i<nu;i++)  {
				_u[i] = s.getU()[i]; 
				_udot[i] = s.getUDot()[i];
			}
    }
	else{
		_model->getMultibodySystem().realize(s, SimTK::Stage::Velocity);
		for(i=0;i<nu;i++)  {
           _u[i] = s.getU()[i];
		}
    }

	// CONVERT TO DEGREES
	if(getInDegrees()) {
		_model->getSimbodyEngine().convertRadiansToDegrees(_q,_q);
		_model->getSimbodyEngine().convertRadiansToDegrees(_u,_u);
		if(_recordAccelerations) _model->getSimbodyEngine().convertRadiansToDegrees(_udot, _udot);
	}

	// RECORD RESULTS
	int nvalues = _coordinateIndices.getSize();
	for(int i=0;i<nvalues;i++)  _values[i] = _q[_coordinateIndices[i]];
	_pStore->append(s.getTime(),nvalues,&_values[0]);

	for(int i=0;i<nvalues;i++) _values[i] = _u[_coordinateIndices[i]];
	_vStore->append(s.getTime(),nvalues,&_values[0]);

	if(_recordAccelerations) {
		for(int i=0;i<nvalues;i++) _values[i] = _udot[_coordinateIndices[i]];
		_aStore->append(s.getTime(),nvalues,&_values[0]);
	}


	return(0);
}
//_____________________________________________________________________________
/**
 * This method is called at the beginning of an analysis so that any
 * necessary initializations may be performed.
 *
 * This method is meant to be called at the begining of an integration 
 *
 * @param s current state of System
 *
 * @return -1 on error, 0 otherwise.
 */
int Kinematics::
begin( SimTK::State& s )
{
	if(!proceed()) return(0);

	// RESET STORAGE
	_pStore->reset(s.getTime());
	_vStore->reset(s.getTime());
	_aStore->reset(s.getTime());

	// RECORD
	int status = 0;
	if(_pStore->getSize()<=0) {
        if(_recordAccelerations && s.getSystemStage() < SimTK::Stage::Acceleration ) 
            _model->getMultibodySystem().realize(s, SimTK::Stage::Acceleration);
		else
			_model->getMultibodySystem().realize(s, SimTK::Stage::Velocity);

		status = record(s);
	}

	return(status);
}
//_____________________________________________________________________________
/**
 * This method is called to perform the analysis.  It can be called during
 * the execution of a forward integrations or after the integration by
 * feeding it the necessary data.
 *
 * When called during an integration, this method is meant to be called 
 *
 * This method should be overriden in derived classes.  It is
 * included here so that the derived class will not have to implement it if
 * it is not necessary.
 *
 * @param s current state of system
 *
 * @return -1 on error, 0 otherwise.
 */
int Kinematics::
step(const SimTK::State& s, int stepNumber )
{
	if(proceed(stepNumber) && getOn()) record(s);

	return(0);
}
//_____________________________________________________________________________
/**
 * This method is called at the end of an analysis so that any
 * necessary finalizations may be performed.
 *
 * This method is meant to be called at the end of an integration 
 *
 * This method should be overriden in the child class.  It is
 * included here so that the child class will not have to implement it if it
 * is not necessary.
 *
 * @param s current state of System
 *
 * @return -1 on error, 0 otherwise.
 */
int Kinematics::
end( SimTK::State& s )
{
	if (!proceed()) return 0;

	record(s);

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
	if(_recordAccelerations) {
		Storage::printResult(_aStore,aBaseName+"_"+getName()+"_dudt",aDir,aDT,aExtension);
	}

	// VELOCITIES
	Storage::printResult(_vStore,aBaseName+"_"+getName()+"_u",aDir,aDT,aExtension);

	// POSITIONS
	Storage::printResult(_pStore,aBaseName+"_"+getName()+"_q",aDir,aDT,aExtension);

	// POSITIONS (.mot file)
	// For now (until we resolve the .sto vs .mot issue) also write
	// an .mot file so the motion can be viewed in SIMM -- Eran.
	bool writeSIMMHeader=_pStore->getWriteSIMMHeader();
	_pStore->setWriteSIMMHeader(true);
	Storage::printResult(_pStore,aBaseName,aDir,aDT,"");
	_pStore->setWriteSIMMHeader(writeSIMMHeader);

	return(0);
}
