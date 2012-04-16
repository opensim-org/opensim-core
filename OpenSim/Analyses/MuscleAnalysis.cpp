// MuscleAnalysis.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Katherine Holzbaur, Frank C. Anderson
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
#include <OpenSim/Simulation/Model/Muscle.h>
#include <OpenSim/Simulation/Model/Actuator.h>
#include <OpenSim/Simulation/SimbodyEngine/Coordinate.h>
#include <OpenSim/Simulation/Model/CoordinateSet.h>
#include <OpenSim/Simulation/Model/ForceSet.h>
#include "MuscleAnalysis.h"


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
MuscleAnalysis::~MuscleAnalysis()
{
	// Individual storages where added to the Analysis' _storageList
	// which takes ownerwhip of the Storage objects and deletes them.
}
//_____________________________________________________________________________
/**
 * Construct a MuscleAnalysis object for recording the MuscleAnalysis of
 * a model's generalized coodinates during a simulation.
 *
 * @param aModel Model for which the MuscleAnalysis are to be recorded.
 */
MuscleAnalysis::MuscleAnalysis(Model *aModel) :
	Analysis(aModel)
{
	// NULL
	setNull();

	// CHECK MODEL
	if(_model==NULL) return;

	// STORAGE
	allocateStorageObjects();
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
MuscleAnalysis::MuscleAnalysis(const std::string &aFileName):
Analysis(aFileName, false)
{
	setNull();
	updateFromXMLDocument();
	allocateStorageObjects();
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 */
MuscleAnalysis::MuscleAnalysis(const MuscleAnalysis &aMuscleAnalysis):
Analysis(aMuscleAnalysis)
{
	setNull();
	*this = aMuscleAnalysis;
}


//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Set NULL values for all member variables.
 */
void MuscleAnalysis::
setNull()
{
	setName("MuscleAnalysis");
	setupProperties();
	constructDescription();

	// STORAGE
	_pennationAngleStore = NULL;
	_lengthStore = NULL;
	_fiberLengthStore = NULL;
	_normalizedFiberLengthStore = NULL;
	_tendonLengthStore = NULL;

	_fiberVelocityStore = NULL;
	_normFiberVelocityStore = NULL;
	_pennationAngularVelocityStore = NULL;

	_forceStore = NULL;
	_fiberForceStore = NULL;
	_activeFiberForceStore = NULL;
	_passiveFiberForceStore = NULL;
	_activeFiberForceAlongTendonStore = NULL;
	_passiveFiberForceAlongTendonStore = NULL;

	// DEFAULT VALUES
	_muscleListProp.getValueStrArray().setSize(1);
	_muscleListProp.getValueStrArray().get(0) = "all";
	_coordinateListProp.getValueStrArray().setSize(1);
	_coordinateListProp.getValueStrArray().get(0) = "all";
	_computeMoments = true;
}
//_____________________________________________________________________________
/**
 * Set up the properties.
 */
void MuscleAnalysis::
setupProperties()
{
	_muscleListProp.setComment("List of muscles for which to perform the analysis."
		" Use 'all' to perform the analysis for all muscles.");
	_muscleListProp.setName("muscle_list");
	_propertySet.append( &_muscleListProp );

	_coordinateListProp.setComment("List of generalized coordinates for which to "
		"compute moment arms. Use 'all' to compute for all coordinates.");
	_coordinateListProp.setName("moment_arm_coordinate_list");
	_propertySet.append( &_coordinateListProp );

	_computeMomentsProp.setComment("Flag indicating whether moments should be computed.");
	_computeMomentsProp.setName("compute_moments");
	_propertySet.append( &_computeMomentsProp );

}
//-----------------------------------------------------------------------------
// DESCRIPTION
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Construct the description for the MuscleAnalysis files.
 */
void MuscleAnalysis::
constructDescription()
{
	char descrip[1024];

	strcpy(descrip,"\nThis analysis gathers basic information about muscles ");
	strcat(descrip,"during a simulation (e.g., forces, tendon lenghts, moment arms, etc).");

	strcat(descrip,"\nUnits are S.I. units (second, meters, Newtons, ...)");
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
 * Allocate storage for the muscle variables.
 */
void MuscleAnalysis::
allocateStorageObjects()
{
	if(_model==NULL) return;
	if (!getOn()) return;

	// CLEAR EXISTING WORK ARRAYS
	_storageList.setMemoryOwner(true);
	_storageList.setSize(0);
	_momentArmStorageArray.setMemoryOwner(true);
	_momentArmStorageArray.setSize(0);
	_muscleArray.setMemoryOwner(false);
	_muscleArray.setSize(0);

	// FOR MOMENT ARMS AND MOMEMTS
	const CoordinateSet& qSet = _model->getCoordinateSet();
	int nq = qSet.getSize();
	Storage *store;
	for(int i=0;i<nq;i++) {
		const Coordinate& q = qSet.get(i);
		string name = "MomentArm_" + q.getName();
		store = new Storage(1000,name);
		store->setDescription(getDescription());
		_storageList.append(store);
	}
	for(int i=0;i<nq;i++) {
		const Coordinate& q = qSet.get(i);
		string name = "Moment_" + q.getName();
		store = new Storage(1000,name);
		store->setDescription(getDescription());
		_storageList.append(store);
	}

	// EVERYTHING ELSE
	//_storageList.setMemoryOwner(false);
	_pennationAngleStore = new Storage(1000,"PennationAngle");
	_pennationAngleStore->setDescription(getDescription());
	_storageList.append(_pennationAngleStore );

	_lengthStore = new Storage(1000,"Length");
	_lengthStore->setDescription(getDescription());
	_storageList.append(_lengthStore );

	_fiberLengthStore = new Storage(1000,"FiberLength");
	_fiberLengthStore->setDescription(getDescription());
	_storageList.append(_fiberLengthStore );

	_normalizedFiberLengthStore = new Storage(1000,"NormalizedFiberLength");
	_normalizedFiberLengthStore->setDescription(getDescription());
	_storageList.append(_normalizedFiberLengthStore );

	_tendonLengthStore = new Storage(1000,"TendonLength");
	_tendonLengthStore->setDescription(getDescription());
	_storageList.append(_tendonLengthStore );

	_fiberVelocityStore = new Storage(1000,"FiberVelocity");
	_fiberVelocityStore->setDescription(getDescription());
	_storageList.append(_fiberVelocityStore );

	_normFiberVelocityStore = new Storage(1000,"NormFiberVelocity");
	_normFiberVelocityStore->setDescription(getDescription());
	_storageList.append(_normFiberVelocityStore );

	_pennationAngularVelocityStore = new Storage(1000,"PennationAngularVelocity");
	_pennationAngularVelocityStore->setDescription(getDescription());
	_storageList.append(_pennationAngularVelocityStore );


	_forceStore = new Storage(1000,"TendonForce");
	_forceStore->setDescription(getDescription());
	_storageList.append(_forceStore );

	_fiberForceStore = new Storage(1000,"FiberForce");
	_fiberForceStore->setDescription(getDescription());
	_storageList.append(_fiberForceStore );

	_activeFiberForceStore = new Storage(1000,"ActiveFiberForce");
	_activeFiberForceStore->setDescription(getDescription());
	_storageList.append(_activeFiberForceStore );

	_passiveFiberForceStore = new Storage(1000,"PassiveFiberForce");
	_passiveFiberForceStore->setDescription(getDescription());
	_storageList.append(_passiveFiberForceStore );

	_activeFiberForceAlongTendonStore = new Storage(1000,"ActiveFiberForceAlongTendon");
	_activeFiberForceAlongTendonStore->setDescription(getDescription());
	_storageList.append(_activeFiberForceAlongTendonStore );

	_passiveFiberForceAlongTendonStore = new Storage(1000,"PassiveFiberForceAlongTendon");
	_passiveFiberForceAlongTendonStore->setDescription(getDescription());
	_storageList.append(_passiveFiberForceAlongTendonStore );

	_fiberPowerStore = new Storage(1000,"FiberPower");
	_fiberPowerStore->setDescription(getDescription());
	_storageList.append(_fiberPowerStore );

	_tendonPowerStore = new Storage(1000,"TendonPower");
	_tendonPowerStore->setDescription(getDescription());
	_storageList.append(_tendonPowerStore );

	_musclePowerStore = new Storage(1000,"MuscleActuatorPower");
	_musclePowerStore->setDescription(getDescription());
	_storageList.append(_musclePowerStore );

	// UPDATE ALL STORAGE OBJECTS
	updateStorageObjects();
}
//_____________________________________________________________________________
/**
 * Update storage objects.  This is necessary if the modle, mucle, or
 * coordinate list is changed.
 */
void MuscleAnalysis::
updateStorageObjects()
{
	if(_model==NULL) return;
	if (!getOn()) return;

	// POPULATE MUSCLE LIST FOR "all"
	ForceSet& fSet = _model->updForceSet();
	_muscleList = _muscleListProp.getValueStrArray();
	int nm = _muscleList.getSize();
	if((nm==1) && (_muscleList.get(0)=="all")) {
		_muscleList.setSize(0);
		int nf = fSet.getSize();
		for(int i=0;i<nf;i++) {
			Muscle *m = dynamic_cast<Muscle*>(&fSet.get(i));
            if( m ) _muscleList.append(m->getName());
		}
	}
	// POPULATE ACTIVE MUSCLE ARRAY
	Array<string> tmpMuscleList("");
	nm = _muscleList.getSize();
	_muscleArray.setSize(0);
	for(int i=0; i<nm; i++) {
		if(fSet.contains(_muscleList[i])) {
    		Muscle* mus = dynamic_cast<Muscle*>( &fSet.get(_muscleList[i]) );
			if(mus){
				_muscleArray.append(mus);
				tmpMuscleList.append(mus->getName());
			}
		}
	}
	_muscleList = tmpMuscleList;

	// POPULATE COORDINATE LIST FOR "all"
	CoordinateSet& qSet = _model->updCoordinateSet();
	_coordinateList = _coordinateListProp.getValueStrArray();
	int nq = qSet.getSize();
	int nActiveQ = _coordinateList.getSize();
	if((nActiveQ==1) && (_coordinateList.get(0)=="all")) {
		_coordinateList.setSize(0);
		for(int i=0;i<nq;i++) {
			Coordinate& q = qSet.get(i);
			_coordinateList.append(q.getName());
		}
	}
	// POPULATE ACTIVE MOMENT ARM ARRAY
	Array<string> tmpCoordinateList("");  // For making sure the coordinates in the list really exist.
	_momentArmStorageArray.setSize(0);
	nActiveQ = _coordinateList.getSize();
	for(int i=0; i<nActiveQ; i++) {
		string name = _coordinateList[i];
		for(int j=0; j<nq; j++) {
			Coordinate& q = qSet.get(j);
			if(name == q.getName()) {
				StorageCoordinatePair *pair = new StorageCoordinatePair();
				pair->q = &q;
				pair->momentArmStore = _storageList[j];
				pair->momentStore = _storageList[j+nq];
				_momentArmStorageArray.append(pair);
				tmpCoordinateList.append(q.getName());
			}
		}
	}
	_coordinateList = tmpCoordinateList;
	//cout<<"Number of active moment arm storage array = "<<_momentArmStorageArray.getSize()<<endl;

	// CONSTRUCT AND SET COLUMN LABELS
	constructColumnLabels();
	Storage *store;
	int size = _storageList.getSize();
	for(int i=0;i<size;i++) {
		store = _storageList[i];
		if(store==NULL) continue;
		store->setColumnLabels(getColumnLabels());
	}
}
//-----------------------------------------------------------------------------
// COLUMN LABELS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Construct the column labels for the MuscleAnalysis storage files.
 */
void MuscleAnalysis::
constructColumnLabels()
{
	if(!_model) return;
	int size = _muscleList.getSize();
	Array<string> labels("",size+1);
	labels[0] = "time";
	for(int i=0; i<size; i++) {
		labels[i+1] = _muscleList[i];
	}
	setColumnLabels(labels);
}


//=============================================================================
// OPERATORS
//=============================================================================
MuscleAnalysis& MuscleAnalysis::operator=(const MuscleAnalysis &aAnalysis)
{
	// BASE CLASS
	Analysis::operator=(aAnalysis);

	// MEMBER VARIABLES
	_muscleListProp = aAnalysis._muscleListProp;
	_coordinateListProp = aAnalysis._coordinateListProp;
	_computeMoments = aAnalysis._computeMoments;
	allocateStorageObjects();

	return (*this);
}


//=============================================================================
// GET AND SET
//=============================================================================
//-----------------------------------------------------------------------------
// MODEL
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the model pointer for analysis.
 */
void MuscleAnalysis::setModel(Model& aModel)
{
	Analysis::setModel(aModel);
	allocateStorageObjects();
}
//_____________________________________________________________________________
/**
 * Set the list of muscles to analyze.
 *
 * @param aMuscles is the array of names of muscles to analyze.
 */
void MuscleAnalysis::
setMuscles(OpenSim::Array<std::string>& aMuscles)
{
	int size = aMuscles.getSize();
	_muscleListProp.getValueStrArray().setSize(aMuscles.getSize());
	for(int i=0; i<size; i++){
		_muscleListProp.getValueStrArray().get(i) = aMuscles.get(i);
	}
	updateStorageObjects();
}
//_____________________________________________________________________________
/**
 * Set the list of coordinates.
 *
 * @param aCoordinates Array of coordinates about which to compute moment arms.
 */
void MuscleAnalysis::
setCoordinates(OpenSim::Array<std::string>& aCoordinates)
{
	int size = aCoordinates.getSize();
	_coordinateListProp.getValueStrArray().setSize(size);
	for(int i=0; i<size; i++){
		_coordinateListProp.getValueStrArray().get(i) = aCoordinates[i];
	}
	updateStorageObjects();
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
void MuscleAnalysis::
setStorageCapacityIncrements(int aIncrement)
{
	if(!_model) return;
	Storage *store;
	int size = _storageList.getSize();
	for(int i=0;i<size;i++) {
		store = _storageList[i];
		if(store==NULL) continue;
		store->setCapacityIncrement(aIncrement);
	}
}


//=============================================================================
// ANALYSIS
//=============================================================================
//_____________________________________________________________________________
/**
 * Record the MuscleAnalysis quantities.
 */
int MuscleAnalysis::record(const SimTK::State& s)
{
	if(_model==NULL) return(-1);
	if (!getOn()) return(-1);

	// MAKE SURE ALL ACTUATION QUANTITIES ARE VALID
	// COMPUTE DERIVATIVES
	// ----------------------------------
	// TIME NORMALIZATION
	double tReal = s.getTime();
	// ----------------------------------
	// LOOP THROUGH MUSCLES
	int nm = _muscleArray.getSize();
	// Angles and lengths
	Array<double> penang(0.0,nm);
	Array<double> len(0.0,nm), tlen(0.0,nm);
	Array<double> fiblen(0.0,nm), normfiblen(0.0,nm);

	// Muscle velocity information
	Array<double> fibVel(0.0,nm), normFibVel(0.0,nm);
	Array<double> penAngVel(0.0,nm);

	// Muscle component forces
	Array<double> force(0.0,nm),fibforce(0.0,nm);
	Array<double> actfibforce(0.0,nm),passfibforce(0.0,nm);
	Array<double> actfibforcealongten(0.0,nm),passfibforcealongten(0.0,nm);

	// Muscle and component powers
	Array<double> fibPower(0.0,nm), tendonPower(0.0,nm), muscPower(0.0,nm);

	// state derivatives are gauranteed to be evaluated by acceleration
	_model->getMultibodySystem().realize(s,SimTK::Stage::Acceleration);  

	for(int i=0; i<nm; i++) {
		penang[i] = _muscleArray[i]->getPennationAngle(s);
		len[i] = _muscleArray[i]->getLength(s);
		tlen[i] = _muscleArray[i]->getTendonLength(s);
		fiblen[i] = _muscleArray[i]->getFiberLength(s);
		normfiblen[i] = _muscleArray[i]->getNormalizedFiberLength(s);

		fibVel[i] = _muscleArray[i]->getFiberVelocity(s);
		normFibVel[i] =  _muscleArray[i]->getNormalizedFiberVelocity(s);
		penAngVel[i] =  _muscleArray[i]->getPennationAngularVelocity(s);

		// Compute muscle forces that are dependent on Positions, Velocities
		// so that later quantities are valid and setForce is called
		_muscleArray[i]->computeActuation(s);
		force[i] = _muscleArray[i]->getForce(s);
		fibforce[i] = _muscleArray[i]->getFiberForce(s);
		actfibforce[i] = _muscleArray[i]->getActiveFiberForce(s);
		passfibforce[i] = _muscleArray[i]->getPassiveFiberForce(s);
		actfibforcealongten[i] = _muscleArray[i]->getActiveFiberForceAlongTendon(s);
		passfibforcealongten[i] = _muscleArray[i]->getPassiveFiberForceAlongTendon(s);
	
		//Powers
		fibPower[i] = _muscleArray[i]->getFiberPower(s);
		tendonPower[i] = _muscleArray[i]->getTendonPower(s);
		muscPower[i] = _muscleArray[i]->getMusclePower(s);
	}

	// APPEND TO STORAGE
	_pennationAngleStore->append(tReal,penang.getSize(),&penang[0]);
	_lengthStore->append(tReal,len.getSize(),&len[0]);
	_fiberLengthStore->append(tReal,fiblen.getSize(),&fiblen[0]);
	_normalizedFiberLengthStore->append(tReal,normfiblen.getSize(),&normfiblen[0]);
	_tendonLengthStore->append(tReal,tlen.getSize(),&tlen[0]);

	_fiberVelocityStore->append(tReal,fibVel.getSize(),&fibVel[0]);
	_normFiberVelocityStore->append(tReal,normFibVel.getSize(),&normFibVel[0]);
	_pennationAngularVelocityStore->append(tReal,penAngVel.getSize(),&penAngVel[0]);

	_forceStore->append(tReal,force.getSize(),&force[0]);
	_fiberForceStore->append(tReal,fibforce.getSize(),&fibforce[0]);
	_activeFiberForceStore->append(tReal,actfibforce.getSize(),&actfibforce[0]);
	_passiveFiberForceStore->append(tReal,passfibforce.getSize(),&passfibforce[0]);
	_activeFiberForceAlongTendonStore->append(tReal,actfibforcealongten.getSize(),&actfibforcealongten[0]);
	_passiveFiberForceAlongTendonStore->append(tReal,passfibforcealongten.getSize(),&passfibforcealongten[0]);

	_fiberPowerStore->append(tReal,fibPower.getSize(),&fibPower[0]);
	_tendonPowerStore->append(tReal,tendonPower.getSize(),&tendonPower[0]);
	_musclePowerStore->append(tReal,muscPower.getSize(),&muscPower[0]);

	if (_computeMoments){
	// LOOP OVER ACTIVE MOMENT ARM STORAGE OBJECTS
	Coordinate *q = NULL;
	Storage *maStore=NULL, *mStore=NULL;
	int nq = _momentArmStorageArray.getSize();
	Array<double> ma(0.0,nm),m(0.0,nm);

	for(int i=0; i<nq; i++) {

		q = _momentArmStorageArray[i]->q;
		maStore = _momentArmStorageArray[i]->momentArmStore;
		mStore = _momentArmStorageArray[i]->momentStore;
       
		// Make a writable copy of the state so moment arm can be computed
		SimTK::State tempState = s;

		bool locked = q->getLocked(tempState);

		_model->getMultibodySystem().realize(tempState, s.getSystemStage() );
		// LOOP OVER MUSCLES
		for(int j=0; j<nm; j++) {
            ma[j] = _muscleArray[j]->computeMomentArm(tempState,*q);
			m[j] = ma[j] * force[j];
		}
		maStore->append(s.getTime(),nm,&ma[0]);
		mStore->append(s.getTime(),nm,&m[0]);
	}
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
 * This method should be overriden in the child class.  It is
 * included here so that the child class will not have to implement it if it
 * is not necessary.
 *
 * @param s current system state
 *
 * @return -1 on error, 0 otherwise.
 */
int MuscleAnalysis::
begin(SimTK::State& s )
{
	if(!proceed()) return(0);

	// RESET STORAGE
	Storage *store;
	int size = _storageList.getSize();
	for(int i=0;i<size;i++) {
		store = _storageList[i];
		if(store==NULL) continue;
		store->purge();
	}

	// RECORD
	int status = 0;
	// Make sure cooridnates are not locked
	if (_computeMoments){
	// LOOP OVER ACTIVE MOMENT ARM STORAGE OBJECTS
		Coordinate *q = NULL;
		int nq = _momentArmStorageArray.getSize();
		for(int i=0; i<nq; i++) {
			q = _momentArmStorageArray[i]->q;
			if (q->getLocked(s))
				throw(Exception("Coordinate: "+q->getName()+" is locked and can't be varied. Aborting.")); 
		}
	}
	if(_storageList.getSize()> 0 && _storageList.get(0)->getSize() <= 0) status = record(s);

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
int MuscleAnalysis::
step(const SimTK::State& s, int stepNumber )
{
	if(!proceed(stepNumber)) return(0);

	int status = record(s);

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
 * @param s current state of system
 *
 * @return -1 on error, 0 otherwise.
 */
int MuscleAnalysis::
end(SimTK::State& s )
{
	if (!proceed()) return 0;
	record(s);
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
 * @return -1 on error, 0 otherwise.
 */
int MuscleAnalysis::
printResults(const string &aBaseName,const string &aDir,double aDT,
				 const string &aExtension)
{
	if(!getOn()) {
		printf("MuscleAnalysis.printResults: Off- not printing.\n");
		return(0);
	}

	std::string prefix = aBaseName + "_" + getName() + "_";
	for(int i=0; i<_storageList.getSize(); ++i){
		Storage::printResult(_storageList[i],prefix+_storageList[i]->getName(),aDir,aDT,aExtension);
	}
	/*
	Storage::printResult(_pennationAngleStore,prefix+"PennationAngle",aDir,aDT,aExtension);
	Storage::printResult(_lengthStore,prefix+"Length",aDir,aDT,aExtension);
	Storage::printResult(_fiberLengthStore,prefix+"FiberLength",aDir,aDT,aExtension);
	Storage::printResult(_normalizedFiberLengthStore,prefix+"NormalizedFiberLength",aDir,aDT,aExtension);
	Storage::printResult(_tendonLengthStore,prefix+"TendonLength",aDir,aDT,aExtension);
	Storage::printResult(_forceStore,prefix+"Force",aDir,aDT,aExtension);
	Storage::printResult(_fiberForceStore,prefix+"FiberForce",aDir,aDT,aExtension);
	Storage::printResult(_activeFiberForceStore,prefix+"ActiveFiberForce",aDir,aDT,aExtension);
	Storage::printResult(_passiveFiberForceStore,prefix+"PassiveFiberForce",aDir,aDT,aExtension);
	Storage::printResult(_activeFiberForceAlongTendonStore,prefix+"ActiveFiberForceAlongTendon",aDir,aDT,aExtension);
	Storage::printResult(_passiveFiberForceAlongTendonStore,prefix+"PassiveFiberForceAlongTendon",aDir,aDT,aExtension);
	*/

	int size = _momentArmStorageArray.getSize();
	for(int i=0;i<size;i++) {
		string fileName = prefix + _momentArmStorageArray.get(i)->momentArmStore->getName();
		Storage::printResult(_momentArmStorageArray.get(i)->momentArmStore,fileName,aDir,aDT,aExtension);
		fileName = prefix + _momentArmStorageArray.get(i)->momentStore->getName();
		Storage::printResult(_momentArmStorageArray.get(i)->momentStore,fileName,aDir,aDT,aExtension);
	}

	return(0);
}

