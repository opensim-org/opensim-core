/* -------------------------------------------------------------------------- *
 *                          OpenSim:  Kinematics.cpp                          *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Frank C. Anderson                                               *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */


//=============================================================================
// INCLUDES
//=============================================================================
#include <OpenSim/Simulation/Model/Model.h>
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
    deleteStorage(); 
}
//_____________________________________________________________________________
/**
 * Construct an Kinematics object for recording the kinematics of
 * a model's generalized coordinates during a simulation.
 *
 * @param aModel Model for which the kinematics are to be recorded.
 */
Kinematics::Kinematics(Model *aModel) :
    Analysis(aModel)
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
    Analysis(aFileName, false)
{
    setNull();

    // Serialize from XML
    updateFromXMLDocument();

    // CONSTRUCT DESCRIPTION AND LABELS
    constructDescription();

}

//_____________________________________________________________________________
/**
 * Set all member variables to their null or default values.
 */
void Kinematics::
setNull()
{
    constructProperties();

    setName("Kinematics");
    _pStore=_vStore=_aStore=NULL;

    // Make sure storages are owned/managed by the Analysis
    _storageList.setMemoryOwner(false);

    _recordAccelerations = true;
}
//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void Kinematics::
constructProperties()
{
    Array<std::string> coordArray;
    coordArray.setSize(1);
    coordArray[0] = "all";
    constructProperty_coordinates(coordArray);
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Allocate storage for the kinematics.
 */
void Kinematics::allocateStorage()
{
    //Free-up any past storages by resizing which will delete pointers
    //if memory owner is set to true, as set in setNull() above
    _storageList.setSize(0);

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
    _coordinateIndices.setSize(getProperty_coordinates().size());
    for(int i=0; i<getProperty_coordinates().size(); i++) {
        if(get_coordinates(i) == "all") {
            _coordinateIndices.setSize(coordSet.getSize());
            for(int j=0;j<coordSet.getSize();j++) _coordinateIndices[j]=j;
            break;
        }
        
        int index = coordSet.getIndex(get_coordinates(i));
        if(index<0) 
            throw Exception("Kinematics: ERR- Could not find coordinate named '"+get_coordinates(i)+"'",__FILE__,__LINE__);
        _coordinateIndices[i] = index;
    }
    _values.setSize(_coordinateIndices.getSize());

    if(_values.getSize()==0) {
        log_warn("Kinematics analysis has no coordinates to record values for");
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

    strcpy(descrip, "\nUnits are S.I. units (second, meters, Newtons, ...)");
    strcat(descrip, "\nIf the header above contains a line with ");
    strcat(descrip, "'inDegrees', this indicates whether rotational values ");
    strcat(descrip, "are in degrees (yes) or radians (no).");
    strcat(descrip, "\n\n");

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
    if (_pStore){
        _pStore->setColumnLabels(getColumnLabels());
        if (getInDegrees()) _pStore->setInDegrees(true);
    }
    if (_vStore) {
        _vStore->setColumnLabels(getColumnLabels());
        if (getInDegrees()) _vStore->setInDegrees(true);
    }
    if (_aStore) {
        _aStore->setColumnLabels(getColumnLabels());
        if (getInDegrees()) _aStore->setInDegrees(true);
    }
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

    // Allocate storages to contain the results of the analysis
    allocateStorage();
}

//-----------------------------------------------------------------------------
// STORAGE CAPACITY
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the capacity increments of all storage instances.
 *
 * @param aIncrement Increment by which storage capacities will be increased
 * when storage capacities run out.
 */
void Kinematics::
setStorageCapacityIncrements(int aIncrement)
{
    if (_aStore) _aStore->setCapacityIncrement(aIncrement);
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
int Kinematics::record(const SimTK::State& s)
{
    if(_recordAccelerations){
        _model->getMultibodySystem().realize(s, SimTK::Stage::Acceleration);
    }
    else{
        _model->getMultibodySystem().realize(s, SimTK::Stage::Velocity);
    }
    // RECORD RESULTS
    const CoordinateSet& cs = _model->getCoordinateSet();
    int nvalues = _coordinateIndices.getSize();
    for(int i=0;i<nvalues;i++){
        _values[i] = cs[_coordinateIndices[i]].getValue(s);
        if(getInDegrees() && (cs[_coordinateIndices[i]].getMotionType() == Coordinate::Rotational))
            _values[i] *= SimTK_RADIAN_TO_DEGREE;
    }
    _pStore->append(s.getTime(),nvalues,&_values[0]);

    for(int i=0;i<nvalues;i++){
        _values[i] = cs[_coordinateIndices[i]].getSpeedValue(s);
        if(getInDegrees() && (cs[_coordinateIndices[i]].getMotionType() == Coordinate::Rotational))
            _values[i] *= SimTK_RADIAN_TO_DEGREE;
    }

    _vStore->append(s.getTime(),nvalues,&_values[0]);

    if(_recordAccelerations) {
        for(int i=0;i<nvalues;i++){
            _values[i] = cs[_coordinateIndices[i]].getAccelerationValue(s);
            if(getInDegrees() && (cs[_coordinateIndices[i]].getMotionType() == Coordinate::Rotational))
                _values[i] *= SimTK_RADIAN_TO_DEGREE;
        }
        _aStore->append(s.getTime(),nvalues,&_values[0]);
    }


    return(0);
}
//_____________________________________________________________________________
/**
 * This method is called at the beginning of an analysis so that any
 * necessary initializations may be performed.
 *
 * This method is meant to be called at the beginning of an integration 
 *
 * @param s current state of System
 *
 * @return -1 on error, 0 otherwise.
 */
int Kinematics::
begin( const SimTK::State& s )
{
    if(!proceed()) return(0);

    double time = s.getTime();
    
    // UPDATE LABELS
    updateCoordinatesToRecord();
    constructColumnLabels();

    // RESET STORAGE
    _pStore->reset(time);
    _vStore->reset(time);
    if (_aStore) _aStore->reset(time);

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
 * This method should be overridden in derived classes.  It is
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
 * This method should be overridden in the child class.  It is
 * included here so that the child class will not have to implement it if it
 * is not necessary.
 *
 * @param s current state of System
 *
 * @return -1 on error, 0 otherwise.
 */
int Kinematics::
end( const SimTK::State& s )
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
        log_info("Kinematics.printResults: Off- not printing.");
        return 0;
    }

    // ACCELERATIONS
    if(_recordAccelerations) {
        Storage::printResult(_aStore,aBaseName+"_"+getName()+"_dudt",aDir,aDT,aExtension);
    }

    // VELOCITIES
    Storage::printResult(_vStore,aBaseName+"_"+getName()+"_u",aDir,aDT,aExtension);

    // POSITIONS
    Storage::printResult(_pStore,aBaseName+"_"+getName()+"_q",aDir,aDT,aExtension);

    /*
    // POSITIONS (.mot file)
    // For now (until we resolve the .sto vs .mot issue) also write
    // an .mot file so the motion can be viewed in SIMM -- Eran.
    bool writeSIMMHeader=_pStore->getWriteSIMMHeader();
    _pStore->setWriteSIMMHeader(true);
    Storage::printResult(_pStore,aBaseName,aDir,aDT,".mot");
    _pStore->setWriteSIMMHeader(writeSIMMHeader);
    */
    return(0);
}
