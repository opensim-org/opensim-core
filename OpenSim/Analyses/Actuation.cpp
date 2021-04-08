/* -------------------------------------------------------------------------- *
*                          OpenSim:  Actuation.cpp                           *
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
#include "Actuation.h"
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/Actuator.h>

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
Actuation::~Actuation()
{
    if (_fsp != NULL) { delete[] _fsp;  _fsp = NULL; }
    deleteStorage();
}
//_____________________________________________________________________________
/**
* Construct an Actuation object for recording the Actuation of
* a model's generalized coordinates during a simulation.
*
* @param aModel Model for which the Actuation are to be recorded.
*/
Actuation::Actuation(Model *aModel) :
Analysis(aModel)
{
    // NULL
    setNull();

    // DESCRIPTION
    constructDescription();


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
Actuation::Actuation(const std::string &aFileName) :
Analysis(aFileName, false)
{
    setNull();

    // Serialize from XML
    updateFromXMLDocument();

    // DESCRIPTION
    constructDescription();

    // STORAGE
    allocateStorage();
}

// Copy constructor and virtual copy 
//_____________________________________________________________________________
/**
* Copy constructor.
*
*/
Actuation::Actuation(const Actuation &aActuation) :
Analysis(aActuation)
{
    setNull();
    // COPY TYPE AND NAME
    *this = aActuation;
}


//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
* Set NULL values for all member variables.
*/
void Actuation::
setNull()
{
    // NAME
    setName("Actuation");

    _na = 0;
    _fsp = NULL;
    _forceStore = NULL;
    _speedStore = NULL;
    _powerStore = NULL;
}
//--------------------------------------------------------------------------
// OPERATORS
//--------------------------------------------------------------------------
Actuation& Actuation::operator=(const Actuation &aActuation)
{
    // BASE CLASS
    Analysis::operator=(aActuation);

    // Deallocate _fsp if already allocated
    if (_fsp != NULL) { delete[] _fsp;  _fsp = NULL; }

    // STORAGE
    deleteStorage();
    allocateStorage();

    // CHECK MODEL
    if (_model != NULL) {
        _na = getNumEnabledActuators();
        _fsp = new double[_na];
        constructColumnLabels();
    }

    return (*this);
}
//_____________________________________________________________________________
/**
* Set the model pointer for analysis.
*/
void Actuation::setModel(Model& aModel)
{
    // BASE CLASS
    Analysis::setModel(aModel);

    // NUMBER OF ACTUATORS
    if (_model){
        _na = getNumEnabledActuators();
    }
    else
        _na = 0;

    if (_na <= 0){
        log_warn("Actuation analysis canceled. There are no Actuators in the "
                 "model.");
        return;
    }

    // STORAGE
    deleteStorage();
    allocateStorage();


    // UPDATE LABELS
    constructColumnLabels();
}
//_____________________________________________________________________________
/**
* Allocate storage for the kinematics.
*/
void Actuation::
allocateStorage()
{
    // ACCELERATIONS
    _forceStore = new Storage(1000, "ActuatorForces");
    _forceStore->setDescription(getDescription());
    // Keep references o all storages in a list for uniform access from GUI
    _storageList.append(_forceStore);
    _storageList.setMemoryOwner(false);
    // VELOCITIES
    _speedStore = new Storage(1000, "ActuatorSpeeds");
    _speedStore->setDescription(getDescription());
    _storageList.append(_speedStore);

    // POSITIONS
    _powerStore = new Storage(1000, "ActuatorPowers");
    _powerStore->setDescription(getDescription());
    _storageList.append(_powerStore);
}

//-----------------------------------------------------------------------------
// DESCRIPTION
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
* Construct the description for the Actuation files.
*/
void Actuation::
constructDescription()
{
    char descrip[1024];

    strcpy(descrip, "\nThis file contains either the forces, speeds, or ");
    strcat(descrip, "powers developed\nby the actuators of a model ");
    strcat(descrip, "during a simulation.\n");

    strcat(descrip, "\nAn actuator force is a generalized force, meaning that");
    strcat(descrip, " it can be either a force (N) or a torque (Nm).\n");

    strcat(descrip, "\nAn actuator speed is the rate at which an actuator ");
    strcat(descrip, "shortens. Depending on the actuator,\na speed can be ");
    strcat(descrip, "either a translational speed (m/s) or an angular speed ");
    strcat(descrip, "(deg/s or rad/s).\n");

    strcat(descrip, "\nAn actuator power (Watts) is the rate at which an ");
    strcat(descrip, "actuator does work.  Positive power means\nthat the ");
    strcat(descrip, "actuator is delivering energy to the model; negative ");
    strcat(descrip, "power means that the actuator\nis absorbing energy ");
    strcat(descrip, "from the model.\n");

    strcat(descrip, "\nUnits are S.I. units (second, meters, Newtons, ...)");
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
* Construct the column labels for the actuation storage files.
*/
void Actuation::
constructColumnLabels()
{
    if (_model)
    {
        // ASSIGN
        Array<string> labels;
        labels.append("time");
        const Set<Actuator>& ai = _model->getActuators();
        for (int i = 0; i < ai.getSize(); i++)
            if(ai.get(i).get_appliesForce())
                labels.append(ai.get(i).getName());
        setColumnLabels(labels);
    }
    _forceStore->setColumnLabels(getColumnLabels());
    _speedStore->setColumnLabels(getColumnLabels());
    _powerStore->setColumnLabels(getColumnLabels());
}


//=============================================================================
// DESTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
* Delete storage objects.
*/
void Actuation::
deleteStorage()
{
    if (_forceStore != NULL) { delete _forceStore;  _forceStore = NULL; }
    if (_speedStore != NULL) { delete _speedStore;  _speedStore = NULL; }
    if (_powerStore != NULL) { delete _powerStore;  _powerStore = NULL; }
}


//=============================================================================
// GET AND SET
//=============================================================================

//-----------------------------------------------------------------------------
// STORAGE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
* Get the force storage.
*
* @return Force storage.
*/
Storage* Actuation::
getForceStorage() const
{
    return(_forceStore);
}
//_____________________________________________________________________________
/**
* Get the speed storage.
*
* @return Speed storage.
*/
Storage* Actuation::
getSpeedStorage() const
{
    return(_speedStore);
}
//_____________________________________________________________________________
/**
* Get the power storage.
*
* @return Power storage.
*/
Storage* Actuation::
getPowerStorage() const
{
    return(_powerStore);
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
void Actuation::
setStorageCapacityIncrements(int aIncrement)
{
    _forceStore->setCapacityIncrement(aIncrement);
    _speedStore->setCapacityIncrement(aIncrement);
    _powerStore->setCapacityIncrement(aIncrement);
}



//=============================================================================
// ANALYSIS
//=============================================================================
//_____________________________________________________________________________
/**
* Record the actuation quantities.
*/
int Actuation::
record(const SimTK::State& s)
{
    if (_model == NULL) return(-1);

    // MAKE SURE ALL ACTUATION QUANTITIES ARE VALID
    _model->getMultibodySystem().realize(s, SimTK::Stage::Dynamics);

    // TIME NORMALIZATION
    double tReal = s.getTime();

    // FORCE
    const Set<Actuator>& fSet = _model->getActuators();
    for (int i = 0, iact = 0; i<fSet.getSize(); i++) {
        ScalarActuator* act = dynamic_cast<ScalarActuator*>(&fSet[i]);
        if(fSet.get(i).get_appliesForce())
            _fsp[iact++] = act->getActuation(s);
    }
    _forceStore->append(tReal, _na, _fsp);

    // SPEED
    for (int i = 0, iact = 0; i<fSet.getSize(); i++) {
        ScalarActuator* act = dynamic_cast<ScalarActuator*>(&fSet[i]);
        if(fSet.get(i).get_appliesForce())
            _fsp[iact++] = act->getSpeed(s);
    }
    _speedStore->append(tReal, _na, _fsp);

    // POWER
    for (int i = 0, iact = 0; i<fSet.getSize(); i++) {
        if(fSet.get(i).get_appliesForce())
            _fsp[iact++] = fSet.get(i).getPower(s);
    }
    _powerStore->append(tReal, _na, _fsp);


    return(0);
}
//_____________________________________________________________________________
/**
* This method is called at the beginning of an analysis so that any
* necessary initializations may be performed.
*
* This method is meant to be called at the beginning of an integration in
* Model::integBeginCallback() and has the same argument list.
*
* This method should be overridden in the child class.  It is
* included here so that the child class will not have to implement it if it
* is not necessary.
*
* @param state system State
* @param aStep Step number of the integration.
*
* @return -1 on error, 0 otherwise.
*/
int Actuation::
begin(const SimTK::State& s)
{
    if (!proceed()) return(0);

    // NUMBER OF ACTUATORS
    int na = getNumEnabledActuators();
    _na = na;
    // WORK ARRAY
    if (_fsp != NULL) delete[] _fsp;
    _fsp = new double[_na];

    // RESET STORAGE
    if (_forceStore == NULL)
        _forceStore = new Storage();
    if (_speedStore == NULL)
        _speedStore = new Storage();
    if (_powerStore == NULL)
        _powerStore = new Storage();

    // RESET STORAGE
    _forceStore->reset(s.getTime());
    _speedStore->reset(s.getTime());
    _powerStore->reset(s.getTime());

    // RECORD
    int status = 0;
    if (_forceStore->getSize() <= 0) {
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
* When called during an integration, this method is meant to be called in
* Model::integStepCallback(), which has the same argument list.
*
* This method should be overridden in derived classes.  It is
* included here so that the derived class will not have to implement it if
* it is not necessary.
*
* @param state System state
*
* @return -1 on error, 0 otherwise.
*/
int Actuation::
step(const SimTK::State& s, int stepNumber)
{
    if (!proceed(stepNumber)) return(0);

    record(s);

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
* This method should be overridden in the child class.  It is
* included here so that the child class will not have to implement it if it
* is not necessary.
* @param state System state
*
* @return -1 on error, 0 otherwise.
*/
int Actuation::end(const SimTK::State& s)
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
int Actuation::
printResults(const string &aBaseName, const string &aDir, double aDT,
const string &aExtension)
{
    if (!getOn()) {
        log_info("Actuation.printResults: Off- not printing.");
        return 0;
    }

    std::string prefix = aBaseName + "_" + getName() + "_";
    Storage::printResult(_forceStore, prefix + "force", aDir, aDT, aExtension);
    Storage::printResult(_speedStore, prefix + "speed", aDir, aDT, aExtension);
    Storage::printResult(_powerStore, prefix + "power", aDir, aDT, aExtension);

    return(0);
}
//_____________________________________________________________________________
/**
* Utility to get number of "Enabled" actuators in model where Enabled is based
* on the "Property" rather than the live/state value.
*
*
* @return number of Actuators which are enabled
*/
int Actuation::
getNumEnabledActuators()
{

    const Set<Actuator>& actuators = _model->getActuators();
    int numActuators = actuators.getSize();
    int numEnabled = numActuators;
    for (int i = 0; i< numActuators; i++)
        if(!actuators[i].get_appliesForce())
            numEnabled--;

    return numEnabled;
}
