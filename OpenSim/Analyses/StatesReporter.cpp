/* -------------------------------------------------------------------------- *
 *                        OpenSim:  StatesReporter.cpp                        *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Ayman Habib                                                     *
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
#include "StatesReporter.h"

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
StatesReporter::~StatesReporter()
{
}
//_____________________________________________________________________________
/**
 * Construct an StatesReporter object for recording the states of the model during a simulation.
 *
 * @param aModel Model for which the states are to be recorded.
 */
StatesReporter::StatesReporter(Model *aModel) :
    Analysis(aModel),
    _statesStore(1000,"ModelStates")
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
StatesReporter::StatesReporter(const std::string &aFileName):
Analysis(aFileName, false),
_statesStore(1000,"ModelStates")
{
    setNull();

    // Serialize from XML
    updateFromXMLDocument();

    // DESCRIPTION
    constructDescription();

    // STORAGE
    setupStorage();
}

// Copy constructor and virtual copy 
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 */
StatesReporter::StatesReporter(const StatesReporter &aStatesReporter):
Analysis(aStatesReporter),
_statesStore(aStatesReporter._statesStore)
{
    setNull();
    // COPY TYPE AND NAME
    *this = aStatesReporter;
}


//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Set NULL values for all member variables.
 */
void StatesReporter::
setNull()
{
    // NAME
    setName("StatesReporter");
}
//--------------------------------------------------------------------------
// OPERATORS
//--------------------------------------------------------------------------
StatesReporter& StatesReporter::operator=(const StatesReporter &aStatesReporter)
{
    // BASE CLASS
    Analysis::operator=(aStatesReporter);

    // STORAGE
    setupStorage();

    return (*this);
}
//_____________________________________________________________________________
/**
 * Allocate storage for the forces.
 */
void StatesReporter::
setupStorage()
{
    // ACCELERATIONS
    _statesStore.setDescription(getDescription());
    // Keep references o all storages in a list for uniform access from GUI
    _storageList.append(&_statesStore);
    _storageList.setMemoryOwner(false);
}

//-----------------------------------------------------------------------------
// DESCRIPTION
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Construct the description for the StatesReporter files.
 */
void StatesReporter::
constructDescription()
{
    char descrip[1024];

    strcpy(descrip, "\nThis file contains the states of a model ");
    strcat(descrip, "during a simulation.\n");
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
 * Construct the column labels for the StatesReporter storage files.
 */
void StatesReporter::
constructColumnLabels()
{
    if (_model)
    {
        // ASSIGN
        Array<string> columnLabels = _model->getStateVariableNames();
        columnLabels.insert(0, "time");     
        _statesStore.setColumnLabels(columnLabels);
    }
}

//=============================================================================
// ANALYSIS
//=============================================================================
//_____________________________________________________________________________
/**
 * Record the StatesReporter quantities.
 */
int StatesReporter::
record(const SimTK::State& s)
{
    if(_model==NULL) return(-1);

    // MAKE SURE ALL StatesReporter QUANTITIES ARE VALID
    _model->getMultibodySystem().realize(s, SimTK::Stage::Velocity );

    SimTK::Vector stateValues = _model->getStateVariableValues(s);
    StateVector nextRow(s.getTime(), stateValues);
    _statesStore.append(nextRow);

    return(0);
}
//_____________________________________________________________________________
/**
 * This method is called at the beginning of an analysis so that any
 * necessary initializations may be performed.
 *
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
int StatesReporter::
begin( const SimTK::State& s)
{
    if(!proceed()) return(0);
    // LABELS
    constructColumnLabels();
    // RESET STORAGE
    _statesStore.reset(s.getTime());

    // RECORD
    int status = 0;
    if(_statesStore.getSize()<=0) {
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
 * @param state System state
 *
 * @return -1 on error, 0 otherwise.
 */
int StatesReporter::
step(const SimTK::State& s, int stepNumber )
{
    if(!proceed(stepNumber)) return(0);

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
 * This method should be overridden in the child class.  It is
 * included here so that the child class will not have to implement it if it
 * is not necessary.
 * @param state System state
 *
 * @return -1 on error, 0 otherwise.
 */
int StatesReporter::
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
int StatesReporter::
printResults(const string &aBaseName,const string &aDir,double aDT,
                 const string &aExtension)
{
    if(!getOn()) {
        log_info("StatesReporter.printResults: Off- not printing.");
        return 0;
    }

    std::string prefix=aBaseName+"_"+getName()+"_";
    Storage::printResult(&_statesStore, prefix+"states", aDir, aDT, aExtension);

    return(0);
}
