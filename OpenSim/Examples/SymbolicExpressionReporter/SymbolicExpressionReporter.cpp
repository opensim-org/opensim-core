/* -------------------------------------------------------------------------- *
 *                  OpenSim:  SymbolicExpressionReporter.cpp                  *
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

#include "Lepton.h"
#include "SymbolicExpressionReporter.h"
#include <iostream>
#include <string>

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
SymbolicExpressionReporter::~SymbolicExpressionReporter()
{
}
//_____________________________________________________________________________
/**
 * Construct an SymbolicExpressionReporter object for recording an expression based on the states of the model during a simulation.
 *
 * @param aModel Model for which the states are to be recorded.
 */
SymbolicExpressionReporter::SymbolicExpressionReporter(Model *aModel) :
    Analysis(aModel),
    _expressionStr(_expressionStrProp.getValueStr())
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
SymbolicExpressionReporter::SymbolicExpressionReporter(const std::string &aFileName):
Analysis(aFileName, false),
_expressionStr(_expressionStrProp.getValueStr()),
_resultStore(100,"Expression")
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
SymbolicExpressionReporter::SymbolicExpressionReporter(const SymbolicExpressionReporter &aSymbolicExpressionReporter):
Analysis(aSymbolicExpressionReporter),
_expressionStr(_expressionStrProp.getValueStr())
{
    setNull();
    // COPY TYPE AND NAME
    *this = aSymbolicExpressionReporter;
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Set NULL values for all member variables.
 */
void SymbolicExpressionReporter::
setNull()
{
    // NAME
    setName("SymbolicExpressionReporter");

    setupProperties();
}
//--------------------------------------------------------------------------
// OPERATORS
//--------------------------------------------------------------------------
SymbolicExpressionReporter& SymbolicExpressionReporter::operator=(const SymbolicExpressionReporter &aSymbolicExpressionReporter)
{
    // BASE CLASS
    Analysis::operator=(aSymbolicExpressionReporter);

    // STORAGE
    setupStorage();

    _expressionStr = aSymbolicExpressionReporter._expressionStr;
    return (*this);
}
//_____________________________________________________________________________
/**
 * Set up the properties.
 */
void SymbolicExpressionReporter::
setupProperties()
{
    _expressionStrProp.setComment("Expression to be evaluated, variables allowed are state names.");
    _expressionStrProp.setName("expression");
    _propertySet.append( &_expressionStrProp );
}

//_____________________________________________________________________________
/**
 * Allocate storage for the forces.
 */
void SymbolicExpressionReporter::
setupStorage()
{
    // ACCELERATIONS
    _resultStore.setDescription(getDescription());
    // Keep references o all storages in a list for uniform access from GUI
    _storageList.append(&_resultStore);
    _storageList.setMemoryOwner(false);
}

//-----------------------------------------------------------------------------
// DESCRIPTION
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Construct the description for the SymbolicExpressionReporter files.
 */
void SymbolicExpressionReporter::
constructDescription()
{
    char descrip[1024];

    strcpy(descrip, "\nThis file contains expression evaluation ");
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
 * Construct the column labels for the SymbolicExpressionReporter storage files.
 */
void SymbolicExpressionReporter::
constructColumnLabels()
{
    if (_model)
    {
        // ASSIGN
        Array<string> columnLabels;
        columnLabels.insert(0, "time"); 
        columnLabels.insert(1, "expression");
        _resultStore.setColumnLabels(columnLabels);
    }
}

//=============================================================================
// ANALYSIS
//=============================================================================
//_____________________________________________________________________________
/**
 * Record the SymbolicExpressionReporter quantities.
 */
int SymbolicExpressionReporter::
record(const SimTK::State& s)
{
    if(_model==NULL) return(-1);

    // MAKE SURE ALL QUANTITIES ARE VALID
    _model->getMultibodySystem().realize(s, SimTK::Stage::Velocity );
    // Get state variable names and values in same order and use that to update map
    Array<std::string> stateNames = _model->getStateVariableNames();
    SimTK::Vector rStateValues = _model->getStateVariableValues(s);

    // update map between names and values, then pass it to expression evaluator
    for(int i=0; i<stateNames.getSize(); i++){
        _variables.find(stateNames[i])->second = rStateValues[i];
    }
    double value = Lepton::Parser::parse(_expressionStr).evaluate(_variables);
    StateVector nextRow{s.getTime(), {}};
     nextRow.getData().append(value);
    _resultStore.append(nextRow);

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
int SymbolicExpressionReporter::
begin( const SimTK::State& s)
{
    if(!proceed()) return(0);
    // LABELS
    constructColumnLabels();
    // RESET STORAGE
    _resultStore.reset(s.getTime());
    // Populate list of names, initial values from s
    Array<std::string> stateNames = _model->getStateVariableNames();
    for(int i=0; i< stateNames.getSize(); i++){
        _variables.insert(pair<string, double>(stateNames[i], 0.0));
    }
    // RECORD
    int status = 0;
    if(_resultStore.getSize()<=0) {
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
int SymbolicExpressionReporter::
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
int SymbolicExpressionReporter::
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
int SymbolicExpressionReporter::
printResults(const string &aBaseName,const string &aDir,double aDT,
                 const string &aExtension)
{
    if(!getOn()) {
        printf("SymbolicExpressionReporter.printResults: Off- not printing.\n");
        return(0);
    }

    std::string prefix=aBaseName+"_"+getName()+"_";
    Storage::printResult(&_resultStore, prefix+"states", aDir, aDT, aExtension);

    return(0);
}
