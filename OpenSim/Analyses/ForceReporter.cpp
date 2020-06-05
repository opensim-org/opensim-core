/* -------------------------------------------------------------------------- *
 *                        OpenSim:  ForceReporter.cpp                         *
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
#include "ForceReporter.h"
#include <OpenSim/Simulation/Model/Model.h>

using namespace OpenSim;
using namespace std;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
ForceReporter::~ForceReporter()
{
    deleteStorage();
}
//_____________________________________________________________________________
/**
 * Construct an ForceReporter object for recording the Forces exerted on a model during a simulation.
 *
 * @param aModel Model for which the Forces are to be recorded.
 */
ForceReporter::ForceReporter(Model *aModel) :   
    Analysis(aModel),
    _includeConstraintForces(_includeConstraintForcesProp.getValueBool()),
    _forceStore(1000,"ModelForces")
{
    // NULL
    setNull();

    // DESCRIPTION
    constructDescription();

    // STORAGE
    allocateStorage();

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
ForceReporter::ForceReporter(const std::string &aFileName): 
    Analysis(aFileName, false),
    _includeConstraintForces(_includeConstraintForcesProp.getValueBool()),
    _forceStore(1000,"ModelForces")
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
ForceReporter::ForceReporter(const ForceReporter &aForceReporter):
    Analysis(aForceReporter),
    _includeConstraintForces(_includeConstraintForcesProp.getValueBool()),
    _forceStore(aForceReporter._forceStore)
{
    setNull();
    // COPY TYPE AND NAME
    *this = aForceReporter;
}


//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Set NULL values for all member variables.
 */
void ForceReporter::setNull()
{
    setAuthors("Ayman Habib ");
    // NAME
    setName("ForceReporter");

    _includeConstraintForcesProp.setComment("Flag indicating whether to include forces due to constraints.");
    _includeConstraintForcesProp.setName("include_constraint_forces");
    _includeConstraintForcesProp.setValue(false);
    _propertySet.append( &_includeConstraintForcesProp );
}


//--------------------------------------------------------------------------
// OPERATORS
//--------------------------------------------------------------------------
ForceReporter& ForceReporter::operator=(const ForceReporter &aForceReporter)
{
    // BASE CLASS
    Analysis::operator=(aForceReporter);

    // STORAGE
    deleteStorage();
    allocateStorage();

    _includeConstraintForces = aForceReporter._includeConstraintForces;

    return (*this);
}
//_____________________________________________________________________________
/**
 * Set the model pointer for analysis.
 */
void ForceReporter::setModel(Model& aModel)
{
    // BASE CLASS
    Analysis::setModel(aModel);
}

//_____________________________________________________________________________
/**
 * Allocate storage for the forces.
 */
void ForceReporter::allocateStorage()
{
    // ACCELERATIONS
    _forceStore.setDescription(getDescription());
    // Keep references o all storages in a list for uniform access from GUI
    _storageList.append(&_forceStore);
    _storageList.setMemoryOwner(false);
}

//-----------------------------------------------------------------------------
// DESCRIPTION
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Construct the description for the ForceReporter files.
 */
void ForceReporter::constructDescription()
{
    char descrip[1024];

    strcpy(descrip, "\nThis file contains the forces exerted on a model ");
    strcat(descrip, "during a simulation.\n");

    strcat(descrip, "\nA force is a generalized force, meaning that");
    strcat(descrip, " it can be either a force (N) or a torque (Nm).\n");

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
 * Construct the column labels for the ForceReporter storage files.
 */
void ForceReporter::constructColumnLabels(const SimTK::State& s)
{
    if (_model)
    {
        // ASSIGN
        Array<string> columnLabels;
        columnLabels.append("time");
        
        auto forces = _model->getComponentList<Force>();

        for(auto& force : forces) {
            // If body force we need to record six values for torque+force
            // If muscle we record one scalar
            if(!force.appliesForce(s)) continue; // Skip over disabled forces
            Array<string> forceLabels = force.getRecordLabels();
            // If prescribed force we need to record point, 
            columnLabels.append(forceLabels);
        }

        if(_includeConstraintForces){
            auto constraints = _model->getComponentList<Constraint>();
            for(auto& c : constraints) {
                if (!c.isEnforced(s))
                    continue; // Skip over disabled constraints
                // Ask constraint how many columns and their names it reports
                Array<string> forceLabels = c.getRecordLabels();
                // If prescribed force we need to record point, 
                columnLabels.append(forceLabels);
            }
        }
        _forceStore.setColumnLabels(columnLabels);
    }
}


//=============================================================================
// DESTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Delete storage objects.
 */
void ForceReporter::deleteStorage()
{
    //if(_forceStore!=NULL) { delete _forceStore;  _forceStore=NULL; }
}

//=============================================================================
// ANALYSIS
//=============================================================================
//_____________________________________________________________________________
/**
 * Record the ForceReporter quantities.
 */
int ForceReporter::record(const SimTK::State& s)
{
    if(_model==NULL) return(-1);

    // MAKE SURE ALL ForceReporter QUANTITIES ARE VALID
    _model->getMultibodySystem().realize(s, SimTK::Stage::Dynamics );

    StateVector nextRow(s.getTime());

    // Model Forces
    auto forces = _model->getComponentList<Force>();

    for(auto& force : forces) {
        // If body force we need to record six values for torque+force
        // If muscle we record one scalar
        if(!force.appliesForce(s)) continue;
        Array<double> values = force.getRecordValues(s);
        nextRow.getData().append(values);
    }

    if(_includeConstraintForces){
        // Model Constraints
        auto constraints = _model->getComponentList<Constraint>();
        for (auto& constraint : constraints) {
            if (!constraint.isEnforced(s))
                continue;
            Array<double> values = constraint.getRecordValues(s);
            nextRow.getData().append(values);
        }
    }
    _forceStore.append(nextRow);

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
int ForceReporter::begin(const SimTK::State& s)
{
    if(!proceed()) return(0);

    tidyForceNames();
    // LABELS
    constructColumnLabels(s);
    // RESET STORAGE
    _forceStore.reset(s.getTime());

    // RECORD
    int status = 0;
    if(_forceStore.getSize()<=0) {
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
int ForceReporter::step(const SimTK::State& s, int stepNumber )
{
    if(!proceed( stepNumber )) return(0);

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
int ForceReporter::end(const SimTK::State& s )
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
int ForceReporter::
printResults(const string &aBaseName,const string &aDir,double aDT,
                 const string &aExtension)
{
    if(!getOn()) {
        log_info("ForceReporter.printResults: Off- not printing.");
        return 0;
    }

    std::string prefix=aBaseName+"_"+getName()+"_";
    Storage::printResult(&_forceStore, prefix+"forces", aDir, aDT, aExtension);

    return(0);
}


void ForceReporter::tidyForceNames()
{
    OpenSim::Array<string> forceNames("");
    const ForceSet& forces = _model->getForceSet(); // This does not contain gravity
    forces.getNames(forceNames);
    // Make sure names are unique and non-empty. If empty assign names AutoForcexxx
    int nf = forceNames.getSize();
    for(int i=0; i<nf; i++){
        int j=1;
        if (forceNames[i]==""){
            bool validNameFound=false;
            char pad[100];
            // Make up a candidate name
            sprintf(pad, "%s%d", 
                    forces.get(i).getConcreteClassName().c_str(), j++);
            while(!validNameFound){
                validNameFound = (forceNames.findIndex(string(pad))== -1);
                if (!validNameFound){
                    sprintf(pad, "Force%d", j++);
                }
            }
            string newName(pad);
            _model->updForceSet()[i].setName(newName);
            forceNames.set(i, newName);
            log_info("Changing blank name for force to {}.", newName);
        }
    }
}
