/* -------------------------------------------------------------------------- *
 *                        OpenSim:  ProbeReporter.cpp                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Tim Dorn                                                        *
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
// INCLUDES and STATICS
//=============================================================================
#include "ProbeReporter.h"

using namespace OpenSim;
using namespace std;

//=============================================================================
// CONSTRUCTOR(S) AND SETUP
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
ProbeReporter::~ProbeReporter()
{
    deleteStorage();
}
//_____________________________________________________________________________
/**
 * Construct an ProbeReporter object for recording the Probes on a model during a simulation.
 *
 * @param aModel Model for which the Probes are to be recorded.
 */
ProbeReporter::ProbeReporter(Model *aModel) : Analysis(aModel),
    _probeStore(1000,"ModelProbes")
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
ProbeReporter::ProbeReporter(const std::string &aFileName): Analysis(aFileName, false),
    _probeStore(1000,"ModelProbes")
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
ProbeReporter::ProbeReporter(const ProbeReporter &aProbeReporter):
    Analysis(aProbeReporter),
    _probeStore(aProbeReporter._probeStore)
{
    setNull();
    // COPY TYPE AND NAME
    *this = aProbeReporter;
}


//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Set NULL values for all member variables.
 */
void ProbeReporter::setNull()
{
    setAuthors("Tim Dorn");
    // NAME
    setName("ProbeReporter");
}


//--------------------------------------------------------------------------
// OPERATORS
//--------------------------------------------------------------------------
ProbeReporter& ProbeReporter::operator=(const ProbeReporter &aProbeReporter)
{
    // BASE CLASS
    Super::operator=(aProbeReporter);

    // STORAGE
    deleteStorage();
    allocateStorage();

    return (*this);
}
//_____________________________________________________________________________
/**
 * Set the model pointer for analysis.
 */
void ProbeReporter::setModel(Model& aModel)
{
    // BASE CLASS
    Super::setModel(aModel);
}

//_____________________________________________________________________________
/**
 * Allocate storage for the probe outputs.
 */
void ProbeReporter::allocateStorage()
{
    // ACCELERATIONS
    _probeStore.setDescription(getDescription());
    // Keep references to all storages in a list for uniform access from GUI
    _storageList.append(&_probeStore);
    _storageList.setMemoryOwner(false);
}

//-----------------------------------------------------------------------------
// DESCRIPTION
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Construct the description for the ProbeReporter files.
 */
void ProbeReporter::constructDescription()
{
    char descrip[1024];

    strcpy(descrip, "\nThis file contains the probes on a model ");
    strcat(descrip, "during a simulation.\n");

    strcat(descrip, "\nThe units used are dependent on the type of probe component");

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
 * Construct the column labels for the ProbeReporter storage files.
 */
void ProbeReporter::constructColumnLabels(const SimTK::State& s)
{
    if (_model)
    {
        // ASSIGN
        Array<string> columnLabels;
        columnLabels.append("time");
        int nP=_model->getProbeSet().getSize();

        for(int i=0 ; i<nP ; i++) {
            Probe& p = _model->getProbeSet().get(i);

            if (!p.isEnabled()) continue; // Skip over disabled probes

            // Get column names for the probe after the operation
            Array<string> probeLabels = p.getProbeOutputLabels();
            columnLabels.append(probeLabels);
        }
        //cout << "COL SIZE = " << columnLabels.getSize() << endl;
        _probeStore.setColumnLabels(columnLabels);
    }
}


//=============================================================================
// DESTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Delete storage objects.
 */
void ProbeReporter::deleteStorage()
{
    //if(_probeStore!=NULL) { delete _probeStore;  _probeStore=NULL; }
}

//=============================================================================
// ANALYSIS
//=============================================================================
//_____________________________________________________________________________
/**
 * Record the ProbeReporter quantities.
 */
int ProbeReporter::record(const SimTK::State& s)
{
    if(_model==NULL) return -1;

    // MAKE SURE ALL ProbeReporter QUANTITIES ARE VALID
    _model->getMultibodySystem().realize(s, SimTK::Stage::Report );

    StateVector nextRow(s.getTime());

    // NUMBER OF Probes
    const ProbeSet& probes = _model->getProbeSet();
    int nP = probes.getSize();

    for(int i=0 ; i<nP ; i++) {
        Probe& nextProbe = (Probe&)probes[i];

        if (nextProbe.isEnabled())
        {
            // Get probe values after the probe operation
            SimTK::Vector values = nextProbe.getProbeOutputs(s);
            for (int i=0; i<values.size(); i++)
                nextRow.getData().append(values(i));
        }
    }

    _probeStore.append(nextRow);

    return 0;
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
int ProbeReporter::
begin(const SimTK::State& s)
{
    if(!proceed()) return 0;

    // LABELS
    constructColumnLabels(s);
    // RESET STORAGE
    _probeStore.reset(s.getTime());

    // RECORD
    int status = 0;
    if(_probeStore.getSize()<=0) {
        status = record(s);
    }

    return status;
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
int ProbeReporter::
step(const SimTK::State& s, int stepNumber )
{
    if(!proceed( stepNumber )) return 0;

    record(s);

    return 0;
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
int ProbeReporter::
end(const SimTK::State&s )
{
    if (!proceed()) return 0;

    record(s);

    return 0;
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
int ProbeReporter::
printResults(const string &aBaseName,const string &aDir,double aDT,
                 const string &aExtension)
{
    if(!getOn()) {
        log_info("ProbeReporter.printResults: Off- not printing.");
        return 0;
    }

    std::string prefix=aBaseName+"_"+getName()+"_";
    Storage::printResult(&_probeStore, prefix+"probes", aDir, aDT, aExtension);

    return 0;
}

