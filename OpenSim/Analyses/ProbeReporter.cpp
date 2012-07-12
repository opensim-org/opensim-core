// ProbeReporter.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Tim Dorn
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/* Copyright (c)  2012 Stanford University
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

// Copy constrctor and virtual copy 
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

    strcpy(descrip,"\nThis file contains the probes on a model ");
    strcat(descrip,"during a simulation.\n");

    strcat(descrip,"\nThe units used are dependent on the type of probe component");

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

            if (p.isDisabled()) continue; // Skip over disabled probes

            // Get column names for the probe after the operation
            Array<string> probeLabels = p.getProbeLabels();
            columnLabels.append(probeLabels);
        }
        
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
    if(_model==NULL) return(-1);

    // MAKE SURE ALL ProbeReporter QUANTITIES ARE VALID
    _model->getMultibodySystem().realize(s, SimTK::Stage::Report );

    StateVector nextRow = StateVector(s.getTime());

    // NUMBER OF Probes
    const ProbeSet& probes = _model->getProbeSet();
    int nP = probes.getSize();

    for(int i=0 ; i<nP ; i++) {
        Probe& nextProbe = (Probe&)probes[i];

        if (!nextProbe.isDisabled())
        {
            // Get probe values after the probe operation
            SimTK::Vector values = nextProbe.getProbeOutputs(s);
            for (int i=0; i<values.size(); i++)
                nextRow.getData().append(values(i));
        }
    }

    _probeStore.append(nextRow);

    return(0);
}
//_____________________________________________________________________________
/**
 * This method is called at the beginning of an analysis so that any
 * necessary initializations may be performed.
 *
 *
 * This method should be overriden in the child class.  It is
 * included here so that the child class will not have to implement it if it
 * is not necessary.
 *
 * @param state system State
 * @param aStep Step number of the integration.
 *
 * @return -1 on error, 0 otherwise.
 */
int ProbeReporter::
begin(SimTK::State& s)
{
    if(!proceed()) return(0);

    //tidyProbeNames();
    // LABELS
    constructColumnLabels(s);
    // RESET STORAGE
    _probeStore.reset(s.getTime());

    // RECORD
    int status = 0;
    if(_probeStore.getSize()<=0) {
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
 * @param state System state
 *
 * @return -1 on error, 0 otherwise.
 */
int ProbeReporter::
step(const SimTK::State& s, int stepNumber )
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
 * This method should be overriden in the child class.  It is
 * included here so that the child class will not have to implement it if it
 * is not necessary.
 * @param state System state
 *
 * @return -1 on error, 0 otherwise.
 */
int ProbeReporter::
end(SimTK::State& s )
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
int ProbeReporter::
printResults(const string &aBaseName,const string &aDir,double aDT,
                 const string &aExtension)
{
    if(!getOn()) {
        printf("ProbeReporter.printResults: Off- not printing.\n");
        return(0);
    }

    std::string prefix=aBaseName+"_"+getName()+"_";
    Storage::printResult(&_probeStore, prefix+"probes", aDir, aDT, aExtension);

    return(0);
}

