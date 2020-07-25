#ifndef OPENSIM_PROBE_REPORTER_H_
#define OPENSIM_PROBE_REPORTER_H_
/* -------------------------------------------------------------------------- *
 *                         OpenSim:  ProbeReporter.h                          *
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
// INCLUDES
//=============================================================================
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/Analysis.h>
#include <OpenSim/Simulation/Model/Probe.h>
#include "osimAnalysesDLL.h"


#ifdef SWIG
    #ifdef OSIMANALYSES_API
        #undef OSIMANALYSES_API
        #define OSIMANALYSES_API
    #endif
#endif
//=============================================================================
//=============================================================================
namespace OpenSim { 

class ProbeSet;

/**
 * A class for reporting the outputs of all model-connected Probes to file during
 * a simulation. This analysis will, at each 'step_interval', cycle through all
 * model Probes and retrieve the SimTK::Vector of probe outputs via getProbeOutputs(s)
 * and store them into a Storage object, which will get recorded to a file at the
 * termination of the simulation. The column labels for each Probe output will
 * come from the overridden method Array<string> getProbeOutputLabels() from the specific
 * Probe subclass. A schematic of the ProbeReporter functionality is shown below
 * (diagram below assumes a single probe, but ProbeReporter will cycle through every
 * Probe in the Model):
   \verbatim
                                
                                DEVELOPER NEEDS TO IMPLEMENT
                                THIS INSIDE THE CHILD PROBE
                                ============================
                                |  SimTK::Vector           |
                          |---> |  computeProbeOutputs(s)  | ----|
                          |     ============================     |
  ===================     |                                      |     ==========================
  |  ProbeReporter  | -----                                      ----> |  Output to file        |
  |  Analysis       | -----                                      ----> |  at end of simulation  |
  ===================     |                                      |     ==========================
                          |     ============================     |
                          |---> |  Array<string>           | ----|
                                |  getProbeOutputLabels()  |
                                ============================
                                DEVELOPER NEEDS TO IMPLEMENT
                                THIS INSIDE THE CHILD PROBE

  \endverbatim
 * 
 *
 * @author Tim Dorn
 */
class OSIMANALYSES_API ProbeReporter : public Analysis 
{
    OpenSim_DECLARE_CONCRETE_OBJECT(ProbeReporter, Analysis);
//=============================================================================
// DATA
//=============================================================================
protected:

    /** Probe storage. */
    Storage _probeStore;

//=============================================================================
// METHODS
//=============================================================================
public:
    //--------------------------------------------------------------------------
    // Constructor(s) and Setup
    //--------------------------------------------------------------------------
    ProbeReporter(Model *aModel=0);
    ProbeReporter(const std::string &aFileName);
    // Copy constructor and virtual copy 
    ProbeReporter(const ProbeReporter &aObject);
    virtual ~ProbeReporter();
    
private:
    void setNull();
    void constructDescription();
    void constructColumnLabels(const SimTK::State& s);
    void allocateStorage();
    void deleteStorage();

public:

    //--------------------------------------------------------------------------
    // Operators
    //--------------------------------------------------------------------------
#ifndef SWIG
    ProbeReporter& operator=(const ProbeReporter &aActuation);
#endif

    //--------------------------------------------------------------------------
    // Get and Set
    //--------------------------------------------------------------------------
    // STORAGE
    const Storage& getProbeStorage() const
    {
        return _probeStore;
    };
    Storage& updProbeStorage()
    {
        return _probeStore;
    }

    // MODEL
    void setModel(Model& aModel) override;

    //--------------------------------------------------------------------------
    // Analysis
    //--------------------------------------------------------------------------

    int
        begin(const SimTK::State& s ) override;
    int
        step(const SimTK::State& s, int setNumber ) override;
    int
        end(const SimTK::State& s ) override;
protected:
    virtual int
        record(const SimTK::State& s );
public:
    void disableIntegrationOnlyProbes() {
        ProbeSet& probes = _model->updProbeSet();
        int nP = probes.getSize();

        for(int i=0 ; i<nP ; i++) {
            Probe& nextProbe = (Probe&)probes[i];
            if (nextProbe.getOperation()=="integrate" || nextProbe.getOperation()=="min" || nextProbe.getOperation()=="max"){
                nextProbe.setEnabled(false);
                log_warn("Disabling probe {} as invalid for non-integration "
                         "context.",
                        nextProbe.getName());
            }
        }
    }
    //--------------------------------------------------------------------------
    // IO
    //--------------------------------------------------------------------------
    int
        printResults(const std::string &aBaseName,const std::string &aDir="",
        double aDT=-1.0,const std::string &aExtension=".sto") override;

//=============================================================================
};  // END of class ProbeReporter

}; //namespace
//=============================================================================
//=============================================================================


#endif // #ifndef OPENSIM_PROBE_REPORTER_H_
