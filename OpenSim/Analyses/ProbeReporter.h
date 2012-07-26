#ifndef OPENSIM_PROBE_REPORTER_H_
#define OPENSIM_PROBE_REPORTER_H_

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
// INCLUDES
//=============================================================================
#include <iostream>
#include <string>
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/Analysis.h>
#include <OpenSim/Simulation/Model/Probe.h>
#include <OpenSim/Simulation/Model/ProbeSet.h>
#include "osimAnalysesDLL.h"
#include "SimTKsimbody.h"


#ifdef SWIG
    #ifdef OSIMANALYSES_API
        #undef OSIMANALYSES_API
        #define OSIMANALYSES_API
    #endif
#endif
//=============================================================================
//=============================================================================
namespace OpenSim { 

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
                                
                                ----------------------------
                                |  SimTK::Vector           |
                          |---> |  computeProbeOutputs(s)  | ----|
                          |     ----------------------------     |
  -------------------     |                                      |     --------------------------------------------
  |  ProbeReporter  | -----                                      ----> |  Output to internal Storage and          |
  |  Analysis       | -----                                      ----> |  ultimately to file at end of simulation |
  -------------------     |                                      |     --------------------------------------------
                          |     ----------------------------     |
                          |---> |  Array<string>           | ----|
                                |  getProbeOutputLabels()  |
                                ----------------------------

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
    // Copy constrctor and virtual copy 
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
    virtual void setModel(Model& aModel);

    //--------------------------------------------------------------------------
    // Analysis
    //--------------------------------------------------------------------------

//#ifndef SWIG
    virtual int
        begin(SimTK::State& s );
    virtual int
        step(const SimTK::State& s, int setNumber );
    virtual int
        end(SimTK::State& s );
protected:
    virtual int
        record(const SimTK::State& s );
//#endif
    //--------------------------------------------------------------------------
    // IO
    //--------------------------------------------------------------------------
public:
    virtual int
        printResults(const std::string &aBaseName,const std::string &aDir="",
        double aDT=-1.0,const std::string &aExtension=".sto");

//=============================================================================
};	// END of class ProbeReporter

}; //namespace
//=============================================================================
//=============================================================================


#endif // #ifndef OPENSIM_PROBE_REPORTER_H_
