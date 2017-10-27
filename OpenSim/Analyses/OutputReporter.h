#ifndef OUTPUT_REPORTER_H_
#define OUTPUT_REPORTER_H_
/* -------------------------------------------------------------------------- *
 *                         OpenSim: OutputReporter.h                          *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s):                                                                 *
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
#include <OpenSim/Simulation/Model/Analysis.h>
#include <OpenSim/Common/Reporter.h>
#include <OpenSim/Common/STOFileAdapter.h>
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

/**
 * The OutputReporter Analysis is a wrapper for a TableReporter which
 * generates TimeSeriesTables of Output values that are listed by name 
 * as properties of this Analysis. OutputReporter enables the AnalyzeTool to
 * report on Outputs via the Analysis interface but backed by a TableReporter.
 * The OutputReporter currently only supports Outputs of type, double, Vec3
 * and SpatialVec.
 */
class OSIMANALYSES_API OutputReporter : public Analysis {
OpenSim_DECLARE_CONCRETE_OBJECT(OutputReporter, Analysis);

public:
OpenSim_DECLARE_LIST_PROPERTY(output_paths, std::string,
    "The names of Outputs to be reported. To select specific Component outputs"
    ", provide its path name");

//=============================================================================
// METHODS
//=============================================================================

    OutputReporter(Model *model = nullptr) : Analysis(model) {
        constructProperty_output_paths();
        setName("OutputReporter");
    }
    OutputReporter(const std::string& fileName) : 
            Analysis(fileName, false) {
        constructProperty_output_paths();
        setName("OutputReporter");
    }

    virtual ~OutputReporter() = default;

protected:
    //--------------------------------------------------------------------------
    // ANALYSIS INTERFACE
    //--------------------------------------------------------------------------
    int begin(SimTK::State& s) override;
    int step(const SimTK::State& s, int setNumber) override;
    int printResults(const std::string& baseName,
        const std::string& dir = "",
        double dT = -1.0,
        const std::string& extension = ".sto") override;

private:
    // Keep a reference to the underlying TableReporter(s) for this Analysis
    SimTK::ReferencePtr< TableReporter_<double> > _tableReporterDouble;
    SimTK::ReferencePtr< TableReporter_<SimTK::Vec3> > _tableReporterVec3;
    SimTK::ReferencePtr< TableReporter_<SimTK::SpatialVec> >
        _tableReporterSpatialVec;

//=============================================================================
};  // END of class OutputReporter


}; //namespace OpenSim


#endif // #ifndef OUTPUT_REPORTER_H_
