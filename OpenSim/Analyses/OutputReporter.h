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
 * The OutputReporter Analysis is a wrapper for a TableReporter. It generates
 * and writes TimeSeriesTables of Output values according to the names listed
 * as properties of the Analysis. OutputReporter enables the AnalyzeTool to
 * report on Outputs via the Analysis interface but backed by a TableReporter.
 * The OutputReporter currently only supports Outputs of type: double, Vec3
 * and SpatialVec. The OutputReporter will automatically write multiple files-
 * a file for each supported Output type: 
 *   - `<results-file-name>.sto` (as doubles), 
 *   - `<results-file-name>Vec3.sto`, and
 *   - `<results-file-name>SpatialVec.sto`.
 *
 * Output paths can be absolute (e.g., `/joint/slider/tx|value`) or relative
 * to the model (by leaving off the first slash; `joint/slider/tx|value`).
 * For outputs on the model itself, you can use `|com_position`, etc. As
 * explained for AbstractInput, the vertical bar denotes the output name.
 *
 * Note that the internal tables are reset at the beginning of a simulation or
 * AnalyzeTool::run() and does not append results to previous tables.
 */
class OSIMANALYSES_API OutputReporter : public Analysis {
OpenSim_DECLARE_CONCRETE_OBJECT(OutputReporter, Analysis);

public:
OpenSim_DECLARE_LIST_PROPERTY(output_paths, std::string,
    "The names of Outputs to be reported. To select specific Component Outputs"
    ", provide its path name. For example, '/jointset/slider/tx|value' is an "
    "Output for the value of a Coordinate 'tx' belonging to the Joint "
    "'slider'.");

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
    int begin(const SimTK::State& s) override final;
    int step(const SimTK::State& s, int setNumber) override final;
    int end(const SimTK::State& s) override final;

    int printResults(const std::string& baseName,
        const std::string& dir = "",
        double dT = -1.0,
        const std::string& extension = ".sto") override;

private:
    // Invoke the reporting of the Outputs to the Tables
    void report(const SimTK::State& s);

    SimTK::Stage _minimumStageToRealize{};

    // Keep a reference to the underlying TableReporter(s) for this Analysis
    SimTK::ReferencePtr< TableReporter_<double> > _tableReporterDouble;
    SimTK::ReferencePtr< TableReporter_<SimTK::Vec3> > _tableReporterVec3;
    SimTK::ReferencePtr< TableReporter_<SimTK::SpatialVec> >
        _tableReporterSpatialVec;

    SimTK::ResetOnCopy<std::unique_ptr<Model>> _pvtModel;

//=============================================================================
};  // END of class OutputReporter


}; //namespace OpenSim


#endif // #ifndef OUTPUT_REPORTER_H_
