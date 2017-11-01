/* -------------------------------------------------------------------------- *
 *                        OpenSim: OutputReporter.cpp                         *
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
#include "OutputReporter.h"
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Common/IO.h>

using namespace OpenSim;
using namespace std;

//=============================================================================
// ANALYSIS
//=============================================================================
int OutputReporter::begin(const SimTK::State& s)
{
    if (!proceed()) return 0;

    _pvtModel.reset(_model->clone());

    _tableReporterDouble = new TableReporter_<double>();
    _tableReporterVec3 = new TableReporter_<SimTK::Vec3>();
    _tableReporterSpatialVec = new TableReporter_<SimTK::SpatialVec>();

    _pvtModel->addComponent(_tableReporterDouble.get());
    _pvtModel->addComponent(_tableReporterVec3.get());
    _pvtModel->addComponent(_tableReporterSpatialVec.get());

    for (int i = 0; i < getProperty_output_paths().size(); ++i) {
        auto outpath = ComponentPath(get_output_paths(i));
        // assume that Outputs are for the Model
        string compName = _pvtModel->getAbsolutePathString();
        // if a a ComponentPath is specified then use the parent path
        if (outpath.getNumPathLevels()) {
            compName = outpath.getParentPathString();
        }
        auto outputName = outpath.getComponentName();

        // In keeping with assumption of Outputs for the Model (Component)
        const Component* comp = _pvtModel.get();

        // If the ComponentPath has a Component name get that Component
        if (!compName.empty())
            comp = &_pvtModel->getComponent(compName);

        auto& out = comp->getOutput(outputName);
        if (out.getTypeName() == "double") {
            _tableReporterDouble->addToReport(out);
        }
        else if (out.getTypeName() == "Vec3") {
            _tableReporterVec3->addToReport(out);
        }
        else if (out.getTypeName() == "SpatialVec") {
            _tableReporterSpatialVec->addToReport(out);
        }
        else {
            cout << "Output '" << out.getPathName() << "' of type "
                << out.getTypeName() << " is not supported by OutputReporter."
                << " Consider adding a TableReporter_ to the Model." << endl;
        }
    }

    _pvtModel->initSystem();
    _pvtModel->realizeReport(s);

    return 0;
}

int OutputReporter::step(const SimTK::State& s, int stepNumber)
{
    if (!proceed(stepNumber)) {
        return 0;
    }
    if (_pvtModel == nullptr)
        return -1;

    _pvtModel->realizeReport(s);

    return 0;
}

int OutputReporter::end(const SimTK::State& s)
{
    if (!proceed()) return 0;
    _pvtModel->realizeReport(s);
    return 0;
}

int OutputReporter::printResults(const std::string& baseName,
    const std::string& dir,  double dT, const std::string& extension)
{
    if (!getOn()) {
        printf("OutputReporter.printResults: Off- not printing.\n");
        return 0;
    }

    OPENSIM_THROW_IF_FRMOBJ(IO::Lowercase(extension) != ".sto",
        Exception, "Only writing results to '.sto' format is supported.");

    auto& tableD = _tableReporterDouble->getTable();
    if (tableD.getNumColumns()) {
        STOFileAdapter_<double>::write(tableD,
            dir + "/"+ baseName + "_Outputs" + extension);
    }

    auto& tableV3 = _tableReporterVec3->getTable();
    if (tableV3.getNumColumns()) {
        STOFileAdapter_<SimTK::Vec3>::write(tableV3,
            dir + "/" + baseName + "_OutputsVec3" + extension);
    }

    auto& tableSV = _tableReporterSpatialVec->getTable();
    if (tableSV.getNumColumns()) {
        STOFileAdapter_<SimTK::SpatialVec>::write(tableSV,
            dir + "/" + baseName + "_OutputsSpatialVec" + extension);
    }

    return 0;
}