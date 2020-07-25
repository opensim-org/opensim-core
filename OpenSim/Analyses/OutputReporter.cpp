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

    if (_model == nullptr)
        return -1;

    _pvtModel.reset(_model->clone());

    _tableReporterDouble = new TableReporter_<double>();
    _tableReporterDouble->setName("ReporterDouble");
    auto& inputDouble = _tableReporterDouble->updInput("inputs");
    _tableReporterVec3 = new TableReporter_<SimTK::Vec3>();
    _tableReporterVec3->setName("ReporterVec3");
    auto& inputVec3 = _tableReporterVec3->updInput("inputs");
    _tableReporterSpatialVec = new TableReporter_<SimTK::SpatialVec>();
    _tableReporterSpatialVec->setName("ReporterSpatialVec");
    auto& inputSpatialVec = _tableReporterSpatialVec->updInput("inputs");

    _pvtModel->addComponent(_tableReporterDouble.get());
    _pvtModel->addComponent(_tableReporterVec3.get());
    _pvtModel->addComponent(_tableReporterSpatialVec.get());

    _minimumStageToRealize = SimTK::Stage::Time;

    for (int i = 0; i < getProperty_output_paths().size(); ++i) {
        std::string componentPath;
        std::string outputName;
        std::string channelName;
        std::string alias;
        AbstractInput::parseConnecteePath(get_output_paths(i),
                                          componentPath, outputName,
                                          channelName, alias);
        // The user supplied a path relative to the model, but that won't be a
        // valid relative path from the reporters. Form an absolue path.
        if (componentPath.empty() || componentPath[0] != '/') {
            componentPath = "/" + componentPath;
        }
        const auto path = AbstractInput::composeConnecteePath(
                componentPath, outputName, channelName, alias);
        const auto& comp = _pvtModel->getComponent(componentPath);
        auto& out = comp.getOutput(outputName);
        if (out.getTypeName() == "double") {
            inputDouble.appendConnecteePath(path);
        }
        else if (out.getTypeName() == "Vec3") {
            inputVec3.appendConnecteePath(path);
        }
        else if (out.getTypeName() == "SpatialVec") {
            inputSpatialVec.appendConnecteePath(path);
        }
        else {
            log_warn("Output '{}' of type {} is not supported by "
                     "OutputReporter. Consider adding a TableReporter_ to the "
                     "Model.", out.getPathName(), out.getTypeName());
        }

        if (out.getDependsOnStage() > _minimumStageToRealize) {
            _minimumStageToRealize = out.getDependsOnStage();
        }
    }

    _pvtModel->initSystem();
    report(s);

    return 0;
}

int OutputReporter::step(const SimTK::State& s, int stepNumber)
{
    if (!proceed(stepNumber)) {
        return 0;
    }

    report(s);
    return 0;
}

int OutputReporter::end(const SimTK::State& s)
{
    if (!proceed()) return 0;
    report(s);
    return 0;
}

int OutputReporter::printResults(const std::string& baseName,
    const std::string& dir,  double dT, const std::string& extension)
{
    log_info("OutputReporter.printResults: ");

    if (!getOn()) {
        log_info("OutputReporter.printResults: Off- not printing.");
        return 0;
    }

    OPENSIM_THROW_IF_FRMOBJ(IO::Lowercase(extension) != ".sto",
        Exception, "Only writing results to '.sto' format is supported.");

    std::string prefix = dir.empty() ? "" : dir + "/";

    auto& tableD = _tableReporterDouble->getTable();
    if (tableD.getNumColumns()) {
        STOFileAdapter_<double>::write(tableD,
            prefix + baseName + "_Outputs" + extension);
    }

    auto& tableV3 = _tableReporterVec3->getTable();
    if (tableV3.getNumColumns()) {
        STOFileAdapter_<SimTK::Vec3>::write(tableV3,
            prefix + baseName + "_OutputsVec3" + extension);
    }

    auto& tableSV = _tableReporterSpatialVec->getTable();
    if (tableSV.getNumColumns()) {
        STOFileAdapter_<SimTK::SpatialVec>::write(tableSV,
            prefix + baseName + "_OutputsSpatialVec" + extension);
    }

    return 0;
}

void OutputReporter::report(const SimTK::State& s)
{
    _pvtModel->getMultibodySystem().realize(s, _minimumStageToRealize);
    _tableReporterDouble->report(s);
    _tableReporterVec3->report(s);
    _tableReporterSpatialVec->report(s);
}
