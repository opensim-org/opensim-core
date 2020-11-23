/* -------------------------------------------------------------------------- *
 * OpenSim: MocoOutputConstraint.cpp                                            *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2020 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Nicholas Bianco                                                 *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0          *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

#include "MocoOutputConstraint.h"

using namespace OpenSim;

MocoOutputConstraint::MocoOutputConstraint() {
    constructProperties(); 
}

void MocoOutputConstraint::constructProperties() {
    constructProperty_output_paths();
    constructProperty_lower_bounds();
    constructProperty_upper_bounds();
}

void MocoOutputConstraint::addOutput(std::string outputPath, double lowerBound, 
        double upperBound) {
    append_output_paths(outputPath);
    append_lower_bounds(lowerBound);
    append_upper_bounds(upperBound);
}


void MocoOutputConstraint::initializeOnModelImpl(
        const Model& model, const MocoProblemInfo&) const {
    std::string componentPath;
    std::string outputName;
    std::string channelName;
    std::string alias;

    std::vector<MocoBounds> bounds;
    for (int i = 0; i < getProperty_output_paths().size(); ++i) {
        AbstractInput::parseConnecteePath(get_output_paths(i), componentPath,
                outputName, channelName, alias);
        const auto& component = getModel().getComponent(componentPath);
        const auto* abstractOutput = &component.getOutput(outputName);
        if (!dynamic_cast<const Output<double>*>(abstractOutput)) {
            OPENSIM_THROW_FRMOBJ(Exception,
                    "Data type of specified model output not supported.");
        }
        m_outputs.emplace_back(abstractOutput);
        bounds.emplace_back(get_lower_bounds(i), get_upper_bounds(i));
    }
    
    setNumEquations(m_outputs.size());
    MocoConstraintInfo info;
    info.setBounds(bounds);
    const_cast<MocoOutputConstraint*>(this)->setConstraintInfo(info);
}

void MocoOutputConstraint::calcPathConstraintErrorsImpl(
        const SimTK::State& state, SimTK::Vector& errors) const {
    getModel().realizeAcceleration(state);
    //getModel().getSystem().realize(state, m_output->getDependsOnStage());
    for (int i = 0; i < m_outputs.size(); ++i) {
        errors[i] = static_cast<const Output<double>*>(m_outputs[i].get())
                            ->getValue(state);
    }
}