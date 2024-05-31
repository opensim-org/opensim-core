/* -------------------------------------------------------------------------- *
 *                       OpenSim:  SynergyController.cpp                      *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2024 Stanford University and the Authors                *
 * Author(s): Nicholas Bianco                                                 *
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

#include "SynergyController.h"

using namespace OpenSim;

//=============================================================================
// SYNERGY VECTOR
//=============================================================================
SynergyVector::SynergyVector() {
    constructProperty_synergy_weights(SimTK::Vector());
}

SynergyVector::SynergyVector(std::string name, SimTK::Vector weights) 
        : SynergyVector() {
    setName(std::move(name));
    set_synergy_weights(std::move(weights));
}

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
SynergyController::SynergyController() : InputController() {
    constructProperty_synergy_vectors();
}

SynergyController::~SynergyController() = default;

SynergyController::SynergyController(const SynergyController& other) = default;

SynergyController&
SynergyController::operator=(const SynergyController& other) = default;

SynergyController::SynergyController(SynergyController&& other) = default;

SynergyController& 
SynergyController::operator=(SynergyController&& other) = default;

//=============================================================================
// INPUT CONTROLLER INTERFACE
//=============================================================================
std::vector<std::string> SynergyController::getInputControlLabels() const {
    std::vector<std::string> labels;
    for (int i = 0; i < getProperty_synergy_vectors().size(); ++i) {
        labels.push_back(fmt::format("synergy_excitation_{}", i));
    }
    return labels;
}

void SynergyController::computeControlsImpl(const SimTK::State& state,
        SimTK::Vector& controls) const {
    const auto& input = getInput<double>("controls");
    const auto& indexes = getControlIndexes();
    for (int i = 0; i < getProperty_synergy_vectors().size(); ++i) {
        SimTK::Vector synergy = get_synergy_vectors(i).get_synergy_weights() * 
                    input.getValue(state, i);
        for (int ic = 0; ic < indexes.size(); ++ic) {
            controls[indexes[ic]] += synergy[ic];
        }
    }
}

//=============================================================================
// MODEL COMPONENT INTERFACE
//=============================================================================
void SynergyController::extendConnectToModel(Model& model) {
    Super::extendConnectToModel(model);
    validateSynergyVectorSizes();
}

//=============================================================================
// METHODS
//=============================================================================
void SynergyController::addSynergyVector(const SimTK::Vector& vector) {
    int index = getProperty_synergy_vectors().size();
    append_synergy_vectors(
            SynergyVector(fmt::format("synergy_vector_{}", index), vector));
}

void SynergyController::updSynergyVector(int index, 
        const SimTK::Vector& vector) {
    OPENSIM_THROW_IF_FRMOBJ(index < 0, Exception,
            "Expected a non-negative synergy vector index, but received {}.",
            index);
    int maxIndex = getProperty_synergy_vectors().size() - 1;
    OPENSIM_THROW_IF_FRMOBJ(index > maxIndex, Exception,
            "Expected a synergy vector index in the range [0, {}], but "
            "received {}.", maxIndex, index);
    upd_synergy_vectors(index).upd_synergy_weights() = vector;    
}

const SimTK::Vector& SynergyController::getSynergyVector(int index) const {
    OPENSIM_THROW_IF_FRMOBJ(index < 0, Exception,
            "Expected a non-negative synergy vector index, but received {}.",
            index);
    int maxIndex = getProperty_synergy_vectors().size() - 1;
    OPENSIM_THROW_IF_FRMOBJ(index > maxIndex, Exception,
            "Expected a synergy vector index in the range [0, {}], but "
            "received {}.", maxIndex, index);
    return get_synergy_vectors(index).get_synergy_weights();
}

int SynergyController::getNumSynergies() const {
    return getProperty_synergy_vectors().size();
}

SimTK::Matrix SynergyController::getSynergyVectorsAsMatrix() const {
    validateSynergyVectorSizes();
    SimTK::Matrix matrix(getNumControls(), 
            getProperty_synergy_vectors().size());
    for (int i = 0; i < getProperty_synergy_vectors().size(); ++i) {
        matrix.updCol(i) = get_synergy_vectors(i).get_synergy_weights();
    }
    return matrix;
}

void SynergyController::validateSynergyVectorSizes() const {
    for (int i = 0; i < getProperty_synergy_vectors().size(); ++i) {
        const auto& weights = get_synergy_vectors(i).get_synergy_weights();
        const auto& name = get_synergy_vectors(i).getName();
        OPENSIM_THROW_IF(weights.size() != getNumControls(),
                Exception, "Expected {} to have size equal to the number "
                "controls associated with connected actuators {}, "
                "but it has size {}.", name, getNumControls());
    }
}
