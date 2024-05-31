#ifndef OPENSIM_SYNERGY_CONTROLLER_H
#define OPENSIM_SYNERGY_CONTROLLER_H
/* -------------------------------------------------------------------------- *
 *                       OpenSim:  SynergyController.h                        *
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

#include "InputController.h"

namespace OpenSim {

class OSIMSIMULATION_API SynergyVector : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(SynergyVector, Object);
public:
    OpenSim_DECLARE_PROPERTY(synergy_weights, SimTK::Vector, "TODO");
    SynergyVector();
    SynergyVector(std::string name, SimTK::Vector weights);
};


class OSIMSIMULATION_API SynergyController : public InputController {
    OpenSim_DECLARE_CONCRETE_OBJECT(SynergyController, InputController);

public:
//=============================================================================
// PROPERTIES
//=============================================================================
    OpenSim_DECLARE_LIST_PROPERTY(synergy_vectors, SynergyVector, "TODO");

//=============================================================================
// METHODS
//=============================================================================
    // CONSTRUCTION AND DESTRUCTION
    SynergyController();
    ~SynergyController() override;

    SynergyController(const SynergyController& other);
    SynergyController& operator=(const SynergyController& other);

    SynergyController(SynergyController&& other);
    SynergyController& operator=(SynergyController&& other);

    // GET AND SET
    void addSynergyVector(const SimTK::Vector& vector);
    void updSynergyVector(int index, const SimTK::Vector& vector);
    const SimTK::Vector& getSynergyVector(int index) const;
    SimTK::Matrix getSynergyVectorsAsMatrix() const;

    // INPUT CONTROLLER INTERFACE
    std::vector<std::string> getInputControlLabels() const override;
    void computeControlsImpl(const SimTK::State& s, 
            SimTK::Vector& controls) const override;

protected:
    // MODEL COMPONENT INTERFACE
    void extendConnectToModel(Model& model) override;

private:
    void checkSynergyVectors() const;

    std::vector<int> m_controlIndexesInConnecteeOrder;
};

} // namespace OpenSim

#endif // OPENSIM_SYNERGY_CONTROLLER_H
