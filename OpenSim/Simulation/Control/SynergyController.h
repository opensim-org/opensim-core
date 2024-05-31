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

/**
 * \section SynergyVector
 * A vector that represents the control weights for a single synergy in a 
 * SynergyController. The size of the vector should be equal to the number of 
 * actuators connected to the controller.
 */
class OSIMSIMULATION_API SynergyVector : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(SynergyVector, Object);
public:
    OpenSim_DECLARE_PROPERTY(synergy_weights, SimTK::Vector, 
            "The vector of control weights for a single synergy.");
    SynergyVector();
    SynergyVector(std::string name, SimTK::Vector weights);
};

/**
 * \section SynergyController
 * A controller that computes controls for a model based on a linear combination
 * of a set of Input control signals and a set of synergy vectors. 
 * 
 * Each synergy vector represents a set of control weights that are multiplied 
 * by the Input control signals to compute the contribution to the total control
 * signal for that synergy. The synergy vectors should have the same size as the
 * number of actuators connected to the controller, and the controller expects
 * the number Input controls to be equal to the number of synergy vectors.
 */
class OSIMSIMULATION_API SynergyController : public InputController {
    OpenSim_DECLARE_CONCRETE_OBJECT(SynergyController, InputController);

public:
//=============================================================================
// PROPERTIES
//=============================================================================
    OpenSim_DECLARE_LIST_PROPERTY(synergy_vectors, SynergyVector, 
            "The set of synergy vectors that define the control weights for "
            "each synergy.");

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
    /**
     * Add a synergy vector to the controller. 
     * 
     * The size of the vector should be equal to the number of actuators
     * connected to the controller. Adding a synergy vector increases the number
     * of control inputs expected by the controller by one.
     */
    void addSynergyVector(const SimTK::Vector& vector);

    /**
     * Update an existing synergy vector in the controller. 
     * 
     * The size of the vector should be equal to the number of actuators
     * connected to the controller. 
     */
    void updSynergyVector(int index, const SimTK::Vector& vector);

    /**
     * Get a synergy vector by index. 
     */
    const SimTK::Vector& getSynergyVector(int index) const;

    /**
     * Get the number of synergies vectors in the controller. 
     * 
     * The controller expects this number of control Inputs to be connected when
     * the connections are finalized in the model.
     */
    int getNumSynergies() const;

    /**
     * Get all synergy vectors as a matrix. 
     * 
     * The number of rows in the matrix is equal to the number of actuators
     * connected to the controller, and the number of columns is equal to the
     * number of synergy vectors in the controller. 
     * 
     * @pre Requires Model::finalizeConnections() to have been called first.
     */
    SimTK::Matrix getSynergyVectorsAsMatrix() const;

    // INPUT CONTROLLER INTERFACE
    std::vector<std::string> getInputControlLabels() const override;
    void computeControlsImpl(const SimTK::State& s, 
            SimTK::Vector& controls) const override;

protected:
    // MODEL COMPONENT INTERFACE
    void extendConnectToModel(Model& model) override;

private:
    // Validate that all synergy vectors have the correct size (i.e., equal to 
    // the number of actuators connected to the controller).
    void validateSynergyVectorSizes() const;
};

} // namespace OpenSim

#endif // OPENSIM_SYNERGY_CONTROLLER_H
