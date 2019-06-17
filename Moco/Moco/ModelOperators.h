#ifndef MOCO_MODELOPERATORS_H
#define MOCO_MODELOPERATORS_H
/* -------------------------------------------------------------------------- *
 * OpenSim Moco: ModelOperators.h                                             *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2019 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Christopher Dembia                                              *
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

#include "Components/DeGrooteFregly2016Muscle.h"
#include "ModelProcessor.h"

#include <OpenSim/Tools/InverseDynamicsTool.h>

namespace OpenSim {

/// Invoke DeGrooteFregly2016Muscle::replaceMuscles() on the model.
class OSIMMOCO_API ModOpReplaceMusclesWithDeGrooteFregly2016
        : public ModelOperator {
    OpenSim_DECLARE_CONCRETE_OBJECT(
            ModOpReplaceMusclesWithDeGrooteFregly2016, ModelOperator);

public:
    void operate(Model& model, const std::string&) const override {
        model.finalizeConnections();
        DeGrooteFregly2016Muscle::replaceMuscles(model);
    }
};

/// Turn off activation dynamics for all muscles in the model.
class OSIMMOCO_API ModOpIgnoreActivationDynamics : public ModelOperator {
    OpenSim_DECLARE_CONCRETE_OBJECT(
            ModOpIgnoreActivationDynamics, ModelOperator);

public:
    void operate(Model& model, const std::string&) const override {
        model.finalizeFromProperties();
        for (auto& muscle : model.updComponentList<Muscle>()) {
            muscle.set_ignore_activation_dynamics(true);
        }
    }
};

/// Turn off tendon compliance for all muscles in the model.
class OSIMMOCO_API ModOpIgnoreTendonCompliance : public ModelOperator {
    OpenSim_DECLARE_CONCRETE_OBJECT(ModOpIgnoreTendonCompliance, ModelOperator);

public:
    void operate(Model& model, const std::string&) const override {
        model.finalizeFromProperties();
        for (auto& muscle : model.updComponentList<Muscle>()) {
            muscle.set_ignore_tendon_compliance(true);
        }
    }
};

/// Remove all muscles contained in the model's ForceSet.
class OSIMMOCO_API ModOpRemoveMuscles : public ModelOperator {
    OpenSim_DECLARE_CONCRETE_OBJECT(ModOpRemoveMuscles, ModelOperator);

public:
    void operate(Model& model, const std::string&) const override {
        model.finalizeConnections();
        ModelFactory::removeMuscles(model);
    }
};

/// Add reserve actuators to the model using
/// ModelFactory::createReserveActuators.
class OSIMMOCO_API ModOpAddReserves : public ModelOperator {
    OpenSim_DECLARE_CONCRETE_OBJECT(ModOpAddReserves, ModelOperator);
    OpenSim_DECLARE_PROPERTY(optimal_force, double,
            "The optimal force for all added reserve actuators.");

public:
    ModOpAddReserves() { constructProperty_optimal_force(1); }
    ModOpAddReserves(double optimalForce) : ModOpAddReserves() {
        set_optimal_force(optimalForce);
    }
    void operate(Model& model, const std::string&) const override {
        model.initSystem();
        ModelFactory::createReserveActuators(model, get_optimal_force());
    }
};

/// Add external loads (e.g., ground reaction forces) to the model from a
/// XML file.
class OSIMMOCO_API ModOpAddExternalLoads : public ModelOperator {
    OpenSim_DECLARE_CONCRETE_OBJECT(ModOpAddExternalLoads, ModelOperator);
    OpenSim_DECLARE_PROPERTY(
            filepath, std::string, "External loads XML file.");

public:
    ModOpAddExternalLoads() { constructProperty_filepath(""); }
    ModOpAddExternalLoads(std::string filepath) : ModOpAddExternalLoads() {
        set_filepath(std::move(filepath));
    }
    /// The ExternalLoads XML file is located relative to `relativeToDirectory`.
    void operate(Model& model,
            const std::string& relativeToDirectory) const override {
        std::string path = get_filepath();
        if (!relativeToDirectory.empty()) {
            using SimTK::Pathname;
            path = Pathname::getAbsolutePathnameUsingSpecifiedWorkingDirectory(
                    relativeToDirectory, path);
        }
        InverseDynamicsTool idTool;
        idTool.createExternalLoads(path, model);
    }
};

} // namespace OpenSim

#endif // MOCO_MODELOPERATORS_H
