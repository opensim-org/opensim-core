#ifndef MOCO_MODELPROCESSOR_H
#define MOCO_MODELPROCESSOR_H
/* -------------------------------------------------------------------------- *
 * OpenSim Moco: ModelProcessor.h                                             *
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
#include "Components/ModelFactory.h"
#include "osimMocoDLL.h"

#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Tools/InverseDynamicsTool.h>

namespace OpenSim {

class OSIMMOCO_API ModelOperator : public Object {
    OpenSim_DECLARE_ABSTRACT_OBJECT(ModelOperator, Object);

public:
    virtual void operate(Model& model) const = 0;
};

class OSIMMOCO_API ModelProcessor : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(ModelProcessor, Object);

public:
    // TODO make Model a property here!
    OpenSim_DECLARE_PROPERTY(
            filepath, std::string, "File path to a Model (.osim).");
    OpenSim_DECLARE_LIST_PROPERTY(operators, ModelOperator,
            "Operators to apply to the source Model of this processor.");
    ModelProcessor() {
        constructProperty_filepath("");
        constructProperty_operators();
    }

    ModelProcessor(Model model) : ModelProcessor() {
        m_modelProvided = true;
        m_model = std::move(model);
    }

    ModelProcessor(std::string filepath) : ModelProcessor() {
        set_filepath(std::move(filepath));
    }

    Model process(std::string relativeToDirectory = {}) const {
        Model model;
        if (get_filepath().empty()) {
            if (m_modelProvided) {
                m_model.finalizeConnections();
                model = m_model;
            } else {
                OPENSIM_THROW_FRMOBJ(Exception, "No source model.");
            }
        } else {
            std::string path = get_filepath();
            if (!relativeToDirectory.empty()) {
                using SimTK::Pathname;
                path = Pathname::
                        getAbsolutePathnameUsingSpecifiedWorkingDirectory(
                                relativeToDirectory, path);
            }
            Model modelFromFile(path);
            model = std::move(modelFromFile);
            model.finalizeFromProperties();
            model.finalizeConnections();
        }

        for (int i = 0; i < getProperty_operators().size(); ++i) {
            get_operators(i).operate(model);
        }
        return model;
    }

    /// Append an operation to the end of the operations in this processor.
    ModelProcessor& append(const ModelOperator& op) {
        append_operators(op);
        return *this;
    }
    ModelProcessor& operator|(const ModelOperator& right) {
        return append(right);
    }

private:
    bool m_modelProvided = false;
    mutable Model m_model;
};

class OSIMMOCO_API ModOpReplaceMusclesWithDeGrooteFregly2016
        : public ModelOperator {
    OpenSim_DECLARE_CONCRETE_OBJECT(
            ModOpReplaceMusclesWithDeGrooteFregly2016, ModelOperator);

public:
    void operate(Model& model) const override {
        model.finalizeConnections();
        DeGrooteFregly2016Muscle::replaceMuscles(model);
    }
};

/// Ignore activation dynamics for all muscles in the model.
class OSIMMOCO_API ModOpIgnoreActivationDynamics : public ModelOperator {
    OpenSim_DECLARE_CONCRETE_OBJECT(
            ModOpIgnoreActivationDynamics, ModelOperator);

public:
    void operate(Model& model) const override {
        model.finalizeFromProperties();
        for (auto& muscle : model.updComponentList<Muscle>()) {
            muscle.set_ignore_activation_dynamics(true);
        }
    }
};

/// Ignore tendon compliance for all muscles in the model.
class OSIMMOCO_API ModOpIgnoreTendonCompliance : public ModelOperator {
    OpenSim_DECLARE_CONCRETE_OBJECT(ModOpIgnoreTendonCompliance, ModelOperator);

public:
    void operate(Model& model) const override {
        model.finalizeFromProperties();
        for (auto& muscle : model.updComponentList<Muscle>()) {
            muscle.set_ignore_tendon_compliance(true);
        }
    }
};

/// Add a reserve actuator (CoordinateActuator) for each
/// unconstrained coordinate in the model.
/// Each actuator will have the specified `optimal_force`.
class OSIMMOCO_API ModOpAddReserves : public ModelOperator {
    OpenSim_DECLARE_CONCRETE_OBJECT(ModOpAddReserves, ModelOperator);
    OpenSim_DECLARE_PROPERTY(optimal_force, double,
            "Optimal force to apply to each CoordinateActuator (default: 1).");

public:
    ModOpAddReserves() { constructProperty_optimal_force(1); }
    ModOpAddReserves(double optimalForce) : ModOpAddReserves() {
        set_optimal_force(optimalForce);
    }
    void operate(Model& model) const override {
        model.initSystem();
        OPENSIM_THROW_IF_FRMOBJ(get_optimal_force() < 0, Exception,
                format("Expected optimal force to be non-negative, but got %f.",
                        get_optimal_force()));
        ModelFactory::createReserveActuators(model, get_optimal_force());
    }
};

/// Add ExternalLoads (to model ground reaction forces, for example) to the
/// model from an ExternalLoads XML file.
class OSIMMOCO_API ModOpAddExternalLoads : public ModelOperator {
    OpenSim_DECLARE_CONCRETE_OBJECT(ModOpAddExternalLoads, ModelOperator);
    OpenSim_DECLARE_PROPERTY(filepath, std::string,
            "XML file (.xml) containing the forces applied to the model as "
            "ExternalLoads.");

public:
    ModOpAddExternalLoads() { constructProperty_filepath(""); }
    ModOpAddExternalLoads(std::string filepath) : ModOpAddExternalLoads() {
        set_filepath(std::move(filepath));
    }
    void operate(Model& model) const override {
        // TODO directory?
        InverseDynamicsTool idTool;
        idTool.createExternalLoads(get_filepath(), model);
    }
};

} // namespace OpenSim

#endif // MOCO_MODELPROCESSOR_H
