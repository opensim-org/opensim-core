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
            modelFromFile.finalizeConnections();
            model = std::move(modelFromFile);
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
    /// Append all operations in another processor to this processor.
    /// The source table of the provided trajectory is ignored.
    ModelProcessor& append(const ModelProcessor& traj) {
        for (int i = 0; i < traj.getProperty_operators().size(); ++i) {
            append_operators(traj.get_operators(i));
        }
        return *this;
    }

private:
    bool m_modelProvided = false;
    mutable Model m_model;
};

inline ModelProcessor operator|(
        ModelProcessor left, const ModelOperator& right) {
    left.append(right);
    return left;
}

inline ModelProcessor& operator|(
        ModelProcessor& left, const ModelOperator& right) {
    return left.append(right);
}

inline ModelProcessor& operator|(
        ModelProcessor& left, const ModelProcessor& right) {
    return left.append(right);
}

class OSIMMOCO_API ModelReplaceMusclesWithDeGrooteFregly2016
        : public ModelOperator {
    OpenSim_DECLARE_CONCRETE_OBJECT(
            ModelReplaceMusclesWithDeGrooteFregly2016, ModelOperator);

public:
    void operate(Model& model) const override {
        model.printBasicInfo(std::cout);
        model.finalizeConnections();
        DeGrooteFregly2016Muscle::replaceMuscles(model);
    }
};

class OSIMMOCO_API ModelIgnoreActivationDynamics : public ModelOperator {
    OpenSim_DECLARE_CONCRETE_OBJECT(
            ModelIgnoreActivationDynamics, ModelOperator);

public:
    void operate(Model& model) const override {
        model.finalizeFromProperties();
        for (auto& muscle : model.updComponentList<Muscle>()) {
            muscle.set_ignore_activation_dynamics(true);
        }
    }
};

class OSIMMOCO_API ModelIgnoreTendonCompliance : public ModelOperator {
    OpenSim_DECLARE_CONCRETE_OBJECT(ModelIgnoreTendonCompliance, ModelOperator);

public:
    void operate(Model& model) const override {
        model.finalizeFromProperties();
        for (auto& muscle : model.updComponentList<Muscle>()) {
            muscle.set_ignore_tendon_compliance(true);
        }
    }
};

class OSIMMOCO_API ModelAddReserves : public ModelOperator {
    OpenSim_DECLARE_CONCRETE_OBJECT(ModelAddReserves, ModelOperator);
    OpenSim_DECLARE_PROPERTY(optimal_force, double, "TODO");

public:
    ModelAddReserves() { constructProperty_optimal_force(1); }
    ModelAddReserves(double optimalForce) : ModelAddReserves() {
        set_optimal_force(optimalForce);
    }
    void operate(Model& model) const override {
        model.initSystem();
        ModelFactory::createReserveActuators(model, get_optimal_force());
    }
};

class OSIMMOCO_API ModelAddExternalLoads : public ModelOperator {
    OpenSim_DECLARE_CONCRETE_OBJECT(ModelAddExternalLoads, ModelOperator);
    OpenSim_DECLARE_PROPERTY(filepath, std::string, "TODO");

public:
    ModelAddExternalLoads() { constructProperty_filepath(""); }
    ModelAddExternalLoads(std::string filepath) : ModelAddExternalLoads() {
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
