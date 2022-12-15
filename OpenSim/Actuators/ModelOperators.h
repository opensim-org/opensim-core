#ifndef OPENSIM_MODELOPERATORS_H
#define OPENSIM_MODELOPERATORS_H
/* -------------------------------------------------------------------------- *
 * OpenSim: ModelOperators.h                                                  *
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

#include "ModelProcessor.h"

#include <OpenSim/Actuators/ModelFactory.h>
#include <OpenSim/Simulation/Model/ExternalLoads.h>

namespace OpenSim {

/// Turn off activation dynamics for all muscles in the model.
class OSIMACTUATORS_API ModOpIgnoreActivationDynamics : public ModelOperator {
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
class OSIMACTUATORS_API ModOpIgnoreTendonCompliance : public ModelOperator {
    OpenSim_DECLARE_CONCRETE_OBJECT(ModOpIgnoreTendonCompliance, ModelOperator);

public:
    void operate(Model& model, const std::string&) const override {
        model.finalizeFromProperties();
        for (auto& muscle : model.updComponentList<Muscle>()) {
            muscle.set_ignore_tendon_compliance(true);
        }
    }
};

/** Scale the max isometric force for all muscles in the model. */
class OSIMACTUATORS_API ModOpScaleMaxIsometricForce : public ModelOperator {
    OpenSim_DECLARE_CONCRETE_OBJECT(ModOpScaleMaxIsometricForce, ModelOperator);
    OpenSim_DECLARE_PROPERTY(scale_factor, double,
            "The max isometric force scale factor.");

public:
    ModOpScaleMaxIsometricForce() {
        constructProperty_scale_factor(1);
    }
    ModOpScaleMaxIsometricForce(double scaleFactor)
            : ModOpScaleMaxIsometricForce() {
        set_scale_factor(scaleFactor);
    }
    void operate(Model& model, const std::string&) const override {
        model.finalizeFromProperties();
        for (auto& muscle : model.updComponentList<Muscle>()) {
            muscle.set_max_isometric_force(
                    get_scale_factor() * muscle.get_max_isometric_force());
        }
    }
};

/** Remove all muscles contained in the model's ForceSet. */
class OSIMACTUATORS_API ModOpRemoveMuscles : public ModelOperator {
    OpenSim_DECLARE_CONCRETE_OBJECT(ModOpRemoveMuscles, ModelOperator);

public:
    void operate(Model& model, const std::string&) const override {
        // Without finalizeFromProperties(), an exception is raised
        // about the model not having any subcomponents.
        model.finalizeFromProperties();
        model.finalizeConnections();
        ModelFactory::removeMuscles(model);
    }
};

/** Add reserve actuators to the model using
ModelFactory::createReserveActuators. */
class OSIMACTUATORS_API ModOpAddReserves : public ModelOperator {
    OpenSim_DECLARE_CONCRETE_OBJECT(ModOpAddReserves, ModelOperator);
    OpenSim_DECLARE_PROPERTY(optimal_force, double,
            "The optimal force for all added reserve actuators. Default: 1.");
    OpenSim_DECLARE_OPTIONAL_PROPERTY(bound, double,
            "Set the min and max control to -bound and bound, respectively. "
            "Default: no bounds.");
    OpenSim_DECLARE_PROPERTY(skip_coordinates_with_actuators, bool,
            "Whether or not to skip coordinates with existing actuators. "
            "Default: true.")
public:
    ModOpAddReserves() {
        constructProperty_optimal_force(1);
        constructProperty_bound();
        constructProperty_skip_coordinates_with_actuators(true);
    }
    ModOpAddReserves(double optimalForce) : ModOpAddReserves() {
        set_optimal_force(optimalForce);
    }
    ModOpAddReserves(double optimalForce, double bound)
            : ModOpAddReserves(optimalForce) {
        set_bound(bound);
    }
    ModOpAddReserves(
            double optimalForce, double bounds, bool skipCoordsWithActu)
            : ModOpAddReserves(optimalForce, bounds) {
        set_skip_coordinates_with_actuators(skipCoordsWithActu);
    }
    void operate(Model& model, const std::string&) const override {
        model.initSystem();
        ModelFactory::createReserveActuators(model, get_optimal_force(),
                getProperty_bound().empty() ? SimTK::NaN : get_bound(),
                get_skip_coordinates_with_actuators());
    }
};

/** Add external loads (e.g., ground reaction forces) to the model from a
XML file. The ExternalLoads setting
external_loads_model_kinematics_file is ignored. */
class OSIMACTUATORS_API ModOpAddExternalLoads : public ModelOperator {
    OpenSim_DECLARE_CONCRETE_OBJECT(ModOpAddExternalLoads, ModelOperator);
    OpenSim_DECLARE_PROPERTY(filepath, std::string,
            "External loads XML file.");

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
        model.addModelComponent(new ExternalLoads(path, true));
    }
};

class OSIMACTUATORS_API ModOpReplaceJointsWithWelds : public ModelOperator {
    OpenSim_DECLARE_CONCRETE_OBJECT(ModOpReplaceJointsWithWelds, ModelOperator);
    OpenSim_DECLARE_LIST_PROPERTY(joint_paths, std::string,
            "Paths to joints to replace with WeldJoints.");

public:
    ModOpReplaceJointsWithWelds() { constructProperty_joint_paths(); }
    ModOpReplaceJointsWithWelds(const std::vector<std::string>& paths) :
            ModOpReplaceJointsWithWelds() {
        for (const auto& path : paths) { append_joint_paths(path); }
    }
    void operate(Model& model, const std::string&) const override {
        model.initSystem();
        for (int i = 0; i < getProperty_joint_paths().size(); ++i) {
            ModelFactory::replaceJointWithWeldJoint(model, get_joint_paths(i));
        }
    }
};

/// Invoke ModelFactory::replaceMusclesWithPathActuators() on the model.
class OSIMACTUATORS_API ModOpReplaceMusclesWithPathActuators
        : public ModelOperator {
    OpenSim_DECLARE_CONCRETE_OBJECT(
            ModOpReplaceMusclesWithPathActuators, ModelOperator);

public:
    void operate(Model& model, const std::string&) const override {
        // Without finalizeFromProperties(), an exception is raised
        // about the model not having any subcomponents.
        model.finalizeFromProperties();
        model.finalizeConnections();
        ModelFactory::replaceMusclesWithPathActuators(model);
    }
};

} // namespace OpenSim

#endif // OPENSIM_MODELOPERATORS_H
