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

/// Turn off passive fiber forces for all DeGrooteFregly2016Muscle%s in the
/// model.
class OSIMMOCO_API ModOpIgnorePassiveFiberForcesDGF : public ModelOperator {
    OpenSim_DECLARE_CONCRETE_OBJECT(
            ModOpIgnorePassiveFiberForcesDGF, ModelOperator);

public:
    void operate(Model& model, const std::string&) const override {
        model.finalizeFromProperties();
        for (auto& muscle :
                model.updComponentList<DeGrooteFregly2016Muscle>()) {
            muscle.set_ignore_passive_fiber_force(true);
        }
    }
};

/// Scale the active fiber force curve width for all DeGrooteFregly2016Muscle%s
/// in the model.
class OSIMMOCO_API ModOpScaleActiveFiberForceCurveWidthDGF :
        public ModelOperator {
    OpenSim_DECLARE_CONCRETE_OBJECT(
            ModOpScaleActiveFiberForceCurveWidthDGF, ModelOperator);
    OpenSim_DECLARE_PROPERTY(scale_factor, double,
            "The active fiber force curve width scale factor.");
public:
    ModOpScaleActiveFiberForceCurveWidthDGF() {
        constructProperty_scale_factor(1);
    }
    ModOpScaleActiveFiberForceCurveWidthDGF(double scaleFactor)
            : ModOpScaleActiveFiberForceCurveWidthDGF() {
        set_scale_factor(scaleFactor);
    }
    void operate(Model& model, const std::string&) const override {
        model.finalizeFromProperties();
        for (auto& muscle :
                model.updComponentList<DeGrooteFregly2016Muscle>()) {
            muscle.set_active_force_width_scale(get_scale_factor());
        }
    }
};

/// Scale the max isometric force for all muscles in the model.
class OSIMMOCO_API ModOpScaleMaxIsometricForce : public ModelOperator {
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
            muscle.set_max_isometric_force(get_scale_factor());
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
    OpenSim_DECLARE_PROPERTY(skip_coordinates_with_actuators, bool,
            "Whether or not to skip coordinates with existing actuators. "
            "Default: true.")
public:
    ModOpAddReserves() {
        constructProperty_optimal_force(1);
        constructProperty_skip_coordinates_with_actuators(true);
    }
    ModOpAddReserves(double optimalForce) : ModOpAddReserves() {
        set_optimal_force(optimalForce);
    }
    ModOpAddReserves(double optimalForce, bool skipCoordsWithActu)
            : ModOpAddReserves() {
        set_optimal_force(optimalForce);
        set_skip_coordinates_with_actuators(skipCoordsWithActu);
    }
    void operate(Model& model, const std::string&) const override {
        model.initSystem();
        ModelFactory::createReserveActuators(model, get_optimal_force(),
                get_skip_coordinates_with_actuators());
    }
};

/// Add external loads (e.g., ground reaction forces) to the model from a
/// XML file.
class OSIMMOCO_API ModOpAddExternalLoads : public ModelOperator {
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
        InverseDynamicsTool idTool;
        idTool.createExternalLoads(path, model);
    }
};

class OSIMMOCO_API ModOpReplaceJointsWithWelds : public ModelOperator {
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

/// Turn on or off path length approximation for all GeometryPath components in
/// the model.
class OSIMMOCO_API ModOpSetPathLengthApproximation : public ModelOperator {
    OpenSim_DECLARE_CONCRETE_OBJECT(
            ModOpSetPathLengthApproximation, ModelOperator);
    OpenSim_DECLARE_PROPERTY(use_approximation, bool, "Whether or not to use "
            "path length approximation for GeometryPath components. "
            "Default: false.")

public:
    ModOpSetPathLengthApproximation() {
        constructProperty_use_approximation(false);
    }
    ModOpSetPathLengthApproximation(bool useApproximation)
            : ModOpSetPathLengthApproximation() {
        set_use_approximation(useApproximation);
    }
    void operate(Model& model, const std::string&) const override {
        model.finalizeFromProperties();
        for (auto& geometryPath  : model.updComponentList<GeometryPath>()) {
            geometryPath.set_use_approximation(get_use_approximation());
        }
    }
};

} // namespace OpenSim

#endif // MOCO_MODELOPERATORS_H
