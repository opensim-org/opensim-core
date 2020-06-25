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

/// For DeGrooteFregly2016Muscle muscles whose 'ignore_tendon_compliance' 
/// property is false, set the tendon compliance dynamics mode to either 
/// 'explicit' or 'implicit'.
class OSIMMOCO_API ModOpTendonComplianceDynamicsModeDGF 
        : public ModelOperator {
    OpenSim_DECLARE_CONCRETE_OBJECT(
            ModOpTendonComplianceDynamicsModeDGF, ModelOperator);
    OpenSim_DECLARE_PROPERTY(mode, std::string,
            "The tendon compliance dynamics mode: 'implicit' or 'explicit'. "
            "Default: 'explicit'.");

public:
    ModOpTendonComplianceDynamicsModeDGF() {
        constructProperty_mode("explicit");
    }
    ModOpTendonComplianceDynamicsModeDGF(std::string mode) : 
            ModOpTendonComplianceDynamicsModeDGF() {
        OPENSIM_THROW_IF(mode != "explicit" && mode != "implicit", Exception,
                "The tendon compliance dynamics mode must be "
                "either 'explicit' or 'implicit', but {} was "
                "provided.",
                mode);
        set_mode(std::move(mode));
    }
    void operate(Model& model, const std::string&) const override {
        model.finalizeFromProperties();
        for (auto& muscle :
                model.updComponentList<DeGrooteFregly2016Muscle>()) {
            if (!muscle.get_ignore_tendon_compliance()) {
                muscle.set_tendon_compliance_dynamics_mode(get_mode());
            }
        }
    }
};

/// Set the tendon compliance dynamics mode to "implicit" for all 
/// DeGrooteFregly2016Muscle%s in the model.
class OSIMMOCO_API ModOpUseImplicitTendonComplianceDynamicsDGF
        : public ModelOperator {
    OpenSim_DECLARE_CONCRETE_OBJECT(
        ModOpUseImplicitTendonComplianceDynamicsDGF, ModelOperator);

public:
    void operate(Model& model, const std::string&) const override {
        model.finalizeFromProperties();
        for (auto& muscle :
                model.updComponentList<DeGrooteFregly2016Muscle>()) {
            if (!muscle.get_ignore_tendon_compliance()) {
                muscle.set_tendon_compliance_dynamics_mode("implicit");
            }
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

/// Set passive fiber stiffness for all DeGrooteFregly2016Muscle%s in the
/// model.
class OSIMMOCO_API ModOpPassiveFiberStrainAtOneNormForceDGF
        : public ModelOperator {
OpenSim_DECLARE_CONCRETE_OBJECT(
        ModOpPassiveFiberStrainAtOneNormForceDGF, ModelOperator);
    OpenSim_DECLARE_PROPERTY(passive_fiber_strain_at_one_norm_force, double,
            "Fiber strain when the passive fiber force is 1 normalized force. "
            "Default: 0.6.");

public:
    ModOpPassiveFiberStrainAtOneNormForceDGF() {
        constructProperty_passive_fiber_strain_at_one_norm_force(0.6);
    }
    ModOpPassiveFiberStrainAtOneNormForceDGF(double value) :
            ModOpPassiveFiberStrainAtOneNormForceDGF() {
        set_passive_fiber_strain_at_one_norm_force(value);
    }

    void operate(Model& model, const std::string&) const override {
        model.finalizeFromProperties();
        for (auto& muscle :
                model.updComponentList<DeGrooteFregly2016Muscle>()) {
            muscle.set_passive_fiber_strain_at_one_norm_force(
                    get_passive_fiber_strain_at_one_norm_force());
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

/// Set the fiber damping for all DeGrooteFregly2016Muscle%s in the model.
class OSIMMOCO_API ModOpFiberDampingDGF : public ModelOperator {
OpenSim_DECLARE_CONCRETE_OBJECT(ModOpFiberDampingDGF, ModelOperator);
    OpenSim_DECLARE_PROPERTY(fiber_damping, double,
            "The linear damping of the fiber (default: 0.0).")
public:
    ModOpFiberDampingDGF() {
        constructProperty_fiber_damping(0);
    }
    ModOpFiberDampingDGF(double fiberDamping) : ModOpFiberDampingDGF() {
        set_fiber_damping(fiberDamping);
    }
    void operate(Model& model, const std::string&) const override {
        model.finalizeFromProperties();
        for (auto& muscle :
                model.updComponentList<DeGrooteFregly2016Muscle>()) {
            muscle.set_fiber_damping(get_fiber_damping());
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
            muscle.set_max_isometric_force(
                    get_scale_factor() * muscle.get_max_isometric_force());
        }
    }
};

/// Remove all muscles contained in the model's ForceSet.
class OSIMMOCO_API ModOpRemoveMuscles : public ModelOperator {
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

/// Add reserve actuators to the model using
/// ModelFactory::createReserveActuators.
class OSIMMOCO_API ModOpAddReserves : public ModelOperator {
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

} // namespace OpenSim

#endif // OPENSIM_MODELOPERATORS_H
