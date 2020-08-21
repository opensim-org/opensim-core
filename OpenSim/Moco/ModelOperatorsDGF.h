#ifndef OPENSIM_MODELOPERATORS_DGF_H
#define OPENSIM_MODELOPERATORS_DGF_H
/* -------------------------------------------------------------------------- *
 * OpenSim: ModelOperatorsDGF.h                                               *
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

#include <OpenSim/Actuators/ModelProcessor.h>

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

/** For DeGrooteFregly2016Muscle muscles whose 'ignore_tendon_compliance'
property is false, set the tendon compliance dynamics mode to either
'explicit' or 'implicit'. */
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

/** Set the tendon compliance dynamics mode to "implicit" for all
DeGrooteFregly2016Muscle%s in the model. */
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

/** Turn off passive fiber forces for all DeGrooteFregly2016Muscle%s in the
model. */
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

/** Set passive fiber stiffness for all DeGrooteFregly2016Muscle%s in the
model. */
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

/** Scale the active fiber force curve width for all DeGrooteFregly2016Muscle%s
in the model. */
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

/** Set the fiber damping for all DeGrooteFregly2016Muscle%s in the model. */
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

} // namespace OpenSim

#endif // OPENSIM_MODELOPERATORS_H
