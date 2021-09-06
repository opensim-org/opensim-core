#ifndef OPENSIM_MOCOCONTROLTRACKINGGOAL_H
#define OPENSIM_MOCOCONTROLTRACKINGGOAL_H
/* -------------------------------------------------------------------------- *
 * OpenSim: MocoControlTrackingGoal.h                                         *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2020 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Nicholas Bianco                                                 *
 * Contributor(s): Christopher Dembia                                         *
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

#include "MocoGoal.h"

#include <OpenSim/Common/GCVSplineSet.h>
#include <OpenSim/Common/TimeSeriesTable.h>
#include <OpenSim/Moco/MocoWeightSet.h>
#include <OpenSim/Simulation/TableProcessor.h>
#include <OpenSim/Moco/MocoScaleFactor.h>

namespace OpenSim {

/** Associate a control variable with a column from the reference data. */
class OSIMMOCO_API MocoControlTrackingGoalReference : public Object {
OpenSim_DECLARE_CONCRETE_OBJECT(MocoControlTrackingGoalReference, Object);

public:
    OpenSim_DECLARE_PROPERTY(
            reference, std::string, "Column label from reference table.");

    MocoControlTrackingGoalReference();
    /** Provide the name of a control and the label of the column from the
    reference that this control should track. */
    MocoControlTrackingGoalReference(std::string name, std::string reference);

private:
    void constructProperties();
};

/** 
\section MocoControlTrackingGoal
The squared difference between a control
variable value and a reference control variable value, summed over the control variables for which a
reference is provided, and integrated over the phase. This can be used to
track actuator controls, muscle excitations, etc.

This goal is computed as follows:

\f[
\int_{t_i}^{t_f}
        \sum_{c \in C} w_c \|x_{m,c}(t) - x_{e,c})\|^2 ~dt
\f]
We use the following notation:
- \f$ t_i \f$: the initial time of this phase.
- \f$ t_f \f$: the final time of this phase.
- \f$ C \f$: the set of control variables being tracked.
- \f$ w_c \f$: the weight for control \f$ c \f$.
- \f$ x_{m,c}(t) \f$: control signal \f$ c \f$.
- \f$ x_{e,c}(t) \f$: reference data for control signal \f$ c \f$.

This goal has two labeling modes: 'auto' and 'manual':
- 'auto': The column labels of the reference must exactly match the names
          of controls, and all controls with a matching column in the
          reference data are tracked. By default, all column labels for the
          reference data must match the name of a control.
          Setting `allow_unused_references` to false allows
          the reference to contain columns whose labels do not match a
          control; such columns are then ignored.
- 'manual': The association between controls and columns of the reference
            data is manually specified via the `reference_labels` property.
            Only the controls for which a reference label is specified are
            tracked.
            Enter this mode by providing reference labels through
            the `reference_labels` property or the `setReferenceLabel()`
            function.
            The `allow_unused_references` property does not apply in this
            mode.

## Control variable names

Control variable names are based on paths to actuators,
e.g., `/forceset/soleus_r`. For non-scalar actuators, the control variable
name includes the index for the actuator control;
e.g., `/forceset/body_actuator_0`, where
'body_actuator' is the name of the actuator and `_0` specifies the
control index.

## Reference data

The reference can be provided as a file name to a STO or CSV file (or
other file types for which there is a FileAdapter), or programmatically
as a TimeSeriesTable.

## Scale factors

Use `addScaleFactor()` to add a MocoParameter to the MocoProblem that will
scale the tracking reference data associated with a control in the tracking cost.
Scale factors for this goal can be useful if the magnitude of the tracking
reference data is either unknown or unreliable (e.g., electromyography data).
Scale factors are applied to the tracking error calculations based on the
following equation:

    error = modelValue - scaleFactor * referenceValue

In other words, scale factors are applied when computing the tracking error for
each control, not to the reference data directly. Therefore, if a column in the
reference data is tracked by two different controls, the scale factor will only
scale the column for the associated control. The tracking error for the other
control is unaffected.

Adding a scale factor to a MocoControlTrackingGoal.
@code
auto* controlTrackingGoal = problem.addGoal<MocoControlTrackingGoal>();
...
controlTrackingGoal->addScaleFactor(
        'soleus_scale_factor', '/forceset/soleus_r', {0.01, 1.0});
@endcode

## Helpful tips

Tracking problems in direct collocation perform best when tracking smooth
data, so it is recommended to filter the data in the reference you provide
to the cost.
@ingroup mocogoal */
class OSIMMOCO_API MocoControlTrackingGoal : public MocoGoal {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoControlTrackingGoal, MocoGoal);

public:
    MocoControlTrackingGoal() { constructProperties(); };
    MocoControlTrackingGoal(std::string name) : MocoGoal(std::move(name)) {
        constructProperties();
    }
    MocoControlTrackingGoal(std::string name, double weight)
            : MocoGoal(std::move(name), weight) {
        constructProperties();
    }

    /// Provide a table containing reference values for the
    /// controls you want to track.
    /// In 'auto' labeling mode, each column label must be a control variable
    /// name. In 'manual' labeling mode, the column labels need not be control
    /// variable names; use setReferenceLabel() to associate controls with
    /// columns.
    /// The table is not loaded until the MocoProblem is initialized.
    void setReference(const TableProcessor& ref) {
        set_reference(std::move(ref));
    }
    /// Set the weight for an individual control variable. If a weight is
    /// already set for the requested control, then the provided weight
    /// replaces the previous weight.
    /// If no weight is specified for a control, a weight of 1.0 is used
    /// internally.
    /// Set the weight to 0 to avoid tracking a given control.
    /// An exception is thrown if a weight for an unknown control is provided.
    void setWeightForControl(
            const std::string& controlName, const double& weight) {
        if (get_control_weights().contains(controlName)) {
            upd_control_weights().get(controlName).setWeight(weight);
        } else {
            upd_control_weights().cloneAndAppend({controlName, weight});
        }
    }
    /// Provide a MocoWeightSet to weight the control variables in the cost.
    /// Replaces the weight set if it already exists.
    void setWeightSet(const MocoWeightSet& weightSet) {
        upd_control_weights() = weightSet;
    }

    /// Set the column of the reference data that a given control should track.
    /// Multiple controls can track the same column of the reference data.
    /// This replaces the reference label for the given control, if one had
    /// already been provided.
    /// If controls are not manually associated with a reference label, then
    /// it is assumed that the column labels for the reference data exactly
    /// match the names of controls.
    void setReferenceLabel(const std::string& control,
            const std::string& label) {
        for (int i = 0; i < getProperty_reference_labels().size(); ++i) {
            if (control == get_reference_labels(i).getName()) {
                upd_reference_labels(i).set_reference(label);
                return;
            }
        }
        append_reference_labels({control, label});
    }
    /// Clear the 'reference_labels' property, which ensures this goal is used
    /// in 'auto' labeling mode.
    void clearReferenceLabels() {
        updProperty_reference_labels().clear();
    }

    /// Specify whether the reference can have columns not associated with
    /// controls.
    /// If set true, then such columns will be ignored by the cost.
    /// If false, such columns will cause an Exception to be raised.
    /// Only takes effect in 'auto' labeling mode.
    void setAllowUnusedReferences(bool tf) { set_allow_unused_references(tf); }

    /// If no reference has been provided, this returns an empty processor.
    const TableProcessor& getReference() const { return get_reference(); }

    bool hasReferenceLabel(const std::string& control) const {
        for (int i = 0; i < getProperty_reference_labels().size(); ++i) {
            if (control == get_reference_labels(i).getName()) {
                return true;
            }
        }
        return false;
    }

    std::string getReferenceLabel(const std::string& control) const {
        for (int i = 0; i < getProperty_reference_labels().size(); ++i) {
            if (control == get_reference_labels(i).getName()) {
                return get_reference_labels(i).get_reference();
            }
        }
        OPENSIM_THROW_FRMOBJ(Exception,
                "No reference label provided for control '{}'.", control);
    }

    bool getAllowUnusedReferences() const {
        return get_allow_unused_references();
    }

    /// Add a MocoParameter to the problem that will scale the tracking reference
    /// data associated with the specified control. Scale factors are applied
    /// to the tracking error calculations based on the following equation:
    ///
    ///     error = modelValue - scaleFactor * referenceValue
    ///
    /// In other words, the scale factor is applied when computing the tracking
    /// error for each control, not to the reference data directly. Therefore, if
    /// a column in the reference data is tracked by two different controls, the
    /// scale factor will only scale the column for the associated control. The
    /// tracking error for the other control is unaffected.
    void addScaleFactor(const std::string& name, const std::string& control,
            const MocoBounds& bounds);

protected:
    // TODO check that the reference covers the entire possible time range.
    void initializeOnModelImpl(const Model& model) const override;
    void calcIntegrandImpl(
            const IntegrandInput& input, SimTK::Real& integrand) const override;
    void calcGoalImpl(
            const GoalInput& input, SimTK::Vector& cost) const override {
        cost[0] = input.integral;
    }
    void printDescriptionImpl() const override;

private:
    OpenSim_DECLARE_PROPERTY(reference, TableProcessor,
            "Trajectories of controls (joint moments, excitations, etc.) "
            "to track.");

    OpenSim_DECLARE_PROPERTY(allow_unused_references, bool,
            "Permit the reference_file to contain columns that do not refer to "
            "controls? Only relevant if reference_labels is empty "
            "('auto' labeling mode). Default: false.");

    OpenSim_DECLARE_PROPERTY(control_weights, MocoWeightSet,
            "Weights for individual control terms.");

    OpenSim_DECLARE_LIST_PROPERTY(reference_labels,
            MocoControlTrackingGoalReference,
            "Association between controls and columns in the reference data.");

    void constructProperties() {
        constructProperty_reference(TableProcessor());
        constructProperty_allow_unused_references(false);
        constructProperty_control_weights(MocoWeightSet());
        constructProperty_reference_labels();
    }

    mutable std::vector<int> m_control_indices;
    mutable std::vector<double> m_control_weights;
    mutable GCVSplineSet m_ref_splines;
    mutable std::vector<int> m_ref_indices;
    mutable std::vector<std::string> m_control_names;
    mutable std::vector<std::string> m_ref_labels;
    mutable std::unordered_map<std::string, std::string> m_scaleFactorMap;
    mutable std::vector<SimTK::ReferencePtr<const MocoScaleFactor>>
    m_scaleFactorRefs;
};

} // namespace OpenSim

#endif // OPENSIM_MOCOCONTROLTRACKINGGOAL_H
