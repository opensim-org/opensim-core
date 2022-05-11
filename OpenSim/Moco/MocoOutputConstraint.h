#ifndef OPENSIM_MOCOOUTPUTCONSTRAINT_H
#define OPENSIM_MOCOOUTPUTCONSTRAINT_H
/* -------------------------------------------------------------------------- *
 * OpenSim: MocoOutputConstraint.h                                            *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2022 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Nicholas Bianco                                                 *
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

#include "MocoConstraint.h"

namespace OpenSim {

class OSIMMOCO_API MocoOutputConstraint : public MocoPathConstraint {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoOutputConstraint, MocoPathConstraint);

public:

    MocoOutputConstraint() { constructProperties(); }

    /** Set the absolute path to the output in the model to be used in this path
    constraint. The format is "/path/to/component|output_name". */
    void setOutputPath(std::string path) { set_output_path(std::move(path)); }
    const std::string& getOutputPath() const { return get_output_path(); }

    /** Set the exponent applied to the output value in the constraint. This
    exponent is applied when minimizing the norm of a vector type output. */
    void setExponent(int exponent) { set_exponent(exponent); }
    int getExponent() const { return get_exponent(); }

    /** Set the index to the value to be constrained when a vector type Output is
    specified. For SpatialVec Outputs, indices 0, 1, and 2 refer to the
    rotational components and indices 3, 4, and 5 refer to the translational
    components. A value of -1 indicates to constrain the vector norm (which is
    the default setting). If an index for a type double Output is provided, an
    exception is thrown. */
    void setOutputIndex(int index) { set_output_index(index); }
    int getOutputIndex() const { return get_output_index(); }

protected:
    void initializeOnModelImpl(
            const Model&, const MocoProblemInfo&) const override;
    void calcPathConstraintErrorsImpl(
            const SimTK::State& state, SimTK::Vector& errors) const override;
    void printDescriptionImpl() const override;

    /** Calculate the Output value for the provided SimTK::State. If using a
    vector Output, either the vector norm or vector element will be returned,
    depending on whether an index was provided via 'setOutputIndex()'. Do not
    call this function until 'initializeOnModelBase()' has been called. */
    double calcOutputValue(const SimTK::State&) const;

    /** Raise a value to the exponent set via 'setExponent()'. Do not call this
    function until 'initializeOnModelBase()' has been called. */
    double setValueToExponent(double value) const {
        return m_power_function(value);
    }

    /** Get the "depends-on stage", or the SimTK::Stage we need to realize the
    system to in order to calculate the Output value. */
    const SimTK::Stage& getDependsOnStage() const {
        return m_dependsOnStage;
    }

private:
    OpenSim_DECLARE_PROPERTY(output_path, std::string,
            "The absolute path to the output in the model to be used in this "
            "path constraint.");
    OpenSim_DECLARE_PROPERTY(exponent, int,
            "The exponent applied to the output value in the constraint "
            "(default: 1).");
    OpenSim_DECLARE_PROPERTY(output_index, int,
            "The index to the value to be constrained when a vector type "
            "Output is specified. For SpatialVec Outputs, indices 0, 1, "
            "and 2 refer to the rotational components and indices 3, 4, "
            "and 5 refer to the translational components. A value of -1 "
            "indicates to constrain the vector norm (default: -1).");
    void constructProperties();

    enum DataType {
        Type_double,
        Type_Vec3,
        Type_SpatialVec,
    };
    mutable DataType m_data_type;
    mutable SimTK::ReferencePtr<const AbstractOutput> m_output;
    mutable std::function<double(const double&)> m_power_function;
    mutable int m_index1;
    mutable int m_index2;
    mutable bool m_minimizeVectorNorm;
    mutable SimTK::Stage m_dependsOnStage = SimTK::Stage::Acceleration;
};

} // namespace OpenSim

#endif //OPENSIM_MOCOOUTPUTCONSTRAINT_H
