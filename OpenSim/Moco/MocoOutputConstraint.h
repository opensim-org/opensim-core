#ifndef OPENSIM_MOCOOUTPUTCONSTRAINT_H
#define OPENSIM_MOCOOUTPUTCONSTRAINT_H
/* -------------------------------------------------------------------------- *
 * OpenSim: MocoOutputConstraint.h                                            *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2020 Stanford University and the Authors                     *
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
#include "osimMocoDLL.h"

namespace OpenSim {

/**

TODO

@note Only scalar (double) Outputs are currently supported.

@note This class represents a path constraint, *not* a model kinematic
constraint. Therefore, there are no Lagrange multipliers or constraint
forces associated with this constraint. The model's force elements
(including actuators) must generate the forces necessary for satisfying this
constraint.

@ingroup mocopathcon */
class OSIMMOCO_API MocoOutputConstraint : public MocoPathConstraint {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoOutputConstraint, MocoPathConstraint);

public:
    OpenSim_DECLARE_LIST_PROPERTY(output_paths, std::string,
            "The absolute path to the output in the model to use as the "
            "integrand for this goal.");
    OpenSim_DECLARE_LIST_PROPERTY(lower_bounds, double, "TODO");
    OpenSim_DECLARE_LIST_PROPERTY(upper_bounds, double, "TODO");

    MocoOutputConstraint();
    void addOutput(std::string outputPath, double lowerBound, 
        double upperBound);

protected:
    void initializeOnModelImpl(
            const Model& model, const MocoProblemInfo&) const override;
    void calcPathConstraintErrorsImpl(
            const SimTK::State& state, SimTK::Vector& errors) const override;

private:
    void constructProperties();

    mutable std::vector<SimTK::ReferencePtr<const AbstractOutput>> m_outputs;
};


} // namespace OpenSim

#endif // OPENSIM_MOCOOUTPUTCONSTRAINT_H
