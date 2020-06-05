#ifndef MOCO_MOCOOUTPUTGOAL_H
#define MOCO_MOCOOUTPUTGOAL_H
/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoOutputGoal.h                                             *
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

#include "MocoGoal.h"

namespace OpenSim {

/// This goal allows you to use any (double, or scalar) Output in the model
/// as the integrand of a goal.
/// @ingroup mocogoal
class OSIMMOCO_API MocoOutputGoal : public MocoGoal {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoOutputGoal, MocoGoal);

public:
    MocoOutputGoal() { constructProperties(); }
    MocoOutputGoal(std::string name) : MocoGoal(std::move(name)) {
        constructProperties();
    }
    MocoOutputGoal(std::string name, double weight)
            : MocoGoal(std::move(name), weight) {
        constructProperties();
    }

    /// Set the absolute path to the output in the model to use as the integrand
    /// for this goal. The format is "/path/to/component|output_name".
    void setOutputPath(std::string path) { set_output_path(std::move(path)); }
    const std::string& getOutputPath() const { return get_output_path(); }

    /// Set if the goal should be divided by the displacement of the system's
    /// center of mass over the phase.
    void setDivideByDisplacement(bool tf) { set_divide_by_displacement(tf); }
    bool getDivideByDisplacement() const {
        return get_divide_by_displacement();
    }

    /// Set if the goal should be divided by the total mass of the model.
    void setDivideByMass(bool tf) { set_divide_by_mass(tf); }
    bool getDivideByMass() const {
        return get_divide_by_mass();
    }

protected:
    void initializeOnModelImpl(const Model&) const override;
    void calcIntegrandImpl(
            const IntegrandInput& state, double& integrand) const override;
    void calcGoalImpl(
            const GoalInput& input, SimTK::Vector& cost) const override;

private:
    OpenSim_DECLARE_PROPERTY(output_path, std::string,
            "The absolute path to the output in the model to use as the "
            "integrand for this goal.");
    OpenSim_DECLARE_PROPERTY(divide_by_displacement, bool,
            "Divide by the model's displacement over the phase (default: "
            "false)");
    OpenSim_DECLARE_PROPERTY(divide_by_mass, bool,
            "Divide by the model's total mass (default: false)");
    void constructProperties();

    mutable SimTK::ReferencePtr<const Output<double>> m_output;
};

} // namespace OpenSim

#endif // MOCO_MOCOOUTPUTGOAL_H
