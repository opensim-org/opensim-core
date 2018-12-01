#ifndef MUSCOLLO_MUCOJOINTREACTIONNORMCOST_H
#define MUSCOLLO_MUCOJOINTREACTIONNORMCOST_H
/* -------------------------------------------------------------------------- *
 * OpenSim Muscollo: MucoJointReactionNormCost.h                              *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017 Stanford University and the Authors                     *
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

#include "MucoCost.h"

namespace OpenSim {

/// Minimize the reaction loads on the child body of a specified joint. 
/// The norm of the reaction forces and moments integrated over the phase is 
/// the specific quantity minimized.
/// This cost requires realizing to the Acceleration stage.
/// @ingroup mucocost
// TODO allow a list property of multiple joints?
// TODO allow specification of the components of the reaction load SpatialVec
//      to be minimized.
// TODO allow specification of either child or parent reaction loads to 
//      to minimize.
class OSIMMUSCOLLO_API MucoJointReactionNormCost : public MucoCost {
OpenSim_DECLARE_CONCRETE_OBJECT(MucoJointReactionNormCost, MucoCost);
public: 
    MucoJointReactionNormCost();
    MucoJointReactionNormCost(std::string name) : MucoCost(std::move(name)) {
        constructProperties();
    }
    MucoJointReactionNormCost(std::string name, double weight)
            : MucoCost(std::move(name), weight) {
        constructProperties();
    }
    /// Provide a valid model path for joint whose reaction loads will be
    /// minimized. 
    // TODO when using implicit dynamics, we will need to revisit this cost.
    void setJointPath(const std::string& path) 
    {   set_joint_path(path); }

protected:
    void initializeOnModelImpl(const Model&) const override;
    void calcIntegralCostImpl(const SimTK::State& state,
            double& integrand) const override;

private:
    void constructProperties();
    OpenSim_DECLARE_PROPERTY(joint_path, std::string, "The model path for the "
            "joint with minimized reaction loads.");
};

} // namespace OpenSim

#endif // MUSCOLLO_MUCOJOINTREACTIONNORMCOST_H
