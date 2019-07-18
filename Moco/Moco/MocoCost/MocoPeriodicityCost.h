#ifndef MOCO_MOCOPERIODICITYCOST_H
#define MOCO_MOCOPERIODICITYCOST_H
/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoPeriodicityCost.h                                        *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017-19 Stanford University and the Authors                  *
 *                                                                            *
 * Author(s): Antoine Falisse                                                 *
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

#include "../MocoWeightSet.h"
#include "MocoCost.h"

namespace OpenSim {

class OSIMMOCO_API MocoPeriodicityCostPair : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoPeriodicityCostPair, Object);

public:
    OpenSim_DECLARE_PROPERTY(
            first, std::string, "First element of the pair.");
    OpenSim_DECLARE_PROPERTY(
            second, std::string, "Second element of the pair.");

    MocoPeriodicityCostPair();
    MocoPeriodicityCostPair(std::string name);

private:
    void constructProperties();

};

// TODO
/// Minimize the sum of squared controls, integrated over the phase.
/// The default weight for each control is 1.0; this can be changed by
/// calling setWeight() or editing the `control_weights` property in XML.
/// @ingroup mococost
class OSIMMOCO_API MocoPeriodicityCost : public MocoCost {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoPeriodicityCost, MocoCost);

public:
    OpenSim_DECLARE_LIST_PROPERTY(
            state_pairs, MocoPeriodicityCostPair, "Periodic pair.");
    OpenSim_DECLARE_LIST_PROPERTY(
            control_pairs, MocoPeriodicityCostPair, "Periodic pair.");

    MocoPeriodicityCost();
    MocoPeriodicityCost(std::string name) : MocoCost(std::move(name)) {
        constructProperties();
    }

protected:

    bool getSupportsEndpointConstraintImpl() const override { return true; }
    bool getDefaultEndpointConstraintImpl() const override { return true; }

    int getNumOutputsImpl() const override;
    int getNumIntegralsImpl() const override { return 0; }

    void initializeOnModelImpl(const Model& model) const override;

    void calcCostImpl(
            const CostInput& input, SimTK::Vector& cost) const override;

private:
    void constructProperties();
    mutable std::vector<std::pair<int, int>> m_indices_states;
    mutable std::vector<std::pair<int, int>> m_indices_controls;
};

} // namespace OpenSim

#endif // MOCO_MOCOPERIODICITYCOST_H
