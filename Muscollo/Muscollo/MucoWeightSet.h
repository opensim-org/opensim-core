#ifndef MUSCOLLO_MUCOWEIGHTSET_H
#define MUSCOLLO_MUCOWEIGHTSET_H
/* -------------------------------------------------------------------------- *
 * OpenSim Muscollo: MucoWeightSet.h                                          *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017 Stanford University and the Authors                     *
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

#include <OpenSim/Common/Set.h>

#include "osimMuscolloDLL.h"

namespace OpenSim {

/// This class contains a single property that holds a weighting factor to be
/// used in a MucoCost. The meaning of the name given to this object depends on
/// where the weight is used. In a MucoStateTrackingCost, the name is the name
/// (path) of a state variable.
class OSIMMUSCOLLO_API MucoWeight : public Object {
OpenSim_DECLARE_CONCRETE_OBJECT(MucoWeight, Object);
private:
    OpenSim_DECLARE_PROPERTY(weight, double, "Weight (default: 1)");

public:
    MucoWeight() : Object() { constructProperties(); }

    MucoWeight(const std::string& name, double weight) : MucoWeight() {
        setName(name);
        setWeight(weight);
    }

    void setWeight(double weight) { set_weight(weight); }
    double getWeight() const {return get_weight(); }

private:
    void constructProperties() {
        constructProperty_weight(1.0);
    }
};

/// A container for %Muscollo weights. The meaning of the weights depends upon
/// where they are used. This container can be written to and read from an
/// XML file.
class OSIMMUSCOLLO_API MucoWeightSet : public Set<MucoWeight> {
    OpenSim_DECLARE_CONCRETE_OBJECT(MucoWeightSet, Set<MucoWeight>);
public:
    MucoWeightSet() = default;
    MucoWeightSet(const std::string& filename) : Set<MucoWeight>(filename) {}
};

} // namespace OpenSim

#endif // MUSCOLLO_MUCOWEIGHTSET_H
