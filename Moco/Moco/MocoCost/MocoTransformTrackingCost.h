#ifndef MOCO_MOCOTRANSFORMTRACKINGCOST_H
#define MOCO_MOCOTRANSFORMTRACKINGCOST_H
/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoTransformTrackingCost.h                                  *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2019 Stanford University and the Authors                     *
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

#include "MocoCost.h"

namespace OpenSim {

class OSIMMOCO_API MocoTransformTrackingCost : public MocoCost {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoTransformTrackingCost, MocoCost);
public:
    MocoTransformTrackingCost() {
        constructProperties();
    }
    MocoTransformTrackingCost(std::string name) : MocoCost(std::move(name)) {
        constructProperties();
    }
    MocoTransformTrackingCost(std::string name, double weight)
            : MocoCost(std::move(name), weight) {
        constructProperties();
    }

protected:

private:
    OpenSim_DECLARE_PROPERTY(reference_type, std::string, 
            "'states' or 'transform'")

    OpenSim_DECLARE_PROPERTY(reference_file, std::string,
            "Path to file (.sto, .csv, ...) containing values of controls "
            "(joint moments, excitations, etc.) to track. Column labels "
            "should be control variable paths, e.g., '/forceset/soleus_r'");

    OpenSim_DECLARE_PROPERTY(allow_unused_references, bool,
            "Flag to determine whether or not references contained in the "
            "reference_file are allowed to be ignored by the cost.");

    // TODO update description
    //OpenSim_DECLARE_PROPERTY(transform_weights, MocoWeightSet,
    //        "Set of weight objects to weight the tracking of individual "
    //        "control variables in the cost.");

    void constructProperties() {
        constructProperty_reference_type("states");
        constructProperty_reference_file("");
        constructProperty_allow_unused_references(false);
        //constructProperty_transform_weights(MocoWeightSet());
    }

    TimeSeriesTable m_table;
    mutable GCVSplineSet m_refsplines;
    //mutable std::vector<double> m_transform_weights;
};

} // namespace OpenSim

#endif // MOCO_MOCOTRANSFORMTRACKINGCOST_H
