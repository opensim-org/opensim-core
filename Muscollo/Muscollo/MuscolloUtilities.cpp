/* -------------------------------------------------------------------------- *
 * OpenSim Muscollo: MuscolloUtilities.cpp                                               *
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

#include "MuscolloUtilities.h"

#include <OpenSim/Common/TimeSeriesTable.h>

using namespace OpenSim;

Storage OpenSim::convertTableToStorage(const TimeSeriesTable& table) {
    Storage sto;
    OpenSim::Array<std::string> labels("", (int)table.getNumColumns() + 1);
    labels[0] = "time";
    for (int i = 0; i < (int)table.getNumColumns(); ++i) {
        labels[i + 1] = table.getColumnLabel(i);
    }
    sto.setColumnLabels(labels);
    const auto& times = table.getIndependentColumn();
    for (unsigned i_time = 0; i_time < table.getNumRows(); ++i_time) {
        auto rowView = table.getRowAtIndex(i_time);
        sto.append(times[i_time], SimTK::Vector(rowView.transpose()));
    }
    return sto;
}

void OpenSim::visualize(const StatesTrajectory& st) {
    std::cout << "DEBUG VISUALIZE " << std::endl;

}
