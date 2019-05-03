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

#include "MocoTransformTrackingCost.h"
#include "../MocoUtilities.h"
#include <OpenSim/Simulation/StatesTrajectory.h>

using namespace OpenSim;

void MocoTransformTrackingCost::initializeOnModelImpl(const Model& model) const
{
    // Get the reference data.
    if (m_transform_table.getNumColumns() != 0) {
        

    } else {
        TimeSeriesTable tableToUse;
        if (get_reference_file() != "") {
            // Should not be able to supply both.
            assert(m_states_table.getNumColumns() == 0);

            auto tablesFromFile = FileAdapter::readFile(get_reference_file());
            // There should only be one table.
            OPENSIM_THROW_IF_FRMOBJ(tablesFromFile.size() != 1, Exception,
                format("Expected reference file '%s' to contain 1 table, but "
                    "it contains %i tables.",
                    get_reference_file(), tablesFromFile.size()));
            // Get the first table.
            auto* firstTable =
                dynamic_cast<TimeSeriesTable*>(
                    tablesFromFile.begin()->second.get());
            OPENSIM_THROW_IF_FRMOBJ(!firstTable, Exception,
                "Expected reference file to contain a (scalar) "
                "TimeSeriesTable, but it contains a different type of table.");
            tableToUse = *firstTable;
        } else if (m_states_table.getNumColumns() != 0) {
            tableToUse = m_states_table;
        } else {
            OPENSIM_THROW_FRMOBJ(Exception,
                "Expected user to either provide a reference"
                " file or to programmatically provide a reference table, but "
                " the user supplied neither.");
        }

        // Create the StatesTrajectory.
        Storage sto = convertTableToStorage(tableToUse);
        auto statesTraj = StatesTrajectory::createFromStatesStorage(model, sto);
    }

    

}


GCVSplineSet MocoTransformTrackingCost::createReferenceSplines(
        const TimeSeriesTable_<Transform>& table) {
    
    // Create a new scalar-valued TimeSeriesTable using the time index from the
    // transform table argument. We'll populate this table with the transform
    // values we need when calculating the integral tracking cost, namely the
    // transform position vector and a quaternion representation of the
    // transform rotations.

}
