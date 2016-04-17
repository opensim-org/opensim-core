/* -------------------------------------------------------------------------- *
 *                            OpenSim:  TableSource.h                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2015 Stanford University and the Authors                *
 * Authors:                                                                   *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

#ifndef OPENSIM_TABLE_SOURCE_H_
#define OPENSIM_TABLE_SOURCE_H_

#include "OpenSim/Common/TimeSeriesTable.h"
#include "ModelComponent.h"

namespace OpenSim {

template<typename ET>
class TableSource : public ModelComponent {
    OpenSim_DECLARE_CONCRETE_OBJECT_T(TableSource, ET, ModelComponent);

public:
    OpenSim_DECLARE_LIST_OUTPUT(column, ET, getColumnAtTime, 
                                SimTK::Stage::Time);

    ET getColumnAtTime(const SimTK::State& state, 
                       const std::string& columnLabel) const {
        OPENSIM_THROW_IF(_table.getNumRows() == 0, EmptyTable);

        const auto time = state.getTime();
        const auto colInd = _table.getColumnIndex(columnLabel);
        const auto& timeCol = _table.getIndependentColumn();
        auto lb = std::lower_bound(timeCol.begin(), timeCol.end(), time);
        unsigned rowInd{};
        if(*lb == timeCol.begin())
            rowInd = 0;
        if(*lb == timeCol.end())
            rowInd = timeCol.size() - 1;
        else if(*lb == time)
            rowInd = lb - timeCol.begin();
        else {
            if((time - *(lb - 1)) < (*lb - time))
                rowInd = lb - 1 - timeCol.begin();
            else
                rowInd = lb - timeCol.begin();
        }

        return _table.getMatrix().getElt(rowInd, colInd);
    }


private:
    TimeSeriesTable_<ET> _table;

};

} // namespace Opensim

#endif // OPENSIM_TABLE_SOURCE_H_
