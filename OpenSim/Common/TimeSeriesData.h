#ifndef OPENSIM_TIME_SERIES_DATA_H_
#define OPENSIM_TIME_SERIES_DATA_H_
/* ------------------------------------------------------------------------- *
*                         OpenSim:  TimeSeriesData.h                         *
* -------------------------------------------------------------------------- *
* The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
* See http://opensim.stanford.edu and the NOTICE file for more information.  *
* OpenSim is developed at Stanford University and supported by the US        *
* National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
* through the Warrior Web program.                                           *
*                                                                            *
* Copyright (c) 2005-2015 Stanford University and the Authors                *
* Author(s): Ajay Seth                                                       *
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

/** @file
* This file defines the concrete TimeSeriesData class.  It is used by
* Data components to provide a consistent (DataTable) interface for time
* series data in memory for other OpenSim components to consume.
*/

#include "DataTable.h"

namespace OpenSim {
//=============================================================================
//=============================================================================
/**
* TimeSeriesData_ is a DataTable where the rows in the table are guaranteed
* to be sequentially increasing in time. Intended to store trajectories of
* data (states, controls, points, etc...) as a function of time. Therefore,
* in addition to rows being accessible by index, they are also accessible
* by time.
* The columns of the DataTable are considered to be dependent variables and
* time is considered the independent variable. Time, therefore, cannot be 
* accessed as a column of the table. Time is accessed explicitly. 
* 
* @author Ajay Seth
*/
template<typename T = SimTK::Real>
class TimeSeriesData_ : public DataTable_<T> {
public:
    TimeSeriesData_() = default;

    TimeSeriesData_(const TimeSeriesData_&) = default;

    TimeSeriesData_<T>& operator = (const TimeSeriesData_&) = default;

    /** Copy construct TimeSeriesData from a generic DataTable */
    TimeSeriesData_(const DataTable_& dt, const std::string& timeLabel = "time") : 
                DataTable_(dt) {
        const TimeSeriesData_* tsd =
            dynamic_cast<const TimeSeriesData_<T>*>(&dt);
        if (tsd) {
            // source DataTable is already TimeSeriesData so make this a copy.
            *this = *tsd;
        }
        else if (hasColumn(timeLabel)){
            size_t c = dt.getColumnIndex(timeLabel);
            if (!isStrictlyIncreasing(dt.getColumn(c)))
                throw Exception("TimeSeriesData: cannot construct "
                "from a DataTable without a sequential time column.");
            _times = dt.getColumn(int(c));
            updColumnLabels().remove(int(c));
            int nr = int(dt.getNumRows());
            int nc = int(dt.getNumCols() - 1);
            updAsMatrix().resize(nr, nc) = dt.getAsMatrix().block(0, 1, nr, nc);
        }
        else {
            throw Exception("TimeSeriesData: cannot construct "
                "from a DataTable without a time column.");
        }
    }

    virtual ~TimeSeriesData_() {}

    /** Time to index options */
    enum TimeToIndexOption
    {
        Before,
        After,
        Nearest
    };

    /** Access the time data as a Vector of Reals */
    const SimTK::Vector& getTimes() const { return _times; }

    /** Get the time range that the data spans. */
    SimTK::Vec2 getTimeRange() const { 
        return SimTK::Vec2(_times[0], _times[_times.size()-1]);
    }

    /** Update the time data */
    SimTK::Vector& updTimes() const { return _times; }
    
    /** Determine the closest row index for a given time */
    size_t findRowIndexForTime(const SimTK::Real& time,
                                TimeToIndexOption opt,
                                           size_t startIndex) const {
        size_t nt = _times.getSize();
        SimTK_ASSERT_ALWAYS(startIndex < nt, 
            "TimeSeriesData_::findRowIndexForTime()"
            " supplied startIndex exceeds size of time data.");

        // If time supplied exceeds the limits by more that SimTK::SqrtEps, it is 
        // an error, not round-off!
        SimTK_ASSERT_ALWAYS(time > _times[0] - SimTK::SqrtEps,
            "TimeSeriesData_::findRowIndexForTime() "
            "supplied time exceeds first time in series.");

        SimTK_ASSERT_ALWAYS(time < _times[nt - 1] + SimTK::SqrtEps,
            "TimeSeriesData_::findRowIndexForTime() "
            "supplied time exceeds maximum time in series.");

        size_t foundIndex = -1;

        for (size_t i = startIndex; i < nt; ++i) {
            if (_times[i] >= time){
                foundIndex = i;
                break;
            }
        }

        switch (opt) {
        case After:
            return foundIndex;
        case:Before :
            return foundIndex - 1;
        case Nearest: {
            SimTK::Real dBefore = time - _times[foundIndex - 1];
            SimTK::Real dAfter = _times[foundIndex] - time;
            return (dBefore < dAfter) ? (foundIndex - 1) : foundIndex;
        }
        default:
            throw Exception("TimeSeriesData_::findRowIndexForTime() "
                "Illegal TimeToIndexOption provided.");
        }
    }

    void dumpToStream(std::ostream &out) const override {
        out << getTimeLabel() << getColumnLabels() << std::endl;
        for (int i = 0; i < getNumRows(); ++i){
            out << _times(i) << " " << getRow(i) << std::endl;
        }
    }

    std::string getTimeLabel() const { return "time"; }

    SimTK_DOWNCAST(TimeSeriesData_, DataTable_);

protected:
    /** Helper method to te check if a particular series (e.g. a column of 
       a table is strictlyIncreasing, which for time, for example, must be
       the case. */
    bool isStrictlyIncreasing(const SimTK::Vector& series) const {
        size_t i = 1;
        size_t nt = series.size();
        while (series[int(i)] > series[int(i - 1)]) {
            if (++i == nt)
                return true;
        }
        return false;
    }
private:
    // regardless of the DataType contained in the DatTable, the time is always
    // an ordered (monotonically increasing) Vector of SimTK::Reals.
    SimTK::Vector_<SimTK::Real> _times;
};

typedef TimeSeriesData_<SimTK::Real> TimeSeriesData;

} //namespace OpenSim

#endif // OPENSIM_TIME_SERIES_DATA_H_