/* -------------------------------------------------------------------------- *
 *                            OpenSim:  TimeSeriesTable.h                     *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2015 Stanford University and the Authors                *
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

/** \file
This file defines the TimeSeriesTable_ class, which is used by OpenSim to 
provide an in-memory container for data access and manipulation.              */

#ifndef OPENSIM_COMMON_TIMESERIESDATATABLE_H
#define OPENSIM_COMMON_TIMESERIESDATATABLE_H

#include "OpenSim/Common/DataTable.h"


namespace OpenSim {

/** TimeSeriesTable_ is a DataTable_ that adds support for a time column. 
The time column can be of any arithmetic type -- float, double, int, long etc. 
In this documentaion, words time & timestamp are used interchangeably to mean
an entry of the time column.

The time column is enforced to be strictly increasing. Entries in the 
time-series column can be used to access the rows of the DataTable.           

\tparam ET Type of the entries in the underlying matrix. Defaults to
           SimTK::Real (alias for double).
\tparam TS Type of the time column.                                           */
template<typename ETY = SimTK::Real>
class TimeSeriesTable_ : public DataTable_<double, ETY> {
protected:
    void validateAppendRow() const override {
        
    }
};

/** See TimeSeriesTable_ for details on the interface.                        */
using TimeSeriesTable = TimeSeriesTable_<SimTK::Real>;

} // namespace OpenSim

#endif // OPENSIM_COMMON_TIMESERIESDATATABLE_H
