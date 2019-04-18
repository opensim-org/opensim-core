#ifndef OPENSENSE_UTILITIES_H_
#define OPENSENSE_UTILITIES_H_
/* -------------------------------------------------------------------------- *
 *                           OpenSenseUtilities.h                             *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2019 Stanford University and the Authors                *
 * Author(s): OpenSim Team                                                    *
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

#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Common/TimeSeriestable.h>
#include <SimTKCommon/internal/Rotation.h>

namespace OpenSense {

    typedef OpenSim::TimeSeriesTable_<SimTK::Quaternion> TimeSeriesTableQuaternion;
    typedef OpenSim::TimeSeriesTable_<SimTK::Rotation> TimeSeriesTableRotation;

/// @name Convert Table of Quaternions into a Table for Rotations
/// @{
/** Load a TimeSeriesTable of Rotation matrices from a Storage file containing
    quaternions as data elements. Optionally provide a range of times for data
    to be averaged. By default just uses the first time frame.*/
    TimeSeriesTableRotation  __declspec(dllexport) convertQuaternionsToRotations(
        const TimeSeriesTableQuaternion& qauternionsTable, 
        std::tuple<size_t, size_t> startEnd = std::tuple<size_t, size_t>{ 0, 1 }
    );
/// @}

} // end of namespace OpenSense

#endif // OPENSENSE_UTILITIES_H_
