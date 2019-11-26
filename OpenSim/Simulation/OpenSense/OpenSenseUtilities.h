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
#include <OpenSim/Common/TimeSeriesTable.h>
#include <OpenSim/Simulation/Model/Model.h>
#include "IMUPlacer.h"

namespace OpenSim {

class OSIMSIMULATION_API OpenSenseUtilities {
public:
     /** Apply the passed in Rotation matrix to a TimeSeriesTable of Quaternions.
        The rotation is done in place so the table passed in is modified
    */
    static void rotateOrientationTable(
            OpenSim::TimeSeriesTable_<SimTK::Quaternion_<double>>&
                    quaternionsTable,
            const SimTK::Rotation_<double>& rotationMatrix);

    /// @name Convert Table of Quaternions into a Table for Rotations
    /// @{
    /** Convert a TimeSeriesTable with quaternions as data elements into a TimeSeriesTable
        of Rotation matrices.
        */
    static  OpenSim::TimeSeriesTable_<SimTK::Rotation_<double>> 
        convertQuaternionsToRotations(
            const OpenSim::TimeSeriesTable_<SimTK::Quaternion_<double>>& qauternionsTable);

    /** Compute a SimTK::Vec3 of Space-fixed XYZ Euler angles that aligns the specified 
        baseIMU and CoordinateDirection combination with the positive X (=forward) direction 
        in OpenSim based on the first frame of the passed in table of quaternions
        quatTimeSeries. 
    */
    static SimTK::Vec3 computeHeadingCorrection(
            OpenSim::Model& model,
            OpenSim::TimeSeriesTable_<SimTK::Quaternion_<double>>&
                    quatTimeSeries, 
            const std::string& baseIMU, 
            const SimTK::CoordinateDirection);
    /// @}
    /**
        * Create Orientations as a TimeSeriesTable based on passed in markerFile
        */
    static TimeSeriesTable_<SimTK::Quaternion>
        createOrientationsFileFromMarkers(const std::string& markersFile);

    /// form a Transform from 3 points origin (op), along x (xp - op), along y(yp - op)
    static SimTK::Transform formTransformFromPoints(const SimTK::Vec3& op, 
        const SimTK::Vec3& xp,  const SimTK::Vec3& yp);
  
}; // end of class OpenSenseUtilities
}
#endif // OPENSENSE_UTILITIES_H_
