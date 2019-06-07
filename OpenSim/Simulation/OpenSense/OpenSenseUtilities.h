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
#include <SimTKsimbody.h>

namespace OpenSim {

    class OSIMSIMULATION_API OpenSenseUtilities {
    public:
        typedef OpenSim::TimeSeriesTable_<SimTK::Quaternion> TimeSeriesTableQuaternion;
        typedef OpenSim::TimeSeriesTable_<SimTK::Rotation> TimeSeriesTableRotation;

        /// @name Convert Table of Quaternions into a Table for Rotations
        /// @{
        /** Load a TimeSeriesTable of Rotation matrices from a Storage file containing
            quaternions as data elements. Optionally provide a range of times for data
            to be averaged. By default just uses the first time frame.
            Additional options include the name of the base IMU and its axis that
            represents the heading (forward) direction. These are used to perform
            a heading correction on all the experimental (quaternion) data so that
            when tracking rotation data, the initial pose of the model is facing
            forward. If the baseImuName is empty, no correction is made. If no
            axis is specified, the default is the Z axis.*/
        static  OpenSim::TimeSeriesTable_<SimTK::Rotation_<double>>  convertQuaternionsToRotations(
            const OpenSim::TimeSeriesTable_<SimTK::Quaternion_<double>>& qauternionsTable,
            const SimTK::Array_<int>& startEnd = { 0, 1 },
            const std::string& baseImuName = "",
            const SimTK::CoordinateAxis& baseHeadingAxis = SimTK::ZAxis
        );
        /// @}
        /** Create a calibrated model from a model in the calibration pose and 
            the sensor (IMU) orientations (as quaternions) during the calibration.
            modelCalibrationPoseFile: a model file where default pose matches that
                                      used to collect the calibration data.
            calibrationOrientationsFile: a storage file with IMU orientations 
                                         expressed as Quaternions.
            baseImuName: The label of the base IMU in the orientations_file used
                         to account for the heading difference between the sensor
                         data and the forward direction of the model. Leave blank
                         if no heading correction is to be applied.
            baseHeadingAxis: The axis of the base IMU that corresponds to its
                             heading direction. Options are SimTK::X/Y/ZAxis
            Assumptions about the inputs: 
             1) the model default pose is the same as the pose used to collect
                calibration data
             2) the sensors are labeled <body(or physical frame)_in_model>_imu in
                the orientations data. The underlying PhysicalFrame in the model
                is then identified and a corresponding Offset frame is attached, 
                which represents the sensor affixed to the model.
         */
        static Model calibrateModelFromOrientations(
            const std::string& modelCalibrationPoseFile,
            const std::string& calibrationOrientationsFile,
            const std::string& baseImuName = "",
            const SimTK::CoordinateAxis& baseHeadingAxis = SimTK::ZAxis,
            bool visualizeCalibratedModel =true);
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
