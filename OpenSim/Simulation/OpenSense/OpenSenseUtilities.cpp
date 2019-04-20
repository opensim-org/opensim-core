/* -------------------------------------------------------------------------- *
 *                            OpenSenseUtilities.cpp                          *
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
#include <OpenSim/Common/STOFileAdapter.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/MarkersReference.h>
#include <OpenSim/Simulation/InverseKinematicsSolver.h>
#include "OpenSenseUtilities.h"

using namespace OpenSim;
using namespace SimTK;
using namespace std;

TimeSeriesTable_<SimTK::Rotation> OpenSenseUtilities::
    convertQuaternionsToRotations(
        const TimeSeriesTableQuaternion& qauternionsTable,
        std::tuple<size_t, size_t> startEnd)
{
    // Fixed transform to rotate sensor orientations in world with Z up into the 
    // OpenSim ground reference frame with Y up and X forward.
    SimTK::Rotation R_XG = SimTK::Rotation(
        SimTK::BodyOrSpaceType::BodyRotationSequence,
        -SimTK_PI / 2, SimTK::XAxis,
        0, SimTK::YAxis,
        0, SimTK::ZAxis);

    // OpenSim to the local reference frame of the pelvis (back)
    //SimTK::Rotation R_XG(SimTK::BodyOrSpaceType::BodyRotationSequence,
    //    SimTK::Pi / 2, SimTK::ZAxis,
    //    SimTK::Pi / 2, SimTK::XAxis,
    //    0., SimTK::ZAxis);

    //SimTK::Vec3 grav(9.807, 0., 0.);
    //double theta = acos(avg_accel.transpose() * grav / (avg_accel.norm() * grav.norm()));
    //SimTK::Vec3 C = SimTK::cross(avg_accel, grav);
    //C = C / C.norm();

    //SimTK::Rotation tilt_correction = SimTK::Rotation(axisAngleToRotation(C, theta));
    //R_XG = R_XG * tilt_correction;

    int nc = int(qauternionsTable.getNumColumns());

    const auto& times = qauternionsTable.getIndependentColumn();

    size_t nt = std::get<1>(startEnd) - std::get<0>(startEnd) + 1;

    std::vector<double> newTimes(nt, SimTK::NaN);
    SimTK::Matrix_<SimTK::Rotation> matrix(int(nt), nc, Rotation());

    int cnt = 0;
    for (size_t i = std::get<0>(startEnd); i <= std::get<1>(startEnd); ++i) {
        newTimes[cnt] = times[i];
        const auto& quatRow = qauternionsTable.getRowAtIndex(i);
        for (int j = 0; j < nc; ++j) {
            const Quaternion& quatO = quatRow[j];
            matrix.updElt(cnt, j) = R_XG*Rotation(quatO);
        }
        cnt++;
    }

    TimeSeriesTable_<SimTK::Rotation> orientationTable(newTimes,
        matrix,
        qauternionsTable.getColumnLabels());
    orientationTable.updTableMetaData() = qauternionsTable.getTableMetaData();
    orientationTable.setDependentsMetaData(qauternionsTable.getDependentsMetaData());

    // Base will rotate to match <base>_imu, so we must first remove the base 
    // rotation from the other IMUs to get their orientation with respect to 
    // individual model bodies and thereby compute correct offsets unbiased by the 
    // initial base orientation.
    auto imuLabels = orientationTable.getColumnLabels();
    auto pix = distance(imuLabels.begin(), std::find(imuLabels.begin(), imuLabels.end(), "pelvis_imu")); //torso_imu
    auto startRow = orientationTable.getRowAtIndex(0);
    const Rotation& base_R = startRow.getElt(0, int(pix));

    //// Heading direction of the base IMU in this case the pelvis_imu heading is its ZAxis
    UnitVec3 pelvisHeading = base_R(SimTK::ZAxis);
    UnitVec3 groundX = UnitVec3(1, 0, 0);
    SimTK::Real angularDifference = acos(~pelvisHeading*groundX);

    // If the forward axis actually is the backward axis, change direction by 180 degrees
    if (angularDifference >= SimTK_PI / 2) {
        if (angularDifference >= 0) {
            angularDifference -= SimTK_PI;
        }
        else if (angularDifference <= -SimTK_PI / 2) {
            angularDifference += SimTK_PI;
        }
    }

    std::cout << "Heading correction computed to be "
        << angularDifference * SimTK_RADIAN_TO_DEGREE
        << "degs about ground Y" << std::endl;


    SimTK::Rotation R_HG = SimTK::Rotation(
        SimTK::BodyOrSpaceType::SpaceRotationSequence,
        0, SimTK::XAxis,
        angularDifference, SimTK::YAxis,
        0, SimTK::ZAxis
    );

    for (size_t i = 0; i < orientationTable.getNumRows(); ++i) {
        RowVectorView_<SimTK::Rotation>& rotationsRow = orientationTable.updRowAtIndex(i);
        for (int j = 0; j < nc; ++j) {
            rotationsRow[j] = R_HG*rotationsRow[j];
        }
    }

    return orientationTable;
}
