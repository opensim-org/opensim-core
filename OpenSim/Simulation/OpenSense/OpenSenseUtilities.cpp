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
#include "IMUPlacer.h"

using namespace OpenSim;
using namespace SimTK;
using namespace std;

TimeSeriesTable_<SimTK::Rotation> OpenSenseUtilities::
    convertQuaternionsToRotations(
        const TimeSeriesTableQuaternion& quaternionsTable)
{

    int nc = int(quaternionsTable.getNumColumns());

    const auto& times = quaternionsTable.getIndependentColumn();

    size_t nt = int(quaternionsTable.getNumRows());

    std::vector<double> newTimes(nt, SimTK::NaN);
    SimTK::Matrix_<SimTK::Rotation> matrix(int(nt), nc, Rotation());

    int cnt = 0;
    for (size_t i = 0; i < nt; ++i) {
        newTimes[cnt] = times[i];
        const auto& quatRow = quaternionsTable.getRowAtIndex(i);
        for (int j = 0; j < nc; ++j) {
            const Quaternion& quatO = quatRow[j];
            matrix.updElt(cnt, j) = Rotation(quatO);
        }
        cnt++;
    }

    TimeSeriesTable_<SimTK::Rotation> orientationTable(newTimes,
        matrix,
        quaternionsTable.getColumnLabels());
    orientationTable.updTableMetaData() = quaternionsTable.getTableMetaData();
    orientationTable.setDependentsMetaData(quaternionsTable.getDependentsMetaData());

    return orientationTable;
}


void OpenSim::OpenSenseUtilities::rotateOrientationTable(
        OpenSim::TimeSeriesTable_<SimTK::Quaternion_<double>>&
                quaternionsTable,
        const SimTK::Rotation_<double>& rotationMatrix)
{
    SimTK::Rotation R_XG = rotationMatrix;

    int nc = int(quaternionsTable.getNumColumns());
    size_t nt = quaternionsTable.getNumRows();

    for (size_t i = 0; i < nt; ++i) {
        auto quatRow = quaternionsTable.updRowAtIndex(i);
        for (int j = 0; j < nc; ++j) {
            // This can be done completely in Quaternions but this is easier to debug for now
            Quaternion quatO = (R_XG * Rotation(quatRow[j])).convertRotationToQuaternion();
            quatRow[j] = quatO;
        }
    }
    return;
}

SimTK::Transform OpenSenseUtilities::formTransformFromPoints(const Vec3& op,
    const Vec3& xp,
    const Vec3& yp)
{
    OPENSIM_THROW_IF(op.isNaN() || xp.isNaN() || yp.isNaN(),
        OpenSim::Exception,
        "formTransformFromPoints: An input point is NaN.");

    UnitVec3 ux{ xp - op };
    UnitVec3 uy{ yp - op };
    UnitVec3 uz{ ux % uy };

    Mat33 nearRot{ ux, uy, uz };
    Rotation R{ nearRot };

    SimTK::Transform X{ R, op };

    return X;
}

TimeSeriesTable_<SimTK::Quaternion>
OpenSenseUtilities::createOrientationsFileFromMarkers(const std::string& markersFile)
{
    TimeSeriesTableVec3 table{ markersFile };

    // labels of markers including those <bodyName>O,X,Y that identify the
    // IMU sensor placement/alignment on the body expressed in Ground
    auto labels = table.getColumnLabels();

    std::string suffix{ "_IMU" };

    std::vector<std::string> imuLabels;
    std::vector<std::vector<int>> imuIndices;

    auto it = labels.begin();

    size_t nImu = 0;
    size_t index;
    while (it != labels.end()) {
        std::vector<int> indices;
        auto ix = it->find(suffix);
        if (ix > it->size()) {
            it++;
            continue;
        }
        string base = it->substr(0, ix);

        imuLabels.push_back(IO::Lowercase(base + suffix));

        index = table.getColumnIndex(base + suffix + "_O");
        indices.push_back(int(index));
        it++;

        index = table.getColumnIndex(base + suffix + "_X");
        indices.push_back(int(index));
        it++;

        index = table.getColumnIndex(base + suffix + "_Y");
        indices.push_back(int(index));
        it++;

        index = table.getColumnIndex(base + suffix + "_D");
        indices.push_back(int(index));
        it++;

        // Left leg plates have additional marker to differentiate from right side.
        if (table.hasColumn(base + suffix + "_5")) it++;

        imuIndices.push_back(indices);
    }

    const auto& markerData = table.getMatrix();
    SimTK::Matrix_<SimTK::Quaternion> quatData{ markerData.nrow(), int(imuLabels.size()) };

    Vec3 op, xp, yp, dp;
    SimTK::Quaternion quat{ SimTK::NaN, SimTK::NaN, SimTK::NaN, SimTK::NaN };

    for (int row = 0; row < markerData.nrow(); ++row) {
        // reset marker and quaternion to NaN
        op = xp = yp = dp = Vec3{ SimTK::NaN };
        quat *= SimTK::NaN;

        for (size_t i = 0; i < imuLabels.size(); ++i) {
            op = markerData.getElt(row, imuIndices[i][0]);
            xp = markerData.getElt(row, imuIndices[i][1]);
            yp = markerData.getElt(row, imuIndices[i][2]);
            dp = markerData.getElt(row, imuIndices[i][3]);

            if (op.isNaN() || xp.isNaN() || yp.isNaN()) {
                log_warn("marker(s) for IMU '{}' is NaN and orientation will also be NaN.",
                    imuLabels[i]);
            }
            else {
                // Transform of the IMU formed from markers expressed in Ground
                auto X_FG = formTransformFromPoints(op, xp, yp);
                quat = X_FG.R().convertRotationToQuaternion();
            }
            quatData(row, int(i)) = quat;
        }

    }

    TimeSeriesTable_<SimTK::Quaternion> quaternions{
        table.getIndependentColumn(), quatData, imuLabels };

    auto ix = markersFile.find(".");
    string fileName = markersFile.substr(0, ix) + "_quaternions.sto";

    STOFileAdapter_<SimTK::Quaternion>::write(quaternions, fileName);

    return quaternions;
 }

SimTK::Vec3 OpenSenseUtilities::computeHeadingCorrection(
        Model& model,
        const SimTK::State& state,
            OpenSim::TimeSeriesTable_<SimTK::Quaternion_<double>>&
                    quaternionsTable,
            const std::string& baseImuName,
            const SimTK::CoordinateDirection baseHeadingDirection)
{
     SimTK::Vec3 rotations{0};

    // if a base imu is specified, perform heading correction, otherwise skip
    if (!baseImuName.empty()) {

        auto imuLabels = quaternionsTable.getColumnLabels();
        auto pix = distance(imuLabels.begin(),
                std::find(imuLabels.begin(), imuLabels.end(), baseImuName));

        // if no base can be found but one was provided, throw.
        if (pix >= int(imuLabels.size())) {
            OPENSIM_THROW(Exception, "No column with base IMU name '" +
                                             baseImuName + "' found.");
        }

        auto startRow = quaternionsTable.getRowAtIndex(0);
        Rotation base_R = Rotation(startRow.getElt(0, int(pix)));

        // Heading direction of the base IMU in this case the pelvis_imu heading
        // is its ZAxis
        UnitVec3 baseSegmentXHeading = base_R(baseHeadingDirection.getAxis());
        if (baseHeadingDirection.getDirection() < 0)
            baseSegmentXHeading = baseSegmentXHeading.negate();
        bool baseFrameFound = false;

        const Frame* baseFrame = nullptr;
        for (int j = 0; j < model.getNumJoints() && !baseFrameFound; j++) {
            auto& jnt = model.getJointSet().get(j);
            // Look for joint whose parent is Ground, child will be baseFrame
            if (jnt.getParentFrame().findBaseFrame() == model.getGround()) { 
                baseFrame = &(jnt.getChildFrame().findBaseFrame());
                baseFrameFound = true;
                break;
            }
        }
        OPENSIM_THROW_IF(!baseFrameFound, Exception,
                "No base segment was found, disable heading correction and retry.");
       
        Vec3 baseFrameX = UnitVec3(1, 0, 0);
        const SimTK::Transform& baseXform =
                baseFrame->getTransformInGround(state);
        Vec3 baseFrameXInGround = baseXform.xformFrameVecToBase(baseFrameX);
        SimTK::Real angularDifference =
                acos(~baseSegmentXHeading * baseFrameXInGround);
        // Compute the sign of the angular correction.
        SimTK::Vec3 xproduct = (baseFrameXInGround % baseSegmentXHeading);
        if (xproduct.get(1) > 0) { 
            angularDifference *= -1; 
        }

        log_info("Heading correction computed to be {} degs about ground Y.",
            angularDifference * SimTK_RADIAN_TO_DEGREE);

        rotations = SimTK::Vec3( 0, angularDifference,  0);

    }
    else
        OPENSIM_THROW(Exception,
                "Heading correction attempted without base imu specification. Aborting.'");
    return rotations;
}

std::vector<OpenSim::IMU* > OpenSenseUtilities::addModelIMUs(
    Model& model, std::vector<std::string>& paths) {

    std::vector<OpenSim::IMU*> selectedIMUs;
    for (auto path : paths) {
        IMU* next_imu = new IMU();
        const Component& comp = model.getComponent(path);
        next_imu->setName(comp.getName() + "_imu");
        next_imu->connectSocket_frame(comp);
        model.addModelComponent(next_imu);
        // make sure it's a Frame
        selectedIMUs.push_back(next_imu); 
    }
    model.finalizeConnections();
    return selectedIMUs;
}