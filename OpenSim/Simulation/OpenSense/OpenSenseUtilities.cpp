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
        const TimeSeriesTableQuaternion& quaternionsTable,
        const SimTK::Array_<int>& startEnd,
        const std::string& baseImuName,
        const SimTK::CoordinateAxis& baseHeadingAxis)
{

    // Fixed transform to rotate sensor orientations in world with Z up into the 
    // OpenSim ground reference frame with Y up and X forward.
    SimTK::Rotation R_XG = SimTK::Rotation(
        SimTK::BodyOrSpaceType::BodyRotationSequence,
        -SimTK_PI / 2, SimTK::XAxis,
        0, SimTK::YAxis,
        0, SimTK::ZAxis);

    //SimTK::Vec3 grav(9.807, 0., 0.);
    //double theta = acos(avg_accel.transpose() * grav / (avg_accel.norm() * grav.norm()));
    //SimTK::Vec3 C = SimTK::cross(avg_accel, grav);
    //C = C / C.norm();

    //SimTK::Rotation tilt_correction = SimTK::Rotation(axisAngleToRotation(C, theta));
    //R_XG = R_XG * tilt_correction;

    int nc = int(quaternionsTable.getNumColumns());

    const auto& times = quaternionsTable.getIndependentColumn();

    size_t nt = startEnd[1] - startEnd[0] + 1;

    std::vector<double> newTimes(nt, SimTK::NaN);
    SimTK::Matrix_<SimTK::Rotation> matrix(int(nt), nc, Rotation());

    int cnt = 0;
    for (size_t i = startEnd[0]; i <= startEnd[1]; ++i) {
        newTimes[cnt] = times[i];
        const auto& quatRow = quaternionsTable.getRowAtIndex(i);
        for (int j = 0; j < nc; ++j) {
            const Quaternion& quatO = quatRow[j];
            matrix.updElt(cnt, j) = R_XG*Rotation(quatO);
        }
        cnt++;
    }

    TimeSeriesTable_<SimTK::Rotation> orientationTable(newTimes,
        matrix,
        quaternionsTable.getColumnLabels());
    orientationTable.updTableMetaData() = quaternionsTable.getTableMetaData();
    orientationTable.setDependentsMetaData(quaternionsTable.getDependentsMetaData());

    // if a base imu is specified, perform heading correction, otherwise skip
    if (!baseImuName.empty()) {

        // Base will rotate to match <base>_imu, so we must first remove the base 
        // rotation from the other IMUs to get their orientation with respect to 
        // individual model bodies and thereby compute correct offsets unbiased by the 
        // initial base orientation.
        auto imuLabels = orientationTable.getColumnLabels();
        auto pix = distance(imuLabels.begin(),
            std::find(imuLabels.begin(), imuLabels.end(), baseImuName));

        // if no base can be found but one was provided, throw.
        if (pix >= int(imuLabels.size())) {
            OPENSIM_THROW(Exception, 
                "No column with base IMU name '"+ baseImuName + "' found.");
        }

        auto startRow = orientationTable.getRowAtIndex(0);
        const Rotation& base_R = startRow.getElt(0, int(pix));

        // Heading direction of the base IMU in this case the pelvis_imu heading is its ZAxis
        UnitVec3 pelvisHeading = base_R(baseHeadingAxis);
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
            RowVectorView_<SimTK::Rotation> rotationsRow = orientationTable.updRowAtIndex(i);
            for (int j = 0; j < nc; ++j) {
                rotationsRow[j] = R_HG*rotationsRow[j];
            }
        }
    }

    return orientationTable;
}


void OpenSenseUtilities::calibrateModelFromOrientations(
    const string& modelCalibrationPoseFile,
    const string& calibrationOrientationsFile,
    const std::string& baseImuName,
    const SimTK::CoordinateAxis& baseHeadingAxis,
    bool visualizeCalibratedModel)
{
    Model model(modelCalibrationPoseFile);

    const SimTK::Array_<int>& startEnd = { 0, 1 };

    TimeSeriesTable_<SimTK::Quaternion> quatTable =
        STOFileAdapter_<SimTK::Quaternion>::readFile(calibrationOrientationsFile);


    TimeSeriesTable_<SimTK::Rotation> orientationsData =
        OpenSenseUtilities::convertQuaternionsToRotations(quatTable,
            startEnd, baseImuName, baseHeadingAxis);

    std::cout << "Loaded orientations as quaternions from "
        << calibrationOrientationsFile << std::endl;

    auto imuLabels = orientationsData.getColumnLabels();
    auto& times = orientationsData.getIndependentColumn();

    // The rotations of the IMUs at the start time in order
    // the labels in the TimerSeriesTable of orientations
    auto rotations = orientationsData.updRowAtIndex(0);

    SimTK::State& s0 = model.initSystem();
    s0.updTime() = times[0];

    // default pose of the model defined by marker-based IK 
    model.realizePosition(s0);

    size_t imuix = 0;
    std::vector<PhysicalFrame*> bodies{ imuLabels.size(), nullptr };
    std::map<std::string, SimTK::Rotation> imuBodiesInGround;

    // First compute the transform of each of the imu bodies in ground
    for (auto& imuName : imuLabels) {
        auto ix = imuName.rfind("_imu");
        if (ix != std::string::npos) {
            auto bodyName = imuName.substr(0, ix);
            auto body = model.findComponent<PhysicalFrame>(bodyName);
            if (body) {
                bodies[imuix] = const_cast<PhysicalFrame*>(body);
                imuBodiesInGround[imuName] =
                    body->getTransformInGround(s0).R();
            }
        }
        ++imuix;
    }

    // Now cycle through each imu with a body and compute the relative
    // offset of the IMU measurement relative to the body and
    // update the modelOffset OR add an offset if none exists
    imuix = 0;
    for (auto& imuName : imuLabels) {
        cout << "Processing " << imuName << endl;
        if (imuBodiesInGround.find(imuName) != imuBodiesInGround.end()) {
            cout << "Computed offset for " << imuName << endl;
            auto R_FB = ~imuBodiesInGround[imuName] * rotations[int(imuix)];
            cout << "Offset is " << R_FB << endl;
            PhysicalOffsetFrame* imuOffset = nullptr;
            const PhysicalOffsetFrame* mo = nullptr;
            if ((mo = model.findComponent<PhysicalOffsetFrame>(imuName))) {
                imuOffset = const_cast<PhysicalOffsetFrame*>(mo);
                auto X = imuOffset->getOffsetTransform();
                X.updR() = R_FB;
                imuOffset->setOffsetTransform(X);
            }
            else {
                cout << "Creating offset frame for " << imuName << endl;
                OpenSim::Body* body = dynamic_cast<OpenSim::Body*>(bodies[imuix]);
                SimTK::Vec3 p_FB(0);
                if (body) {
                    p_FB = body->getMassCenter();
                }

                imuOffset = new PhysicalOffsetFrame(imuName,
                    *bodies[imuix], SimTK::Transform(R_FB, p_FB));
                auto* brick = new Brick(Vec3(0.02, 0.01, 0.005));
                brick->setColor(SimTK::Orange);
                imuOffset->attachGeometry(brick);
                bodies[imuix]->addComponent(imuOffset);
                cout << "Added offset frame for " << imuName << endl;
            }
            cout << imuOffset->getName() << " offset computed from " <<
                imuName << " data from file." << endl;
        }
        imuix++;
    }

    model.finalizeConnections();

    auto filename = "calibrated_" + model.getName() + ".osim";
    cout << "Wrote calibrated model to file: '" << filename << "'." << endl;
    model.print(filename);
    if (visualizeCalibratedModel) {
        model.setUseVisualizer(true);
        SimTK::State& s = model.initSystem();

        s.updTime() = times[0];

        // create the solver given the input data
        MarkersReference mRefs{};
        OrientationsReference oRefs(orientationsData);
        SimTK::Array_<CoordinateReference> coordRefs{};

        const double accuracy = 1e-4;
        InverseKinematicsSolver ikSolver(model, mRefs, oRefs, coordRefs);
        ikSolver.setAccuracy(accuracy);

        model.getVisualizer().getSimbodyVisualizer().setShowSimTime(true);
        ikSolver.assemble(s);
        model.getVisualizer().show(s);

        char c;
        std::cout << "Press any key and return to close visualizer." << std::endl;
        std::cin >> c;
    }
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
                cout << "marker(s) for IMU '" << imuLabels[i] <<
                    "' is NaN and orientation will also be NaN." << endl;
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