/* -------------------------------------------------------------------------- *
 *                         OpenSim:  IMUPlacer.cpp                      *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2019 Stanford University and the Authors                *
 * Author(s): Ayman Habib                                                     *
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

//=============================================================================
// INCLUDES
//=============================================================================
#include "IMUPlacer.h"

#include "OpenSenseUtilities.h"

#include <OpenSim/Simulation/InverseKinematicsSolver.h>
#include <OpenSim/Simulation/MarkersReference.h>
#include <OpenSim/Simulation/Model/Model.h>

using namespace std;
using namespace OpenSim;
using SimTK::Vec3;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
IMUPlacer::IMUPlacer() {
    constructProperties();
    _calibrated = false;
}

IMUPlacer::IMUPlacer(const std::string& setupFile) : Object(setupFile, true) {
    constructProperties();
    updateFromXMLDocument();
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
IMUPlacer::~IMUPlacer() {}

//=============================================================================
// CONSTRUCTION

//_____________________________________________________________________________
/**
 */
void IMUPlacer::constructProperties() {
    constructProperty_model_file("");
    constructProperty_base_imu_label("");
    constructProperty_base_heading_axis("");
    constructProperty_sensor_to_opensim_rotations(SimTK::Vec3(0));
    constructProperty_orientation_file_for_calibration("");
    constructProperty_output_model_file("");
}

//=============================================================================
// UTILITY
//=============================================================================
//_____________________________________________________________________________
/**
 * This method runs the calibration method on the _model maintained by
 * this IMUPlacer
 */
bool IMUPlacer::run(bool visualizeResults) {

    _calibrated = false;
    // Check there's a model file specified before trying to open it
    if (_model.empty() && get_model_file().size() == 0) {
        OPENSIM_THROW(Exception, "No model or model_file was specified for IMUPlacer.");
    }
    if (_model.empty()) { _model.reset(new Model(get_model_file())); }
    TimeSeriesTable_<SimTK::Quaternion> quatTable(
            get_orientation_file_for_calibration());

    const SimTK::Vec3& sensor_to_opensim_rotations =
            get_sensor_to_opensim_rotations();
    SimTK::Rotation sensorToOpenSim =
            SimTK::Rotation(SimTK::BodyOrSpaceType::SpaceRotationSequence,
                    sensor_to_opensim_rotations[0], SimTK::XAxis,
                    sensor_to_opensim_rotations[1], SimTK::YAxis,
                    sensor_to_opensim_rotations[2], SimTK::ZAxis);
    // Rotate data so Y-Axis is up
    OpenSenseUtilities::rotateOrientationTable(quatTable, sensorToOpenSim);
    // Heading correction requires initSystem is already called. Do it now.
    SimTK::State& s0 = _model->initSystem();
    _model->realizePosition(s0);
    // Check consistent heading correction specification
    // both base_heading_axis and base_imu_label should be specified
    // finer error checking is done downstream
    bool performHeadingRequested =
            !get_base_heading_axis().empty() && !get_base_imu_label().empty();
    if (performHeadingRequested) {
        std::string imu_axis = IO::Lowercase(get_base_heading_axis());

        SimTK::CoordinateDirection directionOnIMU(SimTK::ZAxis);
        int direction = 1;
        if (imu_axis.front() == '-') direction = -1;
        char& back = imu_axis.back();
        if (back == 'x')
            directionOnIMU =
                    SimTK::CoordinateDirection(SimTK::XAxis, direction);
        else if (back == 'y')
            directionOnIMU =
                    SimTK::CoordinateDirection(SimTK::YAxis, direction);
        else if (back == 'z')
            directionOnIMU =
                    SimTK::CoordinateDirection(SimTK::ZAxis, direction);
        else { // Throw, invalid specification
            OPENSIM_THROW(Exception, "Invalid specification of heading axis '" +
                                             imu_axis + "' found.");
        }

        // Compute rotation matrix so that (e.g. "pelvis_imu"+ SimTK::ZAxis)
        // lines up with model forward (+X)
        SimTK::Vec3 headingRotationVec3 =
                OpenSenseUtilities::computeHeadingCorrection(*_model, s0, quatTable,
                        get_base_imu_label(), directionOnIMU);
        SimTK::Rotation headingRotation(
                SimTK::BodyOrSpaceType::SpaceRotationSequence,
                headingRotationVec3[0], SimTK::XAxis, headingRotationVec3[1],
                SimTK::YAxis, headingRotationVec3[2], SimTK::ZAxis);

        OpenSenseUtilities::rotateOrientationTable(quatTable, headingRotation);
    } else
        log_info("No heading correction is applied.");

    // This is now plain conversion, no Rotation or magic underneath
    TimeSeriesTable_<SimTK::Rotation>
            orientationsData =
    OpenSenseUtilities::convertQuaternionsToRotations(quatTable);

    auto imuLabels = orientationsData.getColumnLabels();
    auto& times = orientationsData.getIndependentColumn();

    // The rotations of the IMUs at the start time in order
    // the labels in the TimerSeriesTable of orientations
    auto rotations = orientationsData.updRowAtIndex(0);

    s0.updTime() = times[0];

    // default pose of the model defined by marker-based IK
    _model->realizePosition(s0);

    size_t imuix = 0;
    std::vector<PhysicalFrame*> bodies{imuLabels.size(), nullptr};
    std::map<std::string, SimTK::Rotation> imuBodiesInGround;

    // First compute the transform of each of the imu bodies in ground
    int cnt = 0; // check name match between data and model frames
    for (auto& imuName : imuLabels) {
        auto ix = imuName.rfind("_imu");
        if (ix != std::string::npos) {
            auto bodyName = imuName.substr(0, ix);
            auto body = _model->findComponent<PhysicalFrame>(bodyName);
            if (body) {
                bodies[imuix] = const_cast<PhysicalFrame*>(body);
                imuBodiesInGround[imuName] = body->getTransformInGround(s0).R();
                cnt++;
            }
        }
        ++imuix;
    }
    // we check cnt >= 1 since no frames is rather meaningless I think
    // -Ayman 10/20
    if (cnt < 1) {
        log_error("IMUPlacer: Calibration data does not "
                  "correspond to any model frames. "
                  "Column names must formatted as (bodyname)_imu");
        throw Exception("IMUPlacer: Calibration data does not "
                        "correspond to any model frames."
                        "Column names must formatted as (bodyname)_imu");
    }
    // Now cycle through each imu with a body and compute the relative
    // offset of the IMU measurement relative to the body and
    // update the modelOffset OR add an offset if none exists
    imuix = 0;
    for (auto& imuName : imuLabels) {
        log_info("Processing {}", imuName);
        if (imuBodiesInGround.find(imuName) != imuBodiesInGround.end()) {
            log_info("Computed offset for {}", imuName);
            SimTK::Rotation R_FB =
                    ~imuBodiesInGround[imuName] * rotations[int(imuix)];
            log_info("Offset is {}", R_FB);
            PhysicalOffsetFrame* imuOffset = nullptr;
            const PhysicalOffsetFrame* mo = nullptr;
            if ((mo = _model->findComponent<PhysicalOffsetFrame>(imuName))) {
                imuOffset = const_cast<PhysicalOffsetFrame*>(mo);
                auto X = imuOffset->getOffsetTransform();
                X.updR() = R_FB;
                imuOffset->setOffsetTransform(X);
            } else {
                log_info("Creating offset frame for {}", imuName);
                OpenSim::Body* body =
                        dynamic_cast<OpenSim::Body*>(bodies[imuix]);
                SimTK::Vec3 p_FB(0);
                if (body) { p_FB = body->getMassCenter(); }

                imuOffset = new PhysicalOffsetFrame(
                        imuName, *bodies[imuix], SimTK::Transform(R_FB, p_FB));
                bodies[imuix]->addComponent(imuOffset);
                // Create an IMU Object in the model, connect it to imuOffset
                IMU* modelImu = new IMU();
                modelImu->setName(imuName);
                modelImu->connectSocket_frame(*imuOffset);
                _model->addModelComponent(modelImu);
                log_info("Added offset frame for {}.", imuName);
            }
            log_info("{} offset computed from {} data from file.",
                    imuOffset->getName(), imuName);
        }
        imuix++;
    }

    _model->finalizeConnections();

    if (!get_output_model_file().empty())
        _model->print(get_output_model_file());

    _calibrated = true;
    if (visualizeResults) {
        _model->setUseVisualizer(true);
        SimTK::State& s = _model->initSystem();

        s.updTime() = times[0];

        // create the solver given the input data
        OrientationsReference oRefs(orientationsData);
        SimTK::Array_<CoordinateReference> coordRefs{};

        const double accuracy = 1e-4;
        InverseKinematicsSolver ikSolver(*_model, nullptr,
                std::make_shared<OrientationsReference>(oRefs), coordRefs);
        ikSolver.setAccuracy(accuracy);

        SimTK::Visualizer& viz = _model->updVisualizer().updSimbodyVisualizer();
        // We use the input silo to get key presses.
        auto silo = &_model->updVisualizer().updInputSilo();
        silo->clear(); // Ignore any previous key presses.

        SimTK::DecorativeText help("Press any key to quit.");
        help.setIsScreenText(true);
        viz.addDecoration(SimTK::MobilizedBodyIndex(0), SimTK::Vec3(0), help);
        _model->getVisualizer().getSimbodyVisualizer().setShowSimTime(true);
        ikSolver.assemble(s);
        _model->getVisualizer().show(s);

        unsigned key, modifiers;
        silo->waitForKeyHit(key, modifiers);
        viz.shutdown();
    }
    return true;
}

Model& IMUPlacer::getCalibratedModel() const {
    if (_calibrated) return *_model;
    OPENSIM_THROW(Exception, "Attempt to retrieve calibrated model without "
                             "invoking IMU_Placer::run.");
}
