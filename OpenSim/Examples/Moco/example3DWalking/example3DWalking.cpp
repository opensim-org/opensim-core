/* -------------------------------------------------------------------------- *
 * OpenSim Moco: example3DWalking.cpp                                         *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2024 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Nicholas Bianco                                                 *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0          *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

#include <OpenSim/Moco/osimMoco.h>
#include <OpenSim/Common/LinearFunction.h>
#include <OpenSim/Tools/InverseKinematicsTool.h>
#include <OpenSim/Common/STOFileAdapter.h>
#include <OpenSim/Simulation/VisualizerUtilities.h>

using namespace OpenSim;

void addContactsToModel(Model& model) {
    ContactGeometrySet contactGeometrySet("subject_walk_scaled_ContactGeometrySet.xml");
    for (int i = 0; i < contactGeometrySet.getSize(); ++i) {
        ContactGeometry* contactGeometry = contactGeometrySet.get(i).clone();
        // Raise the ContactSpheres up by 2 cm so that the bottom of the spheres
        // are better aligned with the ground.
        if (contactGeometry->getName() != "floor") {
            SimTK::Vec3& location = contactGeometry->upd_location();
            location[1] += 0.02; 
        }
        model.addContactGeometry(contactGeometry);
    }
    model.finalizeConnections();

    ForceSet contactForceSet("subject_walk_scaled_ContactForceSet.xml");
    for (int i = 0; i < contactForceSet.getSize(); ++i) {
        model.addForce(contactForceSet.get(i).clone());
    }
    model.finalizeConnections();
}

void addGroundToFootJoint(Model& model, const std::string& footBodyName) {
    SpatialTransform transform;
    transform[0].setCoordinateNames(OpenSim::Array<std::string>(
            fmt::format("{}_rx", footBodyName), 1, 1));
    transform[0].setFunction(new LinearFunction());
    transform[0].setAxis(SimTK::Vec3(0, 0, 1));

    transform[1].setCoordinateNames(OpenSim::Array<std::string>(
            fmt::format("{}_ry", footBodyName), 1, 1));
    transform[1].setFunction(new LinearFunction());
    transform[1].setAxis(SimTK::Vec3(1, 0, 0));

    transform[2].setCoordinateNames(OpenSim::Array<std::string>(
            fmt::format("{}_rz", footBodyName), 1, 1));
    transform[2].setFunction(new LinearFunction());
    transform[2].setAxis(SimTK::Vec3(0, 1, 0));

    transform[3].setCoordinateNames(OpenSim::Array<std::string>(
            fmt::format("{}_tx", footBodyName), 1, 1));
    transform[3].setFunction(new LinearFunction());
    transform[3].setAxis(SimTK::Vec3(1, 0, 0));

    transform[4].setCoordinateNames(OpenSim::Array<std::string>(
            fmt::format("{}_ty", footBodyName), 1, 1));
    transform[4].setFunction(new LinearFunction());
    transform[4].setAxis(SimTK::Vec3(0, 1, 0));

    transform[5].setCoordinateNames(OpenSim::Array<std::string>(
            fmt::format("{}_tz", footBodyName), 1, 1));
    transform[5].setFunction(new LinearFunction());
    transform[5].setAxis(SimTK::Vec3(0, 0, 1));

    CustomJoint* joint = new CustomJoint(
            fmt::format("ground_{}", footBodyName), 
            model.getGround(), 
            model.getComponent<Body>(fmt::format("/bodyset/{}", footBodyName)), 
            transform);
    model.addJoint(joint);
}

int main() {

    // Load the base model.
    Model model("subject_walk_scaled.osim");
    model.initSystem();

    // Add the contact geometry and forces to the model.
    addContactsToModel(model);
    model.print("subject_walk_scaled_with_contacts.osim");

    // Create a model with only the feet of the original model.
    Model feetModel;
    feetModel.addBody(model.getComponent<Body>("/bodyset/calcn_r").clone());
    feetModel.addBody(model.getComponent<Body>("/bodyset/toes_r").clone());
    feetModel.addJoint(model.getComponent<Joint>("/jointset/mtp_r").clone());
    feetModel.addBody(model.getComponent<Body>("/bodyset/calcn_l").clone());
    feetModel.addBody(model.getComponent<Body>("/bodyset/toes_l").clone());
    feetModel.addJoint(model.getComponent<Joint>("/jointset/mtp_l").clone());

    // Define the ground-to-foot joints using CustomJoints.
    addGroundToFootJoint(feetModel, "calcn_r");
    addGroundToFootJoint(feetModel, "calcn_l");

    // Add the markers from the feet to the new model.
    std::vector<std::string> feetMarkers = {"Toe", "ToeGround", "MT5", 
            "MT5Ground", "Heel", "HeelGround"};
    for (const auto& markerName : feetMarkers) {
        for (const auto& side : {"R.", "L."}) {
            auto marker = model.getComponent<Marker>(
                    fmt::format("/markerset/{}{}", side, markerName)).clone();
            feetModel.addMarker(marker);
        }
    }
    feetModel.finalizeConnections();

    // Add the contact geometry and forces to the feet model.
    addContactsToModel(feetModel);
    feetModel.print("feet.osim");

    // Run inverse kinematics on the foot model.
    Set<MarkerWeight> markerWeights;
    markerWeights.cloneAndAppend({"R.Heel", 10});
    markerWeights.cloneAndAppend({"R.MT5", 5});
    markerWeights.cloneAndAppend({"R.Toe", 2});
    markerWeights.cloneAndAppend({"L.Heel", 10});
    markerWeights.cloneAndAppend({"L.MT5", 5});
    markerWeights.cloneAndAppend({"L.Toe", 2});
    markerWeights.cloneAndAppend({"R.HeelGround", 1});
    markerWeights.cloneAndAppend({"R.MT5Ground", 1});
    markerWeights.cloneAndAppend({"R.ToeGround", 1});
    markerWeights.cloneAndAppend({"L.HeelGround", 1});
    markerWeights.cloneAndAppend({"L.MT5Ground", 1});
    markerWeights.cloneAndAppend({"L.ToeGround", 1});

    IKTaskSet ikTaskSet;
    ikTaskSet.createMarkerWeightSet(markerWeights);

    InverseKinematicsTool iktool;
    iktool.setModel(feetModel);
    iktool.setMarkerDataFileName("marker_trajectories.trc");
    iktool.set_IKTaskSet(ikTaskSet);
    iktool.setOutputMotionFileName("feet_ik_solution.sto");    
    iktool.run();

    // Update the IK solution to something that we can visualize and use 
    // in Moco.
    TableProcessor tableProcessor = 
            TableProcessor("feet_ik_solution.sto") |
            TabOpConvertDegreesToRadians() |
            TabOpAppendCoordinateValueDerivativesAsSpeeds() |
            TabOpUseAbsoluteStateNames();
    TimeSeriesTable feetCoordinateReference = 
            tableProcessor.processAndConvertToRadians("", feetModel);
    STOFileAdapter::write(feetCoordinateReference, 
            "feet_coordinate_reference.sto");

    VisualizerUtilities::showMotion(feetModel, feetCoordinateReference);

    return EXIT_SUCCESS;
}