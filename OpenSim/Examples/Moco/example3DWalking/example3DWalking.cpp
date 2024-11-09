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

using namespace OpenSim;

void addContactsToModel(Model& model) {
    ContactGeometrySet contactGeometrySet("subject_walk_scaled_ContactGeometrySet.xml");
    for (int i = 0; i < contactGeometrySet.getSize(); ++i) {
        model.addContactGeometry(contactGeometrySet.get(i).clone());
    }
    model.finalizeConnections();

    ForceSet contactForceSet("subject_walk_scaled_ContactForceSet.xml");
    for (int i = 0; i < contactForceSet.getSize(); ++i) {
        model.addForce(contactForceSet.get(i).clone());
    }
    model.finalizeConnections();
}

int main() {

    Model model("subject_walk_scaled.osim");
    model.initSystem();
    addContactsToModel(model);
    model.print("subject_walk_scaled_with_contacts.osim");


    Model feetModel;
    feetModel.addBody(model.getComponent<Body>("/bodyset/calcn_r").clone());
    feetModel.addBody(model.getComponent<Body>("/bodyset/toes_r").clone());
    feetModel.addJoint(model.getComponent<Joint>("/jointset/mtp_r").clone());
    feetModel.addBody(model.getComponent<Body>("/bodyset/calcn_l").clone());
    feetModel.addBody(model.getComponent<Body>("/bodyset/toes_l").clone());
    feetModel.addJoint(model.getComponent<Joint>("/jointset/mtp_l").clone());

    feetModel.finalizeConnections();
    addContactsToModel(feetModel);
    feetModel.print("feet.osim");

    return EXIT_SUCCESS;
}