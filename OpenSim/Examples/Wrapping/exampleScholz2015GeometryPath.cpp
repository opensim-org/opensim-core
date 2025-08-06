/* -------------------------------------------------------------------------- *
 *                OpenSim:  exampleScholz2015GeometryPath.cpp                 *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2025 Stanford University and the Authors                *
 * Author(s): Nicholas Bianco                                                 *
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

#include <OpenSim/OpenSim.h>

using namespace OpenSim;

int main() {

    Model model = ModelFactory::createDoublePendulum();
    model.setUseVisualizer(true);

    // Create a PathActuator with a Scholz2015GeometryPath.
    auto* actu = new PathActuator();
    actu->set_path(Scholz2015GeometryPath());
    model.addComponent(actu);   

    // Set the path's origin and insertion.
    Scholz2015GeometryPath& path = actu->updPath<Scholz2015GeometryPath>();
    path.setOrigin(model.getGround(), SimTK::Vec3(0.25, 0, 0));
    path.setInsertion(model.getComponent<Body>("/bodyset/b1"), 
            SimTK::Vec3(-0.5, 0.1, 0));

    // auto* obstacle = new ContactEllipsoid(SimTK::Vec3(0.1, 0.1, 0.3),
        // SimTK::Vec3(0.5, -0.05, 0), SimTK::Vec3(0), model.getComponent<Body>("/bodyset/root"));
    // auto* obstacle = new ContactCylinder(0.3,
    //     SimTK::Vec3(0.5, -0.05, 0), SimTK::Vec3(0), model.getComponent<Body>("/bodyset/root"));
    // model.addComponent(obstacle);
    // path.addObstacle(*obstacle, SimTK::Vec3(0.0, -0.3, 0.0));

    // Initialize the system.
    SimTK::State state = model.initSystem();


    // Simulate.
    Manager manager(model);
    manager.setIntegratorMaximumStepSize(1e-3);
    manager.initialize(state);
    manager.integrate(10.0);
}