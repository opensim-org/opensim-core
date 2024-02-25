/* -------------------------------------------------------------------------- *
 * OpenSim Moco: sandboxSandbox.cpp                                           *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2019 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Christopher Dembia                                              *
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

// This file provides a way to easily prototype or test temporary snippets of
// code during development.

#include <OpenSim/Moco/osimMoco.h>
#include <OpenSim/Actuators/ModelFactory.h>

using namespace OpenSim;

int main() {
    Model model = ModelFactory::createSlidingPointMass();
    model.initSystem();

    auto* origin = new Station();
    origin->setName("origin");
    origin->set_location(SimTK::Vec3(0));
    origin->setParentFrame(model.getGround());
    model.addComponent(origin);

    auto* insertion = new Station();
    insertion->setName("insertion");
    insertion->set_location(SimTK::Vec3(0));
    insertion->setParentFrame(model.getComponent<Body>("/body"));
    model.addComponent(insertion);

    auto* wrapCylinder = new GeodesicWrapCylinder();
    wrapCylinder->setName("wrap_cylinder");
    wrapCylinder->setRadius(1.0);
    wrapCylinder->connectSocket_frame(model.getGround());
    model.addComponent(wrapCylinder);
    model.finalizeConnections();

    Scholz2015GeodesicPath geodesicPath;
    geodesicPath.setName("geodesic_path");


    auto* actu = new PathActuator();
    actu->set_path(geodesicPath);
    actu->setName("actuator");
    actu->setOptimalForce(1);
    model.addComponent(actu);


    std::vector<std::reference_wrapper<GeodesicWrapSurface>> surfaces;
    surfaces.emplace_back(*wrapCylinder);

    std::vector<GeodesicInitialConditions> initialConditions;
    GeodesicInitialConditions ic({1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}, 1.0);
    initialConditions.push_back(ic);

    auto& path = model.updComponent<PathActuator>("/actuator").updPath<Scholz2015GeodesicPath>();
    path.addPathSegment(surfaces,
            initialConditions,
            model.getComponent<Station>("/origin"),
            model.getComponent<Station>("/insertion"));
    model.finalizeConnections();


    model.setUseVisualizer(true);
    SimTK::State state = model.initSystem();

    const auto& viz = model.getVisualizer();
    viz.show(state);

    model.print("sandboxScholz2015GeodesicPath_model.osim");

    return EXIT_SUCCESS;
}
