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

    auto* wrapCylinder = new GeodesicWrapCylinder(0.2, "implicit");
    wrapCylinder->setName("wrap_cylinder");
    wrapCylinder->connectSocket_frame(model.getGround());
    model.addComponent(wrapCylinder);

    auto* geodesicPath = new Scholz2015GeodesicPath;
    geodesicPath->setName("geodesic_path");

    GeodesicPathSegment* segment1 = geodesicPath->addPathSegment(
            "segment1", model.getGround(), Vec3(0),
            model.getComponent<Body>("/body"), Vec3(0));

    GeodesicInitialConditions initialConditions1(Vec3(0), Vec3(0), 1.0);
    segment1->addWrapObject(
            model.getComponent<GeodesicWrapCylinder>("wrap_cylinder"),
            initialConditions1);

    GeodesicPathSegment* segment2 = geodesicPath->addPathSegment(
            "segment2", model.getComponent<Body>("/body"), Vec3(1.0));

    GeodesicInitialConditions initialConditions2(Vec3(1.0), Vec3(1.0), 1.0);
    segment2->addWrapObject(
            model.getComponent<GeodesicWrapCylinder>("wrap_cylinder"),
            initialConditions2);

    auto* actu = new PathActuator();
    actu->setPath(geodesicPath);
    actu->setName("actuator");
    actu->setOptimalForce(1);
    model.addComponent(actu);

    model.setUseVisualizer(true);
    SimTK::State state = model.initSystem();

    const auto& viz = model.getVisualizer();
    viz.show(state);

    model.print("sandboxScholz2015GeodesicPath_model.osim");

    return EXIT_SUCCESS;
}
