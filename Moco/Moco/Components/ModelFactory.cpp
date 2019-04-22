/* -------------------------------------------------------------------------- *
 * OpenSim Moco: ModelFactory.cpp                                             *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2018 Stanford University and the Authors                     *
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

#include "ModelFactory.h"

#include "../MocoUtilities.h"

#include <OpenSim/Actuators/CoordinateActuator.h>
#include <OpenSim/Simulation/SimbodyEngine/PinJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/SliderJoint.h>

using namespace OpenSim;

using SimTK::Inertia;
using SimTK::Vec3;

Model ModelFactory::createNLinkPendulum(int numLinks) {
    Model model;
    OPENSIM_THROW_IF(numLinks < 0, Exception, "numLinks must be nonnegative.");
    std::string name;
    if (numLinks == 0) {
        name = "empty_model";
    } else if (numLinks == 1) {
        name = "pendulum";
    } else if (numLinks == 2) {
        name = "double_pendulum";
    } else {
        name = std::to_string(numLinks) + "_link_pendulum";
    }
    model.setName(name);
    const auto& ground = model.getGround();

    using SimTK::Inertia;
    using SimTK::Vec3;

    Ellipsoid bodyGeometry(0.5, 0.1, 0.1);
    bodyGeometry.setColor(SimTK::Gray);

    const PhysicalFrame* prevBody = &ground;
    for (int i = 0; i < numLinks; ++i) {
        const std::string istr = std::to_string(i);
        auto* bi = new OpenSim::Body("b" + istr, 1, Vec3(0), Inertia(1));
        model.addBody(bi);

        // Assume each body is 1 m long.
        auto* ji = new PinJoint("j" + istr, *prevBody, Vec3(0), Vec3(0), *bi,
                Vec3(-1, 0, 0), Vec3(0));
        auto& qi = ji->updCoordinate();
        qi.setName("q" + istr);
        model.addJoint(ji);

        auto* taui = new CoordinateActuator();
        taui->setCoordinate(&ji->updCoordinate());
        taui->setName("tau" + istr);
        taui->setOptimalForce(1);
        model.addComponent(taui);

        auto* marker = new Marker("marker" + istr, *bi, Vec3(0));
        model.addMarker(marker);

        // Attach an ellipsoid to a frame located at the center of each body.
        PhysicalOffsetFrame* bicenter = new PhysicalOffsetFrame(
                "b" + istr + "center", *bi, SimTK::Transform(Vec3(-0.5, 0, 0)));
        bi->addComponent(bicenter);
        bicenter->attachGeometry(bodyGeometry.clone());

        prevBody = bi;
    }

    model.finalizeConnections();

    return model;
}

Model ModelFactory::createPlanarPointMass() {
    Model model;
    model.setName("planar_point_mass");

    auto* intermed = new Body("intermed", 0, Vec3(0), Inertia(0));
    model.addBody(intermed);
    auto* body = new Body("body", 1, Vec3(0), Inertia(0));
    model.addBody(body);

    body->attachGeometry(new Sphere(0.05));

    auto* jointX = new SliderJoint();
    jointX->setName("tx");
    jointX->connectSocket_parent_frame(model.getGround());
    jointX->connectSocket_child_frame(*intermed);
    auto& coordX = jointX->updCoordinate(SliderJoint::Coord::TranslationX);
    coordX.setName("tx");
    model.addJoint(jointX);

    // The joint's x axis must point in the global "+y" direction.
    auto* jointY = new SliderJoint("ty", *intermed, Vec3(0),
            Vec3(0, 0, 0.5 * SimTK::Pi), *body, Vec3(0),
            Vec3(0, 0, .5 * SimTK::Pi));
    auto& coordY = jointY->updCoordinate(SliderJoint::Coord::TranslationX);
    coordY.setName("ty");
    model.addJoint(jointY);

    {
        auto* forceX = new CoordinateActuator();
        forceX->setCoordinate(&coordX);
        forceX->setName("force_x");
        model.addForce(forceX);
    }

    {
        auto* forceY = new CoordinateActuator();
        forceY->setCoordinate(&coordY);
        forceY->setName("force_y");
        model.addForce(forceY);
    }

    model.finalizeConnections();

    return model;
}

Model ModelFactory::createBrachistochrone() {
    Model model;
    class Brachistochrone : public ScalarActuator {
        OpenSim_DECLARE_CONCRETE_OBJECT(Brachistochrone, ScalarActuator);

    public:
        Brachistochrone() {
            g = std::abs(Model().get_gravity()[1]);
        }
        void extendAddToSystem(SimTK::MultibodySystem& system) const override {
            Super::extendAddToSystem(system);
            addStateVariable("x");
            addStateVariable("y");
            addStateVariable("v");
        }
        void extendInitStateFromProperties(SimTK::State& s) const override {
            Super::extendInitStateFromProperties(s);
            setStateVariableValue(s, "x", 0);
            setStateVariableValue(s, "y", 0);
            setStateVariableValue(s, "v", 0);
        }
        void computeStateVariableDerivatives(
                const SimTK::State& s) const override {
            const auto v = getStateVariableValue(s, "v");
            const auto u = getControl(s);
            setStateVariableDerivativeValue(s, "x", v * std::cos(u));
            setStateVariableDerivativeValue(s, "y", v * std::sin(u));
            setStateVariableDerivativeValue(s, "v", g * std::sin(u));
        }
        double computeActuation(const SimTK::State&) const override {
            return 0;
        }
    private:
        double g;
    };
    auto* b = new Brachistochrone();
    b->setName("brachistochrone");
    model.addComponent(b);
    model.finalizeConnections();
    return model;
}

void ModelFactory::createReserveActuators(Model& model, double optimalForce) {
    OPENSIM_THROW_IF(optimalForce <= 0, Exception,
            format("Invalid value (%g) for create_reserve_actuators; "
                   "should be -1 or positive.",
                    optimalForce));

    std::cout << "Adding reserve actuators with an optimal force of "
            << optimalForce << "..." << std::endl;

    Model modelCopy(model);
    auto state = modelCopy.initSystem();
    std::vector<std::string> coordPaths;
    // Borrowed from
    // CoordinateActuator::CreateForceSetOfCoordinateAct...
    for (const auto& coord : modelCopy.getComponentList<Coordinate>()) {
        if (!coord.isConstrained(state)) {
            auto* actu = new CoordinateActuator();
            actu->setCoordinate(
                    &model.updComponent<Coordinate>(
                            coord.getAbsolutePathString()));
            auto path = coord.getAbsolutePathString();
            coordPaths.push_back(path);
            // Get rid of model name.
            // Get rid of slashes in the path; slashes not allowed in names.
            std::replace(path.begin(), path.end(), '/', '_');
            actu->setName("reserve_" + path);
            actu->setOptimalForce(optimalForce);
            model.addForce(actu);
        }
    }
    // Re-make the system, since there are new actuators.
    model.initSystem();
    std::cout << "Added " << coordPaths.size()
            << " reserve actuator(s), "
               "for each of the following coordinates:"
            << std::endl;
    for (const auto& name : coordPaths) {
        std::cout << "  " << name << std::endl;
    }
}
