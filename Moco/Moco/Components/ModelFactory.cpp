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
#include <OpenSim/Simulation/SimbodyEngine/PinJoint.h>
#include <OpenSim/Actuators/CoordinateActuator.h>
#include "../MocoUtilities.h"
#include <OpenSim/Simulation/SimbodyEngine/WeldJoint.h>


using namespace OpenSim;

void ModelFactory::replaceMusclesWithPathActuators(OpenSim::Model &model) {

    // Create path actuators from muscle properties and add to the model. Save
    // a list of pointers of the muscles to delete.
    std::vector<Muscle*> musclesToDelete;
    auto& muscleSet = model.updMuscles();
    for (int i = 0; i < muscleSet.getSize(); ++i) {
        auto& musc = muscleSet.get(i);
        auto* actu = new PathActuator();
        actu->setName(musc.getName());
        musc.setName(musc.getName() + "_delete");
        actu->setOptimalForce(musc.getMaxIsometricForce());
        actu->setMinControl(musc.getMinControl());
        actu->setMaxControl(musc.getMaxControl());

        const auto& pathPointSet = musc.getGeometryPath().getPathPointSet();
        auto& geomPath = actu->updGeometryPath();
        for (int i = 0; i < pathPointSet.getSize(); ++i) {
            auto* pathPoint = pathPointSet.get(i).clone();
            const auto& socketNames = pathPoint->getSocketNames();
            for (const auto& socketName : socketNames) {
                pathPoint->updSocket(socketName)
                        .connect(pathPointSet.get(i)
                                         .getSocket(socketName)
                                         .getConnecteeAsObject());
            }
            geomPath.updPathPointSet().adoptAndAppend(pathPoint);
        }
        model.addComponent(actu);
        musclesToDelete.push_back(&musc);
    }

    // Delete the muscles.
    for (const auto* musc : musclesToDelete) {
        int index = model.getForceSet().getIndex(musc, 0);
        OPENSIM_THROW_IF(index == -1, Exception,
                         format("Muscle with name %s not found in ForceSet.",
                                musc->getName()));
        bool success = model.updForceSet().remove(index);
        OPENSIM_THROW_IF(!success, Exception,
                         format("Attempt to remove muscle with "
                                "name %s was unsuccessful.",
                                musc->getName()));
    }
}


void ModelFactory::replaceJointWithWeldJoint(
        Model& model, const std::string& jointName) {
    OPENSIM_THROW_IF(!model.getJointSet().hasComponent(jointName), Exception,
                     "Joint with name '" + jointName +
                     "' not found in the model JointSet.");

    // This is needed here to access offset frames.
    model.finalizeConnections();

    // Get the current joint and save a copy of the parent and child offset
    // frames.
    auto& current_joint = model.updJointSet().get(jointName);
    PhysicalOffsetFrame* parent_offset = PhysicalOffsetFrame().safeDownCast(
            current_joint.getParentFrame().clone());
    PhysicalOffsetFrame* child_offset = PhysicalOffsetFrame().safeDownCast(
            current_joint.getChildFrame().clone());

    // Save the original names of the body frames (not the offset frames), so we
    // can find them when the new joint is created.
    parent_offset->finalizeConnections(model);
    child_offset->finalizeConnections(model);
    std::string parent_body_path =
            parent_offset->getParentFrame().getAbsolutePathString();
    std::string child_body_path =
            child_offset->getParentFrame().getAbsolutePathString();

    // Remove the current Joint from the the JointSet.
    model.updJointSet().remove(&current_joint);

    // Create the new joint and add it to the model.
    auto* new_joint = new WeldJoint(jointName,
                                    model.getComponent<PhysicalFrame>(parent_body_path),
                                    parent_offset->get_translation(), parent_offset->get_orientation(),
                                    model.getComponent<PhysicalFrame>(child_body_path),
                                    child_offset->get_translation(), child_offset->get_orientation());
    model.addJoint(new_joint);

    model.finalizeConnections();
}

void ModelFactory::removeMuscles(Model& model) {

    // Save a list of pointers of the muscles to delete.
    std::vector<Muscle*> musclesToDelete;
    auto& muscleSet = model.updMuscles();
    for (int i = 0; i < muscleSet.getSize(); ++i) {
        musclesToDelete.push_back(&muscleSet.get(i));
    }

    // Delete the muscles.
    for (const auto* musc : musclesToDelete) {
        int index = model.getForceSet().getIndex(musc, 0);
        OPENSIM_THROW_IF(index == -1, Exception,
                         format("Muscle with name %s not found in ForceSet.",
                                musc->getName()));
        model.updForceSet().remove(index);
    }
}

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

    using SimTK::Vec3;
    using SimTK::Inertia;

    Ellipsoid bodyGeometry(0.5, 0.1, 0.1);
    bodyGeometry.setColor(SimTK::Gray);

    const PhysicalFrame* prevBody = &ground;
    for (int i = 0; i < numLinks; ++i) {
        const std::string istr = std::to_string(i);
        auto* bi = new OpenSim::Body("b" + istr, 1, Vec3(0), Inertia(1));
        model.addBody(bi);

        // Assume each body is 1 m long.
        auto* ji = new PinJoint("j" + istr, *prevBody, Vec3(0), Vec3(0),
                *bi, Vec3(-1, 0, 0), Vec3(0));
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
