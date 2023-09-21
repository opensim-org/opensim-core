#ifndef OPENSIM_MODELFACTORY_H
#define OPENSIM_MODELFACTORY_H
/* -------------------------------------------------------------------------- *
 * OpenSim: ModelFactory.h                                                    *
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

#include "osimActuatorsDLL.h"
#include <OpenSim/Simulation/Model/FunctionBasedPath.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Actuators/ModelProcessor.h>

namespace OpenSim {

/// This class provides utilities for creating OpenSim models.
class OSIMACTUATORS_API ModelFactory {
public:
    /// @name Create a model
    /// @{

    /// Create a pendulum with the provided number of links.
    /// For each link, there is a body `/bodyset/b#` (where `#` is the link
    /// index starting at 0), a PinJoint `/jointset/j#` with coordinate
    /// `/jointset/j#/q#`, a CoordinateActuator `/tau#`, a Marker
    /// `/markerset/marker#` at the origin of the link's body, and a
    /// PhysicalOffsetFrame \c /b\#center at the center of the link.
    static Model createNLinkPendulum(int numLinks);
    /// This is a convenience for `createNLinkPendulum(1)`.
    static Model createPendulum() { return createNLinkPendulum(1); }
    /// This is a convenience for `createNLinkPendulum(2)`.
    static Model createDoublePendulum() { return createNLinkPendulum(2); }
    /// This model contains:
    /// - 1 body: mass 1.0 kg, `/bodyset/body`.
    /// - 1 joint: SliderJoint along x axis, `/jointset/slider`, with
    ///            coordinate `/jointset/slider/position`.
    /// - 1 actuator: CoordinateActuator, controls [-10, 10], `/actuator`.
    /// Gravity is default; that is, (0, -g, 0).
    static Model createSlidingPointMass();
    /// This model contains:
    /// - 2 bodies: a massless body "intermed", and "body" with mass 1.
    /// - 2 slider joints: "tx" and "ty" (coordinates "tx" and "ty").
    /// - 2 coordinate actuators: "force_x" and "force_y".
    /// Gravity is default; that is, (0, -g, 0).
    static Model createPlanarPointMass();


    /// @}

    /// @name Modify a Model
    /// @{

    /// Replace muscles in a model with a PathActuator of the same path,
    /// optimal force, and min/max control defaults.
    /// @note This only replaces muscles within the model's ForceSet.
    static void replaceMusclesWithPathActuators(Model& model);

    /// Remove muscles from the model.
    /// @note This only removes muscles within the model's ForceSet.
    static void removeMuscles(Model& model);

    /// Replace a joint in the model with a WeldJoint.
    /// @note This assumes the joint is in the JointSet and that the joint's
    ///       connectees are PhysicalOffsetFrames.
    static void replaceJointWithWeldJoint(
            Model& model, const std::string& jointName);

    /// Add CoordinateActuator%s for each unconstrained coordinate (e.g.,
    /// `! Coordinate::isConstrained()`) in the model, using the provided optimal
    /// force. Increasing the optimal force decreases the required control
    /// signal to generate a given actuation level. The actuators are added to
    /// the model's ForceSet and are named "reserve_<coordinate-path>" with
    /// forward slashes converted to underscores. The `bound` argument, if
    /// supplied, sets the min and max controls to `-bound` and `bound`,
    /// respectively.
    /// The fourth (optional) argument
    /// specifies whether or not to skip coordinates that already have
    /// CoordinateActuator%s associated with them (default: true).
    static void createReserveActuators(Model& model, double optimalForce,
            double bound = SimTK::NaN,
            bool skipCoordinatesWithExistingActuators = true);
    
    /// Replace the paths of the forces in the model with the 
    /// FunctionBasedPath%s specified in the file 'pathsFileName'. The file must 
    /// be a Set of FunctionBasedPath%s where the name of each path matches the 
    /// path of a corresponding Force in the model. The path name is appended
    /// with "_path" to avoid name ambiguity in the final model.
    static void replacePathsWithFunctionBasedPaths(Model& model, 
            const std::string& pathsFileName) {
        Set<FunctionBasedPath> pathSet(pathsFileName);
        for (int i = 0; i < pathSet.getSize(); ++i) {
            auto path = pathSet.get(i);
            
            // Get the force component associated with this path.
            OPENSIM_THROW_IF(!model.hasComponent<Force>(path.getName()), 
                    Exception, "Model does not contain a Force at path {}.", 
                    path.getName());
            auto& force = model.updComponent<Force>(path.getName());
            
            // Check that the force has a path property.
            OPENSIM_THROW_IF(
                    !force.hasProperty("path"), Exception,
                    "Force {} does not have a path property.", path.getName());
            
            // Update the path.
            path.setName(fmt::format("{}_path", force.getName()));
            force.updProperty<AbstractPath>("path").setValue(path);
        }
        model.finalizeFromProperties();
        model.finalizeConnections();
    }
};

} // namespace OpenSim

#endif // OPENSIM_MODELFACTORY_H
