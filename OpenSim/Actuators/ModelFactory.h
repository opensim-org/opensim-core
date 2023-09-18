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
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/FunctionBasedPath.h>

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
    /// name of a corresponding Force in the model. The path name is appended
    /// with "_path" to avoid name ambiguity in the final model.
    /// @note This checks both the model's ForceSet and the model's components
    /// list for forces matches the names of the FunctionBasedPath%s in the
    /// file.
    template <typename T>
    static void replacePathsWithFunctionBasedPaths(Model& model, 
            const std::string& pathsFileName) {
        Set<FunctionBasedPath> pathSet(pathsFileName);
        for (int i = 0; i < pathSet.getSize(); ++i) {
            auto path = pathSet.get(i);
            const auto& forceName = path.getName();
            const auto componentPath = fmt::format("/{}", forceName);
            const auto forceSetPath = fmt::format("/forceset/{}", forceName);
            OPENSIM_THROW_IF(!model.hasComponent<T>(forceSetPath) && 
                    !model.hasComponent<T>(componentPath), Exception,
                    "Model does not contain a force with name {}.", forceName);
            OPENSIM_THROW_IF(model.hasComponent<T>(forceSetPath) && 
                             model.hasComponent<T>(componentPath), Exception,
                    "Model does not contain a force with name {} at two "
                    "paths: {} and {}", forceName, forceSetPath, componentPath);
            
            // Get the force component.
            const auto forcePath = model.hasComponent<T>(forceSetPath) ? 
                    forceSetPath : componentPath;
            auto& force = model.updComponent<T>(forcePath);
            
            // Check that the force has a path property.
            OPENSIM_THROW_IF(
                    !force.template hasProperty<AbstractPath>(), Exception,
                    "Force {} does not have a path property.", forceName);
            
            // Overwrite the path with the function-based path.
            path.setName(fmt::format("{}_path", forceName)))
            force.set_path(path);
        }
        model.finalizeConnections();
    }
    
    /// @copydoc replacePathsWithFunctionBasedPaths()
    static void replacePathSpringPathsWithFunctionBasedPaths(Model& model, 
            const std::string& pathsFileName) {
        replacePathsWithFunctionBasedPaths<PathSpring>(model, pathsFileName);
    }
    
    /// @copydoc replacePathsWithFunctionBasedPaths()
    /// @note This will replace the paths of all Muscle%s in the model.
    static void replacePathActuatorPathsWithFunctionBasedPaths(Model& model, 
            const std::string& pathsFileName) {
        replacePathsWithFunctionBasedPaths<PathActuator>(model, pathsFileName);
    }
    
    /// @copydoc replacePathsWithFunctionBasedPaths()
    static void replaceMusclePathsWithFunctionBasedPaths(Model& model, 
            const std::string& pathsFileName) {
        replacePathsWithFunctionBasedPaths<Muscle>(model, pathsFileName);
    }
    
    /// @copydoc replacePathsWithFunctionBasedPaths()
    static void replaceLigamentPathsWithFunctionBasedPaths(Model& model, 
            const std::string& pathsFileName) {
        replacePathsWithFunctionBasedPaths<Ligament>(model, pathsFileName);
    }
    
    /// @copydoc replacePathsWithFunctionBasedPaths()
    static void replaceBlankevoort1991LigamentPathsWithFunctionBasedPaths(
            Model& model, const std::string& pathsFileName) {
        replacePathsWithFunctionBasedPaths<Blankevoort1991Ligament>(
                model, pathsFileName);
    }
};

} // namespace OpenSim

#endif // OPENSIM_MODELFACTORY_H
