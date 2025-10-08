#ifndef OPENSIM_CUSTOM_JOINT_H_
#define OPENSIM_CUSTOM_JOINT_H_
/* -------------------------------------------------------------------------- *
 *                          OpenSim:  CustomJoint.h                           *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2025 Stanford University and the Authors                *
 * Author(s): Frank C. Anderson, Ajay Seth                                    *
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

#include "Joint.h"

namespace OpenSim {

class SpatialTransform;

/**
 * A class implementing a custom joint.
 *
 * The underlying implementation in Simbody is a
 * `SimTK::MobilizedBody::FunctionBased`. `CustomJoint`s offer a generic joint
 * representation, which can be used to model both conventional (pins, slider,
 * universal, etc.) as well as more complex biomechanical joints.
 *
 * The behavior of a `CustomJoint` is specified by its `SpatialTransform`. A
 * `SpatialTransform` is comprised of 6 `TransformAxis` objects (3 rotations and
 * 3 translations) that define the spatial position of the child body in the
 * parent body frame as a function of the joint's coordinates. Each transform
 * axis has a `Function` of joint coordinates that describes the motion about or
 * along the `TransformAxis`. The order of the `SpatialTransform` is fixed with
 * rotations first followed by translations. Subsequently, coupled motion (i.e.,
 * describing motion of two degrees of freedom as a function of one coordinate)
 * is handled by `TransformAxis` functions that depend on the same
 * coordinate(s).
 *
 * @author Ajay Seth, Frank C. Anderson
*/
class OSIMSIMULATION_API CustomJoint : public Joint {
OpenSim_DECLARE_CONCRETE_OBJECT(CustomJoint, Joint);
public:
//==============================================================================
// PROPERTIES
//==============================================================================
    OpenSim_DECLARE_UNNAMED_PROPERTY(SpatialTransform,
        "The spatial transform defining how the child body moves with respect"
        "to the parent as a function of the generalized coordinates.");

//==============================================================================
// METHODS
//==============================================================================
    /**
     * Default constructor.
     */
    CustomJoint();

    /**
     * Construct joint with supplied coordinates and transform axes.
     */
    CustomJoint(const std::string& name,
            const PhysicalFrame& parent,
            const PhysicalFrame& child,
            const SpatialTransform& spatialTransform);

    /**
     * Joint constructor with explicit parent and child offsets in terms of
     * their location and orientation.
     */
    CustomJoint(const std::string& name,
            const PhysicalFrame& parent,
            const SimTK::Vec3& locationInParent,
            const SimTK::Vec3& orientationInParent,
            const PhysicalFrame& child,
            const SimTK::Vec3& locationInChild,
            const SimTK::Vec3& orientationInChild,
            const SpatialTransform& spatialTransform);


    //** @name Accessors */
    // @{
    /**
     * Get the `SpatialTransform` defining the joint motion.
     */
    const SpatialTransform& getSpatialTransform() const;

    /**
     * Update the `SpatialTransform` defining the joint motion.
     */
    SpatialTransform& updSpatialTransform();

    /**
     * Set the `SpatialTransform` defining the joint motion.
     */
    void setSpatialTransform(const SpatialTransform& transform);

    /**
     * Exposes getCoordinate() method defined in base class (overloaded below).
     * @see Joint
     */
    using Joint::getCoordinate;

    /**
     * Exposes updCoordinate() method defined in base class (overloaded below).
     * @see Joint
     */
    using Joint::updCoordinate;

    /**
     * Get a const reference to a Coordinate associated with this Joint.
     */
    const Coordinate& getCoordinate(unsigned idx) const;

    /**
     * Get a writable reference to a Coordinate associated with this Joint.
     */
    Coordinate& updCoordinate(unsigned idx);

    // @}

    //** @name ModelComponent interface */
    // @{
    void extendScale(const SimTK::State& s, const ScaleSet& scaleSet) override;
    // @}

    //** @name Object interface */
    // @{
    void updateFromXMLNode(SimTK::Xml::Element& aNode,
            int versionNumber=-1) override;
    // @}

private:
    // MODEL COMPONENT INTERFACE
    void extendFinalizeFromProperties() override;
    void extendConnectToModel(Model& aModel) override;
    void extendAddToSystem(SimTK::MultibodySystem& system) const override;

    // CONVENIENCE METHODS
    void constructProperties();

    // Construct coordinates according to the SpatialTransform of the
    // CustomJoint.
    void constructCoordinates();

    template <typename T>
    T createMobilizedBody(SimTK::MobilizedBody& inboard,
        const SimTK::Transform& inboardTransform,
        const SimTK::Body& outboard,
        const SimTK::Transform& outboardTransform,
        int startingCoorinateIndex) const {};

};  // class CustomJoint

// Template specialization for `SimTK::MobilizedBody::FunctionBased`.
template <>
SimTK::MobilizedBody::FunctionBased
CustomJoint::createMobilizedBody<SimTK::MobilizedBody::FunctionBased>(
        SimTK::MobilizedBody& inboard, const SimTK::Transform& inboardTransform,
        const SimTK::Body& outboard, const SimTK::Transform& outboardTransform,
        int startingCoordinateIndex) const;

} // namespace OpenSim

#endif // OPENSIM_CUSTOM_JOINT_H_
