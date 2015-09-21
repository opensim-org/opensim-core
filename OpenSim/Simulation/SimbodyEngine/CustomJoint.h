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
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
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

// INCLUDE
#include "Joint.h"

namespace OpenSim {

class SpatialTransform;

//==============================================================================
//                              CUSTOM JOINT
//==============================================================================
/**
A class implementing a custom joint.  The underlying implementation in Simbody 
is a SimTK::MobilizedBody::FunctionBased. Custom joints offer a generic joint
representation, which can be used to model both conventional (pins, slider,
universal, etc…) as well as more complex biomechanical joints. The behavior of
a custom joint is specified by its SpatialTransform. A SpatialTransform is com-
prised of 6 TransformAxes (3 rotations and 3 translations) that define the
spatial position of Child in Parent as a function of coordinates. Each transform
axis has a function of joint coordinates that describes the motion about or along
the transform axis. The order of the spatial transform is fixed with rotations
first followed by translations. Subsequently, coupled motion (i.e., describing
motion of two degrees of freedom as a function of one coordinate) is handled by
transform axis functions that depend on the same coordinate(s).

@author Ajay Seth, Frank C. Anderson
*/
class OSIMSIMULATION_API CustomJoint : public Joint {
OpenSim_DECLARE_CONCRETE_OBJECT(CustomJoint, Joint);
public:
//==============================================================================
// PROPERTIES
//==============================================================================

    /** Spatial transform defining how the child body moves with respect
    to the parent body as a function of the generalized coordinates.
    Motion over 6 (independent) spatial axes must be defined. */
    OpenSim_DECLARE_UNNAMED_PROPERTY(SpatialTransform,
        "Defines how the child body moves with respect to the parent as "
        "a function of the generalized coordinates.");

//==============================================================================
// PUBLIC METHODS
//==============================================================================
    // CONSTRUCTION
    /** Default Constructor */
    CustomJoint();

    /** Construct joint with supplied coordinates and transform axes */
    CustomJoint( const std::string &name,
                 const std::string& parentName,
                 const std::string& childName,
                 SpatialTransform& aSpatialTransform,
                 bool reverse=false);

    /** Joint constructor with explicit parent and child offsets in terms of
        their location and orientation. */
    CustomJoint(const std::string& name,
        const PhysicalFrame& parent,
        const SimTK::Vec3& locationInParent,
        const SimTK::Vec3& orientationInParent,
        const PhysicalFrame& child,
        const SimTK::Vec3& locationInChild,
        const SimTK::Vec3& orientationInChild,
        SpatialTransform& aSpatialTransform,
        bool reverse=false);


    // default destructor, copy constructor, copy assignment

    // Get and Set Transforms
    const SpatialTransform& getSpatialTransform() const
    {   return get_SpatialTransform(); }
    SpatialTransform& updSpatialTransform()
    {   return upd_SpatialTransform(); }

    // SCALE
    void scale(const ScaleSet& aScaleSet) override;

    /** Override of the default implementation to account for versioning. */
    void updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber=-1)
        override;

private:
    // ModelComponent extension interface
    void extendFinalizeFromProperties() override;
    void extendConnectToModel(Model& aModel) override;
    void extendAddToSystem(SimTK::MultibodySystem& system) const override;

    void constructProperties() override;

    // Construct coordinates according to the SpatialTransform of the CustomJoint
    void constructCoordinates();

    template <typename T>
    T createMobilizedBody(SimTK::MobilizedBody& inboard,
        const SimTK::Transform& inboardTransform,
        const SimTK::Body& outboard,
        const SimTK::Transform& outboardTransform,
        int& startingCoorinateIndex) const {};

//==============================================================================
};  // END of class CustomJoint
//==============================================================================
//==============================================================================
template <>
SimTK::MobilizedBody::FunctionBased
    CustomJoint::createMobilizedBody<SimTK::MobilizedBody::FunctionBased>(
                            SimTK::MobilizedBody& inboard,
                            const SimTK::Transform& inboardTransform,
                            const SimTK::Body& outboard,
                            const SimTK::Transform& outboardTransform,
                            int& startingCoorinateIndex) const;

} // end of namespace OpenSim

#endif // OPENSIM_CUSTOM_JOINT_H_
