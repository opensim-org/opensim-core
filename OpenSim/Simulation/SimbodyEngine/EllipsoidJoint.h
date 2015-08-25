#ifndef OPENSIM_ELLIPSOID_JOINT_H_
#define OPENSIM_ELLIPSOID_JOINT_H_
/* -------------------------------------------------------------------------- *
 *                         OpenSim:  EllipsoidJoint.h                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Ajay Seth                                                       *
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

//=============================================================================
//=============================================================================
/**

A class implementing a Ellipsoid joint. The underlying implementation
in Simbody is a SimTK::MobilizedBody::Ellipsoid. An Ellipsoid joint provides three
mobilities – coordinated rotation and translation along the surface of an ellipsoid
 fixed to the parent body. The ellipsoid surface is determined by an input Vec3 which
describes the ellipsoid radius. Generalized speeds are equal to the computed angular
velocities (\f$\vec{u} = \vec{\omega}\f$), not a differentiation of
position (\f$\vec{u} \neq \dot{\vec{q}}\f$)

\image html ellipsoid.gif

@author Ajay Seth
*/
class OSIMSIMULATION_API EllipsoidJoint : public Joint {
OpenSim_DECLARE_CONCRETE_OBJECT(EllipsoidJoint, Joint);

    /** Specify the Coordinates of the EllipsoidJoint */
    Coordinate rx{ constructCoordinate(Coordinate::MotionType::Rotational) };
    Coordinate ry{ constructCoordinate(Coordinate::MotionType::Rotational) };
    Coordinate rz{ constructCoordinate(Coordinate::MotionType::Rotational) };

public:
//==============================================================================
// PROPERTIES
//==============================================================================
    /** @name Property declarations
    These are the serializable properties associated with an EllipsoidJoint. **/
    /**@{**/
    OpenSim_DECLARE_PROPERTY(radii_x_y_z, SimTK::Vec3,
        "Radii of the ellipsoid fixed to the parent frame, "
        "specified as a Vec3(rX, rY, rZ).");
    /**@}**/

//=============================================================================
// METHODS
//=============================================================================
    // CONSTRUCTION
    EllipsoidJoint();
    /** Convenience Joint like Constructor */
    EllipsoidJoint( const std::string& name,
                    const std::string& parentName,
                    const std::string& child,
                    const SimTK::Vec3& ellipsoidRadii,
                    bool reverse = false);

    /** Deprecated Joint Constructor*/
    EllipsoidJoint::EllipsoidJoint(const std::string& name,
        const PhysicalFrame& parent,
        const SimTK::Vec3& locationInParent,
        const SimTK::Vec3& orientationInParent,
        const PhysicalFrame& child,
        const SimTK::Vec3& locationInChild,
        const SimTK::Vec3& orientationInChild,
        const SimTK::Vec3& ellipsoidRadii,
        bool reverse=false);

    //Set properties
    void setEllipsoidRadii(const SimTK::Vec3& radii);

    // SCALE
    void scale(const ScaleSet& aScaleSet) override;

protected:
    // ModelComponent interface.
    void extendAddToSystem(SimTK::MultibodySystem& system) const override;
    void extendInitStateFromProperties(SimTK::State& s) const override;
    void extendSetPropertiesFromState(const SimTK::State& state) override;

    // Visual support in SimTK visualizer
    void generateDecorations(
        bool fixed,
        const ModelDisplayHints&                    hints,
        const SimTK::State&                         state,
        SimTK::Array_<SimTK::DecorativeGeometry>&   geometryArray) const override;

private:
    void constructProperties();

//=============================================================================
};  // END of class EllipsoidJoint
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_ELLIPSOID_JOINT_H_
