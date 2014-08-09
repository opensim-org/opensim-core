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
 * A class implementing an Ellipsoid joint.  The underlying implementation
 * in Simbody is a MobilizedBody::Ellipsoid.
 *
 * @author Ajay Seth
 * @version 1.0
 */
class OSIMSIMULATION_API EllipsoidJoint : public Joint {
    OpenSim_DECLARE_CONCRETE_OBJECT(EllipsoidJoint, Joint);

private:
    static const int _numMobilities = 3;
//=============================================================================
// DATA
//=============================================================================
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
public:
    // CONSTRUCTION
    EllipsoidJoint();
    // convenience constructor
    EllipsoidJoint(const std::string &name, OpenSim::Body& parent, SimTK::Vec3 locationInParent, SimTK::Vec3 orientationInParent,
                   OpenSim::Body& body, SimTK::Vec3 locationInBody, SimTK::Vec3 orientationInBody,
                   SimTK::Vec3 ellipsoidRadii, bool reverse=false);

    virtual ~EllipsoidJoint();

    int numCoordinates() const override {
        return _numMobilities;
    }

    //Set properties
    void setEllipsoidRadii(const SimTK::Vec3& radii);

    // SCALE
    void scale(const ScaleSet& aScaleSet) override;

protected:
    // ModelComponent interface.
    void addToSystem(SimTK::MultibodySystem& system) const override;
    void initStateFromProperties(SimTK::State& s) const override;
    void setPropertiesFromState(const SimTK::State& state) override;

    // Visual support in SimTK visualizer
    void generateDecorations(
        bool fixed,
        const ModelDisplayHints&                    hints,
        const SimTK::State&                         state,
        SimTK::Array_<SimTK::DecorativeGeometry>&   geometryArray) const;

private:
    void constructProperties();

//=============================================================================
};	// END of class EllipsoidJoint
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_ELLIPSOID_JOINT_H_


