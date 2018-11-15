#ifndef OPENSIM_CONTACT_GEOMETRY_H_
#define OPENSIM_CONTACT_GEOMETRY_H_ 
/* -------------------------------------------------------------------------- *
 *                        OpenSim:  ContactGeometry.h                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Peter Eastman                                                   *
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
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include "OpenSim/Simulation/Model/ModelComponent.h"
#include "OpenSim/Simulation/Model/PhysicalFrame.h"
#include "Appearance.h"

namespace OpenSim {

class ScaleSet;

/** This class represents the physical shape of an object for use in contact
 * modeling.  It is an abstract class, with subclasses for particular geometric
 * representations. The geometry is attached to a PhysicalFrame, which is
 * specified using a Socket named "frame".
 *
 * Note that ContactGeometry is not scaled with the Model.
 *
 * @author Peter Eastman
 */
class OSIMSIMULATION_API ContactGeometry : public ModelComponent {
OpenSim_DECLARE_ABSTRACT_OBJECT(ContactGeometry, ModelComponent);

public:

//=============================================================================
// PROPERTIES
//=============================================================================

    OpenSim_DECLARE_PROPERTY(location, SimTK::Vec3,
        "Location of geometry center in the PhysicalFrame.");

    OpenSim_DECLARE_PROPERTY(orientation, SimTK::Vec3,
        "Orientation of geometry in the PhysicalFrame "
        "(body-fixed XYZ Euler angles).");

    // Default display properties e.g. Representation, color, texture, etc.
    OpenSim_DECLARE_UNNAMED_PROPERTY(Appearance,
        "Default appearance for this Geometry");

    OpenSim_DECLARE_SOCKET(frame, PhysicalFrame,
        "The frame to which this geometry is attached.");

//=============================================================================
// METHODS
//=============================================================================
public:
    // CONSTRUCTION
    /** Construct an empty ContactGeometry. */
    ContactGeometry();

    /** This constructor connects this ContactGeometry to the provided `frame`,
     * and uses the default location and orientation (both `Vec3(0)`).
     *
     * @param frame        the PhysicalFrame this geometry is attached to;
     */
    explicit ContactGeometry(const PhysicalFrame& frame);

    /**
     * @param location     the location of the geometry expressed in `frame`
     * @param orientation  the orientation of the geometry expressed in `frame`
     *                     as XYZ body-fixed Euler angles.
     * @param frame        the PhysicalFrame this geometry is attached to;
     *                     this constructor connects this ContactGeometry to
     *                     the provided `frame`
     */
    ContactGeometry(const SimTK::Vec3& location,
                    const SimTK::Vec3& orientation,
                    const PhysicalFrame& frame);


    // ACCESSORS
    /** Get the PhysicalFrame this geometry is attached to. */
    const PhysicalFrame& getFrame() const;
    /** %Set the PhysicalFrame this geometry is attached to. */
    void setFrame(const PhysicalFrame& frame);

    /** Create a new SimTK::ContactGeometry based on this object. */
    virtual SimTK::ContactGeometry createSimTKContactGeometry() const = 0;

    /** Get a Transform representing the position and orientation of the
     * geometry relative to the PhysicalFrame `F` to which this geometry is
     * connected.
     *
     * If you want the transform of this geometry relative to the Frame (or
     * Ground) `B` in which this geometry is fixed, you can use the following
     * code:
     * @code{.cpp}
     * const auto& X_BF = geom.getFrame().findTransformInBaseFrame();
     * const auto X_FP = geom.getTransform();
     * const auto X_BP = X_BF * X_FP;
     * @endcode
     *
     * Prior to OpenSim 4.0, there wwas no intermediate PhysicalFrame `F`, so
     * this method essentially returned `X_BP`. */
    SimTK::Transform getTransform() const;

    /**
    * Scale a ContactGeometry based on XYZ scale factors for the bodies.
    * 
    * @param aScaleSet Set of XYZ scale factors for the bodies.
    */
    virtual void scale(const ScaleSet& aScaleSet);

    /** @name Deprecated */
    // @{
    /** <b>(Deprecated)</b> Use get_location() instead. */
    DEPRECATED_14("use get_location() instead")
    const SimTK::Vec3& getLocation() const;

    /** <b>(Deprecated)</b> Use set_location() instead. */
    DEPRECATED_14("use set_location() instead")
    void setLocation(const SimTK::Vec3& location);

    /** <b>(Deprecated)</b> Use get_orientation() instead. */
    DEPRECATED_14("use get_orientation() instead")
    const SimTK::Vec3& getOrientation() const;

    /** <b>(Deprecated)</b> Use set_orientation() instead. */
    DEPRECATED_14("use set_orientation() instead")
    void setOrientation(const SimTK::Vec3& orientation);

    /** <b>(Deprecated)</b> Use getFrame() instead.
     * Get the Body this geometry is attached to. */
    DEPRECATED_14("use getFrame() instead")
    const PhysicalFrame& getBody() const;

    /** <b>(Deprecated)</b> Use setFrame() instead.
     * %Set the Body this geometry is attached to. */
    DEPRECATED_14("use setFrame() instead")
    void setBody(const PhysicalFrame& body);
    // @}

protected:

    void updateFromXMLNode(SimTK::Xml::Element& node, int versionNumber)
        override;

private:
    // INITIALIZATION
    void setNull();
    void constructProperties();
//=============================================================================
};  // END of class ContactGeometry
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_CONTACT_GEOMETRY_H_ 
