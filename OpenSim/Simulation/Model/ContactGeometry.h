#ifndef __ContactGeometry_h__
#define __ContactGeometry_h__
/* -------------------------------------------------------------------------- *
 *                        OpenSim:  ContactGeometry.h                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
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
#include "OpenSim/Common/Object.h"
#include "OpenSim/Simulation/SimbodyEngine/Body.h"
#include "OpenSim/Simulation/Model/ModelComponent.h"
#include <SimTKsimbody.h>

namespace OpenSim {

class ScaleSet;

/** This class represents the physical shape of an object for use in contact
 * modeling.  It is an abstract class, with subclasses for particular geometric
 * representations.
 *
 * @author Peter Eastman
 */
class OSIMSIMULATION_API ContactGeometry : public ModelComponent {
OpenSim_DECLARE_ABSTRACT_OBJECT(ContactGeometry, ModelComponent);

public:

//=============================================================================
// PROPERTIES
//=============================================================================
    // TODO OpenSim_DECLARE_PROPERTY(body_name, std::string,
    // TODO     "Body name to connect the contact geometry to");

    // TODO OpenSim_DECLARE_PROPERTY(location, SimTK::Vec3,
    // TODO     "Location of geometry center in the body frame");

    // TODO OpenSim_DECLARE_PROPERTY(orientation, SimTK::Vec3,
    // TODO     "Orientation of geometry in the body frame");

    OpenSim_DECLARE_PROPERTY(display_preference, int,
        "0:Hide 1:Wire 3:Flat 4:Shaded");

    OpenSim_DECLARE_LIST_PROPERTY_SIZE(color, double, 3,
        "Display Color to apply to the contact geometry.");

//=============================================================================
// METHODS
//=============================================================================
public:
    // CONSTRUCTION
    /** Construct an empty ContactGeometry. */
    ContactGeometry();

    // TODO add constructor that just takes a frame.
    /**
     * Backwards-compatible convenience constructor. This constructor will
     * create a new intermediate PhysicalOffsetFrame using the provided
     * location and orientation, and add that frame as a subcomponent of this
     * component with the name `frame.getName() + "_offset"`.
     *
     * @param location     the location of the geometry expressed in `frame`
     * @param orientation  the orientation of the geometry expressed in `frame`
     *                     as XYZ body-fixed Euler angles.
     * @param frame        the PhysicalFrame this geometry is attached to
     */
    ContactGeometry(const SimTK::Vec3& location,
                    const SimTK::Vec3& orientation,
                    PhysicalFrame& frame); // TODO this arg must be const.


    // ACCESSORS
    // TODO /**
    // TODO  * Get the location of the geometry within the Body it is attached to.
    // TODO  */
    // TODO const SimTK::Vec3& getLocation() const;
    // TODO /**
    // TODO  * %Set the location of the geometry within the Body it is attached to.
    // TODO  */
    // TODO void setLocation(const SimTK::Vec3& location);
    // TODO /**
    // TODO  * Get the orientation of the geometry within the Body it is attached to.
    // TODO  */
    // TODO const SimTK::Vec3& getOrientation() const;
    // TODO /**
    // TODO  * %Set the orientation of the geometry within the Body it is attached to.
    // TODO  */
    // TODO void setOrientation(const SimTK::Vec3& orientation);
#ifndef SWIG
    /**
     * Get the PhysicalFrame this geometry is attached to.
     */
    const PhysicalFrame& getFrame() const;
#endif
    /**
     * %Set the PhysicalFrame this geometry is attached to.
     */
    void setFrame(PhysicalFrame& body);
    /**
     * Get the path name of the PhysicalFrame this geometry is attached to.
     */
    const std::string& getFrameName() const;
    /**
     * %Set the path name (relative or absolute) of the PhysicalFrame this
     * geometry is attached to.
     */
    void setFrameName(const std::string& name);
    /**
     * Get the display_preference of this geometry.
     */
    const int getDisplayPreference();
    /**
     * %Set the display_preference of this geometry.
     */
    void setDisplayPreference(const int dispPref);
    /**
     * Create a new SimTK::ContactGeometry based on this object.
     */
    virtual SimTK::ContactGeometry createSimTKContactGeometry() = 0;
    /**
     * Get a Transform representing the position and orientation of the geometry
     * within the Body it is attached to (*not* the "frame" that the geometry
     * is connected to).
     */
    // TODO rename to findTransform (for consistency).
    SimTK::Transform getTransform() const;

    /**
    * Scale a ContactGeometry based on XYZ scale factors for the bodies.
    * 
    * @param aScaleSet Set of XYZ scale factors for the bodies.
    */
    virtual void scale(const ScaleSet& aScaleSet);

    // Override this method if geometry changes/deforms
    virtual void updateGeometry() {};

    /**
     * TODO
     * Returns a heap-allocated %ContactGeometry that contains a
     * PhysicalOffsetFrame as a subcomponent named `frame.getName() + "_offset".
     * This new frame's parent is `frame` and it is offset by the given
     * `location` and `orientation`.
     */
    static void setFrameWithOffset(const SimTK::Vec3& location,
                                   const SimTK::Vec3& orientation,
                                   PhysicalFrame& frame,
                                   ContactGeometry& geom); // TODO const

    /** @name Deprecated */
    // @{
#ifndef SWIG
    /** <b>(Deprecated)</b> Use getFrame() instead.
     * Get the Body this geometry is attached to.
     */
    DEPRECATED_14("use getFrame() instead")
    const PhysicalFrame& getBody() const;
#endif
    // TODO /** <b>(Deprecated)</b> Use updFrame() instead.
    // TODO  * Get a writeable reference to the Body this geometry is attached to.
    // TODO  */
    // TODO DEPRECATED_14("use updFrame() instead")
    // TODO OpenSim::PhysicalFrame& updBody();
    /** <b>(Deprecated)</b> Use setFrame() instead.
     * %Set the Body this geometry is attached to.
     */
    DEPRECATED_14("use setFrame() instead")
    void setBody(PhysicalFrame& body);
    /** <b>(Deprecated)</b> Use getFrameName() instead.
     * Get the name of the Body this geometry is attached to.
     */
    DEPRECATED_14("use getFrameName() instead")
    const std::string& getBodyName() const;
    /** <b>(Deprecated)</b> Use setFrameName() instead.
     * %Set the name of the Body this geometry is attached to.
     */
    DEPRECATED_14("use setFrameName() instead")
    void setBodyName(const std::string& name);
    // @}

protected:
    // ModelComponent interface
    // TODO void extendConnectToModel(Model& aModel) override;

    void updateFromXMLNode(SimTK::Xml::Element& node, int versionNumber)
        override;

private:
    // INITIALIZATION
    void setNull();
    void constructProperties() override;
    void constructConnectors() override;

//=============================================================================
// DATA
//=============================================================================

protected:
    // TODO SimTK::ReferencePtr<PhysicalFrame> _body;

//=============================================================================
};  // END of class ContactGeometry
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __ContactGeometry_h__
