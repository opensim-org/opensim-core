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

/**
 * This class represents the physical shape of an object for use in contact modeling.
 * It is an abstract class, with subclasses for particular geometric representations.
 *
 * @author Peter Eastman
 */
class OSIMSIMULATION_API ContactGeometry : public ModelComponent {
OpenSim_DECLARE_ABSTRACT_OBJECT(ContactGeometry, ModelComponent);

//=============================================================================
// DATA
//=============================================================================
protected:
	Body* _body;
	VisibleObject	_displayer;

//=============================================================================
// METHODS
//=============================================================================
public:
	// CONSTRUCTION
	/**
	 * Construct an empty ContactGeometry.  This constructor is protected, and is used
	 * by subclasses.
	 */
	ContactGeometry();
	/**
	 * Construct a ContactGeometry.  This constructor is protected, and is used
	 * by subclasses.
	 *
	 * @param location     the location of the geometry within the Body it is attached to
	 * @param orientation  the orientation of the geometry within the Body it is attached to
	 * @param body         the Body this geometry is attached to
	 */
    ContactGeometry(const SimTK::Vec3& location, const SimTK::Vec3& orientation, OpenSim::Body& body);


    /** Body name.  **/
    OpenSim_DECLARE_PROPERTY(body_name, std::string,
        "Body name to connect the contact geometry to");

    /** Location.  **/
    OpenSim_DECLARE_PROPERTY(location, SimTK::Vec3,
        "Location of geometry center in the body frame");

    /** Orientation.  **/
    OpenSim_DECLARE_PROPERTY(orientation, SimTK::Vec3,
        "Orientation of geometry in the body frame");

    /** Display Preference to apply to the contact geometry.  **/
    OpenSim_DECLARE_PROPERTY(display_preference, int,
        "Display Pref. 0:Hide 1:Wire 3:Flat 4:Shaded");

    /** Display Color to apply to the contact geometry.  **/
    OpenSim_DECLARE_LIST_PROPERTY_SIZE(color, double, 3,
        "Display Color");


	// ACCESSORS
	/**
	 * Get the location of the geometry within the Body it is attached to.
	 */
	const SimTK::Vec3& getLocation() const;
	/**
	 * Set the location of the geometry within the Body it is attached to.
	 */
	void setLocation(const SimTK::Vec3& location);
	/**
	 * Get the orientation of the geometry within the Body it is attached to.
	 */
	const SimTK::Vec3& getOrientation() const;
	/**
	 * Set the orientation of the geometry within the Body it is attached to.
	 */
	void setOrientation(const SimTK::Vec3& orientation);
#ifndef SWIG
	/**
	 * Get the Body this geometry is attached to.
	 */
	const OpenSim::Body& getBody() const;
#endif
	/**
	 * Get the Body this geometry is attached to.
	 */
	OpenSim::Body& getBody();
	/**
	 * Set the Body this geometry is attached to.
	 */
	void setBody(OpenSim::Body& body);
	/**
	 * Get the name of the Body this geometry is attached to.
	 */
    const std::string& getBodyName();
	/**
	 * Set the name of the Body this geometry is attached to.  This will cause the
     * Body to be set to NULL, then resolved when connectToModel() is called.
	 */
    void setBodyName(const std::string& name);
    /**
	 * Get the display_preference of this geometry.
	 */
    const int getDisplayPreference();
    /**
	 * Set the display_preference of this geometry.
	 */
    void setDisplayPreference(const int dispPref);
	/**
	 * Create a new SimTK::ContactGeometry based on this object.
	 */
    virtual SimTK::ContactGeometry createSimTKContactGeometry() = 0;
    /**
     * Get a Transform representing the position and orientation of the geometry
     * within the Body it is attached to.
     */
    SimTK::Transform getTransform();

	/**
	* Scale a ContactGeometry based on XYZ scale factors for the bodies.
	* 
	* @param aScaleSet Set of XYZ scale factors for the bodies.
	*/
	virtual void scale(const ScaleSet& aScaleSet);

	// Visible Object Support
	virtual const VisibleObject* getDisplayer() const { return &_displayer; };
	virtual VisibleObject* updDisplayer() { return &_displayer; };
	// Override this method if geometry changes/deforms
	virtual void updateGeometry() {};

protected:
	// ModelComponent interface
	void connectToModel(Model& aModel) OVERRIDE_11;

private:
    // INITIALIZATION
	void setNull();
    void constructProperties();
//=============================================================================
};	// END of class ContactGeometry
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __ContactGeometry_h__
