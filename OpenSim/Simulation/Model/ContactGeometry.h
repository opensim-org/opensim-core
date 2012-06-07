#ifndef __ContactGeometry_h__
#define __ContactGeometry_h__
// ContactGeometry.h
// Author: Peter Eastman
/*
 * Copyright (c)  2009 Stanford University. All rights reserved. 
* Use of the OpenSim software in source form is permitted provided that the following
* conditions are met:
* 	1. The software is used only for non-commercial research and education. It may not
*     be used in relation to any commercial activity.
* 	2. The software is not distributed or redistributed.  Software distribution is allowed 
*     only through https://simtk.org/home/opensim.
* 	3. Use of the OpenSim software or derivatives must be acknowledged in all publications,
*      presentations, or documents describing work in which OpenSim or derivatives are used.
* 	4. Credits to developers may not be removed from executables
*     created from modifications of the source.
* 	5. Modifications of source code must retain the above copyright notice, this list of
*     conditions and the following disclaimer. 
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
*  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
*  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
*  SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
*  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
*  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR BUSINESS INTERRUPTION) OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
*  WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
// INCLUDE
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include "OpenSim/Common/Object.h"
#include "OpenSim/Simulation/SimbodyEngine/Joint.h"
#include "OpenSim/Simulation/SimbodyEngine/Body.h"
#include "OpenSim/Simulation/Model/ModelComponent.h"
#include <SimTKsimbody.h>

namespace OpenSim {

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
	PropertyStr _bodyNameProp;
    std::string& _bodyName;
	PropertyDblVec3 _locationInBodyProp;
	SimTK::Vec3& _locationInBody;
	PropertyDblVec3 _orientationInBodyProp;
	SimTK::Vec3& _orientationInBody;

	VisibleObject	_displayer;
//=============================================================================
// METHODS
//=============================================================================
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
public:
	ContactGeometry(const ContactGeometry& geom);
	~ContactGeometry();

    #ifndef SWIG
    ContactGeometry& operator=(const ContactGeometry& source) {
        if (&source != this) {
            Super::operator=(source);
            copyData(source);
        }
        return *this;
    }
    #endif

	void copyData(const ContactGeometry& geom);

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
	void setupProperties();
//=============================================================================
};	// END of class ContactGeometry
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __ContactGeometry_h__
