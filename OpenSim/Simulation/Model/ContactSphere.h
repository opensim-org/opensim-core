#ifndef __ContactSphere_h__
#define __ContactSphere_h__
// ContactSphere.h
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
#include "ContactGeometry.h"

namespace OpenSim {

/**
 * This class represents a spherical object for use in contact modeling.
 *
 * @author Peter Eastman
 */
class OSIMSIMULATION_API ContactSphere : public ContactGeometry {
OpenSim_DECLARE_CONCRETE_OBJECT(ContactSphere, ContactGeometry);

//=============================================================================
// DATA
//=============================================================================
protected:
	PropertyDbl _radiusProp;
	double& _radius;

//=============================================================================
// METHODS
//=============================================================================
public:
	// CONSTRUCTION
    /**
     * Construct an empty, uninitialized ContactSphere.
     */
    ContactSphere();
	/**
	 * Construct a ContactSphere.
	 *
	 * @param radius       the radius of the sphere
	 * @param location     the location of the center of the sphere within the Body it is attached to
	 * @param body         the Body this sphere is attached to
	 */
	ContactSphere(double radius, const SimTK::Vec3& location, Body& body);
	/**
	 * Construct a ContactSphere.
	 *
	 * @param radius       the radius of the sphere
	 * @param location     the location of the center of the sphere within the Body it is attached to
	 * @param body         the Body this sphere is attached to
     * @param name         the name of this object
	 */
    ContactSphere(double radius, const SimTK::Vec3& location, Body& body, const std::string& name);
	ContactSphere(const ContactSphere& geom);

    #ifndef SWIG
    ContactSphere& operator=(const ContactSphere& source) {
        if (&source != this) {
            Super::operator=(source);
            copyData(source);
        }
        return *this;
    }
    #endif

    void copyData(const ContactSphere& source) {
        _radius = source._radius;
    }
    SimTK::ContactGeometry createSimTKContactGeometry();

	// ACCESSORS
	/**
	 * Get the radius of the sphere.
	 */
	double getRadius() const;
	/**
	 * Set the radius of the sphere.
	 */
	void setRadius(double radius);
private:
    // INITIALIZATION
	void setNull();
	void setupProperties();

//=============================================================================
};	// END of class ContactSphere
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __ContactSphere_h__
