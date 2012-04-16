#ifndef __ContactHalfSpace_h__
#define __ContactHalfSpace_h__
// ContactHalfSpace.h
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
 * This class represents a half space (that is, everything to one side of an infinite plane)
 * for use in contact modeling.  In its local coordinate system, all points for which x>0 are
 * considered to be inside the geometry.  Its location and orientation properties can be used
 * to move and rotateit to represent other half spaces.
 *
 * @author Peter Eastman
 */
class OSIMSIMULATION_API ContactHalfSpace : public ContactGeometry {
OpenSim_DECLARE_CONCRETE_OBJECT(ContactHalfSpace, ContactGeometry);

public:
//=============================================================================
// METHODS
//=============================================================================
    // CONSTRUCTION
    /**
     * Construct an empty, uninitialized ContactHalfSpace.
     */
    ContactHalfSpace();
    /**
     * Construct a ContactHalfSpace.  All points in its local coordinate system for which
     * x>0 are considered to be inside the geometry.
     *
	 * @param location     the location of the geometry within the Body it is attached to
	 * @param transform    the location and orientation of the half space within the Body it is attached to
     * @param body         the Body this half space is attached to
     */
    ContactHalfSpace(const SimTK::Vec3& location, const SimTK::Vec3& orientation, Body& body);
    /**
     * Construct a ContactHalfSpace.  All points in its local coordinate system for which
     * x>0 are considered to be inside the geometry.
     *
	 * @param location     the location of the geometry within the Body it is attached to
	 * @param transform    the location and orientation of the half space within the Body it is attached to
     * @param body         the Body this half space is attached to
     * @param name         the name of this object
     */
    ContactHalfSpace(const SimTK::Vec3& location, const SimTK::Vec3& orientation, Body& body, const std::string& name);
	ContactHalfSpace(const ContactHalfSpace& geom);

    SimTK::ContactGeometry createSimTKContactGeometry();
private:
    // INITIALIZATION
	void setNull();

//=============================================================================
};	// END of class ContactHalfSpace
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __ContactHalfSpace_h__
