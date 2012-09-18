#ifndef __ContactMesh_h__
#define __ContactMesh_h__
/* -------------------------------------------------------------------------- *
 *                          OpenSim:  ContactMesh.h                           *
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
#include "ContactGeometry.h"

namespace OpenSim {

/**
 * This class represents a polygonal mesh for use in contact modeling.
 *
 * @author Peter Eastman
 */
class OSIMSIMULATION_API ContactMesh : public ContactGeometry {
OpenSim_DECLARE_CONCRETE_OBJECT(ContactMesh, ContactGeometry);

//=============================================================================
// DATA
//=============================================================================
private:
    SimTK::ContactGeometry::TriangleMesh* _geometry;
	PropertyStr _filenameProp;
    std::string& _filename;
public:
//=============================================================================
// METHODS
//=============================================================================
	// CONSTRUCTION
    /**
     * Construct an empty, uninitialized ContactMesh.
     */
    ContactMesh();
	/**
	 * Construct a ContactMesh.
	 *
	 * @param filename     the name of the file to load the mesh from
	 * @param location     the location of the mesh within the Body it is attached to
	 * @param orientation  the orientation of the mesh within the Body it is attached to
	 * @param body         the Body this mesh is attached to
	 */
    ContactMesh(const std::string& filename, const SimTK::Vec3& location, const SimTK::Vec3& orientation, Body& body);
	/**
	 * Construct a ContactMesh.
	 *
	 * @param filename     the name of the file to load the mesh from
	 * @param location     the location of the mesh within the Body it is attached to
	 * @param orientation  the orientation of the mesh within the Body it is attached to
	 * @param body         the Body this mesh is attached to
     * @param name         the name of this object
	 */
    ContactMesh(const std::string& filename, const SimTK::Vec3& location, const SimTK::Vec3& orientation, Body& body, const std::string& name);
	ContactMesh(const ContactMesh& geom);

    #ifndef SWIG
    ContactMesh& operator=(const ContactMesh& source) {
        if (&source != this) {
            Super::operator=(source);
            copyData(source);
        }
        return *this;
    }
    #endif

    void copyData(const ContactMesh& source) {
        _geometry = source._geometry;
        _filename = source._filename;
    }
    SimTK::ContactGeometry createSimTKContactGeometry();

	// ACCESSORS
	/**
	 * Get the name of the file the mesh is loaded from.
	 */
    const std::string& getFilename() const;
	/**
	 * Set the name of the file to load the mesh from.
	 */
    void setFilename(const std::string& filename);
private:
    // INITIALIZATION
	void setNull();
	void setupProperties();
    /**
     * Load the mesh from disk.
     */
    void loadMesh(const std::string& filename);

//=============================================================================
};	// END of class ContactMesh
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __ContactMesh_h__
