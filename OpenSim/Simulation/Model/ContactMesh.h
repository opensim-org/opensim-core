#ifndef __ContactMesh_h__
#define __ContactMesh_h__
// ContactMesh.h
// Author: Peter Eastman
/*
 * Copyright (c) 2009 Stanford University. All rights reserved. 
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
