#ifndef OPENSIM_CONTACT_MESH_H_
#define OPENSIM_CONTACT_MESH_H_
/* -------------------------------------------------------------------------- *
 *                          OpenSim:  ContactMesh.h                           *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2025 Stanford University and the Authors                *
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

 #include "ContactGeometry.h"
namespace OpenSim {

/**
 * \section ContactMesh
 * This class represents a polygonal mesh for use in contact modeling.
 *
 * A `SimTK::ContactGeometry::TriangleMesh` is constructed when
 * `createSimTKContactGeometry()` is called.
 *
 * @author Peter Eastman
 */
class OSIMSIMULATION_API ContactMesh : public ContactGeometry {
OpenSim_DECLARE_CONCRETE_OBJECT(ContactMesh, ContactGeometry);

public:
//=============================================================================
// PROPERTIES
//=============================================================================
    OpenSim_DECLARE_PROPERTY(filename, std::string,
            "Path to a mesh geometry file (supports .obj, .stl, .vtp). "
            "The mesh should be closed and water-tight.");

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
     * @param location     the location of the mesh within the PhysicalFrame it
     *                     is attached to
     * @param orientation  the orientation of the mesh within the PhysicalFrame
     *                     it is attached to
     * @param frame        the PhysicalFrame this mesh is attached to
     */
    ContactMesh(const std::string& filename, const SimTK::Vec3& location,
            const SimTK::Vec3& orientation, const PhysicalFrame& frame);

    /**
     * Construct a ContactMesh.
     *
     * @param filename     the name of the file to load the mesh from
     * @param location     the location of the mesh within the PhysicalFrame it
     *                     is attached to
     * @param orientation  the orientation of the mesh within the PhysicalFrame
     *                     it is attached to
     * @param frame        the PhysicalFrame this mesh is attached to
     * @param name         the name of this object
     */
    ContactMesh(const std::string& filename, const SimTK::Vec3& location,
            const SimTK::Vec3& orientation, const PhysicalFrame& frame,
            const std::string& name);

    // ACCESSORS
    /**
     * Get the name of the file the mesh is loaded from.
     */
    const std::string& getFilename() const;

    /**
     * %Set the name of the file to load the mesh from.
     */
    void setFilename(const std::string& filename);

    /** @name Visualization */
    // @{
    void generateDecorations(bool fixed, const ModelDisplayHints& hints,
        const SimTK::State& s,
        SimTK::Array_<SimTK::DecorativeGeometry>& geometry) const override;
    // @}

private:
    // INITIALIZATION
    void setNull();
    void constructProperties();
    void extendFinalizeFromProperties() override;

    // CONTACT GEOMETRY INTERFACE
    SimTK::ContactGeometry createSimTKContactGeometryImpl() const override;

    // LOAD MESH
    SimTK::ContactGeometry::TriangleMesh* loadMesh(
        const std::string& filename) const;

    // DATA
    mutable SimTK::ResetOnCopy<std::unique_ptr<SimTK::ContactGeometry::TriangleMesh>>
        _geometry;
    mutable SimTK::ResetOnCopy<std::unique_ptr<SimTK::DecorativeMesh>>
        _decorativeGeometry;

};

} // namespace OpenSim

#endif // OPENSIM_CONTACT_MESH_H_
