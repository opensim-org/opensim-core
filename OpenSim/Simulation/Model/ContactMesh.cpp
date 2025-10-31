/* -------------------------------------------------------------------------- *
 *                         OpenSim:  ContactMesh.cpp                          *
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

 #include "ContactMesh.h"

 #include <fstream>
 #include <OpenSim/Common/IO.h>
 #include <OpenSim/Simulation/Model/Model.h>

using namespace OpenSim;

ContactMesh::ContactMesh()
{
    setNull();
    constructProperties();
}

ContactMesh::ContactMesh(const std::string& filename,
                         const SimTK::Vec3& location,
                         const SimTK::Vec3& orientation,
                         const PhysicalFrame& frame) :
    ContactGeometry(location, orientation, frame)
{
    setNull();
    constructProperties();
    setFilename(filename);
    if (filename != ""){
        std::ifstream file;
        file.open(filename.c_str());
        OPENSIM_THROW_IF_FRMOBJ(file.fail(), Exception,
            "Error loading mesh file '{}': The file should exist in same "
            "folder with model. Model loading is aborted.");

        file.close();
        SimTK::PolygonalMesh mesh;
        mesh.loadFile(filename);
        _geometry.reset(new SimTK::ContactGeometry::TriangleMesh(mesh));
        _decorativeGeometry.reset(new SimTK::DecorativeMesh(mesh));
    }
}

ContactMesh::ContactMesh(const std::string& filename,
                         const SimTK::Vec3& location,
                         const SimTK::Vec3& orientation,
                         const PhysicalFrame& frame,
                         const std::string& name) :
    ContactMesh(filename, location, orientation, frame)
{
    setName(name);
}

void ContactMesh::setNull()
{
    setAuthors("Peter Eastman");
}

void ContactMesh::constructProperties()
{
    constructProperty_filename("");
}

void ContactMesh::extendFinalizeFromProperties() {
    _geometry.reset();
    _decorativeGeometry.reset();
}

const std::string& ContactMesh::getFilename() const
{
    return get_filename();
}

void ContactMesh::setFilename(const std::string& filename)
{
    set_filename(filename);
    _geometry.reset();
    _decorativeGeometry.reset();
}

SimTK::ContactGeometry::TriangleMesh*
ContactMesh::loadMesh(const std::string& filename) const
{
    SimTK::PolygonalMesh mesh;
    std::ifstream file;
    assert (_model);

    auto cwd = IO::CwdChanger::noop();
    if ((_model->getInputFileName()!="")
            && (_model->getInputFileName()!="Unassigned")) {
        cwd = IO::CwdChanger::changeToParentOf(_model->getInputFileName());
    }

    file.open(filename.c_str());
    if (file.fail()){
        throw Exception("Error loading mesh file: "+filename+". "
                "The file should exist in same folder with model.\n "
                "Loading is aborted.");
    }
    file.close();
    mesh.loadFile(filename);
    _decorativeGeometry.reset(new SimTK::DecorativeMesh(mesh));
    return new SimTK::ContactGeometry::TriangleMesh(mesh);
}

//=============================================================================
// CONTACT GEOMETRY INTERFACE
//=============================================================================
SimTK::ContactGeometry ContactMesh::createSimTKContactGeometryImpl() const
{
    if (!_geometry)
        _geometry.reset(loadMesh(get_filename()));
    return *_geometry;
}

//=============================================================================
// VISUALIZATION
//=============================================================================
void ContactMesh::generateDecorations(
    bool fixed,
    const ModelDisplayHints& hints,
    const SimTK::State& s,
    SimTK::Array_<SimTK::DecorativeGeometry>& geometry) const
{
    // There is no fixed geometry to generate here.
    if (fixed) { return; }

    // Model-wide hints indicate that contact geometry shouldn't be shown.
    if (!hints.get_show_contact_geometry()) { return; }

    // The decoration has been toggled off by its `Appearance` block.
    if (!get_Appearance().get_visible())  { return; }

    // Guard against the case where the Force was disabled or mesh failed to load.
    if (_decorativeGeometry == nullptr) { return; }

    // B: base Frame (Body or Ground)
    // F: PhysicalFrame that this ContactGeometry is connected to
    // P: the frame defined (relative to F) by the location and orientation
    //    properties.

    const auto& X_BF = getFrame().findTransformInBaseFrame();
    const auto& X_FP = getTransform();
    const auto X_BP = X_BF * X_FP;
    geometry.push_back(SimTK::DecorativeMesh(*_decorativeGeometry)
        .setTransform(X_BP)
        .setRepresentation(get_Appearance().get_representation())
        .setBodyId(getFrame().getMobilizedBodyIndex())
        .setColor(get_Appearance().get_color())
        .setScale(1)
        .setOpacity(get_Appearance().get_opacity()));
}
