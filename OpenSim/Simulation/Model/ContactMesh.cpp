/* -------------------------------------------------------------------------- *
 *                         OpenSim:  ContactMesh.cpp                          *
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
#include "../simulation.h"


namespace OpenSim {

ContactMesh::ContactMesh() 
{
    setNull();
    constructProperties();
}

ContactMesh::ContactMesh(const std::string& filename,
                         const SimTK::Vec3& location,
                         const SimTK::Vec3& orientation,
                         PhysicalFrame& body) :
    ContactGeometry(location, orientation, body)
{
    setNull();
    constructProperties();
    setFilename(filename);
    if (filename != ""){
        std::ifstream file;
        file.open(filename.c_str());
        if (file.fail())
            throw Exception("Error loading mesh file: "+filename+". The file should exist in same folder with model.\n Model loading is aborted.");
        file.close();
        SimTK::PolygonalMesh mesh;
        mesh.loadFile(filename);
        _geometry.reset(new SimTK::ContactGeometry::TriangleMesh(mesh));
    }
}

ContactMesh::ContactMesh(const std::string& filename,
                         const SimTK::Vec3& location,
                         const SimTK::Vec3& orientation,
                         PhysicalFrame& body,
                         const std::string& name) :
    ContactMesh(filename, location, orientation, body)
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
}

const std::string& ContactMesh::getFilename() const
{
    return get_filename();
}

void ContactMesh::setFilename(const std::string& filename)
{
    set_filename(filename);
    _geometry.reset();
}

void ContactMesh::loadMesh(const std::string& filename)
{
    if (!_geometry){
        SimTK::PolygonalMesh mesh;
        std::ifstream file;
        assert (_model);
        const std::string& savedCwd = IO::getCwd();
        bool restoreDirectory = false;
        if ((_model->getInputFileName()!="")
                && (_model->getInputFileName()!="Unassigned")) {
            std::string parentDirectory = IO::getParentDirectory(
                    _model->getInputFileName());
            IO::chDir(parentDirectory);
            restoreDirectory=true;
        }
        file.open(filename.c_str());
        if (file.fail()){
            if (restoreDirectory) IO::chDir(savedCwd);
            throw Exception("Error loading mesh file: "+filename+". "
                    "The file should exist in same folder with model.\n "
                    "Loading is aborted.");
        }
        file.close();
        mesh.loadFile(filename);
        if (restoreDirectory) IO::chDir(savedCwd);
        _geometry.reset(new SimTK::ContactGeometry::TriangleMesh(mesh));
    }

}

SimTK::ContactGeometry ContactMesh::createSimTKContactGeometry()
{
    if (!_geometry)
        loadMesh(get_filename());
    return *_geometry;
}

} // end of namespace OpenSim
