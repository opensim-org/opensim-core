// ContactMesh.cpp
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

#include <fstream>
#include <OpenSim/Common/IO.h>
#include "ContactMesh.h"
#include "Model.h"

namespace OpenSim {

ContactMesh::ContactMesh() :
    ContactGeometry(),
    _filename(_filenameProp.getValueStr()),
    _geometry(NULL)
{
    setNull();
}

ContactMesh::ContactMesh(const std::string& filename, const SimTK::Vec3& location, const SimTK::Vec3& orientation, Body& body) :
    ContactGeometry(location, orientation, body),
    _filename(_filenameProp.getValueStr()),
    _geometry(NULL)
{
	setNull();
	setupProperties();
    setFilename(filename);
	if (filename != ""){
		std::ifstream file;
		file.open(filename.c_str());
		if (file.fail())
			throw Exception("Error loading mesh file: "+filename+". The file should exist in same folder with model.\n Model loading is aborted.");
		SimTK::PolygonalMesh mesh;
		mesh.loadObjFile(file);
		file.close();
		_geometry = new SimTK::ContactGeometry::TriangleMesh(mesh);
	}
}

ContactMesh::ContactMesh(const std::string& filename, const SimTK::Vec3& location, const SimTK::Vec3& orientation, Body& body, const std::string& name) :
    ContactGeometry(location, orientation, body),
    _filename(_filenameProp.getValueStr()),
    _geometry(NULL)
{
	setNull();
	setupProperties();
    setFilename(filename);
    setName(name);
	if (filename != ""){
		std::ifstream file;
		file.open(filename.c_str());
		if (file.fail())
			throw Exception("Error loading mesh file: "+filename+". The file should exist in same folder with model.\n Loading is aborted.");
		file.close();
	}
}

ContactMesh::ContactMesh(const ContactMesh& geom) :
    ContactGeometry(geom),
    _filename(_filenameProp.getValueStr()),
    _geometry(NULL)
{
	setNull();
	setupProperties();
	_filename = geom._filename;
}

void ContactMesh::setNull()
{
    setType("ContactMesh");
}

void ContactMesh::setupProperties()
{
	_filenameProp.setName("filename");
	_propertySet.append(&_filenameProp);
}

Object* ContactMesh::copy() const
{
	ContactMesh* copy = new ContactMesh(*this);
	return copy;
}

const std::string& ContactMesh::getFilename() const
{
    return _filename;
}

void ContactMesh::setFilename(const std::string& filename)
{
    _filename = filename;
    if (_geometry != NULL)
        delete _geometry;
    _geometry = NULL;
}

void ContactMesh::loadMesh(const std::string& filename)
{
	if (_geometry==NULL){
		SimTK::PolygonalMesh mesh;
		std::ifstream file;
		assert (_model);
		const std::string& savedCwd = IO::getCwd();
		bool restoreDirectory = false;
		if ((_model->getInputFileName()!="") && (_model->getInputFileName()!="Unassigned")) {
			std::string parentDirectory = IO::getParentDirectory(_model->getInputFileName());
			IO::chDir(parentDirectory);
			restoreDirectory=true;
		}
		file.open(filename.c_str());
		if (file.fail()){
			if (restoreDirectory) IO::chDir(savedCwd);
			throw Exception("Error loading mesh file: "+filename+". The file should exist in same folder with model.\n Loading is aborted.");
		}
		mesh.loadObjFile(file);
		file.close();
		if (restoreDirectory) IO::chDir(savedCwd);
		_geometry = new SimTK::ContactGeometry::TriangleMesh(mesh);
	}
	_displayer.addGeometry(new PolyhedralGeometry(filename));

}

SimTK::ContactGeometry ContactMesh::createSimTKContactGeometry()
{
    if (_geometry == NULL)
        loadMesh(_filename);
    return *_geometry;
}

} // end of namespace OpenSim
