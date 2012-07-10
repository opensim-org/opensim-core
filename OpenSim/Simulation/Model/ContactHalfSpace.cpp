// ContactHalfSpace.cpp
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

#include "ContactHalfSpace.h"

using namespace SimTK;

namespace OpenSim {

ContactHalfSpace::ContactHalfSpace() :
    ContactGeometry()
{
    setNull();
}

ContactHalfSpace::ContactHalfSpace(const SimTK::Vec3& location, const SimTK::Vec3& orientation, Body& body) :
    ContactGeometry(location, orientation, body)
{
    setNull();
}

ContactHalfSpace::ContactHalfSpace(const SimTK::Vec3& location, const SimTK::Vec3& orientation, Body& body, const std::string& name) :
    ContactGeometry(location, orientation, body)
{
    setNull();
    setName(name);
}

ContactHalfSpace::ContactHalfSpace(const ContactHalfSpace& geom) :
    ContactGeometry(geom)
{
    setNull();
}

void ContactHalfSpace::setNull()
{
}


SimTK::ContactGeometry ContactHalfSpace::createSimTKContactGeometry()
{
    _displayer.freeGeometry();
	_displayer.addGeometry(new PolyhedralGeometry("unit_plane.obj"));
	return SimTK::ContactGeometry::HalfSpace();
}

} // end of namespace OpenSim
