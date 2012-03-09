// PropertyTable.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c)  2011-12, Stanford University. All rights reserved. 
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

//============================================================================
// INCLUDES
//============================================================================
#include "PropertyTable.h"


using namespace OpenSim;
using namespace std;

//_____________________________________________________________________________
// Default constructor is inline

//_____________________________________________________________________________
// Copy constructor has to clone the source properties.
PropertyTable::PropertyTable(const PropertyTable& source)
{
	replaceProperties(source.properties);
}

//_____________________________________________________________________________
// Destructor must deep-delete the properties since they are owned by the
// PropertyTable.
PropertyTable::~PropertyTable()
{
	deleteProperties();
}

//_____________________________________________________________________________
// Copy assignment has to clone the source properties.
PropertyTable& PropertyTable::operator=(const PropertyTable& source)
{
	replaceProperties(source.properties);
	return *this;
}

//_____________________________________________________________________________
// Equality operator compares each property with the one at the same position.
bool PropertyTable::equals(const PropertyTable& other) const {
    if (getNumProperties() != other.getNumProperties())
        return false;
    for (int i=0; i < getNumProperties(); ++i) {
        if (!(getAbstractPropertyByIndex(i) == other.getAbstractPropertyByIndex(i)))
            return false;
    }
    return true;
}

// Private method to look up the property by name in the map to find its index
// in the property array and return that. If the name isn't there, return -1.
// This private method is reused in the implementation of any method that
// takes a property by name.
int PropertyTable::findPropertyIndex(const std::string& name) const {
    const std::map<std::string, int>::const_iterator 
        it = propertyIndex.find(name);
    return it == propertyIndex.end() ? -1 : it->second;
}

// Private method to replace the existing properties with a deep copy of 
// the source, and update the index map to match.
void PropertyTable::replaceProperties
   (const SimTK::Array_<AbstractProperty*>& source) {
    deleteProperties();
    for (unsigned i=0; i < source.size(); ++i) {
        properties.push_back(source[i]->copy());
        propertyIndex[source[i]->getName()] = i;
    }
}

// Private method to delete all the properties and clear the index.
void PropertyTable::deleteProperties() {
    for (unsigned i=0; i < properties.size(); ++i)
        delete properties[i];
    properties.clear();
    propertyIndex.clear();
}

