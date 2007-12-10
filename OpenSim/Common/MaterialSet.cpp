// MaterialSet.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c)  2005, Stanford University. All rights reserved. 
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

/*  
 * Author:  
 */

#include "MaterialSet.h"
#include "Material.h"




using namespace OpenSim;
using namespace std;


//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * No-arg constructor used for registration.
 *
 */
MaterialSet::MaterialSet():
Set<Material>()
{
	setNull();

	// TYPE
	setType("MaterialSet");
	setName("");
}

//_____________________________________________________________________________
/**
 * Constructor from an xml file passed in by Name.
 *
 */
MaterialSet::MaterialSet(const string &aFileName):
Set<Material>(aFileName, false)
{
	// NULL STATES
	setNull();

	updateFromXMLNode();
}
//_____________________________________________________________________________
/**
 * Copy constructor
 *
 */
MaterialSet::MaterialSet(const MaterialSet &aMaterialSet):
Set<Material>(aMaterialSet)
{
	setNull();

	// ASSIGN
	*this = aMaterialSet;
}
//_____________________________________________________________________________
/**
 * Destructor
 *
 */
MaterialSet::~MaterialSet()
{
}
//_____________________________________________________________________________
/**
 * Set all member variables to their null or default values.
 */
void MaterialSet::
setNull()
{
}
//_____________________________________________________________________________
/**
 * Serialization support.
 */
void MaterialSet::
setupProperties()
{
}
//_____________________________________________________________________________
/**
 * Add a material
 *
 */
bool MaterialSet::
addMaterial(Material& aMaterial)
{
	if(materialExists(aMaterial.getName()))
		return(false);

	append(&aMaterial);
	return(true);
}

//_____________________________________________________________________________
/**
 * Modify/Update a material
 *
 */
bool MaterialSet::
updateMaterial(Material& materialToUpdate)
{
	if (!materialExists(materialToUpdate.getName()))
		return false;

	removeMaterial(materialToUpdate);
	addMaterial(materialToUpdate);

	return true;
}
//_____________________________________________________________________________
/**
 * Remove a material
 *
 */
bool MaterialSet::
removeMaterial(Material& materialToRemove)
{
	if (!materialExists(materialToRemove.getName()))
		return false;

	bool found = false;
	for (int i=0; i < getSize() && !found; i++){
		Material &currentMaterial = *(get(i));
		if (currentMaterial==materialToRemove){
			found= true;
			remove(i);
			break;
		}
	}
	return found;
}
//_____________________________________________________________________________
/**
 * Check if a metrial exists
 *
 */
bool MaterialSet::
materialExists(const string &materialName) const
{
	bool exists = false;
	for (int i=0; i < getSize(); i++){
		Material &currentMaterial = *(get(i));
		if (currentMaterial.getName() == materialName){
			exists= true;
			break;
		}
	}
	return exists;
}
//_____________________________________________________________________________
/**
 * Gets the number of materials maintained by the material manager
 *
 */
int MaterialSet::
getNumberOfMaterials() const
{
	return getSize();
}

//_____________________________________________________________________________
/**
 * Retrieve a material based on index 
 * if material is not found then a default material as defined by Material::getDefaultMaterial
 * is returned. This allows the caller to reliably assume that it has a const ref to a material 
 */
const Material &MaterialSet::
getMaterialByIndex(int idx) const
{
	return (*get(idx));
}

//_____________________________________________________________________________
/**
 * Retrieve a material based on name 
 * if material is not found then a default material as defined by Material::getDefaultMaterial
 * is returned. This allows the caller to reliably assume that it has a const ref to a material
 *
 */
const Material &MaterialSet::
getMaterialByName(const char *materialName) const
{
	for (int i=0; i < getSize(); i++){
		Material &currentMaterial = *(get(i));
		if (currentMaterial.getName() == materialName){
			return currentMaterial;
		}
	}
	// If material was not found return default
	return(Material::GetDefaultMaterial());
}
