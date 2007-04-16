// MaterialManager.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c) 2005, Stanford University. All rights reserved. 
* Redistribution and use in source and binary forms, with or without 
* modification, are permitted provided that the following conditions
* are met: 
*  - Redistributions of source code must retain the above copyright 
*    notice, this list of conditions and the following disclaimer. 
*  - Redistributions in binary form must reproduce the above copyright 
*    notice, this list of conditions and the following disclaimer in the 
*    documentation and/or other materials provided with the distribution. 
*  - Neither the name of the Stanford University nor the names of its 
*    contributors may be used to endorse or promote products derived 
*    from this software without specific prior written permission. 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN 
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
* POSSIBILITY OF SUCH DAMAGE. 
*/

/*  
 * Author:  
 */

#include "PropertyObjArray.h"
#include "MaterialManager.h"
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
MaterialManager::MaterialManager()
{
	setNull();

	// TYPE
	setType("MaterialManager");
	setName("");
}

//_____________________________________________________________________________
/**
 * Constructor from an xml file passed in by Name.
 *
 */
MaterialManager::MaterialManager(const string &aFileName):
Object(aFileName, false)
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
MaterialManager::MaterialManager(const MaterialManager &aMaterialManager):
Object(aMaterialManager)
{
	setNull();

	// ASSIGN
	*this = aMaterialManager;
}
//_____________________________________________________________________________
/**
 * Another incarnation of copy that's virtual.
 *
 * @param aElement XMLnode to construct MaterialManager from.
 */

Object *MaterialManager::
copy() const
{
	return(new MaterialManager(*this));
}
//_____________________________________________________________________________
/**
 * Destructor
 *
 */
MaterialManager::~MaterialManager()
{
}
//_____________________________________________________________________________
/**
 * Set all member variables to their null or default values.
 */
void MaterialManager::
setNull()
{
	_availableMaterials=NULL;
	setupSerializedMembers();
}
//_____________________________________________________________________________
/**
 * Serialization support.
 */
void MaterialManager::
setupSerializedMembers()
{
	_propertySet.append( new PropertyObjArray("Materials") );
	_availableMaterials = (ArrayPtrs<Material> *)
		&_propertySet.get("Materials")->getValueObjArray();

}
//-----------------------------------------------------------------------------
// ASSIGNMENT
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Assign this object to the values of another.
 *
 * @return Reference to this object.
 */
MaterialManager& MaterialManager::
operator=(const MaterialManager &aObject)
{
	// BASE CLASS
	Object::operator=(aObject);

	// Class Members
	*_availableMaterials = (*aObject._availableMaterials);
	return(*this);
}
//_____________________________________________________________________________
/**
 * Add a material
 *
 */
bool MaterialManager::
addMaterial(Material& aMaterial)
{
	if(materialExists(aMaterial.getName()))
		return(false);

	_availableMaterials->append(&aMaterial);
	return(true);
}

//_____________________________________________________________________________
/**
 * Modify/Update a material
 *
 */
bool MaterialManager::
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
bool MaterialManager::
removeMaterial(Material& materialToRemove)
{
	if (!materialExists(materialToRemove.getName()))
		return false;

	bool found = false;
	for (int i=0; i < _availableMaterials->getSize() && !found; i++){
		Material &currentMaterial = *(*_availableMaterials)[i];
		if (currentMaterial==materialToRemove){
			found= true;
			_availableMaterials->remove(i);
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
bool MaterialManager::
materialExists(const string &materialName) const
{
	bool exists = false;
	for (int i=0; i < _availableMaterials->getSize(); i++){
		Material &currentMaterial = *(*_availableMaterials)[i];
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
int MaterialManager::
getNumberOfMaterials() const
{
	return _availableMaterials->getSize();
}

//_____________________________________________________________________________
/**
 * Retrieve a material based on index 
 * if material is not found then a default material as defined by Material::getDefaultMaterial
 * is returned. This allows the caller to reliably assume that it has a const ref to a material 
 */
const Material &MaterialManager::
getMaterialByIndex(int idx) const
{
	const Material &retMaterial = Material::GetDefaultMaterial();
	if(idx < _availableMaterials->getSize())
		return( *(*_availableMaterials)[idx] );

	return retMaterial;
}

//_____________________________________________________________________________
/**
 * Retrieve a material based on name 
 * if material is not found then a default material as defined by Material::getDefaultMaterial
 * is returned. This allows the caller to reliably assume that it has a const ref to a material
 *
 */
const Material &MaterialManager::
getMaterialByName(const char *materialName) const
{
	for (int i=0; i < _availableMaterials->getSize(); i++){
		Material &currentMaterial = *(*_availableMaterials)[i];
		if (currentMaterial.getName() == materialName){
			return currentMaterial;
		}
	}
	// If material was not found return default
	return(Material::GetDefaultMaterial());
}
