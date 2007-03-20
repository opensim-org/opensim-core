#ifndef _MaterialManager_h_
#define _MaterialManager_h_
// MaterialManager.h
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

#include "osimCommonDLL.h"
#include "Object.h"
#include "Material.h"

namespace OpenSim { 

//=============================================================================
/**
 * A class for representing the list of Materials in a model
 * It maintains a list (eventually a hashTable between names and Materials)
 *
 * @author Ayman Habib
 * @version 1.0
 */
class OSIMCOMMON_API MaterialManager: public Object
{

//=============================================================================
// DATA
//=============================================================================
protected:
	/* serialized data members */
	/** List of Materials in the model */
	ArrayPtrs<Material>		*_availableMaterials;
//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	MaterialManager();
	MaterialManager(const std::string &aFileName);
	MaterialManager(const MaterialManager &aMaterial);
	virtual ~MaterialManager();

	// Other operators to support Object API
	virtual Object* copy() const;
private:
	void setNull();
protected:
	// Serialization support
	virtual void setupSerializedMembers();
	
	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:
#ifndef SWIG
	MaterialManager& operator=(const MaterialManager &aMaterialManager);
#endif

	//--------------------------------------------------------------------------
	// OPERATIONS
	//--------------------------------------------------------------------------
	bool addMaterial(Material &materialToAdd);
	bool updateMaterial(Material &materialToUpdate);
	bool removeMaterial(Material &materialToRemove);
	bool materialExists(const std::string &materialName) const;
	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
	int getNumberOfMaterials() const;
	const Material& getMaterialByIndex(int idx) const;
	const Material& getMaterialByName(const char *materialName) const;

//=============================================================================
};	// END of class MaterialManager

}; //namespace
//=============================================================================
//=============================================================================

#endif //__MaterialManager_h__
