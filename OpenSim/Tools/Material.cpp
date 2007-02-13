// Material.cpp
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


//============================================================================
// INCLUDES
//============================================================================
#include "Material.h"
#include "Exception.h"
#include "PropertyDbl.h"
#include "PropertyDblArray.h"



using namespace OpenSim;
using namespace std;

//============================================================================
// CONSTANTS
//============================================================================
Material *Material::_defaultMaterial=0;

//=============================================================================
// CONSTRUCTOR(S)
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
Material::~Material()
{
}

//_____________________________________________________________________________
/**
 * Default constructor.
 */
Material::Material():
_translucency(_propTranslucency.getValueDbl()),
_ambientColor(_propAmbientColor.getValueDblArray()),
_diffuseColor(_propDiffuseColor.getValueDblArray()),
_specularColor(_propSpecularColor.getValueDblArray())
{
	// NULL STATES
	setNull();

	// MEMBER VARIABLES
	setName("");
}

//_____________________________________________________________________________
/**
 * Construct an object from file.
 *
 * The object is constructed from the root element of the XML document.
 * The type of object is the tag name of the XML root element.
 *
 * @param aFileName File name of the document.
 */
Material::Material(const string &aFileName):
Object(aFileName),
_translucency(_propTranslucency.getValueDbl()),
_ambientColor(_propAmbientColor.getValueDblArray()),
_diffuseColor(_propDiffuseColor.getValueDblArray()),
_specularColor(_propSpecularColor.getValueDblArray())
{
	// NULL STATES
	setNull();

	updateFromXMLNode();
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * Copy constructors for all Material's only copy the non-XML variable
 * members of the object; that is, the object's DOMnode and XMLDocument
 * are not copied but set to NULL.  The reason for this is that for the
 * object and all its derived classes to establish the correct connection
 * to the XML document nodes, the the object would need to reconstruct based
 * on the XML document not the values of the object's member variables.
 *
 * There are three proper ways to generate an XML document for an Material:
 *
 * 1) Construction based on XML file (@see Material(const char *aFileName)).
 * In this case, the XML document is created by parsing the XML file.
 *
 * 2) Construction by Material(const XMLDocument *aDocument).
 * This constructor explictly requests construction based on an
 * XML document.  In this way the proper connection between an object's node
 * and the corresponding node within the XML document is established.
 * This constructor is a copy constructor of sorts because all essential
 * Material member variables should be held within the XML document.
 * The advantage of this style of construction is that nodes
 * within the XML document, such as comments that may not have any
 * associated Material member variable, are preserved.
 *
 * 3) A call to generateXMLDocument().
 * This method generates an XML document for the VisibleObject from scratch.
 * Only the essential document nodes are created (that is, nodes that
 * correspond directly to member variables.).
 *
 * @param aObject Object to be copied.
 * @see Material(const XMLDocument *aDocument)
 * @see Material(const char *aFileName)
 * @see generateXMLDocument()
 */
Material::Material(const Material &aObject):
Object(aObject),
_translucency(_propTranslucency.getValueDbl()),
_ambientColor(_propAmbientColor.getValueDblArray()),
_diffuseColor(_propDiffuseColor.getValueDblArray()),
_specularColor(_propSpecularColor.getValueDblArray())
{
	// NULL MEMBER VARIABLES
	setNull();

	// COPY TYPE AND NAME
	*this = aObject;
}
//_____________________________________________________________________________
/**
 * Virtual copy constructor 
 */
Object* Material::
copy() const
{
	return(new Material(*this));
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Set all member variables to their null or default values.
 */
void Material::
setNull()
{
	setType("Material");
	//...
	setupProperties();
}
//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void Material::
setupProperties()
{
   double white[3] = { 1.0, 1.0, 1.0 };

   _propertySet.append( new PropertyDbl("translucency",1.0) );
   _propertySet.append( new PropertyDblArray("ambient",3,white) );
   _propertySet.append( new PropertyDblArray("diffuse",3,white) );
   _propertySet.append( new PropertyDblArray("specular",3,white) );

    _propertySet.append( &_propTranslucency );
	_propTranslucency.setName("translucency");
	_propTranslucency.setValue(1.0);

	_propertySet.append( &_propAmbientColor);
	_propAmbientColor.setName("ambient");
	_propAmbientColor.setValue(3, white);

	_propertySet.append( &_propDiffuseColor);
	_propDiffuseColor.setName("diffuse");
	_propDiffuseColor.setValue(3, white);

	_propertySet.append( &_propSpecularColor );
	_propDiffuseColor.setName("specular");
	_propDiffuseColor.setValue(3, white);

}


//=============================================================================
// OPERATORS
//=============================================================================
//-----------------------------------------------------------------------------
// ASSIGNMENT
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Assign this object to the values of another.
 *
 * @return Reference to this object.
 */
Material& Material::
operator=(const Material &aObject)
{
	// BASE CLASS
	Object::operator=(aObject);

	// Class Members
	_translucency = (aObject._translucency);
	for(int i=0; i < 3; i++){
		_ambientColor[i] = aObject._ambientColor[i];
		_diffuseColor[i] = aObject._diffuseColor[i];
		_specularColor[i] = aObject._specularColor[i];
	}
	return(*this);
}

//-----------------------------------------------------------------------------
// EQUALITY
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Determine if two objects are equal.
 *
 * @return True if the two objects are equal, false otherwise.
 */
bool Material::
operator==(const Material &aObject)
{
	if(getType() != aObject.getType()) return(false);
	if(getName() != aObject.getName()) return(false);
	return(true);
}


//=============================================================================
// GET AND SET
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the Translucency for this Material.
 */
void Material::
setTranslucency(const double aTranslucency)
{
	_translucency=aTranslucency;
}

/**
 * Get the Translucency for this Material.
 */
double Material::
getTranslucency() const
{
	return _translucency;
}

/**
 * Set the Ambient Color for this Material.
 */

void Material::
setAmbientColor(const double aAmbientColor[3])
{
	for(int i=0; i < 3; i++)
		_ambientColor[i]=aAmbientColor[i];
}
/**
 * Get the Ambient Color for this Material.
 */
const double* Material::
getAmbientColor() const
{
	return &_ambientColor[0];
}

/**
 * Set the Diffuse Color for this Material.
 */
void Material::
setDiffuseColor(const double aDiffuseColor[3])
{
	for(int i=0; i < 3; i++)
		_diffuseColor[i]=aDiffuseColor[i];
}

/**
 * Get the Diffuse Color for this Material.
 */
const double* Material::
getDiffuseColor() const
{
	return &(_diffuseColor[0]);
}
/**
 * Set the Specular Color for this Material.
 */
void Material::
setSpecularColor(const double aSpecularColor[3])
{
	for(int i=0; i < 3; i++)
		_specularColor[i]=aSpecularColor[i];
}
/**
 * Get the Specular Color for this Material.
 */
const double* Material::
getSpecularColor() const
{
	return &(_specularColor[0]);
}


//=============================================================================
// XML serialization support
//=============================================================================
//=============================================================================
// Static functions to handle default Material
//=============================================================================
//_____________________________________________________________________________
/**
 * These methods use lazy evaluation so the default is created only on demand
 */
const string& Material::
GetDefaultMaterialName()
{
	// create default material here if needed then get its name
	const Material &defMat = GetDefaultMaterial();

	return(defMat.getName());
}
//_____________________________________________________________________________
/**
 * Return the default Material
 */
const Material &Material::
GetDefaultMaterial()
{
	if (_defaultMaterial)
		return (*_defaultMaterial);
	_defaultMaterial = new Material();
	_defaultMaterial->setName("DEFAULT");
	// Other properties are defaults generated in setNull function above
	return (*_defaultMaterial);
}
