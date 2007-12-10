// VisibleProperties.cpp
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


//============================================================================
// INCLUDES
//============================================================================
#include "VisibleProperties.h"
#include "Exception.h"
#include "Material.h"
#include "PropertyInt.h"
#include "PropertyBool.h"
#include "PropertyStr.h"




using namespace OpenSim;
using namespace std;


//============================================================================
// CONSTANTS
//============================================================================


//=============================================================================
// CONSTRUCTOR(S)
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
VisibleProperties::~VisibleProperties()
{
}

//_____________________________________________________________________________
/**
 * Default constructor.
 */
VisibleProperties::VisibleProperties():
_displayPreference((DisplayPreference&)_propDisplayPreference.getValueInt()),
_showNormals(_propShowNormals.getValueBool()),
_showAxes(_propShowAxes.getValueBool()),
_materialName(_propMaterialName.getValueStr())
{
	// NULL STATES
	setNull();

	// MEMBER VARIABLES
	setType("VisibleProperties");
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
VisibleProperties::VisibleProperties(const string &aFileName):
Object(aFileName, false),
_displayPreference((DisplayPreference&)_propDisplayPreference.getValueInt()),
_showNormals(_propShowNormals.getValueBool()),
_showAxes(_propShowAxes.getValueBool()),
_materialName(_propMaterialName.getValueStr())
{
	// NULL STATES
	setNull();
	// Serialize from XML
	updateFromXMLNode();
}
//_____________________________________________________________________________
/**
 * Construct an object from a document.
 *
 * The document is copied and this object, including its derived classes,
 * are constructed based on the nodes within the document.
 */
VisibleProperties::VisibleProperties(const XMLDocument *aDocument):
_displayPreference((DisplayPreference&)_propDisplayPreference.getValueInt()),
_showNormals(_propShowNormals.getValueBool()),
_showAxes(_propShowAxes.getValueBool()),
_materialName(_propMaterialName.getValueStr())
{
	// NULL STATES
	setNull();
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * Copy constructors for all VisibleProperties's only copy the non-XML variable
 * members of the object; that is, the object's DOMnode and XMLDocument
 * are not copied but set to NULL.  The reason for this is that for the
 * object and all its derived classes to establish the correct connection
 * to the XML document nodes, the the object would need to reconstruct based
 * on the XML document not the values of the object's member variables.
 *
 * There are three proper ways to generate an XML document for an VisibleProperties:
 *
 * 1) Construction based on XML file (@see VisibleProperties(const char *aFileName)).
 * In this case, the XML document is created by parsing the XML file.
 *
 * 2) Construction by VisibleProperties(const XMLDocument *aDocument).
 * This constructor explictly requests construction based on an
 * XML document.  In this way the proper connection between an object's node
 * and the corresponding node within the XML document is established.
 * This constructor is a copy constructor of sorts because all essential
 * VisibleProperties member variables should be held within the XML document.
 * The advantage of this style of construction is that nodes
 * within the XML document, such as comments that may not have any
 * associated VisibleProperties member variable, are preserved.
 *
 * 3) A call to generateXMLDocument().
 * This method generates an XML document for the VisibleProperties from scratch.
 * Only the essential document nodes are created (that is, nodes that
 * correspond directly to member variables.).
 *
 * @param aObject Object to be copied.
 * @see VisibleProperties(const XMLDocument *aDocument)
 * @see VisibleProperties(const char *aFileName)
 * @see generateXMLDocument()
 */
VisibleProperties::VisibleProperties(const VisibleProperties &aObject):
_displayPreference((DisplayPreference&)_propDisplayPreference.getValueInt()),
_showNormals(_propShowNormals.getValueBool()),
_showAxes(_propShowAxes.getValueBool()),
_materialName(_propMaterialName.getValueStr())
{
	// NULL MEMBER VARIABLES
	setNull();

	// COPY TYPE AND NAME
	*this = aObject;
}
/**
 * virtual copy constructor
 */
Object* VisibleProperties::
copy() const
{

	VisibleProperties *object = new VisibleProperties(*this);
	return(object);
}


//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Set all member variables to their null or default values.
 */
void VisibleProperties::
setNull()
{
	setupProperties();

	_displayPreference = GouraudShaded;
	_showNormals=false;
	_showAxes=false;
	_materialName="DEFAULT";
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void VisibleProperties::
setupProperties()
{
	_propertySet.append( &_propDisplayPreference );
	_propDisplayPreference.setName("display_preference");
	_propDisplayPreference.setValue(1);

	_propertySet.append( &_propShowNormals );
	_propShowNormals.setName("show_normals");
	_propShowNormals.setValue(false);

	_propertySet.append( &_propShowAxes );
	_propShowAxes.setName("show_axes");
	_propShowAxes.setValue(false);

	_propertySet.append( &_propMaterialName );
	_propMaterialName.setName("material_name");
	_propMaterialName.setValue("DEFAULT");

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
VisibleProperties& VisibleProperties::
operator=(const VisibleProperties &aObject)
{
	// BASE CLASS
	Object::operator=(aObject);

	// Class Members
	_displayPreference = (aObject._displayPreference);
	_showNormals = (aObject._showNormals);
	_showAxes = (aObject._showAxes);
	_materialName =  (aObject._materialName);
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
bool VisibleProperties::
operator==(const VisibleProperties &aObject)
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
 * setDisplayPreference: Wireframe, shaded, ...
 *
 */
void VisibleProperties::
setDisplayPreference(const DisplayPreference aDisplayPreference)
{
	_displayPreference =aDisplayPreference;
}

//_____________________________________________________________________________
/**
 * getDisplayPreference: Wireframe, shaded, ...
 *
 */
VisibleProperties::DisplayPreference VisibleProperties::
getDisplayPreference() const
{
	return _displayPreference;
}
//_____________________________________________________________________________
/**
 * setShowNormals flag
 *
 */
void VisibleProperties::
setShowNormals(const bool showNormals)
{
	_showNormals =showNormals;
}

//_____________________________________________________________________________
/**
 * getShowNormals flag
 *
 */
bool VisibleProperties::
getShowNormals() const
{
	return _showNormals;
}

//_____________________________________________________________________________
/**
 * setShowAxes flag
 *
 */
void VisibleProperties::
setShowAxes(const bool showAxes)
{
	_showAxes =showAxes;
}

//_____________________________________________________________________________
/**
 * getShowAxes flag
 *
 */
bool VisibleProperties::
getShowAxes() const
{
	return _showAxes;
}

//_____________________________________________________________________________
/**
 * get the name of the material associated with object
 *
 */
const char *VisibleProperties::
getMaterialName() const
{
	return _materialName.c_str();
}

//_____________________________________________________________________________
/**
 * set the name of the material associated with object
 *
 */
void VisibleProperties::
setMaterialName(const char *matName)
{
	_materialName = matName;
}
