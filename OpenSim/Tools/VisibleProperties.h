#ifndef _VisibleProperties_h_
#define _VisibleProperties_h_
// VisibleProperties.h
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

#include "rdTools.h"
#include "Object.h"
#include "PropertyInt.h"
#include "PropertyBool.h"
#include "PropertyStr.h"

namespace OpenSim { 

class XMLNode;
class XMLDocument;
class Material;

// CONSTANTS

//=============================================================================
//=============================================================================
/**
 * Class VisibleProperties is intended to contain all visible properties of an object
 * including Wireframe/Surface Shading-Quality,..
 *
 * @version 1.0
 * @author Ayman Habib
 */
class RDTOOLS_API VisibleProperties: public Object
{
public:
	enum  DisplayPreference {
			None=0,
			WireFrame=1,
			SolidFill=2,
			FlatShaded=3,
			GouraudShaded=4,
			PhongShaded=5,
			BoundingBox=6
	};

//=============================================================================
// DATA
//=============================================================================
private:
	// PROPERTIES
	/** DisplayPreference */
	PropertyInt	_propDisplayPreference;
	/** Whether to show normals when displaying the visible object */
	PropertyBool	_propShowNormals;
	/** Whether to show coordinate axes when displaying the visible object */
	PropertyBool	_propShowAxes;
	/** Name of the material assigned to this object */
	PropertyStr	_propMaterialName;

	// REFERENCES
	DisplayPreference&	_displayPreference;
	bool&				_showNormals;
	bool&				_showAxes;
	std::string&		_materialName;
	double				_color[3];
//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	VisibleProperties();
	VisibleProperties(const std::string &aFileName);
	VisibleProperties(const XMLDocument *aDocument);
	VisibleProperties(const VisibleProperties &aVisibleProperties);
	virtual ~VisibleProperties();

	Object* copy() const;
	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:
#ifndef SWIG
	VisibleProperties& operator=(const VisibleProperties &aVisibleProperties);
	bool operator==(const VisibleProperties &aObject);
#endif
private:
	void setNull();
	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
public:
	void setDisplayPreference(const DisplayPreference aDisplayPreference);
	DisplayPreference getDisplayPreference() const;

	void setShowNormals(const bool showNormals);
	bool getShowNormals() const;

	void setShowAxes(const bool showAxes);
	bool getShowAxes() const;

	void setMaterialName(const char *matName);
	const char *getMaterialName() const;
	
	void getColor(double aColor[3]) 
	{
		for(int i=0; i< 3; i++)
			aColor[i] = _color[i];
	}
	void setColor(double aColor[3])
	{
		for(int i=0; i< 3; i++)
			_color[i] = aColor[i];
	}
private:
	const Material &getMaterial(const char *matName) const;
	//--------------------------------------------------------------------------
	// XML
	//--------------------------------------------------------------------------
	void setupProperties();

//=============================================================================
};	// END of class VisibleProperties

}; //namespace
//=============================================================================
//=============================================================================

#endif //__VisibleProperties_h__
