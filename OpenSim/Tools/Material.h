#ifndef _Material_h_
#define _Material_h_
// Material.h
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
#include "PropertyDbl.h"
#include "PropertyDblArray.h"
namespace OpenSim { 

class XMLNode;
class XMLDocument;

//=============================================================================
//=============================================================================
/**
 * A class for representing the material properties of an object.
 *
 * @author Ayman Habib
 * @version 1.0
 */
class RDTOOLS_API Material: public Object
{
//=============================================================================
// DATA
//=============================================================================
public:
	/* serialized data members */
	// PROPERTIES
	/** Translucency. */
	PropertyDbl		_propTranslucency;
	/** Ambient color */
	PropertyDblArray  _propAmbientColor;
	/* Diffuse Color */
	PropertyDblArray _propDiffuseColor;
	/* Specular Color */
	PropertyDblArray _propSpecularColor;

	// REFERENCES
	double&					_translucency;
	Array<double>&		_ambientColor;
	Array<double>&		_diffuseColor;
	Array<double>&		_specularColor;

	// Could also include texture here.
	static Material *_defaultMaterial;
protected:

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	Material();
	Material(const std::string &aFileName);
	Material(DOMElement *aNode);
	Material(const Material &aMaterial);
	virtual ~Material();
	virtual Object* copy() const;
	virtual Object* copy(DOMElement *aElement) const;
private:
	void setNull();
protected:
	// Serialization support
	virtual void setupProperties();

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:
#ifndef SWIG
	Material& operator=(const Material &aObject);
	virtual bool operator==(const Material &aObject);
#endif
	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
	void setTranslucency(const double aTranslucency);
	double getTranslucency() const;

	void setAmbientColor(const double aAmbientColor[3]);
	const double* getAmbientColor() const;

	void setDiffuseColor(const double aDiffuseColor[3]);
	const double* getDiffuseColor() const;

	void setSpecularColor(const double aSpecularColor[3]);
	const double* getSpecularColor() const;

	static const std::string& GetDefaultMaterialName();
	static const Material &GetDefaultMaterial();
	//--------------------------------------------------------------------------
	// XML
	//--------------------------------------------------------------------------
public:
	DOMElement* getNode() const;

private:
	void generateDocument();

//=============================================================================
};	// END of class Material

}; //namespace
//=============================================================================
//=============================================================================

#endif //__Material_h__
