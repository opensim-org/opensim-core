#ifndef _XMLDocument_h_
#define _XMLDocument_h_
// XMLDocument.h
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

/* Note: This code was originally developed by Realistic Dynamics Inc. 
 * Author: Frank C. Anderson 
 */


// INCLUDES
#include <iostream>	// Ayman: Remove .h extension per .NET 2003
#include "osimCommonDLL.h"
#include <SimTKcommon.h>
#include "Array.h"

//using namespace std;	// Ayman:per .NET 2003



namespace OpenSim { 
//=============================================================================
//=============================================================================
/**
 * A class for managing and for performing operations on an XML document.
 * The document can originate from one of two sources:  1) parsed from an
 * XML source or 2) created locally in memory.
 *
 * @version 1.0
 * @author Ayman Habib, Frank C. Anderson, 
 */
#ifdef WIN32
#pragma warning( disable : 4251 )	// VC2010 no-dll export of std::string

#endif

class Object;

class OSIMCOMMON_API XMLDocument  : public SimTK::Xml::Document {

//=============================================================================
// DATA
//=============================================================================
public:
	/** Latest version of the code encoded as an int xxyyzz where x: major release, y: minor, z: patch */
	static const int LatestVersion;
private:
	/** Name of the XML Document */
	std::string _fileName;
	/** Document Version as written to the file */
	int _documentVersion;
	OpenSim::Array<Object*> _defaultObjects;
//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION AND DESTRUCTION
	//--------------------------------------------------------------------------
public:
	virtual ~XMLDocument();
	XMLDocument();
	XMLDocument(const std::string &aFileName);
	XMLDocument(const XMLDocument &aDocument);
	void copyDefaultObjects(const XMLDocument &aDocument){
		_defaultObjects = aDocument._defaultObjects;
	}
	void writeDefaultObjects(SimTK::Xml::Element& elmt);
	//--------------------------------------------------------------------------
	// VERSIONING /BACKWARD COMPATIBILITY SUPPORT
	//--------------------------------------------------------------------------	
	static const int& getLatestVersion() { return LatestVersion; };
	static void renameChildNode(SimTK::Xml::Element& aNode, std::string oldElementName, std::string newElementName);
	const int& getDocumentVersion() const { return _documentVersion; };
	static void getVersionAsString(const int aVersion, std::string& aString); 
	Xml::Element getRootDataElement();
	bool isEqualTo(XMLDocument& aOtherDocument, double toleranceForDoubles=1e-6, 
		bool compareDefaults=false, bool compareVersionNumbers=false);
private:
	static bool isElementEqual(SimTK::Xml::Element& elt1, SimTK::Xml::Element& elt2, double toleranceForDoubles);
	void updateDocumentVersion();

public:
	//--------------------------------------------------------------------------
	// SET AND GET
	//--------------------------------------------------------------------------
	void setFileName(const std::string &aFileName);
	const std::string &getFileName() const;
	void addDefaultObject(OpenSim::Object* aDefaultObject);
	bool hasDefaultObjects() const { return (_defaultObjects.getSize()>0); };
	//--------------------------------------------------------------------------
	// IO
	//--------------------------------------------------------------------------
	bool print(const std::string &aFileName=NULL);

//=============================================================================
};	// END CLASS XMLDocument

}; //namespace
//=============================================================================
//=============================================================================


#endif // __XMLDocument_h__
