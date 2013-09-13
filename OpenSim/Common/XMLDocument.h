#ifndef _XMLDocument_h_
#define _XMLDocument_h_
/* -------------------------------------------------------------------------- *
 *                          OpenSim:  XMLDocument.h                           *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Frank C. Anderson, Ayman Habib                                  *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

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
	void copyDefaultObjects(const XMLDocument &aDocument);
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
