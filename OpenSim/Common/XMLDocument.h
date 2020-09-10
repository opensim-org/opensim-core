#ifndef OPENSIM_XMLDOCUMENT_H
#define OPENSIM_XMLDOCUMENT_H
/* -------------------------------------------------------------------------- *
 *                          OpenSim:  XMLDocument.h                           *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
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
#include "osimCommonDLL.h"
#include "Array.h"
#include "SimTKcommon/internal/Xml.h"
#include "SimTKcommon/SmallMatrix.h"

//using namespace std;  // Ayman:per .NET 2003



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
#ifdef _WIN32
#pragma warning( disable : 4251 )   // VC2010 no-dll export of std::string

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
    SimTK::Xml::Element getRootDataElement();
    bool isEqualTo(XMLDocument& aOtherDocument, double toleranceForDoubles=1e-6, 
        bool compareDefaults=false, bool compareVersionNumbers=false);
    /** This adds an XML element to `element` of the following form:
    @code
    <Connector_PhysicalFrame_ name="parent_frame">
        <connectee_name>...</connectee_name>
    </Connector_PhysicalFrame_>
    @endcode
    This syntax was revised in XML document version 30508; see
    updateConnectors30508(). As such, this function should *not* be used for
    updating versions 30508 or greater. */
    static void addConnector(SimTK::Xml::Element& element,
        const std::string& connectorTag, const std::string& connectorName, 
        const std::string& connectorValue);
    /** In version 30508, the XML syntax for Connectors changed:
    Previous:
    @code
    <connectors>
        <Connector_PhysicalFrame_ name="parent_frame">
            <connectee_name>...</connectee_name>
        </Connector_PhysicalFrame_>
        <Connector_PhysicalFrame_ name="child_frame">
            <connectee_name>...</connectee_name>
        </Connector_PhysicalFrame_>
    </connectors>
    @endcode
    New:
    @code
    <connector_parent_frame_connectee_name>...
        </connector_parent_frame_connectee_name>
    <connector_child_frame_connectee_name>...
        </connector_child_frame_connectee_name>
    @endcode
    
    If there is no `<connectors>` element, then this function does not edit
    componentElt. */
    static void updateConnectors30508(SimTK::Xml::Element& componentElt);
    static void addPhysicalOffsetFrame30505_30517(SimTK::Xml::Element& element,
        const std::string& frameName,
        const std::string& parentFrameName,
        const SimTK::Vec3& location, const SimTK::Vec3& orientation);
    /** Convert component names into the appropriate paths based on where we
     * know components were located in version 30000 model files.
     * @param connecteeSetName "bodyset", "jointset", etc.
     * @param connecteeName The name of the connectee from the version 30000
     *      file.
     * Note: It is okay for connecteePath to be a reference to connecteeName. */
    static std::string updateConnecteePath30517(
            const std::string& connecteeSetName,
            const std::string& connecteeName);
    /** Find the first XML Element (depth-first search) with the provided
    `name`, anywhere in the XML document that contains `element`. If the XML
    document does not contain an element with name `name`, or if `name` is
    empty, then the returned Xml::Element is empty (Xml::Element::isValid()
    returns false). */
    static SimTK::Xml::Element findElementWithName(
            SimTK::Xml::Element& element, const std::string& name);

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
    /// If the filename is empty, the file is printed to cout.
    bool print(const std::string& aFileName = {});

//=============================================================================
};  // END CLASS XMLDocument

}; //namespace
//=============================================================================
//=============================================================================


#endif // OPENSIM_XMLDOCUMENT_H
