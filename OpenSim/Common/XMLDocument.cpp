/* -------------------------------------------------------------------------- *
 *                         OpenSim:  XMLDocument.cpp                          *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Frank C. Anderson                                               *
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


//-----------------------------------------------------------------------------
// INCLUDES
//-----------------------------------------------------------------------------
#include "XMLDocument.h"
#include "Object.h"


using namespace OpenSim;
using namespace std;


//-----------------------------------------------------------------------------
// CONSTANTS
//-----------------------------------------------------------------------------

// This list of version numbers is not complete
// 20301 for separation of RRATool, CMCTool
// 20302 for Muscle's pennation_angle -> pennation_angle_at_optimal
// 20303
// 30000 for OpenSim 3.0 release
// 30500 for OpenSim 4.0 development and Connectors
// 30501 for changing serialization of Marker
// 30502 for changing serialization of Geometry
// 30503 for changing serialization of Ground
// 30505 for changing serialization of Joint to create offset frames
// 30506 for testing 30505 conversion code
// 30507 for changing serialization of Coordinates owned by Joint
// 30508 for moving Connector's connectee_name to enclosing Component.
// 30509 for replacing 'isDisabled' with: 'appliesForce', 'isEnforced' and
//       'enabled', for Force, Constraint and Controller, respectively
// 30510 for renaming Connector to Socket.
// 30511 for replacing Probe::isDisabled with Probe::enabled.
// 30512 for removing Model::FrameSet and moving frames to components list
// 30513 for removing internal (silent) clamping of Muscle controls (excitations)
// 30514 for removing "reverse" property from Joint
const int XMLDocument::LatestVersion = 30514;
//=============================================================================
// DESTRUCTOR AND CONSTRUCTOR(S)
//=============================================================================
//-----------------------------------------------------------------------------
// DESTRUCTOR
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Handle delete of an XMLDocument object.
 */
XMLDocument::~XMLDocument()
{
    for(int i = 0; i < _defaultObjects.size(); i++) {
        delete _defaultObjects.get(i);
    }
    _defaultObjects.setSize(0);
}

//-----------------------------------------------------------------------------
// CONSTRUCTOR(S)
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Construct a XMLDocument object with a locally generated DOMDocument.
 * This constructor is used when an XML document is going to be generated
 * locally in memory without reference to an XML file.  The initial
 * DOMDocument is empty.
 */
XMLDocument::XMLDocument()
{
    setRootTag("OpenSimDocument");
    stringstream latestVersionString;
    latestVersionString << LatestVersion;
    _documentVersion = LatestVersion;
    getRootElement().setAttributeValue("Version", latestVersionString.str());
}

//_____________________________________________________________________________
/**
 * Construct an XMLDocument object from an XML document.
 * A parser is created for the purpose of reading in the XML file.
 *
 * @param aFileName File name of the XML document.
 */
XMLDocument::XMLDocument(const string &aFileName) :
SimTK::Xml::Document(aFileName)
{


    _fileName = aFileName;

    // Update document version based on parsing
    updateDocumentVersion();
}

//_____________________________________________________________________________
/**
 * Construct a copy of an XMLDocument object.  The document an all its nodes
 * are copied; however, the parser associated with the copied document, if
 * any, is not copied.
 */
XMLDocument::XMLDocument(const XMLDocument &aDocument):
SimTK::Xml::Document(aDocument)
{
    _documentVersion = aDocument.getDocumentVersion();
    _fileName = aDocument.getFileName();
}



//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________


//=============================================================================
// SET AND GET
//=============================================================================
//-----------------------------------------------------------------------------
// DOCUMENT
//-----------------------------------------------------------------------------

void XMLDocument::
setFileName(const string &aFileName)
{
    _fileName = aFileName;
}

const string &XMLDocument::
getFileName() const
{
    return _fileName;
}

//=============================================================================
// IO
//=============================================================================
//-----------------------------------------------------------------------------
// PRINT
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Print the XML document to file.
 *
 * @param aFileName File name of the document to which to print
 */
bool XMLDocument::
print(const string &aFileName)
{
    // Standard Out
    if(aFileName.empty()) {
        cout << *this;
        cout << flush;
    // File
    } else {
        setIndentString("\t");
        writeToFile(aFileName);
    }
    return true;
}
//_____________________________________________________________________________

//-----------------------------------------------------------------------------
// FORMATTER
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
// STREAM OUTPUT
//-----------------------------------------------------------------------------
//_____________________________________________________________________________

//--------------------------------------------------------------------------
// VERSIONING /BACKWARD COMPATIBILITY SUPPORT
//--------------------------------------------------------------------------    
//_____________________________________________________________________________
/**
 * Convert passed in version number to a string 
 * The string is usually more compact e.g. 010100 -> 1_1 (rather than 11 which would confuse 110000 with 010100)
 */
void XMLDocument::
getVersionAsString(const int aVersion, std::string& aString)
{
    char pad[3];
    int ver = aVersion;
    aString = "";
    int div = 10000;
    for(int i=0; i<3; i++)
    {
        int digits = ver / div;
        sprintf(pad, "%02d",digits); 
        ver -= div*(ver / div);
        div /=100;
        aString += string(pad);
        if (ver ==0) break;
        aString +=(i<2?"_":"");
    }
}
//_____________________________________________________________________________
/**
 * Update member variable  _documentVersion based on parsing
 * Key assumption is that parsing has finished but no Object parsing is done yet
 */
void XMLDocument::
updateDocumentVersion()
{
    // Check root node if it's OpenSimDocument
    std::string rootTag = getRootTag();
    if (rootTag == "OpenSimDocument"){
        _documentVersion = getRootElement().getRequiredAttributeValueAs<int>("Version");
    }
    else {
        _documentVersion = 10500;  // Old version pre 1.6
    }

    // Validate >=  10500 and < latest as sanity check
    assert(_documentVersion >= 10500 && _documentVersion <= LatestVersion);
}
//_____________________________________________________________________________
/**
 * getRootDataElement returns a pointer to the real root node that contains objects 
 * works as a wrapper to get around the new root node <OpenSimDocument introduced in 1.6
 */
SimTK::Xml::Element  XMLDocument::
getRootDataElement()
{
    // Check root node if it's OpenSimDocument
    std::string rootTag = getRootTag();
    if (rootTag == "OpenSimDocument"){
        _documentVersion = getRootElement().getRequiredAttributeValueAs<int>("Version");
        return (*getRootElement().element_begin());
        }
    else {
        _documentVersion = 10500;  // Old version pre 1.6
        return  getRootElement();
    }
}
/**
 */
void XMLDocument::addDefaultObject(OpenSim::Object *aDefaultObject)
{
    _defaultObjects.append(aDefaultObject);
}
void XMLDocument::writeDefaultObjects(SimTK::Xml::Element& elmt)
{
    if (_defaultObjects.getSize()==0) return;
    // Make node for "defaults"
    SimTK::Xml::Element defaultsElement("defaults");
    
    elmt.insertNodeAfter(elmt.node_end(), defaultsElement);
    for(int i=0; i < _defaultObjects.getSize(); i++){
        _defaultObjects.get(i)->updateXMLNode(defaultsElement);
    }
}

void XMLDocument::copyDefaultObjects(const XMLDocument &aDocument){
        _defaultObjects.setSize(0);
        for (int i=0; i< aDocument._defaultObjects.getSize(); i++)
            _defaultObjects.append(aDocument._defaultObjects.get(i)->clone());
}

/*static*/ 
void  XMLDocument::renameChildNode(SimTK::Xml::Element& aNode, std::string oldElementName, std::string newElementName)
{
    SimTK::Xml::element_iterator elmtIter(aNode.element_begin(oldElementName));
    if (elmtIter!=aNode.element_end()){
        elmtIter->setElementTag(newElementName);
    }
}

bool XMLDocument::isEqualTo(XMLDocument& aOtherDocument, double toleranceForDoubles, bool compareDefaults, bool compareVersionNumbers) 
{
    bool equal = true;

    if (compareVersionNumbers)
        equal = (_documentVersion == aOtherDocument._documentVersion);
    if (!equal) return false;
    // Get Roots 
    SimTK::Xml::Element root1=  getRootElement();
    SimTK::Xml::Element root2=  aOtherDocument.getRootElement();

    //if (!equal) return false;
    // Cycle through children and compare. Order is assumed to be the same for now
    SimTK::Array_<SimTK::Xml::Element> elts1 = root1.getAllElements();
    SimTK::Array_<SimTK::Xml::Element> elts2 = root2.getAllElements();
    if (elts1.size() != elts2.size()){
        cout << "Different number of children at Top level" << endl;
        equal = false;
    }
    if (!equal) return false;
    // Recursively compare Elements
    SimTK::String s1,s2;
    for(unsigned it = 0; it < elts1.size(); it++){
        elts1[it].writeToString(s1);
        elts2[it].writeToString(s2);

        if (elts1[it].getElementTag()==elts2[it].getElementTag() && elts1[it].getElementTag()=="defaults" && !compareDefaults) 
            continue;
        equal = isElementEqual(elts1[it], elts2[it], toleranceForDoubles);
        if (!equal){ 
            cout << elts1[it].getElementTag() << " is different" << endl;  
            return false; 
        }
    }
    return true;
}

bool XMLDocument::isElementEqual(SimTK::Xml::Element& elt1, SimTK::Xml::Element& elt2, double toleranceForDoubles)
{
    SimTK::String s1,s2;
    elt1.writeToString(s1);
    elt2.writeToString(s2);
    SimTK::Xml::attribute_iterator att1 = elt1.attribute_begin();
    SimTK::Xml::attribute_iterator att2 = elt2.attribute_begin();
    // Handle different # attributes
    if ( (att1 == elt1.attribute_end() && att2 != elt2.attribute_end()) ||
         (att1 != elt1.attribute_end() && att2 == elt2.attribute_end()) ){
            cout << "Number of attributes is different, element " << elt1.getElementTag() << endl;
            return false;
    }
    bool equal =true;
    // Same size attributes including none
    for(att1 = elt1.attribute_begin(); att1 != elt1.attribute_end() && equal; att1++, att2++){
        equal = (att1->getName() == att2->getName());
        equal = equal && (att1->getValue() == att2->getValue());
        if (!equal) {
            cout << "Attribute " << att1->getName() << " is different " << att1->getValue() << 
            "vs." << att2->getValue() << endl;
        }
    }
    if (!equal) return false;

    // Attributes match now children
    SimTK::Array_<SimTK::Xml::Element> elts1 = elt1.getAllElements();
    SimTK::Array_<SimTK::Xml::Element> elts2 = elt2.getAllElements();
    if (elts1.size() != elts2.size()){
        cout << "Different number of children for Element " << elt1.getElementTag() << endl;
        equal = false;
    }
    if (!equal) return false;
    // Recursively compare Elements unless Value Elements in that case do direct compare
    for(unsigned it = 0; it < elts1.size() && equal; it++){
        SimTK::String elt1Tag = elts1[it].getElementTag();
        cout << "Compare " << elt1Tag << endl;
        SimTK::Xml::element_iterator elt2_iter = elt2.element_begin(elt1Tag);
        if (elt2_iter==elt2.element_end()){
            cout << "Element " << elt1Tag << " was not found in reference document" << endl;
            equal = false;
            break;
        }
        bool value1 = elts1[it].isValueElement();
        bool value2 = elt2_iter->isValueElement();
        equal = (value1 == value2);
        if (!equal){ 
            cout << elts1[it].getElementTag() << " is different. One is Value Element the other isn't" << endl;  
            return false; 
        }
        if (value1){
            // We should check if this's a double or array of doubles in that case we can getValueAs<double>
            try {
                SimTK::Array_<double> v1, v2;
                elts1[it].getValueAs(v1);
                elt2_iter->getValueAs(v2);
                for(unsigned ix=0; ix<v1.size() && equal; ix++)
                    equal = (std::fabs(v1[ix]-v2[ix]) < toleranceForDoubles);
            }
            catch(...){
                equal = (elts1[it].getValue() == elt2_iter->getValue());
            }
        }
        else    // recur
            equal = isElementEqual(elts1[it], elts2[it], toleranceForDoubles);
        if (!equal){ 
            cout << elts1[it].getElementTag() << " is different" << endl;  
            SimTK::String pad;
            elts1[it].writeToString(pad);
            cout << pad << endl;
            cout << "------------------- vs. ------" << endl;
            elts2[it].writeToString(pad);
            cout << pad << endl;
            return equal; 
        }
    }

    return equal;
}

/*
 * Helper function to add connector to the xmlElement passed in
 */
void XMLDocument::addConnector(SimTK::Xml::Element& element,
    const std::string& connectorTag, const std::string& connectorName, 
    const std::string& connectorValue)
{
    SimTK::Xml::element_iterator  connectors_node =  element.element_begin("connectors");
    //SimTK::String debug; //Only used for debugging
    if (connectors_node == element.element_end()){
        SimTK::Xml::Element connectorsElement("connectors");
        element.insertNodeBefore(element.element_begin(), connectorsElement);
        connectors_node =  element.element_begin("connectors");
    }
    // Here we're guaranteed connectors node exists, add individual connector
    SimTK::Xml::Element newConnectorElement(connectorTag);
    newConnectorElement.setAttributeValue("name", connectorName);
    //newConnectorElement.writeToString(debug);

    SimTK::Xml::Element connecteeElement("connectee_name");
    connecteeElement.insertNodeAfter(connecteeElement.element_end(), SimTK::Xml::Text(connectorValue));
    // Insert text under newConnectorElement
    newConnectorElement.insertNodeAfter(newConnectorElement.element_end(), connecteeElement);
    connectors_node->insertNodeAfter(connectors_node->element_end(), newConnectorElement);
    //connectors_node->writeToString(debug);
}

void XMLDocument::updateConnectors30508(SimTK::Xml::Element& componentElt)
{
    using ElementItr = SimTK::Xml::element_iterator;
    
    ElementItr connectors_node = componentElt.element_begin("connectors");
    
    // See if there's a <connectors> element.
    if (connectors_node == componentElt.element_end()) return;
    
    for (ElementItr connectorElt = connectors_node->element_begin();
            connectorElt != componentElt.element_end();
            ++connectorElt) {
        // Grab name of Connector.
        const auto& connectorName =
                connectorElt->getRequiredAttributeValue("name");
        // Grab value of connectee_name property.
        ElementItr connecteeNameElt =
                connectorElt->element_begin("connectee_name");
        SimTK::String connecteeName;
        connecteeNameElt->getValueAs<std::string>(connecteeName);
        
        // Create new element for this connector's connectee name.
        SimTK::Xml::Element newConnecteeNameElt(
                "connector_" + connectorName + "_connectee_name");
        newConnecteeNameElt.setValue(connecteeName);
        componentElt.insertNodeAfter(connectors_node, newConnecteeNameElt);
    }
    
    // No longer want the old syntax for connectors.
    componentElt.eraseNode(connectors_node);
}

void XMLDocument::addPhysicalOffsetFrame(SimTK::Xml::Element& element,
    const std::string& frameName,
    const std::string& parentFrameName, 
    const SimTK::Vec3& location, const SimTK::Vec3& orientation)
{
    SimTK::Xml::element_iterator  frames_node = element.element_begin("frames");
    //SimTK::String debug; //Only used for debugging
    
    if (frames_node == element.element_end()) {
        SimTK::Xml::Element framesElement("frames");
        element.insertNodeBefore(element.element_begin(), framesElement);
        frames_node = element.element_begin("frames");
    }
    // Here we're guaranteed frames node exists, add individual frame
    SimTK::Xml::Element newFrameElement("PhysicalOffsetFrame");
    newFrameElement.setAttributeValue("name", frameName);
    //newFrameElement.writeToString(debug);

    XMLDocument::addConnector(newFrameElement, "Connector_PhysicalFrame_", "parent", parentFrameName);

    std::ostringstream transValue;
    transValue << location[0] << " " << location[1] << " " << location[2];
    SimTK::Xml::Element translationElement("translation", transValue.str());
    newFrameElement.insertNodeAfter(newFrameElement.element_end(), translationElement);

    std::ostringstream orientValue; 
    orientValue << orientation[0] << " " << orientation[1] << " " << orientation[2];
    SimTK::Xml::Element orientationElement("orientation", orientValue.str());
    newFrameElement.insertNodeAfter(newFrameElement.element_end(), orientationElement);

    frames_node->insertNodeAfter(frames_node->element_end(), newFrameElement);
    //frames_node->writeToString(debug);
}
