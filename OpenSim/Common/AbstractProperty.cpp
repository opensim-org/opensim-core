/* -------------------------------------------------------------------------- *
 *                       OpenSim:  AbstractProperty.cpp                       *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
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

//============================================================================
// INCLUDES
//============================================================================
#include "AbstractProperty.h"

#include "Assertion.h"
#include "Component.h"
#include "Object.h"

#include <limits>
#include <sstream>
#include <utility>

using namespace OpenSim;
using namespace SimTK;
using namespace std;

namespace
{
    static std::string generateInvalidPropertyValueMessage(
        const std::string& propertyName,
        const std::string& errorMsg)
    {
        std::stringstream ss;
        ss << "Property '" << propertyName << "' has an invalid property value.\n";
        ss << "(details: " << errorMsg << ").\n";
        return std::move(ss).str();
    }
}

InvalidPropertyValue::InvalidPropertyValue(const std::string& file,
    size_t line,
    const std::string& func,
    const Object& obj,
    const std::string& propertyName,
    const std::string& errorMsg) :
    Exception(file, line, func, obj) {

    addMessage(generateInvalidPropertyValueMessage(propertyName, errorMsg));
}

InvalidPropertyValue::InvalidPropertyValue(
    const std::string& file,
    size_t line,
    const std::string& func,
    const Component& component,
    const std::string& propertyName,
    const std::string& errorMsg) :
    Exception(file, line, func, component) {

    addMessage(generateInvalidPropertyValueMessage(propertyName, errorMsg));
}


//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
AbstractProperty::AbstractProperty()
{
    setNull();
}
//_____________________________________________________________________________
/**
 * Constructor.
 */
AbstractProperty::AbstractProperty(const std::string& name, 
                                   const std::string& comment)
{
    setNull();
    _name       = name;
    _comment    = comment;
}


//_____________________________________________________________________________
/**
 * Set member variables to their null values.
 */
void AbstractProperty::setNull()
{
    _name           = "";
    _comment        = "";
    _valueIsDefault = false;
    _minListSize    = 0;
    _maxListSize    = std::numeric_limits<int>::max();
}

void AbstractProperty::clear() {
    clearValues();
}

// Set the use default flag for this property, and propagate that through
// any contained Objects.
void AbstractProperty::setAllPropertiesUseDefault(bool shouldUseDefault) {
    setValueIsDefault(shouldUseDefault);
    if (!isObjectProperty())
        return;
    for (int i=0; i < size(); ++i)
        updValueAsObject(i).setAllPropertiesUseDefault(shouldUseDefault);
}

// Implement the policy that locates a property value's element within its 
// parent element and then ask the concrete property to deserialize itself from
// that element.
void AbstractProperty::readFromXMLParentElement(Xml::Element& parent,
                                                int           versionNumber)
{
    // If this property has a real name (that is, doesn't use the object type
    // tag as a name), look for the first element whose tag is
    // that name and read it if found. That is, we're looking for
    //      <propName> ... </propName>
    if (!isUnnamedProperty()) {
        Xml::element_iterator propElt = parent.element_begin(getName());
        if (propElt != parent.element_end()) {
            readFromXMLElement(*propElt, versionNumber);
            setValueIsDefault(false);
            return;
        }
    }

    // Didn't find a property element by its name (or it didn't have one).
    // There is still hope: If this is an object property, restricted to 
    // contain exactly one Object, then it is allowed to have an alternate
    // form.

    if (!isOneObjectProperty()) {
        setValueIsDefault(true); // no special format allowed
        return;
    }

    // The property contains just a single object so we can look for the 
    // abbreviated form: 
    //      <ObjectTypeTag name=propName> contents </ObjectTypeTag>
    // In case this is an unnamed property, only the type has to be right in
    // the source XML file, and any name (or no name attribute) is acceptable.

    // If the current property does have a property name, we select the first
    // element that has that as the value of its name attribute; then it is
    // an error if the type tag is not acceptable. On the other hand, if there
    // is no property name, we select the first element that has an acceptable
    // type; we don't care about its name attribute in that case.
    // As a final loophole, if we fail to find a matching name, but there is
    // an unnamed object in the file whose type is acceptable, we'll use that
    // rather than report an error. That allows us to add a name to a 
    // formerly unnamed one-object property in the code yet still read old 
    // files that contain unnamed objects.

    // If we find a promising element, we'll canonicalize by adding a parent
    // property element temporarily to produce:
    //     <propName> 
    //         <ObjectTypeTag name=propName> contents </ObjectTypeTag> 
    //     </propName>
    // or
    //     <Unnamed> 
    //         <ObjectTypeTag> contents </ObjectTypeTag> 
    //     </Unnamed>
    // and then delegate to the concrete property the job of reading in the
    // property value.
    Xml::element_iterator prev = parent.element_end();
    Xml::element_iterator iter = parent.element_begin();
    if (isUnnamedProperty()) {
        for (; iter != parent.element_end(); prev=iter++) 
            if (isAcceptableObjectTag(iter->getElementTag()))
                break; // Found a good tag; name doesn't matter.
    } else { // this property has a name
        // First pass: look for an object with that name attribute
        for (; iter != parent.element_end(); prev=iter++) 
            if (iter->getOptionalAttributeValue("name") == getName()) {
                // Found the right name; tag must be acceptable.
                if (!isAcceptableObjectTag(iter->getElementTag())) {
                    throw OpenSim::Exception
                        ("Found XML element with expected property name=" + getName()
                        + " in parent element " + parent.getElementTag()
                        + " but its tag " + iter->getElementTag()
                        + " was not an acceptable type for this property.");
                    return;
                }
                break;
            }
        if (iter == parent.element_end()) {
            // Second pass: look for an unnamed object with the right type
            prev = parent.element_end();
            iter = parent.element_begin();
            for (; iter != parent.element_end(); prev=iter++) {
                if (   iter->getOptionalAttributeValue("name").empty()
                    && isAcceptableObjectTag(iter->getElementTag()))
                    break; // Found a good tag; we'll ignore the name
            }
        }
    }

    if (iter == parent.element_end()) {
        // Couldn't find an acceptable element for this one-object property.
        setValueIsDefault(true);
        return;
    }

    // Found a match. Borrow the object node briefly and canonicalize it 
    // into a conventional <propName> object </propName> structure.
    std::string propName = isUnnamedProperty() ? "Unnamed" : getName();
    Xml::Element dummy(propName);
    dummy.insertNodeAfter(dummy.node_end(), parent.removeNode(iter));
    parent.insertNodeAfter(parent.node_end(), dummy);

    readFromXMLElement(dummy, versionNumber);
    // Now put the node back where we found it.
    parent.insertNodeBefore(prev, 
                            dummy.removeNode(dummy.element_begin()));
    setValueIsDefault(false);
    parent.removeNode(parent.element_begin(dummy.getElementTag()));
    dummy.clearOrphan();
}


void AbstractProperty::writeToXMLParentElement(Xml::Element& parent) const {
    // Add comment if any.
    if (!getComment().empty())
        parent.insertNodeAfter(parent.node_end(), Xml::Comment(getComment()));

    if (!isOneObjectProperty()) {
        // Concrete property will be represented by an Xml element of
        // the form <propName> value(s) </propName>.
        OPENSIM_ASSERT(!getName().empty());
        Xml::Element propElement(getName());
        writeToXMLElement(propElement);
        parent.insertNodeAfter(parent.node_end(), propElement);
        return;
    }

    // This is a one-object property. It will be represented by an Xml
    // element 
    //      <ObjectTypeTag name=propName ...> value </ObjectTypeTag>
    // (if the property has a name), or 
    //      <ObjectTypeTag ...> value </ObjectTypeTag> 
    // otherwise.

    const Object& obj = getValueAsObject();

    // If this is a named property then the lone object must have its
    // name attribute set to the property name.
    obj.updateXMLNode(parent, this);
}

