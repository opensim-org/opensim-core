// AbstractProperty.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c)  2012, Stanford University. All rights reserved. 
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

//============================================================================
// INCLUDES
//============================================================================
#include "AbstractProperty.h"
#include "Property.h"
#include "Object.h"

#include <string>
#include <limits>

using namespace OpenSim;
using namespace SimTK;
using namespace std;


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
	_name = name;
	_comment = comment;
}


//_____________________________________________________________________________
/**
 * Set member variables to their null values.
 */
void AbstractProperty::setNull()
{
	_name = "unknown";
    _comment = "";
	_useDefault = false;
	_minListSize = 0;
	_maxListSize = std::numeric_limits<int>::max();
}

void AbstractProperty::clear() {
    clearValues();
}

// Set the use default flag for this property, and propagate that through
// any contained Objects.
void AbstractProperty::setAllPropertiesUseDefault(bool shouldUseDefault) {
    setUseDefault(shouldUseDefault);
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
            setUseDefault(false);
            return;
        }
    }

    // Didn't find a property element by its name (or it didn't have one).
    // There is still hope: If this is an object property, restricted to 
    // contain exactly one Object, then it is allowed to have an alternate
    // form.

    if (!isOneObjectProperty()) {
        setUseDefault(true); // no special format allowed
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
        setUseDefault(true);
        return;
    }

    // Found a match. Borrow the object node briefly and canonicalize it 
    // into a conventional <propName> object </propName> structure.
    Xml::Element dummy(isUnnamedProperty() ? "Unnamed" : getName());
    dummy.insertNodeAfter(dummy.node_end(), parent.removeNode(iter));
    readFromXMLElement(dummy, versionNumber);
    // Now put the node back where we found it.
    parent.insertNodeBefore(prev, 
                            dummy.removeNode(dummy.element_begin()));
    setUseDefault(false);
}


void AbstractProperty::writeToXMLParentElement(Xml::Element& parent) {
	// Add comment if any.
	if (!getComment().empty())
		parent.insertNodeAfter(parent.node_end(), Xml::Comment(getComment()));

    if (!isOneObjectProperty()) {
        // Concrete property will be represented by an Xml element of
        // the form <propName> value(s) </propName>.
        assert(!getName().empty());
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

    Object& obj = updValueAsObject();

    // If this is a named property then the lone object must have its
    // name attribute set to the property name.
    obj.setName(isUnnamedProperty() ? "" : getName());

    obj.updateXMLNode(parent);
}

