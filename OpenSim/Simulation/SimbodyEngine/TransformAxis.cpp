// TransformAxis.cpp
// Authors: Peter Loan, Frank C. Anderson, Jeffrey A. Reinbolt, Ajay Seth,
//          Michael Sherman
/*
 * Copyright (c)  2006-2012, Stanford University. All rights reserved. 
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

//=============================================================================
// INCLUDES
//=============================================================================
#include <OpenSim/Common/XMLDocument.h>
#include <OpenSim/Common/Function.h>
#include <OpenSim/Simulation/Model/Model.h>

#include "TransformAxis.h"
#include "Joint.h"
#include "Coordinate.h"
#include "SimbodyEngine.h"

//=============================================================================
// USING
//=============================================================================
using namespace std;
using namespace OpenSim;
using SimTK::Vec3; using SimTK::State; using SimTK::Vector; using SimTK::Xml;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
// Default constructor.
TransformAxis::TransformAxis() {
	setNull();
	constructProperties();
}

//_____________________________________________________________________________
// Constructor with coordinate names and axis.
TransformAxis::TransformAxis(const Array<string>& coordNames, 
                             const Vec3&          axis) {
	setNull();
	constructProperties();
	setCoordinateNames(coordNames);
	setAxis(axis);
}
// Constructor from XML node.
TransformAxis::TransformAxis(Xml::Element& aNode) {
	setNull();
	constructProperties();
	updateFromXMLNode(aNode);
}

//_____________________________________________________________________________
/**
 * Perform set up functions after model has been deserialized or copied.
 *
 * @param aEngine dynamics engine containing this dof.
 * @param aJoint joint containing this dof.
 */
void TransformAxis::connectToJoint(const Joint& aJoint)
{
	string errorMessage;

	_joint = &aJoint;

	// Look up the coordinates by name.
    const Property<string>& coordNames = getProperty_coordinates();
	int nc = coordNames.size();
	const CoordinateSet& coords = _joint->getCoordinateSet();

    // If a Function has been assigned then we have to insist that any 
    // specified coordinates actually exist in this joint.
    // TODO: (sherm 20120418) Why do we allow unrecognized coordinate names
    // if there is no Function?

    if (!hasFunction())
        return; // no need to check

	for(int i=0; i< nc; ++i) {
        if (!coords.contains(coordNames[i])) {
			errorMessage += "Invalid coordinate (" 
                            + coordNames[i] 
                            + ") specified for TransformAxis " 
                            + getName() + " in joint " + aJoint.getName();
			throw (Exception(errorMessage));
		}
	}
}


//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================

//_____________________________________________________________________________
/**
 * Set the data members of this TransformAxis to their null values.
 */
void TransformAxis::setNull()
{
    _joint = NULL;
}

//_____________________________________________________________________________
// Define properties.
void TransformAxis::constructProperties() {
    constructProperty_coordinates();
    constructProperty_axis(Vec3(1,0,0)); // use x-axis by default
    constructProperty_function();
}

//=============================================================================
// GET AND SET
//=============================================================================
//_____________________________________________________________________________
/**
 * Get the current value of the transform.
 *
 * @return Current value of the transform.
 */
double TransformAxis::getValue(const State& s )
{
    const Property<string>& coordNames = getCoordinateNames();
	const int nc = coordNames.size();
	const CoordinateSet& coords = _joint->getCoordinateSet();

	Vector workX(nc, 0.0);
	for (int i=0; i < nc; ++i)
		workX[i] = coords.get(coordNames[i]).getValue(s);

	return getFunction().calcValue(workX);
}

//_____________________________________________________________________________
const OpenSim::Function& TransformAxis::getFunction() const
{
    const Property<Function>& function = getProperty_function();
    if (function.empty())
        throw Exception("TransformAxis::getFunction(): no Function is defined");
	return function.getValue();
}

OpenSim::Function& TransformAxis::updFunction()
{
    Property<Function>& function = updProperty_function();
    if (function.empty())
        throw Exception("TransformAxis::getFunction(): no Function is defined");
	return function.updValue();
}

//_____________________________________________________________________________
// This method takes over ownership.
void TransformAxis::setFunction(OpenSim::Function* func)
{
    Property<Function>& prop = updProperty_function();
    prop.clear();
    prop.adoptAndAppendValue(func);
}

// This method makes a copy.
void TransformAxis::setFunction(const OpenSim::Function& func)
{
    updProperty_function() = func;
}


void TransformAxis::updateFromXMLNode
   (Xml::Element& node, int versionNumber)
{
	// Version before refactoring spatialTransform.
    // TODO: this is handled in CustomJoint's updateFromXMLNode() method
    // but should be dealt with here. Can't do it both places, though.

    //XMLDocument::renameChildNode(node, "coordinate", "coordinates");

	Super::updateFromXMLNode(node, versionNumber);
}