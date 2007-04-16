// Analysis.cpp
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

/* Note: This code was originally developed by Realistic Dynamics Inc. 
 * Author: Frank C. Anderson 
 */


//=============================================================================
// INCLUDES
//=============================================================================
#include "Analysis.h"
#include "Model.h"



using namespace OpenSim;
using namespace std;
//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 *
 * @param aModel Model on which the analysis is to be performed.
 */
Analysis::Analysis(Model *aModel):
	IntegCallback(aModel),
	_inDegrees(_inDegreesProp.getValueBool())
{
	
	setNull();

	// ON
	setOn(true);

	// NAME
	setName("Un-named analysis.");

	// DESCRIPTION
	setDescription("No description.");

	// MODEL
	_model = aModel;

}
//_____________________________________________________________________________
/**
 * Destructor.
 */
Analysis::~Analysis()
{

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
Analysis::Analysis(const string &aFileName, bool aUpdateFromXMLNode):
IntegCallback(aFileName, false),
_inDegrees(_inDegreesProp.getValueBool())
{
	setType("Analysis");
	setNull();
	if(aUpdateFromXMLNode) updateFromXMLNode();
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * Copy constructors for all Analysis's only copy the non-XML variable
 * members of the object; that is, the object's DOMnode and XMLDocument
 * are not copied but set to NULL.  The reason for this is that for the
 * object and all its derived classes to establish the correct connection
 * to the XML document nodes, the the object would need to reconstruct based
 * on the XML document not the values of the object's member variables.
 *
 * There are three proper ways to generate an XML document for an Analysis:
 *
 * 1) Construction based on XML file (@see Analysis(const char *aFileName)).
 * In this case, the XML document is created by parsing the XML file.
 *
 * 2) Construction by Analysis(const XMLDocument *aDocument).
 * This constructor explictly requests construction based on an
 * XML document.  In this way the proper connection between an object's node
 * and the corresponding node within the XML document is established.
 * This constructor is a copy constructor of sorts because all essential
 * Analysis member variables should be held within the XML document.
 * The advantage of this style of construction is that nodes
 * within the XML document, such as comments that may not have any
 * associated Analysis member variable, are preserved.
 *
 * 3) A call to generateXMLDocument().
 * This method generates an XML document for the Analysis from scratch.
 * Only the essential document nodes are created (that is, nodes that
 * correspond directly to member variables.).
 *
 * @param aAnalysis Object to be copied.
 * @see Analysis(const XMLDocument *aDocument)
 * @see Analysis(const char *aFileName)
 * @see generateXMLDocument()
 */
Analysis::Analysis(const Analysis &aAnalysis):
IntegCallback(aAnalysis),
_inDegrees(_inDegreesProp.getValueBool())
{
	setType("Analysis");
	setNull();
	*this = aAnalysis;
}

//_____________________________________________________________________________
/**
 * virtual copy constructor
 */
Object* Analysis::
copy() const
{

	Analysis *object = new Analysis(*this);
	return(object);
}

//_____________________________________________________________________________
/**
 * Set all member variables to their null or default values.
 */
void Analysis::
setNull()
{
	setupProperties();
	_inDegrees=true;
	_storageList.setMemoryOwner(false);
}
//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void Analysis::setupProperties()
{
	_inDegreesProp.setComment("Flag (true or false) indicating whether the "
		"results are in degrees or not.");
	_inDegreesProp.setName("in_degrees");
	_propertySet.append( &_inDegreesProp );
}



//=============================================================================
// OPERATORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Assignment operator.
 *
 * @return Reference to this object.
 */
Analysis& Analysis::
operator=(const Analysis &aAnalysis)
{
	// BASE CLASS
	IntegCallback::operator=(aAnalysis);

	// Data members
	_inDegrees = aAnalysis._inDegrees;

	return(*this);
}

//=============================================================================
// GET AND SET
//=============================================================================
//-----------------------------------------------------------------------------
// IN DEGREES
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set whether or not to write the output of angles in degrees.
 * This flag must be set before an analysis is performed to ensure that
 * the results are in the proper format.
 *
 * @param aTureFalse Output will be in degrees if "true" and in radians
 * if "false".
 */
void Analysis::
setInDegrees(bool aTrueFalse)
{
	_inDegrees = aTrueFalse;
}
//_____________________________________________________________________________
/**
 * Get whether or not output of angles will be in degrees or radians.
 *
 * @return "true" if the output will be in degrees, "false" in radians.
 */
bool Analysis::
getInDegrees() const
{
	return(_inDegrees);
}
//_____________________________________________________________________________
/**
 * set pointer to model to be analyzed.
 */

void Analysis::
setModel(Model *aModel)
{
	// BASE CLASS
	IntegCallback::setModel(aModel);

	// SIDE EFFECTS


}
//-----------------------------------------------------------------------------
// COLUMN LABELS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the column labels for this analysis.
 *
 * The length of the column labels string should be 8191 characters or shorter.
 *
 * @param aLabels String of tab delimited labels.
 */
void Analysis::
setColumnLabels(const Array<string> &aLabels)
{
	_labels = aLabels;
}
//_____________________________________________________________________________
/**
 * Get the columns labels of this analysis.
 *
 * @return Labels for this analysis.
 */
const Array<string> &Analysis::
getColumnLabels() const
{
	return _labels;
}


//=============================================================================
// ANALYSIS
//=============================================================================
//_____________________________________________________________________________
/**
 * Return whether or not to proceed with this analysis.
 * The analysis will not proceed (i.e., returns false) if either the
 * analysis is turned off or if aStep is not an even multiple of
 * the step interval set @see rdStepCallback.
 *
 * @return True or False.
 */
bool Analysis::
proceed(int aStep)
{
	return(getOn() && ((aStep%_stepInterval)==0));
}
//=============================================================================
// IO
//=============================================================================
//_____________________________________________________________________________
/**
 * Print the results of the analysis.
 *
 * @param aFileName File to which to print the data.
 * @param aDT Time interval between results (linear interpolation is used).
 * If not included as an argument or negative, all time steps are printed
 * without interpolation.
 *
 * @return -1 on error, 0 otherwise.
 */
int Analysis::
printResults(const string &aBaseName,const string &aDir,double aDT,
				 const string &aExtension)
{
	printf("Analysis.printResults: Printing results of analysis %s.\n",
		getName().c_str());
	return(0);
}

ArrayPtrs<Storage>& Analysis::getStorageList()
{
	return _storageList;
}
