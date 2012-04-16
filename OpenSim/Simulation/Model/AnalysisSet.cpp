// AnalysisSet.cpp
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


//=============================================================================
// INCLUDES
//=============================================================================
#include "AnalysisSet.h"
#include "Model.h"


using namespace OpenSim;
using namespace std;


//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________

AnalysisSet::AnalysisSet() :
 _enable(_enableProp.getValueBool())
{
	setNull();
}
/**
 * Destructor.
 * Note that the individual analyses are not deleted by
 * this destructor.  To delete the analyses, the caller must do so
 * individually.
 */
AnalysisSet::~AnalysisSet()
{
}
//_____________________________________________________________________________
/**
 * Construct an empty analysis set.
 *
 * @param aModel Model for the analysis set.
 */
AnalysisSet::AnalysisSet(Model *aModel) :
 _enable(_enableProp.getValueBool())
{
	setNull();
	_model = aModel;
}
//_____________________________________________________________________________
/**
 * Construct an analysis set from file.
 *
 * @param aFileName Name of the file.
 */
AnalysisSet::AnalysisSet(const string &aFileName) :
	Set<Analysis>(aFileName, false),
 _enable(_enableProp.getValueBool())
{
	setNull();
	updateFromXMLDocument();
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aSet Analysis set to be copied.
 */
AnalysisSet::AnalysisSet(const AnalysisSet &aSet) :
	Set<Analysis>(aSet),
    _enable(_enableProp.getValueBool())
{
	setNull();
}


//=============================================================================
// CONSTRUCTION AND DESTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the data members of this actuator to their null values.
 */
void AnalysisSet::
setNull()
{
    _enable = true;
}
void AnalysisSet::
setupProperties() {
    
    _enableProp.setComment("enable/disable for AnalysisSet");
    _enableProp.setName("enable");
//    _propertySet.append( &_enableProp );
}

AnalysisSet& AnalysisSet::
operator=(const  AnalysisSet &aSet)
{
     Set<Analysis>::operator=(aSet);
 
     _enable = aSet._enable;
     return(*this);
}
//=============================================================================
// GET AND SET
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the model for all analyses in the set.
 *
 * @param aModel Pointer to the model.
 */
void AnalysisSet::
setModel(Model& aModel)
{
	int i;
	int size = getSize();
	for(i=0;i<size;i++) {
		Analysis& analysis = get(i);
		analysis.setModel(aModel);
	}
}
//_____________________________________________________________________________
/**
 * Get a pointer to the model which is actuated.
 *
 * @return Pointer to the model.
 */
Model& AnalysisSet::
getModel()
{
	return(*_model);
}

//-----------------------------------------------------------------------------
// ON & OFF
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set all the callbacks either on or off.
 *
 * @param aTrueFalse Arguement that, if true, results in all callbacks
 * being turned on; if false, all callbacks are turned off.
 */
void AnalysisSet::
setOn(bool aTrueFalse)
{
	for(int i=0;i<getSize();i++) get(i).setOn(aTrueFalse);
}

void AnalysisSet::
setOn(const OpenSim::Array<bool> &aOn) 
{
	if(aOn.getSize()!=getSize()) throw Exception("AnalysisSet.setOn: ERROR- incompatible array sizes",__FILE__,__LINE__);
	for(int i=0; i<getSize(); i++) get(i).setOn(aOn[i]);
}

OpenSim::Array<bool> AnalysisSet::
getOn() const
{
	Array<bool> on(false,getSize());
	for(int i=0; i<getSize(); i++) on[i] = get(i).getOn();
	return on;
}


//=============================================================================
// CALLBACKS
//=============================================================================
//_____________________________________________________________________________
/**
 * Call the begin method for all integration callbacks.  This method is
 * called at the beginning of an integration and is intended to be used for
 * any initializations that are necessary.
 *
 * @param s Current state 
 */
void AnalysisSet::
begin(SimTK::State& s )
{
	int i;
	for(i=0;i<getSize();i++) {
		Analysis& analysis = get(i);
		if (analysis.getOn()) analysis.begin(s);
	}
}
//_____________________________________________________________________________
/**
 * Call the step method for all integration callbacks.  This method is called
 * after each successful integration time step and is intended to be used for
 * conducting analyses, driving animations, etc.
 *
 * @param s Current state 
 */
void AnalysisSet::
step( const SimTK::State& s, int stepNumber )
{
	int i;
	for(i=0;i<getSize();i++) {
		Analysis& analysis = get(i);
		if (analysis.getOn()) analysis.step(s, stepNumber);
	}
}
//_____________________________________________________________________________
/**
 * Call the end method for all integration callbacks.  This method is called
 * after an integration has been completed and is intended to be used for
 * performing any finalizations necessary.
 *
 * @param s Current state 
 */
void AnalysisSet::
end(SimTK::State& s)
{
	int i;
	for(i=0;i<getSize();i++) {
		Analysis& analysis = get(i);
		if (analysis.getOn()) analysis.end(s);
	}
}



//=============================================================================
// RESULTS
//=============================================================================
//_____________________________________________________________________________
/**
 * Print the results of all analyses in the set.
 *
 * @param aIndex Array index of the callback to be returned.
 * @return Callback at index aIndex.
 */
void AnalysisSet::
printResults(const string &aBaseName,const string &aDir,double aDT,
				 const string &aExtension)
{
	int i;
	int size = getSize();
	for(i=0;i<size;i++) {
		Analysis& analysis = get(i);
		if(analysis.getOn() && analysis.getPrintResultFiles()) analysis.printResults(aBaseName,aDir,aDT,aExtension);
	}
}
//=============================================================================
// UTILITY
//=============================================================================
//_____________________________________________________________________________
/**
 * Fills the passed in analysis set with all available (i.e. registered) analyses.
 */
void AnalysisSet::
getAvailableAnalyses(AnalysisSet& as)
{
	Object::getRegisteredObjectsOfGivenType<Analysis>(as._objects);
}
