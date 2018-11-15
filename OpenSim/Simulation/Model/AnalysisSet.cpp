/* -------------------------------------------------------------------------- *
 *                         OpenSim:  AnalysisSet.cpp                          *
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


//=============================================================================
// INCLUDES
//=============================================================================
#include "AnalysisSet.h"


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
 * @param aTrueFalse Argument that, if true, results in all callbacks
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
void AnalysisSet::begin(const SimTK::State& s )
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
void AnalysisSet:: end(const SimTK::State& s)
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
