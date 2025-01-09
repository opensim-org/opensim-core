/* -------------------------------------------------------------------------- *
 *                           OpenSim:  Analysis.cpp                           *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Frank C. Anderson, Ajay Seth                                    *
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
 * Author: Frank C. Anderson, Ajay Seth 
 */


//=============================================================================
// INCLUDES
//=============================================================================
#include "Analysis.h"
#include "OpenSim/Common/XMLDocument.h"



using namespace OpenSim;
using namespace std;
//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/*
 * Default constructor.
 *
 * param: aModel Model on which the analysis is to be performed.
 */
Analysis::Analysis(Model *aModel):
    _statesStore(NULL),
    _inDegrees(_inDegreesProp.getValueBool()),
    _stepInterval(_stepIntervalProp.getValueInt()),
    _on(_onProp.getValueBool()),
    _startTime(_startTimeProp.getValueDbl()),
    _endTime(_endTimeProp.getValueDbl())
{
    
    _model = aModel;
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
/*
 * Construct an object from file.
 *
 * The object is constructed from the root element of the XML document.
 * The type of object is the tag name of the XML root element.
 *
 * @param aFileName File name of the document.
 */
Analysis::Analysis(const string &aFileName, bool aUpdateFromXMLNode):
    Object(aFileName, false),
    _statesStore(NULL),
    _inDegrees(_inDegreesProp.getValueBool()),
    _stepInterval(_stepIntervalProp.getValueInt()),
    _on(_onProp.getValueBool()),
    _startTime(_startTimeProp.getValueDbl()),
    _endTime(_endTimeProp.getValueDbl())
{
    setNull();
    SimTK::Xml::Element e = updDocument()->getRootDataElement(); 
    if(aUpdateFromXMLNode) updateFromXMLNode(e, getDocument()->getDocumentVersion());
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * Copy constructors for all Analysis's only copy the non-XML variable
 * members of the object; that is, the object's DOMnode and XMLDocument
 * are not copied but set to NULL.  The reason for this is that for the
 * object and all its derived classes to establish the correct connection
 * to the XML document nodes, the object would need to reconstruct based
 * on the XML document not the values of the object's member variables.
 *
 * There are three proper ways to generate an XML document for an Analysis:
 *
 * 1) Construction based on XML file (@see Analysis(const char *aFileName)).
 * In this case, the XML document is created by parsing the XML file.
 *
 * 2) Construction by Analysis(const XMLDocument *aDocument).
 * This constructor explicitly requests construction based on an
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
   Object(aAnalysis),
   _statesStore(NULL),
   _inDegrees(_inDegreesProp.getValueBool()),
   _stepInterval(_stepIntervalProp.getValueInt()),
   _on(_onProp.getValueBool()),
   _startTime(_startTimeProp.getValueDbl()),
   _endTime(_endTimeProp.getValueDbl())
{
    setNull();
    *this = aAnalysis;
}

//_____________________________________________________________________________
/**
 * Set all member variables to their null or default values.
 */
void Analysis::
setNull()
{
    setupProperties();
    _stepInterval = 1;
    _on = true;
    _model = NULL;
    _startTime = -SimTK::Infinity;
    _endTime = SimTK::Infinity;
    _inDegrees=true;
    _storageList.setMemoryOwner(false);
    _printResultFiles=true;
}
//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void Analysis::setupProperties()
{
    _onProp.setComment("Flag (true or false) specifying whether on. "
        "True by default.");
    _onProp.setName("on");
    _propertySet.append(&_onProp);

    _startTimeProp.setComment("Start time.");
    _startTimeProp.setName("start_time");
    _propertySet.append(&_startTimeProp );

    _endTimeProp.setComment("End time.");
    _endTimeProp.setName("end_time");
    _propertySet.append(&_endTimeProp );

    _stepIntervalProp.setComment("Specifies how often to store results during a simulation. "
        "More specifically, the interval (a positive integer) specifies how many successful "
        "integration steps should be taken before results are recorded again.");
    _stepIntervalProp.setName("step_interval");
    _propertySet.append( &_stepIntervalProp );

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
    Object::operator=(aAnalysis);

    // Data members
    _model = aAnalysis._model;
    _on = aAnalysis._on;
    _startTime = aAnalysis._startTime;
    _endTime = aAnalysis._endTime;

    _inDegrees = aAnalysis._inDegrees;
    _printResultFiles = aAnalysis._printResultFiles;

    // Class Members
    setStepInterval(aAnalysis.getStepInterval());

    return(*this);
}
//_____________________________________________________________________________
/**
 * Return whether or not to proceed with this callback.
 * The callback will not proceed (i.e., returns false) if either the
 * analysis is turned off or if aStep is not an even multiple of
 * the step interval.
 *
 * @return True or False.
 */
bool Analysis::
proceed(int aStep)
{
    return(getOn() && ((aStep%_stepInterval)==0));
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
setModel(Model& aModel)
{
    // BASE CLASS
    _model = &aModel;

}
//_____________________________________________________________________________
/**
 * set pointer to states storage to be analyzed.
 */

void Analysis::
setStatesStore(const Storage& aStatesStore)
{
    // BASE CLASS
    _statesStore = &aStatesStore;

}
//-----------------------------------------------------------------------------
// COLUMN LABELS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/*
 * Set the column labels for this analysis.
 * @param aLabels an Array of strings (labels).
 */
void Analysis::
setColumnLabels(const OpenSim::Array<string> &aLabels)
{
    _labels = aLabels;
}
//_____________________________________________________________________________
/**
 * Get the columns labels of this analysis.
 *
 * @return Labels for this analysis.
 */
const OpenSim::Array<string> &Analysis::
getColumnLabels() const
{
    return _labels;
}


//=============================================================================
// IO
//=============================================================================
//_____________________________________________________________________________
/*
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

// GET AND SET
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the step interval.
 *
 * The step interval is used to specify how many integration steps must
 * go by before the Analysis::step() method is executed.
 * Specifically, unless the step number divided by the step interval
 * has no remainder (i.e., (step % stepInterval) == 0), the step
 * method is not executed.
 *
 * @param aStepInterval Step interval. Should be 1 or greater.
 */
void Analysis::
setStepInterval(int aStepInterval)
{
    _stepInterval = aStepInterval;
    if(_stepInterval<1) _stepInterval= 1;
}
//_____________________________________________________________________________
/**
 * Get the step interval.
 *
 * The step interval is used to specify how many integration steps must
 * go by before the Analysis::step() method is executed.
 * Specifically, unless the step number divided by the step interval
 * has no remainder (i.e., (step % stepInterval) == 0), the step
 * method is not executed.
 *
 * @return Step interval.
 */
int Analysis::
getStepInterval() const
{
    return(_stepInterval);
}
//-----------------------------------------------------------------------------
// ON/OFF
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Turn this callback on or off.
 *
 * @param aTureFalse Turns analysis on if "true" and off if "false".
 */
void Analysis::
setOn(bool aTrueFalse)
{
    _on = aTrueFalse;
}
//_____________________________________________________________________________
/**
 * Get whether or not this analysis is on.
 *
 * @return True if on, false if off.
 */
bool Analysis::
getOn() const
{
    return(_on);
}

//-----------------------------------------------------------------------------
// START TIME
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the time at which to begin executing the callback.  Note that the start
 * time should be specified in normalized time units, not in real time units.
 *
 * @param aStartTime Start time expressed in NORMALIZED time units.
 */
void Analysis::
setStartTime(double aStartTime)
{
    _startTime = aStartTime;
}
//_____________________________________________________________________________
/**
 * Get the time at which to begin executing the callback, expressed in
 * normalized time units, not real time units.
 *
 * @return Start time expressed in NORMALIZED time units.
 */
double Analysis::
getStartTime() const
{
    return(_startTime);
}

//-----------------------------------------------------------------------------
// END TIME
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the time at which to end executing the callback.  Note that the end time
 * should be specified in normalized time units, not in real time units.
 *
 * @param aEndTime Time at which the callback should end execution in
 * NORMALIZED time units.
 */
void Analysis::
setEndTime(double aEndTime)
{
    _endTime = aEndTime;
}
//_____________________________________________________________________________
/**
 * Get the time at which to end executing the callback, expressed in
 * normalized time units, not real time units.
 *
 * @return End time expressed in NORMALIZED time units.
 */
double Analysis::
getEndTime() const
{
    return(_endTime);
}

//=============================================================================
//_____________________________________________________________________________
/**
 * This method is called at the beginning of an integration and is intended
 * to be used for any initializations that are necessary.
 *
 * Override this method in derived classes.
 *
 * @param s SimTK state 
 *
 * @return -1 on error, 0 otherwise.
 */
int Analysis::begin(const SimTK::State& s )
{
    //printf("Analysis.begin: %s.\n",getName());
    return (0);
}
//_____________________________________________________________________________
/**
 * This method is called after each successful integration time step and is
 * intended to be used for conducting analyses, driving animations, etc.
 *
 * Override this method in derived classes.
 *
 * @param s SimTK State 
 *
 * @return -1 on error, 0 otherwise.
 */
int Analysis:: 
step( const SimTK::State& s, int stepNumber )
{
    //printf("Analysis.step: %s.\n",getName());
    return (0);
}
//_____________________________________________________________________________
/**
 * This method is called after an integration has been completed and is
 * intended to be used for performing any finalizations necessary.
 *
 * Override this method in derived classes.
 *
 * @param s SimTK State 
 *
 * @return -1 on error, 0 otherwise.
 */
int Analysis::end(const SimTK::State& s )
{
    //printf("Analysis.end: %s.\n",getName());
    return(0);
}
