/* -------------------------------------------------------------------------- *
 *                         OpenSim:  TrackingTask.cpp                         *
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

//=============================================================================
// INCLUDES
//=============================================================================
#include "TrackingTask.h"
#include <OpenSim/Common/Function.h>

using namespace std;
using namespace OpenSim;
//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
TrackingTask::~TrackingTask()
{
    if(_pTrk[0]!=NULL) { delete _pTrk[0];  _pTrk[0]=NULL; }
    if(_pTrk[1]!=NULL) { delete _pTrk[1];  _pTrk[1]=NULL; }
    if(_pTrk[2]!=NULL) { delete _pTrk[2];  _pTrk[2]=NULL; }
    if(_vTrk[0]!=NULL) { delete _vTrk[0];  _vTrk[0]=NULL; }
    if(_vTrk[1]!=NULL) { delete _vTrk[1];  _vTrk[1]=NULL; }
    if(_vTrk[2]!=NULL) { delete _vTrk[2];  _vTrk[2]=NULL; }
    if(_aTrk[0]!=NULL) { delete _aTrk[0];  _aTrk[0]=NULL; }
    if(_aTrk[1]!=NULL) { delete _aTrk[1];  _aTrk[1]=NULL; }
    if(_aTrk[2]!=NULL) { delete _aTrk[2];  _aTrk[2]=NULL; }
}
//_____________________________________________________________________________
/**
 * Construct a default track object for a specified model.
 */
TrackingTask::TrackingTask():
_on(_propOn.getValueBool()),
_w(_propW.getValueDblArray())
{
    setNull();
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aTask Task object to be copied.
 */
TrackingTask::TrackingTask(const TrackingTask& aTask) :
    Object(aTask),
    _on(_propOn.getValueBool()),
    _w(_propW.getValueDblArray())
{
    setNull();
    copyData(aTask);
}


//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set NULL values for all member variables.
 */
void TrackingTask::
setNull()
{
    setName(DEFAULT_NAME);
    setupProperties();

    _model = NULL;
    _nTrk = 0;
    _pTrk[0] = _pTrk[1] = _pTrk[2] = NULL;
    _vTrk[0] = _vTrk[1] = _vTrk[2] = NULL;
    _aTrk[0] = _aTrk[1] = _aTrk[2] = NULL;
}
//_____________________________________________________________________________
/**
 * Set up the properties.
 */
void TrackingTask::
setupProperties()
{
    _propOn.setComment("Flag (true or false) indicating whether or not a task is enabled.");
    _propOn.setName("on");
    _propOn.setValue(true);
    _propertySet.append(&_propOn);

    Array<double> weight(1.0,3);
    _propW.setComment("Weight with which a task is tracked relative to other tasks. "
        "To track a task more tightly, make the weight larger.");
    _propW.setName("weight");
    _propW.setValue(weight);
    _propertySet.append(&_propW);
}

//_____________________________________________________________________________
/**
 * Copy the member data for this class only.
 *
 * @param aTask Object whose data is to be copied.
 */
void TrackingTask::
copyData(const TrackingTask &aTask)
{
    _model = aTask.getModel();
    setOn(aTask.getOn());
    _w = aTask._w;
}


//=============================================================================
// OPERATORS
//=============================================================================
//-----------------------------------------------------------------------------
// ASSIGNMENT
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Assignment operator.
 *
 * @return  Reference to the altered object.
 */
TrackingTask& TrackingTask::
operator=(const TrackingTask &aTask)
{
    // BASE CLASS
    Object::operator =(aTask);

    // DATA
    copyData(aTask);

    return(*this);
}


//=============================================================================
// GET AND SET
//=============================================================================
//-----------------------------------------------------------------------------
// MODEL
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the model to which this track object applies.
 *
 * @param aModel Model.
 */
void TrackingTask::
setModel(Model& aModel)
{
    _model = &aModel;
}

//_____________________________________________________________________________
/**
 * Get the model to which this track object applies.
 *
 * @return Pointer to the model.
 */
Model* TrackingTask::
getModel() const
{
    return(_model);
}

//-----------------------------------------------------------------------------
// ON/OFF
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Turn this track object on or off.
 *
 * @param aTureFalse Turns analysis on if "true" and off if "false".
 */
void TrackingTask::
setOn(bool aTrueFalse)
{
    _on = aTrueFalse;
}
//_____________________________________________________________________________
/**
 * Get whether or not this track object is on.
 *
 * @return True if on, false if off.
 */
bool TrackingTask::
getOn() const
{
    return(_on);
}

//-----------------------------------------------------------------------------
// WEIGHTS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the weight of each track goal.
 *
 * @param aW0 Weight for track goal 0.
 * @param aW1 Weight for track goal 1.
 * @param aW2 Weight for track goal 2.
 */
void TrackingTask::
setWeight(double aW0,double aW1,double aW2)
{
    _w[0] = aW0;
    _w[1] = aW1;
    _w[2] = aW2;
}
//_____________________________________________________________________________
/**
 * Get the weight of each track goal.
 *
 * @param aWhich Number of the track goal in question.
 * @return Weight.
 */
double TrackingTask::
getWeight(int aWhich) const
{
    if(aWhich<0) return(0.0);
    if(aWhich>2) return(0.0);
    return(_w[aWhich]);
}

//-----------------------------------------------------------------------------
// NUMBER OF TRACK FUNCTIONS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the number of position track functions.
 *
 * @return Number of position track functions.
 */
int TrackingTask::
getNumTaskFunctions() const
{
    return(_nTrk);
}

//_____________________________________________________________________________
/**
 * Set the track functions.  Note that this method makes copies of the
 * specified track functions, so the caller may use the specified functions
 * for whatever purposes.
 *
 * @param aF0 Function for track goal 0.
 * @param aF1 Function for track goal 1.
 * @param aF2 Function for track goal 2.
 */
void TrackingTask::setTaskFunctions(Function *aF0,
                 Function *aF1,Function *aF2)
{
    if(_pTrk[0]!=NULL) { delete _pTrk[0];  _pTrk[0]=NULL; }
    if(_pTrk[1]!=NULL) { delete _pTrk[1];  _pTrk[1]=NULL; }
    if(_pTrk[2]!=NULL) { delete _pTrk[2];  _pTrk[2]=NULL; }

    if(aF0!=NULL) _pTrk[0] = aF0->clone();
    if(aF1!=NULL) _pTrk[1] = aF1->clone();
    if(aF2!=NULL) _pTrk[2] = aF2->clone();
}