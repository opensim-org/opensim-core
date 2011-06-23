// TrackingTask.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c)  2010, Stanford University. All rights reserved. 
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
#include <string>
#include "TrackingTask.h"
#include <OpenSim/Common/Exception.h>
#include <OpenSim/Common/PropertyBool.h>
#include <OpenSim/Common/PropertyInt.h>
#include <OpenSim/Common/PropertyDbl.h>
#include <OpenSim/Common/PropertyBoolArray.h>
#include <OpenSim/Common/PropertyIntArray.h>
#include <OpenSim/Common/PropertyDblArray.h>
#include <OpenSim/Simulation/Model/Model.h>

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
	setType("TrackingTask");
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

	if(aF0!=NULL) _pTrk[0] = (Function*)aF0->copy();
	if(aF1!=NULL) _pTrk[1] = (Function*)aF1->copy();
	if(aF2!=NULL) _pTrk[2] = (Function*)aF2->copy();
}