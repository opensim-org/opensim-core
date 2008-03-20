// rdCMC_Task.cpp
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
//
// This software, originally developed by Realistic Dynamics, Inc., was
// transferred to Stanford University on November 1, 2006.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//=============================================================================
// INCLUDES
//=============================================================================
#include <string>
#include "rdCMC_Task.h"
#include <OpenSim/Common/rdMath.h>
#include <OpenSim/Common/Mtx.h>
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
using SimTK::Vec3;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
rdCMC_Task::~rdCMC_Task()
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
rdCMC_Task::rdCMC_Task() :
	_on(_propOn.getValueBool()),
	_wrtBody(_propWRTBody.getValueInt()),
	_expressBody(_propExpressBody.getValueInt()),
	_active(_propActive.getValueBoolArray()),
	_w(_propW.getValueDblArray()),
	_kp(_propKP.getValueDblArray()),
	_kv(_propKV.getValueDblArray()),
	_ka(_propKA.getValueDblArray()),
	_r0(_propR0.getValueDblVec3()),
	_r1(_propR1.getValueDblVec3()),
	_r2(_propR2.getValueDblVec3())
{
	setNull();
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aTask Task object to be copied.
 */
rdCMC_Task::rdCMC_Task(const rdCMC_Task &aTask) :
	Object(aTask),
	_on(_propOn.getValueBool()),
	_wrtBody(_propWRTBody.getValueInt()),
	_expressBody(_propExpressBody.getValueInt()),
	_active(_propActive.getValueBoolArray()),
	_w(_propW.getValueDblArray()),
	_kp(_propKP.getValueDblArray()),
	_kv(_propKV.getValueDblArray()),
	_ka(_propKA.getValueDblArray()),
	_r0(_propR0.getValueDblVec3()),
	_r1(_propR1.getValueDblVec3()),
	_r2(_propR2.getValueDblVec3())
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
void rdCMC_Task::
setNull()
{
	setType("rdTaskObject");
	setName(DEFAULT_NAME);
	setupProperties();

	_model = NULL;
	_wrtBody = -1;
	_expressBody = -1;
	_on = true;
	_active[0] = _active[1] = _active[2] = false;
	_w[0] = _w[1] = _w[2] = 1.0;
	_kp[0] = _kp[1] = _kp[2] = 1.0;
	_kv[0] = _kv[1] = _kv[2] = 0.5;
	_ka[0] = _ka[1] = _ka[2] = 1.0;
	_r0[0] = _r0[1] = _r0[2] = 0.0;
	_r1[0] = _r1[1] = _r1[2] = 0.0;
	_r2[0] = _r2[1] = _r2[2] = 0.0;
	_nTrk = 0;
	_pTrk[0] = _pTrk[1] = _pTrk[2] = NULL;
	_vTrk[0] = _vTrk[1] = _vTrk[2] = NULL;
	_aTrk[0] = _aTrk[1] = _aTrk[2] = NULL;
	_pErrLast[0] = _pErrLast[1] = _pErrLast[2] = 0.0;
	_pErr[0] = _pErr[1] = _pErr[2] = 0.0;
	_vErrLast[0] = _vErrLast[1] = _vErrLast[2] = 0.0;
	_vErr[0] = _vErr[1] = _vErr[2] = 0.0;
	_aDes[0] = _aDes[1] = _aDes[2] = 0.0;
	_a[0] = _a[1] = _a[2] = 0.0;
	_j = NULL;
	_m = NULL;
}
//_____________________________________________________________________________
/**
 * Set up the properties.
 */
void rdCMC_Task::
setupProperties()
{
	_propOn.setComment("Flag (true or false) indicating whether or not a task is on.");
	_propOn.setName("on");
	_propOn.setValue(true);
	_propertySet.append(&_propOn);

	_propWRTBody.setComment("Body frame with respect to which a tracking objective is specified. "
		"This property is not used for tracking joint angles.");
	_propWRTBody.setName("wrt_body");
	_propWRTBody.setValue(-1);
	_propertySet.append(&_propWRTBody);

	_propExpressBody.setComment("Specifies the body frame in which the tracking "
		"objectives are expressed.  This property is not used for tracking joint angles.");
	_propExpressBody.setName("express_body");
	_propExpressBody.setValue(-1);
	_propertySet.append(&_propExpressBody);

	Array<bool> active(false,3);
	_propActive.setComment("Array of 3 flags (each true or false) specifying whether a "
		"component of a task is active.  For example, tracking the trajectory of a point "
		"in space could have three components (x,y,z).  This allows each of those to be "
		"made active (true) or inactive (false).  A task for tracking a joint coordinate only "
		"has one component.");
	_propActive.setName("active");
	_propActive.setValue(active);
	_propertySet.append(&_propActive);

	Array<double> weight(1.0,3);
	_propW.setComment("Weight with which a task is tracked relative to other tasks. "
		"To track a task more tightly, make the weight larger.");
	_propW.setName("weight");
	_propW.setValue(weight);
	_propertySet.append(&_propW);

	Array<double> kp(1.0,3);
	_propKP.setComment("Position error feedback gain (stiffness). "
		"To achieve critical damping of errors, choose kv = 2*sqrt(kp).");
	_propKP.setName("kp");
	_propKP.setValue(kp);
	_propertySet.append(&_propKP);

	Array<double> kv(1.0,3);
	_propKV.setComment("Velocity error feedback gain (damping). "
		"To achieve critical damping of errors, choose kv = 2*sqrt(kp).");
	_propKV.setName("kv");
	_propKV.setValue(kv);
	_propertySet.append(&_propKV);

	Array<double> ka(1.0,3);
	_propKA.setComment("Feedforward acceleration gain.  "
		"This is normally set to 1.0, so no gain.");
	_propKA.setName("ka");
	_propKA.setValue(ka);
	_propertySet.append(&_propKA);

	Vec3 r(0.0);
	_propR0.setComment("Direction vector[3] for component 0 of a task. "
		"Joint tasks do not use this propery.");
	_propR0.setName("r0");
	_propR0.setValue(r);
	_propertySet.append(&_propR0);

	_propR1.setComment("Direction vector[3] for component 1 of a task. "
		"Joint tasks do not use this property.");
	_propR1.setName("r1");
	_propR1.setValue(r);
	_propertySet.append(&_propR1);

	_propR2.setComment("Direction vector[3] for component 2 of a task. "
		"Joint tasks do not use this property.");
	_propR2.setName("r2");
	_propR2.setValue(r);
	_propertySet.append(&_propR2);
}

//_____________________________________________________________________________
/**
 * Copy the member data for this class only.
 *
 * @param aTask Object whose data is to be copied.
 */
void rdCMC_Task::
copyData(const rdCMC_Task &aTask)
{
	int i;
	_model = aTask.getModel();
	setWRTBody(aTask.getWRTBody());
	setExpressBody(aTask.getExpressBody());
	setOn(aTask.getOn());
	for(i=0;i<3;i++) _active[i] = aTask.getActive(i);
	for(i=0;i<3;i++) _kp[i] = aTask.getKP(i);
	for(i=0;i<3;i++) _kv[i] = aTask.getKV(i);
	for(i=0;i<3;i++) _ka[i] = aTask.getKA(i);
	aTask.getDirection_0(_r0);
	aTask.getDirection_1(_r1);
	aTask.getDirection_2(_r2);

	// FUNCTIONS
	const Function *func;
	for(i=0;i<3;i++) {
		// position
		if(_pTrk[i]!=NULL) {delete _pTrk[i];  _pTrk[i]=NULL; }
		func = aTask.getTaskFunction(i);
		if(func!=NULL) _pTrk[i] = (Function *)func->copy();
		// velocity
		if(_vTrk[i]!=NULL) {delete _vTrk[i];  _vTrk[i]=NULL; }
		func = aTask.getTaskFunctionForVelocity(i);
		if(func!=NULL) _vTrk[i] = (Function*)func->copy();
		// acceleration
		if(_aTrk[i]!=NULL) {delete _aTrk[i];  _aTrk[i]=NULL; }
		func = aTask.getTaskFunctionForAcceleration(i);
		if(func!=NULL) _aTrk[i] = (Function*)func->copy();
	}
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
rdCMC_Task& rdCMC_Task::
operator=(const rdCMC_Task &aTask)
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
void rdCMC_Task::
setModel(Model *aModel)
{
	_model = aModel;
}

//_____________________________________________________________________________
/**
 * Get the model to which this track object applies.
 *
 * @return Pointer to the model.
 */
Model* rdCMC_Task::
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
void rdCMC_Task::
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
bool rdCMC_Task::
getOn() const
{
	return(_on);
}

//-----------------------------------------------------------------------------
// WRT BODY
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the body with respect to (WRT) which the track goals are specified.
 *
 * @param aBody Body ID.
 */
void rdCMC_Task::
setWRTBody(int aBody)
{
	_wrtBody = aBody;
}
//_____________________________________________________________________________
/**
 * Get the body with respect to (WRT) which the track goals are specified.
 *
 * @return Body ID.
 */
int rdCMC_Task::
getWRTBody() const
{
	return(_wrtBody);
}

//-----------------------------------------------------------------------------
// EXPRESS BODY
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the body in which the track goals are expressed.
 *
 * @param aBody Body ID.
 */
void rdCMC_Task::
setExpressBody(int aBody)
{
	_expressBody = aBody;
}
//_____________________________________________________________________________
/**
 * Get the body in which the track goals are expressed.
 *
 * @return Body ID.
 */
int rdCMC_Task::
getExpressBody() const
{
	return(_expressBody);
}

//-----------------------------------------------------------------------------
// ACTIVE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set whether or not track goals are active.
 *
 * @param a0 Active flag for track goal 0-- true means active.
 * @param a1 Active flag for track goal 1-- true means active.
 * @param a2 Active flag for track goal 2-- true means active.
 */
void rdCMC_Task::
setActive(bool a0,bool a1,bool a2)
{
	_active[0] = a0;
	_active[1] = a1;
	_active[2] = a2;
}
//_____________________________________________________________________________
/**
 * Get whether a specified track goal is active.
 *
 * @param aWhich Number of the track goal in question.
 * @return True if the specified track goal is active, false otherwise.
 */
bool rdCMC_Task::
getActive(int aWhich) const
{
	if(aWhich<0) return(false);
	if(aWhich>2) return(false);
	return(_active[aWhich]);
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
void rdCMC_Task::
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
double rdCMC_Task::
getWeight(int aWhich) const
{
	if(aWhich<0) return(0.0);
	if(aWhich>2) return(0.0);
	return(_w[aWhich]);
}

//-----------------------------------------------------------------------------
// POSITION GAINS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the position gains for each track goal.
 *
 * @param aK0 Gain for track goal 0.
 * @param aK1 Gain for track goal 1.
 * @param aK2 Gain for track goal 2.
 */
void rdCMC_Task::
setKP(double aK0,double aK1,double aK2)
{
	_kp[0] = aK0;
	_kp[1] = aK1;
	_kp[2] = aK2;
}
//_____________________________________________________________________________
/**
 * Get the position gain for a specified track goal.
 *
 * @param aWhich Number of the track goal in question.
 * @return Position gain.
 */
double rdCMC_Task::
getKP(int aWhich) const
{
	if(aWhich<0) return(0.0);
	if(aWhich>2) return(0.0);
	return(_kp[aWhich]);
}

//-----------------------------------------------------------------------------
// VELOCITY GAINS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the velocity gains for each track goal.
 *
 * @param aK0 Gain for track goal 0.
 * @param aK1 Gain for track goal 1.
 * @param aK2 Gain for track goal 2.
 */
void rdCMC_Task::
setKV(double aK0,double aK1,double aK2)
{
	_kv[0] = aK0;
	_kv[1] = aK1;
	_kv[2] = aK2;
}
//_____________________________________________________________________________
/**
 * Get the velocity gain for a specified track goal.
 *
 * @param aWhich Number of the track goal in question.
 * @return Velocity gain.
 */
double rdCMC_Task::
getKV(int aWhich) const
{
	if(aWhich<0) return(0.0);
	if(aWhich>2) return(0.0);
	return(_kv[aWhich]);
}

//-----------------------------------------------------------------------------
// ACCELERATION GAINS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the acceleration gains for each track goal.
 *
 * @param aK0 Gain for track goal 0.
 * @param aK1 Gain for track goal 1.
 * @param aK2 Gain for track goal 2.
 */
void rdCMC_Task::
setKA(double aK0,double aK1,double aK2)
{
	_ka[0] = aK0;
	_ka[1] = aK1;
	_ka[2] = aK2;
}
//_____________________________________________________________________________
/**
 * Get the acceleration gain for a specified track goal.
 *
 * @param aWhich Number of the track goal in question.
 * @return Acceleration gain.
 */
double rdCMC_Task::
getKA(int aWhich) const
{
	if(aWhich<0) return(0.0);
	if(aWhich>2) return(0.0);
	return(_ka[aWhich]);
}

//-----------------------------------------------------------------------------
// DIRECTION 0
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the direction of track goal 0.
 *
 * @param aR Direction.  This vector is normalized.
 */
void rdCMC_Task::
setDirection_0(const SimTK::Vec3& aR)
{
	_r0 = aR; _r0.normalize();
}
//_____________________________________________________________________________
/**
 * Get the direction of track goal 0.
 *
 * @param aR Direction.
 */
void rdCMC_Task::
getDirection_0(SimTK::Vec3& rR) const
{
	rR=_r0;
}

//-----------------------------------------------------------------------------
// DIRECTION 1
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the direction of track goal 1.
 *
 * @param aR Direction.  This vector is normalized.
 */
void rdCMC_Task::
setDirection_1(const SimTK::Vec3& aR)
{
	_r1=aR; _r1.normalize();
}
//_____________________________________________________________________________
/**
 * Get the direction of track goal 1.
 *
 * @param aR Direction.
 */
void rdCMC_Task::
getDirection_1(SimTK::Vec3& rR) const
{
	rR=_r1;
}

//-----------------------------------------------------------------------------
// DIRECTION 2
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the direction of track goal 2.
 *
 * @param aR Direction.  This vector is normalized.
 */
void rdCMC_Task::
setDirection_2(const SimTK::Vec3& aR)
{
	_r2=aR;
}
//_____________________________________________________________________________
/**
 * Get the direction of track goal 2.
 *
 * @param aR Direction.
 */
void rdCMC_Task::
getDirection_2(SimTK::Vec3& rR) const
{
	rR=_r2;
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
int rdCMC_Task::
getNumTaskFunctions() const
{
	return(_nTrk);
}

//-----------------------------------------------------------------------------
// TRACK FUNCTIONS - POSITION
//-----------------------------------------------------------------------------
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
void rdCMC_Task::
setTaskFunctions(Function *aF0,Function *aF1,Function *aF2)
{
	if(_pTrk[0]!=NULL) { delete _pTrk[0];  _pTrk[0]=NULL; }
	if(_pTrk[1]!=NULL) { delete _pTrk[1];  _pTrk[1]=NULL; }
	if(_pTrk[2]!=NULL) { delete _pTrk[2];  _pTrk[2]=NULL; }

	if(aF0!=NULL) _pTrk[0] = (Function*)aF0->copy();
	if(aF1!=NULL) _pTrk[1] = (Function*)aF1->copy();
	if(aF2!=NULL) _pTrk[2] = (Function*)aF2->copy();
}
//_____________________________________________________________________________
/**
 * Get a specified track function.
 *
 * @param aWhich Specifies which track function (0, 1, or 2).
 * @return Function.
 */
Function* rdCMC_Task::
getTaskFunction(int aWhich) const
{
	if(aWhich<0) return(NULL);
	if(aWhich>2) return(NULL);
	return(_pTrk[aWhich]);
}

//-----------------------------------------------------------------------------
// TRACK FUNCTIONS - VELOCITY
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the velocity track functions.  Note that this method makes copies of the
 * specified functions, so the caller may use the specified functions
 * for whatever purposes.
 *
 * @param aF0 Function for track goal 0.
 * @param aF1 Function for track goal 1.
 * @param aF2 Function for track goal 2.
 */
void rdCMC_Task::
setTaskFunctionsForVelocity(Function *aF0,Function *aF1,Function *aF2)
{
	if(_vTrk[0]!=NULL) { delete _vTrk[0];  _vTrk[0]=NULL; }
	if(_vTrk[1]!=NULL) { delete _vTrk[1];  _vTrk[1]=NULL; }
	if(_vTrk[2]!=NULL) { delete _vTrk[2];  _vTrk[2]=NULL; }

	if(aF0!=NULL) _vTrk[0] = (Function*)aF0->copy();
	if(aF1!=NULL) _vTrk[1] = (Function*)aF1->copy();
	if(aF2!=NULL) _vTrk[2] = (Function*)aF2->copy();
}
//_____________________________________________________________________________
/**
 * Get a specified velocity track function.
 *
 * @param aWhich Specifies which track function (0, 1, or 2).
 * @return Function.
 */
Function* rdCMC_Task::
getTaskFunctionForVelocity(int aWhich) const
{
	if(aWhich<0) return(NULL);
	if(aWhich>2) return(NULL);
	return(_vTrk[aWhich]);
}

//-----------------------------------------------------------------------------
// TRACK FUNCTIONS - POSITION
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the acceleration track functions.  Note that this method makes copies of
 * the specified track functions, so the caller may use the specified functions
 * for whatever purposes.
 *
 * @param aF0 Function for track goal 0.
 * @param aF1 Function for track goal 1.
 * @param aF2 Function for track goal 2.
 */
void rdCMC_Task::
setTaskFunctionsForAcceleration(
	Function *aF0,Function *aF1,Function *aF2)
{
	if(_aTrk[0]!=NULL) { delete _aTrk[0];  _aTrk[0]=NULL; }
	if(_aTrk[1]!=NULL) { delete _aTrk[1];  _aTrk[1]=NULL; }
	if(_aTrk[2]!=NULL) { delete _aTrk[2];  _aTrk[2]=NULL; }

	if(aF0!=NULL) _aTrk[0] = (Function*)aF0->copy();
	if(aF1!=NULL) _aTrk[1] = (Function*)aF1->copy();
	if(aF2!=NULL) _aTrk[2] = (Function*)aF2->copy();
}
//_____________________________________________________________________________
/**
 * Get a specified acceleration track function.
 *
 * @param aWhich Specifies which track function (0, 1, or 2).
 * @return Function.
 */
Function* rdCMC_Task::
getTaskFunctionForAcceleration(int aWhich) const
{
	if(aWhich<0) return(NULL);
	if(aWhich>2) return(NULL);
	return(_aTrk[aWhich]);
}


//-----------------------------------------------------------------------------
// TASK KINEMATICS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the task position.
 *
 * @param aWhich Specifies which task goal (0, 1, or 2).
 * @param aT Time (in real time units).
 * @return Task position.
 * @throws Exception for an invalid task.
 */
double rdCMC_Task::
getTaskPosition(int aWhich,double aT) const
{
	if((aWhich<0)||(aWhich>=_nTrk)) {
		string msg = "rdCMC_Task: ERR- Invalid task.";
		throw( Exception(msg,__FILE__,__LINE__) );
	}
	double position = _pTrk[aWhich]->evaluate(0,aT);
	return(position);
}
//_____________________________________________________________________________
/**
 * Get the task velocity.
 *
 * @param aWhich Specifies which task goal (0, 1, or 2).
 * @param aT Time (in real time units).
 * @return Task velocity.
 * @throws Exception for an invalid task.
 */
double rdCMC_Task::
getTaskVelocity(int aWhich,double aT) const
{
	if((aWhich<0)||(aWhich>=_nTrk)) {
		string msg = "rdCMC_Task: ERR- Invalid task.";
		throw( Exception(msg,__FILE__,__LINE__) );
	}

	double velocity;
	if(_vTrk[aWhich]!=NULL) {
		velocity = _vTrk[aWhich]->evaluate(0,aT);
	} else {
		velocity = _pTrk[aWhich]->evaluate(1,aT);
	}

	return( velocity );
}//_____________________________________________________________________________
/**
 * Get the task acceleration.
 *
 * @param aWhich Specifies which task goal (0, 1, or 2).
 * @param aT Time (in real time units).
 * @return Task acceleration.
 * @throws Exception for an invalid task.
 */
double rdCMC_Task::
getTaskAcceleration(int aWhich,double aT) const
{
	if((aWhich<0)||(aWhich>=_nTrk)) {
		string msg = "rdCMC_Task: ERR- Invalid task.";
		throw( Exception(msg,__FILE__,__LINE__) );
	}

	double acceleration;
	if(_aTrk[aWhich]!=NULL) {
		acceleration = _aTrk[aWhich]->evaluate(0,aT);
	} else {
		acceleration = _pTrk[aWhich]->evaluate(2,aT);
	}

	return( acceleration );
}


//-----------------------------------------------------------------------------
// LAST POSITION ERRORS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the last achieved position error.  This information is useful for
 * checking that error dynamics are being followed.
 *
 * @param aE0 Last position error for track goal 0.
 * @param aE1 Last position error for track goal 1.
 * @param aE2 Last position error for track goal 2.
 */
void rdCMC_Task::
setPositionErrorLast(double aE0,double aE1,double aE2)
{
	_pErrLast[0] = aE0;
	_pErrLast[1] = aE1;
	_pErrLast[2] = aE2;
}
//_____________________________________________________________________________
/**
 * Get the last achieved position error.  This information is useful for
 * checking that error dynamics are being followed.
 *
 * @param aWhich Specifies which track goal (0, 1, or 2).
 * @return Last position error.
 */
double rdCMC_Task::
getPositionErrorLast(int aWhich) const
{
	if(aWhich<0) return(0.0);
	if(aWhich>2) return(0.0);
	return(_pErrLast[aWhich]);
}

//-----------------------------------------------------------------------------
// LAST VELOCITY ERRORS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the last achieved velocity error.  This information is useful for
 * checking that error dynamics are being followed.
 *
 * @param aE0 Last velocity error for track goal 0.
 * @param aE1 Last velocity error for track goal 1.
 * @param aE2 Last velocity error for track goal 2.
 */
void rdCMC_Task::
setVelocityErrorLast(double aE0,double aE1,double aE2)
{
	_vErrLast[0] = aE0;
	_vErrLast[1] = aE1;
	_vErrLast[2] = aE2;
}
//_____________________________________________________________________________
/**
 * Get the last achieved velocity error.  This information is useful for
 * checking that error dynamics are being followed.
 *
 * @param aWhich Specifies which track goal (0, 1, or 2).
 * @return Last velocity error.
 */
double rdCMC_Task::
getVelocityErrorLast(int aWhich) const
{
	if(aWhich<0) return(0.0);
	if(aWhich>2) return(0.0);
	return(_vErrLast[aWhich]);
}

//-----------------------------------------------------------------------------
// TRACK ERRORS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the position track error of a specified track goal.
 *
 * @param aWhich Specifies which track goal (0, 1, or 2).
 * @return Error
 */
double rdCMC_Task::
getPositionError(int aWhich) const
{
	if(aWhich<0) return(0.0);
	if(aWhich>2) return(0.0);
	return(_pErr[aWhich]);
}
//_____________________________________________________________________________
/**
 * Get the velocity track error of a specified track goal.
 *
 * @param aWhich Specifies which track goal (0, 1, or 2).
 * @return Error
 */
double rdCMC_Task::
getVelocityError(int aWhich) const
{
	if(aWhich<0) return(0.0);
	if(aWhich>2) return(0.0);
	return(_vErr[aWhich]);
}

//-----------------------------------------------------------------------------
// DESIRED ACCELERATION
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the desired acceleration of a specified track goal.
 * The method computeDesiredAccelerations() must be called first for the
 * values returned by this method to be valid.
 *
 * @param aWhich Specifies which track goal (0, 1, or 2).
 * @return Desired acceleration.  rdMath::NAN is returned on an error.
 */
double rdCMC_Task::
getDesiredAcceleration(int aWhich) const
{
	if(aWhich<0) return(rdMath::NAN);
	if(aWhich>2) return(rdMath::NAN);
	return(_aDes[aWhich]);
}

//-----------------------------------------------------------------------------
// ACCELERATION
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the acceleration of a specified track goal.  The acceleration returned
 * is the dot product of the appropriate track-goal direction and the
 * acceleration of the point or orientation in question.  In the case of
 * generalized coordinates, the acceleration of the generalized coordinate
 * is returned (i.e., a direction is not appropriate).
 *
 * For the value returned by this method to be valid, the method
 * computeAccelerations() must be called first.
 *
 * @param aWhich Specifies which track goal (0, 1, or 2).
 * @return Acceleration.  rdMath::NAN is returned on an error.
 */
double rdCMC_Task::
getAcceleration(int aWhich) const
{
	if(aWhich<0) return(rdMath::NAN);
	if(aWhich>2) return(rdMath::NAN);
	return(_a[aWhich]);
}


//=============================================================================
// COMPUTATIONS
//=============================================================================
//-----------------------------------------------------------------------------
// JACOBIAN
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Compute the Jacobian.
 */
void rdCMC_Task::
computeJacobian()
{
	printf("rdCMC_Task.computeJacobian: ERROR- this method should be ");
	printf("overriden in derived classes.\n");
}

//-----------------------------------------------------------------------------
// EFFECTIVE MASS MATRIX
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Compute the effective mass matrix.
 */
void rdCMC_Task::
computeEffectiveMassMatrix()
{
	printf("rdCMC_Task.computeEffectiveMassMatrix: ERROR- this method ");
	printf("should be overriden in derived classes.\n");
}


//=============================================================================
// XML
//=============================================================================
//-----------------------------------------------------------------------------
// UPDATE FROM XML NODE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Update this object based on its XML node.
 */
void rdCMC_Task::
updateFromXMLNode()
{
	Object::updateFromXMLNode();

	setWRTBody(_wrtBody);
	setExpressBody(_expressBody);
	setOn(_on);
	setActive(_active[0],_active[1],_active[2]);
	setWeight(_w[0],_w[1],_w[2]);
	setKP(_kp[0],_kp[1],_kp[2]);
	setKV(_kv[0],_kv[1],_kv[2]);
	setKA(_ka[0],_ka[1],_ka[2]);
	setDirection_0(_r0);
	setDirection_1(_r1);
	setDirection_2(_r2);
}
