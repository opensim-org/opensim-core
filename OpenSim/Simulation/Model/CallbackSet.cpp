// CallbackSet.cpp
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
//#include <Object.h>
#include "CallbackSet.h"
#include "Model.h"

//=============================================================================
// STATICS
//=============================================================================


//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________


using namespace OpenSim;
/**
 * Destructor.
 * Note that the individual callbacks are not deleted by
 * this destructor.  To delete the callbacks, the caller must do so
 * individually, or the method Callback::deleteCallbacks() may be called.
 */
CallbackSet::~CallbackSet()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
CallbackSet::CallbackSet(Model *aModel)
{
	setType("CallbackSet");
	setNull();
}


//=============================================================================
// CONSTRUCTION AND DESTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the data members of this actuator to their null values.
 */
void CallbackSet::
setNull()
{
	_model = NULL;
}


//=============================================================================
// GET AND SET
//=============================================================================
//-----------------------------------------------------------------------------
// MODEL
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get a pointer to the model which is actuated.
 *
 * @return Pointer to the model.
 */
Model* CallbackSet::
getModel()
{
	return(_model);
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
void CallbackSet::
setOn(bool aTrueFalse)
{
	int i;
	Callback *callback;
	for(i=0;i<getSize();i++) {
		callback = get(i);
		if(callback==NULL) continue;
		callback->setOn(aTrueFalse);
	}
}
