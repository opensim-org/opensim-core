#ifndef TrackingTask_h__
#define TrackingTask_h__
// TrackingTask.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Contributors: Ayman Habib, Ajay Seth
//
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

// INCLUDES
#include "osimToolsDLL.h"
#include <OpenSim/Common/Object.h>
#include <OpenSim/Simulation/Model/Model.h>

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * An abstract base class for specifying a target for a tracking problem.
 *
 * @author Ayman Habib & Ajay Seth
 * @version 1.0
 */
class OSIMTOOLS_API TrackingTask : public Object
{

//=============================================================================
// DATA
//=============================================================================
protected:
	// PROPERTIES
	/** Property to indicate on or off state. */
	PropertyBool _propOn;
	/** Weights of the task goals. */
	PropertyDblArray _propW;

	/** Reference to the value of the on property. */
	bool &_on;
	/** Reference to the value of the Weight property. */
	Array<double> &_w;

	/** Model. */
	Model *_model;

	/** Number of functions for this target. */
	int _nTrk;

	/** Position task functions.  Different types of tasks can 
	require different numbers of task functions.  For example, to track
	a joint angle, only one task function is needed.  However, to track
	a position, up to three task functions may be needed. */
	Function *_pTrk[3];
	/** Velocity task functions.  If velocity task functions are
	not specified, derivatives of the position task function are used. */
	Function *_vTrk[3];
	/** Acceleration task functions.  If acceleration task functions are
	not specified, derivatives of the position task function are used. */
	Function *_aTrk[3];

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	TrackingTask();
	TrackingTask(const TrackingTask &aTaskObject);
	virtual ~TrackingTask();
	virtual Object* copy() const = 0;
private:
	void setNull();
	void setupProperties();
	void copyData(const TrackingTask &aTaskObject);

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:

#ifndef SWIG
	TrackingTask& operator=(const TrackingTask &aTaskObject);
#endif

	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
	// MODEL
	virtual void setModel(OpenSim::Model& aModel);
	Model* getModel() const;

	// ON,OFF
	void setOn(bool aTrueFalse);
	bool getOn() const;
	// WEIGHTS
	void setWeight(double aW0,double aW1=0.0,double aW2=0.0);
	void setWeights(const Array<double>& aWeights) {_w = aWeights; };
	double getWeight(int aWhich) const;
	const Array<double>& getWeights() const { return _w; };
	// TASK FUNCTIONS
	int getNumTaskFunctions() const;
	virtual void setTaskFunctions(Function *aF0,
		Function *aF1=NULL,Function *aF2=NULL);
//=============================================================================
};	// END of class TrackingTask
//=============================================================================
//=============================================================================

}; // end namespace

#endif // __TrackingTask_h__


