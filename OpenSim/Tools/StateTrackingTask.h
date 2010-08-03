#ifndef StateTrackingTask_h__
#define StateTrackingTask_h__
// StateTrackingTask.h
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
#include "TrackingTask.h"

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * A target for a tracking problem that corresponds to a state variable.
 *
 * @author Ayman Habib & Ajay Seth
 * @version 1.0
 */
class OSIMTOOLS_API StateTrackingTask : public TrackingTask
{

//=============================================================================
// DATA
//=============================================================================
protected:

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	StateTrackingTask() { setNull(); };
	StateTrackingTask(const StateTrackingTask &aTaskObject) :
	TrackingTask(aTaskObject)
	{
		setNull();
		copyData(aTaskObject);
	};
	virtual ~StateTrackingTask() {};
private:
	void setNull() {_stateIndex=-1;};
	void copyData(const StateTrackingTask &aTaskObject) {};

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:

#ifndef SWIG
	StateTrackingTask& operator=(const StateTrackingTask &aTaskObject) {
	// BASE CLASS
	TrackingTask::operator =(aTaskObject);
	// DATA
	copyData(aTaskObject);

	return(*this);
	}
#endif
	virtual double getTaskError(const SimTK::State& s) {
		if (_stateIndex==-1){
			Array<std::string> stateNames;
			_model->getStateNames(stateNames, true);	// Since we'll use YIndex we need to account for internal SimTK states too
			_stateIndex = stateNames.findIndex(getName());
		}
		// map State name to Y index
		double time = s.getTime();
		if (_stateIndex!=-1){
			//std::cout << "target emg=" << _pTrk[0]->calcValue(SimTK::Vector(1,time)) << "at time ="<< time << std::endl;
			//for(double t=0.0; t <= 1.0; t+=0.1)
			//	std::cout<< _pTrk[0]->calcValue(SimTK::Vector(1,t));
			return (_pTrk[0]->calcValue(SimTK::Vector(1,time))-s.getY()[_stateIndex]);
		}
		throw Exception("StateTrackingTask::getTaskError(): State name not found"+getName());
		return(0.);
	}
	/**
	 * Return the gradient of the tracking error as a vector, whose length 
	 * is the number of _controller->getModel().getActuators()
	 */
	virtual SimTK::Vector getTaskErrorGradient(const SimTK::State& s) {
		const Set<Actuator>& fSet = _model->getActuators();
		SimTK::Vector g = SimTK::Vector(fSet.getSize());
		g = 0.;
		double taskGradient = -2 * getTaskError(s)* getWeight(0);
		// set the entry idx in g where fSet[idx] is a substring of getName()
		for(int i=0; i< fSet.getSize(); i++){
			if (getName().find_first_of(fSet[i].getName())!=NULL){
				g[i]=taskGradient;
				break;
			}
		}
		return g;
	}
	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
private:
	int _stateIndex;	// YIndex of state being tracked in _model, -1 if not found
//=============================================================================
};	// END of class StateTrackingTask
//=============================================================================
//=============================================================================

}; // end namespace

#endif // __StateTrackingTask_h__


