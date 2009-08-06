// SimpleFeedbackController.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Contributors: Chand T. John, Samuel R. Hamner, Ajay Seth
//
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
#ifndef SimpleFeedbackController_h__
#define SimpleFeedbackController_h__

//============================================================================
// INCLUDE
//============================================================================
#include "osimToolsDLL.h"
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Control/Controller.h>

#ifdef SWIG
	#ifdef OSIMTOOLS_API
		#undef OSIMTOOLS_API
		#define OSIMTOOLS_API
	#endif
#endif

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * This is a simple feedback controller to be used in generating a forward
 * dynamic simulation.
 *
 * @author Chand T. John, Samuel R. Hamner, Ajay Seth
 * @version 1.0
 */

class OSIMTOOLS_API SimpleFeedbackController : public Controller
{
//=============================================================================
// DATA
//=============================================================================
protected:
	/** Gain for position errors.  This property can be specified in a
	 *  setup file. */
	PropertyDbl _kpProp;
	double &_kp;
	/** Gain for velocity errors.  This property can be specified in a
	 *  setup file. */
	PropertyDbl _kvProp;
	double &_kv;

	/** Input controls for the simulation. */
	ControlSet *_controlSet;
	/** States for the simulation. */
	Storage *_yDesStore;


//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	SimpleFeedbackController();
	SimpleFeedbackController(Model *aModel, ControlSet *aControlSet, Storage *aYDesStore);
	SimpleFeedbackController(const std::string &aFileName);
	SimpleFeedbackController(const SimpleFeedbackController &aController);
	virtual ~SimpleFeedbackController();
	virtual Object* copy() const;
private:
	void setNull();
protected:
	void setupProperties();
	void copyData(const SimpleFeedbackController &aController);

public:
	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
	double getKp() const;
	void setKp(double aKp);
	double getKv() const;
	void setKv(double aKv);
	ControlSet* getControlSet() const;
	virtual void setControlSet(const ControlSet &aControlSet);
	Storage* getDesiredStatesStorage() const;
	virtual void setDesiredStatesStorage(Storage *aYDesStore);

	//--------------------------------------------------------------------------
	// COMPUTATION
	//--------------------------------------------------------------------------
	virtual void
		computeControls(double &rDT,double aT,const double *aY,ControlSet &rX);


//=============================================================================
};	// END of class SimpleFeedbackController
//=============================================================================
//=============================================================================

}; // end namespace

#endif // SimpleFeedbackController_h__


