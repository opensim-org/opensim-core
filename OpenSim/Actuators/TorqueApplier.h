#ifndef _TorqueApplier_h_
#define _TorqueApplier_h_
// TorqueApplier.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Frank C. Anderson, May Q. Liu
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
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


//=============================================================================
// INCLUDES
//=============================================================================
#include <OpenSim/Common/rdMath.h>
#include <OpenSim/Simulation/Model/DerivCallback.h>
#include <OpenSim/Simulation/Manager/Manager.h>
#include <OpenSim/Common/VectorFunction.h>
#include <OpenSim/Common/FunctionSet.h>
#include "osimActuatorsDLL.h"

//=============================================================================
//=============================================================================
/**
 * A derivatives callback used for applying external torques during a
 * simulation.
 *
 * @author Frank C. Anderson, May Q. Liu
 * @version 1.0
 */
namespace OpenSim { 

class Model;
class AbstractBody;

class OSIMACTUATORS_API TorqueApplier : public DerivCallback 
{
//=============================================================================
// DATA
//=============================================================================
protected:
	/** Which body segment. */
	AbstractBody* _body;
	/** Torque to be applied. */
	SimTK::Vec3 _torque;
	/** Vector function containing torque to be applied (t,x,y,z). */
	VectorFunction* _torqueFunction;
	/** Flag to set reference frame of input torque */
	bool _inputTorquesInGlobalFrame;
	/** Flag to indicate whether or not to record the loads that are applied
	during an integration.  Recording these loads takes a lot of memory as they
	are stored every time the derivatives are evaluated (e.g., 6 times per
	integration step). */
	bool _recordAppliedLoads;
	/** Storage for the torque that was actually applied during the simulation */
	Storage *_appliedTorqueStore;

//=============================================================================
// METHODS
//=============================================================================
public:
	TorqueApplier(Model *aModel,AbstractBody *aBody);
	TorqueApplier(Model *aModel,AbstractBody *bodyFrom,AbstractBody *bodyTo,
		Storage *torqueData,int txNum, int tyNum, int tzNum,
		double firstTime, double lastTime);
	virtual ~TorqueApplier();
private:
	void setNull();
	void constructDescription();
	void constructColumnLabels();
	void allocateStorage();
	void deleteStorage();

public:
	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
	void setBody(AbstractBody *aBody);
	AbstractBody* getBody() const;
	void setTorque(const SimTK::Vec3& aTorque);
	void getTorque(SimTK::Vec3& rPoint) const;

	void setTorqueFunction(VectorFunction* aTorqueFunction);
	VectorFunction* getTorqueFunction() const;

	void setRecordAppliedLoads(bool aTrueFalse);
	bool getRecordAppliedLoads() const;
	Storage* getAppliedTorqueStorage();
	void setStorageCapacityIncrements(int aIncrement);

	virtual void reset(); 

	void setInputTorquesInGlobalFrame(bool aTrueFalse);
	bool getInputTorquesInGlobalFrame() const;
	
	//--------------------------------------------------------------------------
	// CALLBACKS
	//--------------------------------------------------------------------------
	virtual void
		applyActuation(double aT,double *aX,double *aY);

	//--------------------------------------------------------------------------
	// IO
	//--------------------------------------------------------------------------
public:
	virtual int
		printResults(const std::string &aBaseName,const std::string &aDir="",
		double aDT=-1.0,const std::string &aExtension=".sto");

//=============================================================================
};	// END of class TorqueApplier

}; //namespace
//=============================================================================
//=============================================================================


#endif // #ifndef __TorqueApplier_h__
