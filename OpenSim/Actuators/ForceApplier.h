#ifndef _ForceApplier_h_
#define _ForceApplier_h_
// ForceApplier.h
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
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Simulation/Model/DerivCallback.h>
#include <OpenSim/Simulation/Manager/Manager.h>
#include <OpenSim/Common/VectorFunction.h>
#include <OpenSim/Common/FunctionSet.h>
#include "osimActuatorsDLL.h"
#include <OpenSim/Analyses/Contact.h>


//=============================================================================
//=============================================================================
/**
 * A derivatives callback used for applying external forces during a
 * simulation.
 *
 * @author Frank C. Anderson, May Q. Liu
 * @version 1.0
 */
namespace OpenSim { 

class Model;
class AbstractBody;

class OSIMACTUATORS_API ForceApplier : public DerivCallback 
{
//=============================================================================
// DATA
//=============================================================================
protected:
	/** Which body segment. */
	AbstractBody* _body;
	/** Point of force application. */
	SimTK::Vec3 _point;
	/** Force to be applied. */
	SimTK::Vec3 _force;
	/** VectorFunction containing points of force application (t,x,y,z). */
	VectorFunction* _pointFunction;
	/** VectorFunction containing force to be applied (t,x,y,z). */
	VectorFunction* _forceFunction;
	/** Flag to set reference frame of input force */
	bool _inputForcesInGlobalFrame;
	/** Flag to indicate whether or not to record the loads that are applied
	during an integration.  Recording these loads takes a lot of memory as they
	are stored every time the derivatives are evaluated (e.g., 6 times per
	integration step). */
	bool _recordAppliedLoads;
	/** Storage for the force that was actually applied during the simulation */
	Storage *_appliedForceStore;

//=============================================================================
// METHODS
//=============================================================================
public:
	ForceApplier(Model *aModel,AbstractBody *aBody);	
	ForceApplier(Model *aModel,AbstractBody *bodyFrom,AbstractBody *bodyTo,
		Storage *forceData,int fxNum,int fyNum,int fzNum,
		int pxNum,int pyNum,int pzNum,Storage *aQStore,Storage *aUStore);
	virtual ~ForceApplier();
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
	void setPoint(const SimTK::Vec3& aPoint);
	void getPoint(SimTK::Vec3& rPoint) const;
	void setForce(const SimTK::Vec3& aForce);
	void getForce(SimTK::Vec3& rPoint) const;

	void setForceFunction(VectorFunction* aForceFunction);
	VectorFunction* getForceFunction() const;
	void setPointFunction(VectorFunction* aPointFunction);
	VectorFunction* getPointFunction() const;

	void setInputForcesInGlobalFrame(bool aTrueFalse);
	bool getInputForcesInGlobalFrame() const;

	void setRecordAppliedLoads(bool aTrueFalse);
	bool getRecordAppliedLoads() const;
	Storage* getAppliedForceStorage();
	void setStorageCapacityIncrements(int aIncrement);

	virtual void reset(); 
	
	void computePointFunction(const Storage &aQStore,const Storage &aUStore,
		VectorFunction &aPGlobal);

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
};	// END of class ForceApplier

}; //namespace
//=============================================================================
//=============================================================================


#endif // #ifndef __ForceApplier_h__
