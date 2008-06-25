#ifndef rdCMC_Task_h__
#define rdCMC_Task_h__
// rdCMC_Task.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Contributors: Frank C. Anderson
//
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

// INCLUDES
#include "osimToolsDLL.h"
#include <OpenSim/Common/PropertyBool.h>
#include <OpenSim/Common/PropertyBoolArray.h>
#include <OpenSim/Common/PropertyInt.h>
#include <OpenSim/Common/PropertyDblArray.h>
#include <OpenSim/Common/PropertyDblVec3.h>
#include <OpenSim/Common/Function.h>
#include <OpenSim/Common/FunctionSet.h>
#include <OpenSim/Simulation/Model/Model.h>

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * An abstract base class for specifying a task objective for
 * a dynamic simulation.  This class supports joint, point, and orientation
 * task objectives.  Specfic implementations for these kinds of control
 * tasks should inherit from this class.
 *
 * @author Frank C. Anderson
 * @version 1.0
 */
class OSIMTOOLS_API rdCMC_Task : public Object
{

//=============================================================================
// DATA
//=============================================================================
protected:
	// PROPERTIES
	/** Property to indicate on or off state. */
	PropertyBool _propOn;
	/** Body with respect to which the task goals are specified. */
	PropertyStr _propWRTBodyName;
	/** Body frame in which the task goals are expressed. */
	PropertyStr _propExpressBodyName;
	/** Property to specify the active task goals. */
	PropertyBoolArray _propActive;
	/** Weights of the task goals. */
	PropertyDblArray _propW;
	/** Position error feedback gain. */
	PropertyDblArray _propKP;
	/** Velocity error feedback gain. */
	PropertyDblArray _propKV;
	/** Feedforward acceleration gain. */
	PropertyDblArray _propKA;
	/** Directions of the task goal 0. */
	PropertyDblVec3 _propR0;
	/** Directions of the task goal 1. */
	PropertyDblVec3 _propR1;
	/** Directions of the task goal 2. */
	PropertyDblVec3 _propR2;

	// REFERENCES TO PROPERTY VALUES
	// NOTE- These refrence variables must be listed in the class after
	// the properties to which they refer.  The order in which member
	// variables are listed determines the order in which the member
	// variables are initialized.  The properties must be initialized
	// before the references can be initialized to something meaningful.
	/** Reference to the value of the on property. */
	bool &_on;
	/** Reference to the value of the WRTBody property. */
	std::string &_wrtBodyName;
	/** Reference to the value of the ExpressBody property. */
	std::string &_expressBodyName;
	/** Reference to the value of the Active property. */
	Array<bool> &_active;
	/** Reference to the value of the Weight property. */
	Array<double> &_w;
	/** Reference to the value of the KP property. */
	Array<double> &_kp;
	/** Reference to the value of the KV property. */
	Array<double> &_kv;
	/** Reference to the value of the KA property. */
	Array<double> &_ka;
	/** Reference to the value of the R0 property. */
	SimTK::Vec3 &_r0;
	/** Reference to the value of the R1 property. */
	SimTK::Vec3 &_r1;
	/** Reference to the value of the R2 property. */
	SimTK::Vec3 &_r2;


	/** Model. */
	Model *_model;

	/** Number of task functions. */
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
	/** Last position error. */
	SimTK::Vec3 _pErrLast;
	/** Position error. */
	SimTK::Vec3 _pErr;
	/** Last velocity error. */
	SimTK::Vec3 _vErrLast;
	/** Velocity error. */
	SimTK::Vec3 _vErr;
	/** Desired accelerations. */
	SimTK::Vec3 _aDes;
	/** Accelerations. */
	SimTK::Vec3 _a;
	/** Jacobian. */
	double *_j;
	/** Effective mass matrix. */
	double *_m;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	rdCMC_Task();
	rdCMC_Task(const rdCMC_Task &aTaskObject);
	virtual ~rdCMC_Task();
	virtual Object* copy() const = 0;
private:
	void setNull();
	void setupProperties();
	void copyData(const rdCMC_Task &aTaskObject);

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:

#ifndef SWIG
	rdCMC_Task& operator=(const rdCMC_Task &aTaskObject);
#endif

	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
	// MODEL
	virtual void setModel(Model *aModel);
	Model* getModel() const;
	// ON,OFF
	void setOn(bool aTrueFalse);
	bool getOn() const;
	// WRT BODY
	void setWRTBodyName(std::string aBodyName);
	std::string getWRTBodyName() const;
	// EXPRESS BODY
	void setExpressBodyName(std::string aBodyName);
	std::string getExpressBodyName() const;
	// ACTIVE
	void setActive(bool a0,bool a1=false,bool a2=false);
	bool getActive(int aWhich) const;
	// WEIGHTS
	void setWeight(double aW0,double aW1=0.0,double aW2=0.0);
	double getWeight(int aWhich) const;
	// POSITION FEEDBACK GAINS
	void setKP(double aK0,double aK1=0.0,double aK2=0.0);
	double getKP(int aWhich) const;
	// VELOCITY FEEDBACK GAINS
	void setKV(double aK0,double aK1=0.0,double aK2=0.0);
	double getKV(int aWhich) const;
	// ACCELERATION FEEDFORWARD GAINS
	void setKA(double aK0,double aK1=0.0,double aK2=0.0);
	double getKA(int aWhich) const;
	// DIRECTION OF TASK 0
	void setDirection_0(const SimTK::Vec3& aR);
	void getDirection_0(SimTK::Vec3& rR) const;
	// DIRECTION OF TASK 1
	void setDirection_1(const SimTK::Vec3& aR);
	void getDirection_1(SimTK::Vec3& rR) const;
	// DIRECTION OF TASK 2
	void setDirection_2(const SimTK::Vec3& aR);
	void getDirection_2(SimTK::Vec3& rR) const;
	// TASK FUNCTIONS
	int getNumTaskFunctions() const;
	void setTaskFunctions(Function *aF0,
		Function *aF1=NULL,Function *aF2=NULL);
	Function* getTaskFunction(int aWhich) const;
	void setTaskFunctionsForVelocity(Function *aF0,
		Function *aF1=NULL,Function *aF2=NULL);
	Function* getTaskFunctionForVelocity(int aWhich) const;
	void setTaskFunctionsForAcceleration(Function *aF0,
		Function *aF1=NULL,Function *aF2=NULL);
	Function* getTaskFunctionForAcceleration(int aWhich) const;
	// TASK KINEMATICS
	double getTaskPosition(int aWhich,double aT) const;
	double getTaskVelocity(int aWhich,double aT) const;
	double getTaskAcceleration(int aWhich,double aT) const;
	// LAST ERRORS
	void setPositionErrorLast(double aE0,double aE1=0.0,double aE2=0.0);
	double getPositionErrorLast(int aWhich) const;
	void setVelocityErrorLast(double aE0,double aE1=0.0,double aE2=0.0);
	double getVelocityErrorLast(int aWhich) const;
	// ERRORS
	double getPositionError(int aWhich) const;
	double getVelocityError(int aWhich) const;
	// DESIRED ACCELERATIONS
	double getDesiredAcceleration(int aWhich) const;
	// ACCELERATIONS
	double getAcceleration(int aWhich) const;

	//--------------------------------------------------------------------------
	// COMPUTATIONS
	//--------------------------------------------------------------------------
	virtual void computeErrors(double aT) = 0;
	virtual void computeDesiredAccelerations(double aT) = 0;
	virtual void computeDesiredAccelerations(double aTI,double aTF) = 0;
	virtual void computeAccelerations() = 0;
	virtual void computeJacobian();
	virtual void computeEffectiveMassMatrix();

	//--------------------------------------------------------------------------
	// XML
	//--------------------------------------------------------------------------
	virtual void updateFromXMLNode();


//=============================================================================
};	// END of class rdCMC_Task
//=============================================================================
//=============================================================================

}; // end namespace

#endif // __rdCMC_Task_h__


