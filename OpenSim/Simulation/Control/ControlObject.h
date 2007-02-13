#ifndef _ControlObject_h_
#define _ControlObject_h_
// ControlObject.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c) 2005, Stanford University. All rights reserved. 
* Redistribution and use in source and binary forms, with or without 
* modification, are permitted provided that the following conditions
* are met: 
*  - Redistributions of source code must retain the above copyright 
*    notice, this list of conditions and the following disclaimer. 
*  - Redistributions in binary form must reproduce the above copyright 
*    notice, this list of conditions and the following disclaimer in the 
*    documentation and/or other materials provided with the distribution. 
*  - Neither the name of the Stanford University nor the names of its 
*    contributors may be used to endorse or promote products derived 
*    from this software without specific prior written permission. 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN 
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
* POSSIBILITY OF SUCH DAMAGE. 
*/

/* Note: This code was originally developed by Realistic Dynamics Inc. 
 * Author: Frank C. Anderson 
 */

// INCLUDES
#include <OpenSim/Simulation/rdSimulationDLL.h>
#include <OpenSim/Tools/rdTools.h>
#include <OpenSim/Tools/Function.h>
#include <OpenSim/Tools/FunctionSet.h>


//=============================================================================
//=============================================================================
/**
 * An abstract base class for specifying an object that is to be tracked
 * during a dynamic simulation.
 *
 * @author Frank C. Anderson
 * @version 1.0
 */
namespace OpenSim { 

class AbstractModel;

class RDSIMULATION_API ControlObject : public Object
{

//=============================================================================
// DATA
//=============================================================================
public:
	static const char DEFAULT_NAME[];
private:
	static const char PROP_WRT_BODY[];
	static const char PROP_EXPRESS_BODY[];
	static const char PROP_ON[];
	static const char PROP_ACTIVE[];
	static const char PROP_W[];
	static const char PROP_KP[];
	static const char PROP_KV[];
	static const char PROP_KA[];
	static const char PROP_R0[];
	static const char PROP_R1[];
	static const char PROP_R2[];
protected:
	/** Model. */
	AbstractModel *_model;
	/** Flag to indicate on or off state. */
	bool _on;
	/** Body with respect to which the track goals are specified. */
	int _wrtBody;
	/** Body frame in which the track goals are expressed. */
	int _expressBody;
	/** Flag to specify the active track goals. */
	bool _active[3];
	/** Weights of the track goals. */
	double _w[3];
	/** Position error feedback gain. */
	double _kp[3];
	/** Velocity error feedback gain. */
	double _kv[3];
	/** Feedforward acceleration gain. */
	double _ka[3];
	/** Directions of the track goals. */
	double _r[3][3];
	/** Number of track functions. */
	int _nTrk;
	/** Position track functions.  Different types of track objects can 
	require different numbers of track functions.  For example, to track
	a joint angle, only one track function is needed.  However, to track
	a position, up to three track functions may be needed. */
	Function *_pTrk[3];
	/** Velocity track functions.  If velocity track functions are
	not specified, derivatives of the position track function are used. */
	Function *_vTrk[3];
	/** Acceleration track functions.  If acceleration track functions are
	not specified, derivatives of the position track function are used. */
	Function *_aTrk[3];
	/** Last position error. */
	double _pErrLast[3];
	/** Position error. */
	double _pErr[3];
	/** Last velocity error. */
	double _vErrLast[3];
	/** Velocity error. */
	double _vErr[3];
	/** Desired accelerations. */
	double _aDes[3];
	/** Accelerations. */
	double _a[3];
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
	ControlObject();
	ControlObject(const ControlObject &aTrackObject);
	virtual ~ControlObject();
	virtual Object* copy() const = 0;
private:
	void setNull();
	void copyData(const ControlObject &aTrackObject);

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:
	ControlObject& operator=(const ControlObject &aTrackObject);

	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
	// MODEL
	void setModel(AbstractModel *aModel);
	AbstractModel* getModel() const;
	// ON,OFF
	void setOn(bool aTrueFalse);
	bool getOn() const;
	// WRT BODY
	void setWRTBody(int aBody);
	int getWRTBody() const;
	// EXPRESS BODY
	void setExpressBody(int aBody);
	int getExpressBody() const;
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
	// DIRECTION OF TRACK GOAL 0
	void setDirection_0(const double aR[3]);
	void getDirection_0(double rR[3]) const;
	// DIRECTION OF TRACK GOAL 1
	void setDirection_1(const double aR[3]);
	void getDirection_1(double rR[3]) const;
	// DIRECTION OF TRACK GOAL 2
	void setDirection_2(const double aR[3]);
	void getDirection_2(double rR[3]) const;
	// FUNCTIONS
	// position
	int getNumTrackFunctions() const;
	void setTrackFunctions(Function *aF0,
		Function *aF1=NULL,Function *aF2=NULL);
	Function* getTrackFunction(int aWhich) const;
	// velocity
	void setTrackFunctionsForVelocity(Function *aF0,
		Function *aF1=NULL,Function *aF2=NULL);
	Function* getTrackFunctionForVelocity(int aWhich) const;
	// acceleration
	void setTrackFunctionsForAcceleration(Function *aF0,
		Function *aF1=NULL,Function *aF2=NULL);
	Function* getTrackFunctionForAcceleration(int aWhich) const;
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
	virtual void computeAccelerations() = 0;
	virtual void computeJacobian();
	virtual void computeEffectiveMassMatrix();

//=============================================================================
};	// END of class ControlObject

}; //namespace
//=============================================================================
//=============================================================================

#endif // __ControlObject_h__


