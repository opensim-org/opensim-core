#ifndef _rdSDFast_h_
#define _rdSDFast_h_
// rdSDFast.h
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


// INCLUDE
#include "rdSDFastDLL.h"
#include <OpenSim/Tools/Storage.h>
#include <OpenSim/Simulation/SIMM/AbstractModel.h>
#include <OpenSim/Tools/NamedValueArray.h>

#ifdef SWIG
	#ifdef RDSDFAST_API
		#undef RDSDFAST_API
		#define RDSDFAST_API
	#endif
#endif
#ifdef WIN32
  #ifndef SWIG
	//template class RDTOOLS_API rdNamedValueArray<double>;
	//template class RDTOOLS_API rdNamedValueArray<int>;
  #endif
#endif

//=============================================================================
//=============================================================================
/**
 * A wrapper class that implements many of the methods in rdModel by
 * interfacing with SDFast.
 *
 * @author Frank C. Anderson, James M. Ziegler
 * @version 1.0
 */
namespace OpenSim { 

class RDSDFAST_API rdSDFast : public AbstractModel  
{

//=============================================================================
// DATA
//=============================================================================
protected:
	// STATICS
	/** Body number for ground. */
	static const int GROUND;
	/** A pointer to the last constructed rdSDFast object.  This has been
	introduced to fulfill the SDFast mechanism of applying user-supplied
	forces through sduforce().  SDFast somtines calls sduForce(), and so
	it is necessary to redirect this call through a static function.  See
	rdSDFast::SDUForce(). */
	static rdSDFast *_Instance;

	// LOCAL
	/** Map from generalized speed (degree of freedom) to joint. */
	int *_u2jMap;
	/** Map from generalized speed (degree of freedom) to axis. */
	int *_u2aMap;

	/** Accelerations of the independent generalized coordinates. */
	double *_dudt;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	rdSDFast();
	rdSDFast(const std::string &aFileName);
	virtual ~rdSDFast();
protected:
	/* Register types to be used when reading an rdSDFast object from xml file. */
	static void RegisterTypes();
private:
	void setNull();
	virtual void init();	
	void constructSystemVariables();
	void constructJointAndAxisMaps();

	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
protected:
	virtual void setCoordinateName(int aIndex,const std::string &aName);
	virtual void setSpeedName(int aIndex,const std::string &aName);
public:
	// NUMBERS OF THINGS
	virtual int getNumJoints() const;
	virtual int getNumCoordinates() const;
	virtual int getNumSpeeds() const;

	//--------------------------------------------------------------------------
	// NAMES, INDICES, MAPPING BACK AND FORTH
	//--------------------------------------------------------------------------
	// Lists maintained by the rdSDFast layer
	// Coordinates
	virtual std::string getCoordinateName(int aIndex) const;
	virtual double getCoordinate(int aIndex) const;
	virtual double getCoordinate(const std::string &aName) const;
	virtual int getCoordinateIndex(const std::string &aName) const;
	virtual void getCoordinates(double rQ[]) const;
	// Speeds
	virtual std::string getSpeedName(int aIndex) const;
	virtual double getSpeed(int aIndex) const;
	virtual double getSpeed(const std::string &aName) const;
	virtual int getSpeedIndex(const std::string &aName) const;
	virtual void getSpeedValues(double rU[]) const;
	// Accelerations
	virtual void getAccelerations(double rDUDT[]) const;
	virtual double getAcceleration(const std::string &aSpeedName) const;
	virtual double getAcceleration(int aIndex) const;

	// GRAVITY
	virtual bool
		setGravity(double aGrav[3]);
	virtual void
		getGravity(double rGrav[3]) const;

	// CONFIGURATION
	virtual void
		setConfiguration(const double aQ[],const double aU[]);
	virtual void
		setConfiguration(const double aY[]);
	virtual void 
		extractConfiguration(const double aY[],double rQ[],double rU[]) const;
	// ASSEMBLY
	virtual int assemble(double aTime,double *rState,int *aLock,double aTol,
		 int aMaxevals,int *rFcnt,int *rErr);
	// SCALING
	virtual bool scale(const ScaleSet& aScaleSet);
	// UTILITY
	virtual void dump(const std::string& aFileName);

	// BODY INFORMATION
	virtual int getGroundID() const;
	virtual void setBodyToJointBodyLocal(int aBody,const double aBTJ[3]);
	virtual void getBodyToJointBodyLocal(int aBody,double rBTJ[3]) const;
	virtual void setInboardToJointBodyLocal(int aBody,const double aBTJ[3]);
	virtual void getInboardToJointBodyLocal(int aBody,double rITJ[3]) const;
	virtual void setPin(int aBody,int aPinNumber,const double aPin[3]);
	virtual void getPin(int aBody,int aPinNumber,double rPin[3]) const;
	virtual void getJointInfo(int aJoint,int rInfo[50],int rSlider[6]) const;

	// INERTIA
	virtual double
		getMass(int aBody) const;
	virtual int
		setMass(int aBody, double aMass);
	virtual int
		getInertiaBodyLocal(int aBody,double rI[3][3]) const;
	virtual int
		getInertiaBodyLocal(int aBody,double *rI) const;
	virtual int
		setInertiaBodyLocal(int aBody,const double aI[3][3]);
	virtual void
		getSystemInertia(double *rM,double rCOM[3],double rI[3][3]) const;
	virtual void
		getSystemInertia(double *rM,double *rCOM,double *rI) const;

	// KINEMATICS
	virtual void
		getPosition(int aBody,const double aPoint[3],double rPos[3]) const;
	virtual void
		getVelocity(int aBody,const double aPoint[3],double rVel[3]) const;
	virtual void
		getAcceleration(int aBody,const double aPoint[3],double rAcc[3]) const;
	virtual void
		getDirectionCosines(int aBody,double rDirCos[3][3]) const;
	virtual void
		getDirectionCosines(int aBody,double *rDirCos) const;
	virtual void
		getAngularVelocity(int aBody,double rAngVel[3]) const;
	virtual void
		getAngularVelocityBodyLocal(int aBody,double rAngVel[3]) const;
	virtual void
		getAngularAcceleration(int aBody,double rAngAcc[3]) const;
	virtual void
		getAngularAccelerationBodyLocal(int aBody,double rAngAcc[3]) const;

	//--------------------------------------------------------------------------
	// LOAD APPLICATION
	//--------------------------------------------------------------------------
	// FORCES EXPRESSED IN INERTIAL FRAME
	virtual void 
		applyForce(int aBody,const double aPoint[3],const double aForce[3]);
	virtual void 
		applyForces(int aN,const int aBodies[],const double aPoints[][3],
		const double aForces[][3]);
	virtual void 
		applyForces(int aN,const int aBodies[],const double *aPoints,
		const double *aForces);

	// FORCES EXPRESSED IN BODY-LOCAL FRAME
	virtual void 
		applyForceBodyLocal(int aBody,const double aPoint[3],
		const double aForce[3]);
	virtual void 
		applyForcesBodyLocal(int aN,const int aBodies[],const double aPoints[][3],
		const double aForces[][3]);
	virtual void 
		applyForcesBodyLocal(int aN,const int aBodies[],const double *aPoints,
		const double *aForces);

	// TORQUES EXPRESSED IN INERTIAL FRAME
	virtual void
		applyTorque(int aBody,const double aTorque[3]);
	virtual void
		applyTorques(int aN,const int aBodies[],const double aTorques[][3]);
	virtual void
		applyTorques(int aN,const int aBodies[],const double *aTorques);

	// TORQUES EXPRESSED IN BODY-LOCAL FRAME (sdbodyt())
	virtual void
		applyTorqueBodyLocal(int aBody,const double aTorque[3]);
	virtual void
		applyTorquesBodyLocal(int aN,const int aBodies[],
		const double aTorques[][3]);
	virtual void
		applyTorquesBodyLocal(int aN,const int aBodies[],const double *aTorques);

	// GENERALIZED FORCES
	virtual void 
		applyGeneralizedForce(int aU,double aF);
	virtual void
		applyGeneralizedForces(const double aF[]);
	virtual void 
		applyGeneralizedForces(int aN,const int aU[],const double aF[]);

	// LOAD ACCESS AND COMPUTATION
	virtual double
		getNetAppliedGeneralizedForce(int aU) const;
	virtual void
		computeGeneralizedForces(double aDUDT[],double rF[]) const;
	virtual void
		computeReactions(double rForces[][3],double rTorques[][3]) const;

	//--------------------------------------------------------------------------
	// PRESCRIBED MOTION
	//--------------------------------------------------------------------------
	virtual void
		prescribeMotion(int aJoint,int aAxis,int aPrescribed);

	//--------------------------------------------------------------------------
	// EQUATIONS OF MOTION
	//--------------------------------------------------------------------------
	virtual void
		formMassMatrix(double *rI);
	virtual void
		formEulerTransform(int aBody,double *rE) const;
	virtual void
		formJacobianTranslation(int aBody,const double aBodyPoint[3],double *rJ,
		int aRefBody=-1) const;
	virtual void
		formJacobianOrientation(int aBody,double *rJ0,
		int aRefBody=-1) const;
	virtual void
		formJacobianEuler(int aBody,double *rJE,
		int aRefBody=-1) const;


	//--------------------------------------------------------------------------
	// DERIVATIVES
	//--------------------------------------------------------------------------
	virtual int
		computeAccelerations(double *dqdt,double *dudt);


	//--------------------------------------------------------------------------
	// UTILITY
	//--------------------------------------------------------------------------
	virtual void
		transform(int aBody1,const double aVec1[3],int aBody2,double rVec2[3])
		const;
	virtual void
		transformPosition(int aBody,const double aPos[3], double rPos[3])
		const;
	virtual void
		convertQuaternionsToAngles(double *aQ,double *rQAng) const;
	virtual void
		convertQuaternionsToAngles(Storage *rQStore) const;
	virtual void
		convertAnglesToQuaternions(double *aQAng,double *rQ) const;
	virtual void
		convertAnglesToQuaternions(Storage *rQStore) const;

	virtual void
		convertRadiansToDegrees(double *aQRad,double *rQDeg) const;
	virtual void
		convertRadiansToDegrees(Storage *rQStore) const;
	virtual void
		convertDegreesToRadians(double *aQDeg,double *rQRad) const;
	virtual void
		convertDegreesToRadians(Storage *rQStore) const;

	virtual void
		convertAnglesToDirectionCosines(double aE1,double aE2,double aE3,
		double rDirCos[3][3]) const;
	virtual void
		convertAnglesToDirectionCosines(double aE1,double aE2,double aE3,
		double *rDirCos) const;

	virtual void
		convertDirectionCosinesToAngles(double aDirCos[3][3],
		double *rE1,double *rE2,double *rE3) const;
	virtual void
		convertDirectionCosinesToAngles(double *aDirCos,
		double *rE1,double *rE2,double *rE3) const;

	virtual void
		convertDirectionCosinesToQuaternions(double aDirCos[3][3],
		double *rQ1,double *rQ2,double *rQ3,double *rQ4) const;
	virtual void
		convertDirectionCosinesToQuaternions(double *aDirCos,
		double *rQ1,double *rQ2,double *rQ3,double *rQ4) const;

	virtual void
		convertQuaternionsToDirectionCosines(
		double aQ1,double aQ2,double aQ3,double aQ4,double rDirCos[3][3]) const;
	virtual void
		convertQuaternionsToDirectionCosines(
		double aQ1,double aQ2,double aQ3,double aQ4,double *rDirCos) const;

	//--------------------------------------------------------------------------
	// SDFAST STATIC METHODS
	//--------------------------------------------------------------------------
	static void SDUForce();



//=============================================================================
};	// END of class rdSDFast

}; //namespace
//=============================================================================
//=============================================================================

#endif // __rdSDFast_h__


