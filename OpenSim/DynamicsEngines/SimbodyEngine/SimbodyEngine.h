#ifndef __SimbodyEngine_h__
#define __SimbodyEngine_h__
// SimbodyEngine.h
// Authors: Frank C. Anderson
/*
 * Copyright (c) 2006, Stanford University. All rights reserved. 
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including 
 * without limitation the rights to use, copy, modify, merge, publish, 
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject
 * to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included 
 * in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

// INCLUDES
#include <iostream>
#include <string>
#include "osimSimbodyEngineDLL.h"
#include <OpenSim/Common/Array.h>
#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/PropertyObj.h>
#include <OpenSim/Common/ArrayPtrs.h>
#include <OpenSim/Common/Function.h>
#include <OpenSim/Common/ScaleSet.h>
#include <OpenSim/Simulation/Model/AbstractDynamicsEngine.h>
#include <OpenSim/Simulation/Model/CoordinateSet.h>
#include <SimTKsimbody.h>
#include "SimbodyBody.h"
#include "SimbodyCoordinate.h"
#include "SimbodyJoint.h"
#include "SimbodySpeed.h"

#ifdef SWIG
	#ifdef OSIMSIMBODYENGINE_API
		#undef OSIMSIMBODYENGINE_API
		#define OSIMSIMBODYENGINE_API
	#endif
#endif

namespace OpenSim {

class SimbodyBody;
class SimbodyCoordinate;
class SimbodyJoint;
class SimbodySpeed;
class CoordinateSet;
class AbstractBody;
class AbstractDof;
class AbstractCoordinate;
class Model;
class Transform;

//=============================================================================
//=============================================================================
/**
 * A class implementing the SimTK Simbody dynamics engine.
 *
 * @authors Frank C. Anderson
 * @version 1.0
 */

class OSIMSIMBODYENGINE_API SimbodyEngine  : public AbstractDynamicsEngine
{

//=============================================================================
// DATA
//=============================================================================
protected:
	/** Body number used for ground. */
	static const int GROUND;
	/** Body used for ground or the inertial frame. */
	AbstractBody* _groundBody;

	/** Matter subsystem. */
	SimTK::SimbodyMatterSubsystem _matter;

	/** States of the Simbody model.  At a minimum, it contains the
	generalized coordinates (q) and generalized speeds (u). */
	SimTK::State _s;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION AND DESTRUCTION
	//--------------------------------------------------------------------------
public:
	virtual ~SimbodyEngine();
	SimbodyEngine();
	SimbodyEngine(const std::string &aFileName);
	SimbodyEngine(const SimbodyEngine& aEngine);
	virtual Object* copy() const;
#ifndef SWIG
	SimbodyEngine& operator=(const SimbodyEngine &aEngine);
#endif
	static void registerTypes();

private:
	void setNull();
	void setupProperties();
	void copyData(const SimbodyEngine &aEngine);
	void constructPendulum();

	AbstractBody* identifyGroundBody();
	SimbodyJoint* getInboardTreeJoint(SimbodyBody *aBody) const;

public:
	void init(Model* aModel);
	virtual void setup(Model* aModel);

	//--------------------------------------------------------------------------
   // GET/SET
	//--------------------------------------------------------------------------

	//--------------------------------------------------------------------------
   // ADDING COMPONENTS
	//--------------------------------------------------------------------------
	void addBody(SimbodyBody* aBody);
	void addJoint(SimbodyJoint* aJoint);
	void addCoordinate(SimbodyCoordinate* aCoord);
	void addSpeed(SimbodySpeed* aSpeed);

	//--------------------------------------------------------------------------
	// COORDINATES
	//--------------------------------------------------------------------------
	virtual void updateCoordinateSet(CoordinateSet& aCoordinateSet);
	virtual void getUnlockedCoordinates(CoordinateSet& rUnlockedCoordinates) const;
	virtual AbstractDof* findUnconstrainedDof(const AbstractCoordinate& aCoordinate, AbstractJoint*& rJoint) { return NULL; }

	//--------------------------------------------------------------------------
	// CONFIGURATION
	//--------------------------------------------------------------------------
	virtual void setConfiguration(const double aY[]);
	virtual void getConfiguration(double rY[]) const;
	virtual void setConfiguration(const double aQ[], const double aU[]);
	virtual void getConfiguration(double rQ[],double rU[]) const;
	virtual void getCoordinates(double rQ[]) const;
	virtual void getSpeeds(double rU[]) const;
	virtual void getAccelerations(double rDUDT[]) const;
	virtual void extractConfiguration(const double aY[], double rQ[], double rU[]) const;
	virtual void applyDefaultConfiguration();
	double* getConfiguration();
	double* getDerivatives();

	//--------------------------------------------------------------------------
	// GRAVITY
	//--------------------------------------------------------------------------
	virtual bool setGravity(double aGrav[3]);

	//--------------------------------------------------------------------------
	// BODY INFORMATION
	//--------------------------------------------------------------------------
	virtual AbstractBody& getGroundBody() const;
	int getGroundBodyIndex() const;
	virtual AbstractBody* getLeafBody(AbstractJoint* aJoint) const { return NULL; }
	bool adjustJointVectorsForNewMassCenter(SimbodyBody* aBody);

	//--------------------------------------------------------------------------
	// INERTIA
	//--------------------------------------------------------------------------
	virtual double getMass() const;
	virtual void getSystemInertia(double *rM, double rCOM[3], double rI[3][3]) const;
	virtual void getSystemInertia(double *rM, double *rCOM, double *rI) const;

	//--------------------------------------------------------------------------
	// KINEMATICS
	//--------------------------------------------------------------------------
	virtual void getPosition(const AbstractBody &aBody, const double aPoint[3], double rPos[3]) const;
	virtual void getVelocity(const AbstractBody &aBody, const double aPoint[3], double rVel[3]) const;
	virtual void getAcceleration(const AbstractBody &aBody, const double aPoint[3], double rAcc[3]) const;
	virtual void getDirectionCosines(const AbstractBody &aBody, double rDirCos[3][3]) const;
	virtual void getDirectionCosines(const AbstractBody &aBody, double *rDirCos) const;
	virtual void getAngularVelocity(const AbstractBody &aBody, double rAngVel[3]) const;
	virtual void getAngularVelocityBodyLocal(const AbstractBody &aBody, double rAngVel[3]) const;
	virtual void getAngularAcceleration(const AbstractBody &aBody, double rAngAcc[3]) const;
	virtual void getAngularAccelerationBodyLocal(const AbstractBody &aBody, double rAngAcc[3]) const;
	virtual OpenSim::Transform getTransform(const AbstractBody &aBody);

	//--------------------------------------------------------------------------
	// LOAD APPLICATION
	//--------------------------------------------------------------------------
	// FORCES EXPRESSED IN INERTIAL FRAME
	virtual void applyForce(const AbstractBody &aBody, const double aPoint[3], const double aForce[3]);
	virtual void applyForces(int aN, const AbstractBody *aBodies[], const double aPoints[][3], const double aForces[][3]);
	virtual void applyForces(int aN, const AbstractBody *aBodies[], const double *aPoints, const double *aForces);

	// FORCES EXPRESSED IN BODY-LOCAL FRAME
	virtual void applyForceBodyLocal(const AbstractBody &aBody, const double aPoint[3], const double aForce[3]);
	virtual void applyForcesBodyLocal(int aN, const AbstractBody *aBodies[], const double aPoints[][3], const double aForces[][3]);
	virtual void applyForcesBodyLocal(int aN, const AbstractBody *aBodies[], const double *aPoints, const double *aForces);

	// TORQUES EXPRESSED IN INERTIAL FRAME
	virtual void applyTorque(const AbstractBody &aBody, const double aTorque[3]);
	virtual void applyTorques(int aN, const AbstractBody *aBodies[], const double aTorques[][3]);
	virtual void applyTorques(int aN, const AbstractBody *aBodies[], const double *aTorques);

	// TORQUES EXPRESSED IN BODY-LOCAL FRAME
	virtual void applyTorqueBodyLocal(const AbstractBody &aBody, const double aTorque[3]);
	virtual void applyTorquesBodyLocal(int aN, const AbstractBody *aBodies[], const double aTorques[][3]);
	virtual void applyTorquesBodyLocal(int aN, const AbstractBody *aBodies[], const double *aTorques);

	// GENERALIZED FORCES
	virtual void applyGeneralizedForce(const AbstractCoordinate &aU, double aF);
	virtual void applyGeneralizedForces(const double aF[]);
	virtual void applyGeneralizedForces(int aN, const AbstractCoordinate *aU[], const double aF[]);

	//--------------------------------------------------------------------------
	// LOAD ACCESS AND COMPUTATION
	//--------------------------------------------------------------------------
	virtual double getNetAppliedGeneralizedForce(const AbstractCoordinate &aU) const;
	virtual void computeGeneralizedForces(double aDUDT[], double rF[]) const;
	virtual void computeReactions(double rForces[][3], double rTorques[][3]) const;

	//--------------------------------------------------------------------------
	// DERIVATIVES
	//--------------------------------------------------------------------------
	virtual void computeDerivatives(double *dqdt,double *dudt);

	//--------------------------------------------------------------------------
	// UTILITY
	//--------------------------------------------------------------------------
	virtual void transform(const AbstractBody &aBodyFrom, const double aVec[3], const AbstractBody &aBodyTo, double rVec[3]) const;
	virtual void transform(const AbstractBody &aBodyFrom, const Array<double>& aVec, const AbstractBody &aBodyTo, Array<double>& rVec) const;
	virtual void transformPosition(const AbstractBody &aBodyFrom, const double aPos[3], const AbstractBody &aBodyTo, double rPos[3]) const;
	virtual void transformPosition(const AbstractBody &aBodyFrom, const Array<double>& aPos, const AbstractBody &aBodyTo, Array<double>& rPos) const;
	virtual void transformPosition(const AbstractBody &aBodyFrom, const double aPos[3], double rPos[3]) const;
	virtual void transformPosition(const AbstractBody &aBodyFrom, const Array<double>& aPos, Array<double>& rPos) const;

	virtual double calcDistance(const AbstractBody &aBody1, const double aPoint1[3], const AbstractBody &aBody2, const double aPoint2[3]) const;
	virtual double calcDistance(const AbstractBody &aBody1, const Array<double>& aPoint1, const AbstractBody &aBody2, const Array<double>& aPoint2) const;

	virtual void convertQuaternionsToAngles(double *aQ, double *rQAng) const;
	virtual void convertQuaternionsToAngles(Storage *rQStore) const;
	virtual void convertAnglesToQuaternions(double *aQAng, double *rQ) const;
	virtual void convertAnglesToQuaternions(Storage *rQStore) const;

	virtual void convertAnglesToDirectionCosines(double aE1, double aE2, double aE3, double rDirCos[3][3]) const;
	virtual void convertAnglesToDirectionCosines(double aE1, double aE2, double aE3, double *rDirCos) const;

	virtual void convertDirectionCosinesToAngles(double aDirCos[3][3], double *rE1, double *rE2, double *rE3) const;
	virtual void convertDirectionCosinesToAngles(double *aDirCos, double *rE1, double *rE2, double *rE3) const;

	virtual void convertDirectionCosinesToQuaternions(double aDirCos[3][3],	double *rQ1, double *rQ2, double *rQ3, double *rQ4) const;
	virtual void convertDirectionCosinesToQuaternions(double *aDirCos, double *rQ1, double *rQ2, double *rQ3, double *rQ4) const;

	virtual void convertQuaternionsToDirectionCosines(double aQ1, double aQ2, double aQ3, double aQ4, double rDirCos[3][3]) const;
	virtual void convertQuaternionsToDirectionCosines(double aQ1, double aQ2, double aQ3, double aQ4, double *rDirCos) const;


private:

	friend class SimbodyBody;
	friend class SimbodyCoordinate;
	friend class SimbodyJoint;

//=============================================================================
};	// END of class SimbodyEngine
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

//=============================================================================
// STATIC METHOD FOR CREATING THIS MODEL
//=============================================================================
extern "C" {
OSIMSIMBODYENGINE_API OpenSim::Model* CreateModel();
OSIMSIMBODYENGINE_API OpenSim::Model* CreateModel_File(const std::string &aModelFile);
}

#endif // __SimbodyEngine_h__


