#ifndef __SimmKinematicsEngine_h__
#define __SimmKinematicsEngine_h__

// SimmKinematicsEngine.h
// Authors: Frank C. Anderson, Ayman Habib, and Peter Loan
/*
 * Copyright (c)  2006, Stanford University. All rights reserved. 
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
#include "osimSimmKinematicsEngineDLL.h"
#include <iostream>
#include <string>
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Common/Array.h>
#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/PropertyObj.h>
#include <OpenSim/Common/ArrayPtrs.h>
#include <OpenSim/Common/Function.h>
#include <OpenSim/Common/NatCubicSpline.h>
#include <OpenSim/Common/ScaleSet.h>
#include <OpenSim/Simulation/Model/AbstractDynamicsEngine.h>
#include <OpenSim/Simulation/Model/JointSet.h>
#include <OpenSim/Simulation/Model/CoordinateSet.h>
#include "SimmPathMatrix.h"
#include "SimTKcommon.h"

#ifdef SWIG
	#ifdef OSIMSIMMKINEMATICSENGINE_API
		#undef OSIMSIMMKINEMATICSENGINE_API
		#define OSIMSIMMKINEMATICSENGINE_API
	#endif
#endif

namespace OpenSim {

class AbstractBody;
class AbstractCoordinate;
class AbstractTransformAxis;
class AbstractDof01_05;
class MarkerSet;
class MarkerData;

//=============================================================================
//=============================================================================
/**
 * A class implementing the SIMM kinematics engine.
 * A kinematics engine is used to compute the positions, velocities, and
 * accelerations of bodies and points on bodies in an articulated linkage.
 *
 * At a minimum, a kinematics engine must contain a description of the
 * topology of the articulated linkage. That is, how many bodies and how
 * those bodies are connected.
 *
 * @authors Frank C. Anderson, Ayman Habib, Peter Loan
 * @version 1.0
 */

class OSIMSIMMKINEMATICSENGINE_API SimmKinematicsEngine  : public AbstractDynamicsEngine
{

//=============================================================================
// DATA
//=============================================================================
protected:

	/** Set containing the joints in this model. */
	PropertyObj _simmJointSetProp;
	JointSet &_simmJointSet;

	/** Set containing the coordinates in this model. */
	PropertyObj _simmCoordinateSetProp;
	CoordinateSet &_simmCoordinateSet;

	/* Matrix of paths (how to get from any body to any other body) */
	SimmPathMatrix _path;

	/** Ground body. */
	AbstractBody* _groundBody;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION AND DESTRUCTION
	//--------------------------------------------------------------------------
public:
	SimmKinematicsEngine();
	SimmKinematicsEngine(const std::string &aFileName);
	virtual ~SimmKinematicsEngine();
	SimmKinematicsEngine(const SimmKinematicsEngine& aEngine);
	virtual Object* copy() const;
#ifndef SWIG
	SimmKinematicsEngine& operator=(const SimmKinematicsEngine &aEngine);
#endif
	static void registerTypes();

private:
	void setNull();
	void setupProperties();
	void copyData(const SimmKinematicsEngine &aEngine);
	AbstractBody* identifyGroundBody();
	//void solveFrames(const IKTrial& aIKOptions, Storage& inputData, Storage& outputData);

protected:
	void createCoordinateJointLists();
	void createCoordinatePathLists();
	void createJointPathLists();

public:
	virtual void setup(Model* aModel);
	void makePaths();
#if 0
	void solveInverseKinematics(const IKTrial& aIKOptions, const std::string aMarkerDataFileName, const std::string aOutputFileName);
	Storage* solveInverseKinematics(const IKTrial& aIKOptions, MarkerData& aMarkerData);
	Storage* solveInverseKinematics(const IKTrial& aIKOptions, MarkerData& aMarkerData, Storage& aCoordinateData);
#endif

	//--------------------------------------------------------------------------
	// NUMBERS
	//--------------------------------------------------------------------------
	virtual int getNumJoints() const;
	virtual int getNumCoordinates() const;

	//--------------------------------------------------------------------------
	// JOINTS
	//--------------------------------------------------------------------------
	virtual JointSet* getJointSet() { return &_simmJointSet; }
#ifndef SWIG
	virtual const JointSet* getJointSet() const { return &_simmJointSet; }
#endif
	
	//--------------------------------------------------------------------------
	// COORDINATES
	//--------------------------------------------------------------------------
	virtual CoordinateSet* getCoordinateSet() { return &_simmCoordinateSet; }
#ifndef SWIG
	virtual const CoordinateSet* getCoordinateSet() const { return &_simmCoordinateSet; }
#endif
	virtual void updateCoordinateSet(CoordinateSet& aCoordinateSet);
	virtual void getUnlockedCoordinates(CoordinateSet& rUnlockedCoordinates) const;
	virtual AbstractTransformAxis* findUnconstrainedDof(const AbstractCoordinate& aCoordinate, AbstractJoint*& rJoint) { return NULL; }
	AbstractDof01_05* findUnconstrainedSimmDof(const AbstractCoordinate& aCoordinate, AbstractJoint*& rJoint);

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
	virtual double getAcceleration(int aIndex) const;
	virtual double getAcceleration(const std::string &aSpeedName) const;
	virtual void extractConfiguration(const double aY[], double rQ[], double rU[]) const;
	virtual void applyDefaultConfiguration();
	//--------------------------------------------------------------------------
	// PROJECT to satisfy constaints
	//--------------------------------------------------------------------------
	virtual bool projectConfigurationToSatisfyConstraints(double uY[], const double cTol, double uYerr[]) {return false;}

	//--------------------------------------------------------------------------
	// ASSEMBLING THE MODEL
	//--------------------------------------------------------------------------
	virtual int assemble(double aTime, double *rState, int *aLock, double aTol, int aMaxevals, int *rFcnt, int *rErr);

	//--------------------------------------------------------------------------
	// SCALING
	//--------------------------------------------------------------------------
	virtual bool scale(const ScaleSet& aScaleSet, double aFinalMass = -1.0, bool aPreserveMassDist = false);

	//--------------------------------------------------------------------------
	// BODY INFORMATION
	//--------------------------------------------------------------------------
	virtual AbstractBody& getGroundBody() const;
	virtual AbstractBody* getLeafBody(AbstractJoint* aJoint) const;

	//--------------------------------------------------------------------------
	// INERTIA
	//--------------------------------------------------------------------------
	virtual double getMass() const;
	virtual void getSystemInertia(double *rM, SimTK::Vec3& rCOM, double rI[3][3]) const;
	virtual void getSystemInertia(double *rM, double *rCOM, double *rI) const;

	//--------------------------------------------------------------------------
	// KINEMATICS
	//--------------------------------------------------------------------------
	virtual void getPosition(const AbstractBody &aBody, const SimTK::Vec3& aPoint, SimTK::Vec3& rPos) const;
	virtual void getVelocity(const AbstractBody &aBody, const SimTK::Vec3& aPoint, SimTK::Vec3& rVel) const;
	virtual void getAcceleration(const AbstractBody &aBody, const SimTK::Vec3& aPoint, SimTK::Vec3& rAcc) const;
	virtual void getDirectionCosines(const AbstractBody &aBody, double rDirCos[3][3]) const;
	virtual void getDirectionCosines(const AbstractBody &aBody, double *rDirCos) const;
	virtual void getAngularVelocity(const AbstractBody &aBody, SimTK::Vec3& rAngVel) const;
	virtual void getAngularVelocityBodyLocal(const AbstractBody &aBody, SimTK::Vec3& rAngVel) const;
	virtual void getAngularAcceleration(const AbstractBody &aBody, SimTK::Vec3& rAngAcc) const;
	virtual void getAngularAccelerationBodyLocal(const AbstractBody &aBody, SimTK::Vec3& rAngAcc) const;
	virtual Transform getTransform(const AbstractBody &aBody);

	//--------------------------------------------------------------------------
	// LOAD APPLICATION
	//--------------------------------------------------------------------------
	// FORCES EXPRESSED IN INERTIAL FRAME
	virtual void applyForce(const AbstractBody &aBody, const SimTK::Vec3& aPoint, const SimTK::Vec3& aForce);
	virtual void applyForces(int aN, const AbstractBody *aBodies[], const double aPoints[][3], const double aForces[][3]);
	virtual void applyForces(int aN, const AbstractBody *aBodies[], const double *aPoints, const double *aForces);

	// FORCES EXPRESSED IN BODY-LOCAL FRAME
	virtual void applyForceBodyLocal(const AbstractBody &aBody, const SimTK::Vec3& aPoint, const SimTK::Vec3& aForce);
	virtual void applyForcesBodyLocal(int aN, const AbstractBody *aBodies[], const double aPoints[][3], const double aForces[][3]);
	virtual void applyForcesBodyLocal(int aN, const AbstractBody *aBodies[], const double *aPoints, const double *aForces);

	// TORQUES EXPRESSED IN INERTIAL FRAME
	virtual void applyTorque(const AbstractBody &aBody, const SimTK::Vec3& aTorque);
	virtual void applyTorques(int aN, const AbstractBody *aBodies[], const double aTorques[][3]);
	virtual void applyTorques(int aN, const AbstractBody *aBodies[], const double *aTorques);

	// TORQUES EXPRESSED IN BODY-LOCAL FRAME
	virtual void applyTorqueBodyLocal(const AbstractBody &aBody, const SimTK::Vec3& aTorque);
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
	virtual void computeReactions(SimTK::Vector_<SimTK::Vec3>& rForces, SimTK::Vector_<SimTK::Vec3>& rTorques) const;

	//--------------------------------------------------------------------------
	// EQUATIONS OF MOTION
	//--------------------------------------------------------------------------
	virtual void formMassMatrix(double *rI);
	virtual void formEulerTransform(const AbstractBody &aBody, double *rE) const;
	virtual void formJacobianTranslation(const AbstractBody &aBody, const SimTK::Vec3& aPoint, double *rJ, const AbstractBody *aRefBody=NULL) const;
	virtual void formJacobianOrientation(const AbstractBody &aBody, double *rJ0, const AbstractBody *aRefBody=NULL) const;
	virtual void formJacobianEuler(const AbstractBody &aBody, double *rJE, const AbstractBody *aRefBody=NULL) const;

	//--------------------------------------------------------------------------
	// DERIVATIVES
	//--------------------------------------------------------------------------
	virtual void computeDerivatives(double *dqdt, double *dudt);

	//--------------------------------------------------------------------------
	// UTILITY
	//--------------------------------------------------------------------------
	virtual void transform(const AbstractBody &aBodyFrom, const double aVec[3], const AbstractBody &aBodyTo, double rVec[3]) const;
	virtual void transform(const AbstractBody &aBodyFrom, const SimTK::Vec3& aVec, const AbstractBody &aBodyTo, SimTK::Vec3& rVec) const;
	virtual void transformPosition(const AbstractBody &aBodyFrom, const double aPos[3], const AbstractBody &aBodyTo, double rPos[3]) const;
	virtual void transformPosition(const AbstractBody &aBodyFrom, const SimTK::Vec3& aPos, const AbstractBody &aBodyTo, SimTK::Vec3& rPos) const;
	virtual void transformPosition(const AbstractBody &aBodyFrom, const double aPos[3], double rPos[3]) const;
	virtual void transformPosition(const AbstractBody &aBodyFrom, const SimTK::Vec3& aPos, SimTK::Vec3& rPos) const;

	virtual double calcDistance(const AbstractBody &aBody1, const double aPoint1[3], const AbstractBody &aBody2, const double aPoint2[3]) const;
	virtual double calcDistance(const AbstractBody &aBody1, const SimTK::Vec3& aPoint1, const AbstractBody &aBody2, const SimTK::Vec3& aPoint2) const;

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

	virtual void computeConstrainedCoordinates(double rQ[]) const;

	OPENSIM_DECLARE_DERIVED(SimmKinematicsEngine, AbstractDynamicsEngine);

//=============================================================================
};	// END of class SimmKinematicsEngine
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __SimmKinematicsEngine_h__


