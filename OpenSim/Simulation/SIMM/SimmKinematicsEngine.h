#ifndef _SimmKinematicsEngine_h_
#define _SimmKinematicsEngine_h_

// SimmKinematicsEngine.h
// Authors: Frank C. Anderson, Ayman Habib, and Peter Loan
/* Copyright (c) 2005, Stanford University, Frank C. Anderson, Ayman Habib, and Peter Loan.
 * 
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
#include <OpenSim/Simulation/rdSimulationDLL.h>
#include <iostream>
#include <string>
#include <OpenSim/Tools/Storage.h>
#include <OpenSim/Tools/Array.h>
#include <OpenSim/Tools/Object.h>
#include <OpenSim/Tools/PropertyObjArray.h>
#include <OpenSim/Tools/ArrayPtrs.h>
#include <OpenSim/Simulation/Model/AbstractDynamicsEngine.h>
#include <OpenSim/Tools/Function.h>
#include <OpenSim/Tools/NatCubicSpline.h>
#include <OpenSim/Tools/ScaleSet.h>
#include "Constant.h"
#include "SimmBody.h"
#include "SimmCoordinate.h"
#include "SimmJoint.h"
#include "SimmDof.h"
#include "SimmRotationDof.h"
#include "SimmTranslationDof.h"
#include "SimmPathMatrix.h"
#include "SimmSdfastBody.h"
#include "SimmMarkerSet.h"
#include "SimmMeasurement.h"
#include "SimmMarkerData.h"
#include "SimmMotionData.h"
#include "SimmUnits.h"
#include "SimmIKTrialParams.h"
#include "SimmSdfastInfo.h"
#include "IKSolverInterface.h"
#include "ScalerInterface.h"

namespace OpenSim { 

class SimmModel;

#ifdef SWIG
	#ifdef RDSIMULATION_API
		#undef RDSIMULATION_API
		#define RDSIMULATION_API
	#endif
#endif


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

class RDSIMULATION_API SimmKinematicsEngine  : public AbstractDynamicsEngine
{

//=============================================================================
// DATA
//=============================================================================
public:
	SimmSdfastInfo _sdfastInfo;

protected:
	PropertyObjArray _bodiesProp;
	ArrayPtrs<SimmBody> &_bodies;

	PropertyObjArray _coordinatesProp;
	ArrayPtrs<SimmCoordinate> &_coordinates;

	PropertyObjArray _jointsProp;
	ArrayPtrs<SimmJoint> &_joints;

	PropertyStr _lengthUnitsStrProp;
	std::string& _lengthUnitsStr;
	SimmUnits _lengthUnits;

	PropertyStr _forceUnitsStrProp;
	std::string& _forceUnitsStr;
	SimmUnits _forceUnits;

	SimmPathMatrix _path;

	SimmModel* _model;

	SimmBody* _groundBody;

	//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION AND DESTRUCTION
	//--------------------------------------------------------------------------
public:
	SimmKinematicsEngine();
	SimmKinematicsEngine(const std::string &aFileName);
	SimmKinematicsEngine(DOMElement *aElement);
	virtual ~SimmKinematicsEngine();
	static void registerTypes();
	SimmKinematicsEngine(const SimmKinematicsEngine& aEngine);
	virtual Object* copy() const;
	virtual Object* copy(DOMElement *aElement) const;
#ifndef SWIG
	SimmKinematicsEngine& operator=(const SimmKinematicsEngine &aEngine);
#endif
	void saveDynamics(const std::string &aFolderName);
	SimmBody* getLeafBody(SimmJoint* aJoint) const;
	SimmDof* markUnconstrainedDof(const SimmCoordinate* aCoordinate);

private:
	void setNull();
	void setupProperties();
	void copyData(const SimmKinematicsEngine &aKE);
	bool checkDynamicParameters();
	SimmBody* identifyGroundBody(void);
	SimmDof* findNthSdfastQ(int n, SimmJoint*& joint) const;
	SimmDof* findUnconstrainedSdfastDof(const SimmCoordinate* coord) const;
	SimmMarker* getMarker(const std::string& name, SimmBody*& body) const;
	const SimmMarker* getMarker(const std::string& name, const SimmBody*& body) const;
	void countSdfastQsAndConstraints(void);
	void initSdfastParameters(void);
	void makeDofSdfastNames(void);
	void makeSdfastJointOrder(void);
	bool validSdfastModel(void);
	void makeSdfastModel(std::string filename, bool writeFile);
	void writeSDHeaderFile(std::string filename);
	void writeSdfastConstraintData(std::ofstream& out);
	void writeSdfastQRestraintData(std::ofstream& out);
	void writeSdfastQRestraintFunctions(std::ofstream& out);
	void writeSdfastQInitCode(std::ofstream& out);
	void writeSdfastInitCode(std::ofstream& out);
	void writeSdfastConstraintCode(std::ofstream& out);
	void writeSdfastWrapObjects(std::ofstream& out);
	void writeSdfastConstraintObjects(std::ofstream& out);
	void writeSdforCFile(std::string filename);
	void writeSdfastParameterFile(std::string filename);
	void solveFrames(const SimmIKTrialParams& aIKOptions, Storage& inputData, Storage& outputData);

protected:
	void createCoordinateJointLists(void);
	void createCoordinatePathLists(void);

public:
	void setup(SimmModel* aModel);
	void makePaths();
	SimmBody* getGroundBodyPtr() const { return _groundBody; }
	void convertPoint(double aPoint[3], const SimmBody* aFrom, const SimmBody* aTo) const;
	void convertPoint(Array<double>& aPoint, const SimmBody* aFrom, const SimmBody* aTo) const;
	double calcDistance(Array<double>& aPoint1, const SimmBody* aBody1, Array<double>& aPoint2, const SimmBody* aBody2) const;
	double calcDistance(const double aPoint1[3], const SimmBody* aBody1, const double aPoint2[3], const SimmBody* aBody2) const;
	int getNumMarkers() const;
/*Reorg
	void solveInverseKinematics(const SimmIKTrialParams& aIKOptions, const std::string aMarkerDataFileName, const std::string aOutputFileName);
	SimmMotionData* solveInverseKinematics(const SimmIKTrialParams& aIKOptions, SimmMarkerData& aMarkerData);
	SimmMotionData* solveInverseKinematics(const SimmIKTrialParams& aIKOptions, SimmMarkerData& aMarkerData, SimmMotionData& aCoordinateData);
*/
	void moveMarkersToCloud(Storage& aMarkerStorage);
	int deleteUnusedMarkers(const Array<std::string>& aMarkerNames);
	int replaceMarkerSet(SimmMarkerSet& aMarkerSet);
	void updateMarkers(ArrayPtrs<SimmMarker>& aMarkerArray);
	void updateCoordinates(ArrayPtrs<SimmCoordinate>& aCoordinateArray);
	double takeMeasurement(const SimmMeasurement& aMeasurement) const;
	const SimmUnits& getLengthUnits() const { return _lengthUnits; }
	const SimmUnits& getForceUnits() const { return _forceUnits; }

	void writeSIMMJointFile(std::string& aFileName) const;
	void writeMarkerFile(std::string& aFileName) const;

	void peteTest() const;

	void getUnlockedCoordinates(SimmCoordinateArray& aUnlockedCoordinates) const;
	virtual SimmBody* getBody(const std::string &aName) const;
	virtual Coordinate* getCoordinate(const std::string &aName) const;
	virtual SimmJoint* getJoint(int index) { return _joints[index]; }
	virtual ArrayPtrs<SimmBody>& getBodies() { return _bodies; }
	virtual ArrayPtrs<SimmCoordinate>& getCoordinates() { return _coordinates; }

	//--------------------------------------------------------------------------
	// NUMBERS
	//--------------------------------------------------------------------------
	virtual int getNumBodies() const { return _bodies.getSize(); }
	virtual int getNumJoints() const { return _joints.getSize(); }
	virtual int getNumCoordinates() const { return _coordinates.getSize(); }
	virtual int getNumSpeeds() const { return _coordinates.getSize(); }
	virtual int getNumControls() const;
	virtual int getNumContacts() const;
	virtual int getNumStates() const;
	virtual int getNumPseudoStates() const;

	//--------------------------------------------------------------------------
	// NAMES
	//--------------------------------------------------------------------------
public:
	virtual void setBodyName(int aIndex, const std::string &aName);
	virtual std::string getBodyName(int aIndex) const;
	virtual std::string getCoordinateName(int aIndex) const;
	virtual std::string getSpeedName(int aIndex) const;
	virtual std::string getControlName(int aIndex) const;
	virtual std::string getStateName(int aIndex) const;
	virtual std::string getPseudoStateName(int aIndex) const;

	//--------------------------------------------------------------------------
	// INDICES FROM NAMES
	//--------------------------------------------------------------------------
	virtual int getBodyIndex(const std::string &aName) const;
	virtual int getCoordinateIndex(const std::string &aName) const;
	virtual int getSpeedIndex(const std::string &aName) const;
	virtual int getControlIndex(const std::string &aName) const;
	virtual int getStateIndex(const std::string &aName) const;
	virtual int getPseudoStateIndex(const std::string &aName) const;

	//--------------------------------------------------------------------------
	// SET CURRENT TIME, CONTROLS, AND STATES
	//--------------------------------------------------------------------------
	virtual void set(double aT, const double aX[], const double aY[]);

	//--------------------------------------------------------------------------
	// INITIAL STATES
	//--------------------------------------------------------------------------
	virtual void setInitialStates(const double aYI[]);
	virtual void getInitialStates(double rYI[]) const;
	virtual double getInitialState(int aIndex) const;
	virtual double getInitialState(const std::string &aName) const;

	//--------------------------------------------------------------------------
	// STATES
	//--------------------------------------------------------------------------
	virtual void setStates(const double aY[]);
	virtual void getStates(double rY[]) const;
	virtual double getState(int aIndex) const;
	virtual double getState(const std::string &aName) const;

	//--------------------------------------------------------------------------
	// INITIAL PSEUDO STATES
	//--------------------------------------------------------------------------
	virtual void setInitialPseudoStates(const double aYPI[]);
	virtual void getInitialPseudoStates(double rYPI[]) const;
	virtual double getInitialPseudoState(int aIndex) const;
	virtual double getInitialPseudoState(const std::string &aName) const;

	//--------------------------------------------------------------------------
	// PSEUDO STATES
	//--------------------------------------------------------------------------
	virtual void setPseudoStates(const double aYP[]);
	virtual void getPseudoStates(double rYP[]) const;
	virtual double getPseudoState(int aIndex) const;
	//??virtual double getPseudoState(const std::string &aName) const;

	//--------------------------------------------------------------------------
	// CONFIGURATION
	//--------------------------------------------------------------------------
	virtual void setConfiguration(const double aY[]);
	virtual void setConfiguration(const double aQ[], const double aU[]);
	virtual void getCoordinateValues(double rQ[]) const;
	virtual double getCoordinateValue(int aIndex) const;
	virtual double getCoordinateValue(const std::string &aName) const;
	virtual void getSpeeds(double rU[]) const;
	virtual double getSpeed(int aIndex) const;
	virtual double getSpeed(const std::string &aName) const;
	virtual void getAccelerations(double rDUDT[]) const; // DYN
	virtual double getAcceleration(int aIndex) const; // DYN
	virtual double getAcceleration(const std::string &aSpeedName) const; // DYN
	virtual void extractConfiguration(const double aY[], double rQ[], double rU[]) const;

	//--------------------------------------------------------------------------
	// ASSEMBLING THE MODEL
	//--------------------------------------------------------------------------
	virtual int assemble(double aTime, double *rState, int *aLock, double aTol, int aMaxevals, int *rFcnt, int *rErr);	//DYN

	//--------------------------------------------------------------------------
	// SCALE THE MODEL
	//--------------------------------------------------------------------------
	virtual bool scale(const ScaleSet& aScaleSet);
	virtual bool scale(const ScaleSet& aScaleSet, bool aPreserveMassDist, double aFinalMass);

	//--------------------------------------------------------------------------
	// GRAVITY
	//--------------------------------------------------------------------------
	virtual void getGravity(double rGrav[3]) const; // DYN
	virtual void setGravity(double aGrav[3]); // DYN

	//--------------------------------------------------------------------------
	// BODY INFORMATION
	//--------------------------------------------------------------------------
	SimmBodyArray& getBodyArray() { return _bodies; }
	virtual int getGroundBodyIndex() const;
	virtual void setBodyToJointBodyLocal(int aBody, const double aBTJ[3]);
	virtual void getBodyToJointBodyLocal(int aBody, double rBTJ[3]) const;
	virtual void setInboardToJointBodyLocal(int aBody, const double aBTJ[3]);
	virtual void getInboardToJointBodyLocal(int aBody, double rBTJ[3]) const;

	//--------------------------------------------------------------------------
	// INERTIA	// DYN
	//--------------------------------------------------------------------------
	virtual double getMass() const;
	virtual double getMass(int aBody) const;
	virtual int getInertiaBodyLocal(int aBody, double rI[3][3]) const;
	virtual int	getInertiaBodyLocal(int aBody, double *rI) const;
	virtual void getSystemInertia(double *rM, double rCOM[3], double rI[3][3]) const;
	virtual void getSystemInertia(double *rM, double *rCOM, double *rI) const;

	//--------------------------------------------------------------------------
	// KINEMATICS
	//--------------------------------------------------------------------------
	virtual void getPosition(int aBody, const double aPoint[3], double rPos[3]) const;
	virtual void getVelocity(int aBody, const double aPoint[3], double rVel[3]) const;
	virtual void getAcceleration(int aBody, const double aPoint[3], double rAcc[3]) const;
	virtual void getDirectionCosines(int aBody, double rDirCos[3][3]) const;
	virtual void getDirectionCosines(int aBody, double *rDirCos) const;
	virtual void getAngularVelocity(int aBody, double rAngVel[3]) const;
	virtual void getAngularVelocityBodyLocal(int aBody, double rAngVel[3]) const;
	virtual void getAngularAcceleration(int aBody, double rAngAcc[3]) const;
	virtual void getAngularAccelerationBodyLocal(int aBody, double rAngAcc[3]) const;

	//--------------------------------------------------------------------------
	// LOAD APPLICATION
	//--------------------------------------------------------------------------
	// FORCES EXPRESSED IN INERTIAL FRAME
	virtual void applyForce(int aBody, const double aPoint[3], const double aForce[3]);
	virtual void applyForces(int aN, const int aBodies[], const double aPoints[][3], const double aForces[][3]);
	virtual void applyForces(int aN, const int aBodies[], const double *aPoints, const double *aForces);

	// FORCES EXPRESSED IN BODY-LOCAL FRAME
	virtual void applyForceBodyLocal(int aBody, const double aPoint[3], const double aForce[3]);
	virtual void applyForcesBodyLocal(int aN, const int aBodies[], const double aPoints[][3], const double aForces[][3]);
	virtual void applyForcesBodyLocal(int aN, const int aBodies[], const double *aPoints, const double *aForces);

	// TORQUES EXPRESSED IN INERTIAL FRAME
	virtual void applyTorque(int aBody, const double aTorque[3]); // DYN
	virtual void applyTorques(int aN, const int aBodies[], const double aTorques[][3]); // DYN
	virtual void applyTorques(int aN, const int aBodies[], const double *aTorques); // DYN

	// TORQUES EXPRESSED IN BODY-LOCAL FRAME (sdbodyt())
	virtual void applyTorqueBodyLocal(int aBody, const double aTorque[3]); // DYN
	virtual void applyTorquesBodyLocal(int aN, const int aBodies[], const double aTorques[][3]); // DYN
	virtual void applyTorquesBodyLocal(int aN, const int aBodies[], const double *aTorques); // DYN

	// GENERALIZED FORCES
	virtual void applyGeneralizedForce(int aU, double aF);
	virtual void applyGeneralizedForces(const double aF[]);
	virtual void applyGeneralizedForces(int aN, const int aU[], const double aF[]);

	//--------------------------------------------------------------------------
	// LOAD ACCESS AND COMPUTATION
	//--------------------------------------------------------------------------
	virtual double getNetAppliedGeneralizedForce(int aU) const; // DYN
	virtual void computeGeneralizedForces(double aDUDT[], double rF[]) const; // DYN
	virtual void computeReactions(double rForces[][3], double rTorques[][3]) const; // DYN

	//--------------------------------------------------------------------------
	// EQUATIONS OF MOTION
	//--------------------------------------------------------------------------
	virtual void formMassMatrix(double *rI); // DYN
	virtual void formEulerTransform(int aBody, double *rE) const; // DYN
	virtual void formJacobianTranslation(int aBody, const double aPoint[3], double *rJ, int aRefBody=-1) const; // DYN
	virtual void formJacobianOrientation(int aBody, double *rJ0, int aRefBody=-1) const; // DYN
	virtual void formJacobianEuler(int aBody, double *rJE, int aRefBody=-1) const; // DYN

	//--------------------------------------------------------------------------
	// DERIVATIVES
	//--------------------------------------------------------------------------
	virtual int computeAccelerations(double *dqdt, double *dudt); // DYN
	virtual void computeAuxiliaryDerivatives(double *dydt);  // DYN

	//--------------------------------------------------------------------------
	// UTILITY
	//--------------------------------------------------------------------------
	virtual void transform(int aBody1, const double aVec1[3], int aBody2, double rVec2[3]) const;
	virtual void transformPosition(int aBody, const double aPos[3], double rPos[3]) const;

	virtual void convertQuaternionsToAngles(double *aQ, double *rQAng) const;
	virtual void convertQuaternionsToAngles(Storage *rQStore) const;
	virtual void convertAnglesToQuaternions(double *aQAng, double *rQ) const;
	virtual void convertAnglesToQuaternions(Storage *rQStore) const;

	virtual void convertRadiansToDegrees(double *aQRad, double *rQDeg) const;
	virtual void convertRadiansToDegrees(Storage *rQStore) const;
	virtual void convertDegreesToRadians(double *aQDeg, double *rQRad) const;
	virtual void convertDegreesToRadians(Storage *rQStore) const;

	virtual void convertAnglesToDirectionCosines(double aE1, double aE2, double aE3, double rDirCos[3][3]) const;
	virtual void convertAnglesToDirectionCosines(double aE1, double aE2, double aE3, double *rDirCos) const;

	virtual void convertDirectionCosinesToAngles(double aDirCos[3][3], double *rE1, double *rE2, double *rE3) const;
	virtual void convertDirectionCosinesToAngles(double *aDirCos, double *rE1, double *rE2, double *rE3) const;

	virtual void convertDirectionCosinesToQuaternions(double aDirCos[3][3],	double *rQ1, double *rQ2, double *rQ3, double *rQ4) const;
	virtual void convertDirectionCosinesToQuaternions(double *aDirCos, double *rQ1, double *rQ2, double *rQ3, double *rQ4) const;

	virtual void convertQuaternionsToDirectionCosines(double aQ1, double aQ2, double aQ3, double aQ4, double rDirCos[3][3]) const;
	virtual void convertQuaternionsToDirectionCosines(double aQ1, double aQ2, double aQ3, double aQ4, double *rDirCos) const;

	//--------------------------------------------------------------------------
	// CONTACT
	//--------------------------------------------------------------------------
	virtual void computeContact();
	virtual void applyContactForce(int aID); // DYN
	virtual void applyContactForces(); // DYN
	virtual int getContactBodyA(int aID) const;
	virtual int getContactBodyB(int aID) const;
	virtual void setContactPointA(int aID, const double aPoint[3]);
	virtual void getContactPointA(int aID, double rPoint[3]) const;
	virtual void setContactPointB(int aID, const double aPoint[3]);
	virtual void getContactPointB(int aID, double rPoint[3]) const;
	virtual void getContactForce(int aID, double rF[3]) const;
	virtual void getContactNormalForce(int aID, double rFP[3], double rFV[3], double rF[3]) const;
	virtual void getContactTangentForce(int aID, double rFP[3], double rFV[3], double rF[3]) const;
	virtual void getContactStiffness(int aID, const double aDX[3], double rDF[3]) const;
	virtual void getContactViscosity(int aID, const double aDV[3], double rDF[3]) const;
	virtual void getContactFrictionCorrection(int aID, double aDFFric[3]) const;
	virtual double getContactForce(int aID) const;
	virtual double getContactSpeed(int aID) const;
	virtual double getContactPower(int aID) const;

//=============================================================================
};	// END of class SimmKinematicsEngine

}; //namespace
//=============================================================================
//=============================================================================

#endif // __SimmKinematicsEngine_h__


