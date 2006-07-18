#ifndef _SimmModel_h_
#define _SimmModel_h_

// SimmModel.h
// Author: Peter Loan
/* Copyright (c) 2005, Stanford University and Peter Loan.
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


// INCLUDE
#include <string>
#include <OpenSim/Simulation/rdSimulationDLL.h>
#include <OpenSim/Tools/Storage.h>
#include <OpenSim/Tools/XMLDocument.h>
#include <OpenSim/Tools/ArrayPtrs.h>
#include <OpenSim/Tools/PropertyObjArray.h>
#include <OpenSim/Tools/NatCubicSpline.h>
#include <OpenSim/Tools/ScaleSet.h>
#include <OpenSim/Simulation/Model/AbstractDynamicsEngine.h>
#include "SimmBody.h"
#include "SimmMuscle.h"
#include "SimmMuscleGroup.h"
#include "SimmMusclePoint.h"
#include "SimmMuscleViaPoint.h"
#include "SimmMarkerSet.h"
#include "SimmMeasurement.h"
#include "SimmMarkerData.h"
#include "SimmMotionData.h"
#include "SimmUnits.h"
#include "SimmIKTrialParams.h"

namespace OpenSim { 

class SimmKinematicsEngine;

//=============================================================================
//=============================================================================
/**
 * A class implementing a SIMM model.
 *
 * @author Peter Loan
 * @version 1.0
 */
class RDSIMULATION_API SimmModel : public Model
{

//=============================================================================
// DATA
//=============================================================================
private:
	PropertyObjArray _musclesProp;
	ArrayPtrs<SimmMuscle> &_muscles;

	ArrayPtrs<SimmMuscleGroup> _muscleGroups;

	PropertyObjArray _kinematicsEngineProp;
	ArrayPtrs<AbstractDynamicsEngine> &_kinematicsEngine;

	PropertyDblArray _gravityProp;
	Array<double> &_gravity;

	std::string _fileName;

	bool _builtOK;

protected:
	
//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	SimmModel();
	SimmModel(const std::string &aFileName);
	SimmModel(DOMElement *aElement);
	SimmModel(const SimmModel &aModel);
	virtual ~SimmModel();
	virtual Object* copy() const;
	virtual Object* copy(DOMElement *aElement) const;
	void copyData(const SimmModel &aModel);
#ifndef SWIG
	SimmModel& operator=(const SimmModel &aModel);
#endif

	int getNumberOfMuscles() const {	return _muscles.getSize(); }
	Object* getMuscle(int index) { return _muscles.get(index); }

	int getNumberOfMuscleGroups() const {	return _muscleGroups.getSize(); }
	SimmMuscleGroup* enterGroup(const std::string& aName);

	void setKinematicsEngine(AbstractDynamicsEngine& aKE);
	AbstractDynamicsEngine& getKinematicsEngine() const;
	SimmKinematicsEngine& getSimmKinematicsEngine() const;
/*
	void solveInverseKinematics(const SimmIKTrialParams& aIKOptions, const std::string aMarkerDataFileName, const std::string aOutputFileName);
	SimmMotionData* solveInverseKinematics(const SimmIKTrialParams& aIKOptions, SimmMarkerData& aMarkerData);
	SimmMotionData* solveInverseKinematics(const SimmIKTrialParams& aIKOptions, SimmMarkerData& aMarkerData, SimmMotionData& aCoordinateData);
*/
	void moveMarkersToCloud(Storage& aMarkerStorage);
	int deleteUnusedMarkers(const Array<std::string>& aMarkerNames);
	int replaceMarkerSet(SimmMarkerSet& aMarkerSet);
	void updateMarkers(ArrayPtrs<SimmMarker>& aMarkerArray);
	void updateCoordinates(ArrayPtrs<SimmCoordinate>& aCoordinateArray);
	double takeMeasurement(const SimmMeasurement& aMeasurement);
	const SimmUnits& getLengthUnits() const;
	const SimmUnits& getForceUnits() const;

	const double* getGravity() const { return &_gravity[0]; }
	const char* getGravityLabel() const;
	bool bodyNeededForDynamics(SimmBody* aBody);

	ArrayPtrs<SimmBody>& getBodies();
	ArrayPtrs<SimmCoordinate>& getCoordinates();

	virtual void setPin(int aBody,int aPinNumber,const double aPin[3]);
	virtual void getPin(int aBody,int aPinNumber,double rPin[3]) const;
	virtual void getJointInfo(int aJoint,int rInfo[50],int rSlider[6]) const;

	const std::string& getInputFileName() const { return _fileName; }
	void writeSIMMJointFile(std::string& aFileName) const;
	void writeSIMMMuscleFile(std::string& aFileName) const;
	void writeMarkerFile(std::string& aFileName) const;

	/* Register types to be used when reading a SimmModel object from xml file. */
	static void registerTypes();

	void setup();
	bool builtOK() { return _builtOK; }
	void peteTest() const;

protected:
	void setupProperties();

private:
	void setNull();

public:
	//--------------------------------------------------------------------------
	// The remaining methods are copied from rdModel.
	//--------------------------------------------------------------------------
	//--------------------------------------------------------------------------
	// NUMBERS
	//--------------------------------------------------------------------------
	virtual int getNJ() const; // SDFAST
	virtual int getNQ() const; // SDFAST GenCoord
	virtual int getNU() const; // SDFAST GenSpeeds
	virtual int getNX() const; // ACTUATED MODEL Controls
	virtual int getNA() const; // ACTUATED MODEL Actuators
	virtual int getNP() const; // ACTUATED MODEL Contacts
	virtual int getNY() const; // ACTUATED MODEL States
	virtual int getNYP() const; // ACTUATED MODEL PseudoStates

	//--------------------------------------------------------------------------
	// NAMES
	//--------------------------------------------------------------------------
protected:
	virtual void setBodyName(int aIndex, const std::string &aName);
public:
	virtual std::string getBodyName(int aIndex) const;
	virtual std::string getCoordinateName(int aIndex) const;
	virtual std::string getSpeedName(int aIndex) const;
	virtual std::string getActuatorName(int aIndex) const;
	virtual std::string getControlName(int aIndex) const;
	virtual std::string getStateName(int aIndex) const;
	virtual std::string getPseudoStateName(int aIndex) const;

	//--------------------------------------------------------------------------
	// INDICES FROM NAMES
	//--------------------------------------------------------------------------
	virtual int getBodyIndex(const std::string &aName) const;
	virtual int getCoordinateIndex(const std::string &aName) const;
	virtual int getSpeedIndex(const std::string &aName) const;
	virtual int getActuatorIndex(const std::string &aName) const;
	virtual int getControlIndex(const std::string &aName) const;
	virtual int getStateIndex(const std::string &aName) const;
	virtual int getPseudoStateIndex(const std::string &aName) const;

	//--------------------------------------------------------------------------
	// SET CURRENT TIME, CONTROLS, AND STATES
	//--------------------------------------------------------------------------
	virtual void set(double aT, const double aX[], const double aY[]);

	//--------------------------------------------------------------------------
	// CONTROLS
	//--------------------------------------------------------------------------
	virtual void setControls(const double aX[]);
	virtual void setControl(int aIndex, double aValue);
	virtual void setControl(const std::string &aName, double aValue);
	virtual void getControls(double rX[]) const;
	virtual double getControl(int aIndex) const;
	virtual double getControl(const std::string &aName) const;

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
	void applyDefaultPose();

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
	virtual void getCoordinates(double rQ[]) const;
	virtual double getCoordinate(int aIndex) const;
	virtual double getCoordinate(const std::string &aName) const;
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
	virtual int getGroundID() const;
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
	// PRESCRIBED MOTION
	//--------------------------------------------------------------------------
	virtual void prescribeMotion(int aJoint, int aAxis, int aPrescribed);  // DYN

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
	// ACTUATION
	//--------------------------------------------------------------------------
	virtual void computeActuation();
	virtual void applyActuatorForce(int aID); // DYN
	virtual void applyActuatorForces(); // DYN
	virtual void setActuatorForce(int aID, double aForce);
	virtual double getActuatorForce(int aID) const;
	virtual double getActuatorStress(int aID) const;
	virtual double getActuatorSpeed(int aID) const;
	virtual double getActuatorPower(int aID) const;

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
};	// END of class SimmModel

}; //namespace
//=============================================================================
//=============================================================================

#endif // __SimmModel_h__


