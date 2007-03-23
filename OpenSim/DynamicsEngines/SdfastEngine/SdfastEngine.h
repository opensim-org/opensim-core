#ifndef __SdfastEngine_h__
#define __SdfastEngine_h__

// SdfastEngine.h
// Authors: Frank C. Anderson, Ayman Habib, and Peter Loan
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
#include <string>
#include "osimSdfastEngineDLL.h"
#include <OpenSim/Common/Array.h>
#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/PropertyStr.h>
#include <OpenSim/Simulation/Model/AbstractDynamicsEngine.h>
#include "SdfastFunctionPointerHelper.h"

#ifdef SWIG
	#ifdef OSIMSDFASTENGINE_API
		#undef OSIMSDFASTENGINE_API
		#define OSIMSDFASTENGINE_API
	#endif
#endif

namespace OpenSim {

class SdfastBody;
class SdfastCoordinate;
class SdfastJoint;
class SdfastSpeed;
class CoordinateSet;
class AbstractBody;
class AbstractDof;
class AbstractCoordinate;
class Model;
class Transform;
class Storage;

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

class OSIMSDFASTENGINE_API SdfastEngine  : public AbstractDynamicsEngine
{

//=============================================================================
// DATA
//=============================================================================
protected:
	/** Body number for ground. */
	static const int GROUND;

	AbstractBody* _groundBody;

	PropertyStr _modelLibraryNameProp;
	std::string &_modelLibraryName;

	/** The following variables are initialized by a call to sdinfo(), but
	they had better match the sizes of the _bodySet, _coordinateSet, and
	_speedSet objects. */
	int _numBodies;
	int _numQs;
	int _numUs;
	int _numJoints;

	/** Configuration:  Array of q's followed by u's. */
	double *_y;

	/** Derivative of configuration: qdot's followed by udot's. */
	double *_dy;

private:
	static const double ASSEMBLY_TOLERANCE;
	static const double BAUMGARTE_STAB;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION AND DESTRUCTION
	//--------------------------------------------------------------------------
public:
	SdfastEngine();
	SdfastEngine(const std::string &aFileName);
	virtual ~SdfastEngine();
	SdfastEngine(const SdfastEngine& aEngine);
	virtual Object* copy() const;
#ifndef SWIG
	SdfastEngine& operator=(const SdfastEngine &aEngine);
#endif
	static void registerTypes();

private:
	void setNull();
	void setupProperties();
	void copyData(const SdfastEngine &aEngine);
	AbstractBody* identifyGroundBody();
	SdfastJoint* getInboardTreeJoint(SdfastBody *aBody) const;
	void constructSystemVariables();
	void linkToModelLibrary();

public:
	void init(Model* aModel);
	virtual void setup(Model* aModel);

	virtual void peteTest() const;

	//--------------------------------------------------------------------------
   // GET/SET
	//--------------------------------------------------------------------------
	void setModelLibraryName(const std::string &aModelLibraryName) { _modelLibraryName = aModelLibraryName; }

	//--------------------------------------------------------------------------
   // ADDING COMPONENTS
	//--------------------------------------------------------------------------
	void addBody(SdfastBody* aBody);
	void addJoint(SdfastJoint* aJoint, bool aSetup=true);
	void addCoordinate(SdfastCoordinate* aCoord, bool aSetup=true);
	void addSpeed(SdfastSpeed* aSpeed, bool aSetup=true);

	//--------------------------------------------------------------------------
	// COORDINATES
	//--------------------------------------------------------------------------
	virtual void updateCoordinateSet(CoordinateSet& aCoordinateSet);
	virtual void getUnlockedCoordinates(CoordinateSet& rUnlockedCoordinates) const;
	virtual AbstractDof* findUnconstrainedDof(const AbstractCoordinate& aCoordinate, AbstractJoint*& rJoint) { return NULL; }

	//--------------------------------------------------------------------------
	// SD/FAST FUNCTIONS
	//--------------------------------------------------------------------------
	int sduforce(double t, double q[], double u[]);
	int sdumotion(double t, double q[], double u[]);
	void sduconsfrc(double t, double q[], double u[], double mults[]);
	void sduperr(double t, double q[], double errors[]);
	void sduverr(double t, double q[], double u[], double errors[]);
	void sduaerr(double t, double q[], double u[], double udot[], double errors[]);

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
	double* getConfiguration() { return _y; }
	double* getDerivatives() { return _dy; }

	//--------------------------------------------------------------------------
	// ASSEMBLING THE MODEL
	//--------------------------------------------------------------------------
	virtual int assemble(double aTime, double *rState, int *aLock, double aTol, int aMaxevals, int *rFcnt, int *rErr);
	void initializeState();
	void prescribe();
	void assemble();

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
	bool adjustJointVectorsForNewMassCenter(SdfastBody* aBody);

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
	virtual Transform getTransform(const AbstractBody &aBody);

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
	// CONSTRAINTS
	//--------------------------------------------------------------------------
	virtual void computeConstrainedCoordinates(double *rQ) const;

	//--------------------------------------------------------------------------
	// EQUATIONS OF MOTION
	//--------------------------------------------------------------------------
	virtual void formMassMatrix(double *rI);
	virtual void formEulerTransform(const AbstractBody &aBody, double *rE) const;
	virtual void formJacobianTranslation(const AbstractBody &aBody, const double aPoint[3], double *rJ, const AbstractBody *aRefBody=NULL) const;
	virtual void formJacobianOrientation(const AbstractBody &aBody, double *rJ0, const AbstractBody *aRefBody=NULL) const;
	virtual void formJacobianEuler(const AbstractBody &aBody, double *rJE, const AbstractBody *aRefBody=NULL) const;

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

	int checkForSderror(const std::string &caller);

	//--------------------------------------------------------------------------
	// FUNCTION POINTERS INTO SDFAST FUNCTIONS
	//--------------------------------------------------------------------------
	// OPENSIM_TYPEDEF_AND_MEMBER_SDFAST_FUNCTION: defines a typedef for that function type, and 
	// declares a member in the SdfastEngine class which will store the pointer to that function.
	// Makes use of OPENSIM_FOR_ALL_SDFAST_FUNCTIONS defined in SdfastFunctionPointerHelper.h
#define OPENSIM_TYPEDEF_AND_MEMBER_SDFAST_FUNCTION(name, returntype, args) \
	typedef returntype (*FUNC_##name)args; \
	FUNC_##name _##name;
	OPENSIM_FOR_ALL_SDFAST_FUNCTIONS(OPENSIM_TYPEDEF_AND_MEMBER_SDFAST_FUNCTION);

	friend class SdfastBody;
	friend class SdfastCoordinate;
	friend class SdfastJoint;

//=============================================================================
};	// END of class SdfastEngine
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

//=============================================================================
// STATIC METHOD FOR CREATING THIS MODEL
//=============================================================================
extern "C" {
OSIMSDFASTENGINE_API OpenSim::Model* CreateModel();
OSIMSDFASTENGINE_API OpenSim::Model* CreateModel_File(const std::string &aModelFile);
}

#endif // __SdfastEngine_h__


