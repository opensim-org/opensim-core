#ifndef __AbstractDynamicsEngine_h__
#define __AbstractDynamicsEngine_h__

// AbstractDynamicsEngine.h
// Authors: Frank C. Anderson, Peter Loan, Ayman Habib
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
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <iostream>
#include <string>
#include <OpenSim/Common/PropertyObj.h>
#include <OpenSim/Common/PropertyDblArray.h>
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Common/Object.h>

namespace OpenSim {

class ScaleSet;
class BodySet;
class JointSet;
class Model;
class AbstractBody;
class AbstractJoint;
class AbstractCoordinate;
class CoordinateSet;
class AbstractSpeed;
class SpeedSet;
class AbstractDof;
class AbstractMarker;
class MarkerSet;
class Transform;
class Storage;
class AbstractWrapObject;

#ifdef SWIG
	#ifdef OSIMSIMULATION_API
		#undef OSIMSIMULATION_API
		#define OSIMSIMULATION_API
	#endif
#endif

//=============================================================================
//=============================================================================
/**
 * An abstract class that specifies the interface for a kinematics or
 * dynamics engine. A kinematics engine is used to compute the positions,
 * velocities, and accelerations of bodies and points on bodies in an
 * aticulated linkage. A dynamics engine does everything a kinematics
 * engine does, plus can be used to apply forces to the bodies and
 * compute the resulting motion.
 *
 * At a minimum, a kinematics engine must contain a description of the
 * topology of the articulated linkage.  That is, how many bodies and how
 * those bodies are connected.
 *
 * @author Frank C. Anderson, Peter Loan, Ayman Habib
 * @version 1.0
 */
class OSIMSIMULATION_API AbstractDynamicsEngine : public Object
{

//=============================================================================
// DATA
//=============================================================================
protected:
	/** Pointer to the model that owns this dynamics engine. */
	Model* _model;

	/** Array containg the acceleration due to gravity. */
	PropertyDblArray _gravityProp;
	Array<double> &_gravity;

	/** Set containing the bodies in this model. */
	PropertyObj _bodySetProp;
	BodySet &_bodySet;

	/** Set containing the joints in this model. */
	PropertyObj _jointSetProp;
	JointSet &_jointSet;

	/** Set containing the generalized coordinates in this model. */
	PropertyObj _coordinateSetProp;
	CoordinateSet &_coordinateSet;

	/** Set containing the generalized speeds in this model. */
	PropertyObj _speedSetProp;
	SpeedSet &_speedSet;

	/** Set of markers for this model. */
	PropertyObj _markerSetProp;
	MarkerSet &_markerSet;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION AND DESTRUCTION
	//--------------------------------------------------------------------------
public:
	AbstractDynamicsEngine();
	AbstractDynamicsEngine(const std::string &aFileName, bool aUpdateFromXMLNode = true);
	virtual ~AbstractDynamicsEngine();
	virtual Object* copy() const = 0;
	virtual void setup(Model* aModel);

protected:
	AbstractDynamicsEngine(const AbstractDynamicsEngine& aDE);
#ifndef SWIG
	AbstractDynamicsEngine& operator=(const AbstractDynamicsEngine &aDE);
#endif
	void setNull();
	void setupProperties();
	void copyData(const AbstractDynamicsEngine& aEngine);

public:

	//--------------------------------------------------------------------------
	// NUMBERS
	//--------------------------------------------------------------------------
	virtual int getNumBodies() const;
	virtual int getNumJoints() const;
	virtual int getNumCoordinates() const;
	virtual int getNumSpeeds() const;
	virtual int getNumMarkers() const;

	//--------------------------------------------------------------------------
	// MODEL
	//--------------------------------------------------------------------------
	Model* getModel() const { return _model; }

	//--------------------------------------------------------------------------
	// GRAVITY
	//--------------------------------------------------------------------------
	virtual void getGravity(double rGrav[3]) const;
	virtual bool setGravity(double aGrav[3]);

	//--------------------------------------------------------------------------
	// BODIES
	//--------------------------------------------------------------------------
	virtual BodySet* getBodySet() { return &_bodySet; }
#ifndef SWIG
	virtual const BodySet* getBodySet() const { return &_bodySet; }
#endif
	virtual AbstractBody& getGroundBody() const = 0;
	virtual AbstractBody* getLeafBody(AbstractJoint* aJoint) const { return NULL; }
	virtual AbstractWrapObject* getWrapObject(const std::string& aName) const;

	//--------------------------------------------------------------------------
	// JOINTS
	//--------------------------------------------------------------------------
	virtual JointSet* getJointSet() { return &_jointSet; }
#ifndef SWIG
	virtual const JointSet* getJointSet() const { return &_jointSet; }
#endif
	
	//--------------------------------------------------------------------------
	// COORDINATES
	//--------------------------------------------------------------------------
	virtual CoordinateSet* getCoordinateSet() { return &_coordinateSet; }
#ifndef SWIG
	virtual const CoordinateSet* getCoordinateSet() const { return &_coordinateSet; }
#endif
	virtual void updateCoordinateSet(CoordinateSet& aCoordinateSet) = 0;
	virtual void getUnlockedCoordinates(CoordinateSet& aUnlockedCoordinates) const = 0;
	virtual AbstractDof* findUnconstrainedDof(const AbstractCoordinate& aCoordinate, AbstractJoint*& rJoint) = 0;

	//--------------------------------------------------------------------------
	// SPEEDS
	//--------------------------------------------------------------------------
	virtual SpeedSet* getSpeedSet() { return &_speedSet; }
#ifndef SWIG
	virtual const SpeedSet* getSpeedSet() const { return &_speedSet; }
#endif

	//--------------------------------------------------------------------------
	// MARKERS
	//--------------------------------------------------------------------------
	virtual MarkerSet* getMarkerSet() { return &_markerSet; }
#ifndef SWIG
	virtual const MarkerSet* getMarkerSet() const { return &_markerSet; }
#endif
	virtual void writeMarkerFile(const std::string& aFileName) const;
	virtual int replaceMarkerSet(MarkerSet& aMarkerSet);
	virtual void updateMarkerSet(MarkerSet& aMarkerSet);
	virtual int deleteUnusedMarkers(const Array<std::string>& aMarkerNames);

	//--------------------------------------------------------------------------
	// CONFIGURATION
	//--------------------------------------------------------------------------
	virtual void setConfiguration(const double aY[]) = 0;
	virtual void getConfiguration(double rY[]) const = 0;
	virtual void setConfiguration(const double aQ[],const double aU[]) = 0;
	virtual void getConfiguration(double rQ[],double rU[]) const = 0;
	virtual void getCoordinates(double rQ[]) const = 0;
	virtual void getSpeeds(double rU[]) const = 0;
	virtual void getAccelerations(double rDUDT[]) const = 0;
	virtual void extractConfiguration(const double aY[],double rQ[],double rU[]) const = 0;
	virtual void extractConfiguration(const Storage &aYStore,Storage &rQStore,Storage &rUStore);
	virtual void applyDefaultConfiguration() = 0;

	//--------------------------------------------------------------------------
	// ASSEMBLING THE MODEL
	//--------------------------------------------------------------------------
	virtual int
		assemble(double aTime,double *rState,int *aLock,double aTol,
		int aMaxevals,int *rFcnt,int *rErr) = 0;

	//--------------------------------------------------------------------------
	// SCALING
	//--------------------------------------------------------------------------
	virtual bool scale(const ScaleSet& aScaleSet, double aFinalMass = -1.0, bool aPreserveMassDist = false);

	//--------------------------------------------------------------------------
	// INERTIA
	//--------------------------------------------------------------------------
	virtual double getMass() const = 0;
	virtual void getSystemInertia(double *rM, double rCOM[3], double rI[3][3]) const = 0;
	virtual void getSystemInertia(double *rM, double *rCOM, double *rI) const = 0;

	//--------------------------------------------------------------------------
	// KINEMATICS
	//--------------------------------------------------------------------------
	virtual void getPosition(const AbstractBody &aBody, const double aPoint[3], double rPos[3]) const = 0;
	virtual void getVelocity(const AbstractBody &aBody, const double aPoint[3], double rVel[3]) const = 0;
	virtual void getAcceleration(const AbstractBody &aBody, const double aPoint[3], double rAcc[3]) const = 0;
	virtual void getDirectionCosines(const AbstractBody &aBody, double rDirCos[3][3]) const = 0;
	virtual void getDirectionCosines(const AbstractBody &aBody, double *rDirCos) const = 0;
	virtual void getAngularVelocity(const AbstractBody &aBody, double rAngVel[3]) const = 0;
	virtual void getAngularVelocityBodyLocal(const AbstractBody &aBody, double rAngVel[3]) const = 0;
	virtual void getAngularAcceleration(const AbstractBody &aBody, double rAngAcc[3]) const = 0;
	virtual void getAngularAccelerationBodyLocal(const AbstractBody &aBody, double rAngAcc[3]) const = 0;
	virtual Transform getTransform(const AbstractBody &aBody) = 0;

	//--------------------------------------------------------------------------
	// LOAD APPLICATION
	//--------------------------------------------------------------------------
	// FORCES EXPRESSED IN INERTIAL FRAME
	virtual void applyForce(const AbstractBody &aBody, const double aPoint[3], const double aForce[3]) = 0;
	virtual void applyForces(int aN, const AbstractBody *aBodies[], const double aPoints[][3], const double aForces[][3]) = 0;
	virtual void applyForces(int aN, const AbstractBody *aBodies[], const double *aPoints, const double *aForces) = 0;

	// FORCES EXPRESSED IN BODY-LOCAL FRAME
	virtual void applyForceBodyLocal(const AbstractBody &aBody, const double aPoint[3], const double aForce[3]) = 0;
	virtual void applyForcesBodyLocal(int aN, const AbstractBody *aBodies[], const double aPoints[][3], const double aForces[][3]) = 0;
	virtual void applyForcesBodyLocal(int aN, const AbstractBody *aBodies[], const double *aPoints, const double *aForces) = 0;

	// TORQUES EXPRESSED IN INERTIAL FRAME
	virtual void applyTorque(const AbstractBody &aBody, const double aTorque[3]) = 0;
	virtual void applyTorques(int aN, const AbstractBody *aBodies[], const double aTorques[][3]) = 0;
	virtual void applyTorques(int aN, const AbstractBody *aBodies[], const double *aTorques) = 0;

	// TORQUES EXPRESSED IN BODY-LOCAL FRAME
	virtual void applyTorqueBodyLocal(const AbstractBody &aBody, const double aTorque[3]) = 0;
	virtual void applyTorquesBodyLocal(int aN, const AbstractBody *aBodies[], const double aTorques[][3]) = 0;
	virtual void applyTorquesBodyLocal(int aN, const AbstractBody *aBodies[], const double *aTorques) = 0;

	// GENERALIZED FORCES
	virtual void applyGeneralizedForce(const AbstractCoordinate &aU, double aF) = 0;
	virtual void applyGeneralizedForces(const double aF[]) = 0;
	virtual void applyGeneralizedForces(int aN, const AbstractCoordinate *aU[], const double aF[]) = 0;

	//--------------------------------------------------------------------------
	// LOAD ACCESS AND COMPUTATION
	//--------------------------------------------------------------------------
	virtual double getNetAppliedGeneralizedForce(const AbstractCoordinate &aU) const = 0;
	virtual void computeGeneralizedForces(double aDUDT[], double rF[]) const = 0;
	virtual void computeReactions(double rForces[][3], double rTorques[][3]) const = 0;

	//--------------------------------------------------------------------------
	// CONSTRAINTS
	//--------------------------------------------------------------------------
	virtual void computeConstrainedCoordinates(double rQ[]) const = 0;
	virtual void formCompleteStorages(const OpenSim::Storage &aQIn,
		OpenSim::Storage *&rQComplete,OpenSim::Storage *&rUComplete) const;

	//--------------------------------------------------------------------------
	// EQUATIONS OF MOTION
	//--------------------------------------------------------------------------
	virtual void formMassMatrix(double *rI) = 0;
	virtual void formEulerTransform(const AbstractBody &aBody, double *rE) const = 0;
	virtual void formJacobianTranslation(const AbstractBody &aBody, const double aPoint[3], double *rJ, const AbstractBody *aRefBody=NULL) const = 0;
	virtual void formJacobianOrientation(const AbstractBody &aBody, double *rJ0, const AbstractBody *aRefBody=NULL) const = 0;
	virtual void formJacobianEuler(const AbstractBody &aBody, double *rJE, const AbstractBody *aRefBody=NULL) const = 0;

	//--------------------------------------------------------------------------
	// DERIVATIVES
	//--------------------------------------------------------------------------
	virtual void computeDerivatives(double *dqdt, double *dudt) = 0;

	//--------------------------------------------------------------------------
	// UTILITY
	//--------------------------------------------------------------------------
	virtual void transform(const AbstractBody &aBodyFrom, const double aVec[3], const AbstractBody &aBodyTo, double rVec[3]) const = 0;
	virtual void transform(const AbstractBody &aBodyFrom, const Array<double>& aVec, const AbstractBody &aBodyTo, Array<double>& rVec) const = 0;
	virtual void transformPosition(const AbstractBody &aBodyFrom, const double aPos[3], const AbstractBody &aBodyTo, double rPos[3]) const = 0;
	virtual void transformPosition(const AbstractBody &aBodyFrom, const Array<double>& aPos, const AbstractBody &aBodyTo, Array<double>& rPos) const = 0;
	virtual void transformPosition(const AbstractBody &aBodyFrom, const double aPos[3], double rPos[3]) const = 0;
	virtual void transformPosition(const AbstractBody &aBodyFrom, const Array<double>& aPos, Array<double>& rPos) const = 0;

	virtual double calcDistance(const AbstractBody &aBody1, const double aPoint1[3], const AbstractBody &aBody2, const double aPoint2[3]) const = 0;
	virtual double calcDistance(const AbstractBody &aBody1, const Array<double>& aPoint1, const AbstractBody &aBody2, const Array<double>& aPoint2) const = 0;

	virtual void convertQuaternionsToAngles(double *aQ, double *rQAng) const = 0;
	virtual void convertQuaternionsToAngles(Storage *rQStore) const = 0;
	virtual void convertAnglesToQuaternions(double *aQAng, double *rQ) const = 0;
	virtual void convertAnglesToQuaternions(Storage *rQStore) const = 0;

	void convertRadiansToDegrees(Storage &rStorage) const;
	void convertDegreesToRadians(Storage &rStorage) const;
	void convertDegreesToRadians(double *aQDeg, double *rQRad) const;
	void convertRadiansToDegrees(double *aQRad, double *rQDeg) const;

	virtual void convertAnglesToDirectionCosines(double aE1, double aE2, double aE3, double rDirCos[3][3]) const = 0;
	virtual void convertAnglesToDirectionCosines(double aE1, double aE2, double aE3, double *rDirCos) const = 0;

	virtual void convertDirectionCosinesToAngles(double aDirCos[3][3], double *rE1, double *rE2, double *rE3) const = 0;
	virtual void convertDirectionCosinesToAngles(double *aDirCos, double *rE1, double *rE2, double *rE3) const = 0;

	virtual void convertDirectionCosinesToQuaternions(double aDirCos[3][3],	double *rQ1, double *rQ2, double *rQ3, double *rQ4) const = 0;
	virtual void convertDirectionCosinesToQuaternions(double *aDirCos, double *rQ1, double *rQ2, double *rQ3, double *rQ4) const = 0;

	virtual void convertQuaternionsToDirectionCosines(double aQ1, double aQ2, double aQ3, double aQ4, double rDirCos[3][3]) const = 0;
	virtual void convertQuaternionsToDirectionCosines(double aQ1, double aQ2, double aQ3, double aQ4, double *rDirCos) const = 0;

private:
	void scaleRotationalDofColumns(Storage &rStorage, double factor) const;
//=============================================================================
};	// END of class AbstractDynamicsEngine
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __AbstractDynamicsEngine_h__


