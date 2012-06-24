#ifndef __SimbodyEngine_h__
#define __SimbodyEngine_h__
// SimbodyEngine.h
// Authors: Frank C. Anderson, Ajay Seth
/*
 * Copyright (c)  2007, Stanford University. All rights reserved. 
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
#include <iostream>
#include <string>
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Common/Array.h>
#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/PropertyObj.h>
#include <OpenSim/Common/ArrayPtrs.h>
#include <OpenSim/Common/ScaleSet.h>
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Simulation/Model/JointSet.h>
#include <OpenSim/Simulation/Model/ConstraintSet.h>
#include <OpenSim/Simulation/Model/CoordinateSet.h>
#include <OpenSim/Simulation/Wrap/WrapObject.h>
#include <SimTKsimbody.h>

#ifdef SWIG
	#ifdef OSIMSIMULATION_API
		#undef OSIMSIMULATION_API
		#define OSIMSIMULATION_API
	#endif
#endif

namespace OpenSim {

class Body;
class Constraint;
class Coordinate;
class Joint;
class Body;
class Joint;
class Model;
class CoordinateSet;
class JointSet;

//=============================================================================
//=============================================================================
/**
 * A wrapper class to use the SimTK Simbody dynamics engine as the underlying
 * engine for OpenSim.
 *
 * @authors Frank C. Anderson, Ajay Seth
 * @version 1.0
 */
class OSIMSIMULATION_API SimbodyEngine  : public Object {
OpenSim_DECLARE_CONCRETE_OBJECT(SimbodyEngine, Object);

//=============================================================================
// DATA
//=============================================================================
public:
    /** Pointer to the model that owns this dynamics engine. */
    Model* _model;

protected:
	/** Body used for ground, the inertial frame. */
	Body *_groundBody;

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

#ifndef SWIG
	SimbodyEngine& operator=(const SimbodyEngine &aEngine);
#endif

private:
	void setNull();
	void copyData(const SimbodyEngine &aEngine);
	
public:

#ifndef SWIG
    const Model& getModel() const { return *_model; }
#endif
	Model& getModel() { return *_model; }
    void setModel(Model& aModel) { _model = &aModel; }

	void connectSimbodyEngineToModel(Model& aModel);
	//--------------------------------------------------------------------------
	// COORDINATES
	//--------------------------------------------------------------------------
#ifndef SWIG
	virtual void getUnlockedCoordinates(const SimTK::State& s, CoordinateSet& rUnlockedCoordinates) const;
#endif

	//--------------------------------------------------------------------------
	// SCALING
	//--------------------------------------------------------------------------
#ifndef SWIG
	virtual bool  scale(SimTK::State& s, const ScaleSet& aScaleSet, double aFinalMass = -1.0, bool aPreserveMassDist = false);
#endif

	//--------------------------------------------------------------------------
	// BODY INFORMATION
	//--------------------------------------------------------------------------
	virtual Body& getGroundBody() const;
    virtual WrapObject* getWrapObject(const std::string& aName) const;

	//--------------------------------------------------------------------------
	// INERTIA
	//--------------------------------------------------------------------------
	virtual double getMass() const;
#ifndef SWIG
	virtual void getSystemInertia(const SimTK::State& s, double& rM, SimTK::Vec3& rCOM, double rI[3][3]) const;
	virtual void getSystemInertia(const SimTK::State& s, double& rM, double *rCOM, double *rI) const;

	//--------------------------------------------------------------------------
	// KINEMATICS
	//--------------------------------------------------------------------------
	virtual void getPosition(const SimTK::State& s, const OpenSim::Body &aBody, const SimTK::Vec3& aPoint, SimTK::Vec3& rPos) const;
	virtual void getVelocity(const SimTK::State& s, const OpenSim::Body &aBody, const SimTK::Vec3& aPoint, SimTK::Vec3& rVel) const;
	virtual void getAcceleration(const SimTK::State& s, const OpenSim::Body &aBody, const SimTK::Vec3& aPoint, SimTK::Vec3& rAcc) const;
	virtual void getDirectionCosines(const SimTK::State& s, const OpenSim::Body &aBody, double rDirCos[3][3]) const;
	virtual void getDirectionCosines(const SimTK::State& s, const OpenSim::Body &aBody, double *rDirCos) const;
	virtual void getAngularVelocity(const SimTK::State& s, const OpenSim::Body &aBody, SimTK::Vec3& rAngVel) const;
	virtual void getAngularVelocityBodyLocal(const SimTK::State& s, const OpenSim::Body &aBody, SimTK::Vec3& rAngVel) const;
	virtual void getAngularAcceleration(const SimTK::State& s, const OpenSim::Body &aBody, SimTK::Vec3& rAngAcc) const;
	virtual void getAngularAccelerationBodyLocal(const SimTK::State& s, const OpenSim::Body &aBody, SimTK::Vec3& rAngAcc) const;
	virtual Transform getTransform(const SimTK::State& s, const OpenSim::Body &aBody) const;

	//--------------------------------------------------------------------------
	// LOAD ACCESS AND COMPUTATION
	//--------------------------------------------------------------------------
	virtual void computeReactions(const SimTK::State& s, SimTK::Vector_<SimTK::Vec3>& rForces, SimTK::Vector_<SimTK::Vec3>& rTorques) const;

	//--------------------------------------------------------------------------
	// CONSTRAINTS
	//--------------------------------------------------------------------------
    virtual void formCompleteStorages( const SimTK::State& s, const OpenSim::Storage &aQIn,
       OpenSim::Storage *&rQComplete,OpenSim::Storage *&rUComplete) const;

	//--------------------------------------------------------------------------
	// EQUATIONS OF MOTION
	//--------------------------------------------------------------------------
	virtual void formEulerTransform(const SimTK::State& s, const OpenSim::Body &aBody, double *rE) const;
#endif
	//unimplemented virtual void formMassMatrix(double *rI) {};
	//unimplemented virtual void formJacobianTranslation(const OpenSim::Body &aBody, const SimTK::Vec3& aPoint, double *rJ, const OpenSim::Body *aRefBody=NULL) const {};
	//unimplemented virtual void formJacobianOrientation(const OpenSim::Body &aBody, double *rJ0, const OpenSim::Body *aRefBody=NULL) const {};
	//unimplemented virtual void formJacobianEuler(const OpenSim::Body &aBody, double *rJE, const OpenSim::Body *aRefBody=NULL) const {};

	//--------------------------------------------------------------------------
	// DERIVATIVES
	//--------------------------------------------------------------------------
	virtual void computeDerivatives(const SimTK::State& s, double *dqdt,double *dudt);

	//--------------------------------------------------------------------------
	// UTILITY
	//--------------------------------------------------------------------------
	virtual void transform(const SimTK::State& s, const OpenSim::Body &aBodyFrom, const double aVec[3], const OpenSim::Body &aBodyTo, double rVec[3]) const;
	virtual void transform(const SimTK::State& s, const OpenSim::Body &aBodyFrom, const SimTK::Vec3& aVec, const OpenSim::Body &aBodyTo, SimTK::Vec3& rVec) const;
	virtual void transformPosition(const SimTK::State& s, const OpenSim::Body &aBodyFrom, const double aPos[3], const OpenSim::Body &aBodyTo, double rPos[3]) const;
	virtual void transformPosition(const SimTK::State& s, const OpenSim::Body &aBodyFrom, const SimTK::Vec3& aPos, const OpenSim::Body &aBodyTo, SimTK::Vec3& rPos) const;
	virtual void transformPosition(const SimTK::State& s, const OpenSim::Body &aBodyFrom, const double aPos[3], double rPos[3]) const;
	virtual void transformPosition(const SimTK::State& s, const OpenSim::Body &aBodyFrom, const SimTK::Vec3& aPos, SimTK::Vec3& rPos) const;

	virtual double calcDistance(const SimTK::State& s, const OpenSim::Body &aBody1, const double aPoint1[3], const OpenSim::Body &aBody2, const double aPoint2[3]) const;
	virtual double calcDistance(const SimTK::State& s, const OpenSim::Body &aBody1, const SimTK::Vec3& aPoint1, const OpenSim::Body &aBody2, const SimTK::Vec3& aPoint2) const;

    void convertRadiansToDegrees(Storage &rStorage) const;
    void convertDegreesToRadians(Storage &rStorage) const;
    void convertDegreesToRadians(double *aQDeg, double *rQRad) const;
    void convertRadiansToDegrees(double *aQRad, double *rQDeg) const;


	virtual void convertAnglesToDirectionCosines(double aE1, double aE2, double aE3, double rDirCos[3][3]) const;
	virtual void convertAnglesToDirectionCosines(double aE1, double aE2, double aE3, double *rDirCos) const;

	virtual void convertDirectionCosinesToAngles(double aDirCos[3][3], double *rE1, double *rE2, double *rE3) const;
	virtual void convertDirectionCosinesToAngles(double *aDirCos, double *rE1, double *rE2, double *rE3) const;

	virtual void convertDirectionCosinesToQuaternions(double aDirCos[3][3],	double *rQ1, double *rQ2, double *rQ3, double *rQ4) const;
	virtual void convertDirectionCosinesToQuaternions(double *aDirCos, double *rQ1, double *rQ2, double *rQ3, double *rQ4) const;

	virtual void convertQuaternionsToDirectionCosines(double aQ1, double aQ2, double aQ3, double aQ4, double rDirCos[3][3]) const;
	virtual void convertQuaternionsToDirectionCosines(double aQ1, double aQ2, double aQ3, double aQ4, double *rDirCos) const;

private:
    void scaleRotationalDofColumns(Storage &rStorage, double factor) const;


private:
	friend class Body;
	friend class Coordinate;
	friend class Joint;
	friend class Constraint;
	friend class WeldConstraint;
	friend class CoordinateCouplerConstraint;
	void updateDynamics(SimTK::Stage desiredStage);
	void updateSimbodyModel();

//=============================================================================
};	// END of class SimbodyEngine
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __SimbodyEngine_h__


