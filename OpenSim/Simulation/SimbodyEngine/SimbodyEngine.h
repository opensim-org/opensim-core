#ifndef __SimbodyEngine_h__
#define __SimbodyEngine_h__
/* -------------------------------------------------------------------------- *
 *                         OpenSim:  SimbodyEngine.h                          *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Frank C. Anderson, Ajay Seth                                    *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

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
    void getUnlockedCoordinates(const SimTK::State& s, CoordinateSet& rUnlockedCoordinates) const;
#endif

    //--------------------------------------------------------------------------
    // SCALING
    //--------------------------------------------------------------------------
#ifndef SWIG
    virtual bool  scale(SimTK::State& s, const ScaleSet& aScaleSet, double aFinalMass = -1.0, bool aPreserveMassDist = false);
#endif

    //--------------------------------------------------------------------------
    // KINEMATICS
    //--------------------------------------------------------------------------
    void getPosition(const SimTK::State& s, const PhysicalFrame &aBody, const SimTK::Vec3& aPoint, SimTK::Vec3& rPos) const;
    void getVelocity(const SimTK::State& s, const PhysicalFrame &aBody, const SimTK::Vec3& aPoint, SimTK::Vec3& rVel) const;
    void getAcceleration(const SimTK::State& s, const PhysicalFrame &aBody, const SimTK::Vec3& aPoint, SimTK::Vec3& rAcc) const;
    void getDirectionCosines(const SimTK::State& s, const PhysicalFrame &aBody, double rDirCos[3][3]) const;
    void getDirectionCosines(const SimTK::State& s, const PhysicalFrame &aBody, double *rDirCos) const;
    void getAngularVelocity(const SimTK::State& s, const PhysicalFrame &aBody, SimTK::Vec3& rAngVel) const;
    void getAngularVelocityBodyLocal(const SimTK::State& s, const PhysicalFrame &aBody, SimTK::Vec3& rAngVel) const;
    void getAngularAcceleration(const SimTK::State& s, const PhysicalFrame &aBody, SimTK::Vec3& rAngAcc) const;
    void getAngularAccelerationBodyLocal(const SimTK::State& s, const PhysicalFrame &aBody, SimTK::Vec3& rAngAcc) const;
    SimTK::Transform getTransform(const SimTK::State& s, const PhysicalFrame &aBody) const;

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
    void formEulerTransform(const SimTK::State& s, const PhysicalFrame &aBody, double *rE) const;

    //unimplemented virtual void formMassMatrix(double *rI) {};
    //unimplemented virtual void formJacobianTranslation(const PhysicalFrame &aBody, const SimTK::Vec3& aPoint, double *rJ, const PhysicalFrame *aRefBody=NULL) const {};
    //unimplemented virtual void formJacobianOrientation(const PhysicalFrame &aBody, double *rJ0, const PhysicalFrame *aRefBody=NULL) const {};
    //unimplemented virtual void formJacobianEuler(const PhysicalFrame &aBody, double *rJE, const PhysicalFrame *aRefBody=NULL) const {};

    //--------------------------------------------------------------------------
    // UTILITY
    //--------------------------------------------------------------------------
    void transform(const SimTK::State& s, const PhysicalFrame &aBodyFrom, const double aVec[3], const PhysicalFrame &aBodyTo, double rVec[3]) const;
    void transform(const SimTK::State& s, const PhysicalFrame &aBodyFrom, const SimTK::Vec3& aVec, const PhysicalFrame &aBodyTo, SimTK::Vec3& rVec) const;
    void transformPosition(const SimTK::State& s, const PhysicalFrame &aBodyFrom, const double aPos[3], const PhysicalFrame &aBodyTo, double rPos[3]) const;
    void transformPosition(const SimTK::State& s, const PhysicalFrame &aBodyFrom, const SimTK::Vec3& aPos, const PhysicalFrame &aBodyTo, SimTK::Vec3& rPos) const;
    void transformPosition(const SimTK::State& s, const PhysicalFrame &aBodyFrom, const double aPos[3], double rPos[3]) const;
    void transformPosition(const SimTK::State& s, const PhysicalFrame &aBodyFrom, const SimTK::Vec3& aPos, SimTK::Vec3& rPos) const;

    double calcDistance(const SimTK::State& s, const PhysicalFrame& aBody1,
        const SimTK::Vec3& aPoint1, const PhysicalFrame& aBody2, const SimTK::Vec3& aPoint2) const;
    double calcDistance(const SimTK::State& s, const PhysicalFrame& aBody1, 
        const double aPoint1[3], const PhysicalFrame& aBody2, const double aPoint2[3]) const;


    void convertRadiansToDegrees(Storage &rStorage) const;
    void convertDegreesToRadians(Storage &rStorage) const;
    void convertDegreesToRadians(double *aQDeg, double *rQRad) const;
    void convertRadiansToDegrees(double *aQRad, double *rQDeg) const;


    void convertAnglesToDirectionCosines(double aE1, double aE2, double aE3, double rDirCos[3][3]) const;
    void convertAnglesToDirectionCosines(double aE1, double aE2, double aE3, double *rDirCos) const;

    void convertDirectionCosinesToAngles(double aDirCos[3][3], double *rE1, double *rE2, double *rE3) const;
    void convertDirectionCosinesToAngles(double *aDirCos, double *rE1, double *rE2, double *rE3) const;

    void convertDirectionCosinesToQuaternions(double aDirCos[3][3], double *rQ1, double *rQ2, double *rQ3, double *rQ4) const;
    void convertDirectionCosinesToQuaternions(double *aDirCos, double *rQ1, double *rQ2, double *rQ3, double *rQ4) const;

    void convertQuaternionsToDirectionCosines(double aQ1, double aQ2, double aQ3, double aQ4, double rDirCos[3][3]) const;
    void convertQuaternionsToDirectionCosines(double aQ1, double aQ2, double aQ3, double aQ4, double *rDirCos) const;

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
};  // END of class SimbodyEngine
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __SimbodyEngine_h__


