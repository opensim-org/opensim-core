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
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
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
#include <string>
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/ScaleSet.h>
#include <OpenSim/Common/TimeSeriesTable.h>
#include <OpenSim/Simulation/Model/ConstraintSet.h>
#include "SimTKcommon/Orientation.h"
#include "SimTKcommon/SmallMatrix.h"
#include "SimTKcommon/basics.h"

#ifdef SWIG
    #ifdef OSIMSIMULATION_API
        #undef OSIMSIMULATION_API
        #define OSIMSIMULATION_API
    #endif
#endif

namespace SimTK { class State; }

namespace OpenSim {

class CoordinateSet;
class Model;
class PhysicalFrame;
class Storage;

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
    // KINEMATICS
    //--------------------------------------------------------------------------

    //--------------------------------------------------------------------------
    // LOAD ACCESS AND COMPUTATION
    //--------------------------------------------------------------------------
    virtual void computeReactions(const SimTK::State& s, SimTK::Vector_<SimTK::Vec3>& rForces, SimTK::Vector_<SimTK::Vec3>& rTorques) const;

    //--------------------------------------------------------------------------
    // CONSTRAINTS
    //--------------------------------------------------------------------------
    /**
    * From a potentially partial specification of the generalized coordinates,
    * form a complete storage of the generalized coordinates (q's) and
    * generalized speeds (u's) in radians and radians/s respectively.
    *
    * @param s Used as working memory.
    * @param aQIn Storage containing the q's or a subset of the q's.
    * @param rQComplete Storage containing all the q's.  If q's were not
    * in aQIn, the values are set to 0.0.  When a q is constrained, its value
    * is altered to be consistent with the constraint.  The caller is responsible
    * for deleting the memory associated with this storage. Units are radians.
    * @param rUComplete Storage containing all the u's.  The generalized speeds
    * are obtained by spline fitting the q's and differentiating the splines.
    * When a u is constrained, its value is altered to be consistent with the
    * constraint.  The caller is responsible for deleting the memory
    * associated with this storage. Units are radians/s.
    */
    void formCompleteStorages( const SimTK::State& s, 
        const OpenSim::Storage &aQIn,
        OpenSim::Storage *&rQComplete,
        OpenSim::Storage *&rUComplete) const;

    //--------------------------------------------------------------------------
    // EQUATIONS OF MOTION
    //--------------------------------------------------------------------------

    //unimplemented virtual void formMassMatrix(double *rI) {};
    //unimplemented virtual void formJacobianTranslation(const PhysicalFrame &aBody, const SimTK::Vec3& aPoint, double *rJ, const PhysicalFrame *aRefBody=NULL) const {};
    //unimplemented virtual void formJacobianOrientation(const PhysicalFrame &aBody, double *rJ0, const PhysicalFrame *aRefBody=NULL) const {};
    //unimplemented virtual void formJacobianEuler(const PhysicalFrame &aBody, double *rJE, const PhysicalFrame *aRefBody=NULL) const {};

    //--------------------------------------------------------------------------
    // UTILITY
    //--------------------------------------------------------------------------

    void convertRadiansToDegrees(Storage &rStorage) const;
    void convertRadiansToDegrees(TimeSeriesTable& table) const;
    void convertDegreesToRadians(Storage &rStorage) const;
    void convertDegreesToRadians(TimeSeriesTable& table) const;
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

    //-------------------------------------------------------------------------
    // DEPRECATED METHODS
    //-------------------------------------------------------------------------
    /** @name Deprecated */
    // @{
    /** <b>(Deprecated)</b> Use Frame::getPositionInGround() instead. */
    DEPRECATED_14("use Frame::getPositionInGround() instead")
    void getPosition(const SimTK::State& s, const PhysicalFrame &aBody, const SimTK::Vec3& aPoint, SimTK::Vec3& rPos) const;
    
    /** <b>(Deprecated)</b> Use Frame::getVelocityInGround() instead. */
    DEPRECATED_14("use Frame::getVelocityInGround() instead")
    void getVelocity(const SimTK::State& s, const PhysicalFrame &aBody, const SimTK::Vec3& aPoint, SimTK::Vec3& rVel) const;
    
    /** <b>(Deprecated)</b> Use Frame::getAccelerationInGround() instead. */
    DEPRECATED_14("use Frame::getAccelerationInGround() instead")
    void getAcceleration(const SimTK::State& s, const PhysicalFrame &aBody, const SimTK::Vec3& aPoint, SimTK::Vec3& rAcc) const;
    
    /** <b>(Deprecated)</b> Use Frame::getTransformInGround().R() instead. */
    DEPRECATED_14("use Frame::getTransformInGround().R() instead")
    void getDirectionCosines(const SimTK::State& s, const PhysicalFrame &aBody, double rDirCos[3][3]) const;
    
    /** <b>(Deprecated)</b> Use Frame::getTransformInGround().R() instead. */
    DEPRECATED_14("use Frame::getTransformInGround().R() instead")
    void getDirectionCosines(const SimTK::State& s, const PhysicalFrame &aBody, double *rDirCos) const;
    
    /** <b>(Deprecated)</b> Use Frame::getVelocityInGround()[0] instead. */
    DEPRECATED_14("use Frame::getVelocityInGround()[0] instead")
    void getAngularVelocity(const SimTK::State& s, const PhysicalFrame &aBody, SimTK::Vec3& rAngVel) const;
    
    /** <b>(Deprecated)</b> See Frame::getVelocityInGround()[0]. */
    DEPRECATED_14("see Frame::getVelocityInGround()[0]")
    void getAngularVelocityBodyLocal(const SimTK::State& s, const PhysicalFrame &aBody, SimTK::Vec3& rAngVel) const;
    
    /** <b>(Deprecated)</b> Use Frame::getAccelerationInGround()[0] instead. */
    DEPRECATED_14("use Frame::getAccelerationInGround()[0] instead")
    void getAngularAcceleration(const SimTK::State& s, const PhysicalFrame &aBody, SimTK::Vec3& rAngAcc) const;
    
    /** <b>(Deprecated)</b> See Frame::getAccelerationInGround()[0]. */
    DEPRECATED_14("see Frame::getAccelerationInGround()[0]")
    void getAngularAccelerationBodyLocal(const SimTK::State& s, const PhysicalFrame &aBody, SimTK::Vec3& rAngAcc) const;
    
    /** <b>(Deprecated)</b> Use Frame::getTransformInGround() instead. */
    DEPRECATED_14("use Frame::getTransformInGround() instead")
    SimTK::Transform getTransform(const SimTK::State& s, const PhysicalFrame &aBody) const;

    /** <b>(Deprecated)</b> Use Frame::expressVectorInAnotherFrame() instead. */
    DEPRECATED_14("use Frame::expressVectorInAnotherFrame() instead") 
    void transform(const SimTK::State& s, const PhysicalFrame &aBodyFrom, const double aVec[3], const PhysicalFrame &aBodyTo, double rVec[3]) const;
    
    /** <b>(Deprecated)</b> Use Frame::expressVectorInAnotherFrame() instead. */
    DEPRECATED_14("use Frame::expressVectorInAnotherFrame() instead")
    void transform(const SimTK::State& s, const PhysicalFrame &aBodyFrom, const SimTK::Vec3& aVec, const PhysicalFrame &aBodyTo, SimTK::Vec3& rVec) const;
    
    /** <b>(Deprecated)</b> Use Frame::findStationLocationInAnotherFrame() instead. */
    DEPRECATED_14("use Frame::findStationLocationInAnotherFrame() instead")
    void transformPosition(const SimTK::State& s, const PhysicalFrame &aBodyFrom, const double aPos[3], const PhysicalFrame &aBodyTo, double rPos[3]) const;
    
    /** <b>(Deprecated)</b> Use Frame::findStationLocationInAnotherFrame() instead. */
    DEPRECATED_14("use Frame::findStationLocationInAnotherFrame() instead")
    void transformPosition(const SimTK::State& s, const PhysicalFrame &aBodyFrom, const SimTK::Vec3& aPos, const PhysicalFrame &aBodyTo, SimTK::Vec3& rPos) const;
    
    /** <b>(Deprecated)</b> Use Frame::findStationLocationInGround() instead. */
    DEPRECATED_14("use Frame::findStationLocationInGround() instead")
    void transformPosition(const SimTK::State& s, const PhysicalFrame &aBodyFrom, const double aPos[3], double rPos[3]) const;
    
    /** <b>(Deprecated)</b> Use Frame::findStationLocationInGround() instead. */
    DEPRECATED_14("use Frame::findStationLocationInGround() instead")
    void transformPosition(const SimTK::State& s, const PhysicalFrame &aBodyFrom, const SimTK::Vec3& aPos, SimTK::Vec3& rPos) const;

    /** <b>(Deprecated)</b> Use Point::calcDistanceBetween() or Frame::findStationLocationInGround() instead */
    DEPRECATED_14("use Point::calcDistanceBetween() or Frame::findStationLocationInGround() instead")
    double calcDistance(const SimTK::State& s, const PhysicalFrame& aBody1, const SimTK::Vec3& aPoint1, const PhysicalFrame& aBody2, const SimTK::Vec3& aPoint2) const;
    
    /** <b>(Deprecated)</b> Use Point::calcDistanceBetween() or Frame::findStationLocationInGround() instead */
    DEPRECATED_14("use Point::calcDistanceBetween() or Frame::findStationLocationInGround() instead")
    double calcDistance(const SimTK::State& s, const PhysicalFrame& aBody1, const double aPoint1[3], const PhysicalFrame& aBody2, const double aPoint2[3]) const;

    // @}

private:
    void scaleRotationalDofColumns(Storage &rStorage, double factor) const;
    void scaleRotationalDofColumns(TimeSeriesTable& table, double factor) const;


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


