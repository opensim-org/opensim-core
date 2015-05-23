/* -------------------------------------------------------------------------- *
 *                        OpenSim:  SimbodyEngine.cpp                         *
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

//=============================================================================
// INCLUDES
//=============================================================================
#include <OpenSim/Common/GCVSplineSet.h>
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Common/LoadOpenSimLibrary.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/MarkerSet.h>
#include <OpenSim/Simulation/Model/BodySet.h>

#include "SimbodyEngine.h"
#include "Joint.h"
#include "Coordinate.h"


//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;
using namespace SimTK;

static std::string SimbodyGroundName = "ground";


//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
SimbodyEngine::~SimbodyEngine()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.  
 */
SimbodyEngine::SimbodyEngine() :
    Object()
{
    setNull();
}

//_____________________________________________________________________________
/**
 * Constructor from an XML Document
 */
SimbodyEngine::SimbodyEngine(const string &aFileName) :
    Object(aFileName, false)
{
    setNull();
    connectSimbodyEngineToModel(*_model);
}


//_____________________________________________________________________________
/**
 * Copy constructor.
 */
SimbodyEngine::SimbodyEngine(const SimbodyEngine& aEngine) :
    Object(aEngine)
{
    setNull();
    copyData(aEngine);
    connectSimbodyEngineToModel(*_model);
}


//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================


//_____________________________________________________________________________
/**
 * Copy data members from one SimbodyEngine to another.
 *
 * @param aEngine SimbodyEngine to be copied.
 */
void SimbodyEngine::copyData(const SimbodyEngine &aEngine)
{
    _model = aEngine._model;
}

//_____________________________________________________________________________
/**
 * Set NULL values for all the variable members of this class.
 */
void SimbodyEngine::setNull()
{
    setAuthors("Frank C. Anderson, Ajay Seth");
    _model = NULL;
}
    

//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aModel model containing this SimbodyEngine.
 */
void SimbodyEngine::connectSimbodyEngineToModel(Model& aModel)
{
    _model = &aModel;
}


//=============================================================================
// OPERATORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Assignment operator.
 *
 * @return Reference to this object.
 */
SimbodyEngine& SimbodyEngine::operator=(const SimbodyEngine &aEngine)
{
    // Base class
    Object::operator=(aEngine);
    copyData(aEngine);
    connectSimbodyEngineToModel(*aEngine._model);
    return(*this);
}



//--------------------------------------------------------------------------
// COORDINATES
//--------------------------------------------------------------------------

//_____________________________________________________________________________
/**
 * Get the set of coordinates that are not locked
 *
 * @param rUnlockedCoordinates set of unlocked coordinates is returned here
 */
void SimbodyEngine::getUnlockedCoordinates(const SimTK::State &s, CoordinateSet& rUnlockedCoordinates) const
{
    rUnlockedCoordinates.setSize(0);
    rUnlockedCoordinates.setMemoryOwner(false);

    for (int i = 0; i < _model->getCoordinateSet().getSize(); i++)
        if (!_model->getCoordinateSet().get(i).getLocked(s))
            rUnlockedCoordinates.adoptAndAppend(&_model->getCoordinateSet().get(i));
}



//--------------------------------------------------------------------------
// KINEMATICS
//--------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the inertial position of a point on a body.
 *
 * Note that the configuration of the model must be set before calling this
 * method.
 *
 * @param aBody Pointer to body.
 * @param aPoint Point on the body expressed in the body-local frame.
 * @param rPos Position of the point in the inertial frame.
 */
void SimbodyEngine::getPosition(const SimTK::State& s,
        const PhysicalFrame& aBody, const Vec3& aPoint, Vec3& rPos) const
{
    rPos = aBody.getMobilizedBody().findStationLocationInGround(s, aPoint);
}

//_____________________________________________________________________________
/**
 * Get the inertial velocity of a point on a body.
 *
 * Note that the configuration of the model must be set before calling this
 * method.
 *
 * @param aBody Pointer to body.
 * @param aPoint Point on the body expressed in the body-local frame.
 * @param rVel Velocity of the point in the inertial frame.
 */
void SimbodyEngine::getVelocity(const SimTK::State& s,
        const PhysicalFrame& aBody, const Vec3& aPoint, Vec3& rVel) const
{
    rVel = aBody.getMobilizedBody().findStationVelocityInGround(s, aPoint);
}

//_____________________________________________________________________________
/**
 * Get the inertial acceleration of a point on a body.
 *
 * Note that the configuration of the model must be set and accelerations of
 * the generalized coordinates must be computed before calling this method.
 *
 * @param aBody Pointer to body.
 * @param aPoint Point on the body expressed in the body-local frame.
 * @param rAcc Acceleration of the point in the inertial frame.
 *
 * @see set()
 * @see computeAccelerations()
 */
void SimbodyEngine::getAcceleration(const SimTK::State& s,
        const PhysicalFrame& aBody, const Vec3& aPoint, Vec3& rAcc) const
{
    rAcc = aBody.getMobilizedBody().findStationAccelerationInGround(s, aPoint);
}

//_____________________________________________________________________________
/**
 * Get the body orientation with respect to the ground.
 *
 * @param aBody Pointer to body.
 * @param rDirCos Orientation of the body with respect to the ground frame.
 */
void SimbodyEngine::getDirectionCosines(const SimTK::State& s,
        const PhysicalFrame& aBody, double rDirCos[3][3]) const
{
    Mat33::updAs(&rDirCos[0][0]) =
        aBody.getMobilizedBody().getBodyRotation(s).asMat33();
}

//_____________________________________________________________________________
/**
 * Get the body orientation with respect to the ground.
 *
 * @param aBody Pointer to body.
 * @param rDirCos Orientation of the body with respect to the ground frame.
 */
void SimbodyEngine::getDirectionCosines(const SimTK::State& s,
        const PhysicalFrame& aBody, double *rDirCos) const
{
    Mat33::updAs(rDirCos) =
        aBody.getMobilizedBody().getBodyRotation(s).asMat33();
}

//_____________________________________________________________________________
/**
 * Get the inertial angular velocity of a body in the ground reference frame.
 *
 * @param aBody Pointer to body.
 * @param rAngVel Angular velocity of the body.
 */
void SimbodyEngine::getAngularVelocity(const SimTK::State& s,
        const PhysicalFrame& aBody, Vec3& rAngVel) const
{
    rAngVel = aBody.getMobilizedBody().getBodyAngularVelocity(s);
}

//_____________________________________________________________________________
/**
 * Get the inertial angular velocity in the local body reference frame.
 *
 * @param aBody Pointer to body.
 * @param rAngVel Angular velocity of the body.
 */
void SimbodyEngine::getAngularVelocityBodyLocal(const SimTK::State& s,
        const PhysicalFrame& aBody, Vec3& rAngVel) const
{
    rAngVel = aBody.getMobilizedBody().getBodyAngularVelocity(s);
}

//_____________________________________________________________________________
/**
 * Get the inertial angular acceleration of a body in the ground reference 
 * frame.
 *
 * @param aBody Pointer to body.
 * @param rAngAcc Angular acceleration of the body.
 */
void SimbodyEngine::getAngularAcceleration(const SimTK::State& s,
        const PhysicalFrame& aBody, Vec3& rAngAcc) const
{
    rAngAcc = aBody.getMobilizedBody().getBodyAngularAcceleration(s);
}

//_____________________________________________________________________________
/**
 * Get the inertial angular acceleration in the local body reference frame.
 *
 * @param aBody Pointer to body.
 * @param rAngAcc Angular acceleration of the body.
 */
void SimbodyEngine::getAngularAccelerationBodyLocal(const SimTK::State& s,
        const PhysicalFrame &aBody, Vec3& rAngAcc) const
{
    rAngAcc = aBody.getMobilizedBody().getBodyAngularAcceleration(s);
}

//_____________________________________________________________________________
/**
 * get a copy of the transform from the inertial frame to a body
 *
 * @param aBody
 * @return Transform from inertial frame to body
 */
SimTK::Transform SimbodyEngine::getTransform(const SimTK::State& s,
        const PhysicalFrame& aBody) const
{
    return aBody.getMobilizedBody().getBodyTransform(s);
}

//--------------------------------------------------------------------------
// LOAD ACCESS AND COMPUTATION
//--------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Compute the reaction forces and torques at all the joints in the model.
 * For each joint, what's reported is the force and torque the joint structure
 * applies on the child body at the joint frame attached to the child body.
 * Both vectors are expressed in the ground reference frame.
 *
 * It is necessary to call computeAccelerations() before this method
 * to get valid results.  The cost to calculate joint forces and moments is 114 
 * flops/body.
 *
 * @param rForces Matrix of reaction forces.  The size should be
 * at least NumberOfJoints x 3.
 * @param rTorques Matrix of reaction torques.  The size should be
 * at least NumberOfJoints x 3.
 */
void SimbodyEngine::computeReactions(const SimTK::State& s, Vector_<Vec3>& rForces, Vector_<Vec3>& rTorques) const
{
    // get the number of mobilized bodies in the underlying SimbodyMatterSubsystem
    int nmb = _model->getMatterSubsystem().getNumBodies();
    
    // get the number of bodies in the OpenSim model
    int nj = _model->getNumJoints();

    int nf = rForces.size();
    int ntorq = rTorques.size();

    // there may be more mobilized bodies than joint exposed in the OpenSim model
    // since joints and other components may use (massless) bodies internally
    assert(nmb >= nj);
    assert(nj == nf);
    assert(nf == ntorq);

    SimTK::Vector_<SpatialVec> reactionForces(nj);

    // Systems must be realized to acceleration stage
    _model->getMultibodySystem().realize(s, Stage::Acceleration);
    _model->getMatterSubsystem().calcMobilizerReactionForces(s, reactionForces);


    const JointSet &joints = _model->getJointSet();

    //Separate SimTK SpatialVecs to Forces and Torques  
    // SpatialVec = Vec2<Vec3 torque, Vec3 force>
    for(int i=0; i<nj; i++){
        const SimTK::MobilizedBodyIndex& ix = 
            joints[i].getChildFrame().getMobilizedBodyIndex();
         
        rTorques[i] = reactionForces[ix](0);
        rForces[i] = reactionForces[ix](1);
    }
}

//--------------------------------------------------------------------------
// UTILITY
//--------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Transform a vector from one body to another
 *
 * @param aBodyFrom the body in which the vector is currently expressed
 * @param aPos the vector to be transformed
 * @param aBodyTo the body the vector will be transformed into
 * @param rPos the vector in the aBodyTo frame is returned here
 */
void SimbodyEngine::transform(const SimTK::State& s, const PhysicalFrame &aBodyFrom, const double aVec[3], const PhysicalFrame &aBodyTo, double rVec[3]) const
{
    if(&aBodyFrom == &aBodyTo) { for(int i=0; i<3; i++) { rVec[i]=aVec[i]; } return; }
    const Body* bFrom = (const Body*)&aBodyFrom;
    const Body* bTo = (const Body*)&aBodyTo;

    //Get input vector as a Vec3 to make the call down to Simbody and update the output vector 
    Vec3::updAs(rVec) = _model->getMatterSubsystem().getMobilizedBody(bFrom->getMobilizedBodyIndex()).expressVectorInAnotherBodyFrame(s, Vec3::getAs(aVec), _model->getMatterSubsystem().getMobilizedBody(bTo->getMobilizedBodyIndex()));
}

//_____________________________________________________________________________
/**
 * Transform a vector from one body to another
 *
 * @param aBodyFrom the body in which the vector is currently expressed
 * @param aPos the vector to be transformed
 * @param aBodyTo the body the vector will be transformed into
 * @param rPos the vector in the aBodyTo frame is returned here
 */
void SimbodyEngine::transform(const SimTK::State& s, const PhysicalFrame &aBodyFrom, const Vec3& aVec, const PhysicalFrame &aBodyTo, Vec3& rVec) const
{
    if(&aBodyFrom == &aBodyTo) { rVec=aVec; return; }   

    // Get input vector as a Vec3 to make the call down to Simbody and update
    // the output vector 
    rVec = aBodyFrom.getMobilizedBody().expressVectorInAnotherBodyFrame(s, aVec,
            aBodyTo.getMobilizedBody());
}

//_____________________________________________________________________________
/**
 * Transform a point from one body to another
 *
 * @param aBodyFrom the body in which the point is currently expressed
 * @param aPos the XYZ coordinates of the point
 * @param aBodyTo the body the point will be transformed into
 * @param rPos the XYZ coordinates of the point in the aBodyTo frame are returned here
 */
void SimbodyEngine::
transformPosition(const SimTK::State& s, const PhysicalFrame &aBodyFrom, const
        double aPos[3], const PhysicalFrame &aBodyTo, double rPos[3]) const
{
    if(&aBodyFrom == &aBodyTo) {
       for (int i=0; i<3; i++) rPos[i] = aPos[i];
        return;
    }

    // Get input vector as a Vec3 to make the call down to Simbody and update
    // the output vector.
    Vec3::updAs(rPos) =
        aBodyFrom.getMobilizedBody().findStationLocationInAnotherBody(s,
                Vec3::getAs(aPos), aBodyTo.getMobilizedBody());
}

//_____________________________________________________________________________
/**
 * Transform a point from one body to another
 *
 * @param aBodyFrom the body in which the point is currently expressed
 * @param aPos the XYZ coordinates of the point
 * @param aBodyTo the body the point will be transformed into
 * @param rPos the XYZ coordinates of the point in the aBodyTo frame are returned here
 */
void SimbodyEngine::
transformPosition(const SimTK::State& s, const PhysicalFrame &aBodyFrom, const
        Vec3& aPos, const PhysicalFrame &aBodyTo, Vec3& rPos) const
{
    if(&aBodyFrom == &aBodyTo) {
       for (int i=0; i<3; i++) rPos[i] = aPos[i];
        return;
    }

    // Get input vector as a Vec3 to make the call down to Simbody and update
    // the output vector.
    rPos = aBodyFrom.getMobilizedBody().findStationLocationInAnotherBody(s,
            aPos, aBodyTo.getMobilizedBody());
}

//_____________________________________________________________________________
/**
 * Transform a point from one body to the ground body
 *
 * @param aBodyFrom the body in which the point is currently expressed
 * @param aPos the XYZ coordinates of the point
 * @param rPos the XYZ coordinates of the point in the ground frame are returned here
 */
void SimbodyEngine::transformPosition(const SimTK::State& s, const PhysicalFrame &aBodyFrom, const double aPos[3], double rPos[3]) const
{
    const Body* bFrom = (const Body*)&aBodyFrom;

    // Get input vector as a Vec3 to make the call down to Simbody and update
    // the output vector.
    Vec3::updAs(rPos) =
        aBodyFrom.getMobilizedBody().findStationLocationInGround(s,
                Vec3::getAs(aPos));
}

//_____________________________________________________________________________
/**
 * Transform a point from one body to the ground body
 *
 * @param aBodyFrom the body in which the point is currently expressed
 * @param aPos the XYZ coordinates of the point
 * @param rPos the XYZ coordinates of the point in the ground frame are returned here
 */
void SimbodyEngine::
transformPosition(const SimTK::State& s, const PhysicalFrame& aBodyFrom,
        const Vec3& aPos, Vec3& rPos) const
{
    _model->getMultibodySystem().realize(s, SimTK::Stage::Position);
    rPos = aBodyFrom.getMobilizedBody().findStationLocationInGround(s, aPos);
}

//_____________________________________________________________________________
/**
 * Calculate the distance between a point on one body and a point on another body
 *
 * @param aBody1 the body that the first point is expressed in
 * @param aPoint1 the XYZ coordinates of the first point
 * @param aBody2 the body that the second point is expressed in
 * @param aPoint2 the XYZ coordinates of the second point
 * @return the distance between aPoint1 and aPoint2
 */
double SimbodyEngine::
calcDistance(const SimTK::State& s, const PhysicalFrame& aBody1,
const Vec3& aPoint1, const PhysicalFrame& aBody2, const Vec3& aPoint2)
    const
{
    return aBody1.getMobilizedBody().calcStationToStationDistance(s, aPoint1,
            aBody2.getMobilizedBody(), aPoint2);
}

//_____________________________________________________________________________
/**
 * Calculate the distance between a point on one body and a point on another body
 *
 * @param aBody1 the body that the first point is expressed in
 * @param aPoint1 the XYZ coordinates of the first point
 * @param aBody2 the body that the second point is expressed in
 * @param aPoint2 the XYZ coordinates of the second point
 * @return the distance between aPoint1 and aPoint2
 */
double SimbodyEngine::calcDistance(const SimTK::State& s, const PhysicalFrame&
        aBody1, const double aPoint1[3], const PhysicalFrame& aBody2, const
        double aPoint2[3]) const
{
    return aBody1.getMobilizedBody().calcStationToStationDistance(s,
            Vec3::getAs(aPoint1), aBody2.getMobilizedBody(),
            Vec3::getAs(aPoint2));
}

//_____________________________________________________________________________
/**
 * Convert angles to direction cosines.
 * @param aE1 1st Euler angle.
 * @param aE2 2nd Euler angle.
 * @param aE3 3rd Euler angle.
 * @param rDirCos Vector of direction cosines.
 */
void SimbodyEngine::convertAnglesToDirectionCosines(double aE1, double aE2, double aE3, double rDirCos[3][3]) const
{
    Vec3 angs(aE1, aE2, aE3);
    Rotation aRot; 
    aRot.setRotationToBodyFixedXYZ(angs);
    Mat33::updAs(&rDirCos[0][0]) = aRot.asMat33();
}

//_____________________________________________________________________________
/**
 * Convert angles to direction cosines.
 * @param aE1 1st Euler angle.
 * @param aE2 2nd Euler angle.
 * @param aE3 3rd Euler angle.
 * @param rDirCos Vector of direction cosines.
 */
void SimbodyEngine::convertAnglesToDirectionCosines(double aE1, double aE2, double aE3, double *rDirCos) const
{
    if(rDirCos==NULL) return;
    
    Vec3 angs(aE1, aE2, aE3);
    Rotation aRot; 
    aRot.setRotationToBodyFixedXYZ(angs);
    Mat33::updAs(&rDirCos[0]) = aRot.asMat33();
}

//_____________________________________________________________________________
/**
 * Convert direction cosines to angles.
 * @param aDirCos Vector of direction cosines.
 * @param rE1 1st Euler angle.
 * @param rE2 2nd Euler angle.
 * @param rE3 3rd Euler angle.
 */
void SimbodyEngine::convertDirectionCosinesToAngles(double aDirCos[3][3], double *rE1, double *rE2, double *rE3) const
{
    Vec3 ang = Rotation(Rotation::getAs(&aDirCos[0][0])).convertRotationToBodyFixedXYZ();
    *rE1 = ang[0];
    *rE2 = ang[1];
    *rE3 = ang[2];
}

//_____________________________________________________________________________
/**
 * Convert direction cosines to angles.
 * @param aDirCos Vector of direction cosines.
 * @param rE1 1st Euler angle.
 * @param rE2 2nd Euler angle.
 * @param rE3 3rd Euler angle.
 */
void SimbodyEngine::convertDirectionCosinesToAngles(double *aDirCos, double *rE1, double *rE2, double *rE3) const
{
    if(!aDirCos) return;
    Vec3 ang = Rotation(Rotation::getAs(aDirCos)).convertRotationToBodyFixedXYZ();
    *rE1 = ang[0];
    *rE2 = ang[1];
    *rE3 = ang[2];
}

//_____________________________________________________________________________
/**
 * Convert direction cosines to quaternions.
 * @param aDirCos Vector of direction cosines.
 * @param rQ1 1st Quaternion.
 * @param rQ2 2nd Quaternion.
 * @param rQ3 3rd Quaternion.
 * @param rQ4 4th Quaternion.
 */
void SimbodyEngine::convertDirectionCosinesToQuaternions(double aDirCos[3][3], double *rQ1, double *rQ2, double *rQ3, double *rQ4) const
{
    Quaternion quat = Rotation(Rotation::getAs(&aDirCos[0][0])).convertRotationToQuaternion();
    *rQ1 = quat[0];
    *rQ2 = quat[1];
    *rQ3 = quat[2];
    *rQ4 = quat[3];
}

//_____________________________________________________________________________
/**
 * Convert direction cosines to quaternions.
 * @param aDirCos Vector of direction cosines.
 * @param rQ1 1st Quaternion.
 * @param rQ2 2nd Quaternion.
 * @param rQ3 3rd Quaternion.
 * @param rQ4 4th Quaternion.
 */
void SimbodyEngine::convertDirectionCosinesToQuaternions(double *aDirCos, double *rQ1, double *rQ2, double *rQ3, double *rQ4) const
{
    if(aDirCos==NULL) return;
    Quaternion quat = Rotation(Rotation::getAs(aDirCos)).convertRotationToQuaternion();
    *rQ1 = quat[0];
    *rQ2 = quat[1];
    *rQ3 = quat[2];
    *rQ4 = quat[3];
}

//_____________________________________________________________________________
/**
 * Convert quaternions to direction cosines.
 * @param aQ1 1st Quaternion.
 * @param aQ2 2nd Quaternion.
 * @param aQ3 3rd Quaternion.
 * @param aQ4 4th Quaternion.
 * @param rDirCos Vector of direction cosines.
 */
void SimbodyEngine::convertQuaternionsToDirectionCosines(double aQ1, double aQ2, double aQ3, double aQ4, double rDirCos[3][3]) const
{
    Rotation R;
    R.setRotationFromQuaternion(Quaternion(Vec4(aQ1, aQ2, aQ3, aQ4)));

    Mat33::updAs(&rDirCos[0][0]) = R.asMat33();
}

//_____________________________________________________________________________
/**
 * Convert quaternions to direction cosines.
 * @param aQ1 1st Quaternion.
 * @param aQ2 2nd Quaternion.
 * @param aQ3 3rd Quaternion.
 * @param aQ4 4th Quaternion.
 * @param rDirCos Vector of direction cosines.
 */
void SimbodyEngine::convertQuaternionsToDirectionCosines(double aQ1, double aQ2, double aQ3, double aQ4, double *rDirCos) const
{
    if(rDirCos==NULL) return;
    Rotation R;
    R.setRotationFromQuaternion(Quaternion(Vec4(aQ1, aQ2, aQ3, aQ4)));

    Mat33::updAs(rDirCos) = R.asMat33();
}


//--- Private Utility Methods Below Here ---


void SimbodyEngine::formEulerTransform(const SimTK::State& s, const PhysicalFrame &aBody, double *rE) const
{
    if (&aBody && rE)
    {
        // GET ORIENTATION OF aBody
        double ang[3], dc[3][3];

        getDirectionCosines(s, aBody, dc);
        convertDirectionCosinesToAngles(dc, &ang[0], &ang[1], &ang[2]);

        // ROW 1
        *rE =  cos(ang[2]) / cos(ang[1]);
        rE++;  *rE = -sin(ang[2]) / cos(ang[1]);
        rE++;  *rE = 0.0;

        // ROW 2
        rE++;  *rE = sin(ang[2]);
        rE++;  *rE = cos(ang[2]);
        rE++;  *rE = 0.0;

        // ROW 3
        rE++;  *rE = -cos(ang[2]) * sin(ang[1]) / cos(ang[1]);
        rE++;  *rE =  sin(ang[1]) * sin(ang[2]) / cos(ang[1]);
        rE++;  *rE = 1.0;
    }
}

//_____________________________________________________________________________
/**
 * Scale the dynamics engine
 *
 * @param aScaleSet the set of XYZ scale factors for the bodies
 * @param aFinalMass the mass that the scaled model should have
 * @param aPreserveMassDist whether or not the masses of the
 *        individual bodies should be scaled with the body scale factors.
 * @return Whether or not scaling was successful.
 */
bool SimbodyEngine::scale(SimTK::State& s, const ScaleSet& aScaleSet, double aFinalMass, bool aPreserveMassDist)
{
    // second argument is a flag to scale the masses of the bodies along with their
    // geometry. If preserve mass distribution is true then the masses are not scaled.
    _model->updBodySet().scale(aScaleSet, !aPreserveMassDist);

    // When bodies are scaled, the properties of the model are changed.
    // The general rule is that you MUST recreate and initialize the system 
    // when properties of the model change. We must do that here or
    // we will be querying a stale system (e.g. wrong body properties!).
    s = _model->initSystem();

    // Now that the masses of the individual bodies have
    // been scaled (if aPreserveMassDist == false), get the
    // total mass and compare it to aFinalMass in order to
    // determine how much to scale the body masses again,
    // so that the total model mass comes out to aFinalMass.
    if (aFinalMass > 0.0)
    {
        double mass = _model->getTotalMass(s);
        if (mass > 0.0)
        {
            double factor = aFinalMass / mass;
            for (int i = 0; i < _model->getBodySet().getSize(); i++){
                _model->getBodySet().get(i).scaleMass(factor);
            }
            
            // recreate system and update state after updating masses
            s = _model->initSystem();

            double newMass = _model->getTotalMass(s);
            double normDiffMass = abs(aFinalMass - newMass) / aFinalMass;

            // check if the difference in after scale mass and the specified 
            // subject (target) mass is significant
            if (normDiffMass > SimTK::SignificantReal) {
                throw Exception("Model::scale() scaled model mass does not match specified subject mass.");
            }
        }
    }
    
    // Now scale the joints.
    _model->updJointSet().scale(aScaleSet);

    // Now scale translational coupled coordinate constraints.
    _model->updConstraintSet().scale(aScaleSet);

    // Now scale the markers.
    _model->updMarkerSet().scale(aScaleSet);

    return true;
}

//=============================================================================
// CONFIGURATION
//=============================================================================
/**
 * From a potentially partial specification of the generalized coordinates,
 * form a complete storage of the generalized coordinates (q's) and
 * generalized speeds (u's).
 *
 * @param aQIn Storage containing the q's or a subset of the q's.  Rotational
 * q's should be in degrees.
 * @param rQComplete Storage containing all the q's.  If q's were not
 * in aQIn, the values are set to 0.0.  When a q is constrained, its value
 * is altered to be consistent with the constraint.  The caller is responsible
 * for deleting the memory associated with this storage.
 * @param rUComplete Storage containing all the u's.  The generalized speeds
 * are obtained by spline fitting the q's and differentiating the splines.
 * When a u is constrained, its value is altered to be consisten with the
 * constraint.  The caller is responsible for deleting the memory
 * associated with this storage.
 */
void SimbodyEngine::
formCompleteStorages( const SimTK::State& s, const OpenSim::Storage &aQIn,
    OpenSim::Storage *&rQComplete,OpenSim::Storage *&rUComplete) const
{
    int i;
    int nq = _model->getNumCoordinates();
    int nu = _model->getNumSpeeds();

    // Get coordinate file indices
    Array<string> columnLabels, speedLabels, coordStateNames;
    columnLabels.append("time");
    speedLabels = columnLabels;

    Array<int> index(-1,nq);
    const CoordinateSet& coordinateSet = _model->getCoordinateSet();
    int sizeCoordSet = coordinateSet.getSize();
    for(i=0;i<sizeCoordSet;i++) {
        Coordinate& coord = coordinateSet.get(i);
        string prefix = coord.getJoint().getName() + "/" + coord.getName() + "/";
        coordStateNames = coord.getStateVariableNames();
        columnLabels.append(prefix+coordStateNames[0]);
        speedLabels.append(prefix+coordStateNames[1]);
        int fix = aQIn.getStateIndex(coord.getName());
        if (fix < 0) {
            fix = aQIn.getStateIndex(columnLabels[i+1]);
        }

        index[i] = fix;
        if(index[i]<0) {
            string msg = "Model::formCompleteStorages(): WARNING- Did not find column ";
            msg += coordStateNames[0];
            msg += " in storage object.\n";
            cout << msg << endl;
        }
    }

    // Extract Coordinates
    double time;
    Array<double> data(0.0);
    Array<double> q(0.0,nq);
    Storage *qStore = new Storage();
    qStore->setInDegrees(aQIn.isInDegrees());
    qStore->setName("GeneralizedCoordinates");
    qStore->setColumnLabels(columnLabels);
    int size = aQIn.getSize();
    StateVector *vector;
    int j;
    for(i=0;i<size;i++) {
        vector = aQIn.getStateVector(i);
        data = vector->getData();
        time = vector->getTime();

        for(j=0;j<nq;j++) {
            q[j] = 0.0;
            if(index[j]<0) continue;
            q[j] = data[index[j]];
        }

        qStore->append(time,nq,&q[0]);
    }

    // Convert to radians
    if (aQIn.isInDegrees())
        convertDegreesToRadians(*qStore);


    // Compute generalized speeds
    GCVSplineSet tempQset(5,qStore);
    Storage *uStore = tempQset.constructStorage(1);

    // Compute constraints
    Array<double> qu(0.0,nq+nu);
    rQComplete = new Storage();
    rUComplete = new Storage();
    State constrainedState = s;
     _model->getMultibodySystem().realize(constrainedState, s.getSystemStage());
    for(i=0;i<size;i++) {
        qStore->getTime(i,time);
        qStore->getData(i,nq,&qu[0]);
        uStore->getData(i,nq,&qu[nq]);
        for (int j = 0; j < nq; j++) {
            Coordinate& coord = coordinateSet.get(j);
            coord.setValue(constrainedState, qu[j], false);
            coord.setSpeedValue(constrainedState, qu[nq+j]);
        }
        _model->assemble(constrainedState);
        for (int j = 0; j < nq; j++) {
            Coordinate& coord = coordinateSet.get(j);
            qu[j] = coord.getValue(constrainedState);
            qu[nq+j] = coord.getSpeedValue(constrainedState);
        }
        rQComplete->append(time,nq,&qu[0]);
        rUComplete->append(time,nu,&qu[nq]);
    }
    
    delete qStore;
    
    // Compute storage object for simulation
    // Need to set column labels before converting rad->deg
    rQComplete->setColumnLabels(columnLabels);
    rUComplete->setColumnLabels(speedLabels);
    // Convert back to degrees
    convertRadiansToDegrees(*rQComplete);
    convertRadiansToDegrees(*rUComplete);
}

//=============================================================================
// UTILITY
//=============================================================================
//_____________________________________________________________________________
/**
 */
void SimbodyEngine::scaleRotationalDofColumns(Storage &rStorage, double factor) const
{
    const Array<std::string>& columnLabels = rStorage.getColumnLabels();
    int ncols = columnLabels.getSize();
    if(ncols == 0)
        throw Exception("SimbodyEngine.scaleRotationalDofColumns: ERROR- storage has no labels, can't determine coordinate types for deg<->rad conversion",
                             __FILE__,__LINE__);

    // Loop through the coordinates in the model. For each one that is rotational,
    // see if it has a corresponding column of data. If it does, multiply that
    // column by the given scaling factor.
    std::string shortName = "";
    std::string prefix = "";
    int index = -1;
    const CoordinateSet& coordinateSet = _model->getCoordinateSet();
    
    // first column is time, so skip
    for (int i = 1; i < ncols; i++) {
        const std::string& name = columnLabels[i];
        index = coordinateSet.getIndex(name);
        if (index < 0){
            std::string::size_type back = name.rfind("/");
            prefix = name.substr(0, back);
            shortName = name.substr(back+1, name.length()-back);
            index = coordinateSet.getIndex(shortName);
            // This is a necessary hack to use new component naming,
            // but SimbodyEngine will be deprecated and so will this code- aseth
            if (index < 0){ // could be a speed then trim off _u
                back = prefix.rfind("/");
                shortName = prefix.substr(back+1, prefix.length()-back);
                index = coordinateSet.getIndex(shortName);
            }
        }
        if (index >= 0){
            const Coordinate& coord = coordinateSet.get(index);
            if (coord.getMotionType() == Coordinate::Rotational) {
                // assumes first data colum is 0 whereas labels has time as 0
                rStorage.multiplyColumn(i-1, factor);
            }
        }
    }
}
//_____________________________________________________________________________
/**
 * Convert the rotational generalized coordinates or speeds from units of
 * degrees to units of radians for all the state-vectors in an Storage
 * object.  Coordinates/speeds are identified by column names.
 *
 * @param rStorage Storage object.
 */
void SimbodyEngine::convertDegreesToRadians(Storage &rStorage) const
{
    assert(rStorage.isInDegrees());
    scaleRotationalDofColumns(rStorage, SimTK_DEGREE_TO_RADIAN);
    rStorage.setInDegrees(false);
}
//_____________________________________________________________________________
/**
 * Convert the rotational generalized coordinates or speeds from units of
 * radians to units of degrees for all the state-vectors in an Storage
 * object.  Coordinates/speeds are identified by column names.
 *
 * @param rStorage Storage object.
 */
void SimbodyEngine::convertRadiansToDegrees(Storage &rStorage) const
{
    assert(!rStorage.isInDegrees());
    scaleRotationalDofColumns(rStorage, SimTK_RADIAN_TO_DEGREE);
    rStorage.setInDegrees(true);
}
//_____________________________________________________________________________
/**
 * Convert an array of Q/U values from degrees to radians. The sizes of the
 * arrays are assumed to be equal to the number of Us.
 *
 * @param aQDeg Array of values, in degrees
 * @param rQRad Array of values, in radians
 */
void SimbodyEngine::convertDegreesToRadians(double *aQDeg, double *rQRad) const
{
    const CoordinateSet& coordinateSet = _model->getCoordinateSet();

    // The arrays aQDeg and rQRad are assumed to be of size getNumSpeeds() or greater.
    // It is also assumed that aQDeg[N] corresponds to the first N coordinates
    // in the model, whether those N values are coordinates or speeds.
    for (int i = 0; i < _model->getNumSpeeds(); i++)
    {
        if (coordinateSet.get(i).getMotionType() == Coordinate::Rotational)
            rQRad[i] = aQDeg[i] * SimTK_DEGREE_TO_RADIAN;
        else
            rQRad[i] = aQDeg[i];
    }
}
//_____________________________________________________________________________
/**
 * Convert an array of Q/U values from radians to degrees. The sizes of the
 * arrays are assumed to be equal to the number of Us.
 *
 * @param aQRad Array of values, in radians
 * @param rQDeg Array of values, in degrees
 */
void SimbodyEngine::convertRadiansToDegrees(double *aQRad, double *rQDeg) const
{
    const CoordinateSet& coordinateSet = _model->getCoordinateSet();

    // The arrays aQRad and rQDeg are assumed to be of size getNumSpeeds() or greater.
    // It is also assumed that aQRad[N] corresponds to the first N coordinates
    // in the model, whether those N values are coordinates or speeds.
    for (int i = 0; i < _model->getNumSpeeds(); i++)
    {
        if (coordinateSet.get(i).getMotionType() == Coordinate::Rotational)
            rQDeg[i] = aQRad[i] * SimTK_RADIAN_TO_DEGREE;
        else
            rQDeg[i] = aQRad[i];
    }
}
