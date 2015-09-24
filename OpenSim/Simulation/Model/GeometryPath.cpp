/* -------------------------------------------------------------------------- *
 *                         OpenSim:  GeometryPath.cpp                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2013 Stanford University and the Authors                *
 * Author(s): Peter Loan, Ajay Seth                                           *
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
#include "GeometryPath.h"
#include <OpenSim/Simulation/SimbodyEngine/Coordinate.h>
#include <OpenSim/Simulation/SimbodyEngine/Body.h>
#include <OpenSim/Simulation/SimbodyEngine/SimbodyEngine.h>
#include "ConditionalPathPoint.h"
#include "MovingPathPoint.h"
#include "PointForceDirection.h"
#include <OpenSim/Simulation/Wrap/PathWrapPoint.h>
#include <OpenSim/Simulation/Wrap/WrapResult.h>
#include <OpenSim/Simulation/Wrap/PathWrap.h>
#include "CoordinateSet.h"
#include "Model.h"

#include "ModelVisualizer.h"
//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;
using namespace SimTK;
using SimTK::Vec3;

static const Vec3 DefaultDefaultColor(.5,.5,.5); // boring gray 

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/*
 * Default constructor.
 */
GeometryPath::GeometryPath() :
    ModelComponent(),
    _preScaleLength(0.0)
{
    setAuthors("Peter Loan");
    constructProperties();
 }

//_____________________________________________________________________________
/*
 * Perform set up functions after model has been deserialized or copied.
 *
 * @param aModel The model containing this path.
 */
void GeometryPath::extendConnectToModel(Model& aModel)
{
    Super::extendConnectToModel(aModel);

    // aModel will be NULL when objects are being registered.
    if (&aModel == NULL)
        return;

    // Name the path points based on the current path
    // (i.e., the set of currently active points is numbered
    // 1, 2, 3, ...).
    namePathPoints(0);

    for (int i = 0; i < get_PathWrapSet().getSize(); i++)
        upd_PathWrapSet().get(i).connectToModelAndPath(aModel, *this);

    for (int i = 0; i < get_PathPointSet().getSize(); i++){
        upd_PathPointSet().get(i).connectToModelAndPath(aModel, *this);
    }

}

//_____________________________________________________________________________
/*
 * Create the SimTK state, discrete and/or cache for this GeometryPath.
 */
 void GeometryPath::extendAddToSystem(SimTK::MultibodySystem& system) const 
{
    Super::extendAddToSystem(system);

    // Allocate cache entries to save the current length and speed(=d/dt length)
    // of the path in the cache. Length depends only on q's so will be valid
    // after Position stage, speed requires u's also so valid at Velocity stage.
    addCacheVariable<double>("length", 0.0, SimTK::Stage::Position);
    addCacheVariable<double>("speed", 0.0, SimTK::Stage::Velocity);
    // Cache the set of points currently defining this path.
    Array<PathPoint *> pathPrototype;
    addCacheVariable<Array<PathPoint *> >
        ("current_path", pathPrototype, SimTK::Stage::Position);
    // When displaying, cache the set of points to be used to draw the path.
    addCacheVariable<Array<PathPoint *> >
        ("current_display_path", pathPrototype, SimTK::Stage::Position);

    // We consider this cache entry valid any time after it has been created
    // and first marked valid, and we won't ever invalidate it.
    addCacheVariable<SimTK::Vec3>("color", get_default_color(), 
                                  SimTK::Stage::Topology);
}

 void GeometryPath::extendInitStateFromProperties(SimTK::State& s) const
{
    Super::extendInitStateFromProperties(s);
    markCacheVariableValid(s, "color"); // it is OK at its default value
}

//------------------------------------------------------------------------------
//                         GENERATE DECORATIONS
//------------------------------------------------------------------------------
// The GeometryPath takes care of drawing itself here, using information it
// can extract from the supplied state, including position information and
// color information that may have been calculated as late as Stage::Dynamics.
// For example, muscles may want the color to reflect activation level and 
// other path-using components might want to use forces (tension). We will
// ensure that the state has been realized to Stage::Dynamics before looking
// at it. (It is only guaranteed to be at Stage::Position here.)
void GeometryPath::
generateDecorations(bool fixed, const ModelDisplayHints& hints, 
                    const SimTK::State& state, 
                    SimTK::Array_<SimTK::DecorativeGeometry>& appendToThis) const
{        
    // There is no fixed geometry to generate here.
    if (fixed) { return; }
    // Ensure that the state has been realized to Stage::Dynamics to give
    // clients of this path a chance to calculate meaningful color information.
    this->getModel().getMultibodySystem().realize(state, SimTK::Stage::Dynamics);

    this->updateDisplayPath(state);

    const Array<PathPoint*>& points = getCurrentDisplayPath(state);

    if (points.getSize() == 0) { return; }

    const PathPoint* lastPoint = points[0];
    Vec3 lastLoc_B = lastPoint->getLocation();
    MobilizedBodyIndex lastBody = lastPoint->getBody().getMobilizedBodyIndex();

    if (hints.get_show_path_points())
        DefaultGeometry::drawPathPoint(lastBody, lastLoc_B, getColor(state),
        appendToThis);

    const SimTK::SimbodyMatterSubsystem& matter = getModel().getMatterSubsystem();
    Vec3 lastPos = matter.getMobilizedBody(lastBody)
        .getBodyTransform(state) * lastLoc_B;

    for (int j = 1; j < points.getSize(); j++) {
        const PathPoint* point = points[j];
        const Vec3 loc_B = point->getLocation();
        const MobilizedBodyIndex body = point->getBody().getMobilizedBodyIndex();

        if (hints.get_show_path_points())
            DefaultGeometry::drawPathPoint(body, loc_B, getColor(state),
            appendToThis);

        Vec3 pos = matter.getMobilizedBody(body).getBodyTransform(state)*loc_B;
        // Line segments will be in ground frame
        appendToThis.push_back(DecorativeLine(lastPos, pos)
            .setLineThickness(4)
            .setColor(getColor(state)).setBodyId(0).setIndexOnBody(j));

        lastPos = pos;
    }

}

//_____________________________________________________________________________
/*
 * Connect properties to local pointers.
 */
void GeometryPath::constructProperties()
{
    constructProperty_PathPointSet(PathPointSet());

    constructProperty_PathWrapSet(PathWrapSet());
    
    Vec3 defaultColor = SimTK::White;
    constructProperty_default_color(defaultColor);
}

//_____________________________________________________________________________
/*
 * Name the path points based on their position in the set. To keep the
 * names up to date, this method should be called every time the path changes.
 *
 * @param aStartingIndex The index of the first path point to name.
 */
void GeometryPath::namePathPoints(int aStartingIndex)
{
    char indx[5];
    for (int i = aStartingIndex; i < get_PathPointSet().getSize(); i++)
    {
        sprintf(indx,"%d",i+1);
        PathPoint& point = get_PathPointSet().get(i);
        if (point.getName()=="" && _owner) {
            point.setName(_owner->getName() + "-P" + indx);
        }
    }
}

//_____________________________________________________________________________
/*
 * get the current path of the path
 *
 * @return The array of currently active path points.
 * 
 */
const OpenSim::Array <PathPoint*> & GeometryPath::
getCurrentPath(const SimTK::State& s)  const
{
    computePath(s);   // compute checks if path needs to be recomputed
    return getCacheVariableValue< Array<PathPoint*> >(s, "current_path");
}

// get the path as PointForceDirections directions 
// CAUTION: the return points are heap allocated; you must delete them yourself! 
// (TODO: that is really lame)
void GeometryPath::
getPointForceDirections(const SimTK::State& s, 
                        OpenSim::Array<PointForceDirection*> *rPFDs) const
{
    int i;
    PathPoint* start;
    PathPoint* end;
    const OpenSim::PhysicalFrame* startBody;
    const OpenSim::PhysicalFrame* endBody;
    const Array<PathPoint*>& currentPath = getCurrentPath(s);

    int np = currentPath.getSize();

    const SimbodyEngine& engine = _model->getSimbodyEngine();

    rPFDs->ensureCapacity(np);
    
    for (i = 0; i < np; i++) {
        PointForceDirection *pfd = 
            new PointForceDirection(currentPath[i]->getLocation(), 
                                    *(OpenSim::Body*)&(currentPath[i]->getBody()), Vec3(0));
        rPFDs->append(pfd);
    }

    for (i = 0; i < np-1; i++) {
        start = currentPath[i];
        end = currentPath[i+1];
        startBody = &start->getBody();
        endBody = &end->getBody();

        if (startBody != endBody)
        {
            Vec3 posStart, posEnd;
            Vec3 direction(0);

            // Find the positions of start and end in the inertial frame.
            //engine.getPosition(s, start->getBody(), start->getLocation(), posStart);
            posStart = start->getBody().getGroundTransform(s)*start->getLocation();
            
            //engine.getPosition(s, end->getBody(), end->getLocation(), posEnd);
            posEnd = end->getBody().getGroundTransform(s)*end->getLocation();

            // Form a vector from start to end, in the inertial frame.
            direction = (posEnd - posStart);

            // Check that the two points are not coincident.
            // This can happen due to infeasible wrapping of the path,
            // when the origin or insertion enters the wrapping surface.
            // This is a temporary fix, since the wrap algorithm should
            // return NaN for the points and/or throw an Exception- aseth
            if (direction.norm() < SimTK::SignificantReal){
                direction = direction*SimTK::NaN;
            }
            else{
                direction = direction.normalize();
            }

            // Get resultant direction at each point 
            rPFDs->get(i)->addToDirection(direction);
            rPFDs->get(i+1)->addToDirection(-direction);
        }
    }
}

/* add in the equivalent spatial forces on bodies for an applied tension 
    along the GeometryPath to a set of bodyForces */
void GeometryPath::addInEquivalentForces(const SimTK::State& s,
    const double& tension, 
    SimTK::Vector_<SimTK::SpatialVec>& bodyForces,
    SimTK::Vector& mobilityForces) const
{
    PathPoint* start = NULL;
    PathPoint* end = NULL;
    const SimTK::MobilizedBody* bo = NULL;
    const SimTK::MobilizedBody* bf = NULL;
    const Array<PathPoint*>& currentPath = getCurrentPath(s);
    int np = currentPath.getSize();

    const SimTK::SimbodyMatterSubsystem& matter = 
                                        getModel().getMatterSubsystem();

    // start point, end point,  direction, and force vectors in ground
    Vec3 po(0), pf(0), dir(0), force(0);
    // partial velocity of point in body expressed in ground 
    Vec3 dPodq_G(0), dPfdq_G(0);

    // gen force (torque) due to moving point under tension
    double fo, ff;

    for (int i = 0; i < np-1; ++i) {
        start = currentPath[i];
        end = currentPath[i+1];

        bo = &start->getBody().getMobilizedBody();
        bf = &end->getBody().getMobilizedBody();

        if (bo != bf) {
            // Find the positions of start and end in the inertial frame.
            po = bo->findStationLocationInGround(s, start->getLocation());
            pf = bf->findStationLocationInGround(s, end->getLocation());

            // Form a vector from start to end, in the inertial frame.
            dir = (pf - po);

            // Check that the two points are not coincident.
            // This can happen due to infeasible wrapping of the path,
            // when the origin or insertion enters the wrapping surface.
            // This is a temporary fix, since the wrap algorithm should
            // return NaN for the points and/or throw an Exception- aseth
            if (dir.norm() < SimTK::SignificantReal){
                dir = dir*SimTK::NaN;
            }
            else{
                dir = dir.normalize();
            }
        
            force = tension*dir;

            // add in the tension point forces to body forces
            bo->applyForceToBodyPoint(s, start->getLocation(), force, 
                bodyForces);
            bf->applyForceToBodyPoint(s, end->getLocation(), -force,
                bodyForces);

            const MovingPathPoint* mppo = 
                    dynamic_cast<MovingPathPoint *>(start);

            if(mppo){
                // torque (genforce) contribution due to relative movement 
                // of a via point w.r.t. the body it is connected to.
                dPodq_G = bo->expressVectorInGroundFrame(s, start->getdPointdQ(s));
                fo = ~dPodq_G*force;            

                // get the mobilized body the coordinate is couple to.
                const SimTK::MobilizedBody& mpbod =
                    matter.getMobilizedBody(mppo->getXCoordinate()->getBodyIndex());

                // apply the generalized (mobility) force to the coordinate's body
                mpbod.applyOneMobilityForce(s, 
                    mppo->getXCoordinate()->getMobilizerQIndex(), 
                    fo, mobilityForces);
            }

            // do the same for the end point of this segment of the path
            const MovingPathPoint* mppf = 
                    dynamic_cast<MovingPathPoint *>(end);

            if(mppf){
                dPfdq_G = bf->expressVectorInGroundFrame(s, end->getdPointdQ(s));
                ff = ~dPfdq_G*(-force);

                // get the mobilized body the coordinate is couple to.
                const SimTK::MobilizedBody& mpbod =
                    matter.getMobilizedBody(mppf->getXCoordinate()->getBodyIndex());

                mpbod.applyOneMobilityForce(s, 
                    mppf->getXCoordinate()->getMobilizerQIndex(), 
                    ff, mobilityForces);
            }
            
        }       
    }
}

//_____________________________________________________________________________
/*
 * get the current display path of the path
 *
 * @return The array of currently active path points, plus points along the
 * surfaces of the wrap objects (if any).
 * 
 */
const OpenSim::Array<PathPoint*>& GeometryPath::
getCurrentDisplayPath(const SimTK::State& s) const
{
    // update the geometry to make sure the current display path is up to date.
    // updateGeometry(s);
    return getCacheVariableValue<Array <PathPoint*> >(s, "current_display_path" );
}

//_____________________________________________________________________________
/*
 * Update the geometric representation of the path.
 * The resulting geometry is maintained at the VisibleObject layer.
 * This function should not be made public. It is called internally
 * by compute() only when the path has changed.
 * 
 */
void GeometryPath::updateGeometry(const SimTK::State& s) const
{
    // Check if the current path needs to recomputed.
    computePath(s);

    // If display path is current do not need to recompute it.
    if (isCacheVariableValid(s, "current_display_path"))
        return;
   
    // Updating the display path will also validate the current_display_path 
    // cache variable.
    updateDisplayPath(s);
}

//=============================================================================
// GET
//=============================================================================
//-----------------------------------------------------------------------------
// LENGTH
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/*
 * Compute the total length of the path.
 *
 * @return Total length of the path.
 */
double GeometryPath::getLength( const SimTK::State& s) const
{
    computePath(s);  // compute checks if path needs to be recomputed
    return( getCacheVariableValue<double>(s, "length") );
}

void GeometryPath::setLength( const SimTK::State& s, double length ) const
{
    setCacheVariableValue<double>(s, "length", length); 
}

void GeometryPath::setColor(const SimTK::State& s, const SimTK::Vec3& color) const
{
    setCacheVariableValue<SimTK::Vec3>(s, "color", color);
}

Vec3 GeometryPath::getColor(const SimTK::State& s) const
{
    return getCacheVariableValue<SimTK::Vec3>(s, "color");
}

//_____________________________________________________________________________
/*
 * Compute the lengthening speed of the path.
 *
 * @return lengthening speed of the path.
 */
double GeometryPath::getLengtheningSpeed( const SimTK::State& s) const
{
    computeLengtheningSpeed(s);
    return getCacheVariableValue<double>(s, "speed");
}
void GeometryPath::setLengtheningSpeed( const SimTK::State& s, double speed ) const
{
    setCacheVariableValue<double>(s, "speed", speed);    
}

void GeometryPath::setPreScaleLength( const SimTK::State& s, double length ) {
    _preScaleLength = length;
}
double GeometryPath::getPreScaleLength( const SimTK::State& s) const {
    return _preScaleLength;
}

//=============================================================================
// UTILITY
//=============================================================================
//_____________________________________________________________________________
/*
 * Add a new path point, with default location, to the path.
 *
 * @param aIndex The position in the pathPointSet to put the new point in.
 * @param aBody The body to attach the point to.
 * @return Pointer to the newly created path point.
 */
PathPoint* GeometryPath::
addPathPoint(const SimTK::State& s, int aIndex, PhysicalFrame& aBody)
{
    PathPoint* newPoint = new PathPoint();
    newPoint->setBody(aBody);
    Vec3& location = newPoint->getLocation();
    placeNewPathPoint(s, location, aIndex, aBody);
    newPoint->connectToModelAndPath(getModel(), *this);
    upd_PathPointSet().insert(aIndex, newPoint);

    // Rename the path points starting at this new one.
    namePathPoints(aIndex);

    // Update start point and end point in the wrap instances so that they
    // refer to the same path points they did before the new point
    // was added. These indices are 1-based.
    aIndex++;
    for (int i=0; i<get_PathWrapSet().getSize(); i++) {
        int startPoint = get_PathWrapSet().get(i).getStartPoint();
        int endPoint = get_PathWrapSet().get(i).getEndPoint();
        if (startPoint != -1 && aIndex <= startPoint)
            get_PathWrapSet().get(i).setStartPoint(s,startPoint + 1);
        if (endPoint != -1 && aIndex <= endPoint)
            get_PathWrapSet().get(i).setEndPoint(s,endPoint + 1);
    }


    return newPoint;
}

PathPoint* GeometryPath::
appendNewPathPoint(const std::string& proposedName, 
                   PhysicalFrame& aBody, const SimTK::Vec3& aPositionOnBody)
{
    PathPoint* newPoint = new PathPoint();
    newPoint->setBody(aBody);
    newPoint->setName(proposedName);
    for (int i=0; i<3; i++) newPoint->setLocationCoord(i, aPositionOnBody[i]);
    upd_PathPointSet().adoptAndAppend(newPoint);

    return newPoint;
}

//_____________________________________________________________________________
/*
 * Determine an appropriate default XYZ location for a new path point.
 * Note that this method is internal and should not be called directly on a new 
 * point as the point is not really added to the path (done in addPathPoint() 
 * instead)
 * @param aOffset The XYZ location to be determined.
 * @param aIndex The position in the pathPointSet to put the new point in.
 * @param aBody The body to attach the point to.
 */
void GeometryPath::
placeNewPathPoint(const SimTK::State& s, SimTK::Vec3& aOffset, int aIndex, 
                  const PhysicalFrame& aBody)
{
    // The location of the point is determined by moving a 'distance' from 'base' 
    // along a vector from 'start' to 'end.' 'base' is the existing path point 
    // that is in or closest to the index aIndex. 'start' and 'end' are existing
    // path points--which ones depends on where the new point is being added. 
    // 'distance' is 0.5 for points added to the middle of a path (so the point
    // appears halfway between the two adjacent points), and 0.2 for points that
    // are added to either end of the path. If there is only one point in the 
    // path, the new point is put 0.01 units away in all three dimensions.
    if (get_PathPointSet().getSize() > 1) {
        int start, end, base;
        double distance;
        if (aIndex == 0) {
            start = 1;
            end = 0;
            base = end;
            distance = 0.2;
        } else if (aIndex >= get_PathPointSet().getSize()) {
            start = aIndex - 2;
            end = aIndex - 1;
            base = end;
            distance = 0.2;
        } else {
            start = aIndex;
            end = aIndex - 1;
            base = start;
            distance = 0.5;
        }

        const Vec3& startPt = get_PathPointSet()[start].getLocation();
        const Vec3& endPt = get_PathPointSet()[end].getLocation();
        const Vec3& basePt = get_PathPointSet()[base].getLocation();

        Vec3 startPt2 = get_PathPointSet()[start].getBody()
            .findLocationInAnotherFrame(s, startPt, aBody);

        Vec3 endPt2 = get_PathPointSet()[end].getBody()
            .findLocationInAnotherFrame(s, endPt, aBody);

        aOffset = basePt + distance * (endPt2 - startPt2);
    } else if (get_PathPointSet().getSize() == 1){
        int foo = 0;
        for (int i = 0; i < 3; i++) {
            aOffset[i] = get_PathPointSet().get(foo).getLocation()[i] + 0.01;
        }
    }
    else {  // first point, do nothing?
    }
}

//_____________________________________________________________________________
/*
 * See if a path point can be deleted. All paths must have at least two
 * active path points to define the path.
 *
 * @param aIndex The index of the point to delete.
 * @return Whether or not the point can be deleted.
 */
bool GeometryPath::canDeletePathPoint( int aIndex)
{
    // A path point can be deleted only if there would remain
    // at least two other fixed points.
    int numOtherFixedPoints = 0;
    for (int i = 0; i < get_PathPointSet().getSize(); i++) {
        if (i != aIndex) {
            if (!(  get_PathPointSet().get(i).getConcreteClassName()
                  ==("ConditionalPathPoint")))
                numOtherFixedPoints++;
        }
    }

    if (numOtherFixedPoints >= 2)
        return true;

    return false;
}

//_____________________________________________________________________________
/*
 * Delete a path point.
 *
 * @param aIndex The index of the point to delete.
 * @return Whether or not the point was deleted.
 */
bool GeometryPath::deletePathPoint(const SimTK::State& s, int aIndex)
{
    if (canDeletePathPoint(aIndex) == false)
        return false;

    upd_PathPointSet().remove(aIndex);

    // rename the path points starting at the deleted position
    namePathPoints(aIndex);

    // Update start point and end point in the wrap instances so that they
    // refer to the same path points they did before the point was
    // deleted. These indices are 1-based. If the point deleted is start
    // point or end point, the path wrap range is made smaller by one point.
    aIndex++;
    for (int i=0; i<get_PathWrapSet().getSize(); i++) {
        int startPoint = get_PathWrapSet().get(i).getStartPoint();
        int endPoint   = get_PathWrapSet().get(i).getEndPoint();

        if (   (startPoint != -1 && aIndex < startPoint) 
            || (startPoint > get_PathPointSet().getSize()))
            get_PathWrapSet().get(i).setStartPoint(s, startPoint - 1);

        if (   endPoint > 1 
            && aIndex <= endPoint 
            && (   (endPoint > startPoint) 
                || (endPoint > get_PathPointSet().getSize())))
            get_PathWrapSet().get(i).setEndPoint(s, endPoint - 1);
    }

    return true;
}

//_____________________________________________________________________________
/*
 * Replace a path point in the set with another point. The new one is made a
 * member of all the same groups as the old one, and is inserted in the same
 * place the old one occupied.
 *
 *  @param aOldPathPoint Path point to remove.
 *  @param aNewPathPoint Path point to add.
 */
bool GeometryPath::
replacePathPoint(const SimTK::State& s, PathPoint* aOldPathPoint, 
                 PathPoint* aNewPathPoint) 
{
    if (aOldPathPoint != NULL && aNewPathPoint != NULL) {
        int count = 0;
        int index = get_PathPointSet().getIndex(aOldPathPoint);
        // If you're switching from non-via to via, check to make sure that the
        // path will be left with at least 2 non-via points.
        ConditionalPathPoint* oldVia = 
            dynamic_cast<ConditionalPathPoint*>(aOldPathPoint);
        ConditionalPathPoint* newVia = 
            dynamic_cast<ConditionalPathPoint*>(aNewPathPoint);
        if (oldVia == NULL && newVia != NULL) {
            for (int i=0; i<get_PathPointSet().getSize(); i++) {
                if (i != index) {
                    if (dynamic_cast<ConditionalPathPoint*>
                                        (&get_PathPointSet().get(i)) == NULL)
                        count++;
                }
            }
        } else {
            count = 2;
        }
        if (count >= 2 && index >= 0) {
            upd_PathPointSet().set(index, aNewPathPoint, true);
            //computePath(s);
            return true;
        }
    }
    return false;
}

//_____________________________________________________________________________
/*
 * Create a new wrap instance and add it to the set.
 *
 * @param aWrapObject The wrap object to use in the new wrap instance.
 */
void GeometryPath::addPathWrap(WrapObject& aWrapObject)
{
    PathWrap* newWrap = new PathWrap();
    newWrap->setWrapObject(aWrapObject);
    newWrap->setMethod(PathWrap::hybrid);
    newWrap->connectToModelAndPath(getModel(), *this);
    upd_PathWrapSet().adoptAndAppend(newWrap);
}

//_____________________________________________________________________________
/*
 * Move a wrap instance up in the list. Changing the order of wrap instances for
 * a path may affect how the path wraps over the wrap objects.
 *
 * @param aIndex The index of the wrap instance to move up.
 */
void GeometryPath::moveUpPathWrap(const SimTK::State& s, int aIndex)
{
    if (aIndex > 0) {
        // Make sure wrap object is not deleted by remove().
        upd_PathWrapSet().setMemoryOwner(false); 

        PathWrap& wrap = get_PathWrapSet().get(aIndex);
        upd_PathWrapSet().remove(aIndex);
        upd_PathWrapSet().insert(aIndex - 1, &wrap);
        upd_PathWrapSet().setMemoryOwner(true);
    }
}

//_____________________________________________________________________________
/*
 * Move a wrap instance down in the list. Changing the order of wrap instances
 * for a path may affect how the path wraps over the wrap objects.
 *
 * @param aIndex The index of the wrap instance to move down.
 */
void GeometryPath::moveDownPathWrap(const SimTK::State& s, int aIndex)
{
    if (aIndex < get_PathWrapSet().getSize() - 1) {
        // Make sure wrap object is not deleted by remove().
        upd_PathWrapSet().setMemoryOwner(false);

        PathWrap& wrap = get_PathWrapSet().get(aIndex);
        upd_PathWrapSet().remove(aIndex);
        upd_PathWrapSet().insert(aIndex + 1, &wrap);
        upd_PathWrapSet().setMemoryOwner(true);
    }
}

//_____________________________________________________________________________
/*
 * Delete a wrap instance.
 *
 * @param aIndex The index of the wrap instance to delete.
 */
void GeometryPath::deletePathWrap(const SimTK::State& s, int aIndex)
{
    upd_PathWrapSet().remove(aIndex);

}

//=============================================================================
// SCALING
//=============================================================================
//_____________________________________________________________________________
/*
 * Perform computations that need to happen before the path is scaled.
 * For this object, that entails calculating and storing the path
 * length in the current body position.
 *
 * @param aScaleSet XYZ scale factors for the bodies.
 */
void GeometryPath::preScale(const SimTK::State& s, const ScaleSet& aScaleSet)
{
    setPreScaleLength( s,  getLength(s) );
}

//_____________________________________________________________________________
/*
 * Scale the path based on XYZ scale factors for each body.
 *
 * @param aScaleSet XYZ scale factors for the bodies.
 * @return Whether path was successfully scaled or not.
 */
void GeometryPath::scale(const SimTK::State& s, const ScaleSet& aScaleSet)
{
    for (int i = 0; i < get_PathPointSet().getSize(); i++)
    {
        const string& bodyName = get_PathPointSet().get(i).getBodyName();
        for (int j = 0; j < aScaleSet.getSize(); j++)
        {
            Scale& aScale = aScaleSet.get(j);
            if (bodyName == aScale.getSegmentName())
            {
                Vec3 scaleFactors(1.0);
                aScale.getScaleFactors(scaleFactors);
                upd_PathPointSet().get(i).scale(s, scaleFactors);
            }
        }
    }
}

//_____________________________________________________________________________
/*
 * Perform computations that need to happen after the path is scaled.
 * For this object, that entails updating the path.
 *
 * @param aScaleSet XYZ scale factors for the bodies.
 */
void GeometryPath::postScale(const SimTK::State& s, const ScaleSet& aScaleSet)
{
    // Recalculate the path. This will also update the geometry.
    // Done here since scale is invoked before bodies are scaled
    // so we may not have enough info to update (e.g. wrapping, via points)
    computePath(s);
}

//--------------------------------------------------------------------------
// COMPUTATIONS
//--------------------------------------------------------------------------
//=============================================================================
// PATH, WRAPPING, AND MOMENT ARM
//=============================================================================
//_____________________________________________________________________________
/*
 * Calculate the current path.
 */
void GeometryPath::computePath(const SimTK::State& s) const
{
    const SimTK::Stage& sg = s.getSystemStage();
    
    if (isCacheVariableValid(s, "current_path"))  {
        return;
    }

    // Clear the current path.
    Array<PathPoint*>& currentPath = 
        updCacheVariableValue<Array<PathPoint*> >(s, "current_path");
    currentPath.setSize(0);

    // >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    // TODO: This is a bug. Here we are going to modify moving path point
    // locations *in the property* which means they are out of sync with 
    // the supplied state. These updates should be done in the state cache.
    GeometryPath * mutableThis = const_cast<GeometryPath*>(this);

    // Add the active fixed and moving via points to the path.
    for (int i = 0; i < get_PathPointSet().getSize(); i++) {
        // If this is a moving path point, update its location *in the
        // PathPointSet property* (BAD).
        mutableThis->upd_PathPointSet()[i].update(s);
        if (get_PathPointSet()[i].isActive(s))
            currentPath.append(&get_PathPointSet()[i]); // <--- !!!!BAD
    }
  
    // Use the current path so far to check for intersection with wrap objects, 
    // which may add additional points to the path.
    applyWrapObjects(s, currentPath);
    calcLengthAfterPathComputation(s, currentPath);

    markCacheVariableValid(s, "current_path");
}

//_____________________________________________________________________________
/*
 * Compute lengthening speed of the path.
 */
void GeometryPath::computeLengtheningSpeed(const SimTK::State& s) const
{
    if (isCacheVariableValid(s, "speed"))
        return;

    SimTK::Vec3 posRelative, velRelative;
    SimTK::Vec3 posStartInertial, posEndInertial, 
                velStartInertial, velEndInertial;
    SimTK::Vec3 velStartLocal, velEndLocal, velStartMoving, velEndMoving;
    PathPoint *start, *end;
    const Array<PathPoint*>& currentPath = getCurrentPath(s);

    double speed = 0.0;

    const SimbodyEngine& engine = _model->getSimbodyEngine();

    for (int i = 0; i < currentPath.getSize() - 1; i++) {
        start = currentPath[i];
        end   = currentPath[i+1];

        // Find the positions and velocities in the inertial frame.
        posStartInertial =
            start->getBody().getGroundTransform(s)*start->getLocation();

        posEndInertial =
            end->getBody().getGroundTransform(s)*end->getLocation();

        velStartInertial = start->getBody().getMobilizedBody()
            .findStationVelocityInGround(s, start->getLocation());

        velEndInertial = end->getBody().getMobilizedBody()
            .findStationVelocityInGround(s, end->getLocation());

        // The points might be moving in their local bodies' reference frames
        // (MovingPathPoints and possibly PathWrapPoints) so find their
        // local velocities and transform them to the inertial frame.
        start->getVelocity(s, velStartLocal);
        end->getVelocity(s, velEndLocal);

        velStartMoving = start->getBody()
            .expressVectorInAnotherFrame(s, velStartLocal, getModel().getGround());

        velEndMoving = end->getBody()
            .expressVectorInAnotherFrame(s, velEndLocal, getModel().getGround());

        // Calculate the relative positions and velocities.
        posRelative = posEndInertial - posStartInertial;
        velRelative =   (velEndInertial + velEndMoving) 
                      - (velStartInertial + velStartMoving);

        // Normalize the vector from start to end.
        posRelative = posRelative.normalize();

        // Dot the relative velocity with the unit vector from start to end,
        // and add this speed to the running total.
        speed += (velRelative[0] * posRelative[0] +
                  velRelative[1] * posRelative[1] +
                  velRelative[2] * posRelative[2]);
    }

    setLengtheningSpeed(s, speed);
}

//_____________________________________________________________________________
/*
 * Apply the wrap objects to the current path.
 */
void GeometryPath::
applyWrapObjects(const SimTK::State& s, Array<PathPoint*>& path) const 
{
    if (get_PathWrapSet().getSize() < 1)
        return;

    WrapResult best_wrap;
    Array<int> result, order;

    result.setSize(get_PathWrapSet().getSize());
    order.setSize(get_PathWrapSet().getSize());

    // Set the initial order to be the order they are listed in the path.
    for (int i = 0; i < get_PathWrapSet().getSize(); i++)
        order[i] = i;

    // If there is only one wrap object, calculate the wrapping only once.
    // If there are two or more objects, perform up to 8 iterations where
    // the result from one wrap object is used as the starting point for
    // the next wrap.
    const int maxIterations = get_PathWrapSet().getSize() < 2 ? 1 : 8;
    double last_length = SimTK::Infinity;
    for (int kk = 0; kk < maxIterations; kk++)
    {
        for (int i = 0; i < get_PathWrapSet().getSize(); i++)
        {
            result[i] = 0;
            PathWrap& ws = get_PathWrapSet().get(order[i]);
            const WrapObject* wo = ws.getWrapObject();
            best_wrap.wrap_pts.setSize(0);
            double min_length_change = SimTK::Infinity;

            // First remove this object's wrapping points from the current path.
            for (int j = 0; j <path.getSize(); j++) {
                if( path.get(j) == &ws.getWrapPoint(0)) {
                    path.remove(j); // remove the first wrap point
                    path.remove(j); // remove the second wrap point
                    break;
                }
            }


            if (wo->getActive()) {
                // startPoint and endPoint in wrapStruct represent the 
                // user-defined starting and ending points in the array of path 
                // points that should be considered for wrapping. These indices 
                // take into account via points, whether or not they are active. 
                // Thus they are indices into mp_orig[], not mp[] (also, mp[] 
                // may contain wrapping points from previous wrap objects, which
                // would mess up the starting and ending indices). But the goal 
                // is to find starting and ending indices in mp[] to consider
                // for wrapping over this wrap object. Here is how that is done:

                // 1. startPoint and endPoint are 1-based, so subtract 1 from 
                // them to get indices into get_PathPointSet(). -1 (or any value
                // less than 1) means use the first (or last) point.
                const int wrapStart = (ws.getStartPoint() < 1
                                            ? 0 
                                            : ws.getStartPoint() - 1);
                const int wrapEnd   = (ws.getEndPoint() < 1
                                            ? get_PathPointSet().getSize() - 1 
                                            : ws.getEndPoint() - 1);

                // 2. Scan forward from wrapStart in get_PathPointSet() to find 
                // the first point that is active. Store a pointer to it (smp).
                int jfwd = wrapStart;
                for (; jfwd <= wrapEnd; jfwd++)
                    if (get_PathPointSet().get(jfwd).isActive(s))
                        break;
                if (jfwd > wrapEnd) // there are no active points in the path
                    return;
                const PathPoint* const smp = &get_PathPointSet().get(jfwd);

                // 3. Scan backwards from wrapEnd in get_PathPointSet() to find 
                // the last point that is active. Store a pointer to it (emp).
                int jrev = wrapEnd;
                for (; jrev >= wrapStart; jrev--)
                    if (get_PathPointSet().get(jrev).isActive(s))
                        break;
                if (jrev < wrapStart) // there are no active points in the path
                    return;
                const PathPoint* const emp = &get_PathPointSet().get(jrev);

                // 4. Now find the indices of smp and emp in _currentPath.
                int start=-1, end=-1;
                for (int j = 0; j < path.getSize(); j++) {
                    if (path.get(j) == smp)
                        start = j;
                    if (path.get(j) == emp)
                        end = j;
                }
                if (start == -1 || end == -1) // this should never happen
                    return;

                // You now have indices into _currentPath (which is a list of 
                // all currently active points, including wrap points) that 
                // represent the used-defined range of points to consider for 
                // wrapping over this wrap object. Check each path segment in 
                // this range, choosing the best wrap as the one that changes 
                // the path segment length the least:
                for (int pt1 = start; pt1 < end; pt1++)
                {
                    const int pt2 = pt1 + 1;

                    // As long as the two points are not auto wrap points on the
                    // same wrap object, check them for wrapping.
                    if (   path.get(pt1)->getWrapObject() == NULL 
                        || path.get(pt2)->getWrapObject() == NULL 
                        || (   path.get(pt1)->getWrapObject() 
                            != path.get(pt2)->getWrapObject()))
                    {
                        WrapResult wr;
                        wr.startPoint = pt1;
                        wr.endPoint   = pt2;

                        result[i] = wo->wrapPathSegment(s, *path.get(pt1), 
                                                        *path.get(pt2), ws, wr);
                        if (result[i] == WrapObject::mandatoryWrap) {
                            // "mandatoryWrap" means the path actually 
                            // intersected the wrap object. In this case, you 
                            // *must* choose this segment as the "best" one for
                            // wrapping. If the path has more than one segment 
                            // that intersects the object, the first one is
                            // taken as the mandatory wrap (this is considered 
                            // an ill-conditioned case).
                            best_wrap = wr;
                            // Store the best wrap in the pathWrap for possible 
                            // use next time.
                            ws.setPreviousWrap(wr);
                            break;
                        }  else if (result[i] == WrapObject::wrapped) {
                            // "wrapped" means the path segment was wrapped over
                            // the object, but you should consider the other 
                            // segments as well to see if one
                            // wraps with a smaller length change.
                            double path_length_change = 
                                calcPathLengthChange(s, *wo, wr, path);
                            if (path_length_change < min_length_change)
                            {
                                best_wrap = wr;
                                // Store the best wrap in the pathWrap for 
                                // possible use next time
                                ws.setPreviousWrap(wr);
                                min_length_change = path_length_change;
                            } else {
                                // The wrap was not shorter than the current 
                                // minimum, so just free the wrap points that 
                                // were allocated.
                                wr.wrap_pts.setSize(0);
                            }
                        } else {
                            // Nothing to do.
                        }
                    }
                }

                // Deallocate previous wrapping points if necessary.
                ws.getWrapPoint(1).getWrapPath().setSize(0);

                if (best_wrap.wrap_pts.getSize() == 0) {
                    ws.resetPreviousWrap();
                    ws.getWrapPoint(1).getWrapPath().setSize(0);
                } else {
                    // If wrapping did occur, copy wrap info into the PathStruct.
                    ws.getWrapPoint(0).getWrapPath().setSize(0);

                    Array<SimTK::Vec3>& wrapPath = ws.getWrapPoint(1).getWrapPath();
                    wrapPath = best_wrap.wrap_pts;

                    // In OpenSim, all conversion to/from the wrap object's 
                    // reference frame will be performed inside 
                    // wrapPathSegment(). Thus, all points in this function will
                    // be in their respective body reference frames.
                    // for (j = 0; j < wrapPath.getSize(); j++){
                    //    convert_from_wrap_object_frame(wo, wrapPath.get(j));
                    //    convert(ms->modelnum, wrapPath.get(j), wo->segment, 
                    //            ms->ground_segment);
                    // }

                    ws.getWrapPoint(0).setWrapLength(0.0);
                    ws.getWrapPoint(1).setWrapLength(best_wrap.wrap_path_length);
                    ws.getWrapPoint(0).setBody(wo->getBody());
                    ws.getWrapPoint(1).setBody(wo->getBody());

                    ws.getWrapPoint(0).setLocation(s,best_wrap.r1);
                    ws.getWrapPoint(1).setLocation(s,best_wrap.r2);

                    // Now insert the two new wrapping points into mp[] array.
                    path.insert(best_wrap.endPoint, &ws.getWrapPoint(0));
                    path.insert(best_wrap.endPoint + 1, &ws.getWrapPoint(1));
                }
            }
        }

        const double length = calcLengthAfterPathComputation(s, path); 
        if (std::abs(length - last_length) < 0.0005) {
            break;
        } else {
            last_length = length;
        }

        if (kk == 0 && get_PathWrapSet().getSize() > 1) {
            // If the first wrap was a no wrap, and the second was a no wrap
            // because a point was inside the object, switch the order of
            // the first two objects and try again.
            if (   result[0] == WrapObject::noWrap 
                && result[1] == WrapObject::insideRadius)
            {
                order[0] = 1;
                order[1] = 0;

                // remove wrap object 0 from the list of path points
                PathWrap& ws = get_PathWrapSet().get(0);
                for (int j = 0; j < path.getSize(); j++) {
                    if (path.get(j) == &ws.getWrapPoint(0)) {
                        path.remove(j); // remove the first wrap point
                        path.remove(j); // remove the second wrap point
                        break;
                    }
                }
            }
        }
    }
}

//_____________________________________________________________________________
/*
 * _calc_path_length_change - given the output of a successful path wrap
 * over a wrap object, determine the percent change in length of the
 * path segment incurred by wrapping.
 */
double GeometryPath::
calcPathLengthChange(const SimTK::State& s, const WrapObject& wo, 
                     const WrapResult& wr, const Array<PathPoint*>& path)  const
{
    const PathPoint* pt1 = path.get(wr.startPoint);
    const PathPoint* pt2 = path.get(wr.endPoint);

    double straight_length = getModel().getSimbodyEngine()
        .calcDistance(s, pt1->getBody(), pt1->getLocation(),
                         pt2->getBody(), pt2->getLocation());

    const Vec3& p1 = pt1->getLocation();
    const Vec3& p2 = pt2->getLocation();
    double wrap_length = getModel().getSimbodyEngine()
        .calcDistance(s, pt1->getBody(), p1, wo.getBody(), wr.r1);
    wrap_length += wr.wrap_path_length;
    wrap_length += getModel().getSimbodyEngine()
        .calcDistance(s, wo.getBody(), wr.r2, pt2->getBody(), p2);

    return wrap_length - straight_length; // return absolute diff, not relative
}

//_____________________________________________________________________________
/*
 * Compute the total length of the path. This function
 * assumes that the path has already been updated.
 */
double GeometryPath::
calcLengthAfterPathComputation(const SimTK::State& s, 
                               const Array<PathPoint*>& currentPath) const
{
    double length = 0.0;

    const SimbodyEngine& engine = _model->getSimbodyEngine();

    for (int i = 0; i < currentPath.getSize() - 1; i++) {
        const PathPoint* p1 = currentPath[i];
        const PathPoint* p2 = currentPath[i+1];

        // If both points are wrap points on the same wrap object, then this
        // path segment wraps over the surface of a wrap object, so just add in 
        // the pre-calculated length.
        if (   p1->getWrapObject() 
            && p2->getWrapObject() 
            && p1->getWrapObject() == p2->getWrapObject()) 
        {
            const PathWrapPoint* smwp = dynamic_cast<const PathWrapPoint*>(p2);
            if (smwp)
                length += smwp->getWrapLength();
        } else {
            length += engine.calcDistance(s, p1->getBody(), p1->getLocation(), 
                                             p2->getBody(), p2->getLocation());
        }
    }

    setLength(s,length);
    return( length );
}

//_____________________________________________________________________________
/*
 * Compute the path's moment arms for  specified coordinate.
 *
 * @param aCoord, the coordinate
 */   
double GeometryPath::
computeMomentArm(const SimTK::State& s, const Coordinate& aCoord) const
{
    if (!_maSolver)
        const_cast<Self*>(this)->_maSolver.reset(new MomentArmSolver(*_model));

    return _maSolver->solve(s, aCoord,  *this);
}

//_____________________________________________________________________________
/*
 * Update the cache entry for current_display_path
 */
void GeometryPath::updateDisplayPath(const SimTK::State& s) const
{
    Array<PathPoint*>& currentDisplayPath = 
        updCacheVariableValue<Array<PathPoint*> >(s, "current_display_path");
    // Clear the current display path. Delete all path points
    // that have a NULL path pointer. This means that they were
    // created by an earlier call to updateDisplayPath() and are
    // not part of the _currentPath.
/*
    for (int i=0; i<currentDisplayPath.getSize(); i++) {
        PathPoint* mp = currentDisplayPath.get(i);
        if (!mp->getPath())
            ;
    }
*/
    currentDisplayPath.setSize(0);

    const Array<PathPoint*>& currentPath =  
        getCacheVariableValue<Array<PathPoint*> >(s, "current_path");
    for (int i=0; i<currentPath.getSize(); i++) {
        PathPoint* mp = currentPath.get(i);
        PathWrapPoint* mwp = dynamic_cast<PathWrapPoint*>(mp);
        if (mwp) {
            // If the point is a PathWrapPoint and has surfacePoints,
            // then this is the second of two tangent points for the
            // wrap instance. So add the surface points to the display
            // path before adding the second tangent point.
            // Note: the first surface point is coincident with the
            // first tangent point, so don't add it to the path.
            const Array<Vec3>& surfacePoints = mwp->getWrapPath();
            for (int j=1; j<surfacePoints.getSize(); j++) {
                PathWrapPoint* p = new PathWrapPoint();
                p->setLocation(s, surfacePoints.get(j));
                p->setBody(mwp->getBody());
                currentDisplayPath.append(p);
            }
        }
        currentDisplayPath.append(mp);
    }

    markCacheVariableValid(s, "current_display_path");
}
