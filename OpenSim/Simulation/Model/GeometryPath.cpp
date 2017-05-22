/* -------------------------------------------------------------------------- *
 *                         OpenSim:  GeometryPath.cpp                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
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
#include "ConditionalPathPoint.h"
#include "MovingPathPoint.h"
#include "PointForceDirection.h"
#include <OpenSim/Simulation/Wrap/PathWrap.h>
#include "Model.h"

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

    // Name the path points based on the current path
    // (i.e., the set of currently active points is numbered
    // 1, 2, 3, ...).
    namePathPoints(0);
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
    Array<AbstractPathPoint *> pathPrototype;
    addCacheVariable<Array<AbstractPathPoint *> >
        ("current_path", pathPrototype, SimTK::Stage::Position);

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
    getModel().realizeDynamics(state);

    const Array<AbstractPathPoint*>& pathPoints = getCurrentPath(state);

    const AbstractPathPoint* lastPoint = pathPoints[0];
    MobilizedBodyIndex mbix(0);

    Vec3 lastPos = lastPoint->getLocationInGround(state);
    if (hints.get_show_path_points())
        DefaultGeometry::drawPathPoint(mbix, lastPos, getColor(state), appendToThis);

    Vec3 pos;

    for (int i = 1; i < pathPoints.getSize(); ++i) {
        AbstractPathPoint* point = pathPoints[i];
        PathWrapPoint* pwp = dynamic_cast<PathWrapPoint*>(point);

        if (pwp) {
            // A PathWrapPoint provides points on the wrapping surface as Vec3s
            Array<Vec3>& surfacePoints = pwp->getWrapPath();
            // The surface points are expressed w.r.t. the wrap surface's body frame.
            // Transform the surface points into the ground reference frame to draw
            // the surface point as the wrapping portion of the GeometryPath
            const Transform& X_BG = pwp->getParentFrame().getTransformInGround(state);
            // Cycle through each surface point and draw it the Ground frame
            for (int j = 0; j<surfacePoints.getSize(); ++j) {
                // transform the surface point into the Ground reference frame
                pos = X_BG*surfacePoints[j];
                if (hints.get_show_path_points())
                    DefaultGeometry::drawPathPoint(mbix, pos, getColor(state),
                        appendToThis);
                // Line segments will be in ground frame
                appendToThis.push_back(DecorativeLine(lastPos, pos)
                    .setLineThickness(4)
                    .setColor(getColor(state)).setBodyId(0).setIndexOnBody(j));
                lastPos = pos;
            }
        } 
        else { // otherwise a regular PathPoint so just draw its location
            pos = point->getLocationInGround(state);
            if (hints.get_show_path_points())
                DefaultGeometry::drawPathPoint(mbix, pos, getColor(state),
                    appendToThis);
            // Line segments will be in ground frame
            appendToThis.push_back(DecorativeLine(lastPos, pos)
                .setLineThickness(4)
                .setColor(getColor(state)).setBodyId(0).setIndexOnBody(i));
            lastPos = pos;
        }
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
    
    Vec3 defaultColor = SimTK::Gray;
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
        AbstractPathPoint& point = get_PathPointSet().get(i);
        if (point.getName()=="" && hasOwner()) {
            point.setName(getOwner().getName() + "-P" + indx);
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
const OpenSim::Array <AbstractPathPoint*> & GeometryPath::
getCurrentPath(const SimTK::State& s)  const
{
    computePath(s);   // compute checks if path needs to be recomputed
    return getCacheVariableValue< Array<AbstractPathPoint*> >(s, "current_path");
}

// get the path as PointForceDirections directions 
// CAUTION: the return points are heap allocated; you must delete them yourself! 
// (TODO: that is really lame)
void GeometryPath::
getPointForceDirections(const SimTK::State& s, 
                        OpenSim::Array<PointForceDirection*> *rPFDs) const
{
    int i;
    AbstractPathPoint* start;
    AbstractPathPoint* end;
    const OpenSim::PhysicalFrame* startBody;
    const OpenSim::PhysicalFrame* endBody;
    const Array<AbstractPathPoint*>& currentPath = getCurrentPath(s);

    int np = currentPath.getSize();
    rPFDs->ensureCapacity(np);
    
    for (i = 0; i < np; i++) {
        PointForceDirection *pfd = 
            new PointForceDirection(currentPath[i]->getLocation(s), 
                                    currentPath[i]->getParentFrame(), Vec3(0));
        rPFDs->append(pfd);
    }

    for (i = 0; i < np-1; i++) {
        start = currentPath[i];
        end = currentPath[i+1];
        startBody = &start->getParentFrame();
        endBody = &end->getParentFrame();

        if (startBody != endBody)
        {
            Vec3 posStart, posEnd;
            Vec3 direction(0);

            // Find the positions of start and end in the inertial frame.
            //engine.getPosition(s, start->getParentFrame(), start->getLocation(), posStart);
            posStart = start->getLocationInGround(s);
            
            //engine.getPosition(s, end->getParentFrame(), end->getLocation(), posEnd);
            posEnd = end->getLocationInGround(s);

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
    AbstractPathPoint* start = NULL;
    AbstractPathPoint* end = NULL;
    const SimTK::MobilizedBody* bo = NULL;
    const SimTK::MobilizedBody* bf = NULL;
    const Array<AbstractPathPoint*>& currentPath = getCurrentPath(s);
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

        bo = &start->getParentFrame().getMobilizedBody();
        bf = &end->getParentFrame().getMobilizedBody();

        if (bo != bf) {
            // Find the positions of start and end in the inertial frame.
            po = start->getLocationInGround(s);
            pf = end->getLocationInGround(s);

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

            const MovingPathPoint* mppo =
                dynamic_cast<MovingPathPoint *>(start);

            // do the same for the end point of this segment of the path
            const MovingPathPoint* mppf =
                dynamic_cast<MovingPathPoint *>(end);

            // add in the tension point forces to body forces
            if (mppo) {// moving path point location is a function of the state
                // transform of the frame of the point to the base mobilized body
                auto X_BF = mppo->getParentFrame().findTransformInBaseFrame();
                bo->applyForceToBodyPoint(s, X_BF*mppo->getLocation(s), force,
                    bodyForces);
            }
            else {
                // transform of the frame of the point to the base mobilized body
                auto X_BF = start->getParentFrame().findTransformInBaseFrame();
                bo->applyForceToBodyPoint(s, X_BF*start->getLocation(s), force,
                    bodyForces);
            }

            if (mppf) {// moving path point location is a function of the state
                // transform of the frame of the point to the base mobilized body
                auto X_BF = mppf->getParentFrame().findTransformInBaseFrame();
                bf->applyForceToBodyPoint(s, X_BF*mppf->getLocation(s), -force,
                    bodyForces);
            }
            else {
                // transform of the frame of the point to the base mobilized body
                auto X_BF = end->getParentFrame().findTransformInBaseFrame();
                bf->applyForceToBodyPoint(s, X_BF*end->getLocation(s), -force,
                    bodyForces);
            }

            // Now account for the work being done by virtue of the moving
            // path point motion relative to the body it is on
            if(mppo){
                // torque (genforce) contribution due to relative movement 
                // of a via point w.r.t. the body it is connected to.
                dPodq_G = bo->expressVectorInGroundFrame(s, start->getdPointdQ(s));
                fo = ~dPodq_G*force;            

                // get the mobilized body the coordinate is couple to.
                const SimTK::MobilizedBody& mpbod =
                    matter.getMobilizedBody(mppo->getXCoordinate().getBodyIndex());

                // apply the generalized (mobility) force to the coordinate's body
                mpbod.applyOneMobilityForce(s, 
                    mppo->getXCoordinate().getMobilizerQIndex(), 
                    fo, mobilityForces);
            }

            if(mppf){
                dPfdq_G = bf->expressVectorInGroundFrame(s, end->getdPointdQ(s));
                ff = ~dPfdq_G*(-force);

                // get the mobilized body the coordinate is couple to.
                const SimTK::MobilizedBody& mpbod =
                    matter.getMobilizedBody(mppf->getXCoordinate().getBodyIndex());

                mpbod.applyOneMobilityForce(s, 
                    mppf->getXCoordinate().getMobilizerQIndex(), 
                    ff, mobilityForces);
            }
        }       
    }
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
 * @param frame The frame to attach the point to.
 * @return Pointer to the newly created path point.
 */
AbstractPathPoint* GeometryPath::
addPathPoint(const SimTK::State& s, int aIndex, PhysicalFrame& frame)
{
    PathPoint* newPoint = new PathPoint();
    newPoint->setParentFrame(frame);
    Vec3 location = newPoint->getLocation(s);
    placeNewPathPoint(s, location, aIndex, frame);
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

AbstractPathPoint* GeometryPath::
appendNewPathPoint(const std::string& proposedName, 
                   PhysicalFrame& frame, const SimTK::Vec3& aPositionOnBody)
{
    PathPoint* newPoint = new PathPoint();
    newPoint->setParentFrame(frame);
    newPoint->setName(proposedName);
    newPoint->setLocation(aPositionOnBody);
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
 * @param frame The body to attach the point to.
 */
void GeometryPath::
placeNewPathPoint(const SimTK::State& s, SimTK::Vec3& aOffset, int aIndex, 
                  const PhysicalFrame& frame)
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

        const Vec3 startPt = get_PathPointSet()[start].getLocation(s);
        const Vec3 endPt = get_PathPointSet()[end].getLocation(s);
        const Vec3 basePt = get_PathPointSet()[base].getLocation(s);

        Vec3 startPt2 = get_PathPointSet()[start].getParentFrame()
            .findStationLocationInAnotherFrame(s, startPt, frame);

        Vec3 endPt2 = get_PathPointSet()[end].getParentFrame()
            .findStationLocationInAnotherFrame(s, endPt, frame);

        aOffset = basePt + distance * (endPt2 - startPt2);
    } else if (get_PathPointSet().getSize() == 1) {
        aOffset= get_PathPointSet()[0].getLocation(s) + 0.01;
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
replacePathPoint(const SimTK::State& s, AbstractPathPoint* aOldPathPoint, 
                 AbstractPathPoint* aNewPathPoint) 
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
    upd_PathWrapSet().adoptAndAppend(newWrap);
    finalizeFromProperties();
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
        const string& bodyName = 
            get_PathPointSet()[i].getParentFrame().getName();
        for (int j = 0; j < aScaleSet.getSize(); j++)
        {
            Scale& aScale = aScaleSet.get(j);
            if (bodyName == aScale.getSegmentName())
            {
                Vec3 scaleFactors(1.0);
                aScale.getScaleFactors(scaleFactors);
                upd_PathPointSet().get(i).scale(scaleFactors);
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
    //const SimTK::Stage& sg = s.getSystemStage();
    
    if (isCacheVariableValid(s, "current_path"))  {
        return;
    }

    // Clear the current path.
    Array<AbstractPathPoint*>& currentPath = 
        updCacheVariableValue<Array<AbstractPathPoint*> >(s, "current_path");
    currentPath.setSize(0);

    // Add the active fixed and moving via points to the path.
    for (int i = 0; i < get_PathPointSet().getSize(); i++) {
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

    const Array<AbstractPathPoint*>& currentPath = getCurrentPath(s);

    double speed = 0.0;
    
    for (int i = 0; i < currentPath.getSize() - 1; i++) {
        speed += currentPath[i]->calcSpeedBetween(s, *currentPath[i+1]);
    }

    setLengtheningSpeed(s, speed);
}

//_____________________________________________________________________________
/*
 * Apply the wrap objects to the current path.
 */
void GeometryPath::
applyWrapObjects(const SimTK::State& s, Array<AbstractPathPoint*>& path) const 
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
                if( path.get(j) == &ws.getWrapPoint1()) {
                    path.remove(j); // remove the first wrap point
                    path.remove(j); // remove the second wrap point
                    break;
                }
            }

            if (wo->get_active()) {
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
                const AbstractPathPoint* const smp = &get_PathPointSet().get(jfwd);

                // 3. Scan backwards from wrapEnd in get_PathPointSet() to find 
                // the last point that is active. Store a pointer to it (emp).
                int jrev = wrapEnd;
                for (; jrev >= wrapStart; jrev--)
                    if (get_PathPointSet().get(jrev).isActive(s))
                        break;
                if (jrev < wrapStart) // there are no active points in the path
                    return;
                const AbstractPathPoint* const emp = &get_PathPointSet().get(jrev);

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
                ws.updWrapPoint2().getWrapPath().setSize(0);

                if (best_wrap.wrap_pts.getSize() == 0) {
                    ws.resetPreviousWrap();
                    ws.updWrapPoint2().getWrapPath().setSize(0);
                } else {
                    // If wrapping did occur, copy wrap info into the PathStruct.
                    ws.updWrapPoint1().getWrapPath().setSize(0);

                    Array<SimTK::Vec3>& wrapPath = ws.updWrapPoint2().getWrapPath();
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

                    ws.updWrapPoint1().setWrapLength(0.0);
                    ws.updWrapPoint2().setWrapLength(best_wrap.wrap_path_length);

                    ws.updWrapPoint1().setLocation(best_wrap.r1);
                    ws.updWrapPoint2().setLocation(best_wrap.r2);

                    // Now insert the two new wrapping points into mp[] array.
                    path.insert(best_wrap.endPoint, &ws.updWrapPoint1());
                    path.insert(best_wrap.endPoint + 1, &ws.updWrapPoint2());
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
                    if (path.get(j) == &ws.updWrapPoint1()) {
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
                     const WrapResult& wr, const Array<AbstractPathPoint*>& path)  const
{
    const AbstractPathPoint* pt1 = path.get(wr.startPoint);
    const AbstractPathPoint* pt2 = path.get(wr.endPoint);

    double straight_length = pt1->calcDistanceBetween(s, *pt2);

    double wrap_length = pt1->calcDistanceBetween(s, wo.getFrame(), wr.r1);
    wrap_length += wr.wrap_path_length;
    wrap_length += pt2->calcDistanceBetween(s, wo.getFrame(), wr.r2);

    return wrap_length - straight_length; // return absolute diff, not relative
}

//_____________________________________________________________________________
/*
 * Compute the total length of the path. This function
 * assumes that the path has already been updated.
 */
double GeometryPath::
calcLengthAfterPathComputation(const SimTK::State& s, 
                               const Array<AbstractPathPoint*>& currentPath) const
{
    double length = 0.0;

    for (int i = 0; i < currentPath.getSize() - 1; i++) {
        const AbstractPathPoint* p1 = currentPath[i];
        const AbstractPathPoint* p2 = currentPath[i+1];

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
            length += p1->calcDistanceBetween(s, *p2);
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

void GeometryPath::extendFinalizeFromProperties()
{
    Super::extendFinalizeFromProperties();
    for (int i = 0; i < get_PathWrapSet().getSize(); ++i) {
        if (upd_PathWrapSet()[i].getName().empty()) {
            std::stringstream label;
            label << "pathwrap_" << i;
            upd_PathWrapSet()[i].setName(label.str());
        }
    }
}
