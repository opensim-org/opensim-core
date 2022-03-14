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


#include "GeometryPath.h"

#include <OpenSim/Simulation/Model/ConditionalPathPoint.h>
#include <OpenSim/Simulation/Model/MovingPathPoint.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/PointForceDirection.h>
#include <OpenSim/Simulation/Wrap/PathWrap.h>
#include <OpenSim/Simulation/Wrap/PathWrapPoint.h>

// normalize a vector, or fill it with NaNs if the resulting direction vector
// would have a close-to-zero length
static void NormalizeIfNotCoindicentOrNaN(SimTK::Vec3& v)
{
    double len = v.norm();

    if (len < SimTK::SignificantReal) {
        v = SimTK::Vec3{SimTK::NaN, SimTK::NaN, SimTK::NaN};
    }
    else {
        v /= len;  // normalize
    }
}

// emplace an element onto the end of a SimTK::Array
template<typename T, typename... Args, typename U = T>
static T& EmplaceBack(SimTK::Array_<U>& out, Args&&... args)
{
    out.emplace_back(T{std::forward<Args>(args)...});
    return static_cast<T&>(out.back());
}

// linearly search an OpenSim container (e.g. PathWrapSet) for an element at the given address
template<typename OpenSimContainer, typename T>
static int IndexOfOrMinusOne(const OpenSimContainer& els, const T* el) {
    for (int i = 0; i < els.getSize(); ++i) {
        if (&els[i] == el) {
            return i;
        }
    }
    return -1;
}

// perform a bounds-checked index lookup into a container
template<typename T>
static const T& ElementAt(const OpenSim::Set<T>& els, int idx) {
    if (0 <= idx && idx < els.getSize()) {
        return els[idx];
    }
    else {
        OPENSIM_THROW(OpenSim::Exception, "out of bounds access detected: this is a developer error");
    }
}

OpenSim::GeometryPath::GeometryPath() : ModelComponent(), _preScaleLength(0.0)
{
    setAuthors("Peter Loan");
    constructProperties();
}
OpenSim::GeometryPath::GeometryPath(const GeometryPath&) = default;
OpenSim::GeometryPath::GeometryPath(GeometryPath&&) noexcept = default;
OpenSim::GeometryPath& OpenSim::GeometryPath::operator=(const GeometryPath&) = default;
OpenSim::GeometryPath& OpenSim::GeometryPath::operator=(GeometryPath&&) noexcept = default;
OpenSim::GeometryPath::~GeometryPath() noexcept = default;

const OpenSim::PathPointSet& OpenSim::GeometryPath::getPathPointSet() const
{
    return get_PathPointSet();
}

OpenSim::PathPointSet& OpenSim::GeometryPath::updPathPointSet()
{
    return upd_PathPointSet();
}

OpenSim::AbstractPathPoint* OpenSim::GeometryPath::addPathPoint(
    const SimTK::State& s,
    int aIndex,
    const PhysicalFrame& frame)
{
    PathPoint* newPoint = new PathPoint();
    newPoint->setParentFrame(frame);
    SimTK::Vec3 newLocation(0.0);
    // Note: placeNewPathPoint() returns a location by reference.
    // It computes a new location according to the index where the new path point
    // will be inserted along the path(among the other path points).
    placeNewPathPoint(s, newLocation, aIndex, frame);
    // Now set computed new location into the newPoint
    newPoint->setLocation(newLocation);
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

        if (startPoint != -1 && aIndex <= startPoint) {
            get_PathWrapSet().get(i).setStartPoint(s,startPoint + 1);
        }

        if (endPoint != -1 && aIndex <= endPoint) {
            get_PathWrapSet().get(i).setEndPoint(s,endPoint + 1);
        }
    }

    return newPoint;
}

OpenSim::AbstractPathPoint* OpenSim::GeometryPath::appendNewPathPoint(
    const std::string& proposedName,
    const PhysicalFrame& frame,
    const SimTK::Vec3& aPositionOnBody)
{
    PathPoint* newPoint = new PathPoint();
    newPoint->setParentFrame(frame);
    newPoint->setName(proposedName);
    newPoint->setLocation(aPositionOnBody);
    upd_PathPointSet().adoptAndAppend(newPoint);

    return newPoint;
}

bool OpenSim::GeometryPath::canDeletePathPoint(int aIndex)
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

    return numOtherFixedPoints >= 2;
}

bool OpenSim::GeometryPath::deletePathPoint(const SimTK::State& s, int aIndex)
{
    if (canDeletePathPoint(aIndex) == false) {
        return false;
    }

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
            || (startPoint > get_PathPointSet().getSize())) {

            get_PathWrapSet().get(i).setStartPoint(s, startPoint - 1);
        }

        if (   endPoint > 1
            && aIndex <= endPoint
            && (   (endPoint > startPoint)
                || (endPoint > get_PathPointSet().getSize()))) {

            get_PathWrapSet().get(i).setEndPoint(s, endPoint - 1);
        }
    }

    return true;
}

bool OpenSim::GeometryPath::replacePathPoint(
    const SimTK::State& s,
    AbstractPathPoint* aOldPathPoint,
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
            return true;
        }
    }
    return false;
}

const OpenSim::PathWrapSet& OpenSim::GeometryPath::getWrapSet() const
{
    return get_PathWrapSet();
}

OpenSim::PathWrapSet& OpenSim::GeometryPath::updWrapSet()
{
    return upd_PathWrapSet();
}

void OpenSim::GeometryPath::addPathWrap(WrapObject& aWrapObject)
{
    PathWrap* newWrap = new PathWrap();
    newWrap->setWrapObject(aWrapObject);
    newWrap->setMethod(PathWrap::hybrid);
    upd_PathWrapSet().adoptAndAppend(newWrap);
    finalizeFromProperties();
}

void OpenSim::GeometryPath::moveUpPathWrap(const SimTK::State& s, int aIndex)
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

void OpenSim::GeometryPath::moveDownPathWrap(const SimTK::State& s, int aIndex)
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

void OpenSim::GeometryPath::deletePathWrap(const SimTK::State& s, int aIndex)
{
    upd_PathWrapSet().remove(aIndex);

}

const SimTK::Vec3& OpenSim::GeometryPath::getDefaultColor() const
{
    return get_Appearance().get_color();
}

void OpenSim::GeometryPath::setDefaultColor(const SimTK::Vec3& color)
{
    updProperty_Appearance().setValueIsDefault(false);
    upd_Appearance().set_color(color);
}

SimTK::Vec3 OpenSim::GeometryPath::getColor(const SimTK::State& s) const
{
    return getCacheVariableValue(s, _colorCV);
}

void OpenSim::GeometryPath::setColor(const SimTK::State& s, const SimTK::Vec3& color) const
{
    setCacheVariableValue(s, _colorCV, color);
}


double OpenSim::GeometryPath::getLength(const SimTK::State& s) const
{
    if (isCacheVariableValid(s, _lengthCV)) {
        return getCacheVariableValue(s, _lengthCV);
    }

    double& v = updCacheVariableValue(s, _lengthCV);
    v = calcPathLength(s, getCurrentPath(s));
    setCacheVariableValue(s, _lengthCV, v);

    return v;
}

void OpenSim::GeometryPath::setLength(const SimTK::State& s, double length) const
{
    setCacheVariableValue(s, _lengthCV, length);
}

double OpenSim::GeometryPath::getPreScaleLength( const SimTK::State& s) const
{
    return _preScaleLength;
}

void OpenSim::GeometryPath::setPreScaleLength( const SimTK::State& s, double length )
{
    _preScaleLength = length;
}

double OpenSim::GeometryPath::getLengtheningSpeed(const SimTK::State& s) const
{
    if (isCacheVariableValid(s, _speedCV)) {
        return getCacheVariableValue(s, _speedCV);
    }

    double& v = updCacheVariableValue(s, _speedCV);
    v = calcLengtheningSpeed(s, getCurrentPath(s));
    markCacheVariableValid(s, _speedCV);
    return v;
}

void OpenSim::GeometryPath::setLengtheningSpeed( const SimTK::State& s, double speed ) const
{
    setCacheVariableValue(s, _speedCV, speed);
}

const OpenSim::Array<const OpenSim::AbstractPathPoint*>& OpenSim::GeometryPath::getCurrentPath(
    const SimTK::State& s) const
{

    if (isCacheVariableValid(s, _currentPathElsCV)) {
        // even though the cache variable is valid, re-populate the pointers cache
        //
        // this is because although it may be valid *in the state* the
        // model-level pointers cache may be dirty with lookups from other
        // states
        const auto& pathEls = getCacheVariableValue(s, _currentPathElsCV);
        _currentPathPtrsCache.setSize(static_cast<int>(pathEls.size()));
        for (int i = 0; i < static_cast<int>(pathEls.size()); ++i) {
            const PathElement& e = pathEls[i];

            const AbstractPathPoint* ptr = nullptr;
            switch (e.type) {
            case PathElement::Type::InPathPointSet:
                ptr = &ElementAt(get_PathPointSet(), e.index);
                break;
            case PathElement::Type::PathWrapPt1:
                ptr = &ElementAt(get_PathWrapSet(), e.index).getWrapPoint1();
                break;
            case PathElement::Type::PathWrapPt2:
                ptr = &ElementAt(get_PathWrapSet(), e.index).getWrapPoint2();
                break;
            default:
                OPENSIM_THROW(OpenSim::Exception, "unhandled PathElement::Type case: this is a developer error");
            }

            _currentPathPtrsCache[i] = ptr;
        }
        return _currentPathPtrsCache;
    }

    // clear the local pointers cache and populate it with an initial (pre-wrapping)
    // guess of the path points
    _currentPathPtrsCache.setSize(0);
    const PathPointSet& pps = get_PathPointSet();
    for (int i = 0; i < pps.getSize(); i++) {

        const AbstractPathPoint& p = pps[i];

        if (p.isActive(s)) {
            _currentPathPtrsCache.append(&p);
        }
    }

    // pump the pre-wrapping guess through the wrapping algorithm. This may mutate
    // the pointers array to contain extra (wrapping) points
    applyWrapObjects(s, _currentPathPtrsCache);

    // the pointers array now contains the "correct" (wrapped) path
    //
    // because the path is state-dependent (different states may wrap differently), the
    // path points (abstract) must be stored in a SimTK cache variable.
    //
    // However, we can't store raw pointers in a cache variable because that may cause
    // aliasing issues in downstream code. E.g. the state may later be used with a
    // copied/moved-from version of the model, rather than the original one, so the
    // pointers may be stale.
    std::vector<PathElement>& pathEls = updCacheVariableValue(s, _currentPathElsCV);
    pathEls.clear();
    pathEls.reserve(_currentPathPtrsCache.getSize());
    for (int i = 0; i < _currentPathPtrsCache.getSize(); ++i) {
        const AbstractPathPoint* p = _currentPathPtrsCache[i];
        const Component* owner = &p->getOwner();

        if (owner == this)
        {
            const PathPointSet& pps = get_PathPointSet();
            int loc = IndexOfOrMinusOne(pps, p);
            if (loc == -1) {
                OPENSIM_THROW(OpenSim::Exception, "cannot find path point element: this is a developer error");
            }
            pathEls.push_back(PathElement{loc, PathElement::Type::InPathPointSet});
        }
        else
        {
            const PathWrapSet& pws = get_PathWrapSet();
            int loc = IndexOfOrMinusOne(pws, owner);
            if (loc == -1) {
                OPENSIM_THROW(OpenSim::Exception, "cannot find path wrap element: this is a developer error");
            }

            if (&pws[loc].getWrapPoint1() == p) {
                pathEls.push_back(PathElement{loc, PathElement::Type::PathWrapPt1});
            }
            else {
                pathEls.push_back(PathElement{loc, PathElement::Type::PathWrapPt2});
            }
        }
    }
    markCacheVariableValid(s, _currentPathElsCV);

    return _currentPathPtrsCache;
}

// get the path as PointForceDirections directions
// CAUTION: the return points are heap allocated; you must delete them yourself!
// (TODO: that is really lame)
void OpenSim::GeometryPath::getPointForceDirections(
    const SimTK::State& s,
    OpenSim::Array<PointForceDirection*> *rPFDs) const
{
    using SimTK::Vec3;

    const Array<const AbstractPathPoint*>& currentPath = getCurrentPath(s);
    int np = currentPath.getSize();

    if (np <= 0) {
        return;  // no points to add
    }

    rPFDs->ensureCapacity(np);

    Vec3 nextDirection(0);
    for (int i = 0; i < np-1; i++) {
        const AbstractPathPoint* pp1 = currentPath[i];
        const AbstractPathPoint* pp2 = currentPath[i+1];

        Vec3 direction;

        if (&pp1->getParentFrame() == &pp2->getParentFrame())
        {
            direction = nextDirection;
            nextDirection = SimTK::Vec3(0);
        }
        else
        {
            // Form a vector from start to end, in the inertial frame.
            Vec3 v = pp2->getLocationInGround(s) - pp1->getLocationInGround(s);

            // Check that the two points are not coincident.
            // This can happen due to infeasible wrapping of the path,
            // when the origin or insertion enters the wrapping surface.
            // This is a temporary fix, since the wrap algorithm should
            // return NaN for the points and/or throw an Exception- aseth
            NormalizeIfNotCoindicentOrNaN(v);

            direction = nextDirection + v;
            nextDirection = -direction;  // negate next point
        }

        // add the PFD
        rPFDs->append(new PointForceDirection(
            pp1->getLocation(s),
            pp1->getParentFrame(),
            direction));
    }

    // add last element
    assert(np > 0);
    const AbstractPathPoint* lastPoint = currentPath[np-1];
    rPFDs->append(new PointForceDirection(
        lastPoint->getLocation(s),
        lastPoint->getParentFrame(),
        nextDirection));
}

void OpenSim::GeometryPath::addInEquivalentForces(
    const SimTK::State& s,
    const double& tension,
    SimTK::Vector_<SimTK::SpatialVec>& bodyForces,
    SimTK::Vector& mobilityForces) const
{
    using SimTK::Vec3;

    constexpr bool useOld = false;

    if (useOld) {

    const AbstractPathPoint* start = NULL;
    const AbstractPathPoint* end = NULL;
    const SimTK::MobilizedBody* bo = NULL;
    const SimTK::MobilizedBody* bf = NULL;
    const Array<const AbstractPathPoint*>& currentPath = getCurrentPath(s);
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
                dynamic_cast<const MovingPathPoint *>(start);

            // do the same for the end point of this segment of the path
            const MovingPathPoint* mppf =
                dynamic_cast<const MovingPathPoint *>(end);

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

    else {

    const Array<const AbstractPathPoint*>& currentPath = getCurrentPath(s);
    int np = currentPath.getSize();

    const SimTK::SimbodyMatterSubsystem& matter = getModel().getMatterSubsystem();

    for (int i = 0; i < np-1; ++i) {
        const AbstractPathPoint& start = *currentPath[i];
        const PhysicalFrame& startParentFrame = start.getParentFrame();
        const SimTK::MobilizedBody& startMb = startParentFrame.getMobilizedBody();

        const AbstractPathPoint& end = *currentPath[i+1];
        const PhysicalFrame& endParentFrame = end.getParentFrame();
        const SimTK::MobilizedBody& endMb = endParentFrame.getMobilizedBody();

        if (&startMb == &endMb) {
            continue;
        }

        // Find the positions of start and end in the inertial frame.
        Vec3 po = start.getLocationInGround(s);
        Vec3 pf = end.getLocationInGround(s);

        // Form a vector from start to end, in the inertial frame.
        //
        // Check that the two points are not coincident.
        // This can happen due to infeasible wrapping of the path,
        // when the origin or insertion enters the wrapping surface.
        // This is a temporary fix, since the wrap algorithm should
        // return NaN for the points and/or throw an Exception- aseth
        Vec3 dir = (pf - po);
        NormalizeIfNotCoindicentOrNaN(dir);

        Vec3 force = tension*dir;

        // add in the tension point forces to body forces
        startMb.applyForceToBodyPoint(
            s,
            startParentFrame.findTransformInBaseFrame() * start.getLocation(s),
            force,
            bodyForces);
        endMb.applyForceToBodyPoint(
            s,
            endParentFrame.findTransformInBaseFrame() * end.getLocation(s),
            -force,
            bodyForces);

        // Now account for the work being done by virtue of the moving
        // path point motion relative to the body it is on

        if (const MovingPathPoint* startAsMpp = dynamic_cast<const MovingPathPoint*>(&start)) {

            // torque (genforce) contribution due to relative movement
            // of a via point w.r.t. the body it is connected to.
            Vec3 dPodq_G = startMb.expressVectorInGroundFrame(s, start.getdPointdQ(s));
            double fo = ~dPodq_G*force;

            // get the mobilized body the coordinate is couple to.
            const Coordinate& xCoord = startAsMpp->getXCoordinate();
            const SimTK::MobilizedBody& mpbod = matter.getMobilizedBody(xCoord.getBodyIndex());

            // apply the generalized (mobility) force to the coordinate's body
            mpbod.applyOneMobilityForce(
                s,
                xCoord.getMobilizerQIndex(),
                fo,
                mobilityForces);
        }

        if (const MovingPathPoint* mppf = dynamic_cast<const MovingPathPoint*>(&end)) {
            Vec3 dPfdq_G = endMb.expressVectorInGroundFrame(s, end.getdPointdQ(s));
            double ff = ~dPfdq_G*(-force);

            // get the mobilized body the coordinate is couple to.
            const Coordinate& xCoord = mppf->getXCoordinate();
            const SimTK::MobilizedBody& mpbod = matter.getMobilizedBody(xCoord.getBodyIndex());

            mpbod.applyOneMobilityForce(
                s,
                xCoord.getMobilizerQIndex(),
                ff,
                mobilityForces);
        }
    }
    }
}

double OpenSim::GeometryPath::computeMomentArm(
    const SimTK::State& s,
    const Coordinate& aCoord) const
{
    if (!_maSolver) {
        const_cast<Self*>(this)->_maSolver.reset(new MomentArmSolver(*_model));
    }

    return _maSolver->solve(s, aCoord,  *this);
}

void OpenSim::GeometryPath::updateGeometry(const SimTK::State& s) const
{
    getCurrentPath(s); // check if the current path needs to recomputed
}

void OpenSim::GeometryPath::extendPreScale(
    const SimTK::State& s,
    const ScaleSet& scaleSet)
{
    Super::extendPreScale(s, scaleSet);
    setPreScaleLength(s, getLength(s));
}

void OpenSim::GeometryPath::extendPostScale(
    const SimTK::State& s,
    const ScaleSet& scaleSet)
{
    Super::extendPostScale(s, scaleSet);
    getCurrentPath(s);
}

void OpenSim::GeometryPath::extendFinalizeFromProperties()
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

void OpenSim::GeometryPath::extendConnectToModel(Model& aModel)
{
    Super::extendConnectToModel(aModel);

    OPENSIM_THROW_IF_FRMOBJ(get_PathPointSet().getSize() < 2,
        InvalidPropertyValue,
        getProperty_PathPointSet().getName(),
        "A valid path must be connected to a model by at least two PathPoints.")

    // Name the path points based on the current path
    // (i.e., the set of currently active points is numbered
    // 1, 2, 3, ...).
    namePathPoints(0);
}

void OpenSim::GeometryPath::extendInitStateFromProperties(SimTK::State& s) const
{
   Super::extendInitStateFromProperties(s);
   markCacheVariableValid(s, _colorCV);  // it is OK at its default value
}

void OpenSim::GeometryPath::extendAddToSystem(SimTK::MultibodySystem& system) const
{
    Super::extendAddToSystem(system);

    // allocate cache variables
    //
    // - length only depends on `q`s, so it will be valid after Stage::Position
    // - speed depends on `q`s and `u`s, so it will be valid after Stage::Velocity
    // - path point positions only depend on the position of points
    // - color is a decoration cache variable that isn't simulation-related

    this->_lengthCV = addCacheVariable("length", 0.0, SimTK::Stage::Position);
    this->_speedCV = addCacheVariable("speed", 0.0, SimTK::Stage::Velocity);
    this->_currentPathElsCV = addCacheVariable("current_path", std::vector<PathElement>{}, SimTK::Stage::Position);
    this->_colorCV = addCacheVariable("color", get_Appearance().get_color(), SimTK::Stage::Topology);
}

void OpenSim::GeometryPath::generateDecorations(
    bool fixed,
    const ModelDisplayHints& hints,
    const SimTK::State& state,
    SimTK::Array_<SimTK::DecorativeGeometry>& appendToThis) const
{
    using SimTK::Vec3;

    // The GeometryPath takes care of drawing itself here, using information it
    // can extract from the supplied state, including position information and
    // color information that may have been calculated as late as Stage::Dynamics.
    //
    // For example, muscles may want the color to reflect activation level and
    // other path-using components might want to use forces (tension). We will
    // ensure that the state has been realized to Stage::Dynamics before looking
    // at it. (It is only guaranteed to be at Stage::Position here.)

    // There is no fixed geometry to generate here.
    if (fixed) {
        return;
    }

    SimTK::MobilizedBodyIndex mbix{0};
    SimTK::Vec3 color = getColor(state);

    // helper method for emitting a point to the output
    auto emitPoint = [mbix, color, &appendToThis](SimTK::Vec3 pos)
    {
        SimTK::DefaultGeometry::drawPathPoint(
            mbix,
            pos,
            color,
            appendToThis);
    };

    // helper method for emitting a line to the output
    auto emitLine = [mbix, color, &appendToThis](SimTK::Vec3 a, SimTK::Vec3 b, int idx)
    {
        EmplaceBack<SimTK::DecorativeLine>(appendToThis, a, b)
            .setLineThickness(4)
            .setColor(color)
            .setBodyId(0)
            .setIndexOnBody(idx);
    };

    const Array<const AbstractPathPoint*>& pathPoints = getCurrentPath(state);
    assert(pathPoints.size() > 1);

    Vec3 prevPos = pathPoints[0]->getLocationInGround(state);

    if (hints.get_show_path_points()) {
        emitPoint(prevPos);
    }

    for (int i = 1; i < pathPoints.getSize(); ++i) {

        const AbstractPathPoint* point = pathPoints[i];

        if (const PathWrapPoint* pwp = dynamic_cast<const PathWrapPoint*>(point)) {

            // A PathWrapPoint provides points on the wrapping surface as Vec3s
            const Array<Vec3>& surfacePoints = pwp->getWrapPath(state);

            // The surface points are expressed w.r.t. the wrap surface's body frame.
            // Transform the surface points into the ground reference frame to draw
            // the surface point as the wrapping portion of the GeometryPath
            const SimTK::Transform& X_BG = pwp->getParentFrame().getTransformInGround(state);

            // Cycle through each surface point and draw it the Ground frame
            for (int j = 0; j<surfacePoints.getSize(); ++j) {

                // transform the surface point into the Ground reference frame
                Vec3 pos = X_BG*surfacePoints[j];

                if (hints.get_show_path_points()) {
                    emitPoint(pos);
                }

                emitLine(prevPos, pos, j);
                prevPos = pos;
            }
        }
        else {
            // otherwise, it's a regular PathPoint, so just draw its location

            Vec3 pos = point->getLocationInGround(state);

            if (hints.get_show_path_points()) {
                emitPoint(pos);
            }

            emitLine(prevPos, pos, i);
            prevPos = pos;
        }
    }
}

void OpenSim::GeometryPath::updateFromXMLNode(
    SimTK::Xml::Element& aNode,
    int versionNumber)
{
    if (versionNumber < XMLDocument::getLatestVersion()) {
        if (versionNumber < 30516) {
            // Create Appearance node under GeometryPath
            SimTK::Xml::Element appearanceElement("Appearance");
            aNode.appendNode(appearanceElement);
            SimTK::Xml::element_iterator visObjectIter = aNode.element_begin("VisibleObject");
            if (visObjectIter != aNode.element_end()) {
                SimTK::Xml::element_iterator oldPrefIter = visObjectIter->element_begin("display_preference");
                // old display_preference was used only for hide/show other options unsupported
                if (oldPrefIter != visObjectIter->element_end()) {
                    int oldPrefAsInt = 4;
                    oldPrefIter->getValueAs<int>(oldPrefAsInt);
                    if (oldPrefAsInt == 0) { // Hidden => Appearance/Visible
                        SimTK::Xml::Element visibleElement("visible");
                        visibleElement.setValueAs<bool>(false);
                        appearanceElement.insertNodeAfter(appearanceElement.element_end(), visibleElement);
                    }
                }
            }
            // If default_color existed, copy it to Appearance/color
            SimTK::Xml::element_iterator defaultColorIter = aNode.element_begin("default_color");
            if (defaultColorIter != aNode.element_end()) {
                // Move default_color to Appearance/color
                SimTK::Xml::Element colorElement("color");
                const SimTK::String& colorAsString = defaultColorIter->getValue();
                colorElement.setValue(colorAsString);
                appearanceElement.appendNode(colorElement);
            }
        }
    }
    // Call base class now assuming aNode has been corrected for current version
    Super::updateFromXMLNode(aNode, versionNumber);
}

void OpenSim::GeometryPath::constructProperties()
{
    constructProperty_PathPointSet(PathPointSet());

    constructProperty_PathWrapSet(PathWrapSet());
    
    Appearance appearance;
    appearance.set_color(SimTK::Gray);
    constructProperty_Appearance(appearance);
}

/*
 * Apply the wrap objects to the current path.
 */
void OpenSim::GeometryPath::applyWrapObjects(
    const SimTK::State& s,
    Array<const AbstractPathPoint*>& path) const
{
    int wrapSetSize = get_PathWrapSet().getSize();
    if (wrapSetSize < 1)
        return;

    WrapResult best_wrap;
    Array<int> result, order;

    result.setSize(wrapSetSize);
    order.setSize(wrapSetSize);

    // Set the initial order to be the order they are listed in the path.
    for (int i = 0; i < wrapSetSize; i++)
        order[i] = i;

    // If there is only one wrap object, calculate the wrapping only once.
    // If there are two or more objects, perform up to 8 iterations where
    // the result from one wrap object is used as the starting point for
    // the next wrap.
    const int maxIterations = wrapSetSize < 2 ? 1 : 8;
    double last_length = SimTK::Infinity;
    for (int kk = 0; kk < maxIterations; kk++)
    {
        for (int i = 0; i < wrapSetSize; i++)
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
                        wr.singleWrap = (wrapSetSize==1);
                        result[i] = wo->wrapPathSegment(s,
                                                        const_cast<AbstractPathPoint&>(*path.get(pt1)),
                                                        const_cast<AbstractPathPoint&>(*path.get(pt2)),
                                                        ws,
                                                        wr);
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
                ws.updWrapPoint2().clearWrapPath(s);

                if (best_wrap.wrap_pts.getSize() == 0) {
                    ws.resetPreviousWrap();
                    ws.updWrapPoint2().clearWrapPath(s);
                } else {
                    // If wrapping did occur, copy wrap info into the PathStruct.
                    ws.updWrapPoint1().clearWrapPath(s);

                    ws.updWrapPoint2().setWrapPath(s, best_wrap.wrap_pts);

                    // In OpenSim, all conversion to/from the wrap object's
                    // reference frame will be performed inside
                    // wrapPathSegment(). Thus, all points in this function will
                    // be in their respective body reference frames.
                    // for (j = 0; j < wrapPath.getSize(); j++){
                    //    convert_from_wrap_object_frame(wo, wrapPath.get(j));
                    //    convert(ms->modelnum, wrapPath.get(j), wo->segment,
                    //            ms->ground_segment);
                    // }

                    ws.updWrapPoint1().setWrapLength(s, 0.0);
                    ws.updWrapPoint2().setWrapLength(s, best_wrap.wrap_path_length);

                    ws.updWrapPoint1().setLocation(s, best_wrap.r1);
                    ws.updWrapPoint2().setLocation(s, best_wrap.r2);

                    // Now insert the two new wrapping points into mp[] array.
                    path.insert(best_wrap.endPoint, &ws.updWrapPoint1());
                    path.insert(best_wrap.endPoint + 1, &ws.updWrapPoint2());
                }
            }
        }

        const double length = calcPathLength(s, path);
        if (std::abs(length - last_length) < 0.0005) {
            break;
        } else {
            last_length = length;
        }

        if (kk == 0 && wrapSetSize > 1) {
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

/*
 * Compute lengthening speed of the path.
 */
double OpenSim::GeometryPath::calcLengtheningSpeed(
    const SimTK::State& s,
    const Array<const AbstractPathPoint*>& path) const
{
    double speed = 0.0;

    for (int i = 0; i < path.getSize() - 1; i++) {
        speed += path[i]->calcSpeedBetween(s, *path[i+1]);
    }

    return speed;
}

/*
 * _calc_path_length_change - given the output of a successful path wrap
 * over a wrap object, determine the percent change in length of the
 * path segment incurred by wrapping.
 */
double OpenSim::GeometryPath::calcPathLengthChange(
    const SimTK::State& s,
    const WrapObject& wo,
    const WrapResult& wr,
    const Array<const AbstractPathPoint*>& path)  const
{
    const AbstractPathPoint* pt1 = path.get(wr.startPoint);
    const AbstractPathPoint* pt2 = path.get(wr.endPoint);
    const PhysicalFrame& woFrame = wo.getFrame();

    double straightLength = pt1->calcDistanceBetween(s, *pt2);

    double wrapLength = pt1->calcDistanceBetween(s, woFrame, wr.r1);
    wrapLength += wr.wrap_path_length;
    wrapLength += pt2->calcDistanceBetween(s, woFrame, wr.r2);

    return wrapLength - straightLength; // return absolute diff, not relative
}

/*
 * Compute the total length of the path. This function
 * assumes that the path has already been updated.
 */
double OpenSim::GeometryPath::calcPathLength(
    const SimTK::State& s,
    const Array<const AbstractPathPoint*>& path) const
{
    if (path.getSize() == 0) {
        return 0.0;
    }

    double length = 0.0;
    const AbstractPathPoint* p1 = path[0];
    SimTK::Vec3 p1InGround = p1->getLocationInGround(s);

    // Transform all points to ground once rather than once per-segment
    for (int i = 1; i < path.getSize() ; i++) {

        const AbstractPathPoint* p2 = path[i];
        SimTK::Vec3 p2InGround = p2->getLocationInGround(s);

        // If both points are wrap points on the same wrap object, then this
        // path segment wraps over the surface of a wrap object, so just add in
        // the pre-calculated length.
        if (   p1->getWrapObject()
            && p2->getWrapObject()
            && p1->getWrapObject() == p2->getWrapObject())
        {
            const PathWrapPoint* smwp = dynamic_cast<const PathWrapPoint*>(p2);
            if (smwp) {
                length += smwp->getWrapLength(s);
            }
        } else {
            length += (p1InGround - p2InGround).norm();
        }

        p1 = p2;
        p1InGround = p2InGround;
    }

    return length;
}

/**
 * Name the path points based on their position in the set. To keep the names up to date,
 * this method should be called every time the path changes.
 *
 * @param aStartingIndex The index of the first path point to name.
 */
void OpenSim::GeometryPath::namePathPoints(int aStartingIndex)
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

/*
 * Determine an appropriate default XYZ location for a new path point.
 * Note that this method is internal and should not be called directly on a new 
 * point as the point is not really added to the path (done in addPathPoint() 
 * instead)
 * @param aOffset The XYZ location to be determined.
 * @param aIndex The position in the pathPointSet to put the new point in.
 * @param frame The body to attach the point to.
 */
void OpenSim::GeometryPath::placeNewPathPoint(
    const SimTK::State& s,
    SimTK::Vec3& aOffset,
    int aIndex,
    const PhysicalFrame& frame)
{
    using SimTK::Vec3;

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
    }
    else if (get_PathPointSet().getSize() == 1) {
        aOffset = get_PathPointSet()[0].getLocation(s) + 0.01;
    }
    else {
        // first point, do nothing?
    }
}
