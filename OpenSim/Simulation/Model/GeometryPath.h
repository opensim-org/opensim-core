#ifndef OPENSIM_GEOMETRY_PATH_H_
#define OPENSIM_GEOMETRY_PATH_H_
/* -------------------------------------------------------------------------- *
 *                          OpenSim:  GeometryPath.h                          *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Peter Loan                                                      *
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

#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Simulation/Model/ModelComponent.h>
#include <OpenSim/Simulation/Model/PathPointSet.h>
#include <OpenSim/Simulation/Wrap/PathWrapSet.h>
#include <OpenSim/Simulation/MomentArmSolver.h>

#ifdef SWIG
    #ifdef OSIMSIMULATION_API
        #undef OSIMSIMULATION_API
        #define OSIMSIMULATION_API
    #endif
#endif

namespace OpenSim {

class Coordinate;
class PointForceDirection;
class ScaleSet;
class WrapResult;
class WrapObject;

/**
 * A class that represents a sequence ("path") of points in 3D space.
 *
 * This path class can be used to represent path-like concepts such as
 * muscles and ligaments.
 *
 * @author Peter Loan
 */
class OSIMSIMULATION_API GeometryPath : public ModelComponent {
OpenSim_DECLARE_CONCRETE_OBJECT(GeometryPath, ModelComponent);

public:

    //=============================================================================
    // OUTPUTS
    //=============================================================================
    OpenSim_DECLARE_OUTPUT(
        length,
        double,
        getLength,
        SimTK::Stage::Position);

    OpenSim_DECLARE_OUTPUT(
        lengthening_speed,
        double,
        getLengtheningSpeed,
        SimTK::Stage::Velocity);

    //=============================================================================
    // PROPERTIES
    //=============================================================================

    OpenSim_DECLARE_UNNAMED_PROPERTY(
        Appearance,
        "Default appearance attributes for this GeometryPath");

private:
    OpenSim_DECLARE_UNNAMED_PROPERTY(
        PathPointSet,
        "The set of points defining the path");

    OpenSim_DECLARE_UNNAMED_PROPERTY(
        PathWrapSet,
        "The wrap objects that are associated with this path");

public:
    //=============================================================================
    // METHODS
    //=============================================================================

    GeometryPath();
    GeometryPath(const GeometryPath&);
    GeometryPath(GeometryPath&&) noexcept;
    GeometryPath& operator=(const GeometryPath&);
    GeometryPath& operator=(GeometryPath&&) noexcept;
    ~GeometryPath() noexcept;

    const PathPointSet& getPathPointSet() const;
    PathPointSet& updPathPointSet();

    /**
     * Add a new path point, with default location, to the path point set.
     *
     * @param aIndex The position in the pathPointSet to put the new point in.
     * @param frame The frame to attach the point to.
     * @return Pointer to the newly created path point.
     */
    AbstractPathPoint* addPathPoint(
        const SimTK::State&,
        int index,
        const PhysicalFrame&);

    AbstractPathPoint* appendNewPathPoint(
        const std::string& proposedName,
        const PhysicalFrame&,
        const SimTK::Vec3& locationOnFrame);

    /**
     * See if a path point can be deleted. All paths must have at least two
     * active path points to define the path.
     *
     * @param aIndex The index of the point to delete.
     * @return Whether or not the point can be deleted.
     */
    bool canDeletePathPoint(int index);

    /**
     * Delete a path point.
     *
     * @param aIndex The index of the point to delete.
     * @return Whether or not the point was deleted.
     */
    bool deletePathPoint(const SimTK::State&, int index);

    /**
     * Replace a path point in the set with another point. The new one is made a
     * member of all the same groups as the old one, and is inserted in the same
     * place the old one occupied.
     *
     *  @param aOldPathPoint Path point to remove.
     *  @param aNewPathPoint Path point to add.
     */
    bool replacePathPoint(
        const SimTK::State&,
        AbstractPathPoint* oldPathPoint,
        AbstractPathPoint* newPathPoint);

    const PathWrapSet& getWrapSet() const;
    PathWrapSet& updWrapSet();

    /**
     * Create a new wrap instance and add it to the set.
     *
     * @param aWrapObject The wrap object to use in the new wrap instance.
     */
    void addPathWrap(WrapObject&);

    /**
     * Move a wrap instance up in the list. Changing the order of wrap instances for
     * a path may affect how the path wraps over the wrap objects.
     *
     * @param aIndex The index of the wrap instance to move up.
     */
    void moveUpPathWrap(const SimTK::State&, int index);

    /**
     * Move a wrap instance down in the list. Changing the order of wrap instances
     * for a path may affect how the path wraps over the wrap objects.
     *
     * @param aIndex The index of the wrap instance to move down.
     */
    void moveDownPathWrap(const SimTK::State&, int index);

    /**
     * Delete a wrap instance.
     *
     * @param aIndex The index of the wrap instance to delete.
     */
    void deletePathWrap(const SimTK::State&, int index);

    /**
     * Returns the color that will be used to initialize the color cache at
     * the next extendAddToSystem() call. The actual color used to draw the
     * path will be taken from the cache variable, which may have changed.
     */
    const SimTK::Vec3& getDefaultColor() const;

    /**
     * If you call this prior to extendAddToSystem() it will be used to initialize
     * the color cache variable. Otherwise %GeometryPath will choose its own
     * default, which varies depending on owner.
     */
    void setDefaultColor(const SimTK::Vec3& color);

    /**
     * Get the current value of the color cache entry owned by this
     * %GeometryPath object in the given state. You can access this value any
     * time after the state is initialized, at which point it will have been
     * set to the default color value specified in a call to setDefaultColor()
     * earlier, or it will have the default color value chosen by %GeometryPath.
     *
     * @see setDefaultColor()
     */
    SimTK::Vec3 getColor(const SimTK::State&) const;

    /**
     * %Set the value of the color cache variable owned by this %GeometryPath
     * object, in the cache of the given state. The value of this variable is used
     * as the color when the path is drawn, which occurs with the state realized
     * to Stage::Dynamics. So you must call this method during realizeDynamics() or
     * earlier in order for it to have any effect.
     */
    void setColor(const SimTK::State&, const SimTK::Vec3& color) const;

    /**
     * Compute the total length of the path.
     *
     * @return Total length of the path.
     */
    double getLength(const SimTK::State&) const;
    void setLength(const SimTK::State&, double length) const;

    double getPreScaleLength(const SimTK::State&) const;
    void setPreScaleLength(const SimTK::State&, double preScaleLength);

    /**
     * Compute the lengthening speed of the path.
     *
     * @return lengthening speed of the path.
     */
    double getLengtheningSpeed(const SimTK::State&) const;
    void setLengtheningSpeed(const SimTK::State&, double speed) const;

    /**
     * Get the current path of the path
     *
     * @return The array of currently active path points.
     */
    const Array<const AbstractPathPoint*>& getCurrentPath(const SimTK::State&) const;

    /**
     * Get the path as PointForceDirections directions, which can be used
     * to apply tension to bodies the points are connected to
     */
    void getPointForceDirections(
        const SimTK::State&,
        OpenSim::Array<PointForceDirection*>* rPFDs) const;

    /**
     * Add in the equivalent body and generalized forces to be applied to the
     * multibody system resulting from a tension along the GeometryPath.
     *
     * @param state    state used to evaluate forces
     * @param[in]  tension      scalar (double) of the applied (+ve) tensile force
     * @param[in,out] bodyForces   Vector of SpatialVec's (torque, force) on bodies
     * @param[in,out] mobilityForces  Vector of generalized forces, one per mobility
     */
    void addInEquivalentForces(
        const SimTK::State&,
        const double& tension,
        SimTK::Vector_<SimTK::SpatialVec>& bodyForces,
        SimTK::Vector& mobilityForces) const;

    /**
     * Compute the path's moment arms for a specified coordinate.
     */
    virtual double computeMomentArm(const SimTK::State&, const Coordinate&) const;

    /**
     * Visualization support: update the geometric representation of the path.
     *
     * The resulting geometry is maintained at the VisibleObjejct layer. It shows (e.g.)
     * location of path points and connecting segments in all global/inertial frames)
     */
    virtual void updateGeometry(const SimTK::State&) const;

    //--------------------------------------------------------------------------
    // API overrides / lifecycle hooks
    //--------------------------------------------------------------------------

protected:
    /**
     * Calculate the path length in the current body position and store it for
     * use after the Model has been scaled.
     */
    void extendPreScale(const SimTK::State&, const ScaleSet&) override;

    /**
     * Recalculate the path after the Model has been scaled.
     */
    void extendPostScale(const SimTK::State&, const ScaleSet&) override;

    void extendFinalizeFromProperties() override;
    void extendConnectToModel(Model&) override;
    void extendInitStateFromProperties(SimTK::State&) const override;
    void extendAddToSystem(SimTK::MultibodySystem&) const override;

    // Visual support GeometryPath drawing in SimTK visualizer.
    void generateDecorations(
        bool fixed,
        const ModelDisplayHints&,
        const SimTK::State&,
        SimTK::Array_<SimTK::DecorativeGeometry>& appendToThis) const override;

private:

    /**
     * Overrides default XML implementation to intercept and fix the XML node
     * to account for versioning.
     */
    void updateFromXMLNode(SimTK::Xml::Element&, int versionNumber = -1) override;


    //--------------------------------------------------------------------------
    // HELPERS
    //--------------------------------------------------------------------------

private:

    void constructProperties();

    Array<const AbstractPathPoint*>& populatePathPtrsCache(const std::vector<ComponentPath>&) const;

    void applyWrapObjects(const SimTK::State&, Array<const AbstractPathPoint*>&) const;

    double calcLengtheningSpeed(const SimTK::State&, const Array<const AbstractPathPoint*>&) const;

    double calcPathLengthChange(
        const SimTK::State&,
        const WrapObject&,
        const WrapResult&,
        const Array<const AbstractPathPoint*>&) const;

    double calcPathLength(
        const SimTK::State&,
        const Array<const AbstractPathPoint*>&) const;

    void namePathPoints(int aStartingIndex);

    void placeNewPathPoint(
        const SimTK::State&,
        SimTK::Vec3& aOffset,
        int index,
        const PhysicalFrame&);

    //=============================================================================
    // DATA
    //=============================================================================

    // used for scaling tendon and fiber lengths
    double _preScaleLength;

    // solver used to compute moment-arms
    //
    // the GeometryPath owns this object, but it is cleared whenever the GeometryPath
    // is copied
    SimTK::ResetOnCopy<std::unique_ptr<MomentArmSolver>> _maSolver;

    // internal cache of raw pointers to the path's points
    //
    // this is here for legacy reasons: the path points are *actually* held in
    // a cache variable as `OpenSim::ComponentPath`s. However, existing APIs might
    // want an array of raw pointers.
    //
    // reset on copy to prevent aliasing issues.
    mutable SimTK::ResetOnCopy<Array<const AbstractPathPoint*>> _currentPathPtrsCache;

    // cache variable for the path's length
    mutable CacheVariable<double> _lengthCV;

    // cache variable for the path's lengthening speed
    mutable CacheVariable<double> _speedCV;

    // cache variable for (component paths to) the path's points
    //
    // populated by the implementation whenever the path's points are computed. The
    // actual data for the points are held elsewhere in the model tree and must be
    // looked up via this (value-type) path.
    mutable CacheVariable<std::vector<ComponentPath>> _currentPathAbspathCV;

    // cache variable for the color of the path (e.g. red, blue)
    mutable CacheVariable<SimTK::Vec3> _colorCV;

//=============================================================================
};  // END of class GeometryPath
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_GEOMETRY_PATH_H_


