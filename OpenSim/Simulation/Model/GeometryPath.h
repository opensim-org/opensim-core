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


// INCLUDE
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include "OpenSim/Simulation/Model/ModelComponent.h"
#include "PathPointSet.h"
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

//=============================================================================
//=============================================================================
/**
 * A base class representing a path (muscle, ligament, etc.).
 *
 * @author Peter Loan
 * @version 1.0
 */
class OSIMSIMULATION_API GeometryPath : public ModelComponent {
OpenSim_DECLARE_CONCRETE_OBJECT(GeometryPath, ModelComponent);
    //=============================================================================
    // OUTPUTS
    //=============================================================================
    OpenSim_DECLARE_OUTPUT(length, double, getLength, SimTK::Stage::Position);
    // 
    OpenSim_DECLARE_OUTPUT(lengthening_speed, double, getLengtheningSpeed,
        SimTK::Stage::Velocity);

//=============================================================================
// DATA
//=============================================================================
private:
    OpenSim_DECLARE_UNNAMED_PROPERTY(PathPointSet,
        "The set of points defining the path");

    OpenSim_DECLARE_UNNAMED_PROPERTY(PathWrapSet,
        "The wrap objects that are associated with this path");
    
    OpenSim_DECLARE_OPTIONAL_PROPERTY(default_color, SimTK::Vec3,
        "Used to initialize the color cache variable");

    // used for scaling tendon and fiber lengths
    double _preScaleLength;

    // Solver used to compute moment-arms. The GeometryPath owns this object,
    // but we cannot simply use a unique_ptr because we want the pointer to be
    // cleared on copy.
    SimTK::ResetOnCopy<std::unique_ptr<MomentArmSolver> > _maSolver;
    
//=============================================================================
// METHODS
//=============================================================================
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
public:
    GeometryPath();
    ~GeometryPath() override = default;

    const PathPointSet& getPathPointSet() const { return get_PathPointSet(); }
    PathPointSet& updPathPointSet() { return upd_PathPointSet(); }
    const PathWrapSet& getWrapSet() const { return get_PathWrapSet(); }
    void addPathWrap(WrapObject& aWrapObject);

    //--------------------------------------------------------------------------
    // UTILITY
    //--------------------------------------------------------------------------
    AbstractPathPoint* addPathPoint(const SimTK::State& s, int index,
        PhysicalFrame& frame);
    AbstractPathPoint* appendNewPathPoint(const std::string& proposedName, 
        PhysicalFrame& frame, const SimTK::Vec3& locationOnFrame);
    bool canDeletePathPoint( int index);
    bool deletePathPoint(const SimTK::State& s, int index);
    
    void moveUpPathWrap(const SimTK::State& s, int index);
    void moveDownPathWrap(const SimTK::State& s, int index);
    void deletePathWrap(const SimTK::State& s, int index);
    bool replacePathPoint(const SimTK::State& s, AbstractPathPoint* oldPathPoint,
        AbstractPathPoint* newPathPoint); 

    //--------------------------------------------------------------------------
    // GET
    //--------------------------------------------------------------------------

    /** If you call this prior to extendAddToSystem() it will be used to initialize
    the color cache variable. Otherwise %GeometryPath will choose its own
    default which will be some boring shade of gray. **/
    void setDefaultColor(const SimTK::Vec3& color) {set_default_color(color); };
    /** Returns the color that will be used to initialize the color cache
    at the next extendAddToSystem() call. The actual color used to draw the path
    will be taken from the cache variable, so may have changed. **/
    const SimTK::Vec3& getDefaultColor() const { return get_default_color(); }

    /** %Set the value of the color cache variable owned by this %GeometryPath
    object, in the cache of the given state. The value of this variable is used
    as the color when the path is drawn, which occurs with the state realized 
    to Stage::Dynamics. So you must call this method during realizeDynamics() or 
    earlier in order for it to have any effect. **/
    void setColor(const SimTK::State& s, const SimTK::Vec3& color) const;

    /** Get the current value of the color cache entry owned by this
    %GeometryPath object in the given state. You can access this value any time
    after the state is initialized, at which point it will have been set to
    the default color value specified in a call to setDefaultColor() earlier,
    or it will have the default color value chosen by %GeometryPath.
    @see setDefaultColor() **/
    SimTK::Vec3 getColor(const SimTK::State& s) const;

    double getLength( const SimTK::State& s) const;
    void setLength( const SimTK::State& s, double length) const;
    double getPreScaleLength( const SimTK::State& s) const;
    void setPreScaleLength( const SimTK::State& s, double preScaleLength);
    const Array<AbstractPathPoint*>& getCurrentPath( const SimTK::State& s) const;

    double getLengtheningSpeed(const SimTK::State& s) const;
    void setLengtheningSpeed( const SimTK::State& s, double speed ) const;

    /** get the path as PointForceDirections directions, which can be used
        to apply tension to bodies the points are connected to.*/
    void getPointForceDirections(const SimTK::State& s, 
        OpenSim::Array<PointForceDirection*> *rPFDs) const;

    /** add in the equivalent body and generalized forces to be applied to the 
        multibody system resulting from a tension along the GeometryPath 
    @param state    state used to evaluate forces
    @param[in]  tension      scalar (double) of the applied (+ve) tensile force 
    @param[in,out] bodyForces   Vector of SpatialVec's (torque, force) on bodies
    @param[in,out] mobilityForces  Vector of generalized forces, one per mobility   
    */
    void addInEquivalentForces(const SimTK::State& state,
                               const double& tension, 
                               SimTK::Vector_<SimTK::SpatialVec>& bodyForces,
                               SimTK::Vector& mobilityForces) const;


    //--------------------------------------------------------------------------
    // COMPUTATIONS
    //--------------------------------------------------------------------------
    virtual double computeMomentArm(const SimTK::State& s, const Coordinate& aCoord) const;

    //--------------------------------------------------------------------------
    // SCALING
    //--------------------------------------------------------------------------
    void preScale(const SimTK::State& s, const ScaleSet& aScaleSet);
    void scale(const SimTK::State& s, const ScaleSet& aScaleSet);
    void postScale(const SimTK::State& s, const ScaleSet& aScaleSet);

    //--------------------------------------------------------------------------
    // Visualization Support
    //--------------------------------------------------------------------------
    // Update the geometry attached to the path (location of path points and
    // connecting segments all in global/inertial frame)
    virtual void updateGeometry(const SimTK::State& s) const;

protected:
    // ModelComponent interface.
    void extendConnectToModel(Model& aModel) override;
    void extendInitStateFromProperties(SimTK::State& s) const override;
    void extendAddToSystem(SimTK::MultibodySystem& system) const override;

    // Visual support GeometryPath drawing in SimTK visualizer.
    void generateDecorations(
            bool                                        fixed,
            const ModelDisplayHints&                    hints,
            const SimTK::State&                         state,
            SimTK::Array_<SimTK::DecorativeGeometry>&   appendToThis) const
            override;

    void extendFinalizeFromProperties() override;

private:

    void computePath(const SimTK::State& s ) const;
    void computeLengtheningSpeed(const SimTK::State& s) const;
    void applyWrapObjects(const SimTK::State& s, Array<AbstractPathPoint*>& path ) const;
    double calcPathLengthChange(const SimTK::State& s, const WrapObject& wo, 
                                const WrapResult& wr, 
                                const Array<AbstractPathPoint*>& path) const; 
    double calcLengthAfterPathComputation
       (const SimTK::State& s, const Array<AbstractPathPoint*>& currentPath) const;

    void constructProperties();
    void namePathPoints(int aStartingIndex);
    void placeNewPathPoint(const SimTK::State& s, SimTK::Vec3& aOffset, 
                           int index, const PhysicalFrame& frame);

//=============================================================================
};  // END of class GeometryPath
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_GEOMETRY_PATH_H_


