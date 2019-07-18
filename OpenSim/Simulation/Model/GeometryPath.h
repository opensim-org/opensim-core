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
#include <OpenSim/Common/Function.h>
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
 * For some applications, it is desirable to approximate the length, lengthening
 * speed, and moment arms of the path using a function approximation. Function
 * approximations may be smoother and faster than the path calculations, which
 * is useful for numerical optimization. To use a function approximation, set
 * the length_approximation, approximation_coordinates, and use_approximation
 * properties. You are responsible for developing the function approxmation
 * yourself (perhaps using a fitting algorithm in Matlab); this class will not
 * create the function approximation for you. The lengthening speed \f$
 * \dot{L}(q) \f$ and moment arm for coordinate \f$ i \f$, \f$ r_i(q) \f$, are
 * computed from the length approximation \f$ L(q) ]\f$ as follows:
 *
 * \f[
 * \begin{alignat*}{2}
 *     r_i(q) &= -\frac{\partial L}{\partial q_i} \\
 *     \dot{L}(q) &= \sum_i \frac{\partial L}{\partial q_i} \dot{q}_i
 * \end{alignat*}
 * \f]
 *
 * Visualization (generateDecorations()) and getPointForceDirections() always
 * use the path points, even if the object is set to use the approximation.
 * Avoid these functions if you want to avoid the more expensive path point
 * calculations. Scaling the model clears the length approximation. You should
 * update your function approximation yourself after scaling your model.
 *
 * @note When using the function approximation, joint reaction calculations will
 * not correctly account for muscles, as muscle forces are applied via
 * generalized forces (mobility forces), not at the geometry path's path points.
 *
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
public:
    OpenSim_DECLARE_UNNAMED_PROPERTY(Appearance,
        "Default appearance attributes for this GeometryPath");

    OpenSim_DECLARE_OPTIONAL_PROPERTY(length_approximation, Function,
        "An approximation for path length as a function of coordinate values.");

    OpenSim_DECLARE_LIST_PROPERTY(approximation_coordinates, std::string,
        "The component paths of coordinates whose values are inputs to "
        "length_approximation. The order must match the order of arguments "
        "for length_approximation.");

    OpenSim_DECLARE_PROPERTY(use_approximation, bool,
        "Default setting for using the length_approximation to "
        "compute length, lengthening speed, and moment arms.");

private:
    OpenSim_DECLARE_UNNAMED_PROPERTY(PathPointSet,
        "The set of points defining the path");

    OpenSim_DECLARE_UNNAMED_PROPERTY(PathWrapSet,
        "The wrap objects that are associated with this path");

    // used for scaling tendon and fiber lengths
    double _preScaleLength;

    // Solver used to compute moment-arms. The GeometryPath owns this object,
    // but we cannot simply use a unique_ptr because we want the pointer to be
    // cleared on copy.
    SimTK::ResetOnCopy<std::unique_ptr<MomentArmSolver> > _maSolver;

    mutable std::vector<SimTK::ReferencePtr<const Coordinate>> m_approxCoords;

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
        const PhysicalFrame& frame);
    AbstractPathPoint* appendNewPathPoint(const std::string& proposedName,
        const PhysicalFrame& frame, const SimTK::Vec3& locationOnFrame);
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
    bool getUseApproximation(const SimTK::State& s) const {
        return getModelingOption(s, USE_APPROXIMATION_NAME) == 1;
    }
    void setUseApproximation(SimTK::State& s, bool approx) const {
        if (approx) {
            OPENSIM_THROW_IF(getProperty_length_approximation().empty(),
                             Exception,
                             "Cannot use length approximation; no function "
                             "provided.");
        }
        setModelingOption(s, USE_APPROXIMATION_NAME, int(approx));
    }

    /** If you call this prior to extendAddToSystem() it will be used to initialize
    the color cache variable. Otherwise %GeometryPath will choose its own
    default which varies depending on owner. **/
    void setDefaultColor(const SimTK::Vec3& color) {
        updProperty_Appearance().setValueIsDefault(false);
        upd_Appearance().set_color(color);
    };
    /** Returns the color that will be used to initialize the color cache
    at the next extendAddToSystem() call. The actual color used to draw the path
    will be taken from the cache variable, so may have changed. **/
    const SimTK::Vec3& getDefaultColor() const { return get_Appearance().get_color(); }

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
    virtual double computeMomentArm(
            const SimTK::State& s, const Coordinate& aCoord) const;

    //--------------------------------------------------------------------------
    // SCALING
    //--------------------------------------------------------------------------

    /** Calculate the path length in the current body position and store it for
        use after the Model has been scaled. */
    void extendPreScale(const SimTK::State& s,
                        const ScaleSet& scaleSet) override;

    /** Recalculate the path after the Model has been scaled. */
    void extendPostScale(const SimTK::State& s,
                         const ScaleSet& scaleSet) override;

    //--------------------------------------------------------------------------
    // Visualization Support
    //--------------------------------------------------------------------------
    // Update the geometry attached to the path (location of path points and
    // connecting segments all in global/inertial frame)
    virtual void updateGeometry(const SimTK::State& s) const;

protected:
    // ModelComponent interface.
    void extendConnectToModel(Model& aModel) override;
    void extendSetPropertiesFromState(const SimTK::State& s) override;
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

    /// Compute a moment arm for a coordinate in m_approxCoords, identified by
    /// approxCoordIndex, using the length_approximation function.
    /// The length of approxQ should be the length of approximation_coordinates.
    double computeMomentArmWithApproximation(
            int approxCoordIndex, const SimTK::Vector& approxQ) const;

    void constructProperties();
    void namePathPoints(int aStartingIndex);
    void placeNewPathPoint(const SimTK::State& s, SimTK::Vec3& aOffset,
                           int index, const PhysicalFrame& frame);
    //--------------------------------------------------------------------------
    // Implement Object interface.
    //--------------------------------------------------------------------------
    /** Override of the default implementation to account for versioning. */
    void updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber = -1) override;

    static const std::string USE_APPROXIMATION_NAME;

//=============================================================================
};  // END of class GeometryPath
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_GEOMETRY_PATH_H_


