#ifndef OPENSIM_ABSTRACT_GEOMETRY_PATH_H
#define OPENSIM_ABSTRACT_GEOMETRY_PATH_H
/* -------------------------------------------------------------------------- *
 *                   OpenSim:  AbstractGeometryPath.h                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2023 Stanford University and the Authors                *
 * Author(s): Nicholas Bianco, Joris Verhagen, Adam Kewley                    *
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
#include <OpenSim/Simulation/Model/Appearance.h>

#ifdef SWIG
    #ifdef OSIMSIMULATION_API
        #undef OSIMSIMULATION_API
        #define OSIMSIMULATION_API
    #endif
#endif

namespace OpenSim { class Coordinate; }
namespace OpenSim { class ForceConsumer; }

namespace OpenSim {

//=============================================================================
//                         ABSTRACT GEOMETRY PATH
//=============================================================================
/**
 * A base class that represents a path that has a computable length and
 * lengthening speed.
 *
 * This class is typically used in places where the model needs to simulate
 * the changes in a path over time. For example, in `OpenSim::Muscle`s,
 * `OpenSim::Ligament`s, etc.
 *
 * This class *only* defines a length and lengthening speed. We do not assume
 * that an `OpenSim::AbstractGeometryPath` is a straight line between two points
 * or assume that it is many straight lines between `n` points. The derived
 * implementation may define a path using points, or it may define a path using
 * a curve fit. It may also define a path based on analytical functions for the
 * length and lengthening speed.
 */
class OSIMSIMULATION_API AbstractGeometryPath : public ModelComponent {
OpenSim_DECLARE_ABSTRACT_OBJECT(AbstractGeometryPath, ModelComponent);

public:
//=============================================================================
// OUTPUTS
//=============================================================================
    OpenSim_DECLARE_OUTPUT(length, double, getLength, SimTK::Stage::Position);
    OpenSim_DECLARE_OUTPUT(lengthening_speed, double, getLengtheningSpeed,
            SimTK::Stage::Velocity);

//=============================================================================
// PROPERTIES
//=============================================================================
    OpenSim_DECLARE_UNNAMED_PROPERTY(Appearance,
            "Default appearance attributes for this AbstractGeometryPath.");

//=============================================================================
// METHODS
//=============================================================================
    
    // CONSTRUCTION AND DESTRUCTION
    AbstractGeometryPath();
    ~AbstractGeometryPath() noexcept override;

    AbstractGeometryPath(const AbstractGeometryPath&);
    AbstractGeometryPath& operator=(const AbstractGeometryPath&);

    AbstractGeometryPath(AbstractGeometryPath&& other);
    AbstractGeometryPath& operator=(AbstractGeometryPath&& other);

    // INTERFACE METHODS
    //
    // Concrete implementations of `AbstractGeometryPath` *must* provide these.

    /**
     * Get the current length of the path.
     *
     * Internally, this may use a variety of methods to figure out how long the
     * path is, such as using spline-fits, or computing the distance between
     * points in space. It is up to concrete implementations (e.g.,
     * `GeometryPath`) to provide a relevant implementation.
     */
    virtual double getLength(const SimTK::State& s) const = 0;

    /**
     * Get the lengthening speed of the path.
     *
     * Internally, this may use a variety of methods to figure out the
     * lengthening speed. It might use the finite difference between two
     * lengths, or an analytic solution, or always return `0.0`. It is up to
     * concrete implementations (e.g., `GeometryPath`) to provide a relevant
     * implementation.
     */
    virtual double getLengtheningSpeed(const SimTK::State& s) const = 0;

    /**
     * Requests that the concrete implementation produces forces resulting from
     * applying a tension along its path, emitting them into the supplied
     * `ForceConsumer`.
     *
     * @param state         the state used to evaluate forces
     * @param tension       scalar of the applied (+ve) tensile force
     * @param forceConsumer a `ForceConsumer` shall receive each produced force
     */
    virtual void produceForces(const SimTK::State& state,
            double tension,
            ForceConsumer& forceConsumer) const = 0;

    /**
     * Add in the equivalent body and generalized forces to be applied to the
     * multibody system resulting from a tension along the AbstractGeometryPath.
     *
     * Note: this internally uses `produceForces`
     *
     *  @param         state           state used to evaluate forces
     *  @param[in]     tension         scalar of the applied (+ve) tensile force
     *  @param[in,out] bodyForces      Vector of forces (SpatialVec's) on bodies
     *  @param[in,out] mobilityForces  Vector of generalized forces
     */
    void addInEquivalentForces(const SimTK::State& state,
            const double& tension,
            SimTK::Vector_<SimTK::SpatialVec>& bodyForces,
            SimTK::Vector& mobilityForces) const;

    /**
     * Returns the moment arm of the path in the given state with respect to
     * the specified coordinate.
     */
    virtual double computeMomentArm(const SimTK::State& s,
            const Coordinate& aCoord) const = 0;
    
    /**
     * Return whether or not a path can be visualized.
     * 
     * Concrete implementations may be visualizable (e.g., `GeometryPath`) or
     * they may not be and therefore must provide a relevant implementation. 
     */
    virtual bool isVisualPath() const = 0;

    // DEFAULTED METHODS
    //
    // These are methods that for which AbstractGeometryPath provides default
    // implementation.

    /**
     * Get the default color of the path.
     *
     * Returns the color that will be used to initialize the color cache
     * at the next extendAddToSystem() call. Use `getColor` to retrieve the
     * (potentially different) color that will be used to draw the path.
     */
    const SimTK::Vec3& getDefaultColor() const;

    /**
     * Set the default color of the path.
     *
     * Sets the internal, default, color value for the path. This is the color
     * that's used when the simulation is initialized (specifically, during the
     * `extendAddToSystem` call).
     *
     * This color is not necessarily the *current* color of the path. Other code
     * in the system (e.g. muscle implementations) may change the runtime color
     * with `setColor`. Use `getColor`, with a particular simulation state, to
     * get the color of the path in that state.
     */
    void setDefaultColor(const SimTK::Vec3& color);
    
    /**
     * Get the current color of the path.
     *
     * This is the runtime, potentially state-dependent, color of the path. It
     * is the color used to display the path in that state (e.g., for UI
     * rendering).
     *
     * This color value is typically initialized with the default color (see:
     * `getDefaultColor`), but the color can change between simulation states
     * because downstream code (e.g. muscles) might call `setColor` to implement
     * state-dependent path coloring.
     * 
     * If not overridden in concrete implementations, this method returns the
     * default color.
     */
    virtual SimTK::Vec3 getColor(const SimTK::State& s) const;
    
    /**
     * Set the current color of the path.
     *
     * Internally, sets the current color value of the path for the provided
     * state (e.g. using cache variables).
     *
     * The value of this variable is used as the color when the path is drawn,
     * which occurs with the state realized to Stage::Dynamics. Therefore, you
     * must call this method during realizeDynamics() or earlier in order for it
     * to have any effect.
     * 
     * If not overridden in concrete implementations, this method does nothing.
     */
    virtual void setColor(
            const SimTK::State& s, const SimTK::Vec3& color) const {};

    /**
     * Get the current length of the path, *before* the last set of scaling
     * operations were applied to it.
     *
     * Internally, the path stores the original length in a `double` during
     * `extendPreScale`. Therefore, be *very* careful with this method, because
     * the recorded length is dependent on the length as computed during
     * `extendPreScale`, which may have been called with a different state.
     */
    double getPreScaleLength(const SimTK::State& s) const;
    void setPreScaleLength(const SimTK::State& s, double preScaleLength);
    
private:
    // Used by `(get|set)PreLengthScale`. Used during `extend(Pre|Post)Scale` by
    // downstream users of AbstractGeometryPath to cache the length of the path
    // before scaling.
    //
    // Ideally, downstream classes would perform the caching themselves, because
    // the AbstractGeometryPath API isn't an ideal place to store this
    // information. This field is mostly here for backwards-compatability with
    // the API.
    double _preScaleLength = 0.0;
};

} // namespace OpenSim

#endif // OPENSIM_ABSTRACT_GEOMETRY_PATH_H
