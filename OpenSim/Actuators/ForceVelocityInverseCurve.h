#ifndef OPENSIM_FORCE_VELOCITY_INVERSE_CURVE_H_
#define OPENSIM_FORCE_VELOCITY_INVERSE_CURVE_H_
/* -------------------------------------------------------------------------- *
 *                   OpenSim:  ForceVelocityInverseCurve.h                    *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Matthew Millard                                                 *
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
#include <OpenSim/Actuators/osimActuatorsDLL.h>
#include <simbody/internal/common.h>
#include <OpenSim/Common/Function.h>
#include <OpenSim/Common/SmoothSegmentedFunctionFactory.h>
#include <OpenSim/Common/SmoothSegmentedFunction.h>

#ifdef SWIG
#ifdef OSIMACTUATORS_API
#undef OSIMACTUATORS_API
#define OSIMACTUATORS_API
#endif
#endif

namespace OpenSim {
/** This class serves as a serializable ForceVelocityInverseCurve for use in
    equilibrium muscle models. The inverse force-velocity curve is
    dimensionless: force is normalized to maximum isometric force and velocity
    is normalized to the maximum muscle contraction velocity (vmax), where vmax
    is expressed in units of optimal_fiber_lengths per second. Negative
    normalized velocities correspond to concentric contraction (i.e.,
    shortening). The inverse force-velocity curve is constructed from 8
    properties, which are identical to those used to construct the
    corresponding force-velocity curve. See ForceVelocityCurve for descriptions
    of these parameters.

    @param concentricSlopeAtVmax
        An exception will be thrown if this parameter is set to 0.
    @param concentricSlopeNearVmax
        An exception will be thrown if this parameter is set to 0.
    @param isometricSlope
    @param eccentricSlopeAtVmax
        An exception will be thrown if this parameter is set to 0.
    @param eccentricSlopeNearVmax
        An exception will be thrown if this parameter is set to 0.
    @param maxEccentricVelocityForceMultiplier
    @param concentricCurviness
    @param eccentricCurviness

    \image html fig_ForceVelocityInverseCurve.png
    <BR>
    \image html fig_ForceVelocityCurve.png

    <B>Conditions</B>
    \verbatim
    1a)  0 < concentricSlopeAtVmax < 1
    1b)  concentricSlopeAtVmax < concentricSlopeNearVmax < 1
    2a)  1 < isometricSlope
    2b)  (maxEccentricVelocityForceMultiplier-1)/1 < isometricSlope
     3)  0 < eccentricSlopeAtVmax < (maxEccentricVelocityForceMultiplier-1)/1
     4)  1 < maxEccentricVelocityForceMultiplier
     5)  0 <= concentricCurviness <= 1
     6)  0 <= eccentricCurviness <= 1
    \endverbatim

    <B>Default Parameter Values</B>
    \verbatim
    concentricSlopeAtVmax .................. 0.1
    concentricSlopeNearVmax ................ 0.25
    isometricSlope ......................... 5.0
    eccentricSlopeAtVmax ................... 0.1
    eccentricSlopeNearVmax ................. 0.15
    maxEccentricVelocityForceMultiplier .... 1.4
    concentricCurviness .................... 0.6
    eccentricCurviness ..................... 0.9
    \endverbatim

    <B>Example</B>
    @code
        ForceVelocityInverseCurve fvInvCurve(0.1, 0.25, 5.0, 0.1, 0.15, 1.4, 0.6, 0.9);
        double fvInvVal  = fvInvCurve.calcValue(1.0);
        double dfvInvVal = fvInvCurve.calcDerivative(1.0, 1);
    @endcode

    Note that this object should be updated through the set methods provided.
    These set methods will take care of rebuilding the curve correctly. If you
    modify the properties directly, the curve will not be rebuilt, and upon
    calling a function like calcValue, calcDerivative, or printCurveToCSVFile,
    an exception will be thrown because the curve is out-of-date with its
    properties.

    @author Matt Millard
*/
class OSIMACTUATORS_API ForceVelocityInverseCurve : public Function {
    OpenSim_DECLARE_CONCRETE_OBJECT(ForceVelocityInverseCurve, Function);
public:
//==============================================================================
// PROPERTIES
//==============================================================================
    /** @name Property declarations
        These are the serializable properties associated with this class. **/
    /**@{**/
    OpenSim_DECLARE_PROPERTY(concentric_slope_at_vmax, double,
                             "Slope of force-velocity curve at the maximum normalized concentric (shortening) velocity (normalized velocity of -1)");
    OpenSim_DECLARE_PROPERTY(concentric_slope_near_vmax, double,
                             "Slope of force-velocity curve just before reaching concentric_slope_at_vmax");
    OpenSim_DECLARE_PROPERTY(isometric_slope, double,
                             "Slope of force-velocity curve at isometric (normalized velocity of 0)");
    OpenSim_DECLARE_PROPERTY(eccentric_slope_at_vmax, double,
                             "Slope of force-velocity curve at the maximum normalized eccentric (lengthening) velocity (normalized velocity of 1)");
    OpenSim_DECLARE_PROPERTY(eccentric_slope_near_vmax, double,
                             "Slope of force-velocity curve just before reaching eccentric_slope_at_vmax");
    OpenSim_DECLARE_PROPERTY(max_eccentric_velocity_force_multiplier, double,
                             "Value of force-velocity curve at the maximum normalized eccentric contraction velocity");
    OpenSim_DECLARE_PROPERTY(concentric_curviness, double,
                             "Shape of concentric branch of force-velocity curve, from linear (0) to maximal curve (1)");
    OpenSim_DECLARE_PROPERTY(eccentric_curviness, double,
                             "Shape of eccentric branch of force-velocity curve, from linear (0) to maximal curve (1)");
    /**@}**/

//==============================================================================
// PUBLIC METHODS
//==============================================================================
    /** The default constructor creates an inverse force-velocity curve using
    the default property values and assigns a default name. */
    ForceVelocityInverseCurve();

    /** Constructs an inverse force-velocity curve using the provided parameters
    and assigns a default name. */
    ForceVelocityInverseCurve(double concentricSlopeAtVmax,
                              double concentricSlopeNearVmax,
                              double isometricSlope,
                              double eccentricSlopeAtVmax,
                              double eccentricSlopeNearVmax,
                              double maxEccentricVelocityForceMultiplier,
                              double concentricCurviness,
                              double eccentricCurviness);

    /** @returns The slope of the force-velocity curve at a normalized velocity
    of -1, which is the minimum slope of the concentric side of the
    force-velocity curve. */
    double getConcentricSlopeAtVmax() const;

    /** @returns The slope of the force-velocity curve near the maximum
    normalized concentric (shortening) contraction velocity (between
    approximately -0.8 and -1). */
    double getConcentricSlopeNearVmax() const;

    /** @returns The slope of the force-velocity curve at a normalized velocity
    of 0, which is the maximum slope of the force-velocity curve. */
    double getIsometricSlope() const;

    /** @returns The slope of the force-velocity curve at a normalized velocity
    of 1, which is the minimum slope of the eccentric side of the force-velocity
    curve. */
    double getEccentricSlopeAtVmax() const;

    /** @returns The slope of the force-velocity curve near the maximum
    normalized eccentric (lengthening) contraction velocity (between
    approximately 0.8 and 1). */
    double getEccentricSlopeNearVmax() const;

    /** @returns The value of the force-velocity curve (i.e., the force-velocity
    multiplier) at the maximum eccentric contraction velocity. */
    double getMaxEccentricVelocityForceMultiplier() const;

    /** @returns A dimensionless parameter between 0 and 1 that describes the
    shape of the concentric branch of the force-velocity curve: a value of 0
    indicates that the curve is very close to a straight line segment and a
    value of 1 indicates a curve that smoothly fills the corner formed by the
    linear extrapolation of 'concentricSlopeNearVmax' and 'isometricSlope', as
    shown in the figure in the class description. */
    double getConcentricCurviness() const;

    /** @returns A dimensionless parameter between 0 and 1 that describes the
    shape of the eccentric branch of the force-velocity curve: a value of 0
    indicates that the curve is very close to a straight line segment and a
    value of 1 indicates a curve that smoothly fills the corner formed by the
    linear extrapolation of 'isometricSlope' and 'eccentricSlopeNearVmax', as
    shown in the figure in the class description. */
    double getEccentricCurviness() const;

    /**
    @param aConcentricSlopeAtVmax
        The slope of the force-velocity curve at a normalized velocity of -1,
        which is the minimum slope of the concentric side of the force-velocity
        curve.
    @param aConcentricSlopeNearVmax
        The slope of the force-velocity curve near the maximum normalized
        concentric (shortening) contraction velocity (between approximately
        -0.8 and -1).
    @param aIsometricSlope
        The slope of the force-velocity curve at a normalized velocity of 0,
        which is the maximum slope of the force-velocity curve.
    @param aEccentricSlopeAtVmax
        The slope of the force-velocity curve at a normalized velocity of 1,
        which is the minimum slope of the eccentric side of the force-velocity
        curve.
    @param aEccentricSlopeNearVmax
        The slope of the force-velocity curve near the maximum normalized
        eccentric (lengthening) contraction velocity (between approximately 0.8
        and 1).
    @param aMaxForceMultiplier
        The value of the force-velocity curve (i.e., the force-velocity
        multiplier) at the maximum eccentric contraction velocity.

    <B>Conditions</B>
    \verbatim
    1a)  0 < concentricSlopeAtVmax < 1
    1b)  concentricSlopeAtVmax < concentricSlopeNearVmax < 1
    2a)  1 < isometricSlope
    2b)  (maxEccentricVelocityForceMultiplier-1)/1 < isometricMaxSlope
    3a)  0 < eccentricSlopeAtVmax < (maxEccentricVelocityForceMultiplier-1)/1
    3b)  eccentricSlopeAtVmax < eccentricSlopeNearVmax < (maxEccentricVelocityForceMultiplier-1)/1
     4)  1 < maxEccentricVelocityForceMultiplier
    \endverbatim
    */
    void setCurveShape(double aConcentricSlopeAtVmax,
                       double aConcentricSlopeNearVmax,
                       double aIsometricSlope,
                       double aEccentricSlopeAtVmax,
                       double aEccentricSlopeNearVmax,
                       double aMaxForceMultiplier);

    /**
    @param aConcentricCurviness
        A dimensionless parameter between 0 and 1 that describes the shape of
        the concentric branch of the force-velocity curve: a value of 0
        indicates that the curve is very close to a straight line segment and a
        value of 1 indicates a curve that smoothly fills the corner formed by
        the linear extrapolation of 'concentricSlopeNearVmax' and
        'isometricSlope', as shown in the figure in the class description.

    <B>Conditions</B>
    \verbatim
    0 <= concentricCurviness <= 1
    \endverbatim
    */
    void setConcentricCurviness(double aConcentricCurviness);

    /**
    @param aEccentricCurviness
        A dimensionless parameter between 0 and 1 that describes the shape of
        the eccentric branch of the force-velocity curve: a value of 0 indicates
        that the curve is very close to a straight line segment and a value of 1
        indicates a curve that smoothly fills the corner formed by the linear
        extrapolation of 'isometricSlope' and 'eccentricSlopeNearVmax', as shown
        in the figure in the class description.

    <B>Conditions</B>
    \verbatim
    0 <= eccentricCurviness <= 1
    \endverbatim
    */
    void setEccentricCurviness(double aEccentricCurviness);

    /** Implement the generic OpenSim::Function interface **/
    double calcValue(const SimTK::Vector& x) const override
    {
        return calcValue(x[0]);
    }

    /** Evaluates the inverse force-velocity curve at a force-velocity
    multiplier value of 'aForceVelocityMultiplier'. */
    double calcValue(double aForceVelocityMultiplier) const;

    /** Calculates the derivative of the inverse force-velocity curve with
    respect to the force-velocity multiplier.
    @param aForceVelocityMultiplier
        The force-velocity multiplier value.
    @param order
        The order of the derivative. Only values of 0, 1, and 2 are acceptable.
    @return
        The derivative of the inverse force-velocity curve with respect to the
        force-velocity multiplier.
    */
    double calcDerivative(double aForceVelocityMultiplier, int order) const;

    /** Returns a SimTK::Vec2 containing the lower (0th element) and upper (1st
    element) bounds on the domain of the curve. Outside this domain, the curve
    is approximated using linear extrapolation.
    @return
        The minimum and maximum value of the domain, x, of the curve y(x).
        Within this range, y(x) is a curve; outside this range, the function
        y(x) is a C2-continuous linear extrapolation.
    */
    SimTK::Vec2 getCurveDomain() const;

    /** Generates a .csv file with a name that matches the curve name (e.g.,
    "bicepsfemoris_ForceVelocityInverseCurve.csv"). This function is not const
    to permit the curve to be rebuilt if it is out-of-date with its properties.
    @param path
        The full destination path. Note that forward slashes ('/') must be used
        and there should not be a slash after the last folder.

    The file will contain the following data:
    \verbatim
    column: 1 | 2 |     3 |       4
      data: x | y | dy/dx | d2y/dx2
    \endverbatim

    Samples will be taken from the concentric linear extrapolation region (the
    region with normalized fiber velocities < -1), through the curve, out to the
    eccentric linear extrapolation region (the region with normalized fiber
    velocities > 1). The width of each linear extrapolation region is 10% of the
    curve domain, or 0.1*(x1-x0). The curve is sampled quite densely: the
    inverse force-velocity .csv file will have 200+20 rows.

    <B>Example</B>
    To read the .csv file into Matlab, you need to set csvread to ignore the
    header row. Since csvread is 0-indexed, the following example will begin
    reading the .csv file from the first column of the second row:
    \verbatim
    data = csvread('bicepsfemoris_ForceVelocityInverseCurve.csv', 1, 0);
    \endverbatim
    */
    void printMuscleCurveToCSVFile(const std::string& path);

    void ensureCurveUpToDate();
//==============================================================================
// PRIVATE
//==============================================================================
private:
    // OpenSim::Function Interface
    // Create the underlying SimTK::Function that implements the calculations
    // necessary for this curve.
    SimTK::Function* createSimTKFunction() const override;

    void setNull();
    void constructProperties();

    // This function will take all of the current property values and build a
    // curve.
    void buildCurve();

    SmoothSegmentedFunction   m_curve;

};

}

#endif // OPENSIM_FORCE_VELOCITY_INVERSE_CURVE_H_
