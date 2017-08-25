#ifndef OPENSIM_FORCE_VELOCITY_CURVE_H_
#define OPENSIM_FORCE_VELOCITY_CURVE_H_
/* -------------------------------------------------------------------------- *
 *                       OpenSim:  ForceVelocityCurve.h                       *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
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
#include <OpenSim/Common/Function.h>
#include <OpenSim/Common/SmoothSegmentedFunction.h>

#ifdef SWIG
    #ifdef OSIMACTUATORS_API
        #undef OSIMACTUATORS_API
        #define OSIMACTUATORS_API
    #endif
#endif

namespace OpenSim {
/** This class serves as a serializable ForceVelocityCurve for use in muscle
    models. The force-velocity curve is dimensionless: force is normalized to
    maximum isometric force and velocity is normalized to the maximum muscle
    contraction velocity (vmax), where vmax is expressed in units of
    optimal_fiber_lengths per second. Negative normalized velocities correspond
    to concentric contraction (i.e., shortening). The force-velocity curve is
    constructed from 8 properties:

    @param concentricSlopeAtVmax
        The slope of the force-velocity curve at a normalized velocity of -1,
        which is the minimum slope of the concentric side of the force-velocity
        curve. A physiologically accurate value for this parameter is 0, though
        values greater than 0 are necessary when the force-velocity curve must
        be inverted.
    @param concentricSlopeNearVmax
        The slope of the force-velocity curve near the maximum normalized
        concentric (shortening) contraction velocity (between approximately -0.8
        and -1).
    @param isometricSlope
        The slope of the force-velocity curve at a normalized velocity of 0,
        which is the maximum slope of the force-velocity curve. A
        physiologically accurate value for this parameter is 5 (according to
        Lieber, page 55), which is the default value. Although this parameter
        can be changed, it must be positive and greater than
        max( (maxEccentricMultiplier-1)/1, 1). The value of this parameter also
        affects how much the eccentric and concentric curves can be bent by the
        'eccentricCurviness' and 'concentricCurviness' parameters, as it places
        an upper limit on the maximum slope of the force-velocity curve.
    @param eccentricSlopeAtVmax
        The slope of the force-velocity curve at a normalized velocity of 1,
        which is the minimum slope of the eccentric side of the force-velocity
        curve.
    @param eccentricSlopeNearVmax
        The slope of the force-velocity curve near the maximum normalized
        eccentric (lengthening) contraction velocity (between approximately 0.8
        and 1).
    @param maxEccentricVelocityForceMultiplier
        The value of the force-velocity curve (i.e., the force-velocity
        multiplier) at the maximum eccentric contraction velocity.
        Physiologically accurate values for this parameter range between 1.1
        and 1.8, and may vary between subjects.
    @param concentricCurviness
        A dimensionless parameter between 0 and 1 that describes the shape of
        the concentric curve: a value of 0 indicates that the curve is very
        close to a straight line segment and a value of 1 indicates a curve that
        smoothly fills the corner formed by the linear extrapolation of
        'concentricSlopeNearVmax' and 'isometricSlope', as shown in the figure.
    @param eccentricCurviness
        A dimensionless parameter between 0 and 1 that describes the shape of
        the eccentric curve: a value of 0 indicates that the curve is very close
        to a straight line segment and a value of 1 indicates a curve that
        smoothly fills the corner formed by the linear extrapolation of
        'isometricSlope' and 'eccentricSlopeNearVmax', as shown in the figure.

    \image html fig_ForceVelocityCurve.png

    <B>Conditions</B>
    \verbatim
     1)  0 <= concentricSlopeAtVmax < 1
    2a)  1 < isometricSlope
    2b)  (maxEccentricVelocityForceMultiplier-1)/1 < isometricMaxSlope
     3)  0 <= eccentricSlopeAtVmax < (maxEccentricVelocityForceMultiplier-1)/1
     4)  1 < maxEccentricVelocityForceMultiplier
     5)  0 <= concentricCurviness <= 1
     6)  0 <= eccentricCurviness <= 1
    \endverbatim

    <B>Default Parameter Values</B>
    \verbatim
    concentricSlopeAtVmax .................. 0.0
    concentricSlopeNearVmax ................ 0.25
    isometricSlope ......................... 5.0
    eccentricSlopeAtVmax ................... 0.0
    eccentricSlopeNearVmax ................. 0.15
    maxEccentricVelocityForceMultiplier .... 1.4
    concentricCurviness .................... 0.6
    eccentricCurviness ..................... 0.9
    \endverbatim

    <B>Example</B>
    @code
        ForceVelocityCurve fvCurve(0.0, 0.25, 5.0, 0.0, 0.15, 1.4, 0.6, 0.9);
        double falVal  = fvCurve.calcValue(1.0);
        double dfalVal = fvCurve.calcDerivative(1.0, 1);
    @endcode

    <B>References</B>
    \li Lieber, R.L. (2010) Skeletal %Muscle Structure, %Function, and
        Plasticity: The Physiological Basis of Rehabilitation, 3rd ed.
        Baltimore: Lippincott Williams & Wilkins.

    Note that this object should be updated through the set methods provided.
    These set methods will take care of rebuilding the curve correctly. If you
    modify the properties directly, the curve will not be rebuilt, and upon
    calling a function like calcValue, calcDerivative, or printCurveToCSVFile,
    an exception will be thrown because the curve is out-of-date with its
    properties.

    @author Matt Millard
*/
class OSIMACTUATORS_API ForceVelocityCurve : public Function {
OpenSim_DECLARE_CONCRETE_OBJECT(ForceVelocityCurve, Function);
public:
//==============================================================================
// PROPERTIES
//==============================================================================
    OpenSim_DECLARE_PROPERTY(concentric_slope_at_vmax, double,
        "Curve slope at the maximum normalized concentric (shortening) velocity (normalized velocity of -1)");
    OpenSim_DECLARE_PROPERTY(concentric_slope_near_vmax, double,
        "Curve slope just before reaching concentric_slope_at_vmax");
    OpenSim_DECLARE_PROPERTY(isometric_slope, double,
        "Curve slope at isometric (normalized velocity of 0)");
    OpenSim_DECLARE_PROPERTY(eccentric_slope_at_vmax, double,
        "Curve slope at the maximum normalized eccentric (lengthening) velocity (normalized velocity of 1)");
    OpenSim_DECLARE_PROPERTY(eccentric_slope_near_vmax, double,
        "Curve slope just before reaching eccentric_slope_at_vmax");
    OpenSim_DECLARE_PROPERTY(max_eccentric_velocity_force_multiplier, double,
        "Curve value at the maximum normalized eccentric contraction velocity");
    OpenSim_DECLARE_PROPERTY(concentric_curviness, double,
        "Concentric curve shape, from linear (0) to maximal curve (1)");
    OpenSim_DECLARE_PROPERTY(eccentric_curviness, double,
        "Eccentric curve shape, from linear (0) to maximal curve (1)");

//==============================================================================
// PUBLIC METHODS
//==============================================================================
    /** The default constructor creates a force-velocity curve using the default
    property values and assigns a default name. */
    ForceVelocityCurve();

    /** Constructs a force-velocity curve using the provided parameters and
    assigns a default name. */
    ForceVelocityCurve(double concentricSlopeAtVmax,
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
    shape of the concentric curve: a value of 0 indicates that the curve is very
    close to a straight line segment and a value of 1 indicates a curve that
    smoothly fills the corner formed by the linear extrapolation of
    'concentricSlopeNearVmax' and 'isometricSlope', as shown in the figure in
    the class description. */
    double getConcentricCurviness() const;

    /** @returns A dimensionless parameter between 0 and 1 that describes the
    shape of the eccentric curve: a value of 0 indicates that the curve is very
    close to a straight line segment and a value of 1 indicates a curve that
    smoothly fills the corner formed by the linear extrapolation of
    'isometricSlope' and 'eccentricSlopeNearVmax', as shown in the figure in the
    class description. */
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
    1a)  0 <= concentricSlopeAtVmax < 1
    1b)  concentricSlopeAtVmax < concentricSlopeNearVmax < 1
    2a)  1 < isometricSlope
    2b)  (maxEccentricVelocityForceMultiplier-1)/1 < isometricMaxSlope
    3a)  0 <= eccentricSlopeAtVmax < (maxEccentricVelocityForceMultiplier-1)/1
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
        the concentric curve: a value of 0 indicates that the curve is very
        close to a straight line segment and a value of 1 indicates a curve that
        smoothly fills the corner formed by the linear extrapolation of
        'concentricSlopeNearVmax' and 'isometricSlope', as shown in the figure
        in the class description.

    <B>Conditions</B>
    \verbatim
    0 <= concentricCurviness <= 1
    \endverbatim
    */
    void setConcentricCurviness(double aConcentricCurviness);

    /**
    @param aEccentricCurviness
        A dimensionless parameter between 0 and 1 that describes the shape of
        the eccentric curve: a value of 0 indicates that the curve is very close
        to a straight line segment and a value of 1 indicates a curve that
        smoothly fills the corner formed by the linear extrapolation of
        'isometricSlope' and 'eccentricSlopeNearVmax', as shown in the figure in
        the class description.

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

    /** Evaluates the force-velocity curve at a normalized fiber velocity of
    'normFiberVelocity'. */
    double calcValue(double normFiberVelocity) const;

    /** Calculates the derivative of the force-velocity multiplier with respect
    to the normalized fiber velocity.
    @param normFiberVelocity
        The normalized velocity of the muscle fiber.
    @param order
        The order of the derivative. Only values of 0, 1, and 2 are acceptable.
    @return
        The derivative of the force-velocity curve with respect to the
        normalized fiber velocity.
    */
    double calcDerivative(double normFiberVelocity, int order) const;
    

    /// If possible, use the simpler overload above.
    double calcDerivative(const std::vector<int>& derivComponents,
                          const SimTK::Vector& x) const override;

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
    "bicepsfemoris_ForceVelocityCurve.csv"). This function is not const to
    permit the curve to be rebuilt if it is out-of-date with its properties.
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
    force-velocity .csv file will have 200+20 rows.

    <B>Example</B>
    To read the .csv file into Matlab, you need to set csvread to ignore the
    header row. Since csvread is 0-indexed, the following example will begin
    reading the .csv file from the first column of the second row:
    \verbatim
    data = csvread('bicepsfemoris_ForceVelocityCurve.csv', 1, 0);
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

    SmoothSegmentedFunction m_curve;
};

}

#endif // OPENSIM_FORCE_VELOCITY_CURVE_H_
