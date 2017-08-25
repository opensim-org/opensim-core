#ifndef OPENSIM_FIBER_FORCE_LENGTH_CURVE_H_
#define OPENSIM_FIBER_FORCE_LENGTH_CURVE_H_
/* -------------------------------------------------------------------------- *
 *                     OpenSim:  FiberForceLengthCurve.h                      *
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
/** This class serves as a serializable FiberForceLengthCurve, commonly used to
    model the parallel elastic element in muscle models. The fiber-force-length
    curve is dimensionless: force is normalized to maximum isometric force and
    length is normalized to resting fiber length. The user can adjust the
    maximum strain at no load and the strain developed under 1 normalized unit
    of force using the fitted curve. Additionally, if desired, it is possible to
    directly set the low-force stiffness of the fiber, the stiffness of the
    fiber at 1 normalized unit of force, and the shape of the curve (its
    'curviness'):

    <h3>Properties</h3>
    - \c strainAtZeroForce<br>
        The fiber strain at which the fiber starts to develop force.
        strainAtZeroForce = 0.0 means that the fiber will begin developing
        tension when it is at its resting length.

    - \c strainAtOneNormForce<br>
        The fiber strain at which the fiber develops 1 unit of normalized force.
        strainAtOneNormForce = 0.6 means that the fiber will develop a tension
        of 1 normalized force when it is strained by 60% of its resting length
        or, equivalently, when it is stretched to 1.6 times its resting length.

    - \c stiffnessAtLowForce<br>
        The normalized stiffness (slope of the curve) when the fiber is just
        beginning to develop tensile force.

    - \c stiffnessAtOneNormForce<br>
        The normalized stiffness (slope of the curve) when the fiber develops a
        tension of 1 normalized unit of force.

    - \c curviness<br>
        A dimensionless parameter between 0 and 1 that describes the shape of
        the curve: a value of 0 indicates a curve that is very close to a
        straight line segment and a value of 1 indicates a curve that smoothly
        fills the corner formed by the linear extrapolation of
        'stiffnessAtOneNormForce' and the x-axis, as shown in the figure.

    Note that we use the Cauchy or engineering definition of strain throughout:
    strain = (l-l0)/l0, where l is the current fiber length and l0 is its
    resting length.

    <h3>Required Properties</h3>
    - \c strainAtZeroForce
    - \c strainAtOneNormForce

    <h3>Optional Properties</h3>
    - \c stiffnessAtLowForce
    - \c stiffnessAtOneNormForce
    - \c curviness

    \image html fig_FiberForceLengthCurve.png

    <h3>Conditions</h3>
    \verbatim
    strainAtZeroForce < strainAtOneNormForce
    stiffnessAtOneNormForce > 1/(strainAtOneNormForce-strainAtZeroForce)
    0 < stiffnessAtLowForce < stiffnessAtOneNormForce
    0 <= curviness <= 1
    \endverbatim

    The required parameters can be set using either the constructor or the
    setCurveStrains function; the optional parameters can be set using the
    setOptionalProperties function. Note that filling in one optional parameter
    but not the others will throw an exception when the curve is built. The
    optional parameters can be used to vary the shape of the curve from a close
    approximation of a line to a sharply-bent curve.
    
    The advantage of this curve over the typical exponential curve used in the
    literature is that it is continuous to the second derivative; the usual
    linearly-extrapolated exponential curve is only C0-continuous. The improved
    smoothness of this curve makes the equations somewhat easier to simulate
    and, more importantly, permits the use of derivative-based numerical methods
    on the curve. In addition, the extra parameters in this curve formulation
    can be adjusted to match a wide variety of shapes, should it be desired to
    fit the curve to a different set of experimental data.

    <h3>Default Parameter Values</h3>
    If the optional parameters are not specified, the curve is fit to the
    experimentally measured fiber-force-length curves of Winters et al. (2010,
    Fig. 3a).

    \verbatim
    strainAtZeroForce .......... 0.0
    strainAtOneNormForce ....... 0.7
    stiffnessAtLowForce ........ 0.2
    stiffnessAtOneNormForce .... 2.0 / (strainAtOneNormForce-strainAtZeroForce) = 2.86
    curviness .................. 0.75
    \endverbatim

    <h3>Example</h3>
    @code
    // Make a fitted fiber-force-length curve.
    FiberForceLengthCurve fpeCurve1;
    fpeCurve1.setCurveStrains(0.0, 0.7);
    double fpeVal1 = fpeCurve1.calcValue(0.1);

    // Make a custom fiber-force-length curve by supplying all parameters.
    FiberForceLengthCurve fpeCurve2(0.0, 0.7, 0.2, 2.86, 0.75);
    double fpeVal2  = fpeCurve2.calcValue(0.02);
    double dfpeVal2 = fpeCurve2.calcDerivative(0.02, 1);
    @endcode

    Note that this object should be updated through the set methods provided.
    These set methods will take care of rebuilding the curve correctly. If you
    modify the properties directly, the curve will not be rebuilt, and upon
    calling a function like calcValue, calcDerivative, or printCurveToCSVFile,
    an exception will be thrown because the curve is out-of-date with its
    properties.

    <h3>References</h3>
    - Thelen, D.G. (2003) Adjustment of muscle mechanics model parameters to
      simulate dynamic contractions in older adults. ASME Journal of
      Biomechanical Engineering 125:70--77.
    - Winters, T.M., Takahashi, M., Lieber, R.L., and Ward, S. (2010) Whole
      muscle length-tension relationships are accurately modeled as scaled
      sarcomeres in rabbit hindlimb muscles. Journal of Biomechanics
      44:109--115.

    @author Matt Millard
*/
class OSIMACTUATORS_API FiberForceLengthCurve : public Function {
OpenSim_DECLARE_CONCRETE_OBJECT(FiberForceLengthCurve, Function);
public:
//==============================================================================
// PROPERTIES
//==============================================================================
    OpenSim_DECLARE_PROPERTY(strain_at_zero_force, double,
        "Fiber strain at zero force");
    OpenSim_DECLARE_PROPERTY(strain_at_one_norm_force, double,
        "Fiber strain at a tension of 1 normalized force");
    OpenSim_DECLARE_OPTIONAL_PROPERTY(stiffness_at_low_force, double,
        "Fiber stiffness at the end of the low-force region");
    OpenSim_DECLARE_OPTIONAL_PROPERTY(stiffness_at_one_norm_force, double,
        "Fiber stiffness at a tension of 1 normalized force");
    OpenSim_DECLARE_OPTIONAL_PROPERTY(curviness, double,
        "Fiber curve bend, from linear (0) to maximum bend (1)");

//==============================================================================
// PUBLIC METHODS
//==============================================================================
    /** The default constructor creates a fiber-force-length curve using the
    default property values and assigns a default name. */
    FiberForceLengthCurve();

    /** Constructs a fiber-force-length curve using the provided parameters and
    assigns a default name. See class documentation for the meaning of these
    parameters, each of which corresponds to a property. */
    FiberForceLengthCurve(double strainAtZeroForce,
                          double strainAtOneNormForce,
                          double stiffnessAtLowForce,
                          double stiffnessAtOneNormForce,
                          double curviness);

    /** @returns The fiber strain at which the fiber starts to develop force.
    strainAtZeroForce = 0.0 means that the fiber will begin developing tension
    when it is at its resting length. This property is set to 0 by default. */
    double getStrainAtZeroForce() const;

    /** @returns The fiber strain at which the fiber develops 1 unit of
    normalized force. strainAtOneNormForce = 0.6 means that the fiber will
    develop a tension of 1 normalized force when it is strained by 60% of its
    resting length or, equivalently, when it is stretched to 1.6 times its
    resting length. This property is set to 0.6 by default. */
    double getStrainAtOneNormForce() const;

    /** @returns The normalized stiffness (slope of the curve) when the fiber is
    just beginning to develop tensile force. This property is set to 0.125 by
    default. */
    double getStiffnessAtLowForceInUse() const;

    /** @returns The normalized stiffness (slope of the curve) when the fiber
    develops a tension of 1 normalized unit of force. This property is set to
    5.0 by default. */
    double getStiffnessAtOneNormForceInUse() const;

    /** @returns A dimensionless parameter between 0 and 1 that describes the
    shape of the curve: a value of 0 indicates a curve that is very close to a
    straight line segment and a value of 1 indicates a curve that smoothly fills
    the corner formed by the linear extrapolation of 'stiffnessAtOneNormForce'
    and the x-axis, as shown in the figure in the class description. This
    property is set to 0.75 by default. */
    double getCurvinessInUse() const;

    /** @returns True if the optional properties are empty and the fitted curve
    is being used, false if the optional properties are filled and are being
    used to construct the curve. */
    bool isFittedCurveBeingUsed() const;

    /**
    @param aStrainAtZeroForce
        The fiber strain at which the fiber starts to develop force.
        strainAtZeroForce = 0.0 means that the fiber will begin developing
        tension when it is at its resting length.
    @param aStrainAtOneNormForce
        The fiber strain at which the fiber develops 1 unit of normalized force.
        strainAtOneNormForce = 0.6 means that the fiber will develop a tension
        of 1 normalized force when it is strained by 60% of its resting length
        or, equivalently, when it is stretched to 1.6 times its resting length.

    <B>Conditions</B>
    \verbatim
    strainAtZeroForce < strainAtOneNormForce
    \endverbatim
    */
    void setCurveStrains(double aStrainAtZeroForce,
                         double aStrainAtOneNormForce);

    /**
    @param stiffnessAtLowForce
        The normalized stiffness (slope of the curve) when the fiber is just
        beginning to develop tensile force.
    @param stiffnessAtOneNormForce
        The normalized stiffness (slope of the curve) when the fiber develops a
        tension of 1 normalized unit of force.
    @param curviness
        A dimensionless parameter between 0 and 1 that controls the shape of the
        curve: a value of 0 will create a curve that is very close to a straight
        line segment and a value of 1 will create a curve that smoothly fills
        the corner formed by the linear extrapolation of
        'stiffnessAtOneNormForce' and the x-axis, as shown in the figure in the
        class description.

    <B>Conditions</B>
    \verbatim
    stiffnessAtOneNormForce > 1/(strainAtOneNormForce-strainAtZeroForce)
    0 < stiffnessAtLowForce < stiffnessAtOneNormForce
    0 <= curviness <= 1
    \endverbatim
    */
    void setOptionalProperties(double stiffnessAtLowForce,
                               double stiffnessAtOneNormForce,
                               double curviness);

    /** Implement the generic OpenSim::Function interface **/
    double calcValue(const SimTK::Vector& x) const override
    {
        return calcValue(x[0]);
    }

    /** Evaluates the fiber-force-length curve at a normalized fiber length of
    'normFiberLength'. */
    double calcValue(double normFiberLength) const;

    /** Calculates the derivative of the fiber-force-length multiplier with
    respect to the normalized fiber length.
    @param normFiberLength
        The normalized length of the muscle fiber.
    @param order
        The order of the derivative. Only values of 0, 1, and 2 are acceptable.
    @return
        The derivative of the fiber-force-length curve with respect to the
        normalized fiber length.
    */
    double calcDerivative(double normFiberLength, int order) const;
    

    /// If possible, use the simpler overload above.
    double calcDerivative(const std::vector<int>& derivComponents,
                          const SimTK::Vector& x) const override;

    /** Calculates the normalized area under the curve. Since it is expensive to
    construct, the curve is built only when necessary.
    @param normFiberLength
        The normalized length of the muscle fiber.
    @return The normalized area under the curve, which corresponds to the
        normalized potential energy stored in the fiber. To calculate the
        potential energy stored in the fiber in units of Joules, multiply the
        returned quantity by normForce*normLength (where normForce is the number
        of Newtons represented by a normalized force of 1.0 and normLength is
        the number of meters represented by a normalized length of 1.0).
    */
    double calcIntegral(double normFiberLength) const;

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
    "bicepsfemoris_FiberForceLengthCurve.csv"). This function is not const to
    permit the curve to be rebuilt if it is out-of-date with its properties.
    @param path
        The full destination path. Note that forward slashes ('/') must be used
        and there should not be a slash after the last folder.

    The file will contain the following data:
    \verbatim
    column: 1 | 2 |     3 |       4
      data: x | y | dy/dx | d2y/dx2
    \endverbatim

    Samples will be taken from the zero-force region, through the curve, out to
    strains beyond which the fiber generates passive normalized forces greater
    than 1. The curve is sampled quite densely: the fiber-force-length .csv file
    will have 200+20 rows.

    <B>Example</B>
    To read the .csv file into Matlab, you need to set csvread to ignore the
    header row. Since csvread is 0-indexed, the following example will begin
    reading the .csv file from the first column of the second row:
    \verbatim
    data = csvread('bicepsfemoris_FiberForceLengthCurve.csv', 1, 0);
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

    // This function will take all of the current property values. If they have
    // changed since the last time the curve was built, the curve is rebuilt.
    // Curve construction costs ~20,500 flops.
    void buildCurve(bool computeIntegral=false);

    // Calculates the properties of the passive force-length curve documented in
    // Thelen (2003). Specifically:
    //   [0]: the strain at which the fiber begins to develop force
    //   [1]: the strainAtOneNormForce for which these properties were computed
    //   [2]: kPE
    //   [3]: normalized slope: dF/depsilon at strainAtOneNormForce (stiffness)
    //   [4]: normalized area under the curve, from a (Cauchy) strain of 0 to
    //        strainAtOneNormForce
    // Note that the equation presented in Thelen (2003) has been improved to
    // fit passive force-length curves that begin to develop force at a strain
    // that is greater than 0. This was done by replacing the e1 terms with
    // (e1-e0) terms.
    SimTK::Vec5 calcReferencePassiveFiber(double strainAtZeroForce,
                                          double strainAtOneNormForce);

    // NO LONGER USED
    double calcCurvinessOfBestFit(double e0, double e1, double k0, double k1,
                                  double area, double relTol);

    SmoothSegmentedFunction m_curve;
    double m_stiffnessAtLowForceInUse;
    double m_stiffnessAtOneNormForceInUse;
    double m_curvinessInUse;
    bool   m_fittedCurveBeingUsed;

};

}

#endif // OPENSIM_FIBER_FORCE_LENGTH_CURVE_H_
